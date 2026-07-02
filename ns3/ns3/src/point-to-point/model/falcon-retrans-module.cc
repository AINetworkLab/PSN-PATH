#include "falcon-retrans-module.h"

#include <algorithm>

namespace ns3 {

void FalconRetransModule::RegisterTxFlow(const RdmaFlowKey &key) { m_txStates[key]; }

void FalconRetransModule::UnregisterTxFlow(const RdmaFlowKey &key) { m_txStates.erase(key); }

void FalconRetransModule::RegisterRxFlow(const RdmaFlowKey &key) { m_rxStates[key]; }

void FalconRetransModule::UnregisterRxFlow(const RdmaFlowKey &key) { m_rxStates.erase(key); }

void FalconRetransModule::OnPacketSent(const RdmaFlowKey &key, uint32_t seq, uint32_t size, Time now,
                                       bool isRetrans) {
    if (size == 0) return;
    TxState &state = m_txStates[key];
    TxSegmentState &seg = state.outstanding[seq];
    seg.seq = seq;
    seg.size = size;
    seg.lastSent = now;
    if (isRetrans) {
        state.retransPending.erase(seq);
    }
}

FalconAckInfo FalconRetransModule::BuildAck(const RxState &state, uint32_t packetSize) const {
    FalconAckInfo ack;
    ack.cumAckSeq = state.expectedSeq;
    const uint32_t step = std::max<uint32_t>(1, packetSize);
    const uint32_t expectedPsn = state.expectedSeq / step;
    ack.bitmapBits = kBitmapBits;
    ack.bitmap = 0;
    ack.hasGap = false;
    for (uint16_t bit = 0; bit < kBitmapBits; ++bit) {
        const uint32_t psn = expectedPsn + bit;
        if (state.bufferedPsns.find(psn) != state.bufferedPsns.end()) {
            ack.bitmap |= (uint64_t(1) << bit);
            ack.hasGap = true;
        }
    }
    return ack;
}

FalconRxResult FalconRetransModule::OnData(const RdmaFlowKey &key, uint32_t seq, uint32_t size,
                                           uint32_t packetSize) {
    RxState &state = m_rxStates[key];
    FalconRxResult result;
    result.sendControl = true;
    const uint32_t step = std::max<uint32_t>(1, packetSize);
    const uint32_t psn = seq / step;
    uint32_t expectedPsn = state.expectedSeq / step;
    const uint32_t bitmapLimit = expectedPsn + kBitmapBits;

    if (psn < expectedPsn) {
        result.ack = BuildAck(state, step);
        return result;
    }

    if (psn >= bitmapLimit) {
        result.ack = BuildAck(state, step);
        return result;
    }

    if (psn == expectedPsn) {
        state.expectedSeq += size;
        expectedPsn = state.expectedSeq / step;
        while (state.bufferedPsns.erase(expectedPsn) != 0) {
            state.expectedSeq += step;
            expectedPsn++;
        }
        result.ack = BuildAck(state, step);
        return result;
    }

    state.bufferedPsns.insert(psn);
    result.ack = BuildAck(state, step);
    result.ack.hasGap = true;
    return result;
}

std::vector<uint32_t> FalconRetransModule::OnAck(const RdmaFlowKey &key, uint32_t cumAckSeq,
                                                 uint16_t bitmapBits, uint64_t bitmap,
                                                 uint32_t packetSize, Time now,
                                                 Time retransAgeThreshold,
                                                 std::vector<std::pair<uint32_t, Time> > *deferredRetrans) {
    std::vector<uint32_t> retransSeqs;
    TxState &state = m_txStates[key];
    const uint32_t step = std::max<uint32_t>(1, packetSize);

    for (std::map<uint32_t, TxSegmentState>::iterator it = state.outstanding.begin();
         it != state.outstanding.end();) {
        if (it->first >= cumAckSeq) break;
        state.selectiveAckedBytes =
            state.selectiveAckedBytes >= it->second.size ? state.selectiveAckedBytes - it->second.size : 0;
        state.retransPending.erase(it->first);
        it = state.outstanding.erase(it);
    }

    const uint16_t validBits = std::min<uint16_t>(bitmapBits, kBitmapBits);
    for (uint16_t bit = 0; bit < validBits; ++bit) {
        if (((bitmap >> bit) & uint64_t(1)) == 0) {
            continue;
        }
        const uint32_t seq = cumAckSeq + static_cast<uint32_t>(bit) * step;
        std::map<uint32_t, TxSegmentState>::iterator it = state.outstanding.find(seq);
        if (it == state.outstanding.end()) {
            continue;
        }
        state.selectiveAckedBytes += it->second.size;
        state.retransPending.erase(seq);
        state.outstanding.erase(it);
    }

    uint16_t scanBits = validBits;
    while (scanBits > 0 && ((bitmap >> (scanBits - 1)) & uint64_t(1)) == 0) {
        --scanBits;
    }
    const bool urgentGap = scanBits >= std::max<uint16_t>(1, validBits / 2);

    for (uint16_t bit = 0; bit < scanBits; ++bit) {
        if (((bitmap >> bit) & uint64_t(1)) != 0) {
            continue;
        }
        const uint32_t seq = cumAckSeq + static_cast<uint32_t>(bit) * step;
        std::map<uint32_t, TxSegmentState>::iterator it = state.outstanding.find(seq);
        if (it == state.outstanding.end()) {
            continue;
        }
        if (!urgentGap && !retransAgeThreshold.IsZero()) {
            Time age = now - it->second.lastSent;
            if (age < retransAgeThreshold) {
                if (deferredRetrans != 0) {
                    deferredRetrans->push_back(std::make_pair(seq, retransAgeThreshold - age));
                }
                continue;
            }
        }
        if (state.retransPending.insert(seq).second) {
            retransSeqs.push_back(seq);
        }
    }
    return retransSeqs;
}

bool FalconRetransModule::MarkRetransPending(const RdmaFlowKey &key, uint32_t seq) {
    return m_txStates[key].retransPending.insert(seq).second;
}

void FalconRetransModule::ClearRetransPending(const RdmaFlowKey &key, uint32_t seq) {
    m_txStates[key].retransPending.erase(seq);
}

void FalconRetransModule::ClearAcked(const RdmaFlowKey &key, uint32_t cumAckSeq) {
    TxState &state = m_txStates[key];
    for (std::map<uint32_t, TxSegmentState>::iterator it = state.outstanding.begin();
         it != state.outstanding.end();) {
        if (it->first >= cumAckSeq) break;
        state.selectiveAckedBytes =
            state.selectiveAckedBytes >= it->second.size ? state.selectiveAckedBytes - it->second.size : 0;
        state.retransPending.erase(it->first);
        it = state.outstanding.erase(it);
    }
}

uint32_t FalconRetransModule::GetOldestOutstanding(const RdmaFlowKey &key, uint32_t fallbackSeq) const {
    std::unordered_map<RdmaFlowKey, TxState, RdmaFlowKeyHash>::const_iterator it = m_txStates.find(key);
    if (it == m_txStates.end() || it->second.outstanding.empty()) {
        return fallbackSeq;
    }
    return it->second.outstanding.begin()->first;
}

uint64_t FalconRetransModule::GetSelectiveAckedBytes(const RdmaFlowKey &key) const {
    std::unordered_map<RdmaFlowKey, TxState, RdmaFlowKeyHash>::const_iterator it = m_txStates.find(key);
    if (it == m_txStates.end()) {
        return 0;
    }
    return it->second.selectiveAckedBytes;
}

}  // namespace ns3
