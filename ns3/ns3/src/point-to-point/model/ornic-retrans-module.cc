#include "ornic-retrans-module.h"

#include <algorithm>

namespace ns3 {

void OrnicRetransModule::SetWindowSize(uint32_t windowSize) {
    m_windowSize = std::max<uint32_t>(1, windowSize);
}

void OrnicRetransModule::SetBandwidthGbps(double bandwidthGbps) {
    m_bandwidthGbps = std::max(0.0, bandwidthGbps);
}

void OrnicRetransModule::RegisterTxFlow(const RdmaFlowKey &key) { m_txStates[key]; }

void OrnicRetransModule::UnregisterTxFlow(const RdmaFlowKey &key) { m_txStates.erase(key); }

void OrnicRetransModule::RegisterRxFlow(const RdmaFlowKey &key) { m_rxStates[key]; }

void OrnicRetransModule::UnregisterRxFlow(const RdmaFlowKey &key) { m_rxStates.erase(key); }

void OrnicRetransModule::OnPacketSent(const RdmaFlowKey &key, uint32_t seq, uint32_t size,
                                      bool isRetrans) {
    if (size == 0 || isRetrans) return;
    TxState &state = m_txStates[key];
    if (!state.outstanding.empty() && state.outstanding.back().seq == seq) return;
    TxSegment seg;
    seg.seq = seq;
    seg.size = size;
    state.outstanding.push_back(seg);
}

void OrnicRetransModule::OnAck(const RdmaFlowKey &key, uint32_t ackSeq) {
    TxState &state = m_txStates[key];
    while (!state.outstanding.empty()) {
        const TxSegment &seg = state.outstanding.front();
        if (seg.seq + seg.size > ackSeq) break;
        state.outstanding.pop_front();
    }
}

void OrnicRetransModule::ClearRetransUpTo(const RdmaFlowKey &key, uint32_t ackSeq) {
    TxState &state = m_txStates[key];
    std::unordered_set<uint32_t>::iterator it = state.pendingRetransSeqs.begin();
    while (it != state.pendingRetransSeqs.end()) {
        if (*it < ackSeq) {
            it = state.pendingRetransSeqs.erase(it);
        } else {
            ++it;
        }
    }
}

bool OrnicRetransModule::TryScheduleRetrans(const RdmaFlowKey &key, uint32_t seq) {
    return m_txStates[key].pendingRetransSeqs.insert(seq).second;
}

void OrnicRetransModule::ClearRetransMarker(const RdmaFlowKey &key, uint32_t seq) {
    m_txStates[key].pendingRetransSeqs.erase(seq);
}

OrnicRxFeedback OrnicRetransModule::OnData(const RdmaFlowKey &key, uint32_t seq, uint32_t size,
                                           uint32_t packetSize, uint64_t oneWayDelayNs) {
    RxState &state = m_rxStates[key];
    if (oneWayDelayNs != 0) {
        state.delaySamplesNs.push_back(oneWayDelayNs);
        while (state.delaySamplesNs.size() > m_windowSize) {
            state.delaySamplesNs.pop_front();
        }
    }
    OrnicRxFeedback feedback;
    uint32_t step = packetSize == 0 ? std::max<uint32_t>(1, size) : packetSize;
    feedback.ackSeq = state.expectedSeq;
    bool hasDelayWindow = state.delaySamplesNs.size() >= 2;
    if (hasDelayWindow) {
        std::deque<uint64_t>::const_iterator minIt =
            std::min_element(state.delaySamplesNs.begin(), state.delaySamplesNs.end());
        std::deque<uint64_t>::const_iterator maxIt =
            std::max_element(state.delaySamplesNs.begin(), state.delaySamplesNs.end());
        feedback.rttMinNs = *minIt * 2;
        feedback.rttMaxNs = *maxIt * 2;
        feedback.mddNs = *maxIt - *minIt;
    }
    feedback.otd = step == 0 ? 0.0 : (static_cast<double>(feedback.mddNs) * m_bandwidthGbps) /
                                      static_cast<double>(step * 8);

    if (seq < state.expectedSeq) {
        feedback.sendControl = true;
        feedback.ackSeq = state.expectedSeq;
        return feedback;
    }

    if (seq == state.expectedSeq) {
        feedback.sendControl = true;
        state.expectedSeq += size;
        while (true) {
            std::unordered_map<uint32_t, uint32_t>::iterator it =
                state.outOfOrder.find(state.expectedSeq);
            if (it == state.outOfOrder.end()) {
                break;
            }
            uint32_t bufferedSize = it->second;
            state.outOfOrder.erase(it);
            state.expectedSeq += bufferedSize;
        }
        feedback.ackSeq = state.expectedSeq;
        return feedback;
    }

    uint32_t expectedPsn = state.expectedSeq / step;
    uint32_t psn = seq / step;
    feedback.gap = psn > expectedPsn ? psn - expectedPsn : 0;
    state.outOfOrder[seq] = size;
    feedback.sendControl = true;
    feedback.isNack = hasDelayWindow && feedback.gap > feedback.otd;
    feedback.ackSeq = state.expectedSeq;
    feedback.nackSeq = state.expectedSeq;
    feedback.nackSize = step;
    return feedback;
}

}  // namespace ns3
