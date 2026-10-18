#include "ornic-retrans-module.h"

#include <algorithm>
#include <cmath>
#include "ns3/abort.h"

namespace ns3 {

void OrnicRetransModule::RegisterTxFlow(const RdmaFlowKey &key) { m_txStates[key]; }

void OrnicRetransModule::UnregisterTxFlow(const RdmaFlowKey &key) { m_txStates.erase(key); }

void OrnicRetransModule::RegisterRxFlow(const RdmaFlowKey &key, double otd) {
    NS_ABORT_MSG_IF(!std::isfinite(otd) || otd < 0,
                    "ORNIC requires an explicit finite, nonnegative OTD in PSN units");
    m_rxStates[key].otd = otd;
}

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
                                           uint32_t packetSize) {
    auto flow = m_rxStates.find(key);
    NS_ABORT_MSG_IF(flow == m_rxStates.end(), "ORNIC receive flow has no configured OTD");
    RxState &state = flow->second;
    OrnicRxFeedback feedback;
    uint32_t step = packetSize == 0 ? std::max<uint32_t>(1, size) : packetSize;
    feedback.ackSeq = state.expectedSeq;
    // ORNIC Section 3.3 configures OTD per QP; it does not prescribe a delay sampler.
    feedback.otd = state.otd;

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
    feedback.isNack = feedback.gap > feedback.otd;
    feedback.ackSeq = state.expectedSeq;
    feedback.nackSeq = state.expectedSeq;
    feedback.nackSize = step;
    return feedback;
}

}  // namespace ns3
