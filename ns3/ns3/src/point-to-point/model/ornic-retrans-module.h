#ifndef ORNIC_RETRANS_MODULE_H
#define ORNIC_RETRANS_MODULE_H

#include <cstdint>
#include <deque>
#include <unordered_map>
#include <unordered_set>

#include "rdma-flow-key.h"

namespace ns3 {

struct OrnicRxFeedback {
    bool sendControl{false};
    bool isNack{false};
    uint32_t ackSeq{0};
    uint32_t nackSeq{0};
    uint32_t nackSize{0};
    uint32_t gap{0};
    double otd{0.0};
    uint64_t mddNs{0};
    uint64_t rttMinNs{0};
    uint64_t rttMaxNs{0};
};

class OrnicRetransModule {
   public:
    void SetWindowSize(uint32_t windowSize);
    void SetBandwidthGbps(double bandwidthGbps);
    void RegisterTxFlow(const RdmaFlowKey &key);
    void UnregisterTxFlow(const RdmaFlowKey &key);
    void RegisterRxFlow(const RdmaFlowKey &key);
    void UnregisterRxFlow(const RdmaFlowKey &key);

    void OnPacketSent(const RdmaFlowKey &key, uint32_t seq, uint32_t size, bool isRetrans);
    void OnAck(const RdmaFlowKey &key, uint32_t ackSeq);
    void ClearRetransUpTo(const RdmaFlowKey &key, uint32_t ackSeq);
    bool TryScheduleRetrans(const RdmaFlowKey &key, uint32_t seq);
    void ClearRetransMarker(const RdmaFlowKey &key, uint32_t seq);

    OrnicRxFeedback OnData(const RdmaFlowKey &key, uint32_t seq, uint32_t size,
                           uint32_t packetSize, uint64_t oneWayDelayNs);

   private:
    struct TxSegment {
        uint32_t seq{0};
        uint32_t size{0};
    };

    struct TxState {
        std::deque<TxSegment> outstanding;
        std::unordered_set<uint32_t> pendingRetransSeqs;
    };

    struct RxState {
        uint32_t expectedSeq{0};
        std::deque<uint64_t> delaySamplesNs;
        std::unordered_map<uint32_t, uint32_t> outOfOrder;
    };

    uint32_t m_windowSize{4};
    double m_bandwidthGbps{0.0};
    std::unordered_map<RdmaFlowKey, TxState, RdmaFlowKeyHash> m_txStates;
    std::unordered_map<RdmaFlowKey, RxState, RdmaFlowKeyHash> m_rxStates;
};

}  // namespace ns3

#endif
