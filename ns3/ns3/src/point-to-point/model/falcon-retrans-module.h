#ifndef FALCON_RETRANS_MODULE_H
#define FALCON_RETRANS_MODULE_H

#include <cstdint>
#include <map>
#include <utility>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "ns3/nstime.h"
#include "rdma-flow-key.h"

namespace ns3 {

class Time;

struct FalconAckInfo {
    uint32_t cumAckSeq{0};
    uint16_t bitmapBits{0};
    uint64_t bitmap{0};
    bool hasGap{false};
};

struct FalconRxResult {
    bool sendControl{false};
    FalconAckInfo ack;
};

class FalconRetransModule {
   public:
    enum : uint16_t { kBitmapBits = 64 };

    void RegisterTxFlow(const RdmaFlowKey &key);
    void UnregisterTxFlow(const RdmaFlowKey &key);
    void RegisterRxFlow(const RdmaFlowKey &key);
    void UnregisterRxFlow(const RdmaFlowKey &key);

    void OnPacketSent(const RdmaFlowKey &key, uint32_t seq, uint32_t size, Time now,
                      bool isRetrans);

    FalconRxResult OnData(const RdmaFlowKey &key, uint32_t seq, uint32_t size,
                          uint32_t packetSize);

    std::vector<uint32_t> OnAck(const RdmaFlowKey &key, uint32_t cumAckSeq,
                                uint16_t bitmapBits, uint64_t bitmap, uint32_t packetSize,
                                Time now, Time retransAgeThreshold,
                                std::vector<std::pair<uint32_t, Time> > *deferredRetrans = 0);

    bool MarkRetransPending(const RdmaFlowKey &key, uint32_t seq);
    void ClearRetransPending(const RdmaFlowKey &key, uint32_t seq);
    void ClearAcked(const RdmaFlowKey &key, uint32_t cumAckSeq);
    uint32_t GetOldestOutstanding(const RdmaFlowKey &key, uint32_t fallbackSeq) const;
    uint64_t GetSelectiveAckedBytes(const RdmaFlowKey &key) const;

   private:
    struct TxSegmentState {
        uint32_t seq{0};
        uint32_t size{0};
        Time lastSent;
    };

    struct TxState {
        std::map<uint32_t, TxSegmentState> outstanding;
        std::unordered_set<uint32_t> retransPending;
        uint64_t selectiveAckedBytes{0};
    };

    struct RxState {
        uint32_t expectedSeq{0};
        std::unordered_set<uint32_t> bufferedPsns;
    };

    FalconAckInfo BuildAck(const RxState &state, uint32_t packetSize) const;

    std::unordered_map<RdmaFlowKey, TxState, RdmaFlowKeyHash> m_txStates;
    std::unordered_map<RdmaFlowKey, RxState, RdmaFlowKeyHash> m_rxStates;
};

}  // namespace ns3

#endif
