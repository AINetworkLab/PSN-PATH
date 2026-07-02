#ifndef BITMAP_RETRANS_MODULE_H
#define BITMAP_RETRANS_MODULE_H

#include <array>
#include <cstdint>
#include <deque>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "rdma-queue-pair.h"
#include "rdma-flow-key.h"

namespace ns3 {

struct BitmapRetransFeedback {
    bool sendControl{false};
    bool isNack{false};
    bool droppedForOverflow{false};
    bool hasGap{false};
    uint32_t ackSeq{0};
    uint32_t bitmapBasePsn{0};
    std::string bitmap;
};

class BitmapRetransModule {
   public:
    void SetBitmapSize(uint32_t bitmapSize);
    void RegisterTxFlow(const RdmaFlowKey &key);
    void UnregisterTxFlow(const RdmaFlowKey &key);
    void RegisterRxFlow(const RdmaFlowKey &key);
    void UnregisterRxFlow(const RdmaFlowKey &key);

    void OnPacketSent(const RdmaFlowKey &key, uint32_t seq, uint32_t size, bool isRetrans);
    void OnAck(const RdmaFlowKey &key, uint32_t ackSeq);
    uint32_t GetOldestUnacked(const RdmaFlowKey &key, uint32_t fallbackSeq) const;
    bool HasOutstanding(const RdmaFlowKey &key) const;
    bool TryScheduleRetrans(const RdmaFlowKey &key, uint32_t seq);
    void ClearRetransMarker(const RdmaFlowKey &key, uint32_t seq);
    void ClearRetransUpTo(const RdmaFlowKey &key, uint32_t ackSeq);
    std::vector<uint32_t> CollectMissingSeqs(const RdmaFlowKey &key, uint32_t ackSeq);

    BitmapRetransFeedback OnData(const RdmaFlowKey &key, uint32_t seq, uint32_t size,
                                 uint32_t packetSize);

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
        std::vector<uint8_t> bitmap;
    };

    uint32_t m_bitmapSize{BITMAP_SIZE};
    std::unordered_map<RdmaFlowKey, TxState, RdmaFlowKeyHash> m_txStates;
    std::unordered_map<RdmaFlowKey, RxState, RdmaFlowKeyHash> m_rxStates;
};

}  // namespace ns3

#endif
