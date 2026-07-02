#ifndef RDMA_QUEUE_PAIR_H
#define RDMA_QUEUE_PAIR_H

#include <ns3/custom-header.h>
#include <ns3/data-rate.h>
#include <ns3/event-id.h>
#include <ns3/int-header.h>
#include <ns3/ipv4-address.h>
#include <ns3/object.h>
#include <ns3/packet.h>
#include <ns3/selective-packet-queue.h>

#include <climits> /* for CHAR_BIT */
#include <array>
#include <deque>
#include <unordered_set>
#include <vector>

#define BITMASK(b) (1 << ((b) % CHAR_BIT))
#define BITSLOT(b) ((b) / CHAR_BIT)
#define BITSET(a, b) ((a)[BITSLOT(b)] |= BITMASK(b))
#define BITCLEAR(a, b) ((a)[BITSLOT(b)] &= ~BITMASK(b))
#define BITTEST(a, b) ((a)[BITSLOT(b)] & BITMASK(b))
#define BITNSLOTS(nb) ((nb + CHAR_BIT - 1) / CHAR_BIT)

#define ESTIMATED_MAX_FLOW_PER_HOST 9120
#define BITMAP_SIZE 32

namespace ns3 {

enum CcMode {
    CC_MODE_DCQCN = 1,
    CC_MODE_HPCC = 3,
    CC_MODE_TIMELY = 7,
    CC_MODE_DCTCP = 8,
    CC_MODE_UNDEFINED = 0,
};

struct PsnPathStats {
    uint64_t ackCount{0};
    uint64_t nackCount{0};
    double score{0};
    bool active{false};
    bool probing{false};
};

struct PsnMappingSnapshot {
    uint32_t epoch{0};
    uint32_t k{0};
    uint32_t o{0};
    uint32_t fpsn{0};
    uint32_t activePathMask{0};
};

struct PsnRetransRequest {
    uint32_t startPsn{0};
    uint32_t endPsn{0};
    uint32_t epoch{0};
};

struct PsnPathState {
    bool enabled{false};
    bool pathSelectionEnabled{false};
    bool pathSwitchEnabled{false};
    bool pathAwareRetransEnabled{false};
    uint32_t k{4};
    uint32_t o{0};
    uint32_t basePort{10000};
    uint32_t activePathCount{4};
    uint32_t packetSize{8192};
    uint32_t fpsn{0};
    uint32_t mappingEpoch{0};
    uint32_t probeEveryN{64};
    uint32_t probeCountdown{0};
    std::vector<PsnPathStats> pathStats;
    std::deque<PsnRetransRequest> pendingRetrans;
    EventId evalEvent;
};

struct OrnicState {
    bool enabled{false};
    uint32_t packetSize{8192};
    std::deque<uint32_t> pendingRetrans;
};

struct HpPathState {
    uint32_t ownerPathSig{0};
    bool initialized{false};
    IntHop hop[IntHeader::maxHop];
    uint64_t lastTxBytes[IntHeader::maxHop];
    uint64_t lastTimestamp[IntHeader::maxHop];

    HpPathState() : ownerPathSig(0), initialized(false) {
        for (uint32_t i = 0; i < IntHeader::maxHop; i++) {
            lastTxBytes[i] = 0;
            lastTimestamp[i] = 0;
        }
    }
};

class IrnSackManager {
   private:
    std::list<std::pair<uint32_t, uint32_t>> m_data;

   public:
    int socketId{-1};

    IrnSackManager();
    IrnSackManager(int flow_id);
    void sack(uint32_t seq, uint32_t size);  // put blocks
    size_t discardUpTo(uint32_t seq);        // return number of blocks removed
    bool IsEmpty();
    bool blockExists(uint32_t seq, uint32_t size);  // query if block exists inside SACK table
    bool peekFrontBlock(uint32_t *pseq, uint32_t *psize);
    size_t getSackBufferOverhead();  // get buffer overhead

    friend std::ostream &operator<<(std::ostream &os, const IrnSackManager &im);
};

class RdmaQueuePair : public Object {
   public:
    Time startTime;
    Ipv4Address sip, dip;
    uint16_t sport, dport;
    uint64_t m_size;
    uint64_t snd_nxt, snd_una;  // next seq to send, the highest unacked seq
    uint16_t m_pg;
    uint16_t m_ipid;
    uint32_t m_win;       // bound of on-the-fly packets
    uint32_t m_cc_mode;   // congestion control mode for window enforcement
    uint64_t m_baseRtt;   // base RTT of this qp
    DataRate m_max_rate;  // max rate
    bool m_var_win;       // variable window size
    Time m_nextAvail;     //< Soonest time of next send
    uint32_t wp;          // current window of packets
    uint32_t lastPktSize;
    int32_t m_flow_id;
    Time m_timeout;
    uint64_t m_selectiveAckedBytes;
    uint64_t m_falconSrttNs;
    bool m_falconSrttValid;
    std::unordered_set<uint32_t> m_falconDeferredRetrans;

    /******************************
     * runtime states
     *****************************/
    DataRate m_rate;  //< Current rate
    struct {
        DataRate m_targetRate;  //< Target rate
        EventId m_eventUpdateAlpha;
        double m_alpha;
        bool m_alpha_cnp_arrived;  // indicate if CNP arrived in the last slot
        bool m_first_cnp;          // indicate if the current CNP is the first CNP
        EventId m_eventDecreaseRate;
        bool m_decrease_cnp_arrived;  // indicate if CNP arrived in the last slot
        uint32_t m_rpTimeStage;
        EventId m_rpTimer;
    } mlx;
    struct {
        uint32_t m_lastUpdateSeq;
        uint32_t m_lastAckSeq;
        uint32_t m_lastFastReactSeq;
        uint32_t m_fastReactBudget;
        uint32_t m_ownerPathSig;   // reserved for future multipath telemetry ownership
        uint32_t m_ownerPathEpoch; // reserved extension state (unused in single-path)
        DataRate m_curRate;
        double m_W;
        double m_Wc;
        double m_eta;
        double m_WAI;
        double m_baseRtt;
        double m_targetUtil;
        uint32_t m_incStage;
        uint32_t m_maxStage;
        IntHop hop[IntHeader::maxHop];
        uint64_t lastTxBytes[IntHeader::maxHop];
        uint64_t lastTimestamp[IntHeader::maxHop];
        std::vector<HpPathState> pathState;
        double u;
    } hp;
    struct {
        uint32_t m_lastUpdateSeq;
        DataRate m_curRate;
        uint32_t m_incStage;
        uint64_t lastRtt;
        double rttDiff;
    } tmly;
    struct {
        uint32_t m_lastUpdateSeq;
        uint32_t m_caState;
        uint32_t m_highSeq;  // when to exit cwr
        double m_alpha;
        uint32_t m_ecnCnt;
        uint32_t m_batchSizeOfAlpha;
    } dctcp;

    struct {
        bool m_enabled;
        uint32_t m_bdp;          // m_irn_maxAck_
        uint32_t m_highest_ack;  // m_irn_maxAck_
        uint32_t m_max_seq;      // m_irn_maxSeq_
        Time m_rtoLow;
        Time m_rtoHigh;
        IrnSackManager m_sack;
        bool m_recovery;
        uint32_t m_recovery_seq;
    } irn;

    struct {
        uint64_t txTotalPkts{0};
        uint64_t txTotalBytes{0};
        uint64_t txRetransPkts{0};
        uint64_t txHighestSeqSent{0};
        uint64_t pathSwitchCount{0};
    } stat;
    PsnPathState psnPath;
    OrnicState ornic;

    // Implement Timeout according to IB Spec Vol. 1 C9-139.
    // For an HCA requester using Reliable Connection service, to detect missing responses,
    // every Send queue is required to implement a Transport Timer to time outstanding requests.
    EventId m_retransmit;

    /***********
     * methods
     **********/
    static TypeId GetTypeId(void);
    RdmaQueuePair(uint16_t pg, Ipv4Address _sip, Ipv4Address _dip, uint16_t _sport,
                  uint16_t _dport);
    void SetSize(uint64_t size);
    void SetWin(uint32_t win);
    void SetBaseRtt(uint64_t baseRtt);
    void SetCcMode(uint32_t mode);
    void SetVarWin(bool v);
    void SetFlowId(int32_t v);
    void SetTimeout(Time v);

    uint64_t GetBytesLeft();
    uint32_t GetHash(void);
    void Acknowledge(uint64_t ack);
    uint64_t GetOnTheFly();
    bool IsWinBound();
    uint64_t GetWin();  // window size calculated from m_rate
    bool IsFinished();
    inline bool IsFinishedConst() const { return snd_una >= m_size; }

    uint64_t HpGetCurWin();  // window size calculated from hp.m_curRate, used by HPCC

    inline uint32_t GetIrnBytesInFlight() const {
        // IRN do not consider SACKed segments for simplicity
        return irn.m_max_seq - irn.m_highest_ack;
    }

    Time GetRto(uint32_t mtu) {
        if (irn.m_enabled) {
            if (GetIrnBytesInFlight() > 3 * mtu) {
                return irn.m_rtoHigh;
            }
            return irn.m_rtoLow;
        } else {
            return m_timeout;
        }
    }

    inline bool CanIrnTransmit(uint32_t mtu) const {
        uint64_t len_left = m_size >= snd_nxt ? m_size - snd_nxt : 0;
        if (!irn.m_enabled) return true;
        if (irn.m_bdp == 0) {
            // Treat zero as "no explicit IRN BDP cap" so custom topologies can still run.
            return true;
        }

        return (GetIrnBytesInFlight() + ((len_left > mtu) ? mtu : len_left)) < irn.m_bdp ||
               (irn.m_highest_ack + irn.m_bdp > snd_nxt);
    }
};

class RdmaRxQueuePair : public Object {  // Rx side queue pair
   public:
    uint32_t outcount{0};
    uint32_t lastoutseq{0};
    std::array<uint32_t, BITMAP_SIZE> psnPathGapPsn{};
    std::array<uint32_t, BITMAP_SIZE> psnPathGapObservedPkts{};
    std::array<bool, BITMAP_SIZE> psnPathGapNackSent{};
    std::deque<uint32_t> psnPathGapPendingNacks;
    uint32_t psnPathEarliestGapPsn{0xffffffffu};
    Time psnPathEarliestGapStart;
    EventId psnPathGapTimerEvent;
    EventId bitmapExpectedTimerEvent;
    uint32_t bitmapExpectedTimeoutSeq{0xffffffffu};
    bool bitmapExpectedNackSent{false};
    std::array<uint8_t, BITMAP_SIZE> bitmap{};
    struct ECNAccount {
        uint16_t qIndex;
        uint8_t ecnbits;
        uint16_t qfb;
        uint16_t total;

        ECNAccount() { memset(this, 0, sizeof(ECNAccount)); }
    };
    ECNAccount m_ecn_source;
    uint32_t sip, dip;
    uint16_t sport, dport;
    uint16_t m_ipid;
    uint32_t ReceiverNextExpectedSeq;
    Time m_nackTimer;
    int32_t m_milestone_rx;
    uint32_t m_lastNACK;
    std::string m_lastNackReason;
    bool m_ackSeqOverrideValid{false};
    uint32_t m_ackSeqOverride{0};
    EventId QcnTimerEvent;  // if destroy this rxQp, remember to cancel this timer
    IrnSackManager m_irn_sack_;
    std::vector<PsnMappingSnapshot> psnMappingHistory;
    int32_t m_flow_id;

    static TypeId GetTypeId(void);
    RdmaRxQueuePair();
    uint32_t GetHash(void);
};

class RdmaQueuePairGroup : public Object {
   public:
    std::vector<Ptr<RdmaQueuePair>> m_qps;
    // std::vector<Ptr<RdmaRxQueuePair> > m_rxQps;
    char m_qp_finished[BITNSLOTS(ESTIMATED_MAX_FLOW_PER_HOST)];

    static TypeId GetTypeId(void);
    RdmaQueuePairGroup(void);
    uint32_t GetN(void);
    Ptr<RdmaQueuePair> Get(uint32_t idx);
    Ptr<RdmaQueuePair> operator[](uint32_t idx);
    void AddQp(Ptr<RdmaQueuePair> qp);
    // void AddRxQp(Ptr<RdmaRxQueuePair> rxQp);
    void Clear(void);
    inline bool IsQpFinished(uint32_t idx) {
        if (__glibc_unlikely(idx >= ESTIMATED_MAX_FLOW_PER_HOST)) return false;
        return BITTEST(m_qp_finished, idx);
    }

    inline void SetQpFinished(uint32_t idx) {
        if (__glibc_unlikely(idx >= ESTIMATED_MAX_FLOW_PER_HOST)) return;
        BITSET(m_qp_finished, idx);
    }
};

}  // namespace ns3

#endif /* RDMA_QUEUE_PAIR_H */
