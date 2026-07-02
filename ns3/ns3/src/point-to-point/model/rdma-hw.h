#ifndef RDMA_HW_H
#define RDMA_HW_H

#include <ns3/custom-header.h>
#include <ns3/callback.h>
#include <ns3/node.h>
#include <ns3/rdma.h>
#include <ns3/selective-packet-queue.h>

#include <string>
#include <unordered_map>
#include <unordered_set>

#include "qbb-net-device.h"
#include "bitmap-retrans-module.h"
#include "falcon-retrans-module.h"
#include "ornic-retrans-module.h"
#include "rdma-flow-key.h"
#include "rdma-queue-pair.h"

namespace ns3 {

enum class RxControlDelayKind {
    None,
    Falcon,
    Ornic,
};

struct RdmaInterfaceMgr {
    Ptr<QbbNetDevice> dev;
    Ptr<RdmaQueuePairGroup> qpGrp;

    RdmaInterfaceMgr() : dev(NULL), qpGrp(NULL) {}
    RdmaInterfaceMgr(Ptr<QbbNetDevice> _dev) { dev = _dev; }
};

class RdmaHw : public Object {
   public:
    static TypeId GetTypeId(void);
    static void SetEndpointLogFiles(const std::string &senderLogPath,
                                    const std::string &receiverLogPath);
    typedef Callback<void, Ipv4Address, Ipv4Address, uint32_t, bool> TxDataPacketCallback;
    typedef Callback<void, Ipv4Address, Ipv4Address, uint32_t> RxDataPacketCallback;
    static void SetTxDataPacketCallback(TxDataPacketCallback cb);
    static void SetRxDataPacketCallback(RxDataPacketCallback cb);
    RdmaHw();

    Ptr<Node> m_node;
    DataRate m_minRate;  //< Min sending rate
    uint32_t m_mtu;
    uint32_t m_cc_mode;
    double m_nack_interval;
    uint32_t m_chunk;
    uint32_t m_ack_interval;
    bool m_backto0;
    bool m_var_win, m_fast_react;
    bool m_rateBound;
    std::vector<RdmaInterfaceMgr> m_nic;  // list of running nic controlled by this RdmaHw
    std::unordered_map<RdmaFlowKey, Ptr<RdmaQueuePair>, RdmaFlowKeyHash> m_qpMap;
    std::unordered_map<RdmaFlowKey, Ptr<RdmaRxQueuePair>, RdmaFlowKeyHash> m_rxQpMap;
    std::unordered_map<uint32_t, std::vector<int>>
        m_rtTable;  // map from ip address (u32) to possible ECMP port (index of dev)

    // qp complete callback
    typedef Callback<void, Ptr<RdmaQueuePair>> QpCompleteCallback;
    QpCompleteCallback m_qpCompleteCallback;

    void SetNode(Ptr<Node> node);
    void Setup(QpCompleteCallback cb);  // setup shared data and callbacks with the QbbNetDevice

    /* Akashic Record of finished QP */
    std::unordered_set<RdmaFlowKey, RdmaFlowKeyHash> akashic_Qp;    // instance for each src
    std::unordered_set<RdmaFlowKey, RdmaFlowKeyHash> akashic_RxQp;  // instance for each dst
    static uint64_t nAllPkts;                   // number of total packets
    static TxDataPacketCallback m_txDataPacketCallback;
    static RxDataPacketCallback m_rxDataPacketCallback;

    /* TxQpeueuPair */
    static RdmaFlowKey GetQpKey(uint32_t dip, uint16_t sport, uint16_t dport,
                                uint16_t pg);       // get the lookup key for m_qpMap
    Ptr<RdmaQueuePair> GetQp(const RdmaFlowKey &key);  // get the qp
    uint32_t GetNicIdxOfQp(Ptr<RdmaQueuePair> qp);  // get the NIC index of the qp
    void DeleteQueuePair(Ptr<RdmaQueuePair> qp);    // delete TxQP

    void AddQueuePair(uint64_t size, uint16_t pg, Ipv4Address _sip, Ipv4Address _dip,
                      uint16_t _sport, uint16_t _dport, uint32_t win, uint64_t baseRtt,
                      int32_t flow_id);  // add a nw qp (new send)
    void AddQueuePair(uint64_t size, uint16_t pg, Ipv4Address _sip, Ipv4Address _dip,
                      uint16_t _sport, uint16_t _dport, uint32_t win, uint64_t baseRtt) {
        this->AddQueuePair(size, pg, _sip, _dip, _sport, _dport, win, baseRtt, -1);
    }

    /* RxQueuePair */
    static RdmaFlowKey GetRxQpKey(uint32_t dip, uint16_t dport, uint16_t sport, uint16_t pg);
    Ptr<RdmaRxQueuePair> GetRxQp(uint32_t sip, uint32_t dip, uint16_t sport, uint16_t dport,
                                 uint16_t pg, bool create);  // get a rxQp
    uint32_t GetNicIdxOfRxQp(Ptr<RdmaRxQueuePair> q);        // get the NIC index of the rxQp
    void DeleteRxQp(uint32_t dip, uint16_t dport, uint16_t sport, uint16_t pg);  // delete RxQP

    int ReceiveUdp(Ptr<Packet> p, CustomHeader &ch);
    int ReceiveCnp(Ptr<Packet> p, CustomHeader &ch);
    int ReceiveAck(Ptr<Packet> p, CustomHeader &ch);  // handle both ACK and NACK
    int Receive(Ptr<Packet> p,
                CustomHeader &
                    ch);  // callback function that the QbbNetDevice should use when receive
                          // packets. Only NIC can call this function. And do not call this upon PFC

    void CheckandSendQCN(Ptr<RdmaRxQueuePair> q);
    int ReceiverCheckSeq(uint32_t &seq, Ptr<RdmaRxQueuePair> q, uint32_t size, bool &cnp);
    int ReceiverCheckSeqPsnPath(uint32_t &seq, Ptr<RdmaRxQueuePair> q, uint32_t size, bool &cnp);
    void SchedulePsnPathGapTimer(Ptr<RdmaRxQueuePair> q);
    void HandlePsnPathGapTimeout(Ptr<RdmaRxQueuePair> q, uint32_t missingPsn);
    void SendPsnPathGapTimeoutNack(Ptr<RdmaRxQueuePair> q, uint32_t missingPsn);
    void AddHeader(Ptr<Packet> p, uint16_t protocolNumber);
    static uint16_t EtherToPpp(uint16_t protocol);

    void RecoverQueue(Ptr<RdmaQueuePair> qp);
    void QpComplete(Ptr<RdmaQueuePair> qp);
    void SetLinkDown(Ptr<QbbNetDevice> dev);

    // call this function after the NIC is setup
    void AddTableEntry(Ipv4Address &dstAddr, uint32_t intf_idx);
    void ClearTable();
    void RedistributeQp();

    Ptr<Packet> GetNxtPacket(Ptr<RdmaQueuePair> qp);  // get next packet to send, inc snd_nxt
    void PktSent(Ptr<RdmaQueuePair> qp, Ptr<Packet> pkt, Time interframeGap);
    void UpdateNextAvail(Ptr<RdmaQueuePair> qp, Time interframeGap, uint32_t pkt_size);
    void ChangeRate(Ptr<RdmaQueuePair> qp, DataRate new_rate);

    void HandleTimeout(Ptr<RdmaQueuePair> qp, Time rto);
    void EvaluatePathState(Ptr<RdmaQueuePair> qp);
    void SchedulePathStateEval(Ptr<RdmaQueuePair> qp);

    /* statistics */
    uint32_t cnp_by_ecn;
    uint32_t cnp_by_ooo;
    uint32_t cnp_total;
    size_t getIrnBufferOverhead();  // get buffer overhead for IRN

    /******************************
     * Mellanox's version of DCQCN
     *****************************/
    double m_g;               // feedback weight
    double m_rateOnFirstCNP;  // the fraction of line rate to set on first CNP
    bool m_EcnClampTgtRate;
    double m_rpgTimeReset;
    double m_rateDecreaseInterval;
    uint32_t m_rpgThreshold;
    double m_alpha_resume_interval;
    DataRate m_rai;   //< Rate of additive increase
    DataRate m_rhai;  //< Rate of hyper-additive increase

    // the Mellanox's version of alpha update:
    // every fixed time slot, update alpha.
    void UpdateAlphaMlx(Ptr<RdmaQueuePair> q);
    void ScheduleUpdateAlphaMlx(Ptr<RdmaQueuePair> q);

    // Mellanox's version of CNP receive
    void cnp_received_mlx(Ptr<RdmaQueuePair> q);

    // Mellanox's version of rate decrease
    // It checks every m_rateDecreaseInterval if CNP arrived (m_decrease_cnp_arrived).
    // If so, decrease rate, and reset all rate increase related things
    void CheckRateDecreaseMlx(Ptr<RdmaQueuePair> q);
    void ScheduleDecreaseRateMlx(Ptr<RdmaQueuePair> q, uint32_t delta);

    // Mellanox's version of rate increase
    void RateIncEventTimerMlx(Ptr<RdmaQueuePair> q);
    void RateIncEventMlx(Ptr<RdmaQueuePair> q);
    void FastRecoveryMlx(Ptr<RdmaQueuePair> q);
    void ActiveIncreaseMlx(Ptr<RdmaQueuePair> q);
    void HyperIncreaseMlx(Ptr<RdmaQueuePair> q);

    // Implement Timeout according to IB Spec Vol. 1 C9-139.
    // For an HCA requester using Reliable Connection service, to detect missing responses,
    // every Send queue is required to implement a Transport Timer to time outstanding requests.
    Time m_waitAckTimeout;

    /***********************
     * High Precision CC
     ***********************/
    double m_targetUtil;
    double m_utilHigh;
    uint32_t m_miThresh;
    bool m_multipleRate;
    bool m_sampleFeedback;  // only react to feedback every RTT, or qlen > 0
    bool m_hpccTrace;       // print HPCC/INT feedback logs
    uint32_t m_hpccTraceInterval;  // print one out of N HPCC ACKs per QP (N>=1)
    uint32_t m_enableConsoleLog;   // 0: off, 1: concise, 2: verbose per-packet logs
    std::string m_consoleLogPsns;   // comma/space separated PSNs for focused receiver logs
    void HandleAckHp(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch, bool ack_progress);
    void UpdateRateHp(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch, bool fast_react);
    void UpdateRateHpTest(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch, bool fast_react);
    void FastReactHp(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch);

    /**********************
     * TIMELY
     *********************/
    double m_tmly_alpha, m_tmly_beta;
    uint64_t m_tmly_TLow, m_tmly_THigh, m_tmly_minRtt;
    void HandleAckTimely(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch);
    void UpdateRateTimely(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch, bool us);
    void FastReactTimely(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch);

    /**********************
     * DCTCP
     *********************/
    DataRate m_dctcp_rai;
    void HandleAckDctcp(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch);

    /**********************
     * IRN
     *********************/
    bool m_irn;
    Time m_irn_rtoLow;
    Time m_irn_rtoHigh;
    uint32_t m_irn_bdp;
    bool m_enablePsnPath;  // legacy umbrella switch for backward compatibility
    bool m_enablePathSelection;
    bool m_enablePathSwitch;
    bool m_enablePathAwareRetrans;
    bool m_enableRxOooNack;
    bool m_enableTxNackGoBack;
    Time m_txNackRetransInterval;
    bool m_enableBitmapRetrans;
    bool m_enableFalcon;
    bool m_enableOrnic;
    double m_falconRetransRttK;
    uint32_t m_bitmapRetransSize;
    Time m_bitmapRetransTimeout;
    Time m_falconRxSendDelay;
    Time m_ornicRxSendDelay;
    double m_ornicBandwidthGbps;
    uint32_t m_psnPathK;
    uint32_t m_psnPathO;
    uint32_t m_psnPathBasePort;
    uint32_t m_psnPathPacketSize;
    double m_psnPathBeta;
    uint32_t m_psnPathTExplore;
    uint32_t m_psnPathTRemove;
    double m_psnPathEvalIntervalUs;
    uint32_t m_psnPathProbeFrequency;
    Time m_psnPathGapTimeout;
    BitmapRetransModule m_bitmapRetrans;
    FalconRetransModule m_falcon;
    OrnicRetransModule m_ornicRetrans;
    void SendRxControlPacket(Ptr<QbbNetDevice> dev, Ptr<Packet> p,
                             RxControlDelayKind delayKind);
    void SendRxControlPacketNow(Ptr<QbbNetDevice> dev, Ptr<Packet> p);
    void EnqueueFalconBitmapRetrans(Ptr<RdmaQueuePair> qp, uint32_t seq);
    void ScheduleDeferredFalconBitmapRetrans(Ptr<RdmaQueuePair> qp, uint32_t seq, Time delay);
    void EnqueueDeferredFalconBitmapRetrans(Ptr<RdmaQueuePair> qp, uint32_t seq);
    void UpdateBitmapExpectedTimeout(Ptr<RdmaRxQueuePair> q,
                                     const BitmapRetransFeedback &feedback);
    void HandleBitmapExpectedTimeout(Ptr<RdmaRxQueuePair> q, uint32_t expectedSeq);
    void SendBitmapExpectedTimeoutNack(Ptr<RdmaRxQueuePair> q, uint32_t expectedSeq);
};

} /* namespace ns3 */

#endif /* RDMA_HW_H */
