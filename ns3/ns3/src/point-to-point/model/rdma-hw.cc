#include "rdma-hw.h"

#include <ns3/ipv4-header.h>
#include <ns3/seq-ts-header.h>
#include <ns3/simulator.h>
#include <ns3/udp-header.h>

#include <algorithm>
#include <climits>
#include <fstream>
#include <sstream>

#include "cn-header.h"
#include "flow-stat-tag.h"
#include "ns3/boolean.h"
#include "ns3/data-rate.h"
#include "ns3/double.h"
#include "ns3/flow-id-num-tag.h"
#include "ns3/pointer.h"
#include "ns3/ppp-header.h"
#include "ns3/settings.h"
#include "ns3/string.h"
#include "ns3/switch-node.h"
#include "ns3/uinteger.h"
#include "ppp-header.h"
#include "psn-path.h"
#include "qbb-header.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("RdmaHw");

namespace {

std::string g_senderLogPath = "sender.log";
std::string g_receiverLogPath = "receiver.log";
std::ofstream g_senderLog;
std::ofstream g_receiverLog;

std::ofstream &GetSenderLog() {
    if (!g_senderLog.is_open()) {
        g_senderLog.open(g_senderLogPath.c_str(), std::ios::out | std::ios::trunc);
    }
    return g_senderLog;
}

std::ofstream &GetReceiverLog() {
    if (!g_receiverLog.is_open()) {
        g_receiverLog.open(g_receiverLogPath.c_str(), std::ios::out | std::ios::trunc);
    }
    return g_receiverLog;
}

void LogReceiverNack(uint32_t nodeId, uint32_t expected, uint32_t got,
                     uint32_t packetSize, const char *reason) {
    std::ofstream &out = GetReceiverLog();
    if (!out.is_open()) return;
    uint32_t psnExpected = packetSize == 0 ? 0 : expected / packetSize;
    uint32_t psnGot = packetSize == 0 ? 0 : got / packetSize;
    out << "[NACK] t=" << Simulator::Now().GetNanoSeconds()
        << " node=" << nodeId
        << " psn_expected=" << psnExpected
        << " psn_got=" << psnGot
        << " reason=" << reason << std::endl;
}

void LogSenderNackRx(uint32_t nodeId, Ptr<RdmaQueuePair> qp, uint32_t missing,
                     uint32_t packetSize, const char *action) {
    std::ofstream &out = GetSenderLog();
    if (!out.is_open()) return;
    uint32_t psn = packetSize == 0 ? 0 : missing / packetSize;
    uint64_t sndNxtPsn = packetSize == 0 ? 0 : qp->snd_nxt / packetSize;
    out << "[NACK_RX] t=" << Simulator::Now().GetNanoSeconds()
        << " node=" << nodeId
        << " qp=" << qp->m_flow_id
        << " snd_nxt=" << sndNxtPsn
        << " psn=" << psn
        << " action=" << action << std::endl;
}

void LogSenderRetrans(uint32_t nodeId, Ptr<RdmaQueuePair> qp, uint32_t seq,
                      uint32_t packetSize) {
    std::ofstream &out = GetSenderLog();
    if (!out.is_open()) return;
    uint32_t psn = packetSize == 0 ? 0 : seq / packetSize;
    out << "[RETX] t=" << Simulator::Now().GetNanoSeconds()
        << " node=" << nodeId
        << " qp=" << qp->m_flow_id
        << " psn=" << psn << std::endl;
}

void LogSenderPacket(uint32_t nodeId, Ptr<RdmaQueuePair> qp, uint32_t seq, uint32_t psn,
                     uint32_t nextSeq, uint32_t nextPsn, const char *nextMode,
                     uint32_t packetSize, uint64_t win, uint64_t onTheFlyBytes,
                     uint64_t winLeftBytes, uint64_t winLeftPkts, uint32_t payloadSize,
                     bool isRetrans, bool retransFromQueue, uint32_t pathId,
                     uint16_t sourcePort) {
    std::ofstream &out = GetSenderLog();
    if (!out.is_open()) return;
    uint64_t sndNxtPsn = packetSize == 0 ? 0 : qp->snd_nxt / packetSize;
    uint64_t sndUnaPsn = packetSize == 0 ? 0 : qp->snd_una / packetSize;
    uint64_t winPkts = packetSize == 0 ? 0 : win / packetSize;
    uint64_t onTheFlyPkts = packetSize == 0 ? 0 : onTheFlyBytes / packetSize;
    out << "[SEND] t=" << Simulator::Now().GetNanoSeconds()
        << " node=" << nodeId
        << " qp=" << qp->m_flow_id
        << " seq=" << seq
        << " psn=" << psn
        << " snd_nxt_psn=" << sndNxtPsn
        << " snd_una_psn=" << sndUnaPsn
        << " snd_nxt=" << qp->snd_nxt
        << " snd_una=" << qp->snd_una
        << " nextSeq=" << nextSeq
        << " nextPsn=" << nextPsn
        << " nextMode=" << nextMode
        << " packetSize=" << packetSize
        << " winPkts=" << winPkts
        << " win=" << win
        << " onTheFlyPkts=" << onTheFlyPkts
        << " onTheFlyBytes=" << onTheFlyBytes
        << " winLeftBytes=" << winLeftBytes
        << " winLeftPkts=" << winLeftPkts
        << " payloadSize=" << payloadSize
        << " isRetrans=" << (isRetrans ? 1 : 0)
        << " retransMode="
        << (retransFromQueue ? "PSN_QUEUE" : (isRetrans ? "GO_BACK_OR_TIMEOUT" : "NEW"))
        << " pathId=" << pathId
        << " srcPort=" << sourcePort
        << " pendingRetrans=" << qp->psnPath.pendingRetrans.size()
        << std::endl;
}

void LogSenderTimeoutRetrans(uint32_t nodeId, Ptr<RdmaQueuePair> qp, uint32_t restartSeq,
                             uint32_t packetSize, uint64_t rtoNs) {
    std::ofstream &out = GetSenderLog();
    if (!out.is_open()) return;
    uint32_t psn = packetSize == 0 ? 0 : restartSeq / packetSize;
    out << "[TIMEOUT_RETX] t=" << Simulator::Now().GetNanoSeconds()
        << " node=" << nodeId
        << " qp=" << qp->m_flow_id
        << " snd_nxt=" << qp->snd_nxt
        << " snd_una=" << qp->snd_una
        << " psn=" << psn
        << " rto_ns=" << rtoNs << std::endl;
}

void LogSenderHpccRateDown(uint32_t nodeId, Ptr<RdmaQueuePair> qp, DataRate oldRate,
                           DataRate newRate, uint32_t pathId, double u, double uPrime) {
    std::ofstream &out = GetSenderLog();
    if (!out.is_open()) return;
    out << "[HPCC_RATE_DOWN] t=" << Simulator::Now().GetNanoSeconds()
        << " node=" << nodeId
        << " qp=" << qp->m_flow_id
        << " oldRate=" << oldRate.GetBitRate()
        << " newRate=" << newRate.GetBitRate()
        << " pathId=" << pathId
        << " u=" << u
        << " uPrime=" << uPrime << std::endl;
}

std::vector<uint32_t> CollectActivePathIndices(const std::vector<PsnPathStats> &stats) {
    std::vector<uint32_t> indices;
    for (uint32_t i = 0; i < stats.size(); ++i) {
        if (stats[i].active) indices.push_back(i);
    }
    return indices;
}

std::vector<uint32_t> CollectInactivePathIndices(const std::vector<PsnPathStats> &stats) {
    std::vector<uint32_t> indices;
    for (uint32_t i = 0; i < stats.size(); ++i) {
        if (!stats[i].active) indices.push_back(i);
    }
    return indices;
}

std::vector<uint32_t> CollectProbingPathIndices(const std::vector<PsnPathStats> &stats) {
    std::vector<uint32_t> indices;
    for (uint32_t i = 0; i < stats.size(); ++i) {
        if (stats[i].probing) indices.push_back(i);
    }
    return indices;
}

std::string FormatPathIndices(const std::vector<uint32_t> &indices) {
    std::ostringstream oss;
    oss << "[";
    for (size_t i = 0; i < indices.size(); ++i) {
        if (i != 0) oss << ",";
        oss << indices[i];
    }
    oss << "]";
    return oss.str();
}

std::string FormatPathStats(const std::vector<PsnPathStats> &stats) {
    std::ostringstream oss;
    oss << "[";
    for (size_t i = 0; i < stats.size(); ++i) {
        if (i != 0) oss << ",";
        oss << "{id=" << i
            << ",active=" << (stats[i].active ? 1 : 0)
            << ",probing=" << (stats[i].probing ? 1 : 0)
            << ",ack=" << stats[i].ackCount
            << ",nack=" << stats[i].nackCount
            << ",score=" << stats[i].score
            << "}";
    }
    oss << "]";
    return oss.str();
}

uint32_t BuildActivePathMask(const std::vector<PsnPathStats> &stats) {
    uint32_t mask = 0;
    for (uint32_t i = 0; i < stats.size() && i < 32; ++i) {
        if (stats[i].active) {
            mask |= (1u << i);
        }
    }
    return mask;
}

std::vector<uint32_t> ActivePathIndicesFromMask(uint32_t mask, uint32_t k) {
    std::vector<uint32_t> indices;
    uint32_t limit = std::min<uint32_t>(k == 0 ? 1 : k, 32);
    for (uint32_t i = 0; i < limit; ++i) {
        if (mask & (1u << i)) {
            indices.push_back(i);
        }
    }
    if (indices.empty()) {
        indices.push_back(0);
    }
    return indices;
}

uint32_t SelectPathFromActiveMask(uint32_t psn, uint32_t k, uint32_t o, uint32_t mask) {
    std::vector<uint32_t> active = ActivePathIndicesFromMask(mask, k);
    uint32_t slot = (psn + o) % active.size();
    return active[slot];
}

uint32_t SelectPathFromStats(uint32_t psn, uint32_t k, uint32_t o,
                             const std::vector<PsnPathStats> &stats) {
    return SelectPathFromActiveMask(psn, k, o, BuildActivePathMask(stats));
}

bool IsPsnInList(uint32_t psn, const std::string &psnList) {
    if (psnList.empty()) {
        return false;
    }

    std::string normalized = psnList;
    for (size_t i = 0; i < normalized.size(); ++i) {
        if (normalized[i] == ',' || normalized[i] == ';' || normalized[i] == ':') {
            normalized[i] = ' ';
        }
    }

    std::istringstream iss(normalized);
    uint32_t trackedPsn = 0;
    while (iss >> trackedPsn) {
        if (trackedPsn == psn) {
            return true;
        }
    }
    return false;
}

bool PsnListOverlapsFalconBitmap(uint32_t cumAckPsn, uint16_t bitmapBits, uint64_t bitmap,
                                 const std::string &psnList) {
    if (psnList.empty() || bitmapBits == 0) {
        return false;
    }

    std::string normalized = psnList;
    for (size_t i = 0; i < normalized.size(); ++i) {
        if (normalized[i] == ',' || normalized[i] == ';' || normalized[i] == ':') {
            normalized[i] = ' ';
        }
    }

    std::istringstream iss(normalized);
    uint32_t trackedPsn = 0;
    while (iss >> trackedPsn) {
        if (trackedPsn == cumAckPsn) {
            return true;
        }
        if (trackedPsn > cumAckPsn) {
            uint32_t offset = trackedPsn - cumAckPsn;
            if (offset < bitmapBits && offset < 64 && ((bitmap >> offset) & 1ULL) != 0) {
                return true;
            }
        }
    }
    return false;
}

}  // namespace

std::unordered_map<unsigned, unsigned> acc_timeout_count;
uint64_t RdmaHw::nAllPkts = 0;
RdmaHw::TxDataPacketCallback RdmaHw::m_txDataPacketCallback;
RdmaHw::RxDataPacketCallback RdmaHw::m_rxDataPacketCallback;

void RdmaHw::SetEndpointLogFiles(const std::string &senderLogPath,
                                 const std::string &receiverLogPath) {
    if (g_senderLog.is_open()) g_senderLog.close();
    if (g_receiverLog.is_open()) g_receiverLog.close();
    g_senderLogPath = senderLogPath;
    g_receiverLogPath = receiverLogPath;
}

void RdmaHw::SetTxDataPacketCallback(TxDataPacketCallback cb) {
    m_txDataPacketCallback = cb;
}

void RdmaHw::SetRxDataPacketCallback(RxDataPacketCallback cb) {
    m_rxDataPacketCallback = cb;
}

TypeId RdmaHw::GetTypeId(void) {
    static TypeId tid =
        TypeId("ns3::RdmaHw")
            .SetParent<Object>()
            .AddAttribute("MinRate", "Minimum rate of a throttled flow",
                          DataRateValue(DataRate("100Mb/s")),
                          MakeDataRateAccessor(&RdmaHw::m_minRate), MakeDataRateChecker())
            .AddAttribute("Mtu", "Mtu.", UintegerValue(1000), MakeUintegerAccessor(&RdmaHw::m_mtu),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("CcMode", "which mode of DCQCN is running", UintegerValue(0),
                          MakeUintegerAccessor(&RdmaHw::m_cc_mode), MakeUintegerChecker<uint32_t>())
            .AddAttribute("NACKGenerationInterval", "The NACK/CNP Generation interval",
                          DoubleValue(4.0), MakeDoubleAccessor(&RdmaHw::m_nack_interval),
                          MakeDoubleChecker<double>())
            .AddAttribute("L2ChunkSize", "Layer 2 chunk size. Disable chunk mode if equals to 0.",
                          UintegerValue(4000), MakeUintegerAccessor(&RdmaHw::m_chunk),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("L2AckInterval", "Layer 2 Ack intervals. Disable ack if equals to 0.",
                          UintegerValue(1), MakeUintegerAccessor(&RdmaHw::m_ack_interval),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("L2BackToZero", "Layer 2 go back to zero transmission.",
                          BooleanValue(false), MakeBooleanAccessor(&RdmaHw::m_backto0),
                          MakeBooleanChecker())
            .AddAttribute("EwmaGain",
                          "Control gain parameter which determines the level of rate decrease",
                          DoubleValue(1.0 / 16), MakeDoubleAccessor(&RdmaHw::m_g),
                          MakeDoubleChecker<double>())
            .AddAttribute("RateOnFirstCnp", "the fraction of rate on first CNP", DoubleValue(1.0),
                          MakeDoubleAccessor(&RdmaHw::m_rateOnFirstCNP),
                          MakeDoubleChecker<double>())
            .AddAttribute("ClampTargetRate", "Clamp target rate.", BooleanValue(false),
                          MakeBooleanAccessor(&RdmaHw::m_EcnClampTgtRate), MakeBooleanChecker())
            .AddAttribute("RPTimer", "The rate increase timer at RP in microseconds",
                          DoubleValue(300.0), MakeDoubleAccessor(&RdmaHw::m_rpgTimeReset),
                          MakeDoubleChecker<double>())
            .AddAttribute("RateDecreaseInterval", "The interval of rate decrease check",
                          DoubleValue(4.0), MakeDoubleAccessor(&RdmaHw::m_rateDecreaseInterval),
                          MakeDoubleChecker<double>())
            .AddAttribute("FastRecoveryTimes", "The rate increase timer at RP", UintegerValue(1),
                          MakeUintegerAccessor(&RdmaHw::m_rpgThreshold),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("AlphaResumInterval", "The interval of resuming alpha", DoubleValue(1.0),
                          MakeDoubleAccessor(&RdmaHw::m_alpha_resume_interval),
                          MakeDoubleChecker<double>())
            .AddAttribute("RateAI", "Rate increment unit in AI period",
                          DataRateValue(DataRate("5Mb/s")), MakeDataRateAccessor(&RdmaHw::m_rai),
                          MakeDataRateChecker())
            .AddAttribute("RateHAI", "Rate increment unit in hyperactive AI period",
                          DataRateValue(DataRate("50Mb/s")), MakeDataRateAccessor(&RdmaHw::m_rhai),
                          MakeDataRateChecker())
            .AddAttribute("VarWin", "Use variable window size or not", BooleanValue(false),
                          MakeBooleanAccessor(&RdmaHw::m_var_win), MakeBooleanChecker())
            .AddAttribute("FastReact", "Fast React to congestion feedback", BooleanValue(true),
                          MakeBooleanAccessor(&RdmaHw::m_fast_react), MakeBooleanChecker())
            .AddAttribute("MiThresh", "Threshold of number of consecutive AI before MI",
                          UintegerValue(5), MakeUintegerAccessor(&RdmaHw::m_miThresh),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("TargetUtil",
                          "The Target Utilization of the bottleneck bandwidth, by default 95%",
                          DoubleValue(0.95), MakeDoubleAccessor(&RdmaHw::m_targetUtil),
                          MakeDoubleChecker<double>())
            .AddAttribute(
                "UtilHigh",
                "The upper bound of Target Utilization of the bottleneck bandwidth, by default 98%",
                DoubleValue(0.98), MakeDoubleAccessor(&RdmaHw::m_utilHigh),
                MakeDoubleChecker<double>())
            .AddAttribute("RateBound", "Bound packet sending by rate, for test only",
                          BooleanValue(true), MakeBooleanAccessor(&RdmaHw::m_rateBound),
                          MakeBooleanChecker())
            .AddAttribute("MultiRate", "Maintain multiple rates in HPCC", BooleanValue(true),
                          MakeBooleanAccessor(&RdmaHw::m_multipleRate), MakeBooleanChecker())
            .AddAttribute("SampleFeedback", "Whether sample feedback or not", BooleanValue(false),
                          MakeBooleanAccessor(&RdmaHw::m_sampleFeedback), MakeBooleanChecker())
            .AddAttribute("HpccTrace", "Print HPCC feedback/utilization/rate update logs",
                          BooleanValue(false), MakeBooleanAccessor(&RdmaHw::m_hpccTrace),
                          MakeBooleanChecker())
            .AddAttribute("HpccTraceInterval",
                          "Print one out of N HPCC ACK updates per QP (N>=1)",
                          UintegerValue(128), MakeUintegerAccessor(&RdmaHw::m_hpccTraceInterval),
                          MakeUintegerChecker<uint32_t>(1))
            .AddAttribute("EnableConsoleLog",
                          "Console packet log level: 0=off, 1=concise, 2=verbose",
                          UintegerValue(0), MakeUintegerAccessor(&RdmaHw::m_enableConsoleLog),
                          MakeUintegerChecker<uint32_t>(0, 2))
            .AddAttribute("ConsoleLogPsns",
                          "Comma/space separated PSNs for focused receiver console logs",
                          StringValue(""), MakeStringAccessor(&RdmaHw::m_consoleLogPsns),
                          MakeStringChecker())
            .AddAttribute("TimelyAlpha", "Alpha of TIMELY", DoubleValue(0.875),
                          MakeDoubleAccessor(&RdmaHw::m_tmly_alpha), MakeDoubleChecker<double>())
            .AddAttribute("TimelyBeta", "Beta of TIMELY", DoubleValue(0.8),
                          MakeDoubleAccessor(&RdmaHw::m_tmly_beta), MakeDoubleChecker<double>())
            .AddAttribute("TimelyTLow", "TLow of TIMELY (ns)", UintegerValue(50000),
                          MakeUintegerAccessor(&RdmaHw::m_tmly_TLow),
                          MakeUintegerChecker<uint64_t>())
            .AddAttribute("TimelyTHigh", "THigh of TIMELY (ns)", UintegerValue(500000),
                          MakeUintegerAccessor(&RdmaHw::m_tmly_THigh),
                          MakeUintegerChecker<uint64_t>())
            .AddAttribute("TimelyMinRtt", "MinRtt of TIMELY (ns)", UintegerValue(20000),
                          MakeUintegerAccessor(&RdmaHw::m_tmly_minRtt),
                          MakeUintegerChecker<uint64_t>())
            .AddAttribute("DctcpRateAI", "DCTCP's Rate increment unit in AI period",
                          DataRateValue(DataRate("1000Mb/s")),
                          MakeDataRateAccessor(&RdmaHw::m_dctcp_rai), MakeDataRateChecker())
            .AddAttribute("IrnEnable", "Enable IRN", BooleanValue(false),
                          MakeBooleanAccessor(&RdmaHw::m_irn), MakeBooleanChecker())
            .AddAttribute("IrnRtoLow", "Low RTO for IRN", TimeValue(MicroSeconds(454)),
                          MakeTimeAccessor(&RdmaHw::m_irn_rtoLow), MakeTimeChecker())
            .AddAttribute("IrnRtoHigh", "High RTO for IRN", TimeValue(MicroSeconds(1350)),
                          MakeTimeAccessor(&RdmaHw::m_irn_rtoHigh), MakeTimeChecker())
            .AddAttribute("IrnBdp", "BDP Limit for IRN in Bytes", UintegerValue(100000),
                          MakeUintegerAccessor(&RdmaHw::m_irn_bdp), MakeUintegerChecker<uint32_t>())
            .AddAttribute("EnablePsnPath", "Legacy umbrella switch for PSN-PATH behavior",
                          BooleanValue(false), MakeBooleanAccessor(&RdmaHw::m_enablePsnPath),
                          MakeBooleanChecker())
            .AddAttribute("EnablePathSelection", "Enable PSN-PATH sender-side path selection",
                          BooleanValue(false),
                          MakeBooleanAccessor(&RdmaHw::m_enablePathSelection),
                          MakeBooleanChecker())
            .AddAttribute("EnablePathSwitch", "Enable PSN-PATH path switching",
                          BooleanValue(false), MakeBooleanAccessor(&RdmaHw::m_enablePathSwitch),
                          MakeBooleanChecker())
            .AddAttribute("EnablePathAwareRetrans", "Enable PSN-PATH retransmission side queue",
                          BooleanValue(false),
                          MakeBooleanAccessor(&RdmaHw::m_enablePathAwareRetrans),
                          MakeBooleanChecker())
            .AddAttribute("EnableRxOooNack", "Enable receiver-side immediate NACK on out-of-order gap",
                          BooleanValue(false), MakeBooleanAccessor(&RdmaHw::m_enableRxOooNack),
                          MakeBooleanChecker())
            .AddAttribute("EnableTxNackGoBack", "Enable sender-side go-back to NACKed sequence",
                          BooleanValue(false), MakeBooleanAccessor(&RdmaHw::m_enableTxNackGoBack),
                          MakeBooleanChecker())
            .AddAttribute("TxNackRetransInterval",
                          "Delay before sender starts go-back retransmission after RX OOO NACK",
                          TimeValue(NanoSeconds(0)),
                          MakeTimeAccessor(&RdmaHw::m_txNackRetransInterval),
                          MakeTimeChecker())
            .AddAttribute("EnableBitmapRetrans",
                          "Enable standalone bitmap-based retransmission module",
                          BooleanValue(false),
                          MakeBooleanAccessor(&RdmaHw::m_enableBitmapRetrans),
                          MakeBooleanChecker())
            .AddAttribute("BitmapRetransSize", "Bitmap retransmission receive window size",
                          UintegerValue(BITMAP_SIZE),
                          MakeUintegerAccessor(&RdmaHw::m_bitmapRetransSize),
                          MakeUintegerChecker<uint32_t>(1))
            .AddAttribute("BitmapRetransTimeout",
                          "Receiver fallback timeout before NACKing the current bitmap expected sequence",
                          TimeValue(NanoSeconds(0)),
                          MakeTimeAccessor(&RdmaHw::m_bitmapRetransTimeout),
                          MakeTimeChecker())
            .AddAttribute("EnableFalcon", "Enable Falcon CumAck+Bitmap selective retransmission",
                          BooleanValue(false), MakeBooleanAccessor(&RdmaHw::m_enableFalcon),
                          MakeBooleanChecker())
            .AddAttribute("FalconRxSendDelay",
                          "Receiver-side fixed delay before sending Falcon ACK/NACK controls",
                          TimeValue(NanoSeconds(0)),
                          MakeTimeAccessor(&RdmaHw::m_falconRxSendDelay),
                          MakeTimeChecker())
            .AddAttribute("FalconRetransRttK",
                          "Gate Falcon bitmap retransmission until packet age reaches k * SRTT. Zero disables the gate.",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&RdmaHw::m_falconRetransRttK),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("EnableOrnic", "Enable ORNIC OTD-based selective retransmission",
                          BooleanValue(false), MakeBooleanAccessor(&RdmaHw::m_enableOrnic),
                          MakeBooleanChecker())
            .AddAttribute("OrnicBandwidthGbps",
                          "ORNIC bandwidth B in Gbps. Zero means use the NIC data rate.",
                          DoubleValue(0.0), MakeDoubleAccessor(&RdmaHw::m_ornicBandwidthGbps),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("OrnicRxSendDelay",
                          "Receiver-side fixed delay before sending ORNIC ACK/NACK controls",
                          TimeValue(NanoSeconds(0)),
                          MakeTimeAccessor(&RdmaHw::m_ornicRxSendDelay),
                          MakeTimeChecker())
            .AddAttribute("PsnPathK", "PSN-PATH K parameter", UintegerValue(4),
                          MakeUintegerAccessor(&RdmaHw::m_psnPathK),
                          MakeUintegerChecker<uint32_t>(1))
            .AddAttribute("PsnPathO", "PSN-PATH O parameter", UintegerValue(0),
                          MakeUintegerAccessor(&RdmaHw::m_psnPathO),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("PsnPathBasePort", "Base UDP source port for PSN-PATH",
                          UintegerValue(10000), MakeUintegerAccessor(&RdmaHw::m_psnPathBasePort),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("PsnPathPacketSize", "Packet size used for PSN derivation",
                          UintegerValue(8192), MakeUintegerAccessor(&RdmaHw::m_psnPathPacketSize),
                          MakeUintegerChecker<uint32_t>(1))
            .AddAttribute("PsnPathBeta", "Beta in score = ACK - beta*NACK", DoubleValue(1.0),
                          MakeDoubleAccessor(&RdmaHw::m_psnPathBeta),
                          MakeDoubleChecker<double>())
            .AddAttribute("PsnPathTExplore", "Explore threshold for path manager",
                          UintegerValue(2), MakeUintegerAccessor(&RdmaHw::m_psnPathTExplore),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("PsnPathTRemove", "Remove threshold for path manager",
                          UintegerValue(0), MakeUintegerAccessor(&RdmaHw::m_psnPathTRemove),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("PsnPathEvalIntervalUs", "Path evaluation interval in microseconds",
                          DoubleValue(100.0), MakeDoubleAccessor(&RdmaHw::m_psnPathEvalIntervalUs),
                          MakeDoubleChecker<double>())
            .AddAttribute("PsnPathProbeFrequency", "Probe interval in packets",
                          UintegerValue(64), MakeUintegerAccessor(&RdmaHw::m_psnPathProbeFrequency),
                          MakeUintegerChecker<uint32_t>(1))
            .AddAttribute("PsnPathGapTimeout", "Receiver-side earliest PSN gap timeout",
                          TimeValue(NanoSeconds(0)),
                          MakeTimeAccessor(&RdmaHw::m_psnPathGapTimeout),
                          MakeTimeChecker())
            .AddAttribute("L2Timeout", "Sender's timer of waiting for the ack",
                          TimeValue(MilliSeconds(4)), MakeTimeAccessor(&RdmaHw::m_waitAckTimeout),
                          MakeTimeChecker());
    return tid;
}

RdmaHw::RdmaHw() {
    cnp_total = 0;
    cnp_by_ecn = 0;
    cnp_by_ooo = 0;
    m_enablePsnPath = false;
    m_enablePathSelection = false;
    m_enablePathSwitch = false;
    m_enablePathAwareRetrans = false;
    m_enableRxOooNack = false;
    m_enableTxNackGoBack = false;
    m_txNackRetransInterval = NanoSeconds(0);
    m_enableBitmapRetrans = false;
    m_enableFalcon = false;
    m_enableOrnic = false;
    m_falconRetransRttK = 0.0;
    m_bitmapRetransSize = BITMAP_SIZE;
    m_bitmapRetransTimeout = NanoSeconds(0);
    m_falconRxSendDelay = NanoSeconds(0);
    m_ornicRxSendDelay = NanoSeconds(0);
    m_ornicBandwidthGbps = 0.0;
    m_psnPathK = 4;
    m_psnPathO = 0;
    m_psnPathBasePort = 10000;
    m_psnPathPacketSize = 8192;
    m_psnPathBeta = 1.0;
    m_psnPathTExplore = 2;
    m_psnPathTRemove = 0;
    m_psnPathEvalIntervalUs = 100.0;
    m_psnPathProbeFrequency = 64;
    m_psnPathGapTimeout = NanoSeconds(0);
    m_enableConsoleLog = 0;
    m_consoleLogPsns = "";
}

void RdmaHw::SetNode(Ptr<Node> node) { m_node = node; }
void RdmaHw::Setup(QpCompleteCallback cb) {
    for (uint32_t i = 0; i < m_nic.size(); i++) {
        Ptr<QbbNetDevice> dev = m_nic[i].dev;
        if (dev == NULL) continue;
        // share data with NIC
        dev->m_rdmaEQ->m_qpGrp = m_nic[i].qpGrp;
        // setup callback
        dev->m_rdmaReceiveCb = MakeCallback(&RdmaHw::Receive, this);
        dev->m_rdmaLinkDownCb = MakeCallback(&RdmaHw::SetLinkDown, this);
        dev->m_rdmaPktSent = MakeCallback(&RdmaHw::PktSent, this);
        // config NIC
        dev->m_rdmaEQ->m_mtu = m_mtu;
        dev->m_rdmaEQ->m_rdmaGetNxtPkt = MakeCallback(&RdmaHw::GetNxtPacket, this);
    }
    // setup qp complete callback
    m_qpCompleteCallback = cb;
}

uint32_t RdmaHw::GetNicIdxOfQp(Ptr<RdmaQueuePair> qp) {
    auto &v = m_rtTable[qp->dip.Get()];
    if (v.size() > 0) {
        return v[qp->GetHash() % v.size()];
    }
    NS_ASSERT_MSG(false, "We assume at least one NIC is alive");
    std::cout << "We assume at least one NIC is alive" << std::endl;
    exit(1);
}

RdmaFlowKey RdmaHw::GetQpKey(uint32_t dip, uint16_t sport, uint16_t dport,
                             uint16_t pg) {  // Sender perspective
    RdmaFlowKey key;
    key.ip = dip;
    key.sport = sport;
    key.dport = dport;
    key.pg = pg;
    return key;
}
Ptr<RdmaQueuePair> RdmaHw::GetQp(const RdmaFlowKey &key) {
    auto it = m_qpMap.find(key);

    // lookup main memory
    if (it != m_qpMap.end()) {
        return it->second;
    }

    return NULL;
}
void RdmaHw::AddQueuePair(uint64_t size, uint16_t pg, Ipv4Address sip, Ipv4Address dip,
                          uint16_t sport, uint16_t dport, uint32_t win, uint64_t baseRtt,
                          int32_t flow_id) {
    // create qp
    Ptr<RdmaQueuePair> qp = CreateObject<RdmaQueuePair>(pg, sip, dip, sport, dport);
    qp->SetSize(size);
    qp->SetWin(win);
    qp->SetBaseRtt(baseRtt);
    qp->SetCcMode(m_cc_mode);
    qp->SetVarWin(m_var_win);
    qp->SetFlowId(flow_id);
    qp->SetTimeout(m_waitAckTimeout);

    if (m_irn) {
        qp->irn.m_enabled = m_irn;
        qp->irn.m_bdp = m_irn_bdp;
        qp->irn.m_rtoLow = m_irn_rtoLow;
        qp->irn.m_rtoHigh = m_irn_rtoHigh;
    }
    qp->psnPath.pathSelectionEnabled = m_enablePathSelection || m_enablePsnPath;
    qp->psnPath.pathSwitchEnabled = m_enablePathSwitch && qp->psnPath.pathSelectionEnabled;
    qp->psnPath.pathAwareRetransEnabled =
        m_enablePathAwareRetrans || m_enablePsnPath || m_enableBitmapRetrans || m_enableFalcon;
    qp->psnPath.enabled =
        qp->psnPath.pathSelectionEnabled || qp->psnPath.pathAwareRetransEnabled;
    qp->psnPath.k = m_psnPathK == 0 ? 1 : m_psnPathK;
    qp->psnPath.o = m_psnPathO;
    qp->psnPath.basePort = m_psnPathBasePort;
    qp->psnPath.activePathCount = qp->psnPath.k;
    qp->psnPath.packetSize = m_psnPathPacketSize == 0 ? m_mtu : m_psnPathPacketSize;
    qp->psnPath.pathStats.assign(qp->psnPath.k, PsnPathStats{});
    for (uint32_t i = 0; i < qp->psnPath.pathStats.size(); ++i) {
        qp->psnPath.pathStats[i].active = true;
    }
    qp->hp.pathState.assign(qp->psnPath.k, HpPathState{});
    qp->psnPath.probeEveryN = m_psnPathProbeFrequency == 0 ? 1 : m_psnPathProbeFrequency;
    qp->psnPath.probeCountdown = qp->psnPath.probeEveryN;
    qp->ornic.enabled = m_enableOrnic;
    qp->ornic.packetSize = m_mtu == 0 ? 1 : m_mtu;

    // add qp
    uint32_t nic_idx = GetNicIdxOfQp(qp);
    m_nic[nic_idx].qpGrp->AddQp(qp);
    auto key = GetQpKey(dip.Get(), sport, dport, pg);
    m_qpMap[key] = qp;
    if (m_enableBitmapRetrans) {
        m_bitmapRetrans.SetBitmapSize(m_bitmapRetransSize);
        m_bitmapRetrans.RegisterTxFlow(key);
    }
    if (m_enableFalcon) {
        m_falcon.RegisterTxFlow(key);
    }
    if (m_enableOrnic) {
        m_ornicRetrans.SetWindowSize(m_psnPathK == 0 ? 1 : m_psnPathK);
        double bandwidthGbps = m_ornicBandwidthGbps;
        if (bandwidthGbps <= 0.0) {
            bandwidthGbps = static_cast<double>(m_nic[nic_idx].dev->GetDataRate().GetBitRate()) /
                            1e9;
        }
        m_ornicRetrans.SetBandwidthGbps(bandwidthGbps);
        m_ornicRetrans.RegisterTxFlow(key);
    }

    // set init variables
    DataRate m_bps = m_nic[nic_idx].dev->GetDataRate();
    qp->m_rate = m_bps;
    qp->m_max_rate = m_bps;
    if (m_cc_mode == 1) {
        qp->mlx.m_targetRate = m_bps;
    } else if (m_cc_mode == 3) {
        qp->hp.m_curRate = m_bps;
        qp->hp.m_baseRtt = std::max(1e-9, (double)qp->m_baseRtt * 1e-9);
        qp->hp.m_targetUtil = m_targetUtil;
        qp->hp.m_eta = std::max(1e-6, m_targetUtil);
        qp->hp.m_WAI = std::max<double>(1.0, m_mtu);
        qp->hp.m_maxStage = m_miThresh;
        double initWin = qp->m_max_rate.GetBitRate() * qp->hp.m_baseRtt / 8.0;
        initWin *= std::max<uint32_t>(qp->psnPath.k/4, 1);
        initWin = std::max(initWin, 1.0);
        qp->hp.m_W = initWin;
        qp->hp.m_Wc = initWin;
    } else if (m_cc_mode == 7) {
        qp->tmly.m_curRate = m_bps;
    }

    // Notify Nic
    m_nic[nic_idx].dev->NewQp(qp);
    if (qp->psnPath.pathSelectionEnabled && qp->psnPath.pathSwitchEnabled) {
        SchedulePathStateEval(qp);
    }
}

void RdmaHw::DeleteQueuePair(Ptr<RdmaQueuePair> qp) {
    // remove qp from the m_qpMap
    auto key = GetQpKey(qp->dip.Get(), qp->sport, qp->dport, qp->m_pg);

    // record to Akashic record
    NS_ASSERT(akashic_Qp.find(key) == akashic_Qp.end());  // should not be already existing
    akashic_Qp.insert(key);

    // delete
    m_qpMap.erase(key);
    if (m_enableBitmapRetrans) {
        m_bitmapRetrans.UnregisterTxFlow(key);
    }
    if (m_enableFalcon) {
        m_falcon.UnregisterTxFlow(key);
    }
    if (m_enableOrnic) {
        m_ornicRetrans.UnregisterTxFlow(key);
    }
}

// DATA UDP's src = this key's dst (receiver's dst)
RdmaFlowKey RdmaHw::GetRxQpKey(uint32_t dip, uint16_t dport, uint16_t sport,
                               uint16_t pg) {  // Receiver perspective
    RdmaFlowKey key;
    key.ip = dip;
    key.sport = sport;
    key.dport = dport;
    key.pg = pg;
    return key;
}

// src/dst are already flipped (this is calleld by UDP Data packet)
Ptr<RdmaRxQueuePair> RdmaHw::GetRxQp(uint32_t sip, uint32_t dip, uint16_t sport, uint16_t dport,
                                     uint16_t pg, bool create) {
    auto rxKey = GetRxQpKey(dip, dport, sport, pg);
    auto it = m_rxQpMap.find(rxKey);

    // main memory lookup
    if (it != m_rxQpMap.end()) return it->second;

    if (create) {
        // create new rx qp
        Ptr<RdmaRxQueuePair> q = CreateObject<RdmaRxQueuePair>();
        // init the qp
        q->sip = sip;
        q->dip = dip;
        q->sport = sport;
        q->dport = dport;
        q->m_ecn_source.qIndex = pg;
        q->m_flow_id = -1;     // unknown
        m_rxQpMap[rxKey] = q;  // store in map
        return q;
    }
    return NULL;
}
uint32_t RdmaHw::GetNicIdxOfRxQp(Ptr<RdmaRxQueuePair> q) {
    auto &v = m_rtTable[q->dip];
    if (v.size() > 0) {
        return v[q->GetHash() % v.size()];
    }
    NS_ASSERT_MSG(false, "We assume at least one NIC is alive");
    std::cout << "We assume at least one NIC is alive" << std::endl;
    exit(1);
}

// Receiver's perspective?
void RdmaHw::DeleteRxQp(uint32_t dip, uint16_t dport, uint16_t sport, uint16_t pg) {
    auto key = GetRxQpKey(dip, dport, sport, pg);

    // record to Akashic record
    NS_ASSERT(akashic_RxQp.find(key) == akashic_RxQp.end());  // should not be already existing
    akashic_RxQp.insert(key);

    // delete
    auto it = m_rxQpMap.find(key);
    if (it != m_rxQpMap.end() && it->second->psnPathGapTimerEvent.IsRunning()) {
        it->second->psnPathGapTimerEvent.Cancel();
    }
    if (it != m_rxQpMap.end() && it->second->bitmapExpectedTimerEvent.IsRunning()) {
        it->second->bitmapExpectedTimerEvent.Cancel();
    }
    m_rxQpMap.erase(key);
    if (m_enableBitmapRetrans) {
        m_bitmapRetrans.UnregisterRxFlow(key);
    }
    if (m_enableFalcon) {
        m_falcon.UnregisterRxFlow(key);
    }
    if (m_enableOrnic) {
        m_ornicRetrans.UnregisterRxFlow(key);
    }
}

int RdmaHw::ReceiveUdp(Ptr<Packet> p, CustomHeader &ch) {
    uint8_t ecnbits = ch.GetIpv4EcnBits();

    uint32_t payload_size = p->GetSize() - ch.GetSerializedSize();
    PsnPathTag pt;
    bool hasPsnPathTag = p->PeekPacketTag(pt);
    uint16_t canonicalFlowSport = ch.udp.sport;
    if (hasPsnPathTag && pt.GetFlowSport() != 0) {
        canonicalFlowSport = pt.GetFlowSport();
    }

    // find corresponding rx queue pair
    Ptr<RdmaRxQueuePair> rxQp =
        GetRxQp(ch.dip, ch.sip, ch.udp.dport, canonicalFlowSport, ch.udp.pg, true);
    if (rxQp == NULL) {
        auto rxKey = GetRxQpKey(ch.sip, canonicalFlowSport, ch.udp.dport, ch.udp.pg);
        if (akashic_RxQp.find(rxKey) != akashic_RxQp.end()) {
            // printf("[GetRxQPUDP] Akashic access: %u(%d) -> %u(%d)\n", this->m_node->GetId(),
            // ch.udp.dport, ch.sip, ch.udp.sport);
            return 1;  // just drop
        } else {
            printf("ERROR: UDP NIC cannot find the flow\n");
            exit(1);
        }
    }

    if (!m_rxDataPacketCallback.IsNull()) {
        m_rxDataPacketCallback(Ipv4Address(ch.sip), Ipv4Address(ch.dip), payload_size);
    }

    if (ecnbits != 0) {
        rxQp->m_ecn_source.ecnbits |= ecnbits;
        rxQp->m_ecn_source.qfb++;
    }

    rxQp->m_ecn_source.total++;
    rxQp->m_milestone_rx = m_ack_interval;

    if (rxQp->m_flow_id < 0) {
        FlowIDNUMTag fit;
        if (p->PeekPacketTag(fit)) {
            rxQp->m_flow_id = fit.GetId();
        }
    }
    if (hasPsnPathTag) {
        uint32_t activeMask = pt.GetActivePathMask();
        if (activeMask == 0) {
            uint32_t limit = std::min<uint32_t>(pt.GetK() == 0 ? 1 : pt.GetK(), 32);
            activeMask = limit == 32 ? 0xffffffffu : ((1u << limit) - 1u);
        }
        bool found = false;
        for (size_t i = 0; i < rxQp->psnMappingHistory.size(); ++i) {
            if (rxQp->psnMappingHistory[i].epoch == pt.GetEpoch()) {
                rxQp->psnMappingHistory[i].k = pt.GetK();
                rxQp->psnMappingHistory[i].o = pt.GetO();
                rxQp->psnMappingHistory[i].fpsn = pt.GetFpsn();
                rxQp->psnMappingHistory[i].activePathMask = activeMask;
                found = true;
                break;
            }
        }
        if (!found) {
            PsnMappingSnapshot s;
            s.epoch = pt.GetEpoch();
            s.k = pt.GetK();
            s.o = pt.GetO();
            s.fpsn = pt.GetFpsn();
            s.activePathMask = activeMask;
            rxQp->psnMappingHistory.push_back(s);
            if (rxQp->psnMappingHistory.size() > 8) {
                rxQp->psnMappingHistory.erase(rxQp->psnMappingHistory.begin());
            }
        }
    }

    bool cnp_check = false;
    uint32_t seqForCheck = ch.udp.seq;
    uint32_t receivedSeq = ch.udp.seq;
    int x = 0;
    BitmapRetransFeedback bitmapFeedback;
    FalconRxResult falconFeedback;
    OrnicRxFeedback ornicFeedback;
    RxControlDelayKind rxControlDelayKind = RxControlDelayKind::None;
    if (m_enableOrnic) {
        auto ornicRxKey = GetRxQpKey(ch.sip, canonicalFlowSport, ch.udp.dport, ch.udp.pg);
        m_ornicRetrans.SetWindowSize(m_psnPathK == 0 ? 1 : m_psnPathK);
        m_ornicRetrans.RegisterRxFlow(ornicRxKey);
        uint64_t nowTs = Simulator::Now().GetTimeStep();
        uint64_t txTs = ch.udp.ih.ts;
        uint64_t oneWayDelayNs = (txTs != 0 && nowTs >= txTs) ? (nowTs - txTs) : 0;
        uint32_t prevExpectedSeq = rxQp->ReceiverNextExpectedSeq;
        ornicFeedback = m_ornicRetrans.OnData(ornicRxKey, seqForCheck, payload_size, m_mtu,
                                              oneWayDelayNs);
        rxQp->ReceiverNextExpectedSeq = ornicFeedback.ackSeq;
        x = ornicFeedback.sendControl ? (ornicFeedback.isNack ? 2 : 1) : 5;
        rxControlDelayKind = RxControlDelayKind::Ornic;
        uint32_t packetSize = m_mtu == 0 ? 1 : m_mtu;
        uint32_t psn = seqForCheck / packetSize;
        if (m_enableConsoleLog == 1 && IsPsnInList(psn, m_consoleLogPsns)) {
            const char *decision = ornicFeedback.isNack
                                       ? "NACK"
                                       : (ornicFeedback.sendControl ? "ACK" : "HOLD");
            const char *reason = "ORNIC_GAP_ACK";
            if (seqForCheck < prevExpectedSeq) {
                reason = "ORNIC_DUPLICATE_OR_OLD";
            } else if (seqForCheck == prevExpectedSeq) {
                reason = "ORNIC_IN_ORDER";
            } else if (ornicFeedback.isNack) {
                reason = "ORNIC_OTD_GAP";
            }
            std::cout << "[PSN_TRACE_RECV] t=" << Simulator::Now().GetNanoSeconds()
                      << " node=" << m_node->GetId()
                      << " flow=" << rxQp->m_flow_id
                      << " seq=" << receivedSeq
                      << " psn=" << psn
                      << " expectedPsnBefore=" << (prevExpectedSeq / packetSize)
                      << " expectedPsnAfter=" << (ornicFeedback.ackSeq / packetSize)
                      << " decision=" << decision
                      << " reason=" << reason
                      << " gap=" << ornicFeedback.gap
                      << " otd=" << ornicFeedback.otd
                      << " mddNs=" << ornicFeedback.mddNs
                      << " rttMinNs=" << ornicFeedback.rttMinNs
                      << " rttMaxNs=" << ornicFeedback.rttMaxNs
                      << std::endl;
        }
        if (m_hpccTrace) {
            std::cout << "[ORNIC_RX] t=" << Simulator::Now().GetNanoSeconds()
                      << " node=" << m_node->GetId() << " flow=" << rxQp->m_flow_id
                      << " seq=" << seqForCheck
                      << " psn=" << (seqForCheck / packetSize)
                      << " expectedPsnBefore=" << (prevExpectedSeq / packetSize)
                      << " expectedPsnAfter=" << (ornicFeedback.ackSeq / packetSize)
                      << " gap=" << ornicFeedback.gap
                      << " otd=" << ornicFeedback.otd
                      << " mddNs=" << ornicFeedback.mddNs
                      << " rttMinNs=" << ornicFeedback.rttMinNs
                      << " rttMaxNs=" << ornicFeedback.rttMaxNs
                      << " decision=" << (ornicFeedback.isNack ? "NACK" :
                                          (ornicFeedback.sendControl ? "ACK" : "HOLD"))
                      << std::endl;
        }
    } else if (m_enableFalcon) {
        auto falconRxKey = GetRxQpKey(ch.sip, canonicalFlowSport, ch.udp.dport, ch.udp.pg);
        m_falcon.RegisterRxFlow(falconRxKey);
        uint32_t prevExpectedSeq = rxQp->ReceiverNextExpectedSeq;
        falconFeedback = m_falcon.OnData(falconRxKey, seqForCheck, payload_size, m_mtu);
        rxQp->ReceiverNextExpectedSeq = falconFeedback.ack.cumAckSeq;
        x = falconFeedback.ack.hasGap ? 2 : 1;
        rxControlDelayKind = RxControlDelayKind::Falcon;
        uint32_t packetSize = m_mtu == 0 ? 1 : m_mtu;
        uint32_t psn = seqForCheck / packetSize;
        if (m_enableConsoleLog == 1 && m_enablePathSelection && hasPsnPathTag &&
            IsPsnInList(psn, m_consoleLogPsns)) {
            const char *decision = falconFeedback.ack.hasGap ? "NACK" : "ACK";
            const char *reason = "FALCON_ACK";
            if (seqForCheck < prevExpectedSeq) {
                reason = "FALCON_DUPLICATE_OR_OLD";
            } else if (seqForCheck == prevExpectedSeq) {
                reason = "FALCON_IN_ORDER";
            } else if (falconFeedback.ack.hasGap) {
                reason = "FALCON_GAP";
            } else {
                reason = "FALCON_OUT_OF_BITMAP";
            }
            std::cout << "[PSN_TRACE_RECV] t=" << Simulator::Now().GetNanoSeconds()
                      << " node=" << m_node->GetId()
                      << " flow=" << rxQp->m_flow_id
                      << " seq=" << receivedSeq
                      << " psn=" << psn
                      << " expectedPsnBefore=" << (prevExpectedSeq / packetSize)
                      << " expectedPsnAfter=" << (falconFeedback.ack.cumAckSeq / packetSize)
                      << " decision=" << decision
                      << " reason=" << reason
                      << " pathId=" << pt.GetPathId()
                      << " flowSport=" << pt.GetFlowSport()
                      << " k=" << pt.GetK()
                      << " epoch=" << pt.GetEpoch()
                      << " probe=" << static_cast<uint32_t>(pt.GetProbe())
                      << " cumAckPsn=" << (falconFeedback.ack.cumAckSeq / packetSize)
                      << " bitmapBits=" << falconFeedback.ack.bitmapBits
                      << " bitmap=0x" << std::hex << falconFeedback.ack.bitmap << std::dec
                      << " hasGap=" << (falconFeedback.ack.hasGap ? 1 : 0)
                      << std::endl;
        }
        if (m_hpccTrace) {
            std::cout << "[FALCON_RX] t=" << Simulator::Now().GetNanoSeconds()
                      << " node=" << m_node->GetId() << " flow=" << rxQp->m_flow_id
                      
                      << " cumAck=" << falconFeedback.ack.cumAckSeq
                      << " bitmapBits=" << falconFeedback.ack.bitmapBits
                      << " bitmap=0x" << std::hex << falconFeedback.ack.bitmap << std::dec
                      << " hasGap=" << (falconFeedback.ack.hasGap ? 1 : 0) << std::endl;
        }
    } else if (m_enableBitmapRetrans) {
        auto bitmapRxKey = GetRxQpKey(ch.sip, canonicalFlowSport, ch.udp.dport, ch.udp.pg);
        m_bitmapRetrans.SetBitmapSize(m_bitmapRetransSize);
        m_bitmapRetrans.RegisterRxFlow(bitmapRxKey);
        uint32_t prevExpectedSeq = rxQp->ReceiverNextExpectedSeq;
        bitmapFeedback = m_bitmapRetrans.OnData(bitmapRxKey, seqForCheck, payload_size, m_mtu);
        rxQp->ReceiverNextExpectedSeq = bitmapFeedback.ackSeq;
        UpdateBitmapExpectedTimeout(rxQp, bitmapFeedback);
        x = bitmapFeedback.sendControl ? (bitmapFeedback.isNack ? 2 : 1) : 5;
        uint32_t packetSize = m_mtu == 0 ? 1 : m_mtu;
        uint32_t psn = seqForCheck / packetSize;
        if (m_enableConsoleLog == 1 && IsPsnInList(psn, m_consoleLogPsns)) {
            const char *decision = bitmapFeedback.sendControl
                                       ? (bitmapFeedback.isNack ? "NACK" : "ACK")
                                       : "HOLD";
            const char *reason = "BITMAP_WINDOW_MARK";
            if (seqForCheck < prevExpectedSeq) {
                reason = "BITMAP_DUPLICATE_OR_OLD";
            } else if (seqForCheck == prevExpectedSeq) {
                reason = "BITMAP_IN_ORDER";
            } else if (bitmapFeedback.droppedForOverflow) {
                reason = "BITMAP_OVERFLOW";
            }
            std::cout << "[PSN_TRACE_RECV] t=" << Simulator::Now().GetNanoSeconds()
                      << " node=" << m_node->GetId()
                      << " seq=" << receivedSeq
                      << " psn=" << psn
                      << " expectedPsnBefore=" << (prevExpectedSeq / packetSize)
                      << " expectedPsnAfter=" << (bitmapFeedback.ackSeq / packetSize)
                      << " decision=" << decision
                      << " reason=" << reason
                      << " bitmapBasePsn=" << bitmapFeedback.bitmapBasePsn
                      << " bitmap=" << bitmapFeedback.bitmap
                      << std::endl;
        }
        if (m_hpccTrace) {
            std::cout << "[BITMAP_RX] t=" << Simulator::Now().GetNanoSeconds()
                      << " node=" << m_node->GetId() << " flow=" << rxQp->m_flow_id
                      << " ackSeq=" << bitmapFeedback.ackSeq
                      << " sendControl=" << (bitmapFeedback.sendControl ? 1 : 0)
                      << " isNack=" << (bitmapFeedback.isNack ? 1 : 0)
                      << " dropped=" << (bitmapFeedback.droppedForOverflow ? 1 : 0)
                      << std::endl;
        }
    } else {
        x = ReceiverCheckSeq(seqForCheck, rxQp, payload_size, cnp_check);
    }

    if (x == 1 || x == 2 || x == 6) {  // generate ACK or NACK
        qbbHeader seqh;
        seqh.SetSeq(rxQp->m_ackSeqOverrideValid ? rxQp->m_ackSeqOverride
                                                 : rxQp->ReceiverNextExpectedSeq);
        seqh.SetPG(ch.udp.pg);
        seqh.SetSport(ch.udp.dport);
        seqh.SetDport(canonicalFlowSport);
        seqh.SetIntHeader(ch.udp.ih);

        if (m_enableFalcon) {
            seqh.SetSeq(falconFeedback.ack.cumAckSeq);
            seqh.SetFalconBitmap(falconFeedback.ack.bitmap, falconFeedback.ack.bitmapBits);
        } else if (m_enableOrnic) {
            seqh.SetSeq(ornicFeedback.ackSeq);
            if (x == 2) {
                seqh.SetIrnNack(ornicFeedback.nackSeq);
                seqh.SetIrnNackSize(ornicFeedback.nackSize);
            }
        } else if (m_enableBitmapRetrans) {
            seqh.SetSeq(bitmapFeedback.ackSeq);
        } else if (m_enableRxOooNack && x == 2) {
            seqh.SetDirectNack();
            seqh.SetIrnNack(seqForCheck);
            seqh.SetIrnNackSize(payload_size);
        } else if (m_irn) {
            if (x == 2) {
                seqh.SetIrnNack(seqForCheck);
                seqh.SetIrnNackSize(payload_size);
            } else {
                seqh.SetIrnNack(0);  // NACK without ackSyndrome (ACK) in loss recovery mode
                seqh.SetIrnNackSize(0);
            }
        } else if ((m_enablePathAwareRetrans || m_enablePsnPath || m_enableTxNackGoBack) &&
                   x == 2) {
            seqh.SetIrnNack(seqForCheck);
            seqh.SetIrnNackSize(payload_size);
        }

        if (x == 2) {
            uint32_t packetSize = m_psnPathPacketSize == 0 ? m_mtu : m_psnPathPacketSize;
            uint32_t expectedSeq = seqh.GetIrnNackSize() != 0 ? seqh.GetIrnNack() : seqh.GetSeq();
            const char *reason = "NACK";
            if (m_enableFalcon) {
                reason = "FALCON_GAP";
            } else if (m_enableOrnic) {
                reason = "ORNIC_OTD_GAP";
            } else if (m_enableBitmapRetrans) {
                reason = bitmapFeedback.droppedForOverflow ? "BITMAP_OVERFLOW" : "BITMAP_GAP";
            } else if (m_enableRxOooNack) {
                reason = "OOO_DIRECT_NACK";
            } else if (m_irn) {
                reason = "IRN_GAP";
            } else {
                reason = "OOO_GAP";
            }
            if (!rxQp->m_lastNackReason.empty()) {
                reason = rxQp->m_lastNackReason.c_str();
            }
            LogReceiverNack(m_node->GetId(), expectedSeq, receivedSeq, packetSize, reason);
        }

        if (ecnbits || cnp_check) {  // NACK accompanies with CNP packet
            // XXX monitor CNP generation at sender
            cnp_total++;
            if (ecnbits) cnp_by_ecn++;
            if (cnp_check) cnp_by_ooo++;
            seqh.SetCnp();
        }

        Ptr<Packet> newp =
            Create<Packet>(std::max(60 - 14 - 20 - (int)seqh.GetSerializedSize(), 0));
        newp->AddHeader(seqh);
        if (m_enableFalcon && m_enableConsoleLog == 1) {
            uint32_t packetSize = m_mtu == 0 ? 1 : m_mtu;
            uint32_t cumAckPsn = falconFeedback.ack.cumAckSeq / packetSize;
            if (IsPsnInList(receivedSeq / packetSize, m_consoleLogPsns) ||
                PsnListOverlapsFalconBitmap(cumAckPsn, falconFeedback.ack.bitmapBits,
                                            falconFeedback.ack.bitmap, m_consoleLogPsns)) {
                std::cout << "[PSN_TRACE_ACK_GEN] t=" << Simulator::Now().GetNanoSeconds()
                          << " node=" << m_node->GetId()
                          << " flow=" << rxQp->m_flow_id
                          << " wireType=" << ((x == 1) ? "ACK" : "NACK")
                          << " recvPsn=" << (receivedSeq / packetSize)
                          << " cumAckPsn=" << cumAckPsn
                          << " bitmapBits=" << falconFeedback.ack.bitmapBits
                          << " bitmap=0x" << std::hex << falconFeedback.ack.bitmap << std::dec
                          << " hasGap=" << (falconFeedback.ack.hasGap ? 1 : 0)
                          << " controlBytes=" << newp->GetSize()
                          << std::endl;
            }
        }
        if (m_hpccTrace && m_cc_mode == 3) {
            const char *wireType = (x == 1) ? "ACK" : "NACK";
            const char *semanticType = (x == 2) ? "NACK" : "ACK";
            std::cout << "[HPCC_ACK_GEN] t=" << Simulator::Now().GetNanoSeconds()
                      << " node=" << m_node->GetId()
                      << " sport=" << ch.udp.sport << " dport=" << ch.udp.dport
                      << " ackSeq=" << rxQp->ReceiverNextExpectedSeq
                      << " wireType=" << wireType
                      << " semanticType=" << semanticType
                      << " directNack=" << static_cast<uint32_t>(seqh.GetDirectNack())
                      << " nackSeq=" << seqh.GetIrnNack()
                      << " nackSize=" << seqh.GetIrnNackSize()
                      << " ecn=" << (uint32_t)ecnbits << " cnp=" << (ecnbits || cnp_check)
                      << " nhop=" << ch.udp.ih.nhop << std::endl;
        }

        Ipv4Header head;  // Prepare IPv4 header
        head.SetDestination(Ipv4Address(ch.sip));
        head.SetSource(Ipv4Address(ch.dip));
        head.SetProtocol(x == 1 ? 0xFC : 0xFD);  // ack=0xFC nack=0xFD
        head.SetTtl(64);
        head.SetPayloadSize(newp->GetSize());
        head.SetIdentification(rxQp->m_ipid++);

        {
            FlowIDNUMTag fit;
            if (p->PeekPacketTag(fit)) {
                newp->AddPacketTag(fit);
            }
        }

        newp->AddHeader(head);
        AddHeader(newp, 0x800);  // Attach PPP header

        // send
        uint32_t nic_idx = GetNicIdxOfRxQp(rxQp);
        SendRxControlPacket(m_nic[nic_idx].dev, newp, rxControlDelayKind);
    }
    return 0;
}

int RdmaHw::ReceiveCnp(Ptr<Packet> p, CustomHeader &ch) {
    std::cerr << "ReceiveCnp is called. Exit this program." << std::endl;
    exit(1);
    // QCN on NIC
    // This is a Congestion signal
    // Then, extract data from the congestion packet.
    // We assume, without verify, the packet is destinated to me
    uint32_t qIndex = ch.cnp.qIndex;
    if (qIndex == 1) {  // DCTCP
        std::cout << "TCP--ignore\n";
        return 0;
    }
    NS_ASSERT(ch.cnp.fid == ch.udp.dport);
    uint16_t udpport = ch.cnp.fid;  // corresponds to the sport (CNP's dport)
    uint16_t sport = ch.udp.sport;  // corresponds to the dport (CNP's sport)
    uint8_t ecnbits = ch.cnp.ecnBits;
    uint16_t qfb = ch.cnp.qfb;
    uint16_t total = ch.cnp.total;

    uint32_t i;
    // get qp
    auto key = GetQpKey(ch.sip, udpport, sport, qIndex);
    Ptr<RdmaQueuePair> qp = GetQp(key);
    if (qp == NULL) {
        // lookup akashic memory
        if (akashic_Qp.find(key) != akashic_Qp.end()) {
            // printf("[GetQPCNP] Akashic access: %u(%d) -> %u(%d)\n", this->m_node->GetId(),
            // udpport, ch.sip, sport);
            return 1;  // just drop
        } else {
            printf("ERROR: QCN NIC cannot find the flow\n");
            exit(1);
        }
    }
    // get nic
    uint32_t nic_idx = GetNicIdxOfQp(qp);
    Ptr<QbbNetDevice> dev = m_nic[nic_idx].dev;

    if (qp->m_rate == 0)  // lazy initialization
    {
        qp->m_rate = dev->GetDataRate();
        if (m_cc_mode == 1) {
            qp->mlx.m_targetRate = dev->GetDataRate();
        } else if (m_cc_mode == 3) {
            qp->hp.m_curRate = dev->GetDataRate();
            qp->hp.m_baseRtt = std::max(1e-9, (double)qp->m_baseRtt * 1e-9);
            qp->hp.m_targetUtil = m_targetUtil;
            qp->hp.m_eta = std::max(1e-6, m_targetUtil);
            qp->hp.m_WAI = std::max<double>(1.0, m_mtu);
            qp->hp.m_maxStage = m_miThresh;
            double initWin = qp->m_max_rate.GetBitRate() * qp->hp.m_baseRtt / 8.0;
            initWin *= std::max<uint32_t>(qp->psnPath.k, 1);
            initWin = std::max(initWin, 1.0);
            qp->hp.m_W = initWin;
            qp->hp.m_Wc = initWin;
        } else if (m_cc_mode == 7) {
            qp->tmly.m_curRate = dev->GetDataRate();
        }
    }
    return 0;
}

int RdmaHw::ReceiveAck(Ptr<Packet> p, CustomHeader &ch) {
    uint16_t qIndex = ch.ack.pg;
    uint16_t port = ch.ack.dport;   // sport for this host
    uint16_t sport = ch.ack.sport;  // dport for this host (sport of ACK packet)
    uint32_t seq = ch.ack.seq;
    uint8_t cnp = (ch.ack.flags >> qbbHeader::FLAG_CNP) & 1;
    int i;
    auto key = GetQpKey(ch.sip, port, sport, qIndex);
    Ptr<RdmaQueuePair> qp = GetQp(key);
    if (qp == NULL) {
        // lookup akashic memory
        if (akashic_Qp.find(key) != akashic_Qp.end()) {
            // printf("[GetQPACK] Akashic access: %u(%d) -> %u(%d)\n", this->m_node->GetId(), port,
            // ch.sip, sport);
            return 1;
        } else {
            printf("ERROR: Node: %u %s - NIC cannot find the flow\n", m_node->GetId(),
                   (ch.l3Prot == 0xFC ? "ACK" : "NACK"));
            exit(1);
        }
    }

    uint32_t nic_idx = GetNicIdxOfQp(qp);
    Ptr<QbbNetDevice> dev = m_nic[nic_idx].dev;
    if (m_enableFalcon) {
        const uint64_t falconBitmap =
            (static_cast<uint64_t>(ch.ack.falconBitmapHi) << 32) | ch.ack.falconBitmapLo;
        const uint16_t falconBitmapBits = ch.ack.falconBitmapBits;
        const uint32_t packetSize = qp->psnPath.packetSize == 0 ? m_mtu : qp->psnPath.packetSize;
        const uint32_t cumAckPsn = packetSize == 0 ? 0 : seq / packetSize;
        const uint32_t old_snd_una = qp->snd_una;
        if (m_enableConsoleLog == 1 &&
            PsnListOverlapsFalconBitmap(cumAckPsn, falconBitmapBits, falconBitmap,
                                        m_consoleLogPsns)) {
            std::cout << "[PSN_TRACE_ACK_RX] t=" << Simulator::Now().GetNanoSeconds()
                      << " node=" << m_node->GetId()
                      << " qp=" << qp->m_flow_id
                      << " wireType=" << ((ch.l3Prot == 0xFC) ? "ACK" :
                                          ((ch.l3Prot == 0xFD) ? "NACK" : "OTHER"))
                      << " cumAckSeq=" << seq
                      << " cumAckPsn=" << cumAckPsn
                      << " bitmapBits=" << falconBitmapBits
                      << " bitmap=0x" << std::hex << falconBitmap << std::dec
                      << " snd_una_psn_before="
                      << (packetSize == 0 ? 0 : old_snd_una / packetSize)
                      << std::endl;
        }
        qp->Acknowledge(seq);
        Time falconRetransAgeThreshold = NanoSeconds(0);
        if (m_falconRetransRttK > 0.0) {
            uint64_t sampleRttNs = 0;
            uint64_t nowTs = Simulator::Now().GetTimeStep();
            if (ch.ack.ih.ts != 0 && nowTs >= ch.ack.ih.ts) {
                sampleRttNs = nowTs - ch.ack.ih.ts;
            }
            if (sampleRttNs != 0) {
                if (qp->m_falconSrttValid) {
                    qp->m_falconSrttNs =
                        static_cast<uint64_t>(0.875 * qp->m_falconSrttNs + 0.125 * sampleRttNs);
                } else {
                    qp->m_falconSrttNs = sampleRttNs;
                    qp->m_falconSrttValid = true;
                }
            } else if (!qp->m_falconSrttValid && qp->m_baseRtt != 0) {
                qp->m_falconSrttNs = qp->m_baseRtt;
                qp->m_falconSrttValid = true;
            }
            if (qp->m_falconSrttValid) {
                falconRetransAgeThreshold =
                    NanoSeconds(static_cast<uint64_t>(m_falconRetransRttK * qp->m_falconSrttNs));
            }
        }
        std::vector<std::pair<uint32_t, Time> > deferredRetransSeqs;
        std::vector<uint32_t> retransSeqs =
            m_falcon.OnAck(key, seq, falconBitmapBits, falconBitmap, qp->psnPath.packetSize,
                           Simulator::Now(), falconRetransAgeThreshold, &deferredRetransSeqs);
        qp->m_selectiveAckedBytes = m_falcon.GetSelectiveAckedBytes(key);
        while (!qp->psnPath.pendingRetrans.empty()) {
            const PsnRetransRequest &req = qp->psnPath.pendingRetrans.front();
            uint32_t reqSeq = req.startPsn * qp->psnPath.packetSize;
            if (reqSeq >= seq) break;
            qp->psnPath.pendingRetrans.pop_front();
        }
        for (size_t idx = 0; idx < retransSeqs.size(); ++idx) {
            EnqueueFalconBitmapRetrans(qp, retransSeqs[idx]);
        }
        for (size_t idx = 0; idx < deferredRetransSeqs.size(); ++idx) {
            ScheduleDeferredFalconBitmapRetrans(qp, deferredRetransSeqs[idx].first,
                                                deferredRetransSeqs[idx].second);
        }
        if (qp->snd_nxt < qp->snd_una) {
            qp->snd_nxt = qp->snd_una;
        }
        if (m_enableConsoleLog == 1 &&
            PsnListOverlapsFalconBitmap(cumAckPsn, falconBitmapBits, falconBitmap,
                                        m_consoleLogPsns)) {
            std::cout << "[PSN_TRACE_ACK_APPLY] t=" << Simulator::Now().GetNanoSeconds()
                      << " node=" << m_node->GetId()
                      << " qp=" << qp->m_flow_id
                      << " cumAckPsn=" << cumAckPsn
                      << " bitmapBits=" << falconBitmapBits
                      << " bitmap=0x" << std::hex << falconBitmap << std::dec
                      << " snd_una_psn_after="
                      << (packetSize == 0 ? 0 : qp->snd_una / packetSize)
                      << " selectiveAckedBytes=" << qp->m_selectiveAckedBytes
                      << " bitmapRetransCount=" << retransSeqs.size()
                      << " pendingRetrans=" << qp->psnPath.pendingRetrans.size()
                      << std::endl;
        }
        if (qp->IsFinished()) {
            QpComplete(qp);
        }
        if (!qp->IsFinished() && qp->GetOnTheFly() > 0) {
            if (qp->m_retransmit.IsRunning()) qp->m_retransmit.Cancel();
            qp->m_retransmit = Simulator::Schedule(qp->GetRto(m_mtu), &RdmaHw::HandleTimeout,
                                                   this, qp, qp->GetRto(m_mtu));
        }
        if (cnp && m_cc_mode == 1) {
            cnp_received_mlx(qp);
        }
        if (m_cc_mode == 3) {
            bool ack_progress = (qp->snd_una > old_snd_una);
            if (ch.ack.seq > qp->hp.m_lastAckSeq) {
                qp->hp.m_lastAckSeq = ch.ack.seq;
            }
            HandleAckHp(qp, p, ch, ack_progress);
        } else if (m_cc_mode == 7) {
            HandleAckTimely(qp, p, ch);
        } else if (m_cc_mode == 8) {
            HandleAckDctcp(qp, p, ch);
        }
        if (m_hpccTrace) {
            std::cout << "[FALCON_ACK_RX] t=" << Simulator::Now().GetNanoSeconds()
                      << " qp=" << qp->m_flow_id << " cumAck=" << seq
                      << " bitmapBits=" << falconBitmapBits << " bitmap=0x" << std::hex
                      << falconBitmap << std::dec
                      << " selectiveAckedBytes=" << qp->m_selectiveAckedBytes
                      << " pendingRetrans=" << qp->psnPath.pendingRetrans.size() << std::endl;
        }
        dev->TriggerTransmit();
        return 0;
    }

    if (m_enableOrnic) {
        uint32_t old_snd_una = qp->snd_una;
        qp->Acknowledge(seq);
        m_ornicRetrans.OnAck(key, seq);
        m_ornicRetrans.ClearRetransUpTo(key, seq);
        while (!qp->ornic.pendingRetrans.empty() && qp->ornic.pendingRetrans.front() < seq) {
            qp->ornic.pendingRetrans.pop_front();
        }
        if (qp->snd_nxt < qp->snd_una) {
            qp->snd_nxt = qp->snd_una;
        }
        if (ch.l3Prot == 0xFD && ch.ack.irnNackSize != 0) {
            uint32_t missingSeq = ch.ack.irnNack;
            if (missingSeq < qp->snd_una) {
                missingSeq = static_cast<uint32_t>(qp->snd_una);
            }
            if (m_ornicRetrans.TryScheduleRetrans(key, missingSeq)) {
                qp->ornic.pendingRetrans.push_back(missingSeq);
            }
            LogSenderNackRx(m_node->GetId(), qp, missingSeq, qp->ornic.packetSize,
                            "ORNIC_SELECTIVE");
            if (m_hpccTrace) {
                std::cout << "[ORNIC_NACK_ENQUEUE] t=" << Simulator::Now().GetNanoSeconds()
                          << " qp=" << qp->m_flow_id
                          << " nackSeq=" << ch.ack.irnNack
                          << " nackSize=" << ch.ack.irnNackSize
                          << " queueDepth=" << qp->ornic.pendingRetrans.size()
                          << std::endl;
            }
        } else if (m_hpccTrace) {
            std::cout << "[ORNIC_ACK_RX] t=" << Simulator::Now().GetNanoSeconds()
                      << " qp=" << qp->m_flow_id << " ackSeq=" << seq
                      << " snd_nxt=" << qp->snd_nxt << " snd_una=" << qp->snd_una
                      << std::endl;
        }
        if (qp->IsFinished()) {
            QpComplete(qp);
        }
        if (!qp->IsFinished() && qp->GetOnTheFly() > 0) {
            if (qp->m_retransmit.IsRunning()) qp->m_retransmit.Cancel();
            qp->m_retransmit = Simulator::Schedule(qp->GetRto(m_mtu), &RdmaHw::HandleTimeout,
                                                   this, qp, qp->GetRto(m_mtu));
        }
        if (cnp && m_cc_mode == 1) {
            cnp_received_mlx(qp);
        }
        if (m_cc_mode == 3) {
            bool ack_progress = (qp->snd_una > old_snd_una);
            if (ch.ack.seq > qp->hp.m_lastAckSeq) {
                qp->hp.m_lastAckSeq = ch.ack.seq;
            }
            HandleAckHp(qp, p, ch, ack_progress);
        } else if (m_cc_mode == 7) {
            HandleAckTimely(qp, p, ch);
        } else if (m_cc_mode == 8) {
            HandleAckDctcp(qp, p, ch);
        }
        dev->TriggerTransmit();
        return 0;
    }

    if (m_enableBitmapRetrans) {
        uint32_t old_snd_una = qp->snd_una;
        qp->Acknowledge(seq);
        m_bitmapRetrans.OnAck(key, seq);
        m_bitmapRetrans.ClearRetransUpTo(key, seq);
        while (!qp->psnPath.pendingRetrans.empty()) {
            const PsnRetransRequest &req = qp->psnPath.pendingRetrans.front();
            uint32_t reqSeq = req.startPsn * qp->psnPath.packetSize;
            if (reqSeq >= seq) break;
            qp->psnPath.pendingRetrans.pop_front();
        }
        if (qp->snd_nxt < qp->snd_una) {
            qp->snd_nxt = qp->snd_una;
        }
        if (ch.l3Prot == 0xFD) {
            std::vector<uint32_t> missingSeqs = m_bitmapRetrans.CollectMissingSeqs(key, seq);
            for (size_t midx = 0; midx < missingSeqs.size(); ++midx) {
                uint32_t missingSeq = missingSeqs[midx];
                uint32_t startPsn = missingSeq / qp->psnPath.packetSize;
                if (m_bitmapRetrans.TryScheduleRetrans(key, missingSeq)) {
                    PsnRetransRequest req;
                    req.startPsn = startPsn;
                    req.endPsn = startPsn;
                    req.epoch = qp->psnPath.mappingEpoch;
                    qp->psnPath.pendingRetrans.push_back(req);
                }
            }
            uint32_t packetSize = qp->psnPath.packetSize == 0 ? m_mtu : qp->psnPath.packetSize;
            uint32_t ackPsn = packetSize == 0 ? 0 : seq / packetSize;
            if (m_enableConsoleLog == 1 && IsPsnInList(ackPsn, m_consoleLogPsns)) {
                std::cout << "[PSN_TRACE_ACK_RX] t=" << Simulator::Now().GetNanoSeconds()
                          << " node=" << m_node->GetId()
                          << " qp=" << qp->m_flow_id
                          << " wireType=NACK"
                          << " ackSeq=" << seq
                          << " ackPsn=" << ackPsn
                          << " snd_nxt_psn=" << (packetSize == 0 ? 0 : qp->snd_nxt / packetSize)
                          << " snd_una_psn=" << (packetSize == 0 ? 0 : qp->snd_una / packetSize)
                          << " missingCount=" << missingSeqs.size()
                          << " pendingRetrans=" << qp->psnPath.pendingRetrans.size()
                          << std::endl;
            }
            if (m_hpccTrace) {
                std::cout << "[BITMAP_NACK_ENQUEUE] t=" << Simulator::Now().GetNanoSeconds()
                          << " qp=" << qp->m_flow_id << " ackSeq=" << seq
                          << " missingCount=" << missingSeqs.size()
                          << " queueDepth=" << qp->psnPath.pendingRetrans.size()
                          << std::endl;
            }
        } else if (m_hpccTrace) {
            std::cout << "[BITMAP_ACK_RX] t=" << Simulator::Now().GetNanoSeconds()
                      << " qp=" << qp->m_flow_id << " ackSeq=" << seq
                      << " snd_nxt=" << qp->snd_nxt << " snd_una=" << qp->snd_una << std::endl;
        }
        if (ch.l3Prot != 0xFD) {
            uint32_t packetSize = qp->psnPath.packetSize == 0 ? m_mtu : qp->psnPath.packetSize;
            uint32_t ackPsn = packetSize == 0 ? 0 : seq / packetSize;
            uint32_t ackedThroughPsn = ackPsn == 0 ? 0 : ackPsn - 1;
            if (m_enableConsoleLog == 1 &&
                (IsPsnInList(ackPsn, m_consoleLogPsns) ||
                 IsPsnInList(ackedThroughPsn, m_consoleLogPsns))) {
                std::cout << "[PSN_TRACE_ACK_RX] t=" << Simulator::Now().GetNanoSeconds()
                          << " node=" << m_node->GetId()
                          << " qp=" << qp->m_flow_id
                          << " wireType=ACK"
                          << " ackSeq=" << seq
                          << " ackPsn=" << ackPsn
                          << " ackedThroughPsn=" << ackedThroughPsn
                          << " snd_nxt_psn=" << (packetSize == 0 ? 0 : qp->snd_nxt / packetSize)
                          << " snd_una_psn=" << (packetSize == 0 ? 0 : qp->snd_una / packetSize)
                          << " pendingRetrans=" << qp->psnPath.pendingRetrans.size()
                          << std::endl;
            }
        }

        if (qp->IsFinished()) {
            QpComplete(qp);
        }
        if (!qp->IsFinished() && qp->GetOnTheFly() > 0) {
            if (qp->m_retransmit.IsRunning()) qp->m_retransmit.Cancel();
            qp->m_retransmit = Simulator::Schedule(qp->GetRto(m_mtu), &RdmaHw::HandleTimeout,
                                                   this, qp, qp->GetRto(m_mtu));
        }

        if (cnp && m_cc_mode == 1) {
            cnp_received_mlx(qp);
        }
        if (m_cc_mode == 3) {
            bool ack_progress = (qp->snd_una > old_snd_una);
            if (ch.ack.seq > qp->hp.m_lastAckSeq) {
                qp->hp.m_lastAckSeq = ch.ack.seq;
            }
            HandleAckHp(qp, p, ch, ack_progress);
        } else if (m_cc_mode == 7) {
            HandleAckTimely(qp, p, ch);
        } else if (m_cc_mode == 8) {
            HandleAckDctcp(qp, p, ch);
        }
        dev->TriggerTransmit();
        return 0;
    }

    bool isDirectNack = ((ch.ack.flags >> qbbHeader::FLAG_DIRECT_NACK) & 1) != 0;
    bool hasNackSyndrome = ch.ack.irnNackSize != 0;
    bool isLossNack = (ch.l3Prot == 0xFD) && (isDirectNack || hasNackSyndrome);
    if (isLossNack) {
        const char *action = qp->psnPath.pathAwareRetransEnabled ? "ENQUEUE" :
                             (m_enableTxNackGoBack ? "GOBACK" : "RECOVER");
        LogSenderNackRx(m_node->GetId(), qp, ch.ack.irnNack, qp->psnPath.packetSize, action);
    }
    const char *wireType = (ch.l3Prot == 0xFC) ? "ACK" : ((ch.l3Prot == 0xFD) ? "NACK" : "OTHER");
    const char *semanticType = isLossNack ? "NACK" : "ACK";

    if (qp->psnPath.pathSelectionEnabled && qp->psnPath.activePathCount > 0) {
        uint32_t statSeq = isLossNack ? ch.ack.irnNack : seq;
        uint32_t statPsn = PsnPath::GetPsnFromSeq(statSeq, qp->psnPath.packetSize);
        uint32_t pathId = SelectPathFromStats(statPsn, qp->psnPath.k, qp->psnPath.o,
                                              qp->psnPath.pathStats);
        if (pathId < qp->psnPath.pathStats.size()) {
            if (isLossNack) {
                qp->psnPath.pathStats[pathId].nackCount++;
            } else {
                qp->psnPath.pathStats[pathId].ackCount++;
            }
            qp->psnPath.pathStats[pathId].score =
                static_cast<double>(qp->psnPath.pathStats[pathId].ackCount) -
                m_psnPathBeta * static_cast<double>(qp->psnPath.pathStats[pathId].nackCount);
            if (m_enableConsoleLog >= 1 && isLossNack) {
                std::cout << "[PATH_NACK_STAT] t=" << Simulator::Now().GetNanoSeconds()
                          << " qp=" << qp->m_flow_id
                          << " ackSeq=" << seq
                          << " nackSeq=" << ch.ack.irnNack
                          << " nackSize=" << ch.ack.irnNackSize
                          << " statSeq=" << statSeq
                          << " statPsn=" << statPsn
                          << " pathId=" << pathId
                          << " pathAck=" << qp->psnPath.pathStats[pathId].ackCount
                          << " pathNack=" << qp->psnPath.pathStats[pathId].nackCount
                          << " pathScore=" << qp->psnPath.pathStats[pathId].score
                          << " beta=" << m_psnPathBeta
                          << " activePaths="
                          << FormatPathIndices(CollectActivePathIndices(qp->psnPath.pathStats))
                          << std::endl;
            }
            if (m_hpccTrace) {
                std::cout << "[PATH_STAT_UPDATE] t=" << Simulator::Now().GetNanoSeconds()
                          << " qp=" << qp->m_flow_id << " ackType=0x" << std::hex
                          << static_cast<uint32_t>(ch.l3Prot) << std::dec << " ackSeq=" << seq
                          << " wireType=" << wireType
                          << " semanticType=" << semanticType
                          << " directNack=" << (isDirectNack ? 1 : 0)
                          << " nackSeq=" << ch.ack.irnNack
                          << " nackSize=" << ch.ack.irnNackSize
                          << " isLossNack=" << (isLossNack ? 1 : 0)
                          << " statPsn=" << statPsn << " pathId=" << pathId
                          << " ackCount=" << qp->psnPath.pathStats[pathId].ackCount
                          << " nackCount=" << qp->psnPath.pathStats[pathId].nackCount
                          << " score=" << qp->psnPath.pathStats[pathId].score
                          << " epoch=" << qp->psnPath.mappingEpoch
                          << " activePaths="
                          << FormatPathIndices(CollectActivePathIndices(qp->psnPath.pathStats))
                          << std::endl;
            }
        }
    }

    uint32_t old_snd_una = qp->snd_una;
    if (m_ack_interval == 0)
        std::cout << "ERROR: shouldn't receive ack\n";
    else {
        if (!m_backto0) {
            qp->Acknowledge(seq);
        } else {
            uint32_t goback_seq = seq / m_chunk * m_chunk;
            qp->Acknowledge(goback_seq);
        }
        if (qp->irn.m_enabled) {
            // IRN can receive both ACK(0xFC) and NACK(0xFD). Under PSN-PATH the receiver may
            // generate regular ACKs for in-order progress, so do not force NACK-only handling.
            if (seq > qp->irn.m_highest_ack) qp->irn.m_highest_ack = seq;

            if (((ch.ack.flags >> qbbHeader::FLAG_DIRECT_NACK) & 1) == 0 &&
                ch.ack.irnNackSize != 0) {
                // ch.ack.irnNack contains the seq triggered this NACK
                qp->irn.m_sack.sack(ch.ack.irnNack, ch.ack.irnNackSize);
            }

            uint32_t sack_seq, sack_len;
            if (qp->irn.m_sack.peekFrontBlock(&sack_seq, &sack_len)) {
                if (qp->snd_una == sack_seq) {
                    qp->snd_una += sack_len;
                }
            }

            qp->irn.m_sack.discardUpTo(qp->snd_una);

            if (qp->snd_nxt < qp->snd_una) {
                qp->snd_nxt = qp->snd_una;
            }
            // if (qp->irn.m_sack.IsEmpty())  { //
            if (qp->irn.m_recovery && qp->snd_una >= qp->irn.m_recovery_seq) {
                qp->irn.m_recovery = false;
            }

            if (qp->psnPath.pathAwareRetransEnabled &&
                ch.ack.irnNackSize != 0) {
                uint32_t startPsn =
                    PsnPath::GetPsnFromSeq(ch.ack.irnNack, qp->psnPath.packetSize);
                uint32_t endPsn = startPsn;
                if (qp->psnPath.packetSize > 0) {
                    endPsn = PsnPath::GetPsnFromSeq(ch.ack.irnNack + ch.ack.irnNackSize - 1,
                                                    qp->psnPath.packetSize);
                }
                bool duplicated = false;
                for (size_t idx = 0; idx < qp->psnPath.pendingRetrans.size(); ++idx) {
                    const PsnRetransRequest &req = qp->psnPath.pendingRetrans[idx];
                    if (req.startPsn == startPsn && req.endPsn == endPsn &&
                        req.epoch == qp->psnPath.mappingEpoch) {
                        duplicated = true;
                        break;
                    }
                }
                if (!duplicated) {
                    PsnRetransRequest req;
                    req.startPsn = startPsn;
                    req.endPsn = endPsn;
                    req.epoch = qp->psnPath.mappingEpoch;
                    qp->psnPath.pendingRetrans.push_back(req);
                    if (qp->psnPath.pendingRetrans.size() > 128) {
                        qp->psnPath.pendingRetrans.pop_front();
                    }
                    if (m_hpccTrace) {
                        std::cout << "[RETX_ENQUEUE] t=" << Simulator::Now().GetNanoSeconds()
                                  << " qp=" << qp->m_flow_id << " reason=IRN_NACK"
                                  << " startPsn=" << startPsn << " endPsn=" << endPsn
                                  << " epoch=" << qp->psnPath.mappingEpoch
                                  << " queueDepth=" << qp->psnPath.pendingRetrans.size()
                                  << std::endl;
                    }
                }
                qp->irn.m_recovery = true;
            }
        } else {
            if (qp->snd_nxt < qp->snd_una) {
                qp->snd_nxt = qp->snd_una;
            }
            if (qp->psnPath.pathAwareRetransEnabled && ch.l3Prot == 0xFD &&
                ch.ack.irnNackSize != 0) {
                uint32_t startPsn =
                    PsnPath::GetPsnFromSeq(ch.ack.irnNack, qp->psnPath.packetSize);
                uint32_t endPsn = startPsn;
                if (qp->psnPath.packetSize > 0) {
                    endPsn = PsnPath::GetPsnFromSeq(ch.ack.irnNack + ch.ack.irnNackSize - 1,
                                                    qp->psnPath.packetSize);
                }
                PsnRetransRequest req;
                req.startPsn = startPsn;
                req.endPsn = endPsn;
                req.epoch = qp->psnPath.mappingEpoch;
                qp->psnPath.pendingRetrans.push_back(req);
                if (qp->psnPath.pendingRetrans.size() > 128) {
                    qp->psnPath.pendingRetrans.pop_front();
                }
                if (m_hpccTrace) {
                    std::cout << "[RETX_ENQUEUE] t=" << Simulator::Now().GetNanoSeconds()
                              << " qp=" << qp->m_flow_id << " reason=NACK"
                              << " startPsn=" << startPsn << " endPsn=" << endPsn
                              << " epoch=" << qp->psnPath.mappingEpoch
                              << " queueDepth=" << qp->psnPath.pendingRetrans.size()
                              << std::endl;
                }
            }
        }
        if (qp->IsFinished()) {
            QpComplete(qp);
        }
    }

    /**
     * IB Spec Vol. 1 o9-85
     * The requester need not separately time each request launched into the
     * fabric, but instead simply begins the timer whenever it is expecting a response.
     * Once started, the timer is restarted each time an acknowledge
     * packet is received as long as there are outstanding expected responses.
     * The timer does not detect the loss of a particular expected acknowledge
     * packet, but rather simply detects the persistent absence of response
     * packets.
     * */
    if (!qp->IsFinished() && qp->GetOnTheFly() > 0) {
        if (qp->m_retransmit.IsRunning()) qp->m_retransmit.Cancel();
        qp->m_retransmit = Simulator::Schedule(qp->GetRto(m_mtu), &RdmaHw::HandleTimeout, this, qp,
                                               qp->GetRto(m_mtu));
    }

    if (m_irn) {
        if ((ch.ack.flags >> qbbHeader::FLAG_DIRECT_NACK) & 1) {
            if (m_enableTxNackGoBack) {
                uint32_t oldSndNxt = static_cast<uint32_t>(qp->snd_nxt);
                uint32_t directSeq = ch.ack.irnNack;
                if (directSeq < qp->snd_una) {
                    directSeq = static_cast<uint32_t>(qp->snd_una);
                }
                qp->irn.m_recovery_seq = std::max(oldSndNxt, directSeq);
                qp->snd_nxt = directSeq;
                qp->irn.m_recovery = true;
                if (m_enableRxOooNack && m_enableTxNackGoBack &&
                    m_txNackRetransInterval.GetNanoSeconds() > 0) {
                    Time delayedNextAvail = Simulator::Now() + m_txNackRetransInterval;
                    if (qp->m_nextAvail < delayedNextAvail) {
                        qp->m_nextAvail = delayedNextAvail;
                    }
                    dev->UpdateNextAvail(qp->m_nextAvail);
                }
                if (m_hpccTrace) {
                    std::cout << "[NACK_GOBACK] t=" << Simulator::Now().GetNanoSeconds()
                              << " qp=" << qp->m_flow_id
                              << " directNackSeq=" << ch.ack.irnNack
                              << " ackSeq=" << ch.ack.seq
                              << " oldSndUna=" << qp->snd_una
                              << " oldSndNxt=" << oldSndNxt
                              << " newSndNxt=" << qp->snd_nxt
                              << " recoverySeq=" << qp->irn.m_recovery_seq
                              << " txNackRetransDelayNs="
                              << (m_enableRxOooNack && m_enableTxNackGoBack
                                      ? m_txNackRetransInterval.GetNanoSeconds()
                                      : 0)
                              << std::endl;
                }
            }
        } else if (ch.ack.irnNackSize != 0) {
            if (!qp->psnPath.pathAwareRetransEnabled) {
                if (!qp->irn.m_recovery) {
                    qp->irn.m_recovery_seq = qp->snd_nxt;
                    RecoverQueue(qp);
                    qp->irn.m_recovery = true;
                }
            }
        } else {
            if (qp->irn.m_recovery) {
                qp->irn.m_recovery = false;
            }
        }

    } else if (ch.l3Prot == 0xFD && !qp->psnPath.pathAwareRetransEnabled) {  // NACK
        if (m_enableTxNackGoBack && ch.ack.irnNackSize != 0) {
            uint32_t oldSndNxt = static_cast<uint32_t>(qp->snd_nxt);
            uint32_t goBackSeq =
                std::max(ch.ack.irnNack, static_cast<uint32_t>(qp->snd_una));
            qp->snd_nxt = goBackSeq;
            if (m_enableRxOooNack && m_enableTxNackGoBack &&
                m_txNackRetransInterval.GetNanoSeconds() > 0) {
                Time delayedNextAvail = Simulator::Now() + m_txNackRetransInterval;
                if (qp->m_nextAvail < delayedNextAvail) {
                    qp->m_nextAvail = delayedNextAvail;
                }
                dev->UpdateNextAvail(qp->m_nextAvail);
            }
            if (m_hpccTrace) {
                std::cout << "[NACK_GOBACK] t=" << Simulator::Now().GetNanoSeconds()
                          << " qp=" << qp->m_flow_id
                          << " nackSeq=" << ch.ack.irnNack
                          << " ackSeq=" << ch.ack.seq
                          << " oldSndUna=" << qp->snd_una
                          << " oldSndNxt=" << oldSndNxt
                          << " newSndNxt=" << qp->snd_nxt
                          << " txNackRetransDelayNs="
                          << (m_enableRxOooNack && m_enableTxNackGoBack
                                  ? m_txNackRetransInterval.GetNanoSeconds()
                                  : 0)
                          << std::endl;
            }
        } else {
            RecoverQueue(qp);
        }
    }

    // handle cnp
    if (cnp) {
        if (m_cc_mode == 1) {  // mlx version
            cnp_received_mlx(qp);
        }
    }

    if (m_cc_mode == 3) {
        bool ack_progress = (qp->snd_una > old_snd_una);
        if (ch.ack.seq > qp->hp.m_lastAckSeq) {
            qp->hp.m_lastAckSeq = ch.ack.seq;
        }
        if (m_hpccTrace) {
            std::cout << "[HPCC_ENTRY] cc_mode=" << m_cc_mode
                      << " ack_type=0x" << std::hex << (uint32_t)ch.l3Prot << std::dec
                      << " ack_seq=" << ch.ack.seq
                      << " ack_progress=" << (ack_progress ? 1 : 0)
                      << " enter_HandleAckHp=1" << std::endl;
            std::cout << "[HPCC_ACK_RX] t=" << Simulator::Now().GetNanoSeconds()
                      << " node=" << m_node->GetId()
                      << " sport=" << ch.ack.sport << " dport=" << ch.ack.dport
                      << " ackSeq=" << ch.ack.seq << " flags=" << ch.ack.flags
                      << " progress=" << (ack_progress ? 1 : 0)
                      << " nhop=" << ch.ack.ih.nhop << " snd_nxt=" << qp->snd_nxt
                      << " snd_una=" << qp->snd_una << " rate=" << qp->m_rate.GetBitRate()
                      << std::endl;
        }
        HandleAckHp(qp, p, ch, ack_progress);
    } else if (m_cc_mode == 7) {
        HandleAckTimely(qp, p, ch);
    } else if (m_cc_mode == 8) {
        HandleAckDctcp(qp, p, ch);
    }
    // ACK may advance the on-the-fly window, allowing more packets to send
    dev->TriggerTransmit();
    return 0;
}

size_t RdmaHw::getIrnBufferOverhead() {
    size_t overhead = 0;
    for (auto it = m_rxQpMap.begin(); it != m_rxQpMap.end(); it++) {
        overhead += it->second->m_irn_sack_.getSackBufferOverhead();
    }
    return overhead;
}

int RdmaHw::Receive(Ptr<Packet> p, CustomHeader &ch) {
    // #if (SLB_DEBUG == true)
    //     std::cout << "[RdmaHw::Receive] Node(" << m_node->GetId() << ")," << PARSE_FIVE_TUPLE(ch)
    //     << "l3Prot:" << ch.l3Prot << ",at" << Simulator::Now() << std::endl;
    // #endif

    if (ch.l3Prot == 0x11) {  // UDP
        return ReceiveUdp(p, ch);
    } else if (ch.l3Prot == 0xFF) {  // CNP
        return ReceiveCnp(p, ch);
    } else if (ch.l3Prot == 0xFD) {  // NACK
        return ReceiveAck(p, ch);
    } else if (ch.l3Prot == 0xFC) {  // ACK
        return ReceiveAck(p, ch);
    }
    return 0;
}

/**
 * @brief Check sequence number when UDP DATA is received
 *
 * @return int
 * 0: should not reach here
 * 1: generate ACK
 * 2: still in loss recovery of IRN
 * 4: OoO, but skip to send NACK as it is already NACKed.
 * 6: NACK but functionality is ACK (indicating all packets are received)
 */
int RdmaHw::ReceiverCheckSeq(uint32_t &seq, Ptr<RdmaRxQueuePair> q, uint32_t size, bool &cnp) {
    q->m_lastNackReason = "";
    q->m_ackSeqOverrideValid = false;
    if (m_enablePathAwareRetrans || m_enablePsnPath) {
        return ReceiverCheckSeqPsnPath(seq, q, size, cnp);
    }
    uint32_t packetSize = m_psnPathPacketSize == 0 ? m_mtu : m_psnPathPacketSize;
    uint32_t expected = q->ReceiverNextExpectedSeq;
    uint32_t expectedBefore = expected;
    uint32_t origSeq = seq;
    uint32_t psn = seq / packetSize;
    auto logRecvCheck = [&](const char *decision, const char *reason) {
        if (!m_hpccTrace) return;
        std::cout << "[RECV_CHECK] t=" << Simulator::Now().GetNanoSeconds()
                  << " node=" << m_node->GetId() << " seq=" << origSeq<< " psn=" << psn
                  << " expectedBefore=" << expectedBefore
                  << " expectedNow=" << q->ReceiverNextExpectedSeq
                  << " size=" << size
                  << " decision=" << decision << " reason=" << reason
                  << " irn=" << (m_irn ? 1 : 0) << std::endl;
    };
    if (seq == expected || (seq < expected && seq + size >= expected)) {
        if (m_irn) {
            if (q->m_milestone_rx < seq + size) q->m_milestone_rx = seq + size;
            q->ReceiverNextExpectedSeq += size - (expected - seq);
            {
                uint32_t sack_seq, sack_len;
                if (q->m_irn_sack_.peekFrontBlock(&sack_seq, &sack_len)) {
                    if (sack_seq <= q->ReceiverNextExpectedSeq)
                        q->ReceiverNextExpectedSeq +=
                            (sack_len - (q->ReceiverNextExpectedSeq - sack_seq));
                }
            }
            size_t progress = q->m_irn_sack_.discardUpTo(q->ReceiverNextExpectedSeq);
            if (q->m_irn_sack_.IsEmpty()) {
                logRecvCheck("ACK", "IRN_IN_ORDER_RECOVERY_DONE");
                return 6;  // This generates NACK, but actually functions as an ACK (indicates all
                           // packet has been received)
            } else {
                // should we put nack timer here
                logRecvCheck("NACK", "IRN_IN_ORDER_RECOVERY");
                return 2;  // Still in loss recovery mode of IRN
            }
            return 0;  // should not reach here
        }

        q->ReceiverNextExpectedSeq += size - (expected - seq);
        if (q->ReceiverNextExpectedSeq >= q->m_milestone_rx) {
            q->m_milestone_rx +=
                m_ack_interval;  // if ack_interval is small (e.g., 1), condition is meaningless
            logRecvCheck("ACK", "IN_ORDER");
            return 1;            // Generate ACK
        } else if (q->ReceiverNextExpectedSeq % m_chunk == 0) {
            logRecvCheck("ACK", "IN_ORDER_CHUNK");
            return 1;
        } else {
            logRecvCheck("HOLD", "IN_ORDER_NO_ACK");
            return 5;
        }
    } else if (seq > expected) {
        // Generate NACK
        if (m_irn) {
            if (m_enableRxOooNack) {
                if (Simulator::Now() < q->m_nackTimer && q->m_lastNACK == expected) {
                    logRecvCheck("HOLD", "IRN_OOO_DIRECT_NACK_DUP");
                    return 4;
                }
                q->m_nackTimer = Simulator::Now() + MicroSeconds(m_nack_interval);
                q->m_lastNACK = expected;
                seq = expected;
                cnp = true;
                logRecvCheck("NACK", "IRN_OOO_DIRECT_NACK");
                return 2;
            }
            if (q->m_milestone_rx < seq + size) q->m_milestone_rx = seq + size;

            // if seq is already nacked, check for nacktimer
            if (q->m_irn_sack_.blockExists(seq, size) && Simulator::Now() < q->m_nackTimer) {
                logRecvCheck("HOLD", "IRN_OOO_DUP_NACK");
                return 4;  // don't need to send nack yet
            }
            q->m_nackTimer = Simulator::Now() + MicroSeconds(m_nack_interval);
            q->m_irn_sack_.sack(seq, size);  // set SACK
            NS_ASSERT(q->m_irn_sack_.discardUpTo(expected) ==
                      0);  // SACK blocks must be larger than expected
            cnp = true;    // XXX: out-of-order should accompany with CNP (?) TODO: Check on CX6
            logRecvCheck("NACK", "IRN_OOO_GAP");
            return 2;      // generate SACK
        }
        if (Simulator::Now() >= q->m_nackTimer || q->m_lastNACK != expected) {  // new NACK
            q->m_nackTimer = Simulator::Now() + MicroSeconds(m_nack_interval);
            q->m_lastNACK = expected;
            if (m_backto0) {
                q->ReceiverNextExpectedSeq = q->ReceiverNextExpectedSeq / m_chunk * m_chunk;
            }
            cnp = true;  // XXX: out-of-order should accompany with CNP (?) TODO: Check on CX6
            logRecvCheck("NACK", "OOO_GAP");
            return 2;
        } else {
            // skip to send NACK
            logRecvCheck("HOLD", "OOO_DUP_NACK");
            return 4;
        }
    } else {
        // Duplicate.
        if (m_irn) {
            // if (q->ReceiverNextExpectedSeq - 1 == q->m_milestone_rx) {
            // 	return 6; // This generates NACK, but actually functions as an ACK (indicates all
            // packet has been received)
            // }
            if (q->m_irn_sack_.IsEmpty()) {
                logRecvCheck("ACK", "IRN_DUPLICATE_RECOVERY_DONE");
                return 6;  // This generates NACK, but actually functions as an ACK (indicates all
                           // packet has been received)
            } else {
                // should we put nack timer here
                logRecvCheck("NACK", "IRN_DUPLICATE_RECOVERY");
                return 2;  // Still in loss recovery mode of IRN
            }
        }
        // Duplicate.
        logRecvCheck("ACK", "DUPLICATE");
        return 1;  // According to IB Spec C9-110
                   /**
                    * IB Spec C9-110
                    * A responder shall respond to all duplicate requests in PSN order;
                    * i.e. the request with the (logically) earliest PSN shall be executed first. If,
                    * while responding to a new or duplicate request, a duplicate request is received
                    * with a logically earlier PSN, the responder shall cease responding
                    * to the original request and shall begin responding to the duplicate request
                    * with the logically earlier PSN.
                    */
    }
}

int RdmaHw::ReceiverCheckSeqPsnPath(uint32_t &seq, Ptr<RdmaRxQueuePair> q, uint32_t size, bool &cnp) {
    cnp = false;
    uint32_t origSeq = seq;
    uint32_t packetSize = m_psnPathPacketSize == 0 ? m_mtu : m_psnPathPacketSize;
    uint32_t epsn = q->ReceiverNextExpectedSeq / packetSize;
    uint32_t psn = seq / packetSize;
    auto resolveSnapshot = [&](uint32_t targetPsn) -> PsnMappingSnapshot {
        PsnMappingSnapshot best;
        best.k = m_psnPathK == 0 ? 1 : m_psnPathK;
        best.o = m_psnPathO;
        best.fpsn = 0;
        uint32_t limit = std::min<uint32_t>(best.k, 32);
        best.activePathMask = limit == 32 ? 0xffffffffu : ((1u << limit) - 1u);
        for (size_t i = 0; i < q->psnMappingHistory.size(); ++i) {
            const PsnMappingSnapshot &s = q->psnMappingHistory[i];
            if (targetPsn >= s.fpsn && s.fpsn >= best.fpsn) {
                best = s;
            }
        }
        if (best.k == 0) best.k = 1;
        if (best.activePathMask == 0) {
            limit = std::min<uint32_t>(best.k, 32);
            best.activePathMask = limit == 32 ? 0xffffffffu : ((1u << limit) - 1u);
        }
        return best;
    };
    auto mappedPath = [&](uint32_t targetPsn) -> uint32_t {
        PsnMappingSnapshot snap = resolveSnapshot(targetPsn);
        return SelectPathFromActiveMask(targetPsn, snap.k, snap.o, snap.activePathMask);
    };
    auto formatBitmapWindow = [&](uint32_t startPsn, uint32_t endPsn) -> std::string {
        std::ostringstream oss;
        oss << "[";
        for (uint32_t p = startPsn; p <= endPsn; ++p) {
            if (p != startPsn) oss << ",";
            oss << p << ":" << (q->bitmap[p % BITMAP_SIZE] != 0 ? 1 : 0);
        }
        oss << "]";
        return oss.str();
    };
    auto logTrackedPsn = [&](const char *decision, const char *reason, uint32_t prevPsn,
                             bool prevReceived, bool earlierLossFound) {
        if (!(m_enableConsoleLog == 1 &&
              m_enablePsnPath && m_enablePathSwitch && m_enablePathAwareRetrans &&
              IsPsnInList(psn, m_consoleLogPsns))) {
            return;
        }
        uint32_t currentPath = mappedPath(psn);
        uint32_t startPsn = epsn <= psn ? epsn : psn;
        uint32_t firstMissingSamePath = UINT32_MAX;
        if (epsn <= psn) {
            for (uint32_t p = epsn; p < psn; ++p) {
                if (mappedPath(p) == currentPath && q->bitmap[p % BITMAP_SIZE] == 0) {
                    firstMissingSamePath = p;
                    break;
                }
            }
        }
        std::cout << "[PSN_TRACE_RECV] t=" << Simulator::Now().GetNanoSeconds()
                  << " node=" << m_node->GetId() << " seq=" << origSeq << " psn=" << psn
                  << " expectedPsn=" << epsn << " prevPsn=" << prevPsn
                  << " prevReceived=" << (prevReceived ? 1 : 0)
                  << " earlierLossFound=" << (earlierLossFound ? 1 : 0)
                  << " firstMissingSamePath="
                  << (firstMissingSamePath == UINT32_MAX
                          ? -1
                          : static_cast<int64_t>(firstMissingSamePath))
                  << " decision=" << decision << " reason=" << reason
                  << " currentPath=" << currentPath
                  << " bitmapWindow=" << formatBitmapWindow(startPsn, psn)
                  << " nackTimerNs=" << q->m_nackTimer.GetNanoSeconds()
                  << " lastNACK=" << q->m_lastNACK << std::endl;
    };
    auto logRecvCheck = [&](const char *decision, const char *reason, uint32_t prevPsn,
                            bool prevReceived) {
        if (!m_hpccTrace) return;
        std::cout << "[RECV_CHECK] t=" << Simulator::Now().GetNanoSeconds()
                  << " node=" << m_node->GetId() << " seq=" << origSeq << " psn=" << psn
                  << " expectedPsn=" << epsn << " prevPsn=" << prevPsn
                  << " prevReceived=" << (prevReceived ? 1 : 0)
                  << " decision=" << decision << " reason=" << reason
                  << " currentPath=" << mappedPath(psn) << std::endl;
    };
    uint32_t pathStride = m_psnPathK == 0 ? 1 : m_psnPathK;
    bool enablePsnPathGapNack =
        m_enablePsnPath && m_enablePathSwitch && m_enablePathAwareRetrans;
    auto prunePsnPathGapNacks = [&](uint32_t uptoPsn) {
        for (uint32_t idx = 0; idx < BITMAP_SIZE; ++idx) {
            if (q->psnPathGapPsn[idx] < uptoPsn) {
                q->psnPathGapPsn[idx] = 0;
                q->psnPathGapObservedPkts[idx] = 0;
                q->psnPathGapNackSent[idx] = false;
            }
        }
        bool resetEarliestGap = q->psnPathEarliestGapPsn < uptoPsn;
        if (resetEarliestGap) {
            q->psnPathEarliestGapPsn = UINT32_MAX;
            q->psnPathEarliestGapStart = Time(0);
            if (q->psnPathGapTimerEvent.IsRunning()) {
                q->psnPathGapTimerEvent.Cancel();
            }
        }
        std::deque<uint32_t> pending;
        while (!q->psnPathGapPendingNacks.empty()) {
            uint32_t missingPsn = q->psnPathGapPendingNacks.front();
            q->psnPathGapPendingNacks.pop_front();
            if (missingPsn >= uptoPsn) {
                pending.push_back(missingPsn);
            }
        }
        q->psnPathGapPendingNacks.swap(pending);
        if (resetEarliestGap) {
            uint32_t nextEarliest = UINT32_MAX;
            for (uint32_t idx = 0; idx < BITMAP_SIZE; ++idx) {
                uint32_t missingPsn = q->psnPathGapPsn[idx];
                if (missingPsn >= uptoPsn && !q->psnPathGapNackSent[idx] &&
                    missingPsn < nextEarliest) {
                    nextEarliest = missingPsn;
                }
            }
            if (nextEarliest != UINT32_MAX) {
                q->psnPathEarliestGapPsn = nextEarliest;
                q->psnPathEarliestGapStart = Simulator::Now();
                SchedulePsnPathGapTimer(q);
            }
        }
    };
    auto sendPsnPathGapNack = [&](uint32_t missingPsn, uint32_t currentPsn,
                                  uint32_t currentPath, uint32_t observedPkts,
                                  uint32_t triggerPkts, const char *reason) {
        uint32_t missingSeq = missingPsn * packetSize;
        uint32_t prevPsn = psn >= pathStride ? psn - pathStride : 0;
        bool prevReceived = psn >= pathStride ? (q->bitmap[prevPsn % BITMAP_SIZE] != 0) : false;
        q->m_nackTimer = Simulator::Now() + MicroSeconds(m_nack_interval);
        q->m_lastNACK = missingSeq;
        q->m_lastNackReason = reason;
        seq = missingSeq;
        cnp = false;
        logTrackedPsn("NACK", reason, prevPsn, prevReceived, false);
        logRecvCheck("NACK", reason, prevPsn, prevReceived);
        if (m_hpccTrace) {
            std::cout << "[NACK_GEN] t=" << Simulator::Now().GetNanoSeconds()
                      << " node=" << m_node->GetId() << " missingPsn=" << missingPsn
                      << " currentPsn=" << currentPsn << " pathId=" << currentPath
                      << " observedPkts=" << observedPkts
                      << " triggerPkts=" << triggerPkts
                      << " reason=" << reason << std::endl;
        }
    };

    if (psn < epsn) {
        uint32_t prevPsn = psn >= pathStride ? psn - pathStride : 0;
        bool prevReceived = psn >= pathStride ? (q->bitmap[prevPsn % BITMAP_SIZE] != 0) : false;
        q->m_ackSeqOverrideValid = true;
        q->m_ackSeqOverride = origSeq;
        logTrackedPsn("ACK", "DUPLICATE_OR_OLD", prevPsn, prevReceived, false);
        logRecvCheck("ACK", "DUPLICATE_OR_OLD", prevPsn, prevReceived);
        return 1;
    }

    if (psn == epsn) {
        q->bitmap[psn % BITMAP_SIZE] = 0;
        q->ReceiverNextExpectedSeq += size;
        uint32_t nextPsn = q->ReceiverNextExpectedSeq / packetSize;
        while (q->bitmap[nextPsn % BITMAP_SIZE] != 0) {
            q->bitmap[nextPsn % BITMAP_SIZE] = 0;
            q->ReceiverNextExpectedSeq += packetSize;
            nextPsn++;
        }
        if (enablePsnPathGapNack) {
            prunePsnPathGapNacks(nextPsn);
        }
        seq = q->ReceiverNextExpectedSeq;
        uint32_t prevPsn = psn >= pathStride ? psn - pathStride : 0;
        bool prevReceived = psn >= pathStride ? (q->bitmap[prevPsn % BITMAP_SIZE] != 0) : false;
        logTrackedPsn("ACCEPT", "IN_ORDER", prevPsn, prevReceived, false);
        logRecvCheck("ACCEPT", "IN_ORDER", prevPsn, prevReceived);
        return size == 0 ? 6 : 1;
    }

        if (psn >= epsn + BITMAP_SIZE) {
        seq = q->ReceiverNextExpectedSeq;
        cnp = false;
        q->m_lastNackReason = "BITMAP_WINDOW_OVERFLOW";
        uint32_t prevPsn = psn >= pathStride ? psn - pathStride : 0;
        bool prevReceived = psn >= pathStride ? (q->bitmap[prevPsn % BITMAP_SIZE] != 0) : false;
        logTrackedPsn("NACK", "WINDOW_OVERFLOW", prevPsn, prevReceived, false);
        logRecvCheck("NACK", "WINDOW_OVERFLOW", prevPsn, prevReceived);
        return 2;
    }

    q->bitmap[psn % BITMAP_SIZE] = 1;
    uint32_t currentPath = mappedPath(psn);
    uint32_t triggerPkts = pathStride + 1;
    if (enablePsnPathGapNack) {
        uint32_t earliestMissingPsn = UINT32_MAX;
        for (uint32_t missingPsn = epsn; missingPsn < psn; ++missingPsn) {
            if (q->bitmap[missingPsn % BITMAP_SIZE] != 0) {
                continue;
            }
            if (earliestMissingPsn == UINT32_MAX) {
                earliestMissingPsn = missingPsn;
            }
            uint32_t slot = missingPsn % BITMAP_SIZE;
            if (q->psnPathGapPsn[slot] != missingPsn) {
                q->psnPathGapPsn[slot] = missingPsn;
                q->psnPathGapObservedPkts[slot] = 0;
                q->psnPathGapNackSent[slot] = false;
            }
            if (!q->psnPathGapNackSent[slot]) {
                q->psnPathGapObservedPkts[slot]++;
                if (q->psnPathGapObservedPkts[slot] >= triggerPkts) {
                    q->psnPathGapNackSent[slot] = true;
                    bool alreadyPending = false;
                    for (size_t idx = 0; idx < q->psnPathGapPendingNacks.size(); ++idx) {
                        if (q->psnPathGapPendingNacks[idx] == missingPsn) {
                            alreadyPending = true;
                            break;
                        }
                    }
                    if (!alreadyPending) {
                        q->psnPathGapPendingNacks.push_back(missingPsn);
                    }
                }
            }
        }
        if (earliestMissingPsn != UINT32_MAX &&
            q->psnPathEarliestGapPsn != earliestMissingPsn) {
            q->psnPathEarliestGapPsn = earliestMissingPsn;
            q->psnPathEarliestGapStart = Simulator::Now();
            SchedulePsnPathGapTimer(q);
        }
    }
    bool earlierLossFound = false;
    for (uint32_t i = epsn; i < psn; ++i) {
        if (mappedPath(i) != currentPath) {
            continue;
        }
        if (q->bitmap[i % BITMAP_SIZE] == 0) {
            earlierLossFound = true;
            uint32_t missingSeq = i * packetSize;
            uint32_t prevPsn = psn >= pathStride ? psn - pathStride : 0;
            bool prevReceived = psn >= pathStride ? (q->bitmap[prevPsn % BITMAP_SIZE] != 0) : false;
            if (Simulator::Now() >= q->m_nackTimer || q->m_lastNACK != missingSeq) {
                q->m_nackTimer = Simulator::Now() + MicroSeconds(m_nack_interval);
                q->m_lastNACK = missingSeq;
                q->m_lastNackReason = "SAME_PATH_GAP";
                seq = missingSeq;
                cnp = false;
                logTrackedPsn("NACK", "SAME_PATH_GAP", prevPsn, prevReceived, earlierLossFound);
                logRecvCheck("NACK", "SAME_PATH_GAP", prevPsn, prevReceived);
                if (m_hpccTrace) {
                    std::cout << "[NACK_GEN] t=" << Simulator::Now().GetNanoSeconds()
                              << " node=" << m_node->GetId() << " missingPsn=" << i
                              << " currentPsn=" << psn << " pathId=" << currentPath
                              << " reason=SAME_PATH_GAP" << std::endl;
                }
                return 2;
            }
            logTrackedPsn("HOLD", "SAME_PATH_GAP_DUP_NACK", prevPsn, prevReceived,
                          earlierLossFound);
            logRecvCheck("HOLD", "SAME_PATH_GAP_DUP_NACK", prevPsn, prevReceived);
            return 4;
        }
    }

    if (enablePsnPathGapNack) {
        if (m_psnPathGapTimeout.GetNanoSeconds() > 0 &&
            q->psnPathEarliestGapPsn != UINT32_MAX) {
            uint32_t missingPsn = q->psnPathEarliestGapPsn;
            uint32_t slot = missingPsn % BITMAP_SIZE;
            if (q->psnPathGapPsn[slot] == missingPsn &&
                !q->psnPathGapNackSent[slot] &&
                Simulator::Now() >= q->psnPathEarliestGapStart + m_psnPathGapTimeout) {
                q->psnPathGapNackSent[slot] = true;
                sendPsnPathGapNack(missingPsn, psn, currentPath,
                                   q->psnPathGapObservedPkts[slot], triggerPkts,
                                   "PSN_PATH_TIMEOUT_GAP");
                return 2;
            }
        }
        if (!q->psnPathGapPendingNacks.empty()) {
            uint32_t missingPsn = q->psnPathGapPendingNacks.front();
            q->psnPathGapPendingNacks.pop_front();
            uint32_t slot = missingPsn % BITMAP_SIZE;
            sendPsnPathGapNack(missingPsn, psn, currentPath,
                               q->psnPathGapObservedPkts[slot], triggerPkts,
                               "PSN_PATH_K_GAP");
            return 2;
        }
    }

    uint32_t prevPsn = psn >= pathStride ? psn - pathStride : 0;
    bool prevReceived = psn >= pathStride ? (q->bitmap[prevPsn % BITMAP_SIZE] != 0) : false;
    if (origSeq == q->lastoutseq && q->outcount >= 50) {
        q->outcount = 0;
        logTrackedPsn("ACCEPT", "CROSS_PATH_OOO_THROTTLED", prevPsn, prevReceived, false);
        logRecvCheck("ACCEPT", "CROSS_PATH_OOO_THROTTLED", prevPsn, prevReceived);
        return 5;
    }
    q->outcount++;
    q->lastoutseq = origSeq;
    logTrackedPsn("ACCEPT", "CROSS_PATH_OOO", prevPsn, prevReceived, false);
    logRecvCheck("ACCEPT", "CROSS_PATH_OOO", prevPsn, prevReceived);
    return 5;
}

void RdmaHw::SchedulePsnPathGapTimer(Ptr<RdmaRxQueuePair> q) {
    if (!(m_enablePsnPath && m_enablePathSwitch && m_enablePathAwareRetrans)) return;
    if (m_psnPathGapTimeout.GetNanoSeconds() <= 0) return;
    if (q == NULL || q->psnPathEarliestGapPsn == UINT32_MAX) return;

    uint32_t missingPsn = q->psnPathEarliestGapPsn;
    uint32_t slot = missingPsn % BITMAP_SIZE;
    if (q->psnPathGapPsn[slot] != missingPsn || q->psnPathGapNackSent[slot]) return;

    Time fireAt = q->psnPathEarliestGapStart + m_psnPathGapTimeout;
    Time delay = fireAt > Simulator::Now() ? fireAt - Simulator::Now() : NanoSeconds(0);
    if (q->psnPathGapTimerEvent.IsRunning()) {
        q->psnPathGapTimerEvent.Cancel();
    }
    q->psnPathGapTimerEvent =
        Simulator::Schedule(delay, &RdmaHw::HandlePsnPathGapTimeout, this, q, missingPsn);
}

void RdmaHw::UpdateBitmapExpectedTimeout(Ptr<RdmaRxQueuePair> q,
                                         const BitmapRetransFeedback &feedback) {
    if (!m_enableBitmapRetrans || q == NULL || m_bitmapRetransTimeout.GetNanoSeconds() <= 0) {
        return;
    }

    if (feedback.isNack) {
        q->bitmapExpectedTimeoutSeq = feedback.ackSeq;
        q->bitmapExpectedNackSent = true;
        if (q->bitmapExpectedTimerEvent.IsRunning()) {
            q->bitmapExpectedTimerEvent.Cancel();
        }
        return;
    }

    if (!feedback.hasGap) {
        q->bitmapExpectedTimeoutSeq = 0xffffffffu;
        q->bitmapExpectedNackSent = false;
        if (q->bitmapExpectedTimerEvent.IsRunning()) {
            q->bitmapExpectedTimerEvent.Cancel();
        }
        return;
    }

    if (q->bitmapExpectedTimeoutSeq != feedback.ackSeq) {
        q->bitmapExpectedTimeoutSeq = feedback.ackSeq;
        q->bitmapExpectedNackSent = false;
        if (q->bitmapExpectedTimerEvent.IsRunning()) {
            q->bitmapExpectedTimerEvent.Cancel();
        }
    }

    if (!q->bitmapExpectedNackSent && !q->bitmapExpectedTimerEvent.IsRunning()) {
        q->bitmapExpectedTimerEvent =
            Simulator::Schedule(m_bitmapRetransTimeout, &RdmaHw::HandleBitmapExpectedTimeout,
                                this, q, q->bitmapExpectedTimeoutSeq);
    }
}

void RdmaHw::HandleBitmapExpectedTimeout(Ptr<RdmaRxQueuePair> q, uint32_t expectedSeq) {
    if (!m_enableBitmapRetrans || q == NULL || m_bitmapRetransTimeout.GetNanoSeconds() <= 0) {
        return;
    }
    if (q->bitmapExpectedTimeoutSeq != expectedSeq ||
        q->ReceiverNextExpectedSeq != expectedSeq ||
        q->bitmapExpectedNackSent) {
        return;
    }

    q->bitmapExpectedNackSent = true;
    SendBitmapExpectedTimeoutNack(q, expectedSeq);
}

void RdmaHw::SendBitmapExpectedTimeoutNack(Ptr<RdmaRxQueuePair> q, uint32_t expectedSeq) {
    uint32_t packetSize = m_mtu == 0 ? 1 : m_mtu;
    q->m_nackTimer = Simulator::Now() + MicroSeconds(m_nack_interval);
    q->m_lastNACK = expectedSeq;
    q->m_lastNackReason = "BITMAP_TIMEOUT_GAP";

    qbbHeader seqh;
    seqh.SetSeq(expectedSeq);
    seqh.SetPG(q->m_ecn_source.qIndex);
    seqh.SetSport(q->sport);
    seqh.SetDport(q->dport);

    Ptr<Packet> newp =
        Create<Packet>(std::max(60 - 14 - 20 - (int)seqh.GetSerializedSize(), 0));
    newp->AddHeader(seqh);

    Ipv4Header head;
    head.SetDestination(Ipv4Address(q->dip));
    head.SetSource(Ipv4Address(q->sip));
    head.SetProtocol(0xFD);
    head.SetTtl(64);
    head.SetPayloadSize(newp->GetSize());
    head.SetIdentification(q->m_ipid++);
    newp->AddHeader(head);
    AddHeader(newp, 0x800);

    LogReceiverNack(m_node->GetId(), expectedSeq, q->ReceiverNextExpectedSeq, packetSize,
                    "BITMAP_TIMEOUT_GAP");
    if (m_hpccTrace) {
        std::cout << "[BITMAP_TIMEOUT_NACK] t=" << Simulator::Now().GetNanoSeconds()
                  << " node=" << m_node->GetId()
                  << " expectedSeq=" << expectedSeq
                  << " expectedPsn=" << (packetSize == 0 ? 0 : expectedSeq / packetSize)
                  << " timeoutNs=" << m_bitmapRetransTimeout.GetNanoSeconds()
                  << std::endl;
    }

    uint32_t nic_idx = GetNicIdxOfRxQp(q);
    m_nic[nic_idx].dev->RdmaEnqueueHighPrioQ(newp);
    m_nic[nic_idx].dev->TriggerTransmit();
}

void RdmaHw::HandlePsnPathGapTimeout(Ptr<RdmaRxQueuePair> q, uint32_t missingPsn) {
    if (!(m_enablePsnPath && m_enablePathSwitch && m_enablePathAwareRetrans)) return;
    if (m_psnPathGapTimeout.GetNanoSeconds() <= 0 || q == NULL) return;

    uint32_t packetSize = m_psnPathPacketSize == 0 ? m_mtu : m_psnPathPacketSize;
    uint32_t expectedPsn = q->ReceiverNextExpectedSeq / packetSize;
    uint32_t slot = missingPsn % BITMAP_SIZE;
    bool stillMissing = missingPsn >= expectedPsn &&
                        q->psnPathGapPsn[slot] == missingPsn &&
                        q->bitmap[slot] == 0 &&
                        !q->psnPathGapNackSent[slot];
    if (!stillMissing) {
        if (q->psnPathEarliestGapPsn != UINT32_MAX) {
            SchedulePsnPathGapTimer(q);
        }
        return;
    }

    q->psnPathGapNackSent[slot] = true;
    q->m_lastNackReason = "PSN_PATH_TIMEOUT_GAP";
    SendPsnPathGapTimeoutNack(q, missingPsn);
}

void RdmaHw::SendPsnPathGapTimeoutNack(Ptr<RdmaRxQueuePair> q, uint32_t missingPsn) {
    uint32_t packetSize = m_psnPathPacketSize == 0 ? m_mtu : m_psnPathPacketSize;
    uint32_t missingSeq = missingPsn * packetSize;
    q->m_nackTimer = Simulator::Now() + MicroSeconds(m_nack_interval);
    q->m_lastNACK = missingSeq;

    qbbHeader seqh;
    seqh.SetSeq(q->ReceiverNextExpectedSeq);
    seqh.SetPG(q->m_ecn_source.qIndex);
    seqh.SetSport(q->sport);
    seqh.SetDport(q->dport);
    seqh.SetIrnNack(missingSeq);
    seqh.SetIrnNackSize(packetSize);

    Ptr<Packet> newp =
        Create<Packet>(std::max(60 - 14 - 20 - (int)seqh.GetSerializedSize(), 0));
    newp->AddHeader(seqh);

    Ipv4Header head;
    head.SetDestination(Ipv4Address(q->dip));
    head.SetSource(Ipv4Address(q->sip));
    head.SetProtocol(0xFD);
    head.SetTtl(64);
    head.SetPayloadSize(newp->GetSize());
    head.SetIdentification(q->m_ipid++);
    newp->AddHeader(head);
    AddHeader(newp, 0x800);

    LogReceiverNack(m_node->GetId(), missingSeq, q->ReceiverNextExpectedSeq, packetSize,
                    "PSN_PATH_TIMEOUT_GAP");
    if (m_hpccTrace) {
        std::cout << "[NACK_GEN] t=" << Simulator::Now().GetNanoSeconds()
                  << " node=" << m_node->GetId() << " missingPsn=" << missingPsn
                  << " currentPsn=" << (packetSize == 0 ? 0 : q->ReceiverNextExpectedSeq / packetSize)
                  << " observedPkts=" << q->psnPathGapObservedPkts[missingPsn % BITMAP_SIZE]
                  << " triggerPkts=" << ((m_psnPathK == 0 ? 1 : m_psnPathK) + 1)
                  << " reason=PSN_PATH_TIMEOUT_GAP_TIMER" << std::endl;
    }

    uint32_t nic_idx = GetNicIdxOfRxQp(q);
    m_nic[nic_idx].dev->RdmaEnqueueHighPrioQ(newp);
    m_nic[nic_idx].dev->TriggerTransmit();
}

void RdmaHw::AddHeader(Ptr<Packet> p, uint16_t protocolNumber) {
    PppHeader ppp;
    ppp.SetProtocol(EtherToPpp(protocolNumber));
    p->AddHeader(ppp);
}

uint16_t RdmaHw::EtherToPpp(uint16_t proto) {
    switch (proto) {
        case 0x0800:
            return 0x0021;  // IPv4
        case 0x86DD:
            return 0x0057;  // IPv6
        default:
            NS_ASSERT_MSG(false, "PPP Protocol number not defined!");
    }
    return 0;
}

void RdmaHw::RecoverQueue(Ptr<RdmaQueuePair> qp) { qp->snd_nxt = qp->snd_una; }

void RdmaHw::QpComplete(Ptr<RdmaQueuePair> qp) {
    NS_ASSERT(!m_qpCompleteCallback.IsNull());
    if (m_cc_mode == 1) {
        Simulator::Cancel(qp->mlx.m_eventUpdateAlpha);
        Simulator::Cancel(qp->mlx.m_eventDecreaseRate);
        Simulator::Cancel(qp->mlx.m_rpTimer);
    }
    if (qp->psnPath.evalEvent.IsRunning()) qp->psnPath.evalEvent.Cancel();
    if (qp->m_retransmit.IsRunning()) qp->m_retransmit.Cancel();

    // This callback will log info. It also calls deletetion the rxQp on the receiver
    m_qpCompleteCallback(qp);
    // delete TxQueuePair
    DeleteQueuePair(qp);
}

void RdmaHw::SetLinkDown(Ptr<QbbNetDevice> dev) {
    printf("RdmaHw: node:%u a link down\n", m_node->GetId());
}

void RdmaHw::SendRxControlPacket(Ptr<QbbNetDevice> dev, Ptr<Packet> p,
                                 RxControlDelayKind delayKind) {
    Time delay = NanoSeconds(0);
    if (delayKind == RxControlDelayKind::Falcon) {
        delay = m_falconRxSendDelay;
    } else if (delayKind == RxControlDelayKind::Ornic) {
        delay = m_ornicRxSendDelay;
    }
    if (!delay.IsZero()) {
        Simulator::Schedule(delay, &RdmaHw::SendRxControlPacketNow, this, dev, p);
        return;
    }
    SendRxControlPacketNow(dev, p);
}

void RdmaHw::SendRxControlPacketNow(Ptr<QbbNetDevice> dev, Ptr<Packet> p) {
    dev->RdmaEnqueueHighPrioQ(p);
    dev->TriggerTransmit();
}

void RdmaHw::EnqueueFalconBitmapRetrans(Ptr<RdmaQueuePair> qp, uint32_t seq) {
    const uint32_t packetSize = qp->psnPath.packetSize == 0 ? m_mtu : qp->psnPath.packetSize;
    const uint32_t startPsn = seq / packetSize;
    for (size_t idx = 0; idx < qp->psnPath.pendingRetrans.size(); ++idx) {
        const PsnRetransRequest &req = qp->psnPath.pendingRetrans[idx];
        if (req.startPsn == startPsn && req.endPsn == startPsn) {
            return;
        }
    }
    PsnRetransRequest req;
    req.startPsn = startPsn;
    req.endPsn = startPsn;
    req.epoch = qp->psnPath.mappingEpoch;
    qp->psnPath.pendingRetrans.push_back(req);
}

void RdmaHw::ScheduleDeferredFalconBitmapRetrans(Ptr<RdmaQueuePair> qp, uint32_t seq,
                                                 Time delay) {
    if (qp == 0 || seq < qp->snd_una || qp->IsFinished()) {
        return;
    }
    if (!qp->m_falconDeferredRetrans.insert(seq).second) {
        return;
    }
    Simulator::Schedule(delay, &RdmaHw::EnqueueDeferredFalconBitmapRetrans, this, qp, seq);
}

void RdmaHw::EnqueueDeferredFalconBitmapRetrans(Ptr<RdmaQueuePair> qp, uint32_t seq) {
    if (qp == 0) {
        return;
    }
    qp->m_falconDeferredRetrans.erase(seq);
    if (seq < qp->snd_una || qp->IsFinished()) {
        return;
    }
    auto key = GetQpKey(qp->dip.Get(), qp->sport, qp->dport, qp->m_pg);
    if (!m_falcon.MarkRetransPending(key, seq)) {
        return;
    }
    EnqueueFalconBitmapRetrans(qp, seq);
    uint32_t nic_idx = GetNicIdxOfQp(qp);
    if (nic_idx < m_nic.size() && m_nic[nic_idx].dev != 0) {
        m_nic[nic_idx].dev->TriggerTransmit();
    }
}

void RdmaHw::AddTableEntry(Ipv4Address &dstAddr, uint32_t intf_idx) {
    uint32_t dip = dstAddr.Get();
    m_rtTable[dip].push_back(intf_idx);
}

void RdmaHw::ClearTable() { m_rtTable.clear(); }

void RdmaHw::RedistributeQp() {
    // clear old qpGrp
    for (uint32_t i = 0; i < m_nic.size(); i++) {
        if (m_nic[i].dev == NULL) continue;
        m_nic[i].qpGrp->Clear();
    }

    // redistribute qp
    for (auto &it : m_qpMap) {
        Ptr<RdmaQueuePair> qp = it.second;
        uint32_t nic_idx = GetNicIdxOfQp(qp);
        m_nic[nic_idx].qpGrp->AddQp(qp);
        // Notify Nic
        m_nic[nic_idx].dev->ReassignedQp(qp);
    }
}

Ptr<Packet> RdmaHw::GetNxtPacket(Ptr<RdmaQueuePair> qp) {
    uint32_t seq = (uint32_t)qp->snd_nxt;
    bool retransFromQueue = false;
    bool retransFromOrnic = false;
    if (qp->ornic.enabled && !qp->ornic.pendingRetrans.empty()) {
        seq = qp->ornic.pendingRetrans.front();
        retransFromQueue = true;
        retransFromOrnic = true;
    } else if (qp->psnPath.pathAwareRetransEnabled &&
        !qp->psnPath.pendingRetrans.empty()) {
        PsnRetransRequest &req = qp->psnPath.pendingRetrans.front();
        seq = req.startPsn * qp->psnPath.packetSize;
        retransFromQueue = true;
    }
    uint32_t payload_size = 0;
    if (retransFromQueue) {
        uint64_t bytes_left_from_seq = qp->m_size > seq ? qp->m_size - seq : 0;
        payload_size = static_cast<uint32_t>(std::min<uint64_t>(m_mtu, bytes_left_from_seq));
    } else {
        payload_size = qp->GetBytesLeft();
        if (m_mtu < payload_size) {  // possibly last packet
            payload_size = m_mtu;
        }
    }
    if (payload_size == 0) {
        if (retransFromOrnic && !qp->ornic.pendingRetrans.empty()) {
            qp->ornic.pendingRetrans.pop_front();
        } else if (retransFromQueue && !qp->psnPath.pendingRetrans.empty()) {
            qp->psnPath.pendingRetrans.pop_front();
        }
        return Create<Packet>(0);
    }
    bool isRetrans = retransFromQueue || (seq < qp->stat.txHighestSeqSent);
    bool proceed_snd_nxt = true;
    qp->stat.txTotalPkts += 1;
    qp->stat.txTotalBytes += payload_size;
    if (isRetrans) {
        qp->stat.txRetransPkts += 1;
        LogSenderRetrans(m_node->GetId(), qp, seq, qp->psnPath.packetSize);
    }
    if (!m_txDataPacketCallback.IsNull()) {
        m_txDataPacketCallback(qp->sip, qp->dip, payload_size, isRetrans);
    }

    Ptr<Packet> p = Create<Packet>(payload_size);
    // add SeqTsHeader
    SeqTsHeader seqTs;
    seqTs.SetSeq(seq);
    seqTs.SetPG(qp->m_pg);
    p->AddHeader(seqTs);
    // add udp header
    UdpHeader udpHeader;
    udpHeader.SetDestinationPort(qp->dport);
    uint16_t sourcePort = qp->sport;
    uint32_t selectedPathId = 0;
    uint8_t probeFlag = 0;
    if (qp->psnPath.pathSelectionEnabled) {
        uint32_t psn = PsnPath::GetPsnFromSeq(seq, qp->psnPath.packetSize);
        selectedPathId = SelectPathFromStats(psn, qp->psnPath.k, qp->psnPath.o,
                                             qp->psnPath.pathStats);
        if (!retransFromQueue && qp->psnPath.pathSwitchEnabled && !qp->psnPath.pathStats.empty()) {
            if (qp->psnPath.probeCountdown == 0) {
                for (uint32_t i = 0; i < qp->psnPath.pathStats.size(); ++i) {
                    if (!qp->psnPath.pathStats[i].active && qp->psnPath.pathStats[i].probing) {
                        selectedPathId = i;
                        probeFlag = 1;
                        break;
                    }
                }
                qp->psnPath.probeCountdown = qp->psnPath.probeEveryN;
            } else {
                qp->psnPath.probeCountdown--;
            }
        }
        sourcePort = static_cast<uint16_t>(qp->psnPath.basePort + selectedPathId);
        if (m_hpccTrace) {
            std::cout << "[SEND_SELECT] t=" << Simulator::Now().GetNanoSeconds()
                      << " qp=" << qp->m_flow_id << " seq=" << seq << " psn=" << psn
                      << " isRetrans=" << (isRetrans ? 1 : 0)
                      << " retransMode="
                      << (retransFromQueue ? "PSN_QUEUE"
                                           : (isRetrans ? "GO_BACK_OR_TIMEOUT" : "NEW"))
                      << " pathId=" << selectedPathId << " srcPort=" << sourcePort
                      << " epoch=" << qp->psnPath.mappingEpoch << " k=" << qp->psnPath.k
                      << " offset=" << qp->psnPath.o
                      << " activePaths="
                      << FormatPathIndices(CollectActivePathIndices(qp->psnPath.pathStats))
                      << " probe=" << static_cast<uint32_t>(probeFlag)
                      << " pendingRetrans=" << qp->psnPath.pendingRetrans.size() << std::endl;
        }
    }
    udpHeader.SetSourcePort(sourcePort);
    p->AddHeader(udpHeader);
    // add ipv4 header
    Ipv4Header ipHeader;
    ipHeader.SetSource(qp->sip);
    ipHeader.SetDestination(qp->dip);
    ipHeader.SetProtocol(0x11);
    ipHeader.SetPayloadSize(p->GetSize());
    ipHeader.SetTtl(64);
    ipHeader.SetTos(0);
    ipHeader.SetIdentification(qp->m_ipid);
    p->AddHeader(ipHeader);
    // add ppp header
    PppHeader ppp;
    ppp.SetProtocol(0x0021);  // EtherToPpp(0x800), see point-to-point-net-device.cc
    p->AddHeader(ppp);

    // attach Stat Tag
    uint8_t packet_pos = UINT8_MAX;
    {
        if (qp->psnPath.pathSelectionEnabled) {
            PsnPathTag psnTag;
            psnTag.SetEpoch(qp->psnPath.mappingEpoch);
            psnTag.SetK(qp->psnPath.k);
            psnTag.SetO(qp->psnPath.o);
            psnTag.SetFpsn(qp->psnPath.fpsn);
            psnTag.SetActivePathMask(BuildActivePathMask(qp->psnPath.pathStats));
            psnTag.SetPathId(selectedPathId);
            psnTag.SetProbe(probeFlag);
            psnTag.SetFlowSport(qp->sport);
            p->AddPacketTag(psnTag);
        }
        FlowIDNUMTag fint;
        if (!p->PeekPacketTag(fint)) {
            fint.SetId(qp->m_flow_id);
            fint.SetFlowSize(qp->m_size);
            p->AddPacketTag(fint);
        }
        FlowStatTag fst;
        uint64_t size = qp->m_size;
        if (!p->PeekPacketTag(fst)) {
            if (size < m_mtu && qp->snd_nxt + payload_size >= qp->m_size) {
                fst.SetType(FlowStatTag::FLOW_START_AND_END);
            } else if (qp->snd_nxt + payload_size >= qp->m_size) {
                fst.SetType(FlowStatTag::FLOW_END);
            } else if (qp->snd_nxt == 0) {
                fst.SetType(FlowStatTag::FLOW_START);
            } else {
                fst.SetType(FlowStatTag::FLOW_NOTEND);
            }
            packet_pos = fst.GetType();
            fst.setInitiatedTime(Simulator::Now().GetSeconds());
            p->AddPacketTag(fst);
        }
    }

    if (qp->irn.m_enabled) {
        if (qp->irn.m_max_seq < seq) qp->irn.m_max_seq = seq;
    }

    // // update state
    if (retransFromOrnic) {
        if (m_hpccTrace) {
            std::cout << "[ORNIC_RETX_SEND] t=" << Simulator::Now().GetNanoSeconds()
                      << " qp=" << qp->m_flow_id << " seq=" << seq
                      << " psn=" << (qp->ornic.packetSize == 0 ? 0 : seq / qp->ornic.packetSize)
                      << std::endl;
        }
        if (!qp->ornic.pendingRetrans.empty()) {
            qp->ornic.pendingRetrans.pop_front();
        }
        auto ornicTxKey = GetQpKey(qp->dip.Get(), qp->sport, qp->dport, qp->m_pg);
        m_ornicRetrans.ClearRetransMarker(ornicTxKey, seq);
        proceed_snd_nxt = false;
        qp->irn.m_recovery = true;
    } else if (retransFromQueue) {
        if (m_hpccTrace) {
            std::cout << "[RETX_SEND] t=" << Simulator::Now().GetNanoSeconds()
                      << " qp=" << qp->m_flow_id << " seq=" << seq
                      << " psn=" << (qp->psnPath.packetSize == 0 ? 0 : seq / qp->psnPath.packetSize)
                      << " pathId=" << selectedPathId << " srcPort=" << sourcePort
                      << " epoch=" << qp->psnPath.mappingEpoch << std::endl;
        }
        if (!qp->psnPath.pendingRetrans.empty()) {
            PsnRetransRequest &req = qp->psnPath.pendingRetrans.front();
            if (req.startPsn < req.endPsn) {
                req.startPsn++;
            } else {
                qp->psnPath.pendingRetrans.pop_front();
            }
        }
        proceed_snd_nxt = false;
        qp->irn.m_recovery = true;
    }
    if (proceed_snd_nxt) qp->snd_nxt += payload_size;
    uint64_t sentEndSeq = static_cast<uint64_t>(seq) + payload_size;
    if (sentEndSeq > qp->stat.txHighestSeqSent) {
        qp->stat.txHighestSeqSent = sentEndSeq;
    }
    {
        uint32_t packetSize = qp->psnPath.packetSize == 0 ? m_mtu : qp->psnPath.packetSize;
        uint32_t psn = packetSize == 0 ? 0 : seq / packetSize;
        bool traceSelectedPsn =
            m_enableConsoleLog == 1 &&
            ((m_enablePsnPath && m_enablePathSwitch && m_enablePathAwareRetrans) ||
             m_enableBitmapRetrans || m_enableFalcon || m_enableOrnic) &&
            IsPsnInList(psn, m_consoleLogPsns);
        if (traceSelectedPsn || m_enableConsoleLog >= 2) {
            uint64_t win = qp->GetWin();
            uint64_t onTheFlyBytes = qp->GetOnTheFly();
            uint64_t winLeftBytes = win > onTheFlyBytes ? win - onTheFlyBytes : 0;
            uint64_t winLeftPkts =
                packetSize == 0 ? 0 : (winLeftBytes + packetSize - 1) / packetSize;
            uint32_t nextSeq = static_cast<uint32_t>(qp->snd_nxt);
            const char *nextMode = "NEW";
            if (qp->psnPath.pathAwareRetransEnabled &&
                !qp->psnPath.pendingRetrans.empty()) {
                nextSeq = qp->psnPath.pendingRetrans.front().startPsn * packetSize;
                nextMode = "PSN_QUEUE";
            } else if (nextSeq < qp->stat.txHighestSeqSent) {
                nextMode = "RETRANS";
            } else if (nextSeq >= qp->m_size) {
                nextMode = "DONE";
            }
            uint32_t nextPsn = packetSize == 0 ? 0 : nextSeq / packetSize;
            uint64_t sndNxtPsn = packetSize == 0 ? 0 : qp->snd_nxt / packetSize;
            uint64_t sndUnaPsn = packetSize == 0 ? 0 : qp->snd_una / packetSize;
            uint64_t winPkts = packetSize == 0 ? 0 : win / packetSize;
            uint64_t onTheFlyPkts = packetSize == 0 ? 0 : onTheFlyBytes / packetSize;
            if (traceSelectedPsn) {
                std::cout << "[PSN_TRACE_SEND] t=" << Simulator::Now().GetNanoSeconds()
                          << " node=" << m_node->GetId()
                          << " qp=" << qp->m_flow_id
                          << " seq=" << seq
                          << " psn=" << psn
                          << " snd_nxt_psn=" << sndNxtPsn
                          << " snd_una_psn=" << sndUnaPsn
                          << " nextSeq=" << nextSeq
                          << " nextPsn=" << nextPsn
                          << " nextMode=" << nextMode
                          << " packetSize=" << packetSize
                          << " winPkts=" << winPkts
                          << " win=" << win
                          << " onTheFlyPkts=" << onTheFlyPkts
                          << " onTheFlyBytes=" << onTheFlyBytes
                          << " winLeftBytes=" << winLeftBytes
                          << " winLeftPkts=" << winLeftPkts
                          << " payloadSize=" << payload_size
                          << " isRetrans=" << (isRetrans ? 1 : 0)
                          << " retransMode="
                          << (retransFromQueue ? "PSN_QUEUE"
                                               : (isRetrans ? "GO_BACK_OR_TIMEOUT" : "NEW"))
                          << " pathId=" << selectedPathId
                          << " srcPort=" << sourcePort
                          << " pendingRetrans=" << qp->psnPath.pendingRetrans.size()
                          << std::endl;
            }
            if (m_enableConsoleLog >= 2) {
                LogSenderPacket(m_node->GetId(), qp, seq, psn, nextSeq, nextPsn, nextMode,
                                packetSize, win, onTheFlyBytes, winLeftBytes, winLeftPkts,
                                payload_size, isRetrans, retransFromQueue, selectedPathId,
                                sourcePort);
            }
        }
    }
    if (m_enableBitmapRetrans) {
        auto bitmapTxKey = GetQpKey(qp->dip.Get(), qp->sport, qp->dport, qp->m_pg);
        m_bitmapRetrans.OnPacketSent(bitmapTxKey, seq, payload_size, isRetrans);
        if (m_hpccTrace) {
            std::cout << "[BITMAP_TX] t=" << Simulator::Now().GetNanoSeconds()
                      << " qp=" << qp->m_flow_id << " seq=" << seq
                      << " isRetrans=" << (isRetrans ? 1 : 0)
                      << " snd_nxt=" << qp->snd_nxt << " snd_una=" << qp->snd_una
                      << std::endl;
        }
    }
    if (m_enableFalcon) {
        auto falconTxKey = GetQpKey(qp->dip.Get(), qp->sport, qp->dport, qp->m_pg);
        m_falcon.OnPacketSent(falconTxKey, seq, payload_size, Simulator::Now(), isRetrans);
    }
    if (m_enableOrnic) {
        auto ornicTxKey = GetQpKey(qp->dip.Get(), qp->sport, qp->dport, qp->m_pg);
        m_ornicRetrans.OnPacketSent(ornicTxKey, seq, payload_size, isRetrans);
        if (m_hpccTrace) {
            std::cout << "[ORNIC_TX] t=" << Simulator::Now().GetNanoSeconds()
                      << " qp=" << qp->m_flow_id << " seq=" << seq
                      << " isRetrans=" << (isRetrans ? 1 : 0)
                      << " pendingRetrans=" << qp->ornic.pendingRetrans.size()
                      << std::endl;
        }
    }


    qp->m_ipid++;

    // return
    return p;
}

void RdmaHw::PktSent(Ptr<RdmaQueuePair> qp, Ptr<Packet> pkt, Time interframeGap) {
    qp->lastPktSize = pkt->GetSize();
    UpdateNextAvail(qp, interframeGap, pkt->GetSize());

    if (pkt) {
        CustomHeader ch(CustomHeader::L2_Header | CustomHeader::L3_Header |
                        CustomHeader::L4_Header);
        pkt->PeekHeader(ch);
#if (SLB_DEBUG == true)
        std::cout << "[RdmaHw::PktSent] Node(" << m_node->GetId() << ")," << PARSE_FIVE_TUPLE(ch)
                  << "l3Prot:" << ch.l3Prot << ",at" << Simulator::Now() << std::endl;
#endif
        RdmaHw::nAllPkts += 1;
        if (ch.l3Prot == 0x11) {  // UDP
            // Update Timer
            if (qp->m_retransmit.IsRunning()) qp->m_retransmit.Cancel();
            qp->m_retransmit = Simulator::Schedule(qp->GetRto(m_mtu), &RdmaHw::HandleTimeout, this,
                                                   qp, qp->GetRto(m_mtu));
        } else if (ch.l3Prot == 0xFC || ch.l3Prot == 0xFD || ch.l3Prot == 0xFF) {  // ACK, NACK, CNP
        } else if (ch.l3Prot == 0xFE) {                                            // PFC
        }
    }
}

void RdmaHw::HandleTimeout(Ptr<RdmaQueuePair> qp, Time rto) {
    // Assume Outstanding Packets are lost
    // std::cerr << "Timeout on qp=" << qp << std::endl;
    if (qp->IsFinished()) {
        return;
    }

    uint32_t packetSize = qp->psnPath.packetSize == 0 ? m_mtu : qp->psnPath.packetSize;
    uint64_t sndNxtPsn = packetSize == 0 ? 0 : qp->snd_nxt / packetSize;
    uint64_t sndUnaPsn = packetSize == 0 ? 0 : qp->snd_una / packetSize;
    std::cout << "[TIMEOUT] t=" << Simulator::Now().GetNanoSeconds()
              << " qp=" << qp->m_flow_id << " rto_ns=" << rto.GetNanoSeconds()
              << " snd_nxt=" << sndNxtPsn << " snd_una=" << sndUnaPsn
              << " snd_nxt_bytes=" << qp->snd_nxt
              << " snd_una_bytes=" << qp->snd_una
              << std::endl;

    uint32_t nic_idx = GetNicIdxOfQp(qp);
    Ptr<QbbNetDevice> dev = m_nic[nic_idx].dev;

    // IRN: disable timeouts when PFC is enabled to prevent spurious retransmissions
    if (qp->irn.m_enabled && dev->IsQbbEnabled()) return;

    if (acc_timeout_count.find(qp->m_flow_id) == acc_timeout_count.end())
        acc_timeout_count[qp->m_flow_id] = 0;
    acc_timeout_count[qp->m_flow_id]++;

    if (m_enableBitmapRetrans) {
        auto bitmapTxKey = GetQpKey(qp->dip.Get(), qp->sport, qp->dport, qp->m_pg);
        uint32_t restartSeq =
            m_bitmapRetrans.GetOldestUnacked(bitmapTxKey, static_cast<uint32_t>(qp->snd_una));
        if (restartSeq < qp->snd_una) restartSeq = static_cast<uint32_t>(qp->snd_una);
        LogSenderTimeoutRetrans(m_node->GetId(), qp, restartSeq, qp->psnPath.packetSize,
                                rto.GetNanoSeconds());
        m_bitmapRetrans.ClearRetransMarker(bitmapTxKey, restartSeq);
        m_bitmapRetrans.TryScheduleRetrans(bitmapTxKey, restartSeq);
        uint32_t startPsn = restartSeq / qp->psnPath.packetSize;
        bool duplicated = false;
        for (size_t idx = 0; idx < qp->psnPath.pendingRetrans.size(); ++idx) {
            const PsnRetransRequest &req = qp->psnPath.pendingRetrans[idx];
            if (req.startPsn == startPsn && req.endPsn == startPsn) {
                duplicated = true;
                break;
            }
        }
        if (!duplicated) {
            PsnRetransRequest req;
            req.startPsn = startPsn;
            req.endPsn = startPsn;
            req.epoch = qp->psnPath.mappingEpoch;
            qp->psnPath.pendingRetrans.push_back(req);
        }
        if (m_enableConsoleLog == 1 && IsPsnInList(startPsn, m_consoleLogPsns)) {
            uint32_t packetSize = qp->psnPath.packetSize == 0 ? m_mtu : qp->psnPath.packetSize;
            std::cout << "[PSN_TRACE_TIMEOUT] t=" << Simulator::Now().GetNanoSeconds()
                      << " node=" << m_node->GetId()
                      << " qp=" << qp->m_flow_id
                      << " restartSeq=" << restartSeq
                      << " psn=" << startPsn
                      << " snd_nxt_psn=" << (packetSize == 0 ? 0 : qp->snd_nxt / packetSize)
                      << " snd_una_psn=" << (packetSize == 0 ? 0 : qp->snd_una / packetSize)
                      << " rto_ns=" << rto.GetNanoSeconds()
                      << " duplicated=" << (duplicated ? 1 : 0)
                      << " pendingRetrans=" << qp->psnPath.pendingRetrans.size()
                      << std::endl;
        }
        if (m_hpccTrace) {
            std::cout << "[BITMAP_TIMEOUT] t=" << Simulator::Now().GetNanoSeconds()
                      << " qp=" << qp->m_flow_id << " restartSeq=" << restartSeq
                      << " snd_una=" << qp->snd_una
                      << " queueDepth=" << qp->psnPath.pendingRetrans.size() << std::endl;
        }
        dev->TriggerTransmit();
        return;
    }
    if (m_enableFalcon) {
        auto falconTxKey = GetQpKey(qp->dip.Get(), qp->sport, qp->dport, qp->m_pg);
        uint32_t restartSeq =
            m_falcon.GetOldestOutstanding(falconTxKey, static_cast<uint32_t>(qp->snd_una));
        if (restartSeq < qp->snd_una) restartSeq = static_cast<uint32_t>(qp->snd_una);
        LogSenderTimeoutRetrans(m_node->GetId(), qp, restartSeq, qp->psnPath.packetSize,
                                rto.GetNanoSeconds());
        if (m_falcon.MarkRetransPending(falconTxKey, restartSeq)) {
            uint32_t startPsn = restartSeq / qp->psnPath.packetSize;
            bool duplicated = false;
            for (size_t idx = 0; idx < qp->psnPath.pendingRetrans.size(); ++idx) {
                const PsnRetransRequest &req = qp->psnPath.pendingRetrans[idx];
                if (req.startPsn == startPsn && req.endPsn == startPsn) {
                    duplicated = true;
                    break;
                }
            }
            if (!duplicated) {
                PsnRetransRequest req;
                req.startPsn = startPsn;
                req.endPsn = startPsn;
                req.epoch = qp->psnPath.mappingEpoch;
                qp->psnPath.pendingRetrans.push_back(req);
            }
        }
        if (m_hpccTrace) {
            std::cout << "[FALCON_TIMEOUT] t=" << Simulator::Now().GetNanoSeconds()
                      << " qp=" << qp->m_flow_id << " restartSeq=" << restartSeq
                      << " snd_una=" << qp->snd_una
                      << " pendingRetrans=" << qp->psnPath.pendingRetrans.size() << std::endl;
        }
        dev->TriggerTransmit();
        return;
    }

    if (qp->irn.m_enabled) qp->irn.m_recovery = true;

    LogSenderTimeoutRetrans(m_node->GetId(), qp, static_cast<uint32_t>(qp->snd_una),
                            qp->psnPath.packetSize, rto.GetNanoSeconds());
    RecoverQueue(qp);
    dev->TriggerTransmit();
}

void RdmaHw::EvaluatePathState(Ptr<RdmaQueuePair> qp) {
    if (!qp->psnPath.pathSelectionEnabled || !qp->psnPath.pathSwitchEnabled) return;
    if (qp->psnPath.pathStats.empty()) {
        SchedulePathStateEval(qp);
        return;
    }

    std::vector<uint32_t> beforeActive = CollectActivePathIndices(qp->psnPath.pathStats);
    std::vector<uint32_t> beforeProbing = CollectProbingPathIndices(qp->psnPath.pathStats);
    uint32_t oldEpoch = qp->psnPath.mappingEpoch;
    uint32_t oldOffset = qp->psnPath.o;
    uint32_t oldFpsn = qp->psnPath.fpsn;
    bool hasPathFeedback = false;
    for (uint32_t i = 0; i < qp->psnPath.pathStats.size(); ++i) {
        if (qp->psnPath.pathStats[i].ackCount > 0 || qp->psnPath.pathStats[i].nackCount > 0) {
            hasPathFeedback = true;
            break;
        }
    }
    std::string beforeStats = FormatPathStats(qp->psnPath.pathStats);

    uint32_t activeCount = 0;
    bool hasInactivePath = false;
    for (uint32_t i = 0; i < qp->psnPath.pathStats.size(); ++i) {
        if (!qp->psnPath.pathStats[i].active) {
            hasInactivePath = true;
            break;
        }
    }
    bool shouldProbeCandidate = false;
    for (uint32_t i = 0; i < qp->psnPath.pathStats.size(); ++i) {
        PsnPathStats &s = qp->psnPath.pathStats[i];
        s.score = static_cast<double>(s.ackCount) - m_psnPathBeta * static_cast<double>(s.nackCount);
        if (s.active) {
            if (s.score < static_cast<double>(m_psnPathTRemove) && qp->psnPath.activePathCount > 1) {
                s.active = false;
                s.probing = false;
            } else if (s.score < static_cast<double>(m_psnPathTExplore)) {
                s.probing = hasInactivePath;
                shouldProbeCandidate = hasInactivePath;
            } else {
                s.probing = false;
            }
        } else {
            s.probing = false;
        }
        if (s.active) activeCount++;
        s.ackCount = 0;
        s.nackCount = 0;
    }

    bool shouldLogEval = m_hpccTrace && (shouldProbeCandidate || !beforeProbing.empty());
    if (shouldLogEval) {
        std::cout << "[PATH_EVAL] t=" << Simulator::Now().GetNanoSeconds()
                  << " qp=" << qp->m_flow_id
                  << " beforeActive=" << FormatPathIndices(beforeActive)
                  << " beforeProbing=" << FormatPathIndices(beforeProbing)
                  << " activeCount=" << activeCount
                  << " shouldProbeCandidate=" << (shouldProbeCandidate ? 1 : 0) << std::endl;
    }

    if (shouldProbeCandidate) {
        for (uint32_t i = 0; i < qp->psnPath.pathStats.size(); ++i) {
            if (!qp->psnPath.pathStats[i].active) {
                qp->psnPath.pathStats[i].probing = true;
                break;
            }
        }
    }

    if (activeCount == 0 && !qp->psnPath.pathStats.empty()) {
        qp->psnPath.pathStats[0].active = true;
        activeCount = 1;
    }

    if (activeCount != qp->psnPath.activePathCount) {
        qp->psnPath.activePathCount = activeCount;
        qp->psnPath.fpsn =
            PsnPath::GetPsnFromSeq(static_cast<uint32_t>(qp->snd_nxt), qp->psnPath.packetSize);
        qp->psnPath.mappingEpoch++;
        for (uint32_t i = 0; i < qp->psnPath.pathStats.size(); ++i) {
            if (qp->psnPath.pathStats[i].active) {
                qp->psnPath.o = i % qp->psnPath.k;
                break;
            }
        }
    }

    bool pathChanged =
        (oldEpoch != qp->psnPath.mappingEpoch || oldOffset != qp->psnPath.o ||
         oldFpsn != qp->psnPath.fpsn ||
         beforeActive != CollectActivePathIndices(qp->psnPath.pathStats));
    if (pathChanged) {
        qp->stat.pathSwitchCount++;
    }

    if (m_enableConsoleLog >= 1 && (hasPathFeedback || pathChanged || shouldProbeCandidate ||
                                    !beforeProbing.empty())) {
        std::cout << "[PATH_EVAL_DETAIL] t=" << Simulator::Now().GetNanoSeconds()
                  << " qp=" << qp->m_flow_id
                  << " beta=" << m_psnPathBeta
                  << " tRemove=" << m_psnPathTRemove
                  << " tExplore=" << m_psnPathTExplore
                  << " evalIntervalUs=" << m_psnPathEvalIntervalUs
                  << " beforeActive=" << FormatPathIndices(beforeActive)
                  << " beforeProbing=" << FormatPathIndices(beforeProbing)
                  << " beforeStats=" << beforeStats
                  << " afterActive="
                  << FormatPathIndices(CollectActivePathIndices(qp->psnPath.pathStats))
                  << " afterProbing="
                  << FormatPathIndices(CollectProbingPathIndices(qp->psnPath.pathStats))
                  << " changed=" << (pathChanged ? 1 : 0)
                  << " shouldProbeCandidate=" << (shouldProbeCandidate ? 1 : 0)
                  << " activeCount=" << activeCount
                  << " oldEpoch=" << oldEpoch
                  << " newEpoch=" << qp->psnPath.mappingEpoch
                  << " oldOffset=" << oldOffset
                  << " newOffset=" << qp->psnPath.o
                  << " oldFpsn=" << oldFpsn
                  << " newFpsn=" << qp->psnPath.fpsn
                  << " pathSwitchCount=" << qp->stat.pathSwitchCount
                  << std::endl;
    }

    if (m_hpccTrace && (pathChanged || shouldProbeCandidate || !beforeProbing.empty())) {
        std::cout << "[PATH_SWITCH] t=" << Simulator::Now().GetNanoSeconds()
                  << " qp=" << qp->m_flow_id
                  << " changed=" << (pathChanged ? 1 : 0)
                  << " activePaths="
                  << FormatPathIndices(CollectActivePathIndices(qp->psnPath.pathStats))
                  << " probingPaths="
                  << FormatPathIndices(CollectProbingPathIndices(qp->psnPath.pathStats))
                  << " epoch=" << qp->psnPath.mappingEpoch << " k=" << qp->psnPath.k
                  << " offset=" << qp->psnPath.o << " fpsn=" << qp->psnPath.fpsn
                  << " oldEpoch=" << oldEpoch << " oldOffset=" << oldOffset
                  << " pathSwitchCount=" << qp->stat.pathSwitchCount
                  << " oldFpsn=" << oldFpsn << std::endl;
    }

    SchedulePathStateEval(qp);
}

void RdmaHw::SchedulePathStateEval(Ptr<RdmaQueuePair> qp) {
    if (!qp->psnPath.pathSelectionEnabled || !qp->psnPath.pathSwitchEnabled) return;
    Time t = MicroSeconds(m_psnPathEvalIntervalUs);
    if (t <= Time(0)) t = MicroSeconds(100);
    qp->psnPath.evalEvent = Simulator::Schedule(t, &RdmaHw::EvaluatePathState, this, qp);
}

void RdmaHw::UpdateNextAvail(Ptr<RdmaQueuePair> qp, Time interframeGap, uint32_t pkt_size) {
    Time sendingTime;
    if (m_rateBound)
        sendingTime = interframeGap + Seconds(qp->m_rate.CalculateTxTime(pkt_size));
    else
        sendingTime = interframeGap + Seconds(qp->m_max_rate.CalculateTxTime(pkt_size));
    qp->m_nextAvail = Simulator::Now() + sendingTime;
}

void RdmaHw::ChangeRate(Ptr<RdmaQueuePair> qp, DataRate new_rate) {
#if 1
    Time sendingTime = Seconds(qp->m_rate.CalculateTxTime(qp->lastPktSize));
    Time new_sendintTime = Seconds(new_rate.CalculateTxTime(qp->lastPktSize));
    qp->m_nextAvail = qp->m_nextAvail + new_sendintTime - sendingTime;
    // update nic's next avail event
    uint32_t nic_idx = GetNicIdxOfQp(qp);
    m_nic[nic_idx].dev->UpdateNextAvail(qp->m_nextAvail);
#endif

    // change to new rate
    qp->m_rate = new_rate;
}

#define PRINT_LOG 0
/******************************
 * Mellanox's version of DCQCN
 *****************************/
void RdmaHw::UpdateAlphaMlx(Ptr<RdmaQueuePair> q) {
#if PRINT_LOG
// std::cout << Simulator::Now() << " alpha update:" << m_node->GetId() << ' ' << q->mlx.m_alpha <<
// ' ' << (int)q->mlx.m_alpha_cnp_arrived << '\n'; printf("%lu alpha update: %08x %08x %u %u
// %.6lf->", Simulator::Now().GetTimeStep(), q->sip.Get(), q->dip.Get(), q->sport, q->dport,
// q->mlx.m_alpha);
#endif
    if (q->mlx.m_alpha_cnp_arrived) {                       // cnp -> increase
        q->mlx.m_alpha = (1 - m_g) * q->mlx.m_alpha + m_g;  // binary feedback
    } else {                                                // no cnp -> decrease
        q->mlx.m_alpha = (1 - m_g) * q->mlx.m_alpha;        // binary feedback
    }
#if PRINT_LOG
// printf("%.6lf\n", q->mlx.m_alpha);
#endif
    q->mlx.m_alpha_cnp_arrived = false;  // clear the CNP_arrived bit
    ScheduleUpdateAlphaMlx(q);
}
void RdmaHw::ScheduleUpdateAlphaMlx(Ptr<RdmaQueuePair> q) {
    q->mlx.m_eventUpdateAlpha = Simulator::Schedule(MicroSeconds(m_alpha_resume_interval),
                                                    &RdmaHw::UpdateAlphaMlx, this, q);
}

void RdmaHw::cnp_received_mlx(Ptr<RdmaQueuePair> q) {
    std::cout << "[DCQCN_CNP_RX] t=" << Simulator::Now().GetNanoSeconds()
              << " node=" << m_node->GetId()
              << " qp=" << q->m_flow_id
              << " rate_bps=" << q->m_rate.GetBitRate()
              << " target_rate_bps=" << q->mlx.m_targetRate.GetBitRate()
              << " alpha=" << q->mlx.m_alpha
              << " firstCnp=" << (q->mlx.m_first_cnp ? 1 : 0)
              << " rateDecreasePending=" << (q->mlx.m_decrease_cnp_arrived ? 1 : 0)
              << std::endl;
    q->mlx.m_alpha_cnp_arrived = true;     // set CNP_arrived bit for alpha update
    q->mlx.m_decrease_cnp_arrived = true;  // set CNP_arrived bit for rate decrease
    if (q->mlx.m_first_cnp) {
        // init alpha
        q->mlx.m_alpha = 1;
        q->mlx.m_alpha_cnp_arrived = false;
        // schedule alpha update
        ScheduleUpdateAlphaMlx(q);
        // schedule rate decrease
        ScheduleDecreaseRateMlx(q, 1);  // add 1 ns to make sure rate decrease is after alpha update
        // set rate on first CNP
        DataRate oldRate = q->m_rate;
        q->mlx.m_targetRate = q->m_rate = m_rateOnFirstCNP * q->m_rate;
        std::cout << "[DCQCN_FIRST_CNP_DECREASE] t=" << Simulator::Now().GetNanoSeconds()
                  << " node=" << m_node->GetId()
                  << " qp=" << q->m_flow_id
                  << " old_rate_bps=" << oldRate.GetBitRate()
                  << " new_rate_bps=" << q->m_rate.GetBitRate()
                  << " rateOnFirstCnp=" << m_rateOnFirstCNP
                  << " alpha=" << q->mlx.m_alpha
                  << std::endl;
        q->mlx.m_first_cnp = false;
    }
}

void RdmaHw::CheckRateDecreaseMlx(Ptr<RdmaQueuePair> q) {
    ScheduleDecreaseRateMlx(q, 0);
    if (q->mlx.m_decrease_cnp_arrived) {
#if PRINT_LOG
        printf("%lu rate dec: %08x %08x %u %u (%0.3lf %.3lf)->", Simulator::Now().GetTimeStep(),
               q->sip.Get(), q->dip.Get(), q->sport, q->dport,
               q->mlx.m_targetRate.GetBitRate() * 1e-9, q->m_rate.GetBitRate() * 1e-9);
#endif
        bool clamp = true;
        if (!m_EcnClampTgtRate) {
            if (q->mlx.m_rpTimeStage == 0) clamp = false;
        }
        if (clamp) {
            q->mlx.m_targetRate = q->m_rate;
        }
        DataRate oldRate = q->m_rate;
        q->m_rate = std::max(m_minRate, q->m_rate * (1 - q->mlx.m_alpha / 2));
        std::cout << "[DCQCN_RATE_DECREASE] t=" << Simulator::Now().GetNanoSeconds()
                  << " node=" << m_node->GetId()
                  << " qp=" << q->m_flow_id
                  << " old_rate_bps=" << oldRate.GetBitRate()
                  << " new_rate_bps=" << q->m_rate.GetBitRate()
                  << " target_rate_bps=" << q->mlx.m_targetRate.GetBitRate()
                  << " min_rate_bps=" << m_minRate.GetBitRate()
                  << " alpha=" << q->mlx.m_alpha
                  << " clamp=" << (clamp ? 1 : 0)
                  << std::endl;
        // reset rate increase related things
        q->mlx.m_rpTimeStage = 0;
        q->mlx.m_decrease_cnp_arrived = false;
        Simulator::Cancel(q->mlx.m_rpTimer);
        q->mlx.m_rpTimer = Simulator::Schedule(MicroSeconds(m_rpgTimeReset),
                                               &RdmaHw::RateIncEventTimerMlx, this, q);
#if PRINT_LOG
        printf("(%.3lf %.3lf)\n", q->mlx.m_targetRate.GetBitRate() * 1e-9,
               q->m_rate.GetBitRate() * 1e-9);
#endif
    }
}
void RdmaHw::ScheduleDecreaseRateMlx(Ptr<RdmaQueuePair> q, uint32_t delta) {
    q->mlx.m_eventDecreaseRate =
        Simulator::Schedule(MicroSeconds(m_rateDecreaseInterval) + NanoSeconds(delta),
                            &RdmaHw::CheckRateDecreaseMlx, this, q);
}

void RdmaHw::RateIncEventTimerMlx(Ptr<RdmaQueuePair> q) {
    q->mlx.m_rpTimer =
        Simulator::Schedule(MicroSeconds(m_rpgTimeReset), &RdmaHw::RateIncEventTimerMlx, this, q);
    RateIncEventMlx(q);
    q->mlx.m_rpTimeStage++;
}
void RdmaHw::RateIncEventMlx(Ptr<RdmaQueuePair> q) {
    // check which increase phase: fast recovery, active increase, hyper increase
    if (q->mlx.m_rpTimeStage < m_rpgThreshold) {  // fast recovery
        FastRecoveryMlx(q);
    } else if (q->mlx.m_rpTimeStage == m_rpgThreshold) {  // active increase
        ActiveIncreaseMlx(q);
    } else {  // hyper increase
        HyperIncreaseMlx(q);
    }
}

void RdmaHw::FastRecoveryMlx(Ptr<RdmaQueuePair> q) {
#if PRINT_LOG
    printf("%lu fast recovery: %08x %08x %u %u (%0.3lf %.3lf)->", Simulator::Now().GetTimeStep(),
           q->sip.Get(), q->dip.Get(), q->sport, q->dport, q->mlx.m_targetRate.GetBitRate() * 1e-9,
           q->m_rate.GetBitRate() * 1e-9);
#endif
    q->m_rate = (q->m_rate / 2) + (q->mlx.m_targetRate / 2);
#if PRINT_LOG
    printf("(%.3lf %.3lf)\n", q->mlx.m_targetRate.GetBitRate() * 1e-9,
           q->m_rate.GetBitRate() * 1e-9);
#endif
}
void RdmaHw::ActiveIncreaseMlx(Ptr<RdmaQueuePair> q) {
#if PRINT_LOG
    printf("%lu active inc: %08x %08x %u %u (%0.3lf %.3lf)->", Simulator::Now().GetTimeStep(),
           q->sip.Get(), q->dip.Get(), q->sport, q->dport, q->mlx.m_targetRate.GetBitRate() * 1e-9,
           q->m_rate.GetBitRate() * 1e-9);
#endif
    // get NIC
    uint32_t nic_idx = GetNicIdxOfQp(q);
    Ptr<QbbNetDevice> dev = m_nic[nic_idx].dev;
    // increate rate
    q->mlx.m_targetRate += m_rai;
    if (q->mlx.m_targetRate > dev->GetDataRate()) q->mlx.m_targetRate = dev->GetDataRate();
    q->m_rate = (q->m_rate / 2) + (q->mlx.m_targetRate / 2);
#if PRINT_LOG
    printf("(%.3lf %.3lf)\n", q->mlx.m_targetRate.GetBitRate() * 1e-9,
           q->m_rate.GetBitRate() * 1e-9);
#endif
}
void RdmaHw::HyperIncreaseMlx(Ptr<RdmaQueuePair> q) {
#if PRINT_LOG
    printf("%lu hyper inc: %08x %08x %u %u (%0.3lf %.3lf)->", Simulator::Now().GetTimeStep(),
           q->sip.Get(), q->dip.Get(), q->sport, q->dport, q->mlx.m_targetRate.GetBitRate() * 1e-9,
           q->m_rate.GetBitRate() * 1e-9);
#endif
    // get NIC
    uint32_t nic_idx = GetNicIdxOfQp(q);
    Ptr<QbbNetDevice> dev = m_nic[nic_idx].dev;
    // increate rate
    q->mlx.m_targetRate += m_rhai;
    if (q->mlx.m_targetRate > dev->GetDataRate()) q->mlx.m_targetRate = dev->GetDataRate();
    q->m_rate = (q->m_rate / 2) + (q->mlx.m_targetRate / 2);
#if PRINT_LOG
    printf("(%.3lf %.3lf)\n", q->mlx.m_targetRate.GetBitRate() * 1e-9,
           q->m_rate.GetBitRate() * 1e-9);
#endif
}

/***********************
 * High Precision CC
 ***********************/
static inline uint32_t
HpccPathSignatureFromInt(const IntHeader &ih)
{
    uint32_t sig = 2166136261u;
    sig ^= ih.nhop;
    sig *= 16777619u;
    uint32_t n = std::min((uint32_t)ih.nhop, (uint32_t)IntHeader::maxHop);
    for (uint32_t i = 0; i < n; i++) {
        uint64_t lr = ih.hop[i].GetLineRate();
        sig ^= (uint32_t)(lr & 0xffffffffu);
        sig *= 16777619u;
        sig ^= (uint32_t)(lr >> 32);
        sig *= 16777619u;
    }
    return sig;
}

void RdmaHw::HandleAckHp(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch, bool ack_progress) {
    uint32_t ack_seq = ch.ack.seq;
    // Duplicate pure ACKs carry stale/no-progress feedback; ignore for HPCC update path.
    if (!ack_progress && ch.l3Prot == 0xFC) {
        if (m_hpccTrace) {
            std::cout << "[HPCC_SKIP] reason=duplicate_ack"
                      << " ack_seq=" << ack_seq << " ack_type=0x" << std::hex
                      << (uint32_t)ch.l3Prot << std::dec << " ack_progress=0" << std::endl;
        }
        return;
    }
    // update rate
    if (ack_seq > qp->hp.m_lastUpdateSeq) {  // if full RTT feedback is ready, do full update
        qp->hp.m_fastReactBudget = 4;
        UpdateRateHp(qp, p, ch, false);
    } else {  // do fast react
        if (ack_progress) {
            FastReactHp(qp, p, ch);
        } else if (m_hpccTrace) {
            std::cout << "[HPCC_SKIP] reason=no_ack_progress"
                      << " ack_seq=" << ack_seq
                      << " lastUpdateSeq=" << qp->hp.m_lastUpdateSeq << std::endl;
        }
    }
}

void RdmaHw::UpdateRateHp(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch, bool fast_react) {
    if (m_hpccTrace) {
        std::cout << "[HPCC] UpdateRateHp called, qp=" << qp->sip << ":" << qp->sport << "->"
                  << qp->dip << ":" << qp->dport << " ackSeq=" << ch.ack.seq
                  << " curRate=" << qp->m_rate.GetBitRate() << " fast=" << (fast_react ? 1 : 0)
                  << std::endl;
    }
    uint32_t next_seq = qp->snd_nxt;
    IntHeader &ih = ch.ack.ih;
    if (ih.nhop == 0 || ih.nhop > IntHeader::maxHop) {
        return;
    }
    uint32_t hpPathId = 0;
    if (qp->psnPath.pathSelectionEnabled && !qp->psnPath.pathStats.empty()) {
        uint32_t ackPsn = PsnPath::GetPsnFromSeq(ch.ack.seq, qp->psnPath.packetSize);
        hpPathId = SelectPathFromStats(ackPsn, qp->psnPath.k, qp->psnPath.o,
                                       qp->psnPath.pathStats);
    }
    uint32_t hpPathCount =
        qp->psnPath.pathSelectionEnabled ? std::max<uint32_t>(qp->psnPath.k, 1) : 1;
    if (qp->hp.pathState.size() < hpPathCount) {
        qp->hp.pathState.resize(hpPathCount);
    }
    if (hpPathId >= qp->hp.pathState.size()) {
        hpPathId = 0;
    }
    HpPathState &hpPath = qp->hp.pathState[hpPathId];
    bool updateWc = ch.ack.seq > qp->hp.m_lastUpdateSeq;

    if (qp->hp.m_lastUpdateSeq == 0) {  // first RTT only seeds the telemetry baseline
        qp->hp.m_lastUpdateSeq = next_seq;
        hpPath.ownerPathSig = HpccPathSignatureFromInt(ih);
        hpPath.initialized = true;
        for (uint32_t i = 0; i < ih.nhop; i++) {
            hpPath.hop[i] = ih.hop[i];
            hpPath.lastTxBytes[i] = ih.hop[i].GetBytes();
            hpPath.lastTimestamp[i] = ih.hop[i].GetTime();
        }
        return;
    }

    if (!hpPath.initialized) {
        hpPath.ownerPathSig = HpccPathSignatureFromInt(ih);
        hpPath.initialized = true;
        for (uint32_t i = 0; i < ih.nhop; i++) {
            hpPath.hop[i] = ih.hop[i];
            hpPath.lastTxBytes[i] = ih.hop[i].GetBytes();
            hpPath.lastTimestamp[i] = ih.hop[i].GetTime();
        }
        if (!fast_react && next_seq > qp->hp.m_lastUpdateSeq) {
            qp->hp.m_lastUpdateSeq = next_seq;
        }
        return;
    }

    if (!fast_react) {
        uint32_t sig = HpccPathSignatureFromInt(ih);
        if (hpPath.ownerPathSig == 0) {
            hpPath.ownerPathSig = sig;
        } else if (hpPath.ownerPathSig != sig) {
            if (m_hpccTrace) {
                std::cout << "[HPCC_SKIP] reason=path_signature_change"
                          << " ack_seq=" << ch.ack.seq << " pathId=" << hpPathId
                          << " oldSig=" << hpPath.ownerPathSig << " newSig=" << sig
                          << std::endl;
            }
            hpPath.ownerPathSig = sig;
            for (uint32_t i = 0; i < ih.nhop; i++) {
                hpPath.hop[i] = ih.hop[i];
                hpPath.lastTxBytes[i] = ih.hop[i].GetBytes();
                hpPath.lastTimestamp[i] = ih.hop[i].GetTime();
            }
            qp->hp.m_fastReactBudget = 0;
            if (next_seq > qp->hp.m_lastUpdateSeq) {
                qp->hp.m_lastUpdateSeq = next_seq;
            }
            return;
        }
    }

    const double baseRttSec = std::max(1e-9, qp->hp.m_baseRtt);
    const double eta = std::max(1e-6, qp->hp.m_eta);
    double maxUPrime = 0.0;
    double tauForU = baseRttSec;
    bool updated_any = false;
    for (uint32_t i = 0; i < ih.nhop; i++) {
        if (m_sampleFeedback && fast_react && ih.hop[i].GetQlen() == 0) {
            continue;
        }

        const uint64_t tsNow = ih.hop[i].GetTime();
        const uint64_t tsLast = hpPath.lastTimestamp[i];
        const uint64_t bytesNow = ih.hop[i].GetBytes();
        const uint64_t bytesLast = hpPath.lastTxBytes[i];
        const double lineRate = (double)ih.hop[i].GetLineRate();
        if (tsLast == 0 || lineRate <= 0) {
            hpPath.lastTimestamp[i] = tsNow;
            hpPath.lastTxBytes[i] = bytesNow;
            hpPath.hop[i] = ih.hop[i];
            continue;
        }

        IntHop prevHop = hpPath.hop[i];
        uint64_t tau = ih.hop[i].GetTimeDelta(prevHop);
        uint32_t tauMultiplier =
            qp->psnPath.pathSelectionEnabled ? std::max<uint32_t>(4, qp->psnPath.k * 4) : 4;
        uint64_t tauLimit = std::max<uint64_t>(1, qp->m_baseRtt * tauMultiplier);
        if (tau == 0 || tau > tauLimit) {
            if (m_hpccTrace) {
                std::cout << "[HPCC_SKIP] reason=stale_tau"
                          << " ack_seq=" << ch.ack.seq << " pathId=" << hpPathId
                          << " hop=" << i << " tau=" << tau
                          << " tauLimit=" << tauLimit << " baseRtt=" << qp->m_baseRtt
                          << std::endl;
            }
            hpPath.lastTimestamp[i] = tsNow;
            hpPath.lastTxBytes[i] = bytesNow;
            hpPath.hop[i] = ih.hop[i];
            continue;
        }

        double duration = tau * 1e-9;
        if (duration <= 0.0) {
            hpPath.lastTimestamp[i] = tsNow;
            hpPath.lastTxBytes[i] = bytesNow;
            hpPath.hop[i] = ih.hop[i];
            continue;
        }
        double txRate = 0.0;
        if (bytesNow >= bytesLast) {
            txRate = (bytesNow - bytesLast) * 8.0 / duration;
        } else {
            // Defensive reset when the INT byte counter appears to move backwards.
            hpPath.lastTimestamp[i] = tsNow;
            hpPath.lastTxBytes[i] = bytesNow;
            hpPath.hop[i] = ih.hop[i];
            continue;
        }
        if (txRate > lineRate * 1.25) {
            if (m_hpccTrace) {
                std::cout << "[HPCC_SKIP] reason=bad_txRate"
                          << " ack_seq=" << ch.ack.seq << " pathId=" << hpPathId
                          << " hop=" << i << " txRate=" << txRate
                          << " lineRate=" << lineRate << std::endl;
            }
            hpPath.lastTimestamp[i] = tsNow;
            hpPath.lastTxBytes[i] = bytesNow;
            hpPath.hop[i] = ih.hop[i];
            continue;
        }

        double qBits = std::min(ih.hop[i].GetQlen(), prevHop.GetQlen()) * 8.0;
        double uPrime = qBits / (lineRate * baseRttSec) + txRate / lineRate;
        if (uPrime < 0.0) {
            uPrime = 0.0;
        }
        if (uPrime > maxUPrime) {
            maxUPrime = uPrime;
            tauForU = std::min(duration, baseRttSec);
        }
        updated_any = true;
        hpPath.lastTimestamp[i] = tsNow;
        hpPath.lastTxBytes[i] = bytesNow;
        hpPath.hop[i] = ih.hop[i];

        if (m_hpccTrace) {
            std::cout << "[HPCC_INT] nhop=" << ih.nhop << " hop=" << i
                      << " pathId=" << hpPathId
                      << " qLen=" << ih.hop[i].GetQlen() << " txBytes=" << ih.hop[i].GetBytes()
                      << " timestamp=" << ih.hop[i].GetTime()
                      << " lineRate=" << ih.hop[i].GetLineRate() << " tau=" << tau
                      << " txRate=" << txRate << " uPrime=" << uPrime << std::endl;
        }
    }

    if (updated_any) {
        double alpha = std::min(std::max(tauForU / baseRttSec, 0.0), 1.0);
        qp->hp.u = (1.0 - alpha) * qp->hp.u + alpha * maxUPrime;

        double W = 0.0;
        if (qp->hp.u >= eta || qp->hp.m_incStage >= qp->hp.m_maxStage) {
            double ratio = std::max(qp->hp.u / eta, 1e-6);
            W = qp->hp.m_Wc / ratio + qp->hp.m_WAI;
            if (updateWc) {
                qp->hp.m_incStage = 0;
                qp->hp.m_Wc = W;
            }
        } else {
            W = qp->hp.m_Wc + qp->hp.m_WAI;
            if (updateWc) {
                qp->hp.m_incStage++;
                qp->hp.m_Wc = W;
            }
        }
        W = std::max(W, 1.0);
        qp->hp.m_W = W;
        double rateBps = W * 8.0 / baseRttSec;
        DataRate new_rate((uint64_t)rateBps);
        if (new_rate < m_minRate) new_rate = m_minRate;
        if (new_rate > qp->m_max_rate) new_rate = qp->m_max_rate;

        DataRate old_rate = qp->m_rate;
        ChangeRate(qp, new_rate);
        qp->hp.m_curRate = new_rate;
        if (new_rate < old_rate) {
            LogSenderHpccRateDown(m_node->GetId(), qp, old_rate, new_rate, hpPathId, qp->hp.u,
                                  maxUPrime);
        }

        if (m_hpccTrace) {
            std::cout << "[HPCC_RATE] qp=" << qp->m_flow_id << " oldRate="
                      << old_rate.GetBitRate() << " newRate=" << new_rate.GetBitRate()
                      << " pathId=" << hpPathId
                      << " u=" << qp->hp.u << " uPrime=" << maxUPrime
                      << " Wc=" << qp->hp.m_Wc << " W=" << W << " alpha=" << alpha
                      << " updateWc=" << (updateWc ? 1 : 0)
                      << " incStage=" << qp->hp.m_incStage
                      << " baseRtt=" << baseRttSec << " fastReact=" << (fast_react ? 1 : 0)
                      << std::endl;
        }
    }

    if (updateWc) {
        qp->hp.m_lastUpdateSeq = next_seq;
    }
}

void RdmaHw::FastReactHp(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch) {
    if (!m_fast_react) {
        return;
    }
    // bound fast-react to avoid repeated reaction on stale/duplicate ACK feedback
    if (qp->hp.m_fastReactBudget == 0) {
        return;
    }
    if (ch.ack.seq <= qp->hp.m_lastFastReactSeq) {
        return;
    }
    qp->hp.m_lastFastReactSeq = ch.ack.seq;
    qp->hp.m_fastReactBudget--;
    UpdateRateHp(qp, p, ch, true);
}

/**********************
 * TIMELY
 *********************/
void RdmaHw::HandleAckTimely(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch) {
    uint32_t ack_seq = ch.ack.seq;
    // update rate
    if (ack_seq > qp->tmly.m_lastUpdateSeq) {  // if full RTT feedback is ready, do full update
        UpdateRateTimely(qp, p, ch, false);
    } else {  // do fast react
        FastReactTimely(qp, p, ch);
    }
}
void RdmaHw::UpdateRateTimely(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch, bool us) {
    uint32_t next_seq = qp->snd_nxt;
    uint64_t rtt = Simulator::Now().GetTimeStep() - ch.ack.ih.ts;
    bool print = !us;
    if (qp->tmly.m_lastUpdateSeq != 0) {  // not first RTT
        int64_t new_rtt_diff = (int64_t)rtt - (int64_t)qp->tmly.lastRtt;
        double rtt_diff = (1 - m_tmly_alpha) * qp->tmly.rttDiff + m_tmly_alpha * new_rtt_diff;
        double gradient = rtt_diff / m_tmly_minRtt;
        bool inc = false;
        double c = 0;
#if PRINT_LOG
        if (print)
            printf("%lu node:%u rtt:%lu rttDiff:%.0lf gradient:%.3lf rate:%.3lf",
                   Simulator::Now().GetTimeStep(), m_node->GetId(), rtt, rtt_diff, gradient,
                   qp->tmly.m_curRate.GetBitRate() * 1e-9);
#endif
        if (rtt < m_tmly_TLow) {
            inc = true;
        } else if (rtt > m_tmly_THigh) {
            c = 1 - m_tmly_beta * (1 - (double)m_tmly_THigh / rtt);
            inc = false;
        } else if (gradient <= 0) {
            inc = true;
        } else {
            c = 1 - m_tmly_beta * gradient;
            if (c < 0) c = 0;
            inc = false;
        }
        if (inc) {
            if (qp->tmly.m_incStage < 5) {
                qp->m_rate = qp->tmly.m_curRate + m_rai;
            } else {
                qp->m_rate = qp->tmly.m_curRate + m_rhai;
            }
            if (qp->m_rate > qp->m_max_rate) qp->m_rate = qp->m_max_rate;
            if (!us) {
                qp->tmly.m_curRate = qp->m_rate;
                qp->tmly.m_incStage++;
                qp->tmly.rttDiff = rtt_diff;
            }
        } else {
            qp->m_rate = std::max(m_minRate, qp->tmly.m_curRate * c);
            if (!us) {
                qp->tmly.m_curRate = qp->m_rate;
                qp->tmly.m_incStage = 0;
                qp->tmly.rttDiff = rtt_diff;
            }
        }
#if PRINT_LOG
        if (print) {
            printf(" %c %.3lf\n", inc ? '^' : 'v', qp->m_rate.GetBitRate() * 1e-9);
        }
#endif
    }
    if (!us && next_seq > qp->tmly.m_lastUpdateSeq) {
        qp->tmly.m_lastUpdateSeq = next_seq;
        // update
        qp->tmly.lastRtt = rtt;
    }
}
void RdmaHw::FastReactTimely(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch) {}

/**********************
 * DCTCP
 *********************/
void RdmaHw::HandleAckDctcp(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch) {
    uint32_t ack_seq = ch.ack.seq;
    uint8_t cnp = (ch.ack.flags >> qbbHeader::FLAG_CNP) & 1;
    bool new_batch = false;

    // update alpha
    qp->dctcp.m_ecnCnt += (cnp > 0);
    if (ack_seq > qp->dctcp.m_lastUpdateSeq) {  // if full RTT feedback is ready, do alpha update
#if PRINT_LOG
        printf("%lu %s %08x %08x %u %u [%u,%u,%u] %.3lf->", Simulator::Now().GetTimeStep(), "alpha",
               qp->sip.Get(), qp->dip.Get(), qp->sport, qp->dport, qp->dctcp.m_lastUpdateSeq,
               ch.ack.seq, qp->snd_nxt, qp->dctcp.m_alpha);
#endif
        new_batch = true;
        if (qp->dctcp.m_lastUpdateSeq == 0) {  // first RTT
            qp->dctcp.m_lastUpdateSeq = qp->snd_nxt;
            qp->dctcp.m_batchSizeOfAlpha = qp->snd_nxt / m_mtu + 1;
        } else {
            double frac = std::min(1.0, double(qp->dctcp.m_ecnCnt) / qp->dctcp.m_batchSizeOfAlpha);
            qp->dctcp.m_alpha = (1 - m_g) * qp->dctcp.m_alpha + m_g * frac;
            qp->dctcp.m_lastUpdateSeq = qp->snd_nxt;
            qp->dctcp.m_ecnCnt = 0;
            qp->dctcp.m_batchSizeOfAlpha = (qp->snd_nxt - ack_seq) / m_mtu + 1;
#if PRINT_LOG
            printf("%.3lf F:%.3lf", qp->dctcp.m_alpha, frac);
#endif
        }
#if PRINT_LOG
        printf("\n");
#endif
    }

    // check cwr exit
    if (qp->dctcp.m_caState == 1) {
        if (ack_seq > qp->dctcp.m_highSeq) qp->dctcp.m_caState = 0;
    }

    // check if need to reduce rate: ECN and not in CWR
    if (cnp && qp->dctcp.m_caState == 0) {
#if PRINT_LOG
        printf("%lu %s %08x %08x %u %u %.3lf->", Simulator::Now().GetTimeStep(), "rate",
               qp->sip.Get(), qp->dip.Get(), qp->sport, qp->dport, qp->m_rate.GetBitRate() * 1e-9);
#endif
        qp->m_rate = std::max(m_minRate, qp->m_rate * (1 - qp->dctcp.m_alpha / 2));
#if PRINT_LOG
        printf("%.3lf\n", qp->m_rate.GetBitRate() * 1e-9);
#endif
        qp->dctcp.m_caState = 1;
        qp->dctcp.m_highSeq = qp->snd_nxt;
    }

    // additive inc
    if (qp->dctcp.m_caState == 0 && new_batch)
        qp->m_rate = std::min(qp->m_max_rate, qp->m_rate + m_dctcp_rai);
}

}  // namespace ns3
