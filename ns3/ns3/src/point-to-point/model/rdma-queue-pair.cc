#include "rdma-queue-pair.h"

#include <algorithm>

#include <ns3/hash.h>
#include <ns3/ipv4-header.h>
#include <ns3/log.h>
#include <ns3/seq-ts-header.h>
#include <ns3/simulator.h>
#include <ns3/udp-header.h>
#include <ns3/uinteger.h>

#include "ns3/ppp-header.h"
#include "ns3/settings.h"
#include "rdma-hw.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("RdmaQueuePair");

uint32_t PsnMappingSnapshot::PathForPsn(uint32_t psn) const {
    NS_ABORT_MSG_IF(k == 0, "PSN-PATH requires positive K for path lookup");
    // The phase anchor defines the split schedule independently of the
    // physical mapping's FPSN. It is carried in simulation tag metadata.
    const bool candidate = probeStep != 0 && psn >= phaseStart &&
        ((psn - phaseStart) / k) % (probeStep + 1) == probeStep;
    return lane + (candidate ? candidateOffset : o);
}

bool PsnMappingSnapshot::PreviousPsn(uint32_t psn, uint32_t &previous) const {
    return PreviousPsnOnPath(psn, PathForPsn(psn), previous);
}

bool PsnMappingSnapshot::PreviousPsnOnPath(uint32_t before, uint32_t path,
                                         uint32_t &previous) const {
    NS_ABORT_MSG_IF(k == 0, "PSN-PATH requires positive K for predecessor lookup");
    if (before <= phaseStart) return false;
    uint64_t ordinal = (uint64_t(before) - phaseStart - 1) / k;
    if (path == lane + o) {
        if (probeStep != 0 && ordinal % (uint64_t(probeStep) + 1) == probeStep)
            --ordinal;
    } else if (probeStep != 0 && path == lane + candidateOffset) {
        const uint64_t period = uint64_t(probeStep) + 1;
        if (ordinal < probeStep) return false;
        ordinal -= (ordinal - probeStep) % period;
    } else {
        return false;
    }
    previous = phaseStart + ordinal * k;
    return true;
}

bool RdmaRxQueuePair::ObservePsnMapping(const PsnMappingSnapshot &mapping) {
    // Keep only the current FPSN* boundary required for this logical lane.
    // Late packets from older mappings must never roll the boundary back.
    auto boundary = psnMappingFpsn.emplace(mapping.lane, mapping.lane).first;
    if (mapping.fpsn <= boundary->second) return false;
    boundary->second = mapping.fpsn;
    return true;
}

bool RdmaRxQueuePair::FindPsnPredecessor(const PsnMappingSnapshot &mapping,
                                       uint32_t psn, uint32_t &previous) const {
    // The current packet describes only its own alternating-send phase.
    // A predecessor across that phase boundary is left to sender RTO recovery.
    return mapping.PreviousPsn(psn, previous) && previous >= mapping.fpsn;
}

bool RdmaRxQueuePair::FindPsnLossRange(const PsnMappingSnapshot &mapping,
                                     uint32_t psn, uint32_t expectedPsn,
                                     PsnPathNackRange &range) const {
    uint32_t previous;
    if (!FindPsnPredecessor(mapping, psn, previous) || previous < expectedPsn ||
        bitmap[previous % BITMAP_SIZE] != 0) return false;
    range.pathId = mapping.pathId;
    range.k = mapping.k;
    range.startPsn = range.endPsn = previous;
    while (range.startPsn > expectedPsn) {
        // Aggregate only losses identifiable from this packet's phase metadata.
        // Do not reconstruct or retain earlier sending schedules.
        if (!FindPsnPredecessor(mapping, range.startPsn, previous) ||
            previous >= range.startPsn || previous < expectedPsn ||
            bitmap[previous % BITMAP_SIZE] != 0) break;
        range.startPsn = previous;
    }
    return true;
}

void PsnPathState::InitializeLanes() {
    NS_ABORT_MSG_IF(k == 0, "PSN-PATH requires explicit positive K");
    if (!lanes.empty()) return;
    NS_ABORT_MSG_IF(basePort == 0 || basePort + uint64_t(k) + o - 1 > UINT16_MAX,
                    "PSN-PATH source port range exceeds UDP port space");
    lanes.resize(k);
    pathStats.resize(k + o);
    for (auto &stats : pathStats) stats.active = false;
    for (uint32_t lane = 0; lane < k; ++lane) {
        lanes[lane].offset = o;
        lanes[lane].fpsn = lane;
        lanes[lane].phaseStart = lane;
        lanes[lane].candidateCursor = o;
        pathStats[lane + o].active = true;
    }
    activePathCount = k;
}

uint32_t PsnPathState::SelectPath(uint32_t psn) const {
    NS_ABORT_MSG_IF(k == 0 || lanes.size() != k, "PSN-PATH lanes are not initialized");
    const uint32_t lane = psn % k;
    return lane + lanes[lane].offset;
}

uint32_t PsnPathState::SelectOriginalPath(uint32_t psn) const {
    const uint32_t laneId = psn % k;
    const PsnLaneState &lane = lanes[laneId];
    PsnMappingSnapshot mapping;
    mapping.k = k;
    mapping.lane = laneId;
    mapping.o = lane.offset;
    mapping.fpsn = lane.fpsn;
    mapping.phaseStart = lane.phaseStart;
    mapping.probeStep = lane.probeStep;
    mapping.candidateOffset = lane.candidateOffset;
    return mapping.PathForPsn(psn);
}

void PsnPathState::RecordTransmission(uint32_t psn, uint32_t size, bool retransmitted) {
    auto it = transmissions.find(psn);
    if (it != transmissions.end()) {
        it->second.retransmitted = true;
        return;
    }
    // A stale retry already covered by cumulative ACK must not become a new success.
    if (retransmitted) return;
    PsnLaneState &lane = lanes[psn % k];
    if (lane.boundaryPending) {
        lane.phaseStart = psn;
        lane.boundaryPending = false;
    }
    if (lane.physicalBoundaryPending) {
        lane.fpsn = psn;
        lane.physicalBoundaryPending = false;
    }
    PsnTransmission tx;
    tx.pathId = SelectOriginalPath(psn);
    tx.phaseStart = lane.phaseStart;
    tx.hasPhaseInfo = lane.probeStep != 0 || lane.phaseClosing;
    tx.offset = lane.offset;
    tx.size = size;
    tx.probeStep = lane.probeStep;
    tx.candidateOffset = lane.candidateOffset;
    tx.candidateGeneration = lane.candidateGeneration;
    tx.probe = lane.probeStep != 0 && tx.pathId == psn % k + lane.candidateOffset;
    if (tx.probe && !lane.candidateStarted) {
        lane.candidateFpsn = psn;
        lane.candidateStarted = true;
    }
    tx.fpsn = tx.probe ? lane.candidateFpsn : lane.fpsn;
    transmissions.emplace(psn, tx);
}

bool PsnPathState::FindAvailablePath(uint32_t laneId, uint32_t startOffset,
                                   uint32_t maxOffset, uint32_t &path) {
    PsnLaneState &lane = lanes[laneId];
    const uint32_t lastOffset = std::min<uint32_t>(maxOffset, UINT16_MAX - basePort - laneId);
    const uint64_t offsetCount = uint64_t(lastOffset) + 1;
    // Revisit released offsets after wrapping the field, while retaining every
    // active, probing, or in-flight mapping until it is safe to reuse.
    for (uint64_t attempt = 1; attempt <= offsetCount; ++attempt) {
        const uint32_t offset = (uint64_t(startOffset) + attempt) % offsetCount;
        const uint32_t nextPath = laneId + offset;
        bool occupied = nextPath < pathStats.size() &&
            (pathStats[nextPath].active || pathStats[nextPath].probing);
        for (const auto &entry : transmissions) {
            if (entry.second.pathId == nextPath) { occupied = true; break; }
        }
        if (occupied) continue;
        lane.candidateCursor = offset;
        path = nextPath;
        return true;
    }
    return false;
}

bool PsnPathState::StartProbe(uint32_t laneId, uint32_t step) {
    NS_ABORT_MSG_IF(step == 0 || step > 0x7f, "PSN-PATH probe step must fit Explore's seven bits");
    PsnLaneState &lane = lanes[laneId];
    if (lane.probeStep != 0) return true;
    uint32_t candidate;
    // Explore carries an eight-bit candidate offset (paper Fig. 7).
    if (!FindAvailablePath(laneId, lane.candidateCursor, UINT8_MAX, candidate)) return false;
    if (pathStats.size() <= candidate) pathStats.resize(candidate + 1);
    pathStats[candidate] = PsnPathStats{};
    pathStats[candidate].probing = true;
    lane.candidateOffset = candidate - laneId;
    lane.candidateStarted = false;
    lane.probeStep = step;
    lane.stepAcks = 0;
    lane.migrating = false;
    lane.candidateGeneration = ++mappingEpoch;
    lane.phaseClosing = false;
    lane.boundaryPending = true;
    return true;
}

void PsnPathState::AbortProbe(uint32_t laneId) {
    PsnLaneState &lane = lanes[laneId];
    if (lane.probeStep == 0) return;
    pathStats[laneId + lane.candidateOffset].probing = false;
    lane.probeStep = 0;
    lane.stepAcks = 0;
    lane.migrating = false;
    lane.candidateGeneration = 0;
    lane.phaseClosing = true;
    ++mappingEpoch;
    lane.boundaryPending = true;
}

void PsnPathState::ConfirmCandidate(uint32_t laneId) {
    PsnLaneState &lane = lanes[laneId];
    NS_ABORT_MSG_IF(lane.probeStep == 0, "Cannot confirm a path which was not probed");
    lane.migrating = true;
    lane.stepAcks = 0;
}

void PsnPathState::CommitCandidate(uint32_t laneId) {
    PsnLaneState &lane = lanes[laneId];
    NS_ABORT_MSG_IF(lane.probeStep == 0 || !lane.candidateStarted,
                    "Cannot commit a path before its first original packet");
    pathStats[laneId + lane.offset].active = false;
    const uint32_t candidate = laneId + lane.candidateOffset;
    pathStats[candidate].active = true;
    pathStats[candidate].probing = false;
    lane.offset = lane.candidateOffset;
    lane.fpsn = lane.candidateFpsn;
    lane.phaseClosing = true;
    lane.probeStep = 0;
    lane.stepAcks = 0;
    lane.migrating = false;
    lane.candidateGeneration = 0;
    ++mappingEpoch;
    lane.boundaryPending = true;
    ++switchCount;
}

bool PsnPathState::ReplacePath(uint32_t laneId) {
    PsnLaneState &lane = lanes[laneId];
    const uint32_t oldPath = laneId + lane.offset;
    AbortProbe(laneId);
    uint32_t nextPath;
    // Severe degradation searches from O_l + 1, independently of earlier probes.
    if (FindAvailablePath(laneId, lane.offset, UINT16_MAX, nextPath)) {
        pathStats[oldPath].active = false;
        pathStats[oldPath].probing = false;
        if (pathStats.size() <= nextPath) pathStats.resize(nextPath + 1);
        pathStats[nextPath] = PsnPathStats{};
        pathStats[nextPath].active = true;
        lane.offset = nextPath - laneId;
        lane.physicalBoundaryPending = true;
        lane.phaseClosing = false;
        ++mappingEpoch;
        lane.boundaryPending = true;
        ++switchCount;
        return true;
    }
    return false;
}

void PsnPathState::RecordSuccess(uint32_t psn, PsnTransmission &tx) {
    if (tx.ackCounted || tx.retransmitted || tx.lossCounted) return;
    tx.ackCounted = true;
    ++pathStats[tx.pathId].ackCount;
    PsnLaneState &lane = lanes[psn % k];
    if (!tx.probe || !lane.migrating ||
        tx.candidateGeneration != lane.candidateGeneration) return;
    if (++lane.stepAcks < lane.probeStep) return;
    lane.stepAcks = 0;
    if (lane.probeStep == 1) {
        CommitCandidate(psn % k);
    } else {
        --lane.probeStep;
        ++mappingEpoch;
        lane.boundaryPending = true;
    }
}

void PsnPathState::RecordLoss(uint32_t psn, PsnTransmission &tx) {
    if (tx.lossCounted || tx.ackCounted) return;
    ++pathStats[tx.pathId].nackCount;
    tx.lossCounted = true;
    const uint32_t laneId = psn % k;
    if (tx.probe && tx.candidateGeneration == lanes[laneId].candidateGeneration)
        AbortProbe(laneId);
}

void PsnPathState::RecordPathNack(const PsnPathNackRange &range, uint64_t ackSeq) {
    NS_ABORT_MSG_IF(range.k == 0 || range.k != k || range.startPsn > range.endPsn ||
                    range.startPsn % k != range.endPsn % k,
                    "Invalid PSN-PATH loss range");
    // Original transmission records disambiguate alternating migration stages.
    // Never expand the numeric interval into packets from other physical paths.
    for (auto it = transmissions.lower_bound(range.startPsn);
         it != transmissions.end() && it->first <= range.endPsn; ++it) {
        PsnTransmission &tx = it->second;
        if (it->first % k != range.startPsn % k || tx.pathId != range.pathId ||
            tx.ackCounted || uint64_t(it->first) * packetSize + tx.size <= ackSeq) continue;
        RecordLoss(it->first, tx);
        bool queued = false;
        for (const auto &request : pendingRetrans) {
            if (request.startPsn <= it->first && it->first <= request.endPsn) {
                queued = true;
                break;
            }
        }
        if (!queued) {
            PsnRetransRequest request;
            request.startPsn = request.endPsn = it->first;
            request.epoch = mappingEpoch;
            pendingRetrans.push_back(request);
        }
    }
}

void PsnPathState::RecordFeedback(uint32_t ackSeq, bool loss,
                                  uint32_t nackSeq, uint32_t nackSize,
                                  bool hasDelivery, uint32_t deliveredSeq, uint32_t deliveredSize) {
    if (packetSize == 0) return;
    // Count losses first: a NACK can also carry cumulative progress.
    if (loss && nackSize != 0) {
        const uint64_t end = uint64_t(nackSeq) + nackSize;
        for (auto it = transmissions.lower_bound(nackSeq / packetSize);
             it != transmissions.end() && uint64_t(it->first) * packetSize < end; ++it) {
            PsnTransmission &tx = it->second;
            RecordLoss(it->first, tx);
        }
    }
    if (hasDelivery) {
        auto it = transmissions.find(deliveredSeq / packetSize);
        if (it != transmissions.end() && it->second.size == deliveredSize)
            RecordSuccess(it->first, it->second);
    }
    for (auto it = transmissions.begin(); it != transmissions.end();) {
        PsnTransmission &tx = it->second;
        if (uint64_t(it->first) * packetSize + tx.size > ackSeq) break;
        RecordSuccess(it->first, tx);
        PsnLaneState &lane = lanes[it->first % k];
        // Only cumulative confirmation of the final boundary closes the
        // transition. Later steady packets cannot scan past it into an older
        // alternating schedule because the receiver has already admitted it.
        if (lane.phaseClosing && !lane.boundaryPending && it->first == lane.phaseStart)
            lane.phaseClosing = false;
        it = transmissions.erase(it);
    }
}

/**************************
 * RdmaQueuePair
 *************************/
TypeId RdmaQueuePair::GetTypeId(void) {
    static TypeId tid = TypeId("ns3::RdmaQueuePair").SetParent<Object>();
    return tid;
}

RdmaQueuePair::RdmaQueuePair(uint16_t pg, Ipv4Address _sip, Ipv4Address _dip, uint16_t _sport,
                             uint16_t _dport) {
    startTime = Simulator::Now();
    sip = _sip;
    dip = _dip;
    sport = _sport;
    dport = _dport;
    m_size = 0;
    snd_nxt = snd_una = 0;
    m_pg = pg;
    m_ipid = 0;
    m_win = 0;
    m_cc_mode = CC_MODE_UNDEFINED;
    m_baseRtt = 0;
    m_max_rate = 0;
    m_var_win = false;
    m_rate = 0;
    m_nextAvail = Time(0);
    mlx.m_alpha = 1;
    mlx.m_alpha_cnp_arrived = false;
    mlx.m_first_cnp = true;
    mlx.m_decrease_cnp_arrived = false;
    mlx.m_rpTimeStage = 0;
    hp.m_lastUpdateSeq = 0;
    hp.m_lastAckSeq = 0;
    hp.m_lastFastReactSeq = 0;
    hp.m_fastReactBudget = 0;
    hp.m_ownerPathSig = 0;
    hp.m_ownerPathEpoch = 0;
    hp.m_W = 1.0;
    hp.m_Wc = 1.0;
    hp.m_eta = 1.0;
    hp.m_WAI = 1.0;
    hp.m_baseRtt = 0.0;
    hp.m_targetUtil = 1.0;
    hp.m_incStage = 0;
    hp.m_maxStage = 0;
    hp.u = 1;
    for (uint32_t i = 0; i < IntHeader::maxHop; i++) {
        hp.lastTxBytes[i] = 0;
        hp.lastTimestamp[i] = 0;
    }

    tmly.m_lastUpdateSeq = 0;
    tmly.m_incStage = 0;
    tmly.lastRtt = 0;
    tmly.rttDiff = 0;

    dctcp.m_lastUpdateSeq = 0;
    dctcp.m_caState = 0;
    dctcp.m_highSeq = 0;
    dctcp.m_alpha = 1;
    dctcp.m_ecnCnt = 0;
    dctcp.m_batchSizeOfAlpha = 0;

    irn.m_enabled = false;
    irn.m_selectiveRetrans = true;
    irn.m_highest_ack = 0;
    irn.m_max_seq = 0;
    irn.m_recovery = false;
    irn.m_recovery_seq = 0;
    irn.m_retransmitFirst = false;
    irn.m_bdp = 0;

    m_timeout = MilliSeconds(4);
    m_selectiveAckedBytes = 0;
    m_falconSrttNs = 0;
    m_falconSrttValid = false;
}

void RdmaQueuePair::SetSize(uint64_t size) { m_size = size; }

void RdmaQueuePair::SetWin(uint32_t win) { m_win = win; }

void RdmaQueuePair::SetBaseRtt(uint64_t baseRtt) { m_baseRtt = baseRtt; }

void RdmaQueuePair::SetCcMode(uint32_t mode) { m_cc_mode = mode; }

void RdmaQueuePair::SetVarWin(bool v) { m_var_win = v; }

void RdmaQueuePair::SetFlowId(int32_t v) {
    m_flow_id = v;
    irn.m_sack.socketId = v;
}

void RdmaQueuePair::SetTimeout(Time v) { m_timeout = v; }

uint64_t RdmaQueuePair::GetBytesLeft() {
    PrepareIrnTransmission();
    return m_size >= snd_nxt ? m_size - snd_nxt : 0;
}

void RdmaQueuePair::PrepareIrnTransmission() {
    if (!irn.m_enabled || !irn.m_selectiveRetrans || psnPath.pathAwareRetransEnabled) return;
    uint32_t end;
    if (irn.m_sack.blockContaining(static_cast<uint32_t>(snd_nxt), &end)) {
        snd_nxt = end;
    }
    // Only the first retransmission is unconditional. Later holes require a
    // higher SACK; an unacknowledged tail may still be in flight.
    if (irn.m_recovery && !irn.m_retransmitFirst &&
        snd_nxt < irn.m_max_seq && snd_nxt >= irn.m_sack.highestSackEnd()) {
        snd_nxt = irn.m_max_seq;
    }
}

void RdmaQueuePair::StartIrnRecovery() {
    if (!irn.m_recovery) {
        // Byte ACKs are exclusive: ACK >= this boundary acknowledges the last
        // regular packet sent before recovery.
        irn.m_recovery_seq = irn.m_max_seq;
    }
    irn.m_recovery = true;
    irn.m_retransmitFirst = true;
    snd_nxt = snd_una;
}

uint32_t RdmaQueuePair::GetHash(void) {
    union {
        struct {
            uint32_t sip, dip;
            uint16_t sport, dport;
        };
        char c[12];
    } buf;
    buf.sip = sip.Get();
    buf.dip = dip.Get();
    buf.sport = sport;
    buf.dport = dport;
    return Hash32(buf.c, 12);
}

void RdmaQueuePair::Acknowledge(uint64_t ack) {
    if (ack > snd_una) {
        snd_una = ack;
    }
}

uint64_t RdmaQueuePair::GetOnTheFly() {
    NS_ASSERT(snd_nxt >= snd_una);
    uint64_t inflight = snd_nxt - snd_una;
    if (inflight <= m_selectiveAckedBytes) {
        return 0;
    }
    return inflight - m_selectiveAckedBytes;
}

bool RdmaQueuePair::IsWinBound() {
    uint64_t w = GetWin();
    return w != 0 && GetOnTheFly() >= w;
}

uint64_t RdmaQueuePair::GetWin() {
    if (m_cc_mode == CC_MODE_HPCC) {
        return (uint64_t)std::max(0.0, hp.m_W);
    }
    if (m_win == 0) return 0;
    uint64_t w;
    if (m_var_win) {
        w = m_win * m_rate.GetBitRate() / m_max_rate.GetBitRate();
        if (w == 0) w = 1;  // must > 0
    } else {
        w = m_win;
    }
    return w;
}

uint64_t RdmaQueuePair::HpGetCurWin() {
    if (m_win == 0) return 0;
    uint64_t w;
    if (m_var_win) {
        w = m_win * hp.m_curRate.GetBitRate() / m_max_rate.GetBitRate();
        if (w == 0) w = 1;  // must > 0
    } else {
        w = m_win;
    }
    return w;
}

bool RdmaQueuePair::IsFinished() {
    return snd_una >= m_size;
}

/*********************
 * RdmaRxQueuePair
 ********************/
TypeId RdmaRxQueuePair::GetTypeId(void) {
    static TypeId tid = TypeId("ns3::RdmaRxQueuePair").SetParent<Object>();
    return tid;
}

RdmaRxQueuePair::RdmaRxQueuePair() {
    sip = dip = sport = dport = 0;
    m_ipid = 0;
    ReceiverNextExpectedSeq = 0;
    m_nackTimer = Time(0);
    m_milestone_rx = 0;
    m_lastNACK = 0;
    m_lastNackReason = "";
    m_ackSeqOverrideValid = false;
    m_ackSeqOverride = 0;
    bitmapExpectedTimeoutSeq = 0xffffffffu;
    bitmapExpectedNackSent = false;
    if (bitmapExpectedTimerEvent.IsRunning()) {
        bitmapExpectedTimerEvent.Cancel();
    }
}

uint32_t RdmaRxQueuePair::GetHash(void) {
    union {
        struct {
            uint32_t sip, dip;
            uint16_t sport, dport;
        };
        char c[12];
    } buf;
    buf.sip = sip;
    buf.dip = dip;
    buf.sport = sport;
    buf.dport = dport;
    return Hash32(buf.c, 12);
}

/*********************
 * RdmaQueuePairGroup
 ********************/
TypeId RdmaQueuePairGroup::GetTypeId(void) {
    static TypeId tid = TypeId("ns3::RdmaQueuePairGroup").SetParent<Object>();
    return tid;
}

RdmaQueuePairGroup::RdmaQueuePairGroup(void) { memset(m_qp_finished, 0, sizeof(m_qp_finished)); }

uint32_t RdmaQueuePairGroup::GetN(void) { return m_qps.size(); }

Ptr<RdmaQueuePair> RdmaQueuePairGroup::Get(uint32_t idx) { return m_qps[idx]; }

Ptr<RdmaQueuePair> RdmaQueuePairGroup::operator[](uint32_t idx) { return m_qps[idx]; }

void RdmaQueuePairGroup::AddQp(Ptr<RdmaQueuePair> qp) { m_qps.push_back(qp); }

// void RdmaQueuePairGroup::AddRxQp(Ptr<RdmaRxQueuePair> rxQp){
// 	m_rxQps.push_back(rxQp);
// }

void RdmaQueuePairGroup::Clear(void) { m_qps.clear(); }

IrnSackManager::IrnSackManager() {}

IrnSackManager::IrnSackManager(int flow_id) { socketId = flow_id; }

std::ostream& operator<<(std::ostream& os, const IrnSackManager& im) {
    auto it = im.m_data.begin();
    for (; it != im.m_data.end(); ++it) {
        uint32_t blockBegin = it->first;             // inclusive
        uint32_t blockEnd = it->first + it->second;  // exclusive
        os << "[" << blockBegin << "-" << blockEnd << "] ";
    }
    return os;
}

// put blocks
void IrnSackManager::sack(uint32_t seq, uint32_t sz) {
    if (!sz) return;
    NS_LOG_LOGIC("Flow " << socketId << " : Inserting Block " << seq << "-" << (seq + sz));
    uint32_t seqEnd = seq + sz;  // exclusive

    auto it = m_data.begin();
    for (; it != m_data.end(); ++it) {
        uint32_t blockBegin = it->first;             // inclusive
        uint32_t blockEnd = it->first + it->second;  // exclusive

        if (blockBegin <= seq && seqEnd <= blockEnd) {
            // seq-seqEnd is included inside block-blockEnd
            return;
        } else if (seq < blockBegin && blockEnd < seqEnd) {
            // block-blockEnd is included inside Endseq-seqEnd
            // first segment : seq - blockBegin
            // second segment : blockEnd - seqEnd
            m_data.insert(it, std::pair<uint32_t, uint32_t>(seq, blockBegin - seq));
            NS_LOG_LOGIC("Flow " << socketId << " : Inserting Seg " << seq << "-" << blockBegin);
            seq = blockEnd;
            sz = seqEnd - blockEnd;
            seqEnd = seq + sz;
        } else if (seq < blockBegin && seqEnd <= blockBegin) {
            // seq-seqEnd is mutually exclusive to block-blockEnd, and smaller than block-blockEnd
            m_data.insert(it, std::pair<uint32_t, uint32_t>(seq, sz));
            NS_LOG_LOGIC("Flow " << socketId << " : Inserting Seg (Mutex)" << seq << "-"
                                 << (seq + sz));
            sz = 0;
            break;
        } else if (blockBegin <= seq && seq <= blockEnd && blockEnd < seqEnd) {
            // front part of seq-seqEnd is overlapped
            // new segment : blockEnd - seqEnd
            seq = blockEnd;
            sz = seqEnd - blockEnd;
        } else if (seq < blockBegin && blockBegin <= seqEnd && seqEnd <= blockEnd) {
            // rear part of seq-seqEnd is overlapped
            // new segment : seq - blockBegin
            m_data.insert(it, std::pair<uint32_t, uint32_t>(seq, blockBegin - seq));
            sz = 0;
            break;
        } else {
            NS_ASSERT(blockEnd <= seq);
        }
    }
    if (sz) {
        NS_LOG_LOGIC("Flow " << socketId << " : Inserting Seg (rem) " << seq << "-" << (seq + sz));
        m_data.insert(it, std::pair<uint32_t, uint32_t>(seq, sz));
    }
    NS_ASSERT(m_data.size() > 0);

    // Sanity check : check duplicate, empty blocks
    // merge neighboring blocks
    auto it_prev = m_data.begin();
    for (it = m_data.begin(); it != m_data.end();) {
        if (it == it_prev) {
            ++it;
            continue;
        }
        NS_ASSERT(it_prev->first + it_prev->second <= it->first);
        NS_ASSERT(it->second > 0);
        if (it_prev->first + it_prev->second == it->first) {
            // merge neighboring block
            NS_LOG_LOGIC("Flow " << socketId << " : Merging Block " << it_prev->first << "-"
                                 << (it_prev->first + it_prev->second) << " and " << it->first
                                 << "-" << (it->first + it->second));
            it_prev->second += it->second;
            it = m_data.erase(it);
        } else {
            it_prev = it;
            ++it;
        }
    }

    NS_LOG_LOGIC("Flow " << socketId << " : Blocks " << *this);
}

// put into return number of blocks removed
size_t IrnSackManager::discardUpTo(uint32_t cumAck) {
    auto it = m_data.begin();
    size_t erase_len = 0;
    for (; it != m_data.end();) {
        if (it->first + it->second <= cumAck) {
            NS_LOG_LOGIC("Flow " << socketId << " : Removing under " << cumAck
                                 << " - Removed Whole " << it->first << "-"
                                 << (it->first + it->second));
            erase_len += it->second;
            it = m_data.erase(it);
        } else if (it->first < cumAck) {  // do we need equal here? Maybe not
            NS_LOG_LOGIC("Flow " << socketId << " : Removing under " << cumAck
                                 << " - Removed Part  " << it->first << "-" << (cumAck)
                                 << " of Entire part " << it->first << "-"
                                 << (it->first + it->second));
            erase_len += cumAck - it->first;
            it->second = it->first + it->second - cumAck;
            it->first = cumAck;
            NS_ASSERT(it->second != 0);
            break;
        } else {
            break;
        }
    }
    return erase_len;
}

bool IrnSackManager::IsEmpty() { return !m_data.size(); }

bool IrnSackManager::blockExists(uint32_t seq, uint32_t size) {
    // query if block exists inside SACK table
    auto it = m_data.begin();
    for (; it != m_data.end(); ++it) {
        if (it->first <= seq && seq + size <= it->first + it->second) {
            return true;
        }
    }
    return false;
}
bool IrnSackManager::peekFrontBlock(uint32_t* pseq, uint32_t* psize) {
    NS_ASSERT(pseq);
    NS_ASSERT(psize);

    if (!m_data.size()) {
        *pseq = 0;
        *psize = 0;
        return false;
    }

    auto it = m_data.begin();
    *pseq = it->first;
    *psize = it->second;
    return true;
}

bool IrnSackManager::blockContaining(uint32_t seq, uint32_t *end) const {
    for (const auto &block : m_data) {
        if (block.first > seq) break;
        if (seq - block.first < block.second) {
            *end = block.first + block.second;
            return true;
        }
    }
    return false;
}

uint32_t IrnSackManager::highestSackEnd() const {
    return m_data.empty() ? 0 : m_data.back().first + m_data.back().second;
}

size_t IrnSackManager::getSackBufferOverhead() {
    size_t overhead = 0;
    auto it = m_data.begin();
    for (; it != m_data.end(); ++it) {
        overhead += it->second;  // Bytes
    }
    return overhead;
}

}  // namespace ns3
