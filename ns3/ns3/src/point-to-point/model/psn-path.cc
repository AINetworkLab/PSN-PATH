#include "psn-path.h"
#include "ns3/abort.h"

namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED(PsnPathTag);

PsnPathTag::PsnPathTag() : Tag() {}

TypeId PsnPathTag::GetTypeId(void) {
    static TypeId tid = TypeId("ns3::PsnPathTag").SetParent<Tag>().AddConstructor<PsnPathTag>();
    return tid;
}

TypeId PsnPathTag::GetInstanceTypeId(void) const { return GetTypeId(); }

uint32_t PsnPathTag::GetSerializedSize(void) const {
    return sizeof(m_k) + sizeof(m_o) + sizeof(m_fpsn) +
           sizeof(m_activePathMask) + sizeof(m_pathId) + sizeof(m_probe) +
           sizeof(m_flowSport) + (HasPhaseInfo() ?
           sizeof(m_phaseStart) + sizeof(m_explore) + sizeof(m_candidateOffset) : 0);
}

void PsnPathTag::Serialize(TagBuffer i) const {
    i.WriteU32(m_k);
    i.WriteU32(m_o);
    i.WriteU32(m_fpsn);
    i.WriteU32(m_activePathMask);
    i.WriteU32(m_pathId);
    i.WriteU8(m_probe);
    i.WriteU16(m_flowSport);
    if (HasPhaseInfo()) {
        i.WriteU32(m_phaseStart);
        i.WriteU8(m_explore);
        i.WriteU8(m_candidateOffset);
    }
}

void PsnPathTag::Deserialize(TagBuffer i) {
    m_k = i.ReadU32();
    m_o = i.ReadU32();
    m_fpsn = i.ReadU32();
    m_activePathMask = i.ReadU32();
    m_pathId = i.ReadU32();
    m_probe = i.ReadU8();
    m_flowSport = i.ReadU16();
    m_phaseStart = 0;
    m_explore = 0;
    m_candidateOffset = 0;
    if (HasPhaseInfo()) {
        m_phaseStart = i.ReadU32();
        m_explore = i.ReadU8();
        m_candidateOffset = i.ReadU8();
    }
}

void PsnPathTag::Print(std::ostream &os) const {
    os << "hasPhaseInfo=" << HasPhaseInfo() << ",phaseStart=" << m_phaseStart
       << ",k=" << m_k << ",o=" << m_o << ",fpsn=" << m_fpsn
       << ",activePathMask=" << m_activePathMask << ",pathId=" << m_pathId
       << ",probe=" << (uint32_t)GetProbe()
       << ",retransmission=" << IsRetransmission()
       << ",step=" << GetProbeStep() << ",candidateOffset=" << GetCandidateOffset()
       << ",flowSport=" << m_flowSport;
}

void PsnPathTag::SetPhaseStart(uint32_t v) {
    m_phaseStart = v;
    m_probe |= 4;
}
bool PsnPathTag::HasPhaseInfo() const { return (m_probe & 4) != 0; }
uint32_t PsnPathTag::GetPhaseStart() const { return m_phaseStart; }
uint32_t PsnPathTag::GetControlBytes() const {
    return PsnPath::kControlBytes + (HasPhaseInfo() ?
        PsnPath::kExploreBytes + PsnPath::kPhaseStartBytes : 0);
}
void PsnPathTag::SetK(uint32_t v) { m_k = v; }
uint32_t PsnPathTag::GetK() const { return m_k; }
void PsnPathTag::SetO(uint32_t v) { m_o = v; }
uint32_t PsnPathTag::GetO() const { return m_o; }
void PsnPathTag::SetFpsn(uint32_t v) { m_fpsn = v; }
uint32_t PsnPathTag::GetFpsn() const { return m_fpsn; }
void PsnPathTag::SetActivePathMask(uint32_t v) { m_activePathMask = v; }
uint32_t PsnPathTag::GetActivePathMask() const { return m_activePathMask; }
void PsnPathTag::SetPathId(uint32_t v) { m_pathId = v; }
uint32_t PsnPathTag::GetPathId() const { return m_pathId; }
void PsnPathTag::SetProbe(uint8_t v) { m_probe = (m_probe & ~uint8_t(1)) | (v ? 1 : 0); }
uint8_t PsnPathTag::GetProbe() const { return m_probe & 1; }
void PsnPathTag::SetRetransmission(bool v) {
    m_probe = (m_probe & ~uint8_t(2)) | (v ? 2 : 0);
}
bool PsnPathTag::IsRetransmission() const { return (m_probe & 2) != 0; }
void PsnPathTag::SetExplore(uint32_t step, uint32_t candidateOffset) {
    NS_ABORT_MSG_IF(step > 0x7f || (step != 0 && candidateOffset > UINT8_MAX),
                    "PSN-PATH Explore metadata exceeds its field widths");
    m_explore = step == 0 ? 0 : uint8_t(0x80 | step);
    m_candidateOffset = step == 0 ? 0 : uint8_t(candidateOffset);
    if (step != 0) m_probe |= 4;
}
uint32_t PsnPathTag::GetProbeStep() const { return (m_explore & 0x80) ? m_explore & 0x7f : 0; }
uint32_t PsnPathTag::GetCandidateOffset() const { return m_candidateOffset; }
void PsnPathTag::SetFlowSport(uint16_t v) { m_flowSport = v; }
uint16_t PsnPathTag::GetFlowSport() const { return m_flowSport; }

uint32_t PsnPath::GetPsnFromSeq(uint32_t seq, uint32_t packetSize) {
    if (packetSize == 0) return 0;
    return seq / packetSize;
}

uint32_t PsnPath::GetPathIdFromPsn(uint32_t psn, uint32_t k, uint32_t o, uint32_t activePathCount) {
    if (activePathCount == 0) return 0;
    if (k == 0) return 0;
    return o + psn % k;
}

uint16_t PsnPath::GetSrcPortFromPsn(uint32_t psn, uint32_t k, uint32_t o, uint16_t basePort,
                                    uint32_t activePathCount) {
    uint32_t pathId = GetPathIdFromPsn(psn, k, o, activePathCount);
    return static_cast<uint16_t>(basePort + pathId);
}

bool PsnPath::IsSamePathLoss(uint32_t missingPsn, uint32_t currentPsn, uint32_t k,
                             uint32_t activePathCount) {
    if (k == 0 || activePathCount == 0) return false;
    return missingPsn % k == currentPsn % k;
}

}  // namespace ns3
