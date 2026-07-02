#include "psn-path.h"

namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED(PsnPathTag);

PsnPathTag::PsnPathTag() : Tag() {}

TypeId PsnPathTag::GetTypeId(void) {
    static TypeId tid = TypeId("ns3::PsnPathTag").SetParent<Tag>().AddConstructor<PsnPathTag>();
    return tid;
}

TypeId PsnPathTag::GetInstanceTypeId(void) const { return GetTypeId(); }

uint32_t PsnPathTag::GetSerializedSize(void) const {
    return sizeof(m_epoch) + sizeof(m_k) + sizeof(m_o) + sizeof(m_fpsn) +
           sizeof(m_activePathMask) + sizeof(m_pathId) + sizeof(m_probe) +
           sizeof(m_flowSport);
}

void PsnPathTag::Serialize(TagBuffer i) const {
    i.WriteU32(m_epoch);
    i.WriteU32(m_k);
    i.WriteU32(m_o);
    i.WriteU32(m_fpsn);
    i.WriteU32(m_activePathMask);
    i.WriteU32(m_pathId);
    i.WriteU8(m_probe);
    i.WriteU16(m_flowSport);
}

void PsnPathTag::Deserialize(TagBuffer i) {
    m_epoch = i.ReadU32();
    m_k = i.ReadU32();
    m_o = i.ReadU32();
    m_fpsn = i.ReadU32();
    m_activePathMask = i.ReadU32();
    m_pathId = i.ReadU32();
    m_probe = i.ReadU8();
    m_flowSport = i.ReadU16();
}

void PsnPathTag::Print(std::ostream &os) const {
    os << "epoch=" << m_epoch << ",k=" << m_k << ",o=" << m_o << ",fpsn=" << m_fpsn
       << ",activePathMask=" << m_activePathMask << ",pathId=" << m_pathId
       << ",probe=" << (uint32_t)m_probe
       << ",flowSport=" << m_flowSport;
}

void PsnPathTag::SetEpoch(uint32_t v) { m_epoch = v; }
uint32_t PsnPathTag::GetEpoch() const { return m_epoch; }
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
void PsnPathTag::SetProbe(uint8_t v) { m_probe = v; }
uint8_t PsnPathTag::GetProbe() const { return m_probe; }
void PsnPathTag::SetFlowSport(uint16_t v) { m_flowSport = v; }
uint16_t PsnPathTag::GetFlowSport() const { return m_flowSport; }

uint32_t PsnPath::GetPsnFromSeq(uint32_t seq, uint32_t packetSize) {
    if (packetSize == 0) return 0;
    return seq / packetSize;
}

uint32_t PsnPath::GetPathIdFromPsn(uint32_t psn, uint32_t k, uint32_t o, uint32_t activePathCount) {
    if (activePathCount == 0) return 0;
    if (k == 0) return o % activePathCount;
    return (o + (psn % k)) % activePathCount;
}

uint16_t PsnPath::GetSrcPortFromPsn(uint32_t psn, uint32_t k, uint32_t o, uint16_t basePort,
                                    uint32_t activePathCount) {
    uint32_t pathId = GetPathIdFromPsn(psn, k, o, activePathCount);
    return static_cast<uint16_t>(basePort + pathId);
}

bool PsnPath::IsSamePathLoss(uint32_t missingPsn, uint32_t currentPsn, uint32_t k,
                             uint32_t activePathCount) {
    if (k == 0 || activePathCount == 0) return false;
    uint32_t missingPath = missingPsn % k;
    uint32_t currentPath = currentPsn % k;
    if (activePathCount < k) {
        missingPath %= activePathCount;
        currentPath %= activePathCount;
    }
    return missingPath == currentPath;
}

}  // namespace ns3
