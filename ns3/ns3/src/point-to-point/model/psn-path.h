#ifndef PSN_PATH_H
#define PSN_PATH_H

#include <cstdint>

#include "ns3/tag.h"
#include "ns3/type-id.h"

namespace ns3 {

class PsnPathTag : public Tag {
   public:
    PsnPathTag();
    static TypeId GetTypeId(void);
    virtual TypeId GetInstanceTypeId(void) const;
    virtual uint32_t GetSerializedSize(void) const;
    virtual void Serialize(TagBuffer i) const;
    virtual void Deserialize(TagBuffer i);
    virtual void Print(std::ostream &os) const;

    void SetEpoch(uint32_t v);
    uint32_t GetEpoch() const;
    void SetK(uint32_t v);
    uint32_t GetK() const;
    void SetO(uint32_t v);
    uint32_t GetO() const;
    void SetFpsn(uint32_t v);
    uint32_t GetFpsn() const;
    void SetActivePathMask(uint32_t v);
    uint32_t GetActivePathMask() const;
    void SetPathId(uint32_t v);
    uint32_t GetPathId() const;
    void SetProbe(uint8_t v);
    uint8_t GetProbe() const;
    void SetFlowSport(uint16_t v);
    uint16_t GetFlowSport() const;

   private:
    uint32_t m_epoch{0};
    uint32_t m_k{0};
    uint32_t m_o{0};
    uint32_t m_fpsn{0};
    uint32_t m_activePathMask{0};
    uint32_t m_pathId{0};
    uint8_t m_probe{0};
    uint16_t m_flowSport{0};
};

class PsnPath {
   public:
    static uint32_t GetPsnFromSeq(uint32_t seq, uint32_t packetSize);
    static uint32_t GetPathIdFromPsn(uint32_t psn, uint32_t k, uint32_t o,
                                     uint32_t activePathCount);
    static uint16_t GetSrcPortFromPsn(uint32_t psn, uint32_t k, uint32_t o, uint16_t basePort,
                                      uint32_t activePathCount);
    static bool IsSamePathLoss(uint32_t missingPsn, uint32_t currentPsn, uint32_t k,
                               uint32_t activePathCount);
};

}  // namespace ns3

#endif  // PSN_PATH_H
