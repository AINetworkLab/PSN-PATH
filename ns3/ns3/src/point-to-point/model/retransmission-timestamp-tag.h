#ifndef RETRANSMISSION_TIMESTAMP_TAG_H
#define RETRANSMISSION_TIMESTAMP_TAG_H

#include "ns3/tag.h"

namespace ns3 {

// Simulation metadata: independent of the INT/TS wire-header union and its
// congestion-control mode. Echoed by the receiver for retransmission RTT samples.
class RetransmissionTimestampTag : public Tag {
public:
    RetransmissionTimestampTag() : m_nanoseconds(0) {}
    static TypeId GetTypeId() {
        static TypeId tid = TypeId("ns3::RetransmissionTimestampTag")
            .SetParent<Tag>().AddConstructor<RetransmissionTimestampTag>();
        return tid;
    }
    virtual TypeId GetInstanceTypeId() const { return GetTypeId(); }
    virtual uint32_t GetSerializedSize() const { return 8; }
    virtual void Serialize(TagBuffer buffer) const { buffer.WriteU64(m_nanoseconds); }
    virtual void Deserialize(TagBuffer buffer) { m_nanoseconds = buffer.ReadU64(); }
    virtual void Print(std::ostream &os) const { os << m_nanoseconds; }
    void SetNanoseconds(uint64_t value) { m_nanoseconds = value; }
    uint64_t GetNanoseconds() const { return m_nanoseconds; }
private:
    uint64_t m_nanoseconds;
};

} // namespace ns3
#endif
