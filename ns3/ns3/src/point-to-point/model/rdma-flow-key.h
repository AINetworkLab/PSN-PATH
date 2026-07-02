#ifndef RDMA_FLOW_KEY_H
#define RDMA_FLOW_KEY_H

#include <cstddef>
#include <cstdint>

namespace ns3 {

struct RdmaFlowKey {
    uint32_t ip{0};
    uint16_t sport{0};
    uint16_t dport{0};
    uint16_t pg{0};

    bool operator==(const RdmaFlowKey &other) const {
        return ip == other.ip && sport == other.sport && dport == other.dport && pg == other.pg;
    }

    bool operator<(const RdmaFlowKey &other) const {
        if (ip != other.ip) return ip < other.ip;
        if (sport != other.sport) return sport < other.sport;
        if (dport != other.dport) return dport < other.dport;
        return pg < other.pg;
    }
};

struct RdmaFlowKeyHash {
    std::size_t operator()(const RdmaFlowKey &key) const {
        uint64_t h = 1469598103934665603ull;
        Mix(h, key.ip);
        Mix(h, key.sport);
        Mix(h, key.dport);
        Mix(h, key.pg);
        return static_cast<std::size_t>(h);
    }

   private:
    static void Mix(uint64_t &h, uint64_t value) {
        h ^= value;
        h *= 1099511628211ull;
    }
};

}  // namespace ns3

#endif
