#ifndef PSN_PATH_SCORE_H
#define PSN_PATH_SCORE_H

#include <cmath>
#include <cstdint>
#include <limits>

namespace ns3 {

// Shared by feedback accounting and periodic path evaluation.
class PsnPathScore {
public:
    enum Action { Keep, Remove, Probe };

    static double UnsetThreshold() {
        // A finite sentinel is accepted by ns-3's Double attribute checker.
        return std::numeric_limits<double>::max();
    }

    static bool ValidThresholds(double beta, double explore, double remove) {
        return std::isfinite(beta) && beta >= 0.0 &&
               std::isfinite(explore) && std::isfinite(remove) &&
               remove >= -beta && remove <= explore && explore <= 1.0;
    }

    static bool TryNormalize(uint64_t ackCount, uint64_t nackCount,
                             double beta, double &score) {
        if (ackCount == 0 && nackCount == 0) return false;
        const double ack = static_cast<double>(ackCount);
        const double nack = static_cast<double>(nackCount);
        const double total = ack + nack;
        // Divide before weighting to avoid integer addition / beta*NACK overflow.
        score = ack / total - beta * (nack / total);
        return true;
    }

    static Action Classify(double score, double explore, double remove,
                           bool canRemove, bool hasInactivePath) {
        if (score < remove && canRemove) return Remove;
        if (score < explore && hasInactivePath) return Probe;
        return Keep;
    }
};

}  // namespace ns3

#endif  // PSN_PATH_SCORE_H
