#pragma once

#include <algorithm>
#include <cmath>

class GaussianDist {
public:
  GaussianDist() : mu_(0.0), sigma_(0.0) {}
  GaussianDist(double mu, double sigma)
      : mu_(mu), sigma_(std::max(sigma, 0.0)) {}

  double mean() const { return mu_; }
  double stddev() const { return sigma_; }
  double variance() const { return sigma_ * sigma_; }

  static double standardNormalCdf(double z) {
    return 0.5 * std::erfc(-z / std::sqrt(2.0));
  }

  double cdf(double x) const {
    if (sigma_ <= 0.0) {
      return (x >= mu_) ? 1.0 : 0.0; // degenerate / deterministic event
    }
    return standardNormalCdf((x - mu_) / sigma_);
  }

  struct Window {
    double lo;
    double hi;
    bool contains(double mu_d) const { return mu_d >= lo && mu_d <= hi; }
  };
  Window feasibleWindow(double walkTime, double k, double wmax) const {
    return Window{mu_ + walkTime - k * sigma_,
                  mu_ + walkTime + k * sigma_ + wmax};
  }

  static GaussianDist slack(const GaussianDist &a, const GaussianDist &d,
                            double rho, double walkTime) {
    double muDelta = d.mu_ - a.mu_ - walkTime;
    double cov = rho * a.sigma_ * d.sigma_;
    double varDelta = std::max(a.variance() + d.variance() - 2.0 * cov, 0.0);
    return GaussianDist(muDelta, std::sqrt(varDelta));
  }

  double feasibilityProbability() const {
    if (sigma_ <= 0.0) {
      return (mu_ >= 0.0) ? 1.0 : 0.0; // recovers classical deterministic test
    }
    return standardNormalCdf(mu_ / sigma_);
  }

  double robustBound(double k = 2.0) const { return mu_ + k * sigma_; }

  GaussianDist shifted(double c) const { return GaussianDist(mu_ + c, sigma_); }

private:
  double mu_;
  double sigma_;
};

inline double transferFeasibilityProbability(const GaussianDist &arrival,
                                             const GaussianDist &departure,
                                             double rho, double walkTime) {
  return GaussianDist::slack(arrival, departure, rho, walkTime)
      .feasibilityProbability();
}

struct DominanceKey {
  double m; // expected time, mu_e
  double b; // robustness bound, mu_e + k*sigma_e

  static DominanceKey fromEvent(const GaussianDist &e, double k = 2.0) {
    return DominanceKey{e.mean(), e.robustBound(k)};
  }

  bool dominates(const DominanceKey &other) const {
    bool leqBoth = (m <= other.m) && (b <= other.b);
    bool strict = (m < other.m) || (b < other.b);
    return leqBoth && strict;
  }
};

inline double ar1PropagatedVariance(double sigmaPrev2, double phi, double nu2) {
  return phi * phi * sigmaPrev2 + nu2;
}

inline double ar1Covariance(double sigmaJ2, double phiProduct) {
  return phiProduct * sigmaJ2;
}

inline double ar1Correlation(double covIJ, double sigmaI, double sigmaJ) {
  if (sigmaI <= 0.0 || sigmaJ <= 0.0)
    return 0.0;
  return covIJ / (sigmaI * sigmaJ);
}

using CorrelationKey = uint64_t;

constexpr CorrelationKey makeKey(StopEventId a, StopEventId b) noexcept {
  const auto x = static_cast<uint32_t>(std::min(a, b));
  const auto y = static_cast<uint32_t>(std::max(a, b));
  return (uint64_t{x} << 32) | y;
}
