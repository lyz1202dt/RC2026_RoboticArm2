#include "arm_calc/trajectory_calc.hpp"

#include <algorithm>
#include <cmath>

namespace arm_calc {

void TrajectoryCalc::reset(const TrajectoryVector& start, const TrajectoryVector& goal, double duration) {
    start_position_ = start;
    duration_ = std::max(duration, 1e-3);

    for (std::size_t i = 0; i < kTrajectoryDim; ++i) {
        segments_[i] = build_segment(start_position_[static_cast<int>(i)],
                                     goal[static_cast<int>(i)],
                                     duration_);
    }
    ready_ = true;
}

TrajectoryVector TrajectoryCalc::sample(double time_from_start) const {
    if (!ready_) {
        return start_position_;
    }

    const double t = std::clamp(time_from_start, 0.0, duration_);
    TrajectoryVector position = TrajectoryVector::Zero();
    for (std::size_t i = 0; i < kTrajectoryDim; ++i) {
        const auto& poly = segments_[i];
        position[static_cast<int>(i)] = eval_position(poly, t);
    }
    return position;
}

bool TrajectoryCalc::active(double time_from_start) const {
    return ready_ && time_from_start <= duration_;
}

TrajectoryCalc::CubicPolynomial TrajectoryCalc::build_segment(double p0, double pf, double duration) {
    const double t = std::max(duration, 1e-3);
    const double t2 = t * t;
    const double t3 = t2 * t;

    CubicPolynomial poly;
    poly.a0 = p0;
    poly.a1 = 0.0;
    poly.a2 = 3.0 * (pf - p0) / t2;
    poly.a3 = -2.0 * (pf - p0) / t3;
    return poly;
}

double TrajectoryCalc::eval_position(const CubicPolynomial& poly, double t) {
    return poly.a0 + poly.a1 * t + poly.a2 * t * t + poly.a3 * t * t * t;
}

}  // namespace arm_calc
