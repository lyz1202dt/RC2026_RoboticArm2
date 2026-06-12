#pragma once

#include <Eigen/Dense>

#include <array>
#include <cstddef>

namespace arm_calc {

constexpr std::size_t kTrajectoryDim = 4;
using TrajectoryVector = Eigen::Matrix<double, static_cast<int>(kTrajectoryDim), 1>;

class TrajectoryCalc {
public:
    struct CubicPolynomial {
        double a0{0.0};
        double a1{0.0};
        double a2{0.0};
        double a3{0.0};
    };

    TrajectoryCalc() = default;

    void reset(const TrajectoryVector& start, const TrajectoryVector& goal, double duration);

    TrajectoryVector sample(double time_from_start) const;

    double duration() const { return duration_; }
    bool active(double time_from_start) const;
    bool is_ready() const { return ready_; }

private:
    static CubicPolynomial build_segment(double p0, double pf, double duration);
    static double eval_position(const CubicPolynomial& poly, double t);

    double duration_{0.0};
    bool ready_{false};
    TrajectoryVector start_position_{TrajectoryVector::Zero()};
    std::array<CubicPolynomial, kTrajectoryDim> segments_{};
};

}  // namespace arm_calc
