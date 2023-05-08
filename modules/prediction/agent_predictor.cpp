///
/// @copyright Copyright (C) 2021-2025, Gawei Xu
///
/// @author Gaowei Xu (gaowexu1991@gmail.com)
/// @date 2023-03-01
///

#include "modules/prediction/agent_predictor.h"

#include "modules/types/utils.h"

namespace perception
{
namespace object_fusion
{

AgentPredictor::AgentPredictor() {}

void AgentPredictor::Predict(const Clock::duration& duration,
                             BEVOFStateVector& state,
                             BEVOFStateCovarianceMatrix& covariance)
{
    // X_pred_ = F_ * X_;
    // P_pred_ = F_ * P_ * F_.transpose() + Q_;
    const auto& F = GetStateTransformMatrix(duration);
    const auto& Q = GetProcessNoiseCovarianceMatrix(duration);

    state = F * state;
    covariance = F * covariance * F.transpose() + Q;
}

BEVOFStateTransformMatrix AgentPredictor::GetStateTransformMatrix(const Clock::duration& duration)
{
    const auto delta_sec = static_cast<float>(ToSeconds(duration));
    const auto delta_sec_squa_half = delta_sec * delta_sec * 0.5F;

    BEVOFStateTransformMatrix F = BEVOFStateTransformMatrix::Identity();

    if (delta_sec <= 0.0F)
        return F;

    /// new position x
    F(static_cast<Eigen::Index>(BEVOFStateIndex::kX), static_cast<Eigen::Index>(BEVOFStateIndex::kVelocityX)) =
        delta_sec;
    F(static_cast<Eigen::Index>(BEVOFStateIndex::kX), static_cast<Eigen::Index>(BEVOFStateIndex::kAccelerationX)) =
        delta_sec_squa_half;

    /// new position y
    F(static_cast<Eigen::Index>(BEVOFStateIndex::kY), static_cast<Eigen::Index>(BEVOFStateIndex::kVelocityY)) =
        delta_sec;
    F(static_cast<Eigen::Index>(BEVOFStateIndex::kY), static_cast<Eigen::Index>(BEVOFStateIndex::kAccelerationY)) =
        delta_sec_squa_half;

    /// new orientation
    F(static_cast<Eigen::Index>(BEVOFStateIndex::kOrientation),
      static_cast<Eigen::Index>(BEVOFStateIndex::kOrientationRate)) = delta_sec;

    /// new velocity x
    F(static_cast<Eigen::Index>(BEVOFStateIndex::kVelocityX),
      static_cast<Eigen::Index>(BEVOFStateIndex::kAccelerationX)) = delta_sec;

    /// new velocity y
    F(static_cast<Eigen::Index>(BEVOFStateIndex::kVelocityY),
      static_cast<Eigen::Index>(BEVOFStateIndex::kAccelerationY)) = delta_sec;

    return F;
}

BEVOFProcessNoiseCovarianceMatrix AgentPredictor::GetProcessNoiseCovarianceMatrix(const Clock::duration& duration)
{
    const auto delta_sec = static_cast<float>(ToSeconds(duration));
    BEVOFProcessNoiseCovarianceMatrix Q = BEVOFProcessNoiseCovarianceMatrix::Identity();

    /// Q indicates the noise between motion predicted result and tracked result.
    /// Decrease process noise Q indicates that we trust the prediction model with a higher degree, i.e., trust the
    /// measurement result with a lower degree.
    Q(static_cast<Eigen::Index>(BEVOFStateIndex::kX), static_cast<Eigen::Index>(BEVOFStateIndex::kX)) =
        80.0F * delta_sec;
    Q(static_cast<Eigen::Index>(BEVOFStateIndex::kY), static_cast<Eigen::Index>(BEVOFStateIndex::kY)) =
        80.0F * delta_sec;
    Q(static_cast<Eigen::Index>(BEVOFStateIndex::kZ), static_cast<Eigen::Index>(BEVOFStateIndex::kZ)) =
        80.0F * delta_sec;
    Q(static_cast<Eigen::Index>(BEVOFStateIndex::kVelocityX), static_cast<Eigen::Index>(BEVOFStateIndex::kVelocityX)) =
        80.0F * delta_sec;
    Q(static_cast<Eigen::Index>(BEVOFStateIndex::kVelocityY), static_cast<Eigen::Index>(BEVOFStateIndex::kVelocityY)) =
        80.0F * delta_sec;
    Q(static_cast<Eigen::Index>(BEVOFStateIndex::kVelocityZ), static_cast<Eigen::Index>(BEVOFStateIndex::kVelocityZ)) =
        80.0F * delta_sec;
    Q(static_cast<Eigen::Index>(BEVOFStateIndex::kAccelerationX),
      static_cast<Eigen::Index>(BEVOFStateIndex::kAccelerationX)) = 80.0F * delta_sec;
    Q(static_cast<Eigen::Index>(BEVOFStateIndex::kAccelerationY),
      static_cast<Eigen::Index>(BEVOFStateIndex::kAccelerationY)) = 80.0F * delta_sec;
    Q(static_cast<Eigen::Index>(BEVOFStateIndex::kAccelerationZ),
      static_cast<Eigen::Index>(BEVOFStateIndex::kAccelerationZ)) = 80.0F * delta_sec;
    Q(static_cast<Eigen::Index>(BEVOFStateIndex::kOrientation),
      static_cast<Eigen::Index>(BEVOFStateIndex::kOrientation)) = 20.0F * delta_sec;
    Q(static_cast<Eigen::Index>(BEVOFStateIndex::kOrientationRate),
      static_cast<Eigen::Index>(BEVOFStateIndex::kOrientationRate)) = 20.0F * delta_sec;
    Q(static_cast<Eigen::Index>(BEVOFStateIndex::kLength), static_cast<Eigen::Index>(BEVOFStateIndex::kLength)) =
        20.0F * delta_sec;
    Q(static_cast<Eigen::Index>(BEVOFStateIndex::kWidth), static_cast<Eigen::Index>(BEVOFStateIndex::kWidth)) =
        20.0F * delta_sec;
    Q(static_cast<Eigen::Index>(BEVOFStateIndex::kHeight), static_cast<Eigen::Index>(BEVOFStateIndex::kHeight)) =
        20.0F * delta_sec;

    return Q;
}

}  // namespace object_fusion
}  // namespace perception