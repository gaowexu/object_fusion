///
/// @copyright Copyright (C) 2021-2025, Gawei Xu
///
/// @author Gaowei Xu (gaowexu1991@gmail.com)
/// @date 2023-03-01
///

#include "modules/fusion/agent_updater.h"

namespace perception
{
namespace object_fusion
{

AgentUpdater::AgentUpdater()
{
    H_ = BEVMeasurementTransformMatrix::Zero();

    H_(static_cast<Eigen::Index>(BEVMeasuredStateIndex::kX), static_cast<Eigen::Index>(BEVOFStateIndex::kX)) = 1.0F;
    H_(static_cast<Eigen::Index>(BEVMeasuredStateIndex::kY), static_cast<Eigen::Index>(BEVOFStateIndex::kY)) = 1.0F;
    H_(static_cast<Eigen::Index>(BEVMeasuredStateIndex::kZ), static_cast<Eigen::Index>(BEVOFStateIndex::kZ)) = 1.0F;
    H_(static_cast<Eigen::Index>(BEVMeasuredStateIndex::kOrientation),
       static_cast<Eigen::Index>(BEVOFStateIndex::kOrientation)) = 1.0F;
    H_(static_cast<Eigen::Index>(BEVMeasuredStateIndex::kLength), static_cast<Eigen::Index>(BEVOFStateIndex::kLength)) =
        1.0F;
    H_(static_cast<Eigen::Index>(BEVMeasuredStateIndex::kWidth), static_cast<Eigen::Index>(BEVOFStateIndex::kWidth)) =
        1.0F;
    H_(static_cast<Eigen::Index>(BEVMeasuredStateIndex::kHeight), static_cast<Eigen::Index>(BEVOFStateIndex::kHeight)) =
        1.0F;

    /// Decrease R will make the fused results more closer to the measurements. i.e., decreasing R means we trust the
    /// meausrement result with a higher degree.
    R_ = BEVMeasurementCovarianceMatrix::Zero();
    R_(static_cast<Eigen::Index>(BEVMeasuredStateIndex::kX), static_cast<Eigen::Index>(BEVMeasuredStateIndex::kX)) =
        1.0F;
    R_(static_cast<Eigen::Index>(BEVMeasuredStateIndex::kY), static_cast<Eigen::Index>(BEVMeasuredStateIndex::kY)) =
        1.0F;
    R_(static_cast<Eigen::Index>(BEVMeasuredStateIndex::kZ), static_cast<Eigen::Index>(BEVMeasuredStateIndex::kZ)) =
        1.0F;
    R_(static_cast<Eigen::Index>(BEVMeasuredStateIndex::kOrientation),
       static_cast<Eigen::Index>(BEVMeasuredStateIndex::kOrientation)) = 0.27415F;  /// (PI/6)^2
    R_(static_cast<Eigen::Index>(BEVMeasuredStateIndex::kLength),
       static_cast<Eigen::Index>(BEVMeasuredStateIndex::kLength)) = 2.50F;
    R_(static_cast<Eigen::Index>(BEVMeasuredStateIndex::kWidth),
       static_cast<Eigen::Index>(BEVMeasuredStateIndex::kWidth)) = 2.50F;
    R_(static_cast<Eigen::Index>(BEVMeasuredStateIndex::kHeight),
       static_cast<Eigen::Index>(BEVMeasuredStateIndex::kHeight)) = 1.0F;
}

void AgentUpdater::Fuse(const BEVMeasurement& measurement, const BEVSensorType& sensor_type, BEVOFTrackObject& track)
{
    // Step 0: Make sure that the orientation is continuous before fusion. For example, if track.orientation is 179
    // degrees and the measurement.orientation is -178 degrees, update track.orientation to -181 firstly.
    const float orientation_diff = std::abs(measurement.orientation - track.orientation);
    const float PI = 3.14159F;
    const float orien_std_dev_half = 0.25F * PI;

    if (measurement.orientation * track.orientation < 0.0F &&
        orientation_diff > (2.0F * PI - 2.0F * orien_std_dev_half))
    {
        const auto delta = track.orientation > 0.0F ? -2.0F * PI : 2.0F * PI;
        track.state_vector(static_cast<Eigen::Index>(BEVOFStateIndex::kOrientation)) += delta;
    }

    const auto H_t = H_.transpose();                                        /// dimension is 14x7
    const auto I = BEVOFIdentityMatrix::Identity();                         /// dimension is 14x14
    const BEVOFStateVector X_pred = track.state_vector;                     /// dimension is 14x1
    const BEVOFStateCovarianceMatrix P_pred = track.covariance_matrix;      /// dimension is 14x14
    BEVMeasurementStateVector Z = BEVMeasurementStateVector{measurement.x,  /// dimension is 7x1
                                                            measurement.y,
                                                            measurement.z,
                                                            measurement.orientation,
                                                            measurement.length,
                                                            measurement.width,
                                                            measurement.height};

    /// TODO: use measurement's uncertainty to update R_ if measurement's uncertainty fields are valid,
    /// otherwise, use default R_.

    /// Step 1: Kalman update phase.
    const auto K = P_pred * H_t * (H_ * P_pred * H_t + R_).inverse();  /// Kalman gain, dimension is 14x7
    const auto X = X_pred + K * (Z - H_ * X_pred);                     /// dimension is 14x1
    const auto P = (I - K * H_) * P_pred;

    track.state_vector = X;
    track.covariance_matrix = P;

    /// Step 2: Fuse `existence` with latest observation `measurement`.
    track.existence = FuseExistence(track.existence, measurement.existence);

    /// Step 3: Fuse `classification` with latest observation `measurement`.
    track.classification = FuseClassificationVector(track.classification, measurement.classification);

    /// Step 4: Assign optimal estimation to final fields.
    track.x = track.state_vector(static_cast<Eigen::Index>(BEVOFStateIndex::kX));
    track.y = track.state_vector(static_cast<Eigen::Index>(BEVOFStateIndex::kY));
    track.z = track.state_vector(static_cast<Eigen::Index>(BEVOFStateIndex::kZ));
    track.velocity_x = track.state_vector(static_cast<Eigen::Index>(BEVOFStateIndex::kVelocityX));
    track.velocity_y = track.state_vector(static_cast<Eigen::Index>(BEVOFStateIndex::kVelocityY));
    track.velocity_z = track.state_vector(static_cast<Eigen::Index>(BEVOFStateIndex::kVelocityZ));
    track.acceleration_x = track.state_vector(static_cast<Eigen::Index>(BEVOFStateIndex::kAccelerationX));
    track.acceleration_y = track.state_vector(static_cast<Eigen::Index>(BEVOFStateIndex::kAccelerationY));
    track.acceleration_z = track.state_vector(static_cast<Eigen::Index>(BEVOFStateIndex::kAccelerationZ));
    track.orientation = track.state_vector(static_cast<Eigen::Index>(BEVOFStateIndex::kOrientation));
    track.orientation_rate = track.state_vector(static_cast<Eigen::Index>(BEVOFStateIndex::kOrientationRate));
    track.length = track.state_vector(static_cast<Eigen::Index>(BEVOFStateIndex::kLength));
    track.width = track.state_vector(static_cast<Eigen::Index>(BEVOFStateIndex::kWidth));
    track.height = track.state_vector(static_cast<Eigen::Index>(BEVOFStateIndex::kHeight));

    track.x_std = track.covariance_matrix(static_cast<Eigen::Index>(BEVOFStateIndex::kX),
                                          static_cast<Eigen::Index>(BEVOFStateIndex::kX));
    track.y_std = track.covariance_matrix(static_cast<Eigen::Index>(BEVOFStateIndex::kY),
                                          static_cast<Eigen::Index>(BEVOFStateIndex::kY));
    track.z_std = track.covariance_matrix(static_cast<Eigen::Index>(BEVOFStateIndex::kZ),
                                          static_cast<Eigen::Index>(BEVOFStateIndex::kZ));
    track.velocity_x_std = track.covariance_matrix(static_cast<Eigen::Index>(BEVOFStateIndex::kVelocityX),
                                                   static_cast<Eigen::Index>(BEVOFStateIndex::kVelocityX));
    track.velocity_y_std = track.covariance_matrix(static_cast<Eigen::Index>(BEVOFStateIndex::kVelocityY),
                                                   static_cast<Eigen::Index>(BEVOFStateIndex::kVelocityY));
    track.velocity_z_std = track.covariance_matrix(static_cast<Eigen::Index>(BEVOFStateIndex::kVelocityZ),
                                                   static_cast<Eigen::Index>(BEVOFStateIndex::kVelocityZ));
    track.acceleration_x_std = track.covariance_matrix(static_cast<Eigen::Index>(BEVOFStateIndex::kAccelerationX),
                                                       static_cast<Eigen::Index>(BEVOFStateIndex::kAccelerationX));
    track.acceleration_y_std = track.covariance_matrix(static_cast<Eigen::Index>(BEVOFStateIndex::kAccelerationY),
                                                       static_cast<Eigen::Index>(BEVOFStateIndex::kAccelerationY));
    track.acceleration_z_std = track.covariance_matrix(static_cast<Eigen::Index>(BEVOFStateIndex::kAccelerationZ),
                                                       static_cast<Eigen::Index>(BEVOFStateIndex::kAccelerationZ));
    track.orientation_std = track.covariance_matrix(static_cast<Eigen::Index>(BEVOFStateIndex::kOrientation),
                                                    static_cast<Eigen::Index>(BEVOFStateIndex::kOrientation));
    track.orientation_rate_std = track.covariance_matrix(static_cast<Eigen::Index>(BEVOFStateIndex::kOrientationRate),
                                                         static_cast<Eigen::Index>(BEVOFStateIndex::kOrientationRate));
    track.length_std = track.covariance_matrix(static_cast<Eigen::Index>(BEVOFStateIndex::kLength),
                                               static_cast<Eigen::Index>(BEVOFStateIndex::kLength));
    track.width_std = track.covariance_matrix(static_cast<Eigen::Index>(BEVOFStateIndex::kWidth),
                                              static_cast<Eigen::Index>(BEVOFStateIndex::kWidth));
    track.height_std = track.covariance_matrix(static_cast<Eigen::Index>(BEVOFStateIndex::kHeight),
                                               static_cast<Eigen::Index>(BEVOFStateIndex::kHeight));
}

float AgentUpdater::FuseExistence(const float a, const float b)
{
    return (a * b) / (1e-5F + a * b + (1.0F - a) * (1.0F - b));
}

BEVDetClassificationType AgentUpdater::FuseClassificationVector(const BEVDetClassificationType& a,
                                                                const BEVDetClassificationType& b)
{
    BEVDetClassificationType prob_dist;
    float sum = 0.0F;

    for (size_t i = 0; i < a.size(); ++i)
    {
        prob_dist[i] = a[i] * b[i];
        sum += prob_dist[i];
    }

    for (size_t i = 0; i < a.size(); ++i)
    {
        prob_dist[i] /= (sum + 1e-5F);
    }

    return prob_dist;
}

}  // namespace object_fusion
}  // namespace perception