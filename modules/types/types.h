///
/// @copyright Copyright (C) 2021-2025, Gawei Xu
///
/// @author Gaowei Xu (gaowexu1991@gmail.com)
/// @date 2023-03-01
///

#ifndef MODULES_TYPES_TYPES_H
#define MODULES_TYPES_TYPES_H

#include <vector>

#include "modules/time/clock.h"

#include <Eigen/Dense>

namespace perception
{
namespace object_fusion
{

struct GenericCartesianVector2D
{
    float x;
    float y;
};

struct CartesianPose2D
{
    GenericCartesianVector2D position;
    float orientation;
};

enum class BEVDetClassification : std::uint8_t
{
    kCar = 0U,
    kBus = 1U,
    kTruck = 2U,
    kTrailer = 3U,
    kCombinationVehicle = 4U,
    kSpecialVehicle = 5U,
    kMicroTricycle = 6U,
    kMotorbike = 7U,
    kBicycle = 8U,
    kOtherVehicle = 9U,
    kPedestrian = 10U,
    kPoliceman = 11U,
    kBarrier = 12U,
    kSafetyBarrel = 13U,
    kTrafficCone = 14U,
    kOtherRoadblock = 15U,
    kTyre = 16U,
    kBabyCarriage = 17U,
    kWheelChair = 18U,
    kAnimal = 19U,
    kUnknown = 20U,
    kMax = 21U,
};

using BEVDetClassificationType = std::array<float, static_cast<std::uint8_t>(BEVDetClassification::kMax)>;

enum class BEVOFStateIndex : std::uint8_t
{
    kX = 0U,                 /// Position x in ego vehicle rear axis frame, unit: [m]
    kY = 1U,                 /// Position y in ego vehicle rear axis frame, unit: [m]
    kZ = 2U,                 /// Position z in ego vehicle rear axis frame, unit: [m]
    kVelocityX = 3U,         /// Velocity projected along x-axis in ego vehicle rear axis frame, unit: [m/s]
    kVelocityY = 4U,         /// Velocity projected along y-axis in ego vehicle rear axis frame, unit: [m/s]
    kVelocityZ = 5U,         /// Velocity projected along z-axis in ego vehicle rear axis frame, unit: [m/s]
    kAccelerationX = 6U,     /// Acceleration projected along x-axis in ego vehicle rear axis frame, unit: [m/s^2]
    kAccelerationY = 7U,     /// Acceleration projected along y-axis in ego vehicle rear axis frame, unit: [m/s^2]
    kAccelerationZ = 8U,     /// Acceleration projected along z-axis in ego vehicle rear axis frame, unit: [m/s^2]
    kOrientation = 9U,       /// Orientation in ego vehicle rear axis frame, unit: [rad]
    kOrientationRate = 10U,  /// Orientation rate in ego vehicle rear axis frame, unit: [rad/s]
    kLength = 11U,           /// Object length, unit: [m]
    kWidth = 12U,            /// Object width, unit: [m]
    kHeight = 13U,           /// Object height, unit: [m]
    kMax = 14U,              /// Estiated state dimension
};

constexpr std::uint32_t bev_of_state_dimen = static_cast<std::uint32_t>(BEVOFStateIndex::kMax);

/// Kalman Filter Formulation:
/// (a). X[t] = F * X[t-1] + B * u[t] + w[t-1]
/// (b). Z[t] = H * X[t] + v[t]
///
/// where,
/// X[t] - state vector's optimal estimation at timestamp t.
/// F[t] - state transformation matrix at timestamp t.
/// w[t] - process noise at timestamp t, its covariance matrix is Q.
/// Z[t] - state measurement at timestamp t.
/// H[t] - state measurement matrix at timestamp t.
/// v[t] - state measurement noise at timestamp t, its covariance matrix is R.
///
/// Prediction phase:
/// (1). X_pred[t] = F * X[t-1] + B * u[t]
/// (2). P_pred[t] = F * P[t-1] * F^T + Q[t]
///
/// Update phase:
/// (3). K[t] = P_pred[t] * H[t]^T * (H[t] * P_pred[t] * H[t]^T + R[t])^-1
/// (4). X[t] = X_pred[t] + K[t] * (Z[t] - H[t] * X_pred[t])
/// (5). P[t] = (I - K[t] * H[t]) * P_pred[t]
///
/// X_pred[t] - state vector's predicted value at timestamp t.
/// X[t]      - state vector's optimal estimation at timestamp t.
/// P_pred[t] - state covariance matrix between ground truth and predicted state vector at timestamp t.
/// P[t]      - state covariance matrix between ground truth and state vector's optimal estimation at timestamp t.
///
/// Attention: State measurement matrix H is with shape (observation_dimen, bev_of_state_dimen) where observation_dimen
/// is different for different modal sensor. Measurement noise covariance matrix R is with shape (observation_dimen,
/// observation_dimen).

using BEVOFStateVector = Eigen::Vector<float, bev_of_state_dimen>;                                       /// X
using BEVOFStateCovarianceMatrix = Eigen::Matrix<float, bev_of_state_dimen, bev_of_state_dimen>;         /// P
using BEVOFProcessNoiseCovarianceMatrix = Eigen::Matrix<float, bev_of_state_dimen, bev_of_state_dimen>;  /// Q
using BEVOFStateTransformMatrix = Eigen::Matrix<float, bev_of_state_dimen, bev_of_state_dimen>;          /// F
using BEVOFIdentityMatrix = Eigen::Matrix<float, bev_of_state_dimen, bev_of_state_dimen>;  /// Identity Matrix Dimension

enum class BEVOFTrackState : std::uint8_t
{
    kInvalid = 0U,
    kInitial = 1U,
    kPredicted = 2U,
    kFused = 3U,
    kLost = 4U,
};

struct BEVOFTrackObject
{
    /// @brief Tracked object's state.
    ///
    /// Optimal estimated state after Kalman prediction and update. These fields will be output for late fusion
    /// module.
    std::uint32_t id{1U};
    float x{0.0F};
    float y{0.0F};
    float z{0.0F};
    float velocity_x{0.0F};
    float velocity_y{0.0F};
    float velocity_z{0.0F};
    float acceleration_x{0.0F};
    float acceleration_y{0.0F};
    float acceleration_z{0.0F};
    float orientation{0.0F};
    float orientation_rate{0.0F};
    float length{0.0F};
    float width{0.0F};
    float height{0.0F};

    /// Optimal estimated existence and classification probabilities distribution.
    float existence{0.0F};
    BEVDetClassificationType classification{};

    /// Optimal estimation of state's uncertainty.
    float x_std{0.0F};
    float y_std{0.0F};
    float z_std{0.0F};
    float velocity_x_std{0.0F};
    float velocity_y_std{0.0F};
    float velocity_z_std{0.0F};
    float acceleration_x_std{0.0F};
    float acceleration_y_std{0.0F};
    float acceleration_z_std{0.0F};
    float orientation_std{0.0F};
    float orientation_rate_std{0.0F};
    float length_std{0.0F};
    float width_std{0.0F};
    float height_std{0.0F};

    /// @brief Kalman's state and its uncertainty, i.e., X and P.
    ///
    /// In Kalman prediction phase, predicted state will be updated and stored in variable `state_vector` (i.e.,
    /// X_pred_, state vector's predicted value), and the predicted state's covariance (P_pred_, i.e., state covariance
    /// matrix between ground truth and predicted state vector) matrix will be stored in variable `covariance_matrix`.
    ///
    /// X_pred_ = F_ * X_;
    /// P_pred_ = F_ * P_ * F_.transpose() + Q_;
    BEVOFStateVector state_vector{BEVOFStateVector::Zero()};
    BEVOFStateCovarianceMatrix covariance_matrix{BEVOFStateCovarianceMatrix::Zero()};

    /// @brief Timestamp of object's initialization.
    ///
    /// When the object was observed at the very beginning, this field should be filled if it was added as a new track
    /// in tracker manager.
    Clock::time_point create_time_stamp{};

    /// @brief Timestamp of latest fusion.
    ///
    /// This field should be updated in fusion phase (i.e., Kalman's update phase).
    Clock::time_point latest_fused_time_stamp{};

    /// @brief Tracked object's state.
    BEVOFTrackState state{BEVOFTrackState::kInvalid};
};

struct BEVOFTrackObjectList
{
    /// @brief Predicted timestamp.
    ///
    /// When an measurement list was observed, the latest timestmap will be exploited as target timestamp to which
    /// Kalman predicts all previous tracked objects. During Kalman prediction phase, this field will be updated.
    Clock::time_point time_stamp{};
    std::vector<BEVOFTrackObject> tracks{};
};

enum class BEVSensorType : std::uint8_t
{
    kBEVCamerasGroup = 0U,           /// BEV cameras only
    kBEVLidarsGroup = 1U,            /// BEV LiDARs only
    kBEVCamerasAndLidarsGroup = 2U,  /// BEV Pre-fusion
    kInvalid = 3U,
};

/// @brief BEVMeasurement structure defines the BEV LiDAR and BEV visual detections' output object.
///
/// Both BEV LiDAR and BEV visual detectors generate 3D bounding boxes which only contains fields: x, y, z, orientation,
/// length, width, height, category's probability.
struct BEVMeasurement
{
    float x{0.0F};                              /// Position x in ego vehicle rear axis frame, unit: [m]
    float y{0.0F};                              /// Position y in ego vehicle rear axis frame, unit: [m]
    float z{0.0F};                              /// Position z in ego vehicle rear axis frame, unit: [m]
    float orientation{0.0F};                    /// Orientation in ego vehicle rear axis frame, unit: [rad]
    float length{0.0F};                         /// Object length, unit: [m]
    float width{0.0F};                          /// Object width, unit: [m]
    float height{0.0F};                         /// Object height, unit: [m]
    float existence{0.0F};                      /// Object's existence probability, range from 0.0F to 1.0F.
    BEVDetClassificationType classification{};  /// Object's probabilities distribution.
};

enum class BEVMeasuredStateIndex : std::uint8_t
{
    kX = 0U,            /// Position x in ego vehicle rear axis frame, unit: [m]
    kY = 1U,            /// Position y in ego vehicle rear axis frame, unit: [m]
    kZ = 2U,            /// Position z in ego vehicle rear axis frame, unit: [m]
    kOrientation = 3U,  /// Orientation in ego vehicle rear axis frame, unit: [rad]
    kLength = 4U,       /// Object length, unit: [m]
    kWidth = 5U,        /// Object width, unit: [m]
    kHeight = 6U,       /// Object height, unit: [m]
    kMax = 7U,          /// Estiated state dimension
};

constexpr std::uint32_t bev_measurement_state_dimen = static_cast<std::uint32_t>(BEVMeasuredStateIndex::kMax);
using BEVMeasurementStateVector = Eigen::Vector<float, bev_measurement_state_dimen>;
using BEVMeasurementTransformMatrix = Eigen::Matrix<float, bev_measurement_state_dimen, bev_of_state_dimen>;
using BEVMeasurementCovarianceMatrix = Eigen::Matrix<float, bev_measurement_state_dimen, bev_measurement_state_dimen>;

/// @brief BEVMeasurement structure defines the 3D bounding boxes of each frame which detected by BEV LiDAR or BEV
/// visual module.
///
/// All detected objects in the same frame are with same detected timestamp.
struct BEVMeasurementList
{
    Clock::time_point detected_time_stamp{};             /// Timestamp of LiDAR point cloud or multi-images
    BEVSensorType sensor_type{BEVSensorType::kInvalid};  /// BEV LiDARs or BEV Cameras or BEVPrefision
    std::vector<BEVMeasurement> measurements{};          /// List of detected 3D objects
};

struct MatchedAgentIndexPair
{
    std::uint32_t measurement_idx;
    std::uint32_t track_idx;

    MatchedAgentIndexPair(const std::uint32_t measurement_idx, const std::uint32_t track_idx)
        : measurement_idx(measurement_idx), track_idx(track_idx)
    {
    }
};

struct AgentAssociatedIndices
{
    std::vector<MatchedAgentIndexPair> matched_indices;
    std::vector<std::uint32_t> unmatched_measurment_indices;
    std::vector<std::uint32_t> unmatched_track_indices;
};

}  // namespace object_fusion
}  // namespace perception

#endif  // MODULES_TYPES_TYPES_H
