///
/// @copyright Copyright (C) 2021-2025, Gawei Xu
///
/// @author Gaowei Xu (gaowexu1991@gmail.com)
/// @date 2023-03-01
///

#include "modules/object_fusion.h"

#include "modules/types/utils.h"

namespace perception
{
namespace object_fusion
{

BEVObjectsFusion::BEVObjectsFusion(const float duration_thresh_for_agent_removal,
                                   const float iou_distance_weight,
                                   const float euclidean_distance_weight,
                                   const float association_iou_gate,
                                   const float association_euclidean_dist_gate)
    : agent_id_(1U), duration_thresh_for_agent_removal_(duration_thresh_for_agent_removal)
{
    agent_predictor_ = std::make_unique<AgentPredictor>();
    agent_associator_ = std::make_unique<AgentAssociator>(
        iou_distance_weight, euclidean_distance_weight, association_iou_gate, association_euclidean_dist_gate);
    agent_fuser_ = std::make_unique<AgentUpdater>();
}

BEVObjectsFusion::~BEVObjectsFusion()
{
    agent_predictor_.reset();
    agent_associator_.reset();
    agent_fuser_.reset();
}

BEVOFTrackObjectList BEVObjectsFusion::Run(const BEVMeasurementList& measurements_list,
                                           const CartesianPose2D& ego_pose_in_odometry)
{
    const BEVMeasurementList measurements_list_in_odometry =
        TransformMeasurementsFromEgoVehicleRearAxisToOdometry(measurements_list, ego_pose_in_odometry);

    KalmanTrackAndFuse(measurements_list_in_odometry);

    const BEVOFTrackObjectList tracks_in_ego_vehicle_rear_axis =
        TransfromTracksFromOdometryToEgoVehicleRearAxis(tracks_, ego_pose_in_odometry);

    return tracks_in_ego_vehicle_rear_axis;
}

void BEVObjectsFusion::KalmanTrackAndFuse(const BEVMeasurementList& measurements_list)
{
    /// Step 1: Predict previous tracks to current timestamp (i.e., measurements_list.detected_time_stamp)
    ///
    /// After this step, the field `state_vector` and `covariance_matrix` of each tracked object will be updated
    /// according to the prediction model.
    const Clock::duration duration = measurements_list.detected_time_stamp - tracks_.time_stamp;

    for (size_t tracked_object_idx = 0; tracked_object_idx < tracks_.tracks.size(); ++tracked_object_idx)
    {
        if (duration.count() > 0)
        {
            agent_predictor_->Predict(duration,
                                      tracks_.tracks[tracked_object_idx].state_vector,
                                      tracks_.tracks[tracked_object_idx].covariance_matrix);

            tracks_.tracks[tracked_object_idx].state = BEVOFTrackState::kPredicted;
        }
    }
    tracks_.time_stamp = measurements_list.detected_time_stamp;

    /// Step 2: Association. The association is expected to be built between `state_vector` and `covariance_matrix` of
    /// tracks and measurement list.
    ///
    /// The return result `asso_info` contains three aspects: (1). matched_indices (2). unmatched_track_indices and (3).
    /// unmatched_measurment_indices.
    const AgentAssociatedIndices asso_info = agent_associator_->Associate(measurements_list, tracks_);

    /// Step 3: Fuse associated tracks with latest measurements. The Kalman filter will update `state_vector` and
    /// `covariance_matrix` fields of tracks.
    ///
    /// Fuse tracks which are associated with latest measurements.
    for (auto match_idx_pair : asso_info.matched_indices)
    {
        agent_fuser_->Fuse(measurements_list.measurements[match_idx_pair.measurement_idx],
                           BEVSensorType::kInvalid,
                           tracks_.tracks[match_idx_pair.track_idx]);

        tracks_.tracks[match_idx_pair.track_idx].latest_fused_time_stamp = measurements_list.detected_time_stamp;
        tracks_.tracks[match_idx_pair.track_idx].state = BEVOFTrackState::kFused;
    }

    /// Step 4: Remove tracks which are not associated with measurements for a duration, such as 300 ms.
    for (auto track_idx : asso_info.unmatched_track_indices)
    {
        const Clock::duration duration =
            measurements_list.detected_time_stamp - tracks_.tracks[track_idx].latest_fused_time_stamp;

        const auto lost_duration_in_secs = static_cast<float>(ToSeconds(duration));

        if (lost_duration_in_secs > duration_thresh_for_agent_removal_)
        {
            tracks_.tracks[track_idx].state = BEVOFTrackState::kLost;
        }
    }

    const auto it = std::remove_if(tracks_.tracks.begin(), tracks_.tracks.end(), [this](const auto& track) {
        return (track.state == BEVOFTrackState::kLost);
    });
    tracks_.tracks.erase(it, tracks_.tracks.end());

    /// Step 5: Add new measurements into tracks.
    for (auto measurement_idx : asso_info.unmatched_measurment_indices)
    {
        BEVOFTrackObject obj;

        obj.id = agent_id_;
        agent_id_ += 1U;  /// increase global agent id

        obj.x = measurements_list.measurements[measurement_idx].x;
        obj.y = measurements_list.measurements[measurement_idx].y;
        obj.z = measurements_list.measurements[measurement_idx].z;
        obj.orientation = measurements_list.measurements[measurement_idx].orientation;
        obj.length = measurements_list.measurements[measurement_idx].length;
        obj.width = measurements_list.measurements[measurement_idx].width;
        obj.height = measurements_list.measurements[measurement_idx].height;
        obj.existence = measurements_list.measurements[measurement_idx].existence;
        obj.classification = measurements_list.measurements[measurement_idx].classification;
        obj.create_time_stamp = measurements_list.detected_time_stamp;
        obj.state = BEVOFTrackState::kInitial;

        obj.state_vector(static_cast<Eigen::Index>(BEVOFStateIndex::kX)) =
            measurements_list.measurements[measurement_idx].x;
        obj.state_vector(static_cast<Eigen::Index>(BEVOFStateIndex::kY)) =
            measurements_list.measurements[measurement_idx].y;
        obj.state_vector(static_cast<Eigen::Index>(BEVOFStateIndex::kZ)) =
            measurements_list.measurements[measurement_idx].z;
        obj.state_vector(static_cast<Eigen::Index>(BEVOFStateIndex::kLength)) =
            measurements_list.measurements[measurement_idx].length;
        obj.state_vector(static_cast<Eigen::Index>(BEVOFStateIndex::kWidth)) =
            measurements_list.measurements[measurement_idx].width;
        obj.state_vector(static_cast<Eigen::Index>(BEVOFStateIndex::kHeight)) =
            measurements_list.measurements[measurement_idx].height;
        obj.state_vector(static_cast<Eigen::Index>(BEVOFStateIndex::kOrientation)) =
            measurements_list.measurements[measurement_idx].orientation;

        tracks_.tracks.emplace_back(obj);
    }
}

BEVMeasurementList BEVObjectsFusion::TransformMeasurementsFromEgoVehicleRearAxisToOdometry(
    const BEVMeasurementList& measurements_list,
    const CartesianPose2D& ego_pose_in_odometry)
{
    BEVMeasurementList measurements_list_in_odometry;
    measurements_list_in_odometry.detected_time_stamp = measurements_list.detected_time_stamp;
    measurements_list_in_odometry.sensor_type = measurements_list.sensor_type;

    const float ego_x_in_odometry = static_cast<float>(ego_pose_in_odometry.position.x);
    const float ego_y_in_odometry = static_cast<float>(ego_pose_in_odometry.position.y);
    const float ego_orientation_in_odometry = static_cast<float>(ego_pose_in_odometry.orientation);

    for (const auto& observed_object : measurements_list.measurements)
    {
        BEVMeasurement observed_object_in_odometry;
        observed_object_in_odometry.x = std::cos(ego_orientation_in_odometry) * observed_object.x -
                                        std::sin(ego_orientation_in_odometry) * observed_object.y + ego_x_in_odometry;
        observed_object_in_odometry.y = std::sin(ego_orientation_in_odometry) * observed_object.x +
                                        std::cos(ego_orientation_in_odometry) * observed_object.y + ego_y_in_odometry;
        observed_object_in_odometry.z = observed_object.z;

        /// Wrap orientation to [-pi, pi) range.
        observed_object_in_odometry.orientation = wrap_to_pi(observed_object.orientation + ego_orientation_in_odometry);

        observed_object_in_odometry.length = observed_object.length;
        observed_object_in_odometry.width = observed_object.width;
        observed_object_in_odometry.height = observed_object.height;
        observed_object_in_odometry.existence = observed_object.existence;
        observed_object_in_odometry.classification = observed_object.classification;

        measurements_list_in_odometry.measurements.emplace_back(observed_object_in_odometry);
    }

    return measurements_list_in_odometry;
}

BEVOFTrackObjectList BEVObjectsFusion::TransfromTracksFromOdometryToEgoVehicleRearAxis(
    const BEVOFTrackObjectList& tracks_list,
    const CartesianPose2D& ego_pose_in_odometry)
{
    BEVOFTrackObjectList tracks_list_in_ego;
    tracks_list_in_ego.time_stamp = tracks_list.time_stamp;

    const float ego_x_in_odometry = static_cast<float>(ego_pose_in_odometry.position.x);
    const float ego_y_in_odometry = static_cast<float>(ego_pose_in_odometry.position.y);
    const float ego_orientation_in_odometry = static_cast<float>(ego_pose_in_odometry.orientation);

    const float odometry_x_in_ego = -std::cos(ego_orientation_in_odometry) * ego_x_in_odometry -
                                    std::sin(ego_orientation_in_odometry) * ego_y_in_odometry;
    const float odometry_y_in_ego = std::sin(ego_orientation_in_odometry) * ego_x_in_odometry -
                                    std::cos(ego_orientation_in_odometry) * ego_y_in_odometry;
    const float odometry_orientation_in_ego = -ego_orientation_in_odometry;

    for (std::uint32_t i = 0U; i < static_cast<std::uint32_t>(tracks_list.tracks.size()); ++i)
    {
        BEVOFTrackObject track_in_ego;
        track_in_ego.id = tracks_list.tracks[i].id;

        track_in_ego.x = std::cos(odometry_orientation_in_ego) * tracks_list.tracks[i].x -
                         std::sin(odometry_orientation_in_ego) * tracks_list.tracks[i].y + odometry_x_in_ego;
        track_in_ego.y = std::sin(odometry_orientation_in_ego) * tracks_list.tracks[i].x +
                         std::cos(odometry_orientation_in_ego) * tracks_list.tracks[i].y + odometry_y_in_ego;
        track_in_ego.z = tracks_list.tracks[i].z;

        track_in_ego.velocity_x = std::cos(odometry_orientation_in_ego) * tracks_list.tracks[i].velocity_x -
                                  std::sin(odometry_orientation_in_ego) * tracks_list.tracks[i].velocity_y;
        track_in_ego.velocity_y = std::sin(odometry_orientation_in_ego) * tracks_list.tracks[i].velocity_x +
                                  std::cos(odometry_orientation_in_ego) * tracks_list.tracks[i].velocity_y;
        track_in_ego.velocity_z = tracks_list.tracks[i].velocity_z;

        track_in_ego.acceleration_x = std::cos(odometry_orientation_in_ego) * tracks_list.tracks[i].acceleration_x -
                                      std::sin(odometry_orientation_in_ego) * tracks_list.tracks[i].acceleration_y;
        track_in_ego.acceleration_y = std::sin(odometry_orientation_in_ego) * tracks_list.tracks[i].acceleration_x +
                                      std::cos(odometry_orientation_in_ego) * tracks_list.tracks[i].acceleration_y;
        track_in_ego.acceleration_z = tracks_list.tracks[i].acceleration_z;

        track_in_ego.orientation = wrap_to_pi(tracks_list.tracks[i].orientation + odometry_orientation_in_ego);

        track_in_ego.orientation_rate = tracks_list.tracks[i].orientation_rate;
        track_in_ego.length = tracks_list.tracks[i].length;
        track_in_ego.width = tracks_list.tracks[i].width;
        track_in_ego.height = tracks_list.tracks[i].height;
        track_in_ego.existence = tracks_list.tracks[i].existence;
        track_in_ego.classification = tracks_list.tracks[i].classification;

        /// Convert uncertainty from odometry to ego_vehicle_rear_axis.
        track_in_ego.x_std =
            std::sqrt(std::cos(odometry_orientation_in_ego) * std::cos(odometry_orientation_in_ego) *
                          tracks_list.tracks[i].x_std * tracks_list.tracks[i].x_std +
                      (-std::sin(odometry_orientation_in_ego)) * (-std::sin(odometry_orientation_in_ego)) *
                          tracks_list.tracks[i].y_std * tracks_list.tracks[i].y_std);
        track_in_ego.y_std =
            std::sqrt(std::sin(odometry_orientation_in_ego) * std::sin(odometry_orientation_in_ego) *
                          tracks_list.tracks[i].x_std * tracks_list.tracks[i].x_std +
                      (std::cos(odometry_orientation_in_ego)) * (std::cos(odometry_orientation_in_ego)) *
                          tracks_list.tracks[i].y_std * tracks_list.tracks[i].y_std);
        track_in_ego.z_std = tracks_list.tracks[i].z_std;

        track_in_ego.velocity_x_std =
            std::sqrt(std::cos(odometry_orientation_in_ego) * std::cos(odometry_orientation_in_ego) *
                          tracks_list.tracks[i].velocity_x_std * tracks_list.tracks[i].velocity_x_std +
                      (-std::sin(odometry_orientation_in_ego)) * (-std::sin(odometry_orientation_in_ego)) *
                          tracks_list.tracks[i].velocity_y_std * tracks_list.tracks[i].velocity_y_std);
        track_in_ego.velocity_y_std =
            std::sqrt(std::sin(odometry_orientation_in_ego) * std::sin(odometry_orientation_in_ego) *
                          tracks_list.tracks[i].velocity_x_std * tracks_list.tracks[i].velocity_x_std +
                      (std::cos(odometry_orientation_in_ego)) * (std::cos(odometry_orientation_in_ego)) *
                          tracks_list.tracks[i].velocity_y_std * tracks_list.tracks[i].velocity_y_std);
        track_in_ego.velocity_z_std = tracks_list.tracks[i].velocity_z_std;

        track_in_ego.acceleration_x_std =
            std::sqrt(std::cos(odometry_orientation_in_ego) * std::cos(odometry_orientation_in_ego) *
                          tracks_list.tracks[i].acceleration_x_std * tracks_list.tracks[i].acceleration_x_std +
                      (-std::sin(odometry_orientation_in_ego)) * (-std::sin(odometry_orientation_in_ego)) *
                          tracks_list.tracks[i].acceleration_y_std * tracks_list.tracks[i].acceleration_y_std);
        track_in_ego.acceleration_y_std =
            std::sqrt(std::sin(odometry_orientation_in_ego) * std::sin(odometry_orientation_in_ego) *
                          tracks_list.tracks[i].acceleration_x_std * tracks_list.tracks[i].acceleration_x_std +
                      (std::cos(odometry_orientation_in_ego)) * (std::cos(odometry_orientation_in_ego)) *
                          tracks_list.tracks[i].acceleration_y_std * tracks_list.tracks[i].acceleration_y_std);
        track_in_ego.acceleration_z_std = tracks_list.tracks[i].acceleration_z_std;

        track_in_ego.orientation_std = tracks_list.tracks[i].orientation_std;
        track_in_ego.orientation_rate_std = tracks_list.tracks[i].orientation_rate_std;
        track_in_ego.length_std = tracks_list.tracks[i].length_std;
        track_in_ego.width_std = tracks_list.tracks[i].width_std;
        track_in_ego.height_std = tracks_list.tracks[i].height_std;

        track_in_ego.create_time_stamp = tracks_list.tracks[i].create_time_stamp;
        track_in_ego.latest_fused_time_stamp = tracks_list.tracks[i].latest_fused_time_stamp;
        track_in_ego.state = tracks_list.tracks[i].state;

        tracks_list_in_ego.tracks.emplace_back(track_in_ego);
    }

    return tracks_list_in_ego;
}

}  // namespace object_fusion
}  // namespace perception