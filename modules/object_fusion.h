///
/// @copyright Copyright (C) 2021-2025, Gawei Xu
///
/// @author Gaowei Xu (gaowexu1991@gmail.com)
/// @date 2023-03-01
///

#ifndef MODULES_OBJECT_FUSION_H
#define MODULES_OBJECT_FUSION_H

#include <memory>
#include "modules/association/agent_associator.h"
#include "modules/fusion/agent_updater.h"
#include "modules/prediction/agent_predictor.h"
#include "modules/types/types.h"

namespace perception
{
namespace object_fusion
{

class BEVObjectsFusion
{
  public:
    BEVObjectsFusion(const float duration_thresh_for_agent_removal = 0.30F,
                     const float iou_distance_weight = 1.0F,
                     const float euclidean_distance_weight = 1.0F,
                     const float association_iou_gate = 0.10F,
                     const float association_euclidean_dist_gate = 2.50F);
    ~BEVObjectsFusion();

    /// @brief Kalman track & fusion processor.
    ///
    /// @param measurements_list BEV detections in ego_vehicle_rear_axis coordinate system.
    /// @param ego_pose_in_odometry Ego pose in odometry coordinate system at measurements_list.detected_time_stamp.
    ///
    /// @return Tracked and fused objects in ego_vehicle_rear_axis.
    BEVOFTrackObjectList Run(const BEVMeasurementList& measurements_list, const CartesianPose2D& ego_pose_in_odometry);

  private:
    void KalmanTrackAndFuse(const BEVMeasurementList& measurements_list);

    BEVMeasurementList TransformMeasurementsFromEgoVehicleRearAxisToOdometry(
        const BEVMeasurementList& measurements_list,
        const CartesianPose2D& ego_pose_in_odometry);

    BEVOFTrackObjectList TransfromTracksFromOdometryToEgoVehicleRearAxis(const BEVOFTrackObjectList& tracks_list,
                                                                         const CartesianPose2D& ego_pose_in_odometry);

  private:
    /// @brief Object's UUID.
    std::uint32_t agent_id_;

    /// @brief Duration threshold (with unit seconds) when an object will be removed if it is not associated with
    /// measurements.
    ///
    /// Defaults to 0.30F, i.e., 300 milliseconds.
    const float duration_thresh_for_agent_removal_;

    /// @brief Optimal estimation of BEV detections in odometry coordinate system.
    BEVOFTrackObjectList tracks_;

    /// @brief Predictor with const acceleration model.
    std::unique_ptr<AgentPredictor> agent_predictor_;

    /// @brief Assoicator of previous tracks and current predicted agents.
    std::unique_ptr<AgentAssociator> agent_associator_;

    /// @brief Fuse agent's kinematic state which includes state, classification and existence.
    std::unique_ptr<AgentUpdater> agent_fuser_;
};

}  // namespace object_fusion
}  // namespace perception

#endif  // MODULES_OBJECT_FUSION_H
