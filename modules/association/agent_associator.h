///
/// @copyright Copyright (C) 2021-2025, Gawei Xu
///
/// @author Gaowei Xu (gaowexu1991@gmail.com)
/// @date 2023-03-01
///

#ifndef MODULES_ASSOCIATION_AGENT_ASSOCIATOR_H
#define MODULES_ASSOCIATION_AGENT_ASSOCIATOR_H

#include "modules/types/types.h"

namespace perception
{
namespace object_fusion
{

class AgentAssociator
{
  public:
    AgentAssociator(const float iou_distance_weight = 1.0F,
                    const float euclidean_distance_weight = 1.0F,
                    const float association_iou_gate = 0.10F,
                    const float association_euclidean_dist_gate = 2.5F);

    AgentAssociatedIndices Associate(const BEVMeasurementList& measurements_list, const BEVOFTrackObjectList& tracks);

  private:
    bool CheckIsAssociated(const float iou_value, const float euclidean_dist_value);
    bool NotFound(const std::uint32_t track_idx, const std::vector<std::uint32_t>& unmatched_track_indices);
    bool NotFound(const std::uint32_t track_idx, std::vector<MatchedAgentIndexPair>& matched_indices);

  private:
    /// @brief IoU distance weight for Munkres assignment distance measurement.
    ///
    /// Defaults to 1.0F.
    const float iou_distance_weight_;

    /// @brief Euclidean distance weight for Munkres assignment distance measurement.
    ///
    /// Defaults to 1.0F.
    const float euclidean_distance_weight_;

    /// @brief Threshold of association double-check phase. If two objects' IoU is larger than this threshold, then the
    /// association will be identified as valid.
    ///
    /// Defaults to 0.10F.
    const float association_iou_gate_;

    /// @brief Threshold of association double-check phase. If two objects' normalized euclidean distance is smaller
    /// than this threshold, then the association will be identified as valid.
    ///
    /// Defaults to 2.50F (meter).
    const float association_euclidean_dist_gate_;
};

}  // namespace object_fusion
}  // namespace perception

#endif  // MODULES_ASSOCIATION_AGENT_ASSOCIATOR_H
