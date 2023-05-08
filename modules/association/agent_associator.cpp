///
/// @copyright Copyright (C) 2021-2025, Gawei Xu
///
/// @author Gaowei Xu (gaowexu1991@gmail.com)
/// @date 2023-03-01
///

#include "modules/association/agent_associator.h"

#include "modules/association/munkres/munkres.h"
#include "modules/types/utils.h"

namespace perception
{
namespace object_fusion
{

AgentAssociator::AgentAssociator(const float iou_distance_weight,
                                 const float euclidean_distance_weight,
                                 const float association_iou_gate,
                                 const float association_euclidean_dist_gate)
    : iou_distance_weight_(iou_distance_weight),
      euclidean_distance_weight_(euclidean_distance_weight),
      association_iou_gate_(association_iou_gate),
      association_euclidean_dist_gate_(association_euclidean_dist_gate)
{
}

AgentAssociatedIndices AgentAssociator::Associate(const BEVMeasurementList& measurements_list,
                                                  const BEVOFTrackObjectList& tracks)
{
    const std::uint32_t measurements_size = static_cast<std::uint32_t>(measurements_list.measurements.size());
    const std::uint32_t tracks_size = static_cast<std::uint32_t>(tracks.tracks.size());
    const std::uint32_t dim = std::max(measurements_size, tracks_size);

    std::vector<std::vector<float>> iou_dist_matrix(dim, std::vector<float>(dim, 0.0F));
    std::vector<std::vector<float>> euclidean_dist_matrix(dim, std::vector<float>(dim, 0.0F));
    std::vector<std::vector<float>> cost_distance(dim, std::vector<float>(dim, 0.0F));

    for (std::uint32_t i = 0U; i < measurements_size; ++i)
    {
        for (std::uint32_t j = 0U; j < tracks_size; ++j)
        {
            const Box3D box_m(measurements_list.measurements[i].x,
                              measurements_list.measurements[i].y,
                              measurements_list.measurements[i].z,
                              measurements_list.measurements[i].length,
                              measurements_list.measurements[i].width,
                              measurements_list.measurements[i].height,
                              measurements_list.measurements[i].orientation);

            const Box3D box_t(tracks.tracks[j].state_vector(static_cast<Eigen::Index>(BEVOFStateIndex::kX)),
                              tracks.tracks[j].state_vector(static_cast<Eigen::Index>(BEVOFStateIndex::kY)),
                              tracks.tracks[j].state_vector(static_cast<Eigen::Index>(BEVOFStateIndex::kZ)),
                              tracks.tracks[j].state_vector(static_cast<Eigen::Index>(BEVOFStateIndex::kLength)),
                              tracks.tracks[j].state_vector(static_cast<Eigen::Index>(BEVOFStateIndex::kWidth)),
                              tracks.tracks[j].state_vector(static_cast<Eigen::Index>(BEVOFStateIndex::kHeight)),
                              tracks.tracks[j].state_vector(static_cast<Eigen::Index>(BEVOFStateIndex::kOrientation)));

            const float iou = CalculateIoU(box_m, box_t);
            const float distance = CalculateEuclideanDistance(box_m, box_t, false, true);

            iou_dist_matrix[i][j] = iou;
            euclidean_dist_matrix[i][j] = distance;

            /// Larger value indicates higher probability that corresponding objects are the same one.
            cost_distance[i][j] = iou_distance_weight_ * iou + euclidean_distance_weight_ * (1.0F / std::exp(distance));
        }
    }

    /// Munkres Matching Solver
    Munkres solver(cost_distance);
    const std::vector<std::int32_t> response = solver.Solve();

    AgentAssociatedIndices output;
    for (std::uint32_t measurement_idx = 0; measurement_idx < measurements_size; ++measurement_idx)
    {
        const std::uint32_t track_idx = static_cast<std::uint32_t>(response[measurement_idx]);

        if ((track_idx < tracks_size))
        {
            if (CheckIsAssociated(iou_dist_matrix[measurement_idx][track_idx],
                                  euclidean_dist_matrix[measurement_idx][track_idx]))
            {
                MatchedAgentIndexPair agent_pair(measurement_idx, track_idx);
                output.matched_indices.emplace_back(agent_pair);
            }
            else
            {
                output.unmatched_measurment_indices.emplace_back(measurement_idx);
                output.unmatched_track_indices.emplace_back(track_idx);
            }
        }
        else
        {
            output.unmatched_measurment_indices.emplace_back(measurement_idx);
        }
    }

    for (std::uint32_t track_idx = 0; track_idx < tracks_size; ++track_idx)
    {
        if (NotFound(track_idx, output.unmatched_track_indices) && NotFound(track_idx, output.matched_indices))
        {
            output.unmatched_track_indices.emplace_back(track_idx);
        }
    }

    return output;
}

bool AgentAssociator::CheckIsAssociated(const float iou_value, const float euclidean_dist_value)
{
    if ((iou_value < association_iou_gate_) && (euclidean_dist_value > association_euclidean_dist_gate_))
        return false;

    return true;
}

bool AgentAssociator::NotFound(const std::uint32_t track_idx, const std::vector<std::uint32_t>& unmatched_track_indices)
{
    for (size_t i = 0; i < unmatched_track_indices.size(); ++i)
    {
        if (track_idx == unmatched_track_indices[i])
            return false;
    }

    return true;
}

bool AgentAssociator::NotFound(const std::uint32_t track_idx, std::vector<MatchedAgentIndexPair>& matched_indices)
{
    for (size_t i = 0; i < matched_indices.size(); ++i)
    {
        if (track_idx == matched_indices[i].track_idx)
            return false;
    }

    return true;
}

}  // namespace object_fusion
}  // namespace perception
