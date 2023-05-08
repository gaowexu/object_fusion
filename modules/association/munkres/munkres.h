///
/// @copyright Copyright (C) 2021-2025, Gawei Xu
///
/// @author Gaowei Xu (gaowexu1991@gmail.com)
/// @date 2023-03-01
///

#ifndef MODULES_ASSOCIATION_MUNKRES_MUNKRES_H
#define MODULES_ASSOCIATION_MUNKRES_MUNKRES_H

#include <cstdint>
#include <vector>

namespace perception
{
namespace object_fusion
{

struct Edge
{
    std::uint32_t x;
    std::uint32_t y;
};

using Edges = std::vector<Edge>;
using Labels = std::vector<float>;
using NodeSet = std::vector<std::uint32_t>;
using Path = std::vector<std::uint32_t>;

const constexpr std::uint32_t InvalidIndex = 4294967295U;

class Munkres
{
  public:
    Munkres(const std::vector<std::vector<float>>& distances);
    ~Munkres() = default;

    std::vector<std::int32_t> Solve();

  private:
    void InitLables();
    void InitEqualityEdges();
    void UpdateLabels(const float val);
    void UpdateEqualityEdges(const Edges& new_edges);
    void AddNeighbourOfXToNlsSet(const std::uint32_t x);
    std::uint32_t PickFreeX();
    void UpdateSTSetByFreeX(const std::uint32_t free_x);
    void UpdateDiffYSet();
    Path FindAugmentPath(const std::uint32_t free_x, const std::uint32_t target_y);
    std::uint32_t FindXMatchedToY(const std::uint32_t y);
    std::uint32_t FindEdgeInEdges(const Edges& edges, const Edge& edge);
    void UpdateMatches(const Path& augment_path);
    std::vector<std::int32_t> ConvertToAssociationMap();
    void GetMinDeltaWeight(float& min_delta, Edges& new_edges);

  protected:
    std::vector<std::vector<float>> square_matrix_;
    const std::uint32_t dim_;
    Labels x_labels_;
    Labels y_labels_;
    Edges matches_{};
    Edges equality_edges_{};
    NodeSet s_set_{};
    NodeSet t_set_{};
    NodeSet nls_set_{};
    NodeSet diff_y_set_{};
    std::uint32_t free_x_{0};
};

}  // namespace object_fusion
}  // namespace perception

#endif  // MODULES_ASSOCIATION_MUNKRES_MUNKRES_H
