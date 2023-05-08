///
/// @copyright Copyright (C) 2021-2025, Gawei Xu
///
/// @author Gaowei Xu (gaowexu1991@gmail.com)
/// @date 2023-03-01
///

#include "modules/association/munkres/munkres.h"

#include <deque>

#include "modules/types/types.h"

namespace perception
{
namespace object_fusion
{

Munkres::Munkres(const std::vector<std::vector<float>>& cost_matrix)
    : dim_(static_cast<std::uint32_t>(cost_matrix.size())), x_labels_(dim_, 0.0F), y_labels_(dim_, 0.0F)
{
    square_matrix_.reserve(dim_);
    for (auto row = 0U; row < dim_; ++row)
    {
        std::vector<float> row_data;
        row_data.reserve(dim_);
        for (auto col = 0U; col < dim_; ++col)
        {
            row_data.emplace_back(cost_matrix[row][col]);
        }

        square_matrix_.emplace_back(std::move(row_data));
    }

    matches_.reserve(dim_);
    equality_edges_.reserve(2 * dim_);
    s_set_.reserve(dim_);
    t_set_.reserve(dim_);
    nls_set_.reserve(dim_);
    diff_y_set_.reserve(dim_);

    InitLables();
}

void Munkres::InitLables()
{
    for (auto row = 0U; row < square_matrix_.size(); ++row)
    {
        x_labels_[row] = *std::max_element(square_matrix_[row].begin(), square_matrix_[row].end());
    }
}

void Munkres::InitEqualityEdges()
{
    equality_edges_.clear();
    const auto dim = x_labels_.size();
    for (auto i = 0U; i < dim; i++)
    {
        for (auto j = 0U; j < dim; j++)
        {
            if (std::abs(x_labels_[i] + y_labels_[j] - square_matrix_[i][j]) < 0.001F)
            {
                equality_edges_.push_back({i, j});
            }
        }
    }
}

void Munkres::UpdateEqualityEdges(const Edges& new_edges)
{
    equality_edges_.erase(std::remove_if(equality_edges_.begin(),
                                         equality_edges_.end(),
                                         [this](Edge& edge) {
                                             if (std::find(s_set_.cbegin(), s_set_.cend(), edge.x) == s_set_.cend() &&
                                                 std::find(t_set_.cbegin(), t_set_.cend(), edge.y) != t_set_.cend() &&
                                                 x_labels_[edge.x] + y_labels_[edge.y] > square_matrix_[edge.x][edge.y])
                                             {
                                                 return true;
                                             }
                                             return false;
                                         }),
                          equality_edges_.end());

    std::copy(new_edges.cbegin(), new_edges.cend(), std::back_inserter(equality_edges_));
}

std::uint32_t Munkres::PickFreeX()
{
    auto free = free_x_;
    ++free_x_;
    return free;
}

void Munkres::UpdateSTSetByFreeX(const std::uint32_t free_x)
{
    s_set_.clear();
    s_set_.push_back(free_x);
    t_set_.clear();
}

void Munkres::AddNeighbourOfXToNlsSet(const std::uint32_t x)
{
    for (const auto& edge : equality_edges_)
    {
        if ((edge.x == x) && (std::find(nls_set_.cbegin(), nls_set_.cend(), edge.y) == nls_set_.cend()))
        {
            nls_set_.push_back(edge.y);
        }
    }
}

void Munkres::UpdateDiffYSet()
{
    diff_y_set_.clear();
    for (const auto& n : nls_set_)
    {
        if (std::find(t_set_.cbegin(), t_set_.cend(), n) == t_set_.cend())
        {
            diff_y_set_.emplace_back(n);
        }
    }
}

void Munkres::GetMinDeltaWeight(float& min_delta, Edges& new_edges)
{
    min_delta = std::numeric_limits<float>::max();
    for (const auto x : s_set_)
    {
        for (std::uint32_t y = 0U; y < square_matrix_.size(); y++)
        {
            if (std::find(t_set_.cbegin(), t_set_.cend(), y) == t_set_.cend())
            {
                const auto delta = x_labels_[x] + y_labels_[y] - square_matrix_[x][y];
                ;
                if (delta > min_delta)
                {
                    continue;
                }
                else if (delta < min_delta)
                {
                    min_delta = delta;
                    new_edges.clear();
                    new_edges.push_back({x, y});
                }
                else
                {
                    new_edges.push_back({x, y});
                }
            }
        }
    }
}

void Munkres::UpdateLabels(const float val)
{
    std::for_each(s_set_.cbegin(), s_set_.cend(), [&val, this](const auto x) { x_labels_[x] -= val; });
    std::for_each(t_set_.cbegin(), t_set_.cend(), [&val, this](const auto x) { y_labels_[x] += val; });
}

Path Munkres::FindAugmentPath(const std::uint32_t free_x, const std::uint32_t target_y)
{
    Path path{};
    for (const auto& edge : equality_edges_)
    {
        if (edge.x == free_x && edge.y == target_y)
        {
            path.push_back(free_x);
            path.push_back(target_y);
            return path;
        }
    }

    path.reserve(dim_);

    auto tmp_t_set = t_set_;
    tmp_t_set.push_back(target_y);

    std::deque<std::uint32_t> needs_to_check;
    std::vector<bool> s_in_deque(dim_, false);
    std::vector<bool> t_in_deque(dim_, false);
    needs_to_check.push_back(free_x);
    s_in_deque[free_x] = true;
    while (!needs_to_check.empty())
    {
        auto current_node = needs_to_check.back();
        bool in_x = current_node < dim_;
        if (std::find(path.cbegin(), path.cend(), current_node) == path.cend())
        {
            path.push_back(current_node);
        }
        bool has_edge(false);
        if (in_x)
        {
            for (const auto& edge : equality_edges_)
            {
                if (edge.x == current_node && t_in_deque[edge.y] == false &&
                    std::find(tmp_t_set.cbegin(), tmp_t_set.cend(), edge.y) != tmp_t_set.cend())
                {
                    needs_to_check.push_back(edge.y + dim_);
                    t_in_deque[edge.y] = true;
                    has_edge = true;
                    if (edge.y == target_y)
                    {
                        path.push_back(edge.y + dim_);
                        for (auto& node : path)
                        {
                            if (node >= dim_)
                            {
                                node -= dim_;
                            }
                        }
                        return path;
                    }
                }
            }
        }
        else
        {
            for (const auto& edge : matches_)
            {
                if (edge.y == current_node - dim_ && s_in_deque[edge.x] == false &&
                    std::find(s_set_.cbegin(), s_set_.cend(), edge.x) != s_set_.cend())
                {
                    needs_to_check.push_back(edge.x);
                    s_in_deque[edge.x] = true;
                    has_edge = true;
                }
            }
        }
        if (!has_edge)  // no more edges from this node was added to deque.
        {
            path.pop_back();
            needs_to_check.pop_back();
        }
    }

    return {};
}

std::uint32_t Munkres::FindXMatchedToY(const std::uint32_t y)
{
    std::uint32_t x = InvalidIndex;
    for (const auto& edge : matches_)
    {
        if (edge.y == y)
        {
            x = edge.x;
            break;
        }
    }

    return x;
}

std::uint32_t Munkres::FindEdgeInEdges(const Edges& edges, const Edge& edge)
{
    for (auto idx = 0U; idx < edges.size(); idx++)
    {
        const auto& one_edge = edges[idx];
        if ((one_edge.x == edge.x) && (one_edge.y == edge.y))
        {
            return idx;
        }
    }

    return InvalidIndex;
}

void Munkres::UpdateMatches(const Path& augment_path)
{
    auto current_node_x_set = true;
    for (auto idx = 0U; idx < augment_path.size() - 1U; idx++)
    {
        Edge edge{augment_path[idx], augment_path[idx + 1U]};
        if (current_node_x_set)
        {
            matches_.push_back(edge);
        }
        else
        {
            std::swap(edge.x, edge.y);
            const auto edge_idx = FindEdgeInEdges(matches_, edge);
            if (edge_idx != InvalidIndex)
            {
                matches_.erase(matches_.cbegin() + edge_idx);
            }
        }

        current_node_x_set = !current_node_x_set;
    }
}

std::vector<std::int32_t> Munkres::ConvertToAssociationMap()
{
    std::vector<std::int32_t> association_map(static_cast<std::uint32_t>(square_matrix_.size()),
                                              static_cast<std::int32_t>(InvalidIndex));
    for (const auto& match : matches_)
    {
        association_map[match.x] = static_cast<std::int32_t>(match.y);
    }

    return association_map;
}

std::vector<std::int32_t> Munkres::Solve()
{
    free_x_ = 0;

    InitEqualityEdges();

    auto free_x = InvalidIndex;

    while (matches_.size() < dim_)
    {
        if (free_x == InvalidIndex)
        {
            free_x = PickFreeX();

            // init st set
            UpdateSTSetByFreeX(free_x);

            // init nls set by free_x
            nls_set_.clear();
            AddNeighbourOfXToNlsSet(free_x);
        }

        // pick a y node and check if diff y equals to nls
        UpdateDiffYSet();

        if (diff_y_set_.empty())
        {
            float min_delta;
            Edges new_edges;
            GetMinDeltaWeight(min_delta, new_edges);
            UpdateLabels(min_delta);

            // InitEqualityEdges();
            UpdateEqualityEdges(new_edges);

            for (const auto& edge : new_edges)
            {
                AddNeighbourOfXToNlsSet(edge.x);
            }
        }
        else
        {
            // pick y
            const auto target_y = diff_y_set_[0U];
            const auto matched_x = FindXMatchedToY(target_y);

            if (matched_x == InvalidIndex)
            {
                const auto& path = FindAugmentPath(free_x, target_y);
                UpdateMatches(path);
                free_x = InvalidIndex;
            }
            else
            {
                // update st set
                s_set_.push_back(matched_x);
                t_set_.push_back(target_y);

                // upate nls set
                AddNeighbourOfXToNlsSet(matched_x);
            }
        }
    }

    const auto& association_map = ConvertToAssociationMap();
    return association_map;
}

}  // namespace object_fusion
}  // namespace perception
