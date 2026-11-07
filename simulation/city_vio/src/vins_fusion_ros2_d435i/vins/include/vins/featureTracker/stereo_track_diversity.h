#pragma once

#include <algorithm>
#include <cstddef>
#include <vector>

namespace vins {

struct StereoTrackDiversityCandidate {
  std::size_t index = 0;
  int cell = 0;
  int depth_bin = 0;
  int track_count = 0;
};

inline std::vector<std::size_t> selectStereoTrackDiversity(
    const std::vector<StereoTrackDiversityCandidate>& candidates,
    int maximum_count, int cell_count, int depth_bin_count,
    int maximum_per_cell, int minimum_count) {
  if (maximum_count <= 0 || cell_count <= 0 || depth_bin_count <= 0 ||
      candidates.empty()) {
    return {};
  }
  maximum_per_cell = std::max(1, maximum_per_cell);
  minimum_count = std::max(0, std::min(minimum_count, maximum_count));

  std::vector<std::vector<std::vector<StereoTrackDiversityCandidate>>> buckets(
      cell_count,
      std::vector<std::vector<StereoTrackDiversityCandidate>>(
          depth_bin_count));
  for (const auto& candidate : candidates) {
    if (candidate.cell < 0 || candidate.cell >= cell_count ||
        candidate.depth_bin < 0 ||
        candidate.depth_bin >= depth_bin_count) {
      continue;
    }
    buckets[candidate.cell][candidate.depth_bin].push_back(candidate);
  }
  for (auto& cell : buckets) {
    for (auto& bucket : cell) {
      std::stable_sort(
          bucket.begin(), bucket.end(),
          [](const StereoTrackDiversityCandidate& first,
             const StereoTrackDiversityCandidate& second) {
            if (first.track_count != second.track_count) {
              return first.track_count > second.track_count;
            }
            return first.index < second.index;
          });
    }
  }

  std::vector<std::vector<std::size_t>> offsets(
      cell_count, std::vector<std::size_t>(depth_bin_count, 0));
  std::vector<int> selected_per_cell(cell_count, 0);
  std::vector<std::size_t> selected;
  selected.reserve(
      std::min<std::size_t>(candidates.size(), maximum_count));

  // Known, farther bins are visited before near and unknown bins. Repeated
  // rounds still retain long tracks from every available bin.
  std::vector<int> depth_order;
  for (int bin = depth_bin_count - 1; bin >= 1; --bin) {
    depth_order.push_back(bin);
  }
  depth_order.push_back(0);

  const int base_per_cell =
      std::max(1, (maximum_count + cell_count - 1) / cell_count);
  const auto select_one_round = [&](int per_cell_limit) {
    bool changed = false;
    for (int cell = 0; cell < cell_count &&
                       static_cast<int>(selected.size()) < maximum_count;
         ++cell) {
      if (selected_per_cell[cell] >= per_cell_limit) continue;
      for (const int depth_bin : depth_order) {
        auto& bucket = buckets[cell][depth_bin];
        auto& offset = offsets[cell][depth_bin];
        if (offset >= bucket.size()) continue;
        selected.push_back(bucket[offset++].index);
        ++selected_per_cell[cell];
        changed = true;
        break;
      }
    }
    return changed;
  };

  while (static_cast<int>(selected.size()) < maximum_count &&
         select_one_round(base_per_cell)) {
  }
  while (static_cast<int>(selected.size()) < maximum_count &&
         select_one_round(maximum_per_cell)) {
  }

  // In a very sparse scene, preserve a bounded minimum number of tracks even
  // if they come from already-full cells. This is only a fail-safe for VINS
  // initialization; normal operation remains governed by the grid cap.
  while (static_cast<int>(selected.size()) < minimum_count) {
    bool changed = false;
    for (int cell = 0; cell < cell_count &&
                       static_cast<int>(selected.size()) < minimum_count;
         ++cell) {
      for (const int depth_bin : depth_order) {
        auto& bucket = buckets[cell][depth_bin];
        auto& offset = offsets[cell][depth_bin];
        if (offset >= bucket.size()) continue;
        selected.push_back(bucket[offset++].index);
        ++selected_per_cell[cell];
        changed = true;
        break;
      }
    }
    if (!changed) break;
  }
  return selected;
}

}  // namespace vins
