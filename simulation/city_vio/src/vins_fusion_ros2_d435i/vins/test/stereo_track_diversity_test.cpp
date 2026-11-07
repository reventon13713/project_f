#include <vins/featureTracker/stereo_track_diversity.h>

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <vector>

namespace {

bool contains(const std::vector<std::size_t>& values, std::size_t value) {
  return std::find(values.begin(), values.end(), value) != values.end();
}

}  // namespace

int main() {
  using vins::StereoTrackDiversityCandidate;
  using vins::selectStereoTrackDiversity;

  const std::vector<StereoTrackDiversityCandidate> balanced{
      {0, 0, 1, 20}, {1, 0, 1, 10}, {2, 0, 4, 5},
      {3, 1, 1, 30}, {4, 1, 2, 15}, {5, 1, 4, 3},
  };
  const auto selected =
      selectStereoTrackDiversity(balanced, 4, 2, 5, 2, 0);
  assert(selected.size() == 4);
  assert(contains(selected, 2));
  assert(contains(selected, 5));

  const std::vector<StereoTrackDiversityCandidate> same_bucket{
      {9, 0, 1, 2}, {4, 0, 1, 8}, {7, 0, 1, 5},
  };
  const auto oldest =
      selectStereoTrackDiversity(same_bucket, 1, 1, 5, 1, 0);
  assert(oldest.size() == 1);
  assert(oldest.front() == 4);

  const std::vector<StereoTrackDiversityCandidate> sparse{
      {0, 0, 1, 5}, {1, 0, 1, 4}, {2, 0, 1, 3},
      {3, 0, 1, 2}, {4, 0, 1, 1},
  };
  const auto minimum =
      selectStereoTrackDiversity(sparse, 5, 4, 5, 1, 4);
  assert(minimum.size() == 4);

  assert(selectStereoTrackDiversity(
             balanced, 0, 2, 5, 2, 0).empty());
  assert(selectStereoTrackDiversity(
             balanced, 4, 0, 5, 2, 0).empty());
  return 0;
}
