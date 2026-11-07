/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science
 *and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/
#include <vins/featureTracker/feature_tracker.h>
#include <vins/featureTracker/stereo_track_diversity.h>
#include <vins/logger/logger.h>

#include <array>
#include <cerrno>
#include <cstdlib>
#include <iomanip>
#include <limits>
#include <sstream>

namespace {
double replayStereoEpipolarMaxVerticalPx() {
  static const double threshold = []() {
    const char *value =
        std::getenv("VINS_REPLAY_STEREO_EPIPOLAR_MAX_VERTICAL_PX");
    if (!value || !*value) return std::numeric_limits<double>::quiet_NaN();
    errno = 0;
    char *end = nullptr;
    const double parsed = std::strtod(value, &end);
    if (errno != 0 || end == value || *end != '\0' ||
        !std::isfinite(parsed) || parsed <= 0.0) {
      throw std::runtime_error(
          "invalid replay stereo epipolar vertical threshold");
    }
    return parsed;
  }();
  return threshold;
}
}  // namespace

bool FeatureTracker::inBorder(const cv::Point2f &pt) {
  const int BORDER_SIZE = 1;
  int img_x = cvRound(pt.x);
  int img_y = cvRound(pt.y);
  return BORDER_SIZE <= img_x && img_x < col - BORDER_SIZE &&
         BORDER_SIZE <= img_y && img_y < row - BORDER_SIZE;
}

double distance(cv::Point2f pt1, cv::Point2f pt2) {
  double dx = pt1.x - pt2.x;
  double dy = pt1.y - pt2.y;
  return sqrt(dx * dx + dy * dy);
}

void reduceVector(vector<cv::Point2f> &v, vector<uchar> status) {
  int j = 0;
  for (int i = 0; i < int(v.size()); i++)
    if (status[i]) v[j++] = v[i];
  v.resize(j);
}

void reduceVector(vector<int> &v, vector<uchar> status) {
  int j = 0;
  for (int i = 0; i < int(v.size()); i++)
    if (status[i]) v[j++] = v[i];
  v.resize(j);
}

FeatureTracker::FeatureTracker() {
  stereo_cam = 0;
  n_id = 0;
  hasPrediction = false;
}

std::unique_ptr<FeatureTracker> FeatureTracker::cloneWithOptions(
    std::shared_ptr<VINSOptions> cloned_options) const {
  auto result = std::unique_ptr<FeatureTracker>(new FeatureTracker);
  result->row = row;
  result->col = col;
  result->imTrack = imTrack.clone();
  result->mask = mask.clone();
  result->fisheye_mask = fisheye_mask.clone();
  result->prev_img = prev_img.clone();
  result->cur_img = cur_img.clone();
  result->n_pts = n_pts;
  result->predict_pts = predict_pts;
  result->predict_pts_debug = predict_pts_debug;
  result->prev_pts = prev_pts;
  result->cur_pts = cur_pts;
  result->cur_right_pts = cur_right_pts;
  result->prev_un_pts = prev_un_pts;
  result->cur_un_pts = cur_un_pts;
  result->cur_un_right_pts = cur_un_right_pts;
  result->pts_velocity = pts_velocity;
  result->right_pts_velocity = right_pts_velocity;
  result->ids = ids;
  result->ids_right = ids_right;
  result->track_cnt = track_cnt;
  result->cur_un_pts_map = cur_un_pts_map;
  result->prev_un_pts_map = prev_un_pts_map;
  result->cur_un_right_pts_map = cur_un_right_pts_map;
  result->prev_un_right_pts_map = prev_un_right_pts_map;
  result->prevLeftPtsMap = prevLeftPtsMap;
  result->stereo_depth_bin_by_id = stereo_depth_bin_by_id;
  result->m_camera = m_camera;
  result->cur_time = cur_time;
  result->prev_time = prev_time;
  result->stereo_cam = stereo_cam;
  result->n_id = n_id;
  result->hasPrediction = hasPrediction;
  result->options = std::move(cloned_options);
  return result;
}

void FeatureTracker::reset() {
  prev_img.release(); cur_img.release(); imTrack.release(); mask.release();
  n_pts.clear(); predict_pts.clear(); predict_pts_debug.clear();
  prev_pts.clear(); cur_pts.clear(); cur_right_pts.clear();
  prev_un_pts.clear(); cur_un_pts.clear(); cur_un_right_pts.clear();
  pts_velocity.clear(); right_pts_velocity.clear();
  ids.clear(); ids_right.clear(); track_cnt.clear();
  cur_un_pts_map.clear(); prev_un_pts_map.clear();
  cur_un_right_pts_map.clear(); prev_un_right_pts_map.clear();
  prevLeftPtsMap.clear();
  stereo_depth_bin_by_id.clear();
  prev_time = 0.0; cur_time = 0.0; n_id = 0; hasPrediction = false;
}

void FeatureTracker::setOptions(std::shared_ptr<VINSOptions> options_) {
  options = options_;
}

void FeatureTracker::setMask() {
  if (options->hasStereoTrackDiversity()) {
    setDiversityMask();
    return;
  }
  mask = cv::Mat(row, col, CV_8UC1, cv::Scalar(255));

  // prefer to keep features that are tracked for long time
  vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;

  for (unsigned int i = 0; i < cur_pts.size(); i++)
    cnt_pts_id.push_back(
        make_pair(track_cnt[i], make_pair(cur_pts[i], ids[i])));

  sort(cnt_pts_id.begin(), cnt_pts_id.end(),
       [](const pair<int, pair<cv::Point2f, int>> &a,
          const pair<int, pair<cv::Point2f, int>> &b) {
         return a.first > b.first;
       });

  cur_pts.clear();
  ids.clear();
  track_cnt.clear();

  for (auto &it : cnt_pts_id) {
    if (mask.at<uchar>(it.second.first) == 255) {
      cur_pts.push_back(it.second.first);
      ids.push_back(it.second.second);
      track_cnt.push_back(it.first);
      cv::circle(mask, it.second.first, options->min_feature_distance, 0, -1);
    }
  }
}

int FeatureTracker::gridCell(const cv::Point2f &point) const {
  const int rows = std::max(1, options->stereo_track_grid_rows);
  const int cols = std::max(1, options->stereo_track_grid_cols);
  const int grid_row = std::max(
      0, std::min(rows - 1,
                  static_cast<int>(point.y * rows / std::max(1, row))));
  const int grid_col = std::max(
      0, std::min(cols - 1,
                  static_cast<int>(point.x * cols / std::max(1, col))));
  return grid_row * cols + grid_col;
}

int FeatureTracker::depthBin(double depth_m) const {
  if (!std::isfinite(depth_m) || depth_m <= 0.0) return 0;
  if (depth_m < options->stereo_track_near_depth_m) return 1;
  if (depth_m < options->stereo_track_mid_depth_m) return 2;
  if (depth_m < options->stereo_track_far_depth_m) return 3;
  return 4;
}

void FeatureTracker::setDiversityMask() {
  mask = cv::Mat(row, col, CV_8UC1, cv::Scalar(255));
  const int cell_count =
      options->stereo_track_grid_rows * options->stereo_track_grid_cols;
  std::vector<vins::StereoTrackDiversityCandidate> candidates;
  candidates.reserve(cur_pts.size());
  for (std::size_t index = 0; index < cur_pts.size(); ++index) {
    const auto depth = stereo_depth_bin_by_id.find(ids[index]);
    candidates.push_back({
        index, gridCell(cur_pts[index]),
        depth == stereo_depth_bin_by_id.end() ? 0 : depth->second,
        track_cnt[index]});
  }
  const auto selected = vins::selectStereoTrackDiversity(
      candidates, options->max_feature_count, cell_count, 5,
      options->stereo_track_max_per_cell,
      options->stereo_track_minimum_count);

  vector<cv::Point2f> selected_points;
  vector<int> selected_ids;
  vector<int> selected_track_counts;
  selected_points.reserve(selected.size());
  selected_ids.reserve(selected.size());
  selected_track_counts.reserve(selected.size());
  for (const std::size_t index : selected) {
    selected_points.push_back(cur_pts[index]);
    selected_ids.push_back(ids[index]);
    selected_track_counts.push_back(track_cnt[index]);
    cv::circle(mask, cur_pts[index], options->min_feature_distance, 0, -1);
  }
  cur_pts = std::move(selected_points);
  ids = std::move(selected_ids);
  track_cnt = std::move(selected_track_counts);
}

void FeatureTracker::detectDiversePoints() {
  n_pts.clear();
  const int grid_rows = std::max(1, options->stereo_track_grid_rows);
  const int grid_cols = std::max(1, options->stereo_track_grid_cols);
  const int cell_count = grid_rows * grid_cols;
  const int base_quota = options->max_feature_count / cell_count;
  const int remainder = options->max_feature_count % cell_count;
  vector<int> existing(cell_count, 0);
  for (const auto &point : cur_pts) ++existing[gridCell(point)];

  for (int cell = 0; cell < cell_count; ++cell) {
    const int maximum_new =
        options->max_feature_count -
        static_cast<int>(cur_pts.size() + n_pts.size());
    if (maximum_new <= 0) break;
    const int target = base_quota + (cell < remainder ? 1 : 0);
    const int needed = std::min(maximum_new, target - existing[cell]);
    if (needed <= 0) continue;
    const int grid_row = cell / grid_cols;
    const int grid_col = cell % grid_cols;
    const int x0 = grid_col * col / grid_cols;
    const int x1 = (grid_col + 1) * col / grid_cols;
    const int y0 = grid_row * row / grid_rows;
    const int y1 = (grid_row + 1) * row / grid_rows;
    const cv::Rect region(x0, y0, x1 - x0, y1 - y0);
    vector<cv::Point2f> local_points;
    cv::goodFeaturesToTrack(
        cur_img(region), local_points, needed, 0.01,
        options->min_feature_distance, mask(region));
    for (auto point : local_points) {
      point.x += static_cast<float>(x0);
      point.y += static_cast<float>(y0);
      n_pts.push_back(point);
      cv::circle(mask, point, options->min_feature_distance, 0, -1);
    }
  }

  const int minimum_target = std::min(
      options->max_feature_count, options->stereo_track_minimum_count);
  const int minimum_deficit =
      minimum_target - static_cast<int>(cur_pts.size() + n_pts.size());
  if (minimum_deficit > 0) {
    vector<cv::Point2f> fallback_points;
    cv::goodFeaturesToTrack(
        cur_img, fallback_points, minimum_deficit, 0.01,
        options->min_feature_distance, mask);
    n_pts.insert(
        n_pts.end(), fallback_points.begin(), fallback_points.end());
  }
}

void FeatureTracker::addPoints() {
  for (auto &p : n_pts) {
    cur_pts.push_back(p);
    ids.push_back(n_id++);
    track_cnt.push_back(1);
  }
}

double FeatureTracker::distance(cv::Point2f &pt1, cv::Point2f &pt2) {
  double dx = pt1.x - pt2.x;
  double dy = pt1.y - pt2.y;
  return sqrt(dx * dx + dy * dy);
}

map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>
FeatureTracker::trackImage(double _cur_time, const cv::Mat &_img,
                           const cv::Mat &_img1) {
  TicToc t_r;
  cur_time = _cur_time;
  cur_img = _img;
  row = cur_img.rows;
  col = cur_img.cols;
  cv::Mat rightImg = _img1;
  cur_pts.clear();

  if (prev_pts.size() > 0) {
    vector<uchar> status;
    if (!options->USE_GPU_ACC_FLOW) {
      TicToc t_o;

      vector<float> err;
      if (hasPrediction) {
        cur_pts = predict_pts;
        cv::calcOpticalFlowPyrLK(
            prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21),
            1,
            cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
                             30, 0.01),
            cv::OPTFLOW_USE_INITIAL_FLOW);

        int succ_num = 0;
        for (size_t i = 0; i < status.size(); i++) {
          if (status[i]) succ_num++;
        }
        if (succ_num < 10)
          cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status,
                                   err, cv::Size(21, 21), 3);
      } else
        cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status,
                                 err, cv::Size(21, 21), 3);
      // reverse check
      if (options->enable_reverse_optical_flow_check) {
        vector<uchar> reverse_status;
        vector<cv::Point2f> reverse_pts = prev_pts;
        cv::calcOpticalFlowPyrLK(
            cur_img, prev_img, cur_pts, reverse_pts, reverse_status, err,
            cv::Size(21, 21), 1,
            cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
                             30, 0.01),
            cv::OPTFLOW_USE_INITIAL_FLOW);
        // cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts, reverse_pts,
        // reverse_status, err, cv::Size(21, 21), 3);
        for (size_t i = 0; i < status.size(); i++) {
          if (status[i] && reverse_status[i] &&
              distance(prev_pts[i], reverse_pts[i]) <= 0.5) {
            status[i] = 1;
          } else
            status[i] = 0;
        }
      }
      // printf("temporal optical flow costs: %fms\n", t_o.toc());
    }
#ifdef GPU_MODE
    else {
      TicToc t_og;
      cv::cuda::GpuMat prev_gpu_img(prev_img);
      cv::cuda::GpuMat cur_gpu_img(cur_img);
      cv::cuda::GpuMat prev_gpu_pts(prev_pts);
      cv::cuda::GpuMat cur_gpu_pts(cur_pts);
      cv::cuda::GpuMat gpu_status;
      if (hasPrediction) {
        cur_gpu_pts = cv::cuda::GpuMat(predict_pts);
        cv::Ptr<cv::cuda::SparsePyrLKOpticalFlow> d_pyrLK_sparse =
            cv::cuda::SparsePyrLKOpticalFlow::create(cv::Size(21, 21), 1, 30,
                                                     true);
        d_pyrLK_sparse->calc(prev_gpu_img, cur_gpu_img, prev_gpu_pts,
                             cur_gpu_pts, gpu_status);

        vector<cv::Point2f> tmp_cur_pts(cur_gpu_pts.cols);
        cur_gpu_pts.download(tmp_cur_pts);
        cur_pts = tmp_cur_pts;

        vector<uchar> tmp_status(gpu_status.cols);
        gpu_status.download(tmp_status);
        status = tmp_status;

        int succ_num = 0;
        for (size_t i = 0; i < tmp_status.size(); i++) {
          if (tmp_status[i]) succ_num++;
        }
        if (succ_num < 10) {
          cv::Ptr<cv::cuda::SparsePyrLKOpticalFlow> d_pyrLK_sparse =
              cv::cuda::SparsePyrLKOpticalFlow::create(cv::Size(21, 21), 3, 30,
                                                       false);
          d_pyrLK_sparse->calc(prev_gpu_img, cur_gpu_img, prev_gpu_pts,
                               cur_gpu_pts, gpu_status);

          vector<cv::Point2f> tmp1_cur_pts(cur_gpu_pts.cols);
          cur_gpu_pts.download(tmp1_cur_pts);
          cur_pts = tmp1_cur_pts;

          vector<uchar> tmp1_status(gpu_status.cols);
          gpu_status.download(tmp1_status);
          status = tmp1_status;
        }
      } else {
        cv::Ptr<cv::cuda::SparsePyrLKOpticalFlow> d_pyrLK_sparse =
            cv::cuda::SparsePyrLKOpticalFlow::create(cv::Size(21, 21), 3, 30,
                                                     false);
        d_pyrLK_sparse->calc(prev_gpu_img, cur_gpu_img, prev_gpu_pts,
                             cur_gpu_pts, gpu_status);

        vector<cv::Point2f> tmp1_cur_pts(cur_gpu_pts.cols);
        cur_gpu_pts.download(tmp1_cur_pts);
        cur_pts = tmp1_cur_pts;

        vector<uchar> tmp1_status(gpu_status.cols);
        gpu_status.download(tmp1_status);
        status = tmp1_status;
      }
      if (options->enable_reverse_optical_flow_check) {
        cv::cuda::GpuMat reverse_gpu_status;
        cv::cuda::GpuMat reverse_gpu_pts = prev_gpu_pts;
        cv::Ptr<cv::cuda::SparsePyrLKOpticalFlow> d_pyrLK_sparse =
            cv::cuda::SparsePyrLKOpticalFlow::create(cv::Size(21, 21), 1, 30,
                                                     true);
        d_pyrLK_sparse->calc(cur_gpu_img, prev_gpu_img, cur_gpu_pts,
                             reverse_gpu_pts, reverse_gpu_status);

        vector<cv::Point2f> reverse_pts(reverse_gpu_pts.cols);
        reverse_gpu_pts.download(reverse_pts);

        vector<uchar> reverse_status(reverse_gpu_status.cols);
        reverse_gpu_status.download(reverse_status);

        for (size_t i = 0; i < status.size(); i++) {
          if (status[i] && reverse_status[i] &&
              distance(prev_pts[i], reverse_pts[i]) <= 0.5) {
            status[i] = 1;
          } else
            status[i] = 0;
        }
      }
      // printf("gpu temporal optical flow costs: %f ms\n",t_og.toc());
    }
#endif

    for (int i = 0; i < int(cur_pts.size()); i++)
      if (status[i] && !inBorder(cur_pts[i])) status[i] = 0;
    reduceVector(prev_pts, status);
    reduceVector(cur_pts, status);
    reduceVector(ids, status);
    reduceVector(track_cnt, status);
  }

  for (auto &n : track_cnt) n++;

  if (1) {
    // rejectWithF();
    TicToc t_m;
    setMask();
    int n_max_cnt =
        options->max_feature_count - static_cast<int>(cur_pts.size());
    if (options->hasStereoTrackDiversity()) {
      if (n_max_cnt > 0)
        detectDiversePoints();
      else
        n_pts.clear();
    } else if (!options->USE_GPU) {
      if (n_max_cnt > 0) {
        TicToc t_t;
        if (mask.empty()) VINS_WARN << "mask is empty ";
        if (mask.type() != CV_8UC1) VINS_WARN << "mask type wrong ";
        cv::goodFeaturesToTrack(cur_img, n_pts,
                                options->max_feature_count - cur_pts.size(),
                                0.01, options->min_feature_distance, mask);
      } else {
        n_pts.clear();
      }
    }
#ifdef GPU_MODE
    // ROS_DEBUG("detect feature costs: %fms", t_t.toc());
    // printf("good feature to track costs: %fms\n", t_t.toc());
    else {
      if (n_max_cnt > 0) {
        if (mask.empty()) cout << "mask is empty " << endl;
        if (mask.type() != CV_8UC1) cout << "mask type wrong " << endl;
        TicToc t_g;
        cv::cuda::GpuMat cur_gpu_img(cur_img);
        cv::cuda::GpuMat d_prevPts;
        TicToc t_gg;
        cv::cuda::GpuMat gpu_mask(mask);
        // printf("gpumat cost: %fms\n",t_gg.toc());
        cv::Ptr<cv::cuda::CornersDetector> detector =
            cv::cuda::createGoodFeaturesToTrackDetector(
                cur_gpu_img.type(), options->max_feature_count - cur_pts.size(),
                0.01, options->min_feature_distance);
        // cout << "new gpu points: "<< options->MAX_CNT - cur_pts.size()<<endl;
        detector->detect(cur_gpu_img, d_prevPts, gpu_mask);
        // std::cout << "d_prevPts size: "<< d_prevPts.size()<<std::endl;
        if (!d_prevPts.empty())
          n_pts = cv::Mat_<cv::Point2f>(cv::Mat(d_prevPts));
        else
          n_pts.clear();
        // sum_n += n_pts.size();
        // printf("total point from gpu: %d\n",sum_n);
        // printf("gpu good feature to track cost: %fms\n", t_g.toc());
      } else
        n_pts.clear();
    }
#endif
    TicToc t_a;
    addPoints();
  }

  cur_un_pts = undistortedPts(cur_pts, m_camera[0]);
  pts_velocity = ptsVelocity(ids, cur_un_pts, cur_un_pts_map, prev_un_pts_map);

  if (!_img1.empty() && stereo_cam) {
    ids_right.clear();
    cur_right_pts.clear();
    cur_un_right_pts.clear();
    right_pts_velocity.clear();
    cur_un_right_pts_map.clear();
    if (!cur_pts.empty()) {
      vector<cv::Point2f> reverseLeftPts;
      vector<uchar> status, statusRightLeft;
      if (!options->USE_GPU_ACC_FLOW) {
        TicToc t_check;
        vector<float> err;
        // cur left ---- cur right
        cv::calcOpticalFlowPyrLK(cur_img, rightImg, cur_pts, cur_right_pts,
                                 status, err, cv::Size(21, 21), 3);
        // reverse check cur right ---- cur left
        if (options->enable_reverse_optical_flow_check) {
          cv::calcOpticalFlowPyrLK(rightImg, cur_img, cur_right_pts,
                                   reverseLeftPts, statusRightLeft, err,
                                   cv::Size(21, 21), 3);
          for (size_t i = 0; i < status.size(); i++) {
            if (status[i] && statusRightLeft[i] && inBorder(cur_right_pts[i]) &&
                distance(cur_pts[i], reverseLeftPts[i]) <= 0.5)
              status[i] = 1;
            else
              status[i] = 0;
          }
        }
      }
#ifdef GPU_MODE
      else {
        TicToc t_og1;
        cv::cuda::GpuMat cur_gpu_img(cur_img);
        cv::cuda::GpuMat right_gpu_Img(rightImg);
        cv::cuda::GpuMat cur_gpu_pts(cur_pts);
        cv::cuda::GpuMat cur_right_gpu_pts;
        cv::cuda::GpuMat gpu_status;
        cv::Ptr<cv::cuda::SparsePyrLKOpticalFlow> d_pyrLK_sparse =
            cv::cuda::SparsePyrLKOpticalFlow::create(cv::Size(21, 21), 3, 30,
                                                     false);
        d_pyrLK_sparse->calc(cur_gpu_img, right_gpu_Img, cur_gpu_pts,
                             cur_right_gpu_pts, gpu_status);

        vector<cv::Point2f> tmp_cur_right_pts(cur_right_gpu_pts.cols);
        cur_right_gpu_pts.download(tmp_cur_right_pts);
        cur_right_pts = tmp_cur_right_pts;

        vector<uchar> tmp_status(gpu_status.cols);
        gpu_status.download(tmp_status);
        status = tmp_status;

        if (options->enable_reverse_optical_flow_check) {
          cv::cuda::GpuMat reverseLeft_gpu_Pts;
          cv::cuda::GpuMat status_gpu_RightLeft;
          cv::Ptr<cv::cuda::SparsePyrLKOpticalFlow> d_pyrLK_sparse =
              cv::cuda::SparsePyrLKOpticalFlow::create(cv::Size(21, 21), 3, 30,
                                                       false);
          d_pyrLK_sparse->calc(right_gpu_Img, cur_gpu_img, cur_right_gpu_pts,
                               reverseLeft_gpu_Pts, status_gpu_RightLeft);

          vector<cv::Point2f> tmp_reverseLeft_Pts(reverseLeft_gpu_Pts.cols);
          reverseLeft_gpu_Pts.download(tmp_reverseLeft_Pts);
          reverseLeftPts = tmp_reverseLeft_Pts;

          vector<uchar> tmp1_status(status_gpu_RightLeft.cols);
          status_gpu_RightLeft.download(tmp1_status);
          statusRightLeft = tmp1_status;
          for (size_t i = 0; i < status.size(); i++) {
            if (status[i] && statusRightLeft[i] && inBorder(cur_right_pts[i]) &&
                distance(cur_pts[i], reverseLeftPts[i]) <= 0.5)
              status[i] = 1;
            else
              status[i] = 0;
          }
        }
        // printf("gpu left right optical flow cost %fms\n",t_og1.toc());
      }
#endif
      const double epipolar_threshold =
          replayStereoEpipolarMaxVerticalPx();
      if (std::isfinite(epipolar_threshold)) {
        int rejected = 0;
        double maximum_vertical_error = 0.0;
        if (status.size() != cur_pts.size() ||
            cur_right_pts.size() != cur_pts.size()) {
          throw std::runtime_error(
              "stereo epipolar gate received inconsistent point vectors");
        }
        for (std::size_t i = 0; i < status.size(); ++i) {
          if (!status[i]) continue;
          const double vertical_error =
              std::abs(static_cast<double>(cur_pts[i].y) -
                       static_cast<double>(cur_right_pts[i].y));
          maximum_vertical_error =
              std::max(maximum_vertical_error, vertical_error);
          if (vertical_error > epipolar_threshold) {
            status[i] = 0;
            ++rejected;
          }
        }
        if (rejected > 0) {
          VINS_WARN << "Replay-only stereo epipolar gate rejected: count="
                    << rejected << ", max_vertical_px="
                    << maximum_vertical_error << ", threshold_px="
                    << epipolar_threshold;
        }
      }
      ids_right = ids;
      reduceVector(cur_right_pts, status);
      reduceVector(ids_right, status);

      cur_un_right_pts = undistortedPts(cur_right_pts, m_camera[1]);
      right_pts_velocity =
          ptsVelocity(ids_right, cur_un_right_pts, cur_un_right_pts_map,
                      prev_un_right_pts_map);
    }
    prev_un_right_pts_map = cur_un_right_pts_map;
  }
  if (options->hasStereoTrackDiversity()) {
    updateStereoDepthBins();
    traceStereoTrackDiversity();
  }
  if (options->shouldTraceFeatureFrame(cur_time)) {
    traceProductionFeatureFrame();
  }
  if (options->shouldShowTrack()) {
    drawTrack(cur_img, rightImg, ids, cur_pts, cur_right_pts, prevLeftPtsMap);
  }

  prev_img = cur_img;
  prev_pts = cur_pts;
  prev_un_pts = cur_un_pts;
  prev_un_pts_map = cur_un_pts_map;
  prev_time = cur_time;
  hasPrediction = false;

  prevLeftPtsMap.clear();
  for (size_t i = 0; i < cur_pts.size(); i++)
    prevLeftPtsMap[ids[i]] = cur_pts[i];

  map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
  for (size_t i = 0; i < ids.size(); i++) {
    int feature_id = ids[i];
    double x, y, z;
    x = cur_un_pts[i].x;
    y = cur_un_pts[i].y;
    z = 1;
    double p_u, p_v;
    p_u = cur_pts[i].x;
    p_v = cur_pts[i].y;
    int camera_id = 0;
    double velocity_x, velocity_y;
    velocity_x = pts_velocity[i].x;
    velocity_y = pts_velocity[i].y;

    Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
    xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
    featureFrame[feature_id].emplace_back(camera_id, xyz_uv_velocity);
  }

  if (!_img1.empty() && stereo_cam) {
    for (size_t i = 0; i < ids_right.size(); i++) {
      int feature_id = ids_right[i];
      double x, y, z;
      x = cur_un_right_pts[i].x;
      y = cur_un_right_pts[i].y;
      z = 1;
      double p_u, p_v;
      p_u = cur_right_pts[i].x;
      p_v = cur_right_pts[i].y;
      int camera_id = 1;
      double velocity_x, velocity_y;
      velocity_x = right_pts_velocity[i].x;
      velocity_y = right_pts_velocity[i].y;

      Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
      xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
      featureFrame[feature_id].emplace_back(camera_id, xyz_uv_velocity);
    }
  }

  return featureFrame;
}

void FeatureTracker::updateStereoDepthBins() {
  map<int, std::size_t> left_index_by_id;
  for (std::size_t index = 0; index < ids.size(); ++index) {
    left_index_by_id[ids[index]] = index;
  }
  map<int, int> current_bins;
  const double baseline =
      options->TIC.size() >= 2
          ? (options->TIC[1] - options->TIC[0]).norm()
          : 0.0;
  if (baseline > std::numeric_limits<double>::epsilon()) {
    for (std::size_t right_index = 0;
         right_index < ids_right.size(); ++right_index) {
      const auto left = left_index_by_id.find(ids_right[right_index]);
      if (left == left_index_by_id.end()) continue;
      const double normalized_disparity =
          std::abs(
              static_cast<double>(cur_un_pts[left->second].x) -
              static_cast<double>(cur_un_right_pts[right_index].x));
      if (normalized_disparity <= 1e-6) continue;
      current_bins[ids_right[right_index]] =
          depthBin(baseline / normalized_disparity);
    }
  }
  stereo_depth_bin_by_id = std::move(current_bins);
}

void FeatureTracker::traceStereoTrackDiversity() const {
  const int cell_count =
      options->stereo_track_grid_rows * options->stereo_track_grid_cols;
  vector<int> grid_counts(cell_count, 0);
  for (const auto &point : cur_pts) ++grid_counts[gridCell(point)];
  std::array<int, 5> depth_counts{{0, 0, 0, 0, 0}};
  for (const int id : ids_right) {
    const auto depth = stereo_depth_bin_by_id.find(id);
    if (depth == stereo_depth_bin_by_id.end())
      ++depth_counts[0];
    else
      ++depth_counts[std::max(0, std::min(4, depth->second))];
  }
  std::ostringstream grid_stream;
  for (std::size_t index = 0; index < grid_counts.size(); ++index) {
    if (index) grid_stream << ",";
    grid_stream << grid_counts[index];
  }
  VINS_INFO << "Stereo track diversity trace: timestamp=" << cur_time
            << ", left=" << ids.size()
            << ", stereo=" << ids_right.size()
            << ", depth_bins_unknown_near_mid_far_veryfar="
            << depth_counts[0] << "," << depth_counts[1] << ","
            << depth_counts[2] << "," << depth_counts[3] << ","
            << depth_counts[4]
            << ", grid=" << grid_stream.str();
}

void FeatureTracker::traceProductionFeatureFrame() const {
  map<int, std::size_t> right_index_by_id;
  for (std::size_t index = 0; index < ids_right.size(); ++index) {
    right_index_by_id[ids_right[index]] = index;
  }
  std::ostringstream observations;
  observations << std::setprecision(9);
  const double baseline =
      options->TIC.size() >= 2
          ? (options->TIC[1] - options->TIC[0]).norm()
          : 0.0;
  for (std::size_t left_index = 0; left_index < ids.size(); ++left_index) {
    if (left_index) observations << ";";
    const int id = ids[left_index];
    observations << id << ":" << track_cnt[left_index] << ":"
                 << cur_pts[left_index].x << ":"
                 << cur_pts[left_index].y << ":";
    const auto right = right_index_by_id.find(id);
    if (right == right_index_by_id.end()) {
      observations << "nan:nan:nan";
      continue;
    }
    const double normalized_disparity =
        std::abs(
            static_cast<double>(cur_un_pts[left_index].x) -
            static_cast<double>(cur_un_right_pts[right->second].x));
    const double depth =
        baseline > 0.0 && normalized_disparity > 1e-6
            ? baseline / normalized_disparity
            : std::numeric_limits<double>::quiet_NaN();
    observations << cur_right_pts[right->second].x << ":"
                 << cur_right_pts[right->second].y << ":" << depth;
  }
  VINS_INFO << "Production feature frame trace: timestamp=" << cur_time
            << ", observations=" << observations.str();
}

void FeatureTracker::rejectWithF() {
  if (cur_pts.size() >= 8) {
    TicToc t_f;
    vector<cv::Point2f> un_cur_pts(cur_pts.size()),
        un_prev_pts(prev_pts.size());
    for (unsigned int i = 0; i < cur_pts.size(); i++) {
      Eigen::Vector3d tmp_p;
      m_camera[0]->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y),
                                  tmp_p);
      tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + col / 2.0;
      tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + row / 2.0;
      un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

      m_camera[0]->liftProjective(Eigen::Vector2d(prev_pts[i].x, prev_pts[i].y),
                                  tmp_p);
      tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + col / 2.0;
      tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + row / 2.0;
      un_prev_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
    }

    vector<uchar> status;
    cv::findFundamentalMat(un_cur_pts, un_prev_pts, cv::FM_RANSAC,
                           options->ransac_reproj_threshold, 0.99, status);
    int size_a = cur_pts.size();
    reduceVector(prev_pts, status);
    reduceVector(cur_pts, status);
    reduceVector(cur_un_pts, status);
    reduceVector(ids, status);
    reduceVector(track_cnt, status);
  }
}

void FeatureTracker::readIntrinsicParameter(const vector<string> &calib_file) {
  m_camera.clear();
  for (size_t i = 0; i < calib_file.size(); i++) {
    camodocal::CameraPtr camera =
        CameraFactory::instance()->generateCameraFromYamlFile(calib_file[i]);
    m_camera.push_back(camera);
  }
  if (calib_file.size() == 2) stereo_cam = 1;
}

void FeatureTracker::showUndistortion(const string &name) {
  cv::Mat undistortedImg(row + 600, col + 600, CV_8UC1, cv::Scalar(0));
  vector<Eigen::Vector2d> distortedp, undistortedp;
  for (int i = 0; i < col; i++)
    for (int j = 0; j < row; j++) {
      Eigen::Vector2d a(i, j);
      Eigen::Vector3d b;
      m_camera[0]->liftProjective(a, b);
      distortedp.push_back(a);
      undistortedp.push_back(Eigen::Vector2d(b.x() / b.z(), b.y() / b.z()));
    }
  for (int i = 0; i < int(undistortedp.size()); i++) {
    cv::Mat pp(3, 1, CV_32FC1);
    pp.at<float>(0, 0) = undistortedp[i].x() * FOCAL_LENGTH + col / 2;
    pp.at<float>(1, 0) = undistortedp[i].y() * FOCAL_LENGTH + row / 2;
    pp.at<float>(2, 0) = 1.0;
    if (pp.at<float>(1, 0) + 300 >= 0 && pp.at<float>(1, 0) + 300 < row + 600 &&
        pp.at<float>(0, 0) + 300 >= 0 && pp.at<float>(0, 0) + 300 < col + 600) {
      undistortedImg.at<uchar>(pp.at<float>(1, 0) + 300,
                               pp.at<float>(0, 0) + 300) =
          cur_img.at<uchar>(distortedp[i].y(), distortedp[i].x());
    } else {
      // ROS_ERROR("(%f %f) -> (%f %f)", distortedp[i].y, distortedp[i].x,
      // pp.at<float>(1, 0), pp.at<float>(0, 0));
    }
  }
  // turn the following code on if you need
  // cv::imshow(name, undistortedImg);
  // cv::waitKey(0);
}

vector<cv::Point2f> FeatureTracker::undistortedPts(vector<cv::Point2f> &pts,
                                                   camodocal::CameraPtr cam) {
  vector<cv::Point2f> un_pts;
  for (unsigned int i = 0; i < pts.size(); i++) {
    Eigen::Vector2d a(pts[i].x, pts[i].y);
    Eigen::Vector3d b;
    cam->liftProjective(a, b);
    un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
  }
  return un_pts;
}

vector<cv::Point2f> FeatureTracker::ptsVelocity(
    vector<int> &ids, vector<cv::Point2f> &pts,
    map<int, cv::Point2f> &cur_id_pts, map<int, cv::Point2f> &prev_id_pts) {
  vector<cv::Point2f> pts_velocity;
  cur_id_pts.clear();
  for (unsigned int i = 0; i < ids.size(); i++) {
    cur_id_pts.insert(make_pair(ids[i], pts[i]));
  }

  // caculate points velocity
  if (!prev_id_pts.empty()) {
    double dt = cur_time - prev_time;

    for (unsigned int i = 0; i < pts.size(); i++) {
      std::map<int, cv::Point2f>::iterator it;
      it = prev_id_pts.find(ids[i]);
      if (it != prev_id_pts.end()) {
        double v_x = (pts[i].x - it->second.x) / dt;
        double v_y = (pts[i].y - it->second.y) / dt;
        pts_velocity.push_back(cv::Point2f(v_x, v_y));
      } else
        pts_velocity.push_back(cv::Point2f(0, 0));
    }
  } else {
    for (unsigned int i = 0; i < cur_pts.size(); i++) {
      pts_velocity.push_back(cv::Point2f(0, 0));
    }
  }
  return pts_velocity;
}

void FeatureTracker::drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight,
                               vector<int> &curLeftIds,
                               vector<cv::Point2f> &curLeftPts,
                               vector<cv::Point2f> &curRightPts,
                               map<int, cv::Point2f> &prevLeftPtsMap) {
  // int rows = imLeft.rows;
  int cols = imLeft.cols;
  if (!imRight.empty() && stereo_cam)
    cv::hconcat(imLeft, imRight, imTrack);
  else
    imTrack = imLeft.clone();
  cv::cvtColor(imTrack, imTrack, cv::COLOR_GRAY2RGB);

  for (size_t j = 0; j < curLeftPts.size(); j++) {
    double len = std::min(1.0, 1.0 * track_cnt[j] / 20);
    cv::circle(imTrack, curLeftPts[j], 2,
               cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
  }
  if (!imRight.empty() && stereo_cam) {
    for (size_t i = 0; i < curRightPts.size(); i++) {
      cv::Point2f rightPt = curRightPts[i];
      rightPt.x += cols;
      cv::circle(imTrack, rightPt, 2, cv::Scalar(0, 255, 0), 2);
      // cv::Point2f leftPt = curLeftPtsTrackRight[i];
      // cv::line(imTrack, leftPt, rightPt, cv::Scalar(0, 255, 0), 1, 8, 0);
    }
  }

  map<int, cv::Point2f>::iterator mapIt;
  for (size_t i = 0; i < curLeftIds.size(); i++) {
    int id = curLeftIds[i];
    mapIt = prevLeftPtsMap.find(id);
    if (mapIt != prevLeftPtsMap.end()) {
      cv::arrowedLine(imTrack, curLeftPts[i], mapIt->second,
                      cv::Scalar(0, 255, 0), 1, 8, 0, 0.2);
    }
  }
}

void FeatureTracker::setPrediction(map<int, Eigen::Vector3d> &predictPts) {
  hasPrediction = true;
  predict_pts.clear();
  predict_pts_debug.clear();
  map<int, Eigen::Vector3d>::iterator itPredict;
  for (size_t i = 0; i < ids.size(); i++) {
    int id = ids[i];
    itPredict = predictPts.find(id);
    if (itPredict != predictPts.end()) {
      Eigen::Vector2d tmp_uv;
      m_camera[0]->spaceToPlane(itPredict->second, tmp_uv);
      predict_pts.push_back(cv::Point2f(tmp_uv.x(), tmp_uv.y()));
      predict_pts_debug.push_back(cv::Point2f(tmp_uv.x(), tmp_uv.y()));
    } else
      predict_pts.push_back(prev_pts[i]);
  }
}

void FeatureTracker::removeOutliers(set<int> &removePtsIds) {
  std::set<int>::iterator itSet;
  vector<uchar> status;
  for (size_t i = 0; i < ids.size(); i++) {
    itSet = removePtsIds.find(ids[i]);
    if (itSet != removePtsIds.end())
      status.push_back(0);
    else
      status.push_back(1);
  }

  reduceVector(prev_pts, status);
  reduceVector(ids, status);
  reduceVector(track_cnt, status);
}

cv::Mat FeatureTracker::getTrackImage() { return imTrack; }
