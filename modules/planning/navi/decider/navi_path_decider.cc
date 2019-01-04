/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 * @brief This file provides the implementation of the class "NaviPathDecider".
 */

#include "modules/planning/navi/decider/navi_path_decider.h"

#include <algorithm>
#include <limits>
#include <utility>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/log.h"
#include "modules/common/math/cartesian_frenet_conversion.h"
#include "modules/common/math/vec2d.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/math/curve1d/quintic_polynomial_curve1d.h"
#include "modules/planning/proto/sl_boundary.pb.h"

namespace apollo {
namespace planning {

using apollo::common::Status;
using apollo::common::math::Box2d;
using apollo::common::math::CartesianFrenetConverter;
using apollo::common::math::Vec2d;

namespace {
constexpr double kStatisticLaneLength = 10.0;
constexpr std::size_t kStatisticPointsNum = 3;
}  // namespace

NaviPathDecider::NaviPathDecider() : NaviTask("NaviPathDecider") {
  // TODO(all): Add your other initialization.
}

bool NaviPathDecider::Init(const PlanningConfig& config) {
  CheckConfig(config);
  move_dest_lane_config_talbe_.clear();
  max_speed_levels_.clear();
  config_ = config.planner_navi_config().navi_path_decider_config();
  auto move_dest_lane_config_talbe = config_.move_dest_lane_config_talbe();
  for (const auto& item : move_dest_lane_config_talbe.lateral_shift()) {
    double max_speed_level = item.max_speed();
    double max_move_dest_lane_shift_y = item.max_move_dest_lane_shift_y();
    if (move_dest_lane_config_talbe_.find(max_speed_level) ==
        move_dest_lane_config_talbe_.end()) {
      move_dest_lane_config_talbe_.emplace(
          std::make_pair(max_speed_level, max_move_dest_lane_shift_y));
      max_speed_levels_.push_back(max_speed_level);
    }
  }
  AINFO << "Maximum speeds and move to dest lane config: ";
  for (const auto& data : move_dest_lane_config_talbe_) {
    auto max_speed = data.first;
    auto max_move_dest_lane_shift_y = data.second;
    AINFO << "[max_speed : " << max_speed
          << " ,max move dest lane shift y : " << max_move_dest_lane_shift_y
          << "]";
  }

  is_init_ = obstacle_decider_.Init(config);
  return is_init_;
}

void NaviPathDecider::CheckConfig(const PlanningConfig& config) {
  CHECK(config.has_planner_navi_config());
  CHECK(config.planner_navi_config().has_navi_path_decider_config());
  config_ = config.planner_navi_config().navi_path_decider_config();
  CHECK(config_.has_move_dest_lane_config_talbe());
  CHECK(config_.has_max_keep_lane_distance());
  CHECK(config_.has_max_keep_lane_shift_y());
  CHECK(config_.has_min_keep_lane_offset());
  CHECK(config_.has_keep_lane_shift_compensation());
  // CHECK(config_.has_enable_plan_start_point());
  CHECK(config_.has_move_dest_lane_compensation());
  CHECK(config_.has_max_kappa_threshold());
  CHECK(config_.has_move_dest_lane_compensation_by_kappa());
  CHECK(config_.has_basic_path_lateral_compensation());
  CHECK(config_.has_max_nudge_distance());
  CHECK(config_.has_nudge_compensation());
  CHECK(config_.has_enable_quintic_polynomial_curve_path());
  CHECK(config_.has_move_dest_lane_longitudinal_coef());
  CHECK(config_.has_move_dest_lane_velocity_coef());
  CHECK(config_.has_move_dest_lane_kappa_coef());
  CHECK(config_.has_move_dest_lane_longitudinal_buffer());
  CHECK(config_.has_dense_sample_resolution());
  CHECK(config_.has_sparse_sample_resolution());
}

Status NaviPathDecider::Execute(Frame* frame,
                                ReferenceLineInfo* const reference_line_info) {
  NaviTask::Execute(frame, reference_line_info);
  vehicle_state_ = frame->vehicle_state();
  cur_reference_line_lane_id_ = reference_line_info->Lanes().Id();
  auto ret = Process(reference_line_info->reference_line(),
                     frame->PlanningStartPoint(), frame->obstacles(),
                     reference_line_info->path_decision(),
                     reference_line_info->mutable_path_data());
  RecordDebugInfo(reference_line_info->path_data());
  if (ret != Status::OK()) {
    reference_line_info->SetDrivable(false);
    AERROR << "Reference Line " << reference_line_info->Lanes().Id()
           << " is not drivable after " << Name();
  }
  return ret;
}

apollo::common::Status NaviPathDecider::Process(
    const ReferenceLine& reference_line,
    const common::TrajectoryPoint& init_point,
    const std::vector<const Obstacle*>& obstacles,
    PathDecision* const path_decision, PathData* const path_data) {
  CHECK_NOTNULL(path_decision);
  CHECK_NOTNULL(path_data);
  start_plan_point_.set_x(vehicle_state_.x());
  start_plan_point_.set_y(vehicle_state_.y());
  start_plan_point_.set_theta(vehicle_state_.heading());
  start_plan_v_ = vehicle_state_.linear_velocity();
  start_plan_a_ = vehicle_state_.linear_acceleration();
  if (config_.enable_plan_start_point()) {
    // start plan point from planning schedule
    start_plan_point_.set_x(init_point.path_point().x());
    start_plan_point_.set_y(init_point.path_point().y());
    start_plan_point_.set_theta(init_point.path_point().theta());
    start_plan_v_ = init_point.v();
    start_plan_a_ = init_point.a();
  }

  // intercept path points from reference line
  std::vector<apollo::common::PathPoint> path_points;
  if (!GetBasicPathData(reference_line, &path_points)) {
    AERROR << "Get path points from reference line failed";
    return Status(apollo::common::ErrorCode::PLANNING_ERROR,
                  "NaviPathDecider GetBasicPathData");
  }

  // according to the position of the start plan point and the reference line,
  // the path trajectory intercepted from the reference line is shifted on the
  // y-axis to adc.
  double dest_ref_line_y = path_points[0].y();

  ADEBUG << "In current plan cycle, adc to ref line distance : "
         << dest_ref_line_y << " lane id : " << cur_reference_line_lane_id_;
  if (config_.enable_quintic_polynomial_curve_path() &&
      IsLaneTooBend(reference_line)) {
    MoveToDestLane(dest_ref_line_y, reference_line, &path_points);
  } else {
    MoveToDestLane(dest_ref_line_y, &path_points);
  }
  KeepLane(dest_ref_line_y, &path_points);

  DiscretizedPath discretized_path(path_points);
  path_data->SetReferenceLine(&(reference_line_info_->reference_line()));
  if (!path_data->SetDiscretizedPath(discretized_path)) {
    AERROR << "Set path data failed.";
    return Status(apollo::common::ErrorCode::PLANNING_ERROR,
                  "NaviPathDecider SetDiscretizedPath");
  }

  return Status::OK();
}

void NaviPathDecider::MoveToDestLane(
    const double dest_ref_line_y,
    std::vector<common::PathPoint>* const path_points) {
  double dest_lateral_distance = std::fabs(dest_ref_line_y);
  if (dest_lateral_distance < config_.max_keep_lane_distance()) {
    return;
  }

  // calculate lateral shift range and theta chage ratio
  double max_shift_y = CalculateDistanceToDestLane();

  // The steering wheel is more sensitive to the left than to the right and
  // requires a compensation value to the right
  max_shift_y = dest_ref_line_y > 0.0
                    ? max_shift_y
                    : (max_shift_y + config_.move_dest_lane_compensation());

  // Go against the curvature, need a lateral shift compensation
  double start_point_kappa = (*path_points)[0].kappa();
  bool bNeedKappaCompensation = false;
  if ((start_point_kappa >= config_.max_kappa_threshold() &&
       dest_ref_line_y < 0.0) ||
      (start_point_kappa <= -config_.max_kappa_threshold() &&
       dest_ref_line_y > 0.0)) {
    bNeedKappaCompensation = true;
    ADEBUG << "In current plan cycle move to dest lane, need kappa "
              "lateral shift compensation, kappa : "
           << start_point_kappa;
  }
  max_shift_y =
      bNeedKappaCompensation
          ? (max_shift_y + config_.move_dest_lane_compensation_by_kappa())
          : max_shift_y;

  double actual_start_point_y = std::copysign(max_shift_y, dest_ref_line_y);

  // lateral shift path_points to the expect position(max_shift_y or max_shift_y
  // + move_dest_lane_compensation_ + kappa_move_dest_lane_compensation_)
  double lateral_shift_value = -dest_ref_line_y + actual_start_point_y;

  ADEBUG << "In current plan cycle move to dest lane, adc shift to dest "
            "reference line : "
         << lateral_shift_value;

  std::transform(path_points->begin(), path_points->end(), path_points->begin(),
                 [lateral_shift_value](common::PathPoint& old_path_point) {
                   common::PathPoint new_path_point = old_path_point;
                   double new_path_point_y =
                       old_path_point.y() + lateral_shift_value;
                   new_path_point.set_y(new_path_point_y);
                   return new_path_point;
                 });

  return;
}

void NaviPathDecider::MoveToDestLane(
    const double dest_ref_line_y, const ReferenceLine& reference_line,
    std::vector<common::PathPoint>* const path_points) {
  double dest_lateral_distance = std::fabs(dest_ref_line_y);
  if (dest_lateral_distance < config_.max_keep_lane_distance()) {
    return;
  }
  CHECK_NOTNULL(path_points);
  ADEBUG << "In current plan cycle move to dest lane by Quintic Polynomial "
            "Curve, adc shift to dest "
            "reference line : ";

  // convert Cartesian point to Frenet point
  common::FrenetFramePoint init_frenet_point;
  if (!CalculateFrenetPoint(reference_line, start_plan_point_,
                            &init_frenet_point)) {
    AERROR << "Failed to convert path point to frent point.";
    path_points->clear();
    return;
  }

  // adjust the dl of init_frenet_point according to config file.
  double compute_dl = init_frenet_point.dl();
  double delta_dl_by_velocity =
      config_.move_curve_dl_velocity_coef() * start_plan_v_;
  double move_to_left_curve_start_dl =
      config_.move_to_left_curve_start_dl() - delta_dl_by_velocity;
  move_to_left_curve_start_dl =
      move_to_left_curve_start_dl < 0.0 ? 0.0 : move_to_left_curve_start_dl;
  double move_to_right_curve_start_dl =
      config_.move_to_right_curve_start_dl() + delta_dl_by_velocity;
  move_to_right_curve_start_dl =
      move_to_right_curve_start_dl > 0.0 ? 0.0 : move_to_right_curve_start_dl;
  double config_dl = init_frenet_point.l() < 0.0 ? move_to_left_curve_start_dl
                                                 : move_to_right_curve_start_dl;
  // double init_dl = std::max(std::fabs(config_dl), std::fabs(compute_dl));
  double init_dl = std::fabs(compute_dl) > std::fabs(config_dl)
                       ? std::fabs(compute_dl)
                       : 0.5 * (std::fabs(compute_dl) + std::fabs(config_dl));
  init_dl = std::copysign(init_dl, config_dl);
  init_frenet_point.set_dl(init_dl);

  // calculate the legth of Quintic Polynomial Curve
  double curve_path_length =
      config_.move_dest_lane_longitudinal_coef() * dest_lateral_distance +
      config_.move_dest_lane_longitudinal_buffer() +
      config_.move_dest_lane_velocity_coef() * std::pow(start_plan_v_, 2) +
      config_.move_dest_lane_kappa_coef() * (*path_points)[0].kappa();

  // generate a Quintic Polynomial Curve
  QuinticPolynomialCurve1d curve(init_frenet_point.l(), init_frenet_point.dl(),
                                 init_frenet_point.ddl(), 0.0, 0.0, 0.0,
                                 curve_path_length);

  // according to Quintic Polynomial Curve generated frenet path
  double current_s = 0.0;
  std::vector<common::FrenetFramePoint> frenet_path;
  common::FrenetFramePoint frenet_frame_point;
  while (current_s + config_.dense_sample_resolution() < curve_path_length) {
    const double l = curve.Evaluate(0, current_s);
    const double dl = curve.Evaluate(1, current_s);
    const double ddl = curve.Evaluate(2, current_s);
    frenet_frame_point.set_s(init_frenet_point.s() + current_s);
    frenet_frame_point.set_l(l);
    frenet_frame_point.set_dl(dl);
    frenet_frame_point.set_ddl(ddl);
    frenet_path.push_back(std::move(frenet_frame_point));
    current_s += config_.dense_sample_resolution();
  }

  if ((curve_path_length - current_s) >
      0.5 * config_.dense_sample_resolution()) {
    const double l = curve.Evaluate(0, curve_path_length);
    const double dl = curve.Evaluate(1, curve_path_length);
    const double ddl = curve.Evaluate(2, curve_path_length);
    frenet_frame_point.set_s(init_frenet_point.s() + curve_path_length);
    frenet_frame_point.set_l(l);
    frenet_frame_point.set_dl(dl);
    frenet_frame_point.set_ddl(ddl);
    frenet_path.push_back(std::move(frenet_frame_point));
  }

  // conver frenet path to discreted path data
  std::vector<common::PathPoint> curve_path_points;
  if (!CovertFrenetPathToDiscretedPath(reference_line, frenet_path,
                                       &curve_path_points)) {
    path_points->clear();
    return;
  }

  // merge path points generated from the Quintic Polynomial Curve
  // into path points intercepted from the reference line
  const auto last_curve_point_s = curve_path_points.back().s();
  auto itr_upper =
      std::find_if(path_points->begin(), path_points->end(),
                   [last_curve_point_s](const common::PathPoint& path_point) {
                     return path_point.s() > last_curve_point_s;
                   });
  path_points->erase(path_points->begin(), itr_upper);
  path_points->insert(path_points->begin(), curve_path_points.begin(),
                      curve_path_points.end());
  return;
}

void NaviPathDecider::KeepLane(
    const double dest_ref_line_y,
    std::vector<common::PathPoint>* const path_points) {
  double dest_lateral_distance = std::fabs(dest_ref_line_y);
  if (dest_lateral_distance < config_.max_keep_lane_distance()) {
    auto& reference_line = reference_line_info_->reference_line();
    auto obstacles = frame_->obstacles();
    auto* path_decision = reference_line_info_->path_decision();
    double actual_dest_point_y =
        NudgeProcess(reference_line, *path_points, obstacles, *path_decision,
                     vehicle_state_);

    double actual_dest_lateral_distance = std::fabs(actual_dest_point_y);
    double actual_shift_y = 0.0;
    if (actual_dest_lateral_distance > config_.min_keep_lane_offset()) {
      double lateral_shift_value = 0.0;
      lateral_shift_value =
          (actual_dest_lateral_distance <
           config_.max_keep_lane_shift_y() + config_.min_keep_lane_offset() -
               config_.keep_lane_shift_compensation())
              ? (actual_dest_lateral_distance - config_.min_keep_lane_offset() +
                 config_.keep_lane_shift_compensation())
              : config_.max_keep_lane_shift_y();
      actual_shift_y = std::copysign(lateral_shift_value, actual_dest_point_y);
    }

    // Since the adc is always driving on the left side of the road, give a
    // lateral compensation to the right lane
    actual_shift_y -= config_.basic_path_lateral_compensation();

    AINFO << "in current plan cycle keep lane, actual dest : "
          << actual_dest_point_y << " adc shift to dest : " << actual_shift_y;
    std::transform(
        path_points->begin(), path_points->end(), path_points->begin(),
        [actual_shift_y](common::PathPoint& old_path_point) {
          common::PathPoint new_path_point = old_path_point;
          double new_path_point_y = old_path_point.y() + actual_shift_y;
          new_path_point.set_y(new_path_point_y);
          return new_path_point;
        });
  }

  return;
}

void NaviPathDecider::RecordDebugInfo(const PathData& path_data) {
  const auto& path_points = path_data.discretized_path().path_points();
  auto* ptr_optimized_path = reference_line_info_->mutable_debug()
                                 ->mutable_planning_data()
                                 ->add_path();
  ptr_optimized_path->set_name(Name());
  ptr_optimized_path->mutable_path_point()->CopyFrom(
      {path_points.begin(), path_points.end()});
}

bool NaviPathDecider::GetBasicPathData(
    const ReferenceLine& reference_line,
    std::vector<common::PathPoint>* const path_points) {
  CHECK_NOTNULL(path_points);

  double min_path_len = config_.min_path_length();
  // get min path plan lenth s = v0 * t + 1 / 2.0 * a * t^2
  double path_len = start_plan_v_ * config_.min_look_forward_time() +
                    start_plan_a_ * pow(0.1, 2) / 2.0;
  path_len = std::max(path_len, min_path_len);

  const double reference_line_len = reference_line.Length();
  if (reference_line_len < path_len) {
    AERROR << "Reference line is too short to generate path trajectory( s = "
           << reference_line_len << ").";
    return false;
  }

  // get the start plan point project s on refernce line and get the length of
  // reference line
  auto start_plan_point_project = reference_line.GetReferencePoint(
      start_plan_point_.x(), start_plan_point_.y());
  common::SLPoint sl_point;
  if (!reference_line.XYToSL(start_plan_point_project.ToPathPoint(0.0),
                             &sl_point)) {
    AERROR << "Failed to get start plan point s from reference "
              "line.";
    return false;
  }
  auto start_plan_point_project_s = sl_point.has_s() ? sl_point.s() : 0.0;

  // get basic path points form reference_line
  ADEBUG << "Basic path data len ; " << reference_line_len;
  for (double s = start_plan_point_project_s; s < reference_line_len;
       s += ((s < path_len) ? config_.dense_sample_resolution()
                            : config_.sparse_sample_resolution())) {
    const auto& ref_point = reference_line.GetReferencePoint(s);
    auto path_point = ref_point.ToPathPoint(s - start_plan_point_project_s);
    path_points->emplace_back(path_point);
  }

  if (path_points->empty()) {
    AERROR << "path poins is empty.";
    return false;
  }

  return true;
}

double NaviPathDecider::NudgeProcess(
    const ReferenceLine& reference_line,
    const std::vector<common::PathPoint>& path_data_points,
    const std::vector<const Obstacle*>& obstacles,
    const PathDecision& path_decision,
    const common::VehicleState& vehicle_state) {
  double nudge_position_y = 0.0;

  if (!FLAGS_enable_nudge_decision) {
    nudge_position_y = path_data_points[0].y();
    return nudge_position_y;
  }

  // get nudge latteral position
  int lane_obstacles_num = 0;
  constexpr double KNudgeEpsilon = 1e-6;
  double nudge_distance = obstacle_decider_.GetNudgeDistance(
      obstacles, reference_line, path_decision, path_data_points, vehicle_state,
      &lane_obstacles_num);
  // adjust plan start point
  if (std::fabs(nudge_distance) > KNudgeEpsilon) {
    ADEBUG << "need latteral nudge distance : " << nudge_distance;
    nudge_position_y = nudge_distance;
    last_lane_id_to_nudge_flag_[cur_reference_line_lane_id_] = true;
  } else {
    // no nudge distance but current lane has obstacles ,keepping path in
    // the last nudge path direction
    bool last_plan_has_nudge = false;
    if (last_lane_id_to_nudge_flag_.find(cur_reference_line_lane_id_) !=
        last_lane_id_to_nudge_flag_.end()) {
      last_plan_has_nudge =
          last_lane_id_to_nudge_flag_[cur_reference_line_lane_id_];
    }

    if (last_plan_has_nudge && lane_obstacles_num != 0) {
      ADEBUG << "Keepping last nudge path direction";
      nudge_position_y = vehicle_state_.y();
    } else {
      // not need nudge or not need nudge keepping
      last_lane_id_to_nudge_flag_[cur_reference_line_lane_id_] = false;
      nudge_position_y = path_data_points[0].y();
    }
  }

  return nudge_position_y;
}

double NaviPathDecider::CalculateDistanceToDestLane() {
  // match an appropriate lateral shift param from the configuration file
  // based on the current state of the vehicle state
  double move_distance = 0.0;
  double max_adc_speed =
      start_plan_v_ + start_plan_a_ * 1.0 / FLAGS_planning_loop_rate;
  double match_speed = 0.5 * (start_plan_v_ + max_adc_speed);
  auto max_speed_level_itr = std::upper_bound(
      max_speed_levels_.begin(), max_speed_levels_.end(), match_speed);
  if (max_speed_level_itr != max_speed_levels_.end()) {
    auto max_speed_level = *max_speed_level_itr;
    move_distance = move_dest_lane_config_talbe_[max_speed_level];
  }

  return move_distance;
}

bool NaviPathDecider::CalculateFrenetPoint(
    const ReferenceLine& reference_line, const common::PathPoint& path_point,
    common::FrenetFramePoint* const frenet_frame_point) {
  CHECK_NOTNULL(frenet_frame_point);
  common::SLPoint sl_point;
  if (!reference_line.XYToSL({path_point.x(), path_point.y()}, &sl_point)) {
    return false;
  }
  frenet_frame_point->set_s(sl_point.s());
  frenet_frame_point->set_l(sl_point.l());

  const float theta = path_point.theta();
  const float kappa = path_point.kappa();
  const float l = frenet_frame_point->l();

  ReferencePoint ref_point;
  ref_point = reference_line.GetReferencePoint(frenet_frame_point->s());

  const float theta_ref = ref_point.heading();
  const float kappa_ref = ref_point.kappa();
  const float dkappa_ref = ref_point.dkappa();

  const float dl = CartesianFrenetConverter::CalculateLateralDerivative(
      theta_ref, theta, l, kappa_ref);
  const float ddl = common::math::CartesianFrenetConverter::
      CalculateSecondOrderLateralDerivative(theta_ref, theta, kappa_ref, kappa,
                                            dkappa_ref, l);
  frenet_frame_point->set_dl(dl);
  frenet_frame_point->set_ddl(ddl);

  return true;
}

bool NaviPathDecider::CovertFrenetPathToDiscretedPath(
    const ReferenceLine& reference_line,
    const std::vector<common::FrenetFramePoint>& frenet_path_points,
    std::vector<common::PathPoint>* const path_points) {
  CHECK_NOTNULL(path_points);
  for (const common::FrenetFramePoint& frenet_point : frenet_path_points) {
    common::SLPoint sl_point;
    common::math::Vec2d cartesian_point;
    sl_point.set_s(frenet_point.s());
    sl_point.set_l(frenet_point.l());
    if (!reference_line.SLToXY(sl_point, &cartesian_point)) {
      AERROR << "Failed to convert sl point to xy point";
      return false;
    }

    ReferencePoint ref_point =
        reference_line.GetReferencePoint(frenet_point.s());
    double theta = CartesianFrenetConverter::CalculateTheta(
        ref_point.heading(), ref_point.kappa(), frenet_point.l(),
        frenet_point.dl());
    double kappa = CartesianFrenetConverter::CalculateKappa(
        ref_point.kappa(), ref_point.dkappa(), frenet_point.l(),
        frenet_point.dl(), frenet_point.ddl());

    common::PathPoint path_point = common::util::MakePathPoint(
        cartesian_point.x(), cartesian_point.y(), 0.0, theta, kappa, 0.0, 0.0);

    if (path_points->empty()) {
      path_point.set_s(0.0);
      path_point.set_dkappa(0.0);
    } else {
      common::math::Vec2d last(path_points->back().x(),
                               path_points->back().y());
      common::math::Vec2d current(path_point.x(), path_point.y());
      double distance = (last - current).Length();
      path_point.set_s(path_points->back().s() + distance);
      path_point.set_dkappa((path_point.kappa() - path_points->back().kappa()) /
                            distance);
    }
    path_points->push_back(std::move(path_point));
  }

  return true;
}

bool NaviPathDecider::IsLaneTooBend(const ReferenceLine& reference_line) {
  common::SLPoint sl_point;
  if (!reference_line.XYToSL({start_plan_point_.x(), start_plan_point_.y()},
                             &sl_point)) {
    sl_point.set_s(0.0);
    sl_point.set_l(0.0);
  }
  double start_s = sl_point.s();
  double end_s = start_s + kStatisticLaneLength;
  auto ref_points = reference_line.GetReferencePoints(start_s, end_s);
  std::size_t large_curvature_num = 0;
  for (const auto& ref_point : ref_points) {
    if (std::fabs(ref_point.kappa()) < config_.min_large_curvature()) {
      continue;
    }
    ++large_curvature_num;
  }
  if (large_curvature_num < kStatisticPointsNum) {
    return false;
  }
}

}  // namespace planning
}  // namespace apollo
