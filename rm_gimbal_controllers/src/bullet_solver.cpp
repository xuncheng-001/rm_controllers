/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, Qiayuan Liao
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

//
// Created by qiayuan on 8/14/20.
//

#include "rm_gimbal_controllers/bullet_solver.h"
#include <cmath>
#include <tf/transform_datatypes.h>
#include <rm_common/ori_tool.h>

namespace rm_gimbal_controllers
{
BulletSolver::BulletSolver(ros::NodeHandle& controller_nh)
{
  config_ = {
    .resistance_coff_qd_10 = getParam(controller_nh, "resistance_coff_qd_10", 0.),
    .resistance_coff_qd_15 = getParam(controller_nh, "resistance_coff_qd_15", 0.),
    .resistance_coff_qd_16 = getParam(controller_nh, "resistance_coff_qd_16", 0.),
    .resistance_coff_qd_18 = getParam(controller_nh, "resistance_coff_qd_18", 0.),
    .resistance_coff_qd_30 = getParam(controller_nh, "resistance_coff_qd_30", 0.),
    .g = getParam(controller_nh, "g", 0.),
    .delay = getParam(controller_nh, "delay", 0.),
    .wait_next_armor_delay = getParam(controller_nh, "wait_next_armor_delay", 0.105),
    .wait_diagonal_armor_delay = getParam(controller_nh, "wait_diagonal_armor_delay", 0.105),
    .dt = getParam(controller_nh, "dt", 0.),
    .timeout = getParam(controller_nh, "timeout", 0.),
    .max_switch_angle = getParam(controller_nh, "max_switch_angle", 40.0),
    .min_switch_angle = getParam(controller_nh, "min_switch_angle", 2.0),
    .switch_angle_offset = getParam(controller_nh, "switch_angle_offset", 0.0),
    .switch_duration_scale = getParam(controller_nh, "switch_duration_scale", 0.),
    .switch_duration_rate = getParam(controller_nh, "switch_duration_rate", 0.),
    .switch_duration_offset = getParam(controller_nh, "switch_duration_offset", 0.08),
    .min_shoot_beforehand_vel = getParam(controller_nh, "min_shoot_beforehand_vel", 4.5),
    .max_chassis_angular_vel = getParam(controller_nh, "max_chassis_angular_vel", 8.5),
    .track_rotate_target_delay = getParam(controller_nh, "track_rotate_target_delay", 0.),
    .track_move_target_delay = getParam(controller_nh, "track_move_target_delay", 0.),
    .min_fit_switch_count = getParam(controller_nh, "min_fit_switch_count", 3),
  };
  max_track_target_vel_ = getParam(controller_nh, "max_track_target_vel", 5.0);
  switch_hysteresis_ = getParam(controller_nh, "switch_hysteresis", 1.0);
  config_rt_buffer_.initRT(config_);
//一个用于在rviz中绘制特性数据点或者轨迹的工具
  marker_desire_.header.frame_id = "odom";
  marker_desire_.ns = "model";
  marker_desire_.action = visualization_msgs::Marker::ADD;
  marker_desire_.type = visualization_msgs::Marker::POINTS;
  marker_desire_.scale.x = 0.02;
  marker_desire_.scale.y = 0.02;
  marker_desire_.color.r = 1.0;
  marker_desire_.color.g = 0.0;
  marker_desire_.color.b = 0.0;
  marker_desire_.color.a = 1.0;

  marker_real_ = marker_desire_;
  marker_real_.color.r = 0.0;
  marker_real_.color.g = 1.0;

  d_srv_ = new dynamic_reconfigure::Server<rm_gimbal_controllers::BulletSolverConfig>(controller_nh);
  dynamic_reconfigure::Server<rm_gimbal_controllers::BulletSolverConfig>::CallbackType cb =
      [this](auto&& PH1, auto&& PH2) { reconfigCB(PH1, PH2); };
  d_srv_->setCallback(cb);

  path_desire_pub_.reset(
      new realtime_tools::RealtimePublisher<visualization_msgs::Marker>(controller_nh, "model_desire", 10));
//发布者创建，具体后面补充
  path_real_pub_.reset(
      new realtime_tools::RealtimePublisher<visualization_msgs::Marker>(controller_nh, "model_real", 10));
  shoot_beforehand_cmd_pub_.reset(
      new realtime_tools::RealtimePublisher<rm_msgs::ShootBeforehandCmd>(controller_nh, "shoot_beforehand_cmd", 10));
  fly_time_pub_.reset(new realtime_tools::RealtimePublisher<std_msgs::Float64>(controller_nh, "fly_time", 10));
  identified_target_change_sub_ =
      controller_nh.subscribe<std_msgs::Bool>("/change", 10, &BulletSolver::identifiedTargetChangeCB, this);
}
//根据子弹速度获取阻力系数
double BulletSolver::getResistanceCoefficient(double bullet_speed) const
{
  // bullet_speed have 5 value:10,15,16,18,30
  double resistance_coff;
  if (bullet_speed < 12.5)
    resistance_coff = config_.resistance_coff_qd_10;
  else if (bullet_speed < 15.5)
    resistance_coff = config_.resistance_coff_qd_15;
  else if (bullet_speed < 17)
    resistance_coff = config_.resistance_coff_qd_16;
  else if (bullet_speed < 24)
    resistance_coff = config_.resistance_coff_qd_18;
  else
    resistance_coff = config_.resistance_coff_qd_30;
  return resistance_coff;
}

bool BulletSolver::solve(geometry_msgs::Point pos, geometry_msgs::Vector3 vel, double bullet_speed, double yaw,
                         double v_yaw, double r1, double r2, double dz, int armors_num, double chassis_angular_vel_z)
{
//实时性线程
  config_ = *config_rt_buffer_.readFromRT();
//函数内变量赋值
  bullet_speed_ = bullet_speed;
  resistance_coff_ = getResistanceCoefficient(bullet_speed_) != 0 ? getResistanceCoefficient(bullet_speed_) : 0.001;

//切换装甲板（通过判断前后识别到装甲板的位置是否发生突变）
  if (abs(yaw - last_yaw_) > 1.)
    filtered_yaw_ = yaw;
//如果没有突变就使用滤波计算（filtered_yaw是滤波之后的yaw）
  else if (last_yaw_ != yaw)
  {
    filtered_yaw_ = filtered_yaw_ + (yaw - filtered_yaw_) * (0.001 / (0.01 + 0.001));
  }
//更新上一次装甲板位置
  last_yaw_ = yaw;

//通过对面的位置计算出云台的输出位置
  double temp_z = pos.z;
  double target_rho = std::sqrt(std::pow(pos.x, 2) + std::pow(pos.y, 2));
//（还不确定），目标中心位置与坐标系x之间的夹角，（猜测：瞄准的时候的云台相对于初始位置的位置偏移
  output_yaw_ = std::atan2(pos.y, pos.x);
  output_pitch_ = std::atan2(temp_z, std::sqrt(std::pow(pos.x, 2) + std::pow(pos.y, 2)));
//飞行时间的计算（粗略的）
double rough_fly_time =
      (-std::log(1 - target_rho * resistance_coff_ / (bullet_speed_ * std::cos(output_pitch_)))) / resistance_coff_;
  selected_armor_ = 0;
  double r = r1;
  double z = pos.z;
//最小切换角度，并且从度数变成弧度
  double min_switch_angle = config_.min_switch_angle / 180 * M_PI;
//跟随模式下
  if (track_target_)
  {
//目标速度大于最大跟随速度就取消跟随模式
//switch_hysteresis是一个滞后阈值，让云台不会一直在跟随和中心两个模式之间切换
    if (std::abs(v_yaw) >= max_track_target_vel_ + switch_hysteresis_)
      track_target_ = false;
  }
  else
  {
    if (std::abs(v_yaw) <= max_track_target_vel_ - switch_hysteresis_)
      track_target_ = true;
  }
//通过判断是否在跟随模式下来确定切换装甲板角度
//如果在跟随模式，输出为  目标两块装甲板间隔  减去  云台切换和子弹飞行时间过程中目标已经转过的角度  加上误差
//不在跟随模式就使用最小切换角度
  double switch_armor_angle =
      track_target_ ? M_PI / armors_num - (2 * rough_fly_time + getGimbalSwitchDuration(abs(v_yaw))) / 2 * abs(v_yaw) +
                          config_.switch_angle_offset :
                      min_switch_angle;
//目标位置减去云台输出（猜测：减去的是云台此时的位置）
  double yaw_subtract = filtered_yaw_ - output_yaw_;
//把yaw_subtract控制在-pi到pi之间
  while (yaw_subtract > M_PI)
    yaw_subtract -= 2 * M_PI;
  while (yaw_subtract < -M_PI)
    yaw_subtract += 2 * M_PI;
//角度偏差超过切换角度（顺时针和逆时针两个方向）
  if (((yaw_subtract > switch_armor_angle) && v_yaw > 1.) || ((yaw_subtract < -switch_armor_angle) && v_yaw < -1.))
  {
    count_++;
//如果切换了装甲板计数器重置，并且状态重置
    if (identified_target_change_)
    {
      count_ = 0;
      identified_target_change_ = false;
    }
//计数器超出    最小的切换次数
    if (count_ >= config_.min_fit_switch_count)
    {
//刚好相等时更新切换装甲板时间
      if (count_ == config_.min_fit_switch_count)
        switch_armor_time_ = ros::Time::now();
//通过v_yaw来判断切换时选择的装甲板
      selected_armor_ = v_yaw > 0. ? -1 : 1;
//通过装甲板数量确定半径大小和高度
      r = armors_num == 4 ? r2 : r1;
      z = armors_num == 4 ? pos.z + dz : pos.z;
    }
  }
//滤波之后另外一块装甲板的位置
  double filtered_aim_armor_yaw = filtered_yaw_ + selected_armor_ * 2 * M_PI / armors_num;
//与前面的yaw_subtract同理
  double aim_yaw_subtract = filtered_aim_armor_yaw - output_yaw_;
  while (aim_yaw_subtract > M_PI)
    aim_yaw_subtract -= 2 * M_PI;
  while (aim_yaw_subtract < -M_PI)
    aim_yaw_subtract += 2 * M_PI;
  is_in_delay_before_switch_ = ((((aim_yaw_subtract + v_yaw * config_.delay) > switch_armor_angle) && v_yaw > 1.) ||
                                (((aim_yaw_subtract + v_yaw * config_.delay) < -switch_armor_angle) && v_yaw < -1.)) &&
                               track_target_;
//目标装甲板添加上平移的延迟
  if (track_target_)
    yaw += v_yaw * config_.track_rotate_target_delay;
  pos.x += vel.x * config_.track_move_target_delay;
  pos.y += vel.y * config_.track_move_target_delay;
  int count{};
  double error = 999;
  if (track_target_)
  {
    //此时锁定装甲板的坐标系位置（pos是目标中心位置
    target_pos_.x = pos.x - r * cos(yaw + selected_armor_ * 2 * M_PI / armors_num);
    target_pos_.y = pos.y - r * sin(yaw + selected_armor_ * 2 * M_PI / armors_num);
  }
  else
  {
    //非跟随状态下瞄准的是车体与自身连线位置所在的装甲板，也就是中心模式
    target_pos_.x = pos.x - r * cos(atan2(pos.y, pos.x));
    target_pos_.y = pos.y - r * sin(atan2(pos.y, pos.x));
    //判断是否可以击打到下一块装甲板，如果不可以就选择下两块
    if ((v_yaw > 1.0 && (yaw_subtract + v_yaw * (fly_time_ + config_.wait_next_armor_delay) +
                         selected_armor_ * 2 * M_PI / armors_num) > 0.) ||
        (v_yaw < -1.0 && (yaw_subtract + v_yaw * (fly_time_ + config_.wait_next_armor_delay) +
                          selected_armor_ * 2 * M_PI / armors_num) < 0.))
      selected_armor_ = v_yaw > 0. ? -2 : 2;
//如果选择第二块装甲板时应该选用的半径和高度
    if (selected_armor_ % 2 == 0)
    {
      r = r1;
      z = pos.z;
    }
  }
  target_pos_.z = z;
  while (error >= 0.001)
  {
    //更新output_yaw_,这个时候变成目标装甲板的位置（？？？？？？？？？？？？
    output_yaw_ = std::atan2(target_pos_.y, target_pos_.x);
    output_pitch_ = std::atan2(temp_z, std::sqrt(std::pow(target_pos_.x, 2) + std::pow(target_pos_.y, 2)));
    //目标的距离
    target_rho = std::sqrt(std::pow(target_pos_.x, 2) + std::pow(target_pos_.y, 2));
    //相关公式计算飞行时间和子弹z方向落点
    fly_time_ =
        (-std::log(1 - target_rho * resistance_coff_ / (bullet_speed_ * std::cos(output_pitch_)))) / resistance_coff_;
    double real_z = (bullet_speed_ * std::sin(output_pitch_) + (config_.g / resistance_coff_)) *
                        (1 - std::exp(-resistance_coff_ * fly_time_)) / resistance_coff_ -
                    config_.g * fly_time_ / resistance_coff_;

    if (track_target_)
    {
      //更新目标装甲板的位置，算入了目标的平移和自转
      target_pos_.x =
          pos.x + vel.x * fly_time_ - r * cos(yaw + v_yaw * fly_time_ + selected_armor_ * 2 * M_PI / armors_num);
      target_pos_.y =
          pos.y + vel.y * fly_time_ - r * sin(yaw + v_yaw * fly_time_ + selected_armor_ * 2 * M_PI / armors_num);
    }
    else
    {
      //中心模式下计算的是子弹到达时对方的位置
      double target_pos_after_fly_time[2];
      target_pos_after_fly_time[0] = pos.x + vel.x * fly_time_;
      target_pos_after_fly_time[1] = pos.y + vel.y * fly_time_;
      target_pos_.x =
          target_pos_after_fly_time[0] - r * cos(atan2(target_pos_after_fly_time[1], target_pos_after_fly_time[0]));
      target_pos_.y =
          target_pos_after_fly_time[1] - r * sin(atan2(target_pos_after_fly_time[1], target_pos_after_fly_time[0]));
    }
    target_pos_.z = z + vel.z * fly_time_;

    //误差角度，以自身为参考系的
    double target_yaw = std::atan2(target_pos_.y, target_pos_.x);
    double error_theta = target_yaw - output_yaw_;
    double error_z = target_pos_.z - real_z;
    temp_z += error_z;
    //最后子弹落点误差
    error = std::sqrt(std::pow(error_theta * target_rho, 2) + std::pow(error_z, 2));
    count++;

    if (count >= 20 || std::isnan(error))
      return false;
  }
  //不太懂，摘抄的解释
  //总结起来，这段代码的主要目的是在一个多线程环境中安全地更新并发布飞行时间的消息。通过使用和方法，确保了在更新和发布过程中不会发生数据竞争。
  //`trylock``unlockAndPublish`方法，确保了更新和发布过程中不会发生数据竞争
  if (fly_time_pub_->trylock())
  {
    fly_time_pub_->msg_.data = fly_time_;
    fly_time_pub_->unlockAndPublish();
  }
  return true;
}

void BulletSolver::getSelectedArmorPosAndVel(geometry_msgs::Point& armor_pos, geometry_msgs::Vector3& armor_vel,
                                             geometry_msgs::Point pos, geometry_msgs::Vector3 vel, double yaw,
                                             double v_yaw, double r1, double r2, double dz, int armors_num)
{
  double r = r1, z = pos.z;
  if (armors_num == 4 && selected_armor_ != 0)
  {
    r = r2;
    z = pos.z + dz;
  }
  pos.x += vel.x * (config_.track_move_target_delay + fly_time_);
  pos.y += vel.y * (config_.track_move_target_delay + fly_time_);
  if (track_target_)
  {
    armor_pos.x = pos.x - r * cos(yaw + v_yaw * (fly_time_ + config_.track_rotate_target_delay) +
                                  selected_armor_ * 2 * M_PI / armors_num);
    armor_pos.y = pos.y - r * sin(yaw + v_yaw * (fly_time_ + config_.track_rotate_target_delay) +
                                  selected_armor_ * 2 * M_PI / armors_num);
    armor_pos.z = z;
    armor_vel.x = vel.x + v_yaw * r *
                              sin(yaw + v_yaw * (fly_time_ + config_.track_rotate_target_delay) +
                                  selected_armor_ * 2 * M_PI / armors_num);
    armor_vel.y = vel.y - v_yaw * r *
                              cos(yaw + v_yaw * (fly_time_ + config_.track_rotate_target_delay) +
                                  selected_armor_ * 2 * M_PI / armors_num);
    armor_vel.z = vel.z;
  }
  else
  {
    armor_pos = pos;
    armor_pos.z = z;
    armor_vel = vel;
  }
}

void BulletSolver::bulletModelPub(const geometry_msgs::TransformStamped& odom2pitch, const ros::Time& time)
{
  marker_desire_.points.clear();
  marker_real_.points.clear();
  double roll{}, pitch{}, yaw{};
  //从odom到云台的坐标变换，然后会给roll，pitch，yaw赋值，
  quatToRPY(odom2pitch.transform.rotation, roll, pitch, yaw);
  geometry_msgs::Point point_desire{}, point_real{};
  double target_rho = std::sqrt(std::pow(target_pos_.x, 2) + std::pow(target_pos_.y, 2));
  int point_num = int(target_rho * 20);
  for (int i = 0; i <= point_num; i++)
  {
    //表示实时子弹轨迹（分成二十个点
    double rt_bullet_rho = target_rho * i / point_num;

    //计算子弹时间和z轴方向上的落点，这里可能存在问题，按照变量名想要表示的是实时性的z轴落点，但此处只能计算最后的落点，则在所有循环中计算出来都是同一个数值
    double fly_time = (-std::log(1 - rt_bullet_rho * resistance_coff_ / (bullet_speed_ * std::cos(output_pitch_)))) /
                      resistance_coff_;
    double rt_bullet_z = (bullet_speed_ * std::sin(output_pitch_) + (config_.g / resistance_coff_)) *
                             (1 - std::exp(-resistance_coff_ * fly_time)) / resistance_coff_ -
                         config_.g * fly_time / resistance_coff_;
    //这里是期望落点位置（就是目标装甲板的位置，为什么不直接用视觉发过来的数据？）
    point_desire.x = rt_bullet_rho * std::cos(output_yaw_) + odom2pitch.transform.translation.x;
    point_desire.y = rt_bullet_rho * std::sin(output_yaw_) + odom2pitch.transform.translation.y;
    point_desire.z = rt_bullet_z + odom2pitch.transform.translation.z;
    marker_desire_.points.push_back(point_desire);
  }
  //这一部分才是计算真实的落点位置，
  for (int i = 0; i <= point_num; i++)
  {
    double rt_bullet_rho = target_rho * i / point_num;
    double fly_time =
        (-std::log(1 - rt_bullet_rho * resistance_coff_ / (bullet_speed_ * std::cos(-pitch)))) / resistance_coff_;
    double rt_bullet_z = (bullet_speed_ * std::sin(-pitch) + (config_.g / resistance_coff_)) *
                             (1 - std::exp(-resistance_coff_ * fly_time)) / resistance_coff_ -
                         config_.g * fly_time / resistance_coff_;
    point_real.x = rt_bullet_rho * std::cos(yaw) + odom2pitch.transform.translation.x;
    point_real.y = rt_bullet_rho * std::sin(yaw) + odom2pitch.transform.translation.y;
    point_real.z = rt_bullet_z + odom2pitch.transform.translation.z;
    marker_real_.points.push_back(point_real);
  }
  marker_desire_.header.stamp = time;
  if (path_desire_pub_->trylock())
  {
    path_desire_pub_->msg_ = marker_desire_;
    path_desire_pub_->unlockAndPublish();
  }
  marker_real_.header.stamp = time;
  if (path_real_pub_->trylock())
  {
    path_real_pub_->msg_ = marker_real_;
    path_real_pub_->unlockAndPublish();
  }
}

double BulletSolver::getGimbalError(geometry_msgs::Point pos, geometry_msgs::Vector3 vel, double yaw, double v_yaw,
                                    double r1, double r2, double dz, int armors_num, double yaw_real, double pitch_real,
                                    double bullet_speed)
{
  double delay;
  if (track_target_)
    delay = 0.;
  else
    delay = selected_armor_ % 2 == 0 ? config_.wait_diagonal_armor_delay : config_.wait_next_armor_delay;
  double r, z;
  if (selected_armor_ % 2 == 0)
  {
    r = r1;
    z = pos.z;
  }
  else
  {
    r = armors_num == 4 ? r2 : r1;
    z = armors_num == 4 ? pos.z + dz : pos.z;
  }
  double error;
  if (track_target_)
  {
    double bullet_rho =
        bullet_speed * std::cos(pitch_real) * (1 - std::exp(-resistance_coff_ * fly_time_)) / resistance_coff_;
    double bullet_x = bullet_rho * std::cos(yaw_real);
    double bullet_y = bullet_rho * std::sin(yaw_real);
    double bullet_z = (bullet_speed * std::sin(pitch_real) + (config_.g / resistance_coff_)) *
                          (1 - std::exp(-resistance_coff_ * fly_time_)) / resistance_coff_ -
                      config_.g * fly_time_ / resistance_coff_;
    error = std::sqrt(std::pow(target_pos_.x - bullet_x, 2) + std::pow(target_pos_.y - bullet_y, 2) +
                      std::pow(target_pos_.z - bullet_z, 2));
  }
  else
  {
    geometry_msgs::Point target_pos_after_fly_time_and_delay{};
    target_pos_after_fly_time_and_delay.x =
        pos.x + vel.x * (fly_time_ + delay) -
        r * cos(yaw + v_yaw * (fly_time_ + delay) + selected_armor_ * 2 * M_PI / armors_num);
    target_pos_after_fly_time_and_delay.y =
        pos.y + vel.y * (fly_time_ + delay) -
        r * sin(yaw + v_yaw * (fly_time_ + delay) + selected_armor_ * 2 * M_PI / armors_num);
    target_pos_after_fly_time_and_delay.z = z + vel.z * (fly_time_ + delay);
    error = std::sqrt(std::pow(target_pos_.x - target_pos_after_fly_time_and_delay.x, 2) +
                      std::pow(target_pos_.y - target_pos_after_fly_time_and_delay.y, 2) +
                      std::pow(target_pos_.z - target_pos_after_fly_time_and_delay.z, 2));
  }
  return error;
}

void BulletSolver::identifiedTargetChangeCB(const std_msgs::BoolConstPtr& msg)
{
  if (msg->data)
    identified_target_change_ = true;
}

double BulletSolver::getGimbalSwitchDuration(double v_yaw)
{
//公式：（还不懂公式的原理）
//云台切换时间=缩放因子*e^(速率*每分钟转数)+切换间隔误差
  gimbal_switch_duration_ =
      config_.switch_duration_scale * std::exp(config_.switch_duration_rate * (v_yaw * 60 / (2 * M_PI))) +
      config_.switch_duration_offset;
  return gimbal_switch_duration_;
}

void BulletSolver::judgeShootBeforehand(const ros::Time& time, double v_yaw)
{
  if (!track_target_)
    shoot_beforehand_cmd_ = rm_msgs::ShootBeforehandCmd::JUDGE_BY_ERROR;
  else if (std::abs(v_yaw) > config_.min_shoot_beforehand_vel)
  {
    if ((ros::Time::now() - switch_armor_time_).toSec() < getGimbalSwitchDuration(std::abs(v_yaw)) - config_.delay)
      shoot_beforehand_cmd_ = rm_msgs::ShootBeforehandCmd::BAN_SHOOT;
    if (((ros::Time::now() - switch_armor_time_).toSec() < getGimbalSwitchDuration(std::abs(v_yaw))))
      shoot_beforehand_cmd_ = rm_msgs::ShootBeforehandCmd::ALLOW_SHOOT;
    if (is_in_delay_before_switch_)
      shoot_beforehand_cmd_ = rm_msgs::ShootBeforehandCmd::BAN_SHOOT;
  }
  else
    shoot_beforehand_cmd_ = rm_msgs::ShootBeforehandCmd::JUDGE_BY_ERROR;
  if (shoot_beforehand_cmd_pub_->trylock())
  {
    shoot_beforehand_cmd_pub_->msg_.stamp = time;
    shoot_beforehand_cmd_pub_->msg_.cmd = shoot_beforehand_cmd_;
    shoot_beforehand_cmd_pub_->unlockAndPublish();
  }
}

void BulletSolver::reconfigCB(rm_gimbal_controllers::BulletSolverConfig& config, uint32_t /*unused*/)
{
  ROS_INFO("[Bullet Solver] Dynamic params change");
  if (!dynamic_reconfig_initialized_)
  {
    Config init_config = *config_rt_buffer_.readFromNonRT();  // config init use yaml
    config.resistance_coff_qd_10 = init_config.resistance_coff_qd_10;
    config.resistance_coff_qd_15 = init_config.resistance_coff_qd_15;
    config.resistance_coff_qd_16 = init_config.resistance_coff_qd_16;
    config.resistance_coff_qd_18 = init_config.resistance_coff_qd_18;
    config.resistance_coff_qd_30 = init_config.resistance_coff_qd_30;
    config.g = init_config.g;
    config.delay = init_config.delay;
    config.wait_next_armor_delay = init_config.wait_next_armor_delay;
    config.wait_diagonal_armor_delay = init_config.wait_diagonal_armor_delay;
    config.dt = init_config.dt;
    config.timeout = init_config.timeout;
    config.max_switch_angle = init_config.max_switch_angle;
    config.min_switch_angle = init_config.min_switch_angle;
    config.switch_angle_offset = init_config.switch_angle_offset;
    config.switch_duration_scale = init_config.switch_duration_scale;
    config.switch_duration_rate = init_config.switch_duration_rate;
    config.switch_duration_offset = init_config.switch_duration_offset;
    config.min_shoot_beforehand_vel = init_config.min_shoot_beforehand_vel;
    config.max_chassis_angular_vel = init_config.max_chassis_angular_vel;
    config.track_rotate_target_delay = init_config.track_rotate_target_delay;
    config.track_move_target_delay = init_config.track_move_target_delay;
    config.min_fit_switch_count = init_config.min_fit_switch_count;
    dynamic_reconfig_initialized_ = true;
  }
  Config config_non_rt{ .resistance_coff_qd_10 = config.resistance_coff_qd_10,
                        .resistance_coff_qd_15 = config.resistance_coff_qd_15,
                        .resistance_coff_qd_16 = config.resistance_coff_qd_16,
                        .resistance_coff_qd_18 = config.resistance_coff_qd_18,
                        .resistance_coff_qd_30 = config.resistance_coff_qd_30,
                        .g = config.g,
                        .delay = config.delay,
                        .wait_next_armor_delay = config.wait_next_armor_delay,
                        .wait_diagonal_armor_delay = config.wait_diagonal_armor_delay,
                        .dt = config.dt,
                        .timeout = config.timeout,
                        .max_switch_angle = config.max_switch_angle,
                        .min_switch_angle = config.min_switch_angle,
                        .switch_angle_offset = config.switch_angle_offset,
                        .switch_duration_scale = config.switch_duration_scale,
                        .switch_duration_rate = config.switch_duration_rate,
                        .switch_duration_offset = config.switch_duration_offset,
                        .min_shoot_beforehand_vel = config.min_shoot_beforehand_vel,
                        .max_chassis_angular_vel = config.max_chassis_angular_vel,
                        .track_rotate_target_delay = config.track_rotate_target_delay,
                        .track_move_target_delay = config.track_move_target_delay,
                        .min_fit_switch_count = config.min_fit_switch_count };
  config_rt_buffer_.writeFromNonRT(config_non_rt);
}
}  // namespace rm_gimbal_controllers
