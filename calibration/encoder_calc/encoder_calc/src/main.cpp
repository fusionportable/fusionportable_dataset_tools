//
// Created by echo on 23-3-30.
//

#include "../include/main.h"

#include <unistd.h>

#include <fstream>

#include "std_msgs/Int32.h"
#include "std_srvs/Empty.h"
#include "std_srvs/EmptyRequest.h"
#include "std_srvs/EmptyResponse.h"

main_fuction m;
nav_msgs::Path encoder_path, gps_path, encoder_inc_path, refined_encoder_path;
// void odomCallback(const sensor_msgs::JointState::ConstPtr& msg,const
// sensor_msgs::JointState::ConstPtr& msg2)
void odomCallback(const sensor_msgs::JointState::ConstPtr &msg) {
  nav_msgs::Odometry result;
  result.header = msg->header;

  if (m.first_call) {
    m.first_call = false;
  } else {
    //        double dt = (msg->header.stamp - m.last_time).toSec();
    double dl = msg->position[0] - m.distance_x_prev;
    double dr = msg->position[1] - m.distance_y_prev;

    if (dl > 1024 * 1024)
      dl = dl - 4294967.296;
    else if (dl < -1024 * 1024)
      dl = dl + 4294967.296;

    if (dr > 1024 * 1024)
      dr = dr - 4294967.296;
    else if (dr < -1024 * 1024)
      dr = dr + 4294967.296;

    double vl = (dl) / 1000 * M_PI * 2 * m.wheel_radius_l;
    double vr = (dr) / 1000 * M_PI * 2 * m.wheel_radius_r;

    double Vave = (vl + vr) / 2;
    m.yaw = m.yaw + ((vr - vl) / m.wheel_track);
    m.curr_x = m.curr_x + Vave * cos(m.yaw);
    m.curr_y = m.curr_y + Vave * sin(m.yaw);

    geometry_msgs::PoseStamped encoder_inc_pose;
    encoder_inc_pose.pose.position.x = vl / m.wheel_radius_l;
    encoder_inc_pose.pose.position.y = vr / m.wheel_radius_r;
    encoder_inc_pose.header.stamp = result.header.stamp;
    encoder_inc_path.poses.push_back(encoder_inc_pose);
    //        std::cerr<<"vx: "<<vx<<" vy: "<<vy<<" (vx-vy)/m.wheel_distance):
    //        "<<(vx-vy)/m.wheel_distance
    //        <<" v ave: "<<Vave<<" yaw: "<<m.yaw<<" m.curr_x: "<<m.curr_x<<"
    //        m.curr_y: "<<m.curr_y<<std::endl;
  }
  geometry_msgs::Quaternion temp_q;
  temp_q = tf::createQuaternionMsgFromRollPitchYaw(0, 0, m.yaw);
  result.pose.pose.orientation.x = temp_q.x;
  result.pose.pose.orientation.y = temp_q.y;
  result.pose.pose.orientation.z = temp_q.z;
  result.pose.pose.orientation.w = temp_q.w;

  result.pose.pose.position.x = m.curr_x;
  result.pose.pose.position.y = m.curr_y;

  result.header.frame_id = "camera_init";

  m.encoder_odom_pub.publish(result);
  m.distance_x_prev = msg->position[0];
  m.distance_y_prev = msg->position[1];
  m.last_time = msg->header.stamp;

  geometry_msgs::PoseStamped encoder_pose;
  encoder_pose.pose = result.pose.pose;
  encoder_pose.header = result.header;
  encoder_path.poses.push_back(encoder_pose);
  encoder_path.header.stamp = result.header.stamp;
  encoder_path.header.frame_id = "camera_init";
  if (encoder_path.poses.size() % 100 == 0)
    m.encoder_path_pub.publish(encoder_path);
}

void lla2enu(const Eigen::Vector3d &init_lla, const Eigen::Vector3d &point_lla,
             Eigen::Vector3d *point_enu) {
  static GeographicLib::LocalCartesian local_cartesian;
  local_cartesian.Reset(init_lla(0), init_lla(1), init_lla(2));
  local_cartesian.Forward(point_lla(0), point_lla(1), point_lla(2),
                          point_enu->data()[0], point_enu->data()[1],
                          point_enu->data()[2]);
}

std::deque<sensor_msgs::Imu::ConstPtr> imu_buffer;
double last_timestamp_imu = 0;
void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in) {
  sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));
  if (msg->header.stamp.toSec() < last_timestamp_imu) {
    ROS_WARN("imu loop back, clear buffer");
    imu_buffer.clear();
  }
  last_timestamp_imu = msg->header.stamp.toSec();
  imu_buffer.push_back(msg);
}

bool gps_init_flag = false;
nav_msgs::Odometry gnss_odom;
Eigen::Vector3d ori_lla;
geometry_msgs::PoseStamped gps_pose;
void gnss_cbk(const nav_msgs::Odometry::ConstPtr &gps_msg) {
  nav_msgs::Odometry::Ptr msg(new nav_msgs::Odometry(*gps_msg));
  Eigen::Vector3d c_lla, c_enu;
  Eigen::Matrix3d c_rot;

  //    msg->header.stamp=ros::Time(gps_msg->header.stamp.sec+gps_msg->header.stamp.nsec*1e-6);
  if (isnan(msg->pose.pose.position.x) || isnan(msg->pose.pose.position.y) ||
      isnan(msg->pose.pose.position.z)) {
    std::cout << " gps error " << std::endl;
    return;
  }

  if (!gps_init_flag) {
    ori_lla << msg->pose.pose.position.y, msg->pose.pose.position.x,
        msg->pose.pose.position.z;  // ned
    gps_pose.pose.position.x = 0;
    gps_pose.pose.position.y = 0;
    gps_pose.pose.position.z = 0;

    gps_pose.header.stamp = msg->header.stamp;
    gps_pose.header.frame_id = "camera_init";
    gps_path.poses.push_back(gps_pose);
    gps_init_flag = true;
    return;
  }

  c_lla << msg->pose.pose.position.y, msg->pose.pose.position.x,
      msg->pose.pose.position.z;  // ned
  lla2enu(ori_lla, c_lla, &c_enu);
  //    std::cout<<  " ori_lla "<< ori_lla<<std::endl;
  //    std::cout<<  " c_lla "<< c_lla<<std::endl;
  //    std::cout<<  " c_enu "<< c_enu<<std::endl;
  c_enu(2) = 0;

  double last_pose_dis = sqrt(pow(gps_pose.pose.position.x - c_enu.x(), 2) +
                              pow(gps_pose.pose.position.y - c_enu.y(), 2));

  if (msg->pose.covariance[0] < m.gps_cov_threshold &&
      (last_pose_dis > m.gps_interval_threshold)) {
    gnss_odom.header.frame_id = "camera_init";
    gnss_odom.header.stamp = msg->header.stamp;
    gnss_odom.pose.pose.position.x = c_enu.x();
    gnss_odom.pose.pose.position.y = c_enu.y();
    gnss_odom.pose.pose.position.z = 0;

    m.gps_odom_pub.publish(gnss_odom);
    gps_pose.pose = gnss_odom.pose.pose;
    gps_pose.header = gnss_odom.header;
    gps_path.poses.push_back(gps_pose);
    gps_path.header.stamp = gnss_odom.header.stamp;
    gps_path.header.frame_id = "camera_init";
    m.gps_path_pub.publish(gps_path);
  }
}

int iter_num = 10;
bool cali_cbk(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  std::cout << "calculate encoder parameters..........." << std::endl;
  auto seg_size = gps_path.poses.size() - 1;
  auto encoder_size = encoder_inc_path.poses.size() - 1;
  auto imu_size = imu_buffer.size() - 1;
  std::cout << "gps: " << seg_size << " encoder_size: " << encoder_size
            << std::endl;
  Eigen::Matrix<double, Eigen::Dynamic, 1> b;
  Eigen::Matrix<double, Eigen::Dynamic, 6> H;
  Eigen::Vector3d gps2wheel(-1.03, 0, 0);  // ins
  // Eigen::Vector3d gps2wheel(0, -0.26, 0); //left
  //    Eigen::Vector3d gps2wheel(0, 0.26, 0); //right
  double gps_rot_yaw = 0;
  b.resize(3 * seg_size, 1);
  H.resize(3 * seg_size, 6);
  for (int k = 0; k < iter_num; k++) {
    H.setZero();
    b.setZero();
    int last_j = 0, last_imu = 0;
    double gps_yaw = 0;
    geometry_msgs::Quaternion temp_q;
    temp_q.x = 0;
    temp_q.y = 0;
    temp_q.z = 0;
    temp_q.w = 1;
    for (int i = 0; i < seg_size; i++) {
      Eigen::Vector3d encoder_inc_xytheta;
      Eigen::Matrix3d encoder_H;
      Eigen::Vector3d gps_k_pos, gps_k_1_pos, g2v_odom;
      Eigen::Matrix3d gps_k_rot, gps_k_1_rot;
      Eigen::Quaterniond tq;

      // calculate direction
      double imu_yaw = 0;
      for (int imu_idx = last_imu; imu_idx < imu_size; imu_idx++) {
        double dt = 0;
        if (imu_buffer.at(imu_idx)->header.stamp >
            gps_path.poses[i + 1].header.stamp) {
          if (imu_idx > 0) last_imu = imu_idx - 1;
          break;
        }
        if (imu_buffer.at(imu_idx + 1)->header.stamp <
            gps_path.poses[i].header.stamp) {
          continue;
        }
        if (imu_buffer.at(imu_idx)->header.stamp <=
                gps_path.poses[i].header.stamp &&
            imu_buffer.at(imu_idx + 1)->header.stamp >=
                gps_path.poses[i].header.stamp)
          dt = (imu_buffer.at(imu_idx + 1)->header.stamp -
                gps_path.poses[i].header.stamp)
                   .toSec();
        else if (imu_buffer.at(imu_idx)->header.stamp <=
                     gps_path.poses[i + 1].header.stamp &&
                 imu_buffer.at(imu_idx + 1)->header.stamp >=
                     gps_path.poses[i + 1].header.stamp)
          dt = (gps_path.poses[i + 1].header.stamp -
                imu_buffer.at(imu_idx)->header.stamp)
                   .toSec();
        else
          dt = (imu_buffer.at(imu_idx + 1)->header.stamp -
                imu_buffer.at(imu_idx)->header.stamp)
                   .toSec();

        imu_yaw += imu_buffer.at(imu_idx)->angular_velocity.z *
                   dt;  // 3dm imu roll=M_PI
      }
      gps_yaw += imu_yaw;
      gps_path.poses.at(i + 1).pose.position.z = gps_yaw;
      // VIWO formula 26 RT*RT*(T_K_1*P-T_K*P)
      gps_k_pos << gps_path.poses[i].pose.position.x,
          gps_path.poses[i].pose.position.y, 0;
      gps_k_1_pos << gps_path.poses[i + 1].pose.position.x,
          gps_path.poses[i + 1].pose.position.y, 0;
      tq.x() = temp_q.x;
      tq.y() = temp_q.y;
      tq.z() = temp_q.z;
      tq.w() = temp_q.w;
      gps_k_rot = tq.toRotationMatrix();
      temp_q = tf::createQuaternionMsgFromRollPitchYaw(0, 0, gps_yaw);
      tq.z() = temp_q.z;
      tq.w() = temp_q.w;
      gps_k_1_rot = tq.toRotationMatrix();
      g2v_odom =
          gps_k_rot.transpose() * ((gps_k_1_rot * gps2wheel + gps_k_1_pos) -
                                   (gps_k_rot * gps2wheel + gps_k_pos));

      encoder_inc_xytheta.setZero();
      encoder_H.setZero();
      for (int j = last_j; j < encoder_size; j++) {
        double dt = 0;
        if (encoder_inc_path.poses[j].header.stamp >
            gps_path.poses[i + 1].header.stamp) {
          if (j > 0) last_j = j - 1;
          break;
        }
        if (encoder_inc_path.poses[j + 1].header.stamp <
            gps_path.poses[i].header.stamp)
          continue;

        if (encoder_inc_path.poses[j].header.stamp <
                gps_path.poses[i].header.stamp &&
            encoder_inc_path.poses[j + 1].header.stamp >
                gps_path.poses[i].header.stamp)
          dt = (encoder_inc_path.poses[j + 1].header.stamp -
                gps_path.poses[i].header.stamp)
                   .toSec();
        else if (encoder_inc_path.poses[j].header.stamp <
                     gps_path.poses[i + 1].header.stamp &&
                 encoder_inc_path.poses[j + 1].header.stamp >
                     gps_path.poses[i + 1].header.stamp)
          dt = (gps_path.poses[i + 1].header.stamp -
                encoder_inc_path.poses[j].header.stamp)
                   .toSec();
        else
          dt = (encoder_inc_path.poses[j + 1].header.stamp -
                encoder_inc_path.poses[j].header.stamp)
                   .toSec();

        dt /= (encoder_inc_path.poses[j + 1].header.stamp -
               encoder_inc_path.poses[j].header.stamp)
                  .toSec();

        double vl, vr, vd;
        vl = encoder_inc_path.poses[j].pose.position.x * m.wheel_radius_l * dt;
        vr = encoder_inc_path.poses[j].pose.position.y * m.wheel_radius_r * dt;
        vd = (vr - vl) / m.wheel_track;
        encoder_inc_xytheta(2) += vd;
        encoder_inc_xytheta(0) +=
            (vl + vr) * cos(encoder_inc_xytheta(2)) / 2.0f;
        encoder_inc_xytheta(1) +=
            (vl + vr) * sin(encoder_inc_xytheta(2)) / 2.0f;
        encoder_H(0, 0) += encoder_inc_path.poses[j].pose.position.x *
                           cos(encoder_inc_xytheta(2)) / 2.0f * dt;
        encoder_H(0, 1) += encoder_inc_path.poses[j].pose.position.y *
                           cos(encoder_inc_xytheta(2)) / 2.0f * dt;
        encoder_H(1, 0) += encoder_inc_path.poses[j].pose.position.x *
                           sin(encoder_inc_xytheta(2)) / 2.0f * dt;
        encoder_H(1, 1) += encoder_inc_path.poses[j].pose.position.y *
                           sin(encoder_inc_xytheta(2)) / 2.0f * dt;
        encoder_H(2, 0) -=
            encoder_inc_path.poses[j].pose.position.x / m.wheel_track * dt;
        encoder_H(2, 1) +=
            encoder_inc_path.poses[j].pose.position.y / m.wheel_track * dt;
        encoder_H(2, 2) -= (vr - vl) / m.wheel_track / m.wheel_track;
        //                    encoder_H(0,2) += (vl + vr) *
        //                    sin(encoder_inc_xytheta(2)) / 2.0f*
        //                                      (vr - vl)/ m.wheel_track  /
        //                                      m.wheel_track;
        //                    encoder_H(1,2) -= (vl + vr) *
        //                    cos(encoder_inc_xytheta(2)) / 2.0f*
        //                                      (vr - vl)/ m.wheel_track  /
        //                                      m.wheel_track;
      }

      H.block<3, 3>(3 * i, 0) = encoder_H;
      b.block<3, 1>(3 * i, 0) = encoder_inc_xytheta;
      // init gps rot

      b(3 * i) -=
          g2v_odom[0] * cos(gps_rot_yaw) - g2v_odom[1] * sin(gps_rot_yaw);
      b(3 * i + 1) -=
          g2v_odom[0] * sin(gps_rot_yaw) + g2v_odom[1] * cos(gps_rot_yaw);
      b(3 * i + 2) -= imu_yaw;
      H(3 * i, 3) -=
          -g2v_odom[0] * sin(gps_rot_yaw) - g2v_odom[1] * cos(gps_rot_yaw);
      H(3 * i + 1, 3) -=
          g2v_odom[0] * cos(gps_rot_yaw) - g2v_odom[1] * sin(gps_rot_yaw);
      H(3 * i, 4) -= -imu_yaw * sin(gps_rot_yaw);
      H(3 * i, 5) -= -imu_yaw * cos(gps_rot_yaw);
      H(3 * i + 1, 4) -= imu_yaw * cos(gps_rot_yaw);
      H(3 * i + 1, 5) -= -imu_yaw * sin(gps_rot_yaw);
    }
    //        std::cout<<"H: " <<H<<std::endl;
    //        std::cout<<"b: " <<b<<std::endl;
    // solver
    Eigen::VectorXd r_x = H.colPivHouseholderQr().solve(b);
    m.wheel_radius_l -= r_x[0];
    m.wheel_radius_r -= r_x[1];
    m.wheel_track -= r_x[2];
    gps_rot_yaw -= r_x[3];
    gps2wheel[0] -= r_x[4];
    gps2wheel[1] -= r_x[5];
    std::cout << k << " iter calculate result: " << m.wheel_radius_l << " "
              << m.wheel_radius_r << " " << m.wheel_track << " " << gps_rot_yaw
              << " " << gps2wheel.transpose() << std::endl;
  }

  // visualize refined path
  double yaw = 0, curr_x = 0, curr_y = 0;
  for (int j = 0; j < encoder_inc_path.poses.size(); j++) {
    double vl, vr;
    vl = encoder_inc_path.poses[j].pose.position.x * m.wheel_radius_l;
    vr = encoder_inc_path.poses[j].pose.position.y * m.wheel_radius_r;
    double Vave = (vl + vr) / 2;
    yaw = yaw + ((vr - vl) / m.wheel_track);
    curr_x = curr_x + Vave * cos(yaw);
    curr_y = curr_y + Vave * sin(yaw);

    geometry_msgs::Quaternion temp_q;
    temp_q = tf::createQuaternionMsgFromRollPitchYaw(0, 0, yaw);
    geometry_msgs::PoseStamped encoder_pose;
    encoder_pose.pose.position.x = curr_x;
    encoder_pose.pose.position.y = curr_y;
    encoder_pose.pose.orientation.z = temp_q.z;
    encoder_pose.pose.orientation.w = temp_q.w;
    encoder_pose.header = encoder_inc_path.poses[j].header;
    refined_encoder_path.poses.push_back(encoder_pose);
  }
  refined_encoder_path.header.stamp =
      encoder_inc_path.poses.back().header.stamp;
  refined_encoder_path.header.frame_id = "camera_init";
  m.refined_encoder_path_pub.publish(refined_encoder_path);

  // publish rot gps to wheel_0 frame

  Eigen::Vector2d gps_xy, new_gps_xy, g2v(gps2wheel[0], gps2wheel[1]);
  Eigen::Matrix2d gps_g0_gk_yaw, gps_g_v_yaw;
  for (int i = 0; i < gps_path.poses.size(); i++) {
    // p_vo_vk=T_v0_g0 * T_g0_gk * p_g_V
    gps_xy << gps_path.poses[i].pose.position.x,
        gps_path.poses[i].pose.position.y;
    gps_g0_gk_yaw << cos(gps_path.poses[i].pose.position.z),
        -sin(gps_path.poses[i].pose.position.z),
        sin(gps_path.poses[i].pose.position.z),
        cos(gps_path.poses[i].pose.position.z);
    gps_g_v_yaw << cos(gps_rot_yaw), -sin(gps_rot_yaw), sin(gps_rot_yaw),
        cos(gps_rot_yaw);
    new_gps_xy = gps_g_v_yaw * (gps_g0_gk_yaw * g2v + gps_xy - g2v);
    gps_path.poses[i].pose.position.x = new_gps_xy[0];
    gps_path.poses[i].pose.position.y = new_gps_xy[1];
  }
  m.gps_path_pub.publish(gps_path);

  std::ofstream ofs;
  ofs.open("../data/RTK.txt", std::ios::out);
  if (!ofs.is_open()) {
    ROS_ERROR("Failed to open traj_file RTK");
  }

  for (const auto &p : gps_path.poses) {
    ofs << std::fixed << std::setprecision(6) << p.header.stamp.toSec() << " "
        << std::setprecision(15) << p.pose.position.x << " "
        << p.pose.position.y << " " << p.pose.position.z << " "
        << p.pose.orientation.x << " " << p.pose.orientation.y << " "
        << p.pose.orientation.z << " " << p.pose.orientation.w << std::endl;
  }
  ofs.close();

  ofs.open("../data/Calibrated.txt", std::ios::out);
  if (!ofs.is_open()) {
    ROS_ERROR("Failed to open traj_file Calibrated");
  }

  for (const auto &p : refined_encoder_path.poses) {
    ofs << std::fixed << std::setprecision(6) << p.header.stamp.toSec() << " "
        << std::setprecision(15) << p.pose.position.x << " "
        << p.pose.position.y << " " << p.pose.position.z << " "
        << p.pose.orientation.x << " " << p.pose.orientation.y << " "
        << p.pose.orientation.z << " " << p.pose.orientation.w << std::endl;
  }
  ofs.close();

  ofs.open("../data/CAD.txt", std::ios::out);
  if (!ofs.is_open()) {
    ROS_ERROR("Failed to open traj_file CAD");
  }

  for (const auto &p : encoder_path.poses) {
    ofs << std::fixed << std::setprecision(6) << p.header.stamp.toSec() << " "
        << std::setprecision(15) << p.pose.position.x << " "
        << p.pose.position.y << " " << p.pose.position.z << " "
        << p.pose.orientation.x << " " << p.pose.orientation.y << " "
        << p.pose.orientation.z << " " << p.pose.orientation.w << std::endl;
  }
  ofs.close();

  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "encoder_result");
  ros::NodeHandle nh("~");
  m.encoder_odom_pub = nh.advertise<nav_msgs::Odometry>("encoder_odom", 1000);
  m.gps_odom_pub = nh.advertise<nav_msgs::Odometry>("gps_odom", 1000);
  m.encoder_path_pub = nh.advertise<nav_msgs::Path>("encoder_path", 1000);
  m.gps_path_pub = nh.advertise<nav_msgs::Path>("gps_path", 1000);
  m.refined_encoder_path_pub =
      nh.advertise<nav_msgs::Path>("refined_encoder_path", 1000);

  message_filters::Subscriber<sensor_msgs::JointState> l_encoder_sub;
  //    message_filters::Subscriber<sensor_msgs::JointState> r_encoder_sub;
  //    typedef
  //    message_filters::sync_policies::ApproximateTime<sensor_msgs::JointState,
  //    sensor_msgs::JointState> syncpolicy;//时间戳对齐规则 typedef
  //    message_filters::Synchronizer<syncpolicy> Sync; boost::shared_ptr<Sync>
  //    sync_;//时间同步器
  //    l_encoder_sub.subscribe(nh,"/mini_hercules/encoder_left", 200000);
  //    r_encoder_sub.subscribe(nh,"/mini_hercules/encoder_right", 200000);
  //    sync_.reset(new Sync(syncpolicy(10), l_encoder_sub, r_encoder_sub));
  //    sync_->registerCallback(boost::bind(&odomCallback, _1, _2));

  ros::Subscriber joint_sub =
      nh.subscribe("/mini_hercules/encoder", 200000, odomCallback);

  ros::Subscriber sub = nh.subscribe("/3dm_ins/nav/odom", 200000, gnss_cbk);
  //    ros::Subscriber sub = nh.subscribe("/3dm_ins/gnss_right/odom", 1000,
  //    gnss_cbk);
  ros::ServiceServer service = nh.advertiseService("cali_encoder", cali_cbk);
  ros::Subscriber sub_imu =
      nh.subscribe("/3dm_ins/imu/data_raw", 200000, imu_cbk,
                   ros::TransportHints().tcpNoDelay());

  ros::Rate r(5000);
  imu_buffer.clear();
  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
