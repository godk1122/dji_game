#ifndef PLANNER_A_H
#define PLANNER_A_H

#include <iostream>
#include <string.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <traj_utils/polynomial_traj.h>
#include <traj_utils/planning_visualization.h>
#include <dji_msgs/Trajectory.h>
#include <dji_msgs/Loop.h>
#include <vector>
#include <std_msgs/Int8.h>
#include <bspline_opt/bspline_optimizer.h>
#include <dji_msgs/Trajectory.h>

#include "planner/cal_yaw.h"
// Trajectory point define 

//
/*Task A   -- Mission for 5 red pillar -- traj init  */
Eigen::Vector3d start_pos_A,start_vel_A,start_acc_A;
Eigen::Vector3d mid_pos1_A,mid_vel1_A,mid_acc1_A;
Eigen::Vector3d mid_pos2_A,mid_vel2_A,mid_acc2_A;
Eigen::Vector3d mid_pos3_A,mid_vel3_A,mid_acc3_A;
Eigen::Vector3d mid_pos4_A,mid_vel4_A,mid_acc4_A;
Eigen::Vector3d mid_pos5_A,mid_vel5_A,mid_acc5_A;
Eigen::Vector3d end_pos_A,end_vel_A,end_acc_A;

auto trajA_1 = PolynomialTraj::one_segment_traj_gen(start_pos_A, start_vel_A, start_acc_A,
                                                  mid_pos1_A, mid_vel1_A, mid_acc1_A, 3.0);
auto trajA_2 = PolynomialTraj::one_segment_traj_gen(mid_pos1_A, mid_vel1_A, mid_acc1_A,
                                                  mid_pos2_A, mid_vel2_A, mid_acc2_A, 3.0);
auto trajA_3 = PolynomialTraj::one_segment_traj_gen(mid_pos2_A, mid_vel2_A, mid_acc2_A,
                                                  mid_pos3_A, mid_vel3_A, mid_acc3_A, 3.0);
auto trajA_4 = PolynomialTraj::one_segment_traj_gen(mid_pos3_A, mid_vel3_A, mid_acc3_A,
                                                  mid_pos4_A, mid_vel4_A, mid_acc4_A, 3.0); 
auto trajA_5 = PolynomialTraj::one_segment_traj_gen(mid_pos4_A, mid_vel4_A, mid_acc4_A,
                                                  mid_pos5_A, mid_vel5_A, mid_acc5_A, 3.0); 
auto trajA_6 = PolynomialTraj::one_segment_traj_gen(mid_pos5_A, mid_vel5_A, mid_acc5_A,
                                                  end_pos_A, end_vel_A, end_acc_A, 3.0);
// yaw define
Eigen::Vector3d yawA_vec1, yawA_vec2, yawA_vec3, yawA_vec4, yawA_vec5, end_vec_A;
float start_yaw_A, taskA_yaw1, taskA_yaw2, taskA_yaw3, taskA_yaw4, taskA_yaw5, end_yaw_A; 

//
void planner_A(const dji_msgs::Loop::ConstPtr& msg) 
{
    //初始点
      // 任务A的起始点
      start_pos_A = Eigen::Vector3d(0.0, 0.0, -0.1);
      start_vel_A = Eigen::Vector3d(0.0, 0.0, 0.0);
      start_acc_A = Eigen::Vector3d(0.0, 0.0, 0.0);

      if (msg->loop_id == 0)
      {
        // A mid position1;
        mid_pos1_A[0] = msg->loop_pos.x;
        mid_pos1_A[1] = msg->loop_pos.y;
        mid_pos1_A[2] = msg->loop_pos.z;
        taskA_yaw1 = msg->loop_yaw;
        // 路径点1的速度向量
        yawA_vec1 = get_yaw_vec(taskA_yaw1);
        mid_vel1_A = yawA_vec1;
        // 路径点1的加速度向量
        mid_acc1_A = Eigen::Vector3d(0.0, 0.0, 0.0);
      }
      else if (msg->loop_id == 1)
      {
        // A mid position2;
        mid_pos2_A[0] = msg->loop_pos.x;
        mid_pos2_A[1] = msg->loop_pos.y;
        mid_pos2_A[2] = msg->loop_pos.z;
        taskA_yaw2 = msg->loop_yaw;
        // 路径点2的速度向量
        yawA_vec2 = get_yaw_vec(taskA_yaw2);
        mid_vel2_A = yawA_vec2;
        // 路径点2的加速度向量
        mid_acc2_A = Eigen::Vector3d(0.0, 0.0, 0.0);
      }
      else if (msg->loop_id == 2)
      {
        // A mid position3;
        mid_pos3_A[0] = msg->loop_pos.x;
        mid_pos3_A[1] = msg->loop_pos.y;
        mid_pos3_A[2] = msg->loop_pos.z;
        taskA_yaw3 = msg->loop_yaw;
        // 路径点3的速度向量
        yawA_vec3 = get_yaw_vec(taskA_yaw3);
        mid_vel3_A = yawA_vec3;
        // 路径点3的加速度向量
        mid_acc3_A = Eigen::Vector3d(0.0, 0.0, 0.0);
      }
      else if (msg->loop_id == 3)
      {
        // A mid position4;
        mid_pos4_A[0] = msg->loop_pos.x;
        mid_pos4_A[1] = msg->loop_pos.y;
        mid_pos4_A[2] = msg->loop_pos.z;
        taskA_yaw4 = msg->loop_yaw;
        // 路径点4的速度向量
        yawA_vec4 = get_yaw_vec(taskA_yaw4);
        mid_vel4_A = yawA_vec4;
        mid_vel4_A[0] = yawA_vec4[0];
        // 路径点4的加速度向量
        mid_acc4_A = Eigen::Vector3d(0.0, 0.0, 0.0);
      }
      else if (msg->loop_id == 4)
      {
        // A mid position5;
        mid_pos5_A[0] = msg->loop_pos.x;
        mid_pos5_A[1] = msg->loop_pos.y;
        mid_pos5_A[2] = msg->loop_pos.z;
        taskA_yaw5 = msg->loop_yaw;
        // 路径点5的速度向量
        yawA_vec5 = get_yaw_vec(taskA_yaw5);
        mid_vel5_A = yawA_vec5;
        mid_vel5_A[0] = -yawA_vec5[0];
        // 路径点5的加速度向量
        mid_acc5_A = Eigen::Vector3d(0.0, 0.0, 0.0);
      }
      else if (msg->loop_id == 5)
      {
        // A end position;
        end_pos_A[0] = msg->loop_pos.x;
        end_pos_A[1] = msg->loop_pos.y;
        end_pos_A[2] = msg->loop_pos.z;
        end_yaw_A = msg->loop_yaw;
        // 路径点6的速度向量
        end_vec_A = get_yaw_vec(end_yaw_A);
        end_vel_A = end_vec_A;
        end_vel_A[0] = -end_vec_A[0];
        end_vel_A[1] = -end_vec_A[1];
        // std::cout << "end_vel_A: " << end_vel_A << std::endl;
        // 路径点6的加速度向量
        end_acc_A = Eigen::Vector3d(0.0, 0.0, 0.0);
      }
      else
      {
        // do nothing
      }   
}

#endif