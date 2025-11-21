#pragma once
#include <Eigen/Dense>
#include <unordered_map>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <atomic>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include "dyros_robot_controller/manipulator/robot_data.h"
#include "dyros_robot_controller/manipulator/robot_controller.h"

class FR3Controller 
{
public:
  FR3Controller(const double dt);
  ~FR3Controller();

  void updateModel(const double current_time,
                   const std::unordered_map<std::string, double>& qpos_dict,
                   const std::unordered_map<std::string, double>& qvel_dict);

  // Compute control for current mode.
  // Returns: actuator torque map (joint name -> tau [Nm]).
  std::unordered_map<std::string, double> compute();

  void setMode(const std::string& control_mode);

private:
  const double dt_;
  int dof_{0};

  // --- Joint-space states (measured / desired / snapshots) ---
  Eigen::VectorXd q_;            // measured joints
  Eigen::VectorXd qdot_;         // measured joint velocities
  Eigen::VectorXd q_desired_;    // desired joints
  Eigen::VectorXd qdot_desired_; // desired joint velocities
  Eigen::VectorXd q_init_;       // snapshot at mode entry
  Eigen::VectorXd qdot_init_;    // snapshot at mode entry
  Eigen::VectorXd tau_desired_;  // output torques

  // --- Task-space (end-effector) states (measured / desired / snapshots) ---
  Eigen::Affine3d x_;            // measured EE pose (4x4)
  Eigen::Vector6d xdot_;         // measured EE twist [vx, vy, vz, wx, wy, wz]
  Eigen::Affine3d x_desired_;    // desired EE pose
  Eigen::Vector6d xdot_desired_; // desired EE twist
  Eigen::Affine3d x_init_;       // snapshot at mode entry
  Eigen::Vector6d xdot_init_;    // snapshot at mode entry
  std::string ee_link_name_{"fr3_link8"}; // EE link name (FR3 URDF)

  // --- Mode bookkeeping (unified naming with Python example) ---
  std::string control_mode_{"Home"};
  bool   is_mode_changed_{true};
  double sim_time_{0.0};
  double control_start_time_{0.0};

  // Dyros model/controller handles
  std::shared_ptr<drc::Manipulator::RobotData>       robot_data_;
  std::shared_ptr<drc::Manipulator::RobotController> robot_controller_;

  // --- Keyboard interface (non-blocking; background thread) ---
  void startKeyListener_();
  void stopKeyListener_();
  void keyLoop_();

  void setRawMode_();
  void restoreTerm_();

  std::atomic<bool> stop_key_{false};
  std::thread key_thread_;
  bool tty_ok_{false};
  struct termios orig_term_{};
};
