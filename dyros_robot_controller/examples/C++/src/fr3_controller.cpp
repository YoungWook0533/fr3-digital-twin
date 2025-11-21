#include "fr3_controller.hpp"

#ifndef ROBOTS_DIRECTORY
#  error "ROBOTS_DIRECTORY is not defined. Define in CMake: -DROBOTS_DIRECTORY=\"${CMAKE_SOURCE_DIR}/../robots\""
#endif

FR3Controller::FR3Controller(const double dt)
: dt_(dt)
{
  // Paths to URDF/SRDF (model files)
  const std::string urdf = std::string(ROBOTS_DIRECTORY) + "/fr3/" + "fr3.urdf";
  const std::string srdf = std::string(ROBOTS_DIRECTORY) + "/fr3/" + "fr3.srdf";

  // Instantiate dyros robot model/controller
  robot_data_ = std::make_shared<drc::Manipulator::RobotData>(urdf, srdf);
  robot_controller_ = std::make_shared<drc::Manipulator::RobotController>(dt_, robot_data_);

  // Degree of freedom
  dof_ = robot_data_->getDof();

  // --- Joint-space states (initialize to zero/snapshot defaults) ---
  q_.setZero(dof_);      
  qdot_.setZero(dof_);
  q_desired_.setZero(dof_);  
  qdot_desired_.setZero(dof_);
  q_init_.setZero(dof_);
  qdot_init_.setZero(dof_);
  tau_desired_.setZero(dof_);

  // --- Task-space states (EE pose/twist and snapshots) ---
  x_.setIdentity();         
  xdot_.setZero();          
  x_desired_.setIdentity(); 
  xdot_desired_.setZero();    
  x_init_.setIdentity();    
  xdot_init_.setZero();    

  // Print FR3 URDF info
  std::cout << "info: \n" << robot_data_->getVerbose() << std::endl; 

  // Global keyboard listener (non-blocking)
  startKeyListener_();
}

FR3Controller::~FR3Controller() 
{
  stopKeyListener_();
}

void FR3Controller::updateModel(const double current_time,
                                const std::unordered_map<std::string, double>& qpos_dict,
                                const std::unordered_map<std::string, double>& qvel_dict)
{
  // Time update (shared convention)
  sim_time_ = current_time;

  // Read joint states (joint naming must match the simulator)
  for (size_t i = 0; i < dof_; ++i) 
  {
    const std::string key = "fr3_joint" + std::to_string(i+1);
    q_(i)  = qpos_dict.at(key);
    qdot_(i) = qvel_dict.at(key);
  }

  // Push to dyros robot model and cache EE pose/twist
  robot_data_->updateState(q_, qdot_);
  x_  = robot_data_->getPose(ee_link_name_);
  xdot_ = robot_data_->getVelocity(ee_link_name_);
}

std::unordered_map<std::string, double> FR3Controller::compute() 
{
  // One-time init per mode entry (snapshot current measured states)
  if (is_mode_changed_) 
  {
    is_mode_changed_ = false;
    control_start_time_ = sim_time_;

    q_init_ = q_;
    qdot_init_ = qdot_;
    x_init_ = x_;
    xdot_init_ = xdot_;

    // Reset desired trajectories to snapshots
    q_desired_ = q_init_;
    qdot_desired_.setZero(dof_);
    x_desired_ = x_init_;
    xdot_desired_.setZero();
  }

  // --- Mode: Home (joint-space cubic to a predefined posture) ---
  if (control_mode_ == "Home") {
    Eigen::Vector7d q_home;
    q_home << 0.0, 0.0, 0.0, -M_PI/2., 0.0, M_PI/2., M_PI / 4.;

    q_desired_ = robot_controller_->moveJointPositionCubic(q_home,
                                                           Eigen::VectorXd::Zero(dof_),
                                                           q_init_,
                                                           qdot_init_,
                                                           sim_time_,
                                                           control_start_time_,
                                                           3.0);

    qdot_desired_ = robot_controller_->moveJointVelocityCubic(q_home,
                                                              Eigen::VectorXd::Zero(dof_),
                                                              q_init_,
                                                              qdot_init_,
                                                              sim_time_,
                                                              control_start_time_,
                                                              3.0);

    tau_desired_ = robot_controller_->moveJointTorqueStep(q_desired_, qdot_desired_);

    // Alternative: synthesize torque over time with a single API
    // tau_desired = robot_controller->moveJointTorqueCubic(...)
  }
  // --- Mode: QPIK (task-space, QP-based IK with cubic profiling) ---
  else if (control_mode_ == "QPIK") 
  {
    Eigen::Affine3d target_x = x_init_;
    target_x.translation() += Eigen::Vector3d(0.0, 0.1, 0.1); // +10 cm in Y and Z

    qdot_desired_ = robot_controller_->QPIKCubic(target_x, 
                                                 Eigen::VectorXd::Zero(6),
                                                 x_init_, 
                                                 xdot_init_,
                                                 sim_time_, 
                                                 control_start_time_, 
                                                 3.0, 
                                                 ee_link_name_);

    // Simple Euler integrate desired joint positions from qdot_desired
    q_desired_   = q_ + qdot_desired_ * dt_;
    tau_desired_ = robot_controller_->moveJointTorqueStep(q_desired_, qdot_desired_);
  }
  // --- Mode: Gravity Compensation (no tracking) ---
  else if (control_mode_ == "Gravity Compensation") 
  {
    tau_desired_ = robot_data_->getGravity();
  }

  // Format output for simulator actuators: joint name -> tau
  std::unordered_map<std::string, double> ctrl_dict;
  ctrl_dict.reserve(dof_);
  for (size_t i = 0; i < dof_; ++i) 
  {
    ctrl_dict.emplace("fr3_joint" + std::to_string(i+1), tau_desired_(i));
  }
  return ctrl_dict;
}

void FR3Controller::setMode(const std::string& control_mode) 
{
  // Switch control mode and trigger per-mode re-initialization
  is_mode_changed_ = true;
  control_mode_ = control_mode;
  std::cout << "Control Mode Changed: " << control_mode_ << std::endl;
}

void FR3Controller::startKeyListener_() 
{
  tty_ok_ = ::isatty(STDIN_FILENO);
  if (!tty_ok_) 
  {
    std::cout << "[FR3Controller] stdin is not a TTY; keyboard control disabled.\n";
    return;
  }
  setRawMode_();
  stop_key_ = false;
  key_thread_ = std::thread(&FR3Controller::keyLoop_, this);

  std::cout << "[FR3Controller] Keyboard: [1]=Home, [2]=QPIK, [3]=Gravity Compensation\n";
}

void FR3Controller::stopKeyListener_() 
{
  if (!tty_ok_) return;
  stop_key_ = true;
  if (key_thread_.joinable()) key_thread_.join();
  restoreTerm_();
}

void FR3Controller::setRawMode_() 
{
  if (!tty_ok_) return;
  struct termios raw;
  if (tcgetattr(STDIN_FILENO, &orig_term_) == -1) 
  {
    perror("tcgetattr");
    tty_ok_ = false;
    return;
  }
  raw = orig_term_;
  raw.c_lflag &= ~(ICANON | ECHO);
  raw.c_cc[VMIN]  = 0;
  raw.c_cc[VTIME] = 1;
  if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) == -1) 
  {
    perror("tcsetattr");
    tty_ok_ = false;
  }
}

void FR3Controller::restoreTerm_() 
{
  if (!tty_ok_) return;
  if (tcsetattr(STDIN_FILENO, TCSANOW, &orig_term_) == -1) 
  {
    perror("tcsetattr restore");
  }
}

void FR3Controller::keyLoop_() 
{
  while (!stop_key_) 
  {
    char c = 0;
    ssize_t n = ::read(STDIN_FILENO, &c, 1);
    if (n == 1) 
    {
      if (c == '1') 
      {
        setMode("Home");
      } else if (c == '2') 
      {
        setMode("QPIK");
      } else if (c == '3') 
      {
        setMode("Gravity Compensation");
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }
}
