#pragma once
#include <vector>
#include <Eigen/Dense>
#include "math_type_define.h"

using namespace Eigen;

namespace drc
{
    namespace Mobile
    {
        /// @brief Enum class defining the type of mobile drive system. (Differential, Mecanum, Caster)
        enum class DriveType 
        {
            Differential, // Differential drive (2-wheel)
            Mecanum,      // Mecanum wheel drive
            Caster        // Caster-based omnidirectional drive
        };

        /**
         * @brief Kinematic parameters for a mobile robot base.
         * 
         * This structure contains geometry and dynamic constraints for various mobile base types 
         * (Differential, Mecanum, or Caster). 
         * 
         * Required fields and valid values vary depending on the selected drive type:
         *
         * @param type              (RobotData::Mobile::DriveType) [Required] Drive type of the mobile base. Must be one of:
         *                          - DriveType::Differential
         *                          - DriveType::Mecanum
         *                          - DriveType::Caster
         *
         * @param wheel_radius      (double) [Required] Radius of the wheels [m]
         * @param max_lin_speed     [Optional] Maximum linear speed in m/s. Default = 2.0.
         *
         * @param max_ang_speed     (double) [Optional] Maximum angular speed in rad/s. Default = 2.0.
         *
         * @param max_lin_acc       (double) [Optional] Maximum linear acceleration in m/s². Default = 2.0.
         *
         * @param max_ang_acc       (double) [Optional] Maximum angular acceleration in rad/s². Default = 2.0.
         *
         * @param base_width        (double) [Required if type == DriveType::Differential]
         *                          Distance between left and right wheels [m].
         *
         * @param roller_angles     (std::vector<double>) [Required if type == DriveType::Mecanum]
         *                          Vector of roller angles [rad] between base and wheel frame.
         *
         * @param base2wheel_positions 
         *                          (std::vector<Eigen::Vector2d>) [Required if type == DriveType::Mecanum or Caster]
         *                          Positions of wheels relative to base frame [m].
         *
         * @param base2wheel_angles (std::vector<double>) [Required if type == DriveType::Mecanum or Caster]
         *                          Orientation angle of each wheel relative to base frame [rad].
         *
         * @param wheel_offset      (double) [Required if type == DriveType::Caster]
         *                          Offset from caster joint rotation axis to wheel center [m].
        */
        struct KinematicParam
        {
            DriveType type;

            double wheel_radius;       // Radius of the wheels [m]
            double max_lin_speed = 2;  // Maximum linear speed of the robot (m/s)
            double max_ang_speed = 2;  // Maximum angular speed of the robot (rad/s)
            double max_lin_acc   = 2;  // Maximum linear acceleration of the robot (m/s^2)
            double max_ang_acc   = 2;  // Maximum angular acceleration of the robot (rad/s^2)
            double base_width; // Distance between the left and right wheels [m]
            std::vector<double> roller_angles;          // Angles of the rollers wrt the wheels [rad]
            std::vector<Vector2d> base2wheel_positions; // Positions of the wheels in the robot frame [x,y] [m]
            std::vector<double> base2wheel_angles;      // Angles of the wheels in the robot frame [rad]
            double wheel_offset;                        // Offset from the rotaiting axis to the wheel [m]
        };
    }

    namespace Manipulator
    {
        /**
         * @brief Result of minimum distance computation between links.
         *
         * Used for self-collision avoidance or proximity monitoring.
         * This structure contains the minimum distance between robot links and its sensitivity
         * (gradient and time derivative) with respect to the joint configuration.
         *
         * @param distance   (double)   Minimum distance between any two links (in meters).
         * @param grad       (VectorXd) Gradient of the minimum distance with respect to joint positions q.
         * @param grad_dot   (VectorXd) Time derivative of grad.
        */
        struct MinDistResult
        {
            double   distance;  // Minimum distance [m]
            VectorXd grad;      // Gradient of distance w.r.t. joint positions
            VectorXd grad_dot;  // Time derivative of gradient

            /**
             * @brief Initializes all fields to zero with given joint size.
             * @param size Number of joints
             */
            void setZero(const int size)
            {
                distance = 0;
                grad.setZero(size);
                grad_dot.setZero(size);
            }
        };

        /**
         * @brief Result of manipulability metric computation.
         *
         * This structure stores the scalar manipulability index
         * and its sensitivity with respect to joint positions.
         *
         * @param manipulability (double)   Scalar value representing manipulability.
         * @param grad           (VectorXd) Gradient of manipulability with respect to joint positions q.
         * @param grad_dot       (VectorXd) Time derivative of grad.
        */
        struct ManipulabilityResult
        {
            double   manipulability; // Scalar manipulability measure
            VectorXd grad;           // Gradient of manipulability w.r.t. joint positions
            VectorXd grad_dot;       // Time derivative of the gradient

            /**
             * @brief Initializes all fields to zero with given joint size.
             * @param size Number of joints
             */
            void setZero(const int size)
            {
                manipulability = 0;
                grad.setZero(size);
                grad_dot.setZero(size);
            }
        };
    }

    namespace MobileManipulator
    {
        /**
         * @brief Index offsets used in unified joint state vectors.
         *
         * In a mobile manipulator system, the full joint vector is typically composed of:
         * - Virtual joints (e.g., floating base or task-space constraints)
         * - Manipulator joints (e.g., 7-DOF arm)
         * - Mobile base joints (e.g., wheel actuators)
         *
         * This struct defines the starting indices of each group within a single concatenated joint vector.
         *
         * @param virtual_start (int) Index at which virtual joints start in the full joint vector.
         * @param mani_start    (int) Index at which manipulator joints start.
         * @param mobi_start    (int) Index at which mobile wheel joints start.
        */
        struct JointIndex
        {
            int virtual_start; // Index where virtual joints start (e.g., floating base)
            int mani_start;    // Index where manipulator joints start
            int mobi_start;    // Index where mobile wheel joints start
        };

         /**
         * @brief Index offsets used in unified actuator vectors.
         *
         * The actuator vector (e.g., torque, effort, command) may be composed of manipulator actuators
         * and mobile base actuators. This struct defines the starting indices of each group within that vector.
         *
         * @param mani_start (int) Index at which manipulator actuator commands (e.g., torques) begin.
         * @param mobi_start (int) Index at which mobile wheel actuator commands begin.
        */
        struct ActuatorIndex
        {
            int mani_start;    // Index where manipulator actuators start
            int mobi_start;    // Index where mobile wheel actuators start
        };
    } // namespace MobileManipulator
} // namespace drc