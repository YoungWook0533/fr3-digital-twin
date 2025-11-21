#include "dyros_robot_controller/mobile/robot_controller.h"

namespace drc
{
    namespace Mobile
    {
        RobotController::RobotController(const double& dt, std::shared_ptr<Mobile::RobotData> robot_data)
        : dt_(dt), robot_data_(robot_data)
        {
            param_ = robot_data_->getKineParam();
            wheel_num_ = robot_data_->getWheelNum();
        }

        VectorXd RobotController::VelocityCommand(const VectorXd& desired_base_vel)
        {
            assert(desired_base_vel.size() == 3); // Ensure both desired_base_vel has three elements for vx, vy, omega

            Vector3d saturated_base_vel;
            saturated_base_vel.setZero();

            // TODO: acceleration Limit Saturation

            // Velocity Limit Saturation
            Vector2d desired_lin_vel = desired_base_vel.head(2); // [vx, vy]
            double desired_lin_speed = desired_lin_vel.norm();
            Vector2d desired_lin_vel_dir;
            if(fabs(desired_lin_speed) < 1E-4)
            {
                desired_lin_vel_dir.setZero();
            }
            else
            {
                desired_lin_vel_dir = desired_lin_vel / desired_lin_speed;
            }
            desired_lin_speed = std::min(std::max(desired_lin_speed, -param_.max_lin_speed), param_.max_lin_speed);
            double desired_ang_vel = std::min(std::max(desired_base_vel(2), -param_.max_ang_speed), param_.max_ang_speed);

            saturated_base_vel.head(2) = desired_lin_vel_dir * desired_lin_speed;
            saturated_base_vel(2) = desired_ang_vel;

            return computeWheelVel(saturated_base_vel);
        }

        VectorXd RobotController::computeWheelVel(const VectorXd& base_vel)
        {
            assert(base_vel.size() == 3); // Ensure base_vel has three elements for vx, vy, omega       
            return computeIKJacobian() * base_vel;
        }

        MatrixXd RobotController::computeIKJacobian()
        {
            switch (param_.type) 
            {
                case Mobile::DriveType::Differential:
                    return DifferentialIKJacobian();
                case Mobile::DriveType::Mecanum:
                    return MecanumIKJacobian();
                case Mobile::DriveType::Caster:
                    return CasterIKJacobian();
                default:
                    throw std::runtime_error("Unknown DriveType");
            }
        }

        MatrixXd RobotController::DifferentialIKJacobian()
        {
            Matrix<double,2,3> J_inv;
            J_inv.setZero();
            J_inv << 1/param_.wheel_radius, 0, -param_.base_width/(2*param_.wheel_radius),
                     1/param_.wheel_radius, 0,  param_.base_width/(2*param_.wheel_radius);
            
            return J_inv;
        }

        MatrixXd RobotController::MecanumIKJacobian()
        {
            MatrixXd J_inv(wheel_num_, 3);
            J_inv.setZero();

            for(size_t i=0; i<wheel_num_; i++)
            {
                double r = param_.wheel_radius;
                double gamma = param_.roller_angles[i];
                double p_x = param_.base2wheel_positions[i](0);
                double p_y = param_.base2wheel_positions[i](1);
                double p_t = param_.base2wheel_angles[i];

                MatrixXd A1, A2, A3;
                A1.setZero(2,3);
                A2.setZero(2,2);
                A3.setZero(1,2);

                A1 << 1, 0, -p_y,
                        0, 1, p_x;
                A2 <<  cos(p_t), sin(p_t),
                        -sin(p_t), cos(p_t);
                A3 << 1, tan(gamma);
                J_inv.row(i) = (1.0 / r) * A3 * A2 * A1;
            }

            return J_inv;
        }

        MatrixXd RobotController::CasterIKJacobian()
        {
            VectorXd steer_angle(int(wheel_num_/2));
            for(size_t i=0; i<steer_angle.size(); i++) steer_angle(i) = robot_data_->getWheelPosition()(2*i);

            MatrixXd J_tilda(wheel_num_, 3);
            J_tilda.setZero();

            for(size_t i=0; i<steer_angle.size(); i++)
            {
                double r = param_.wheel_radius;
                double b = param_.wheel_offset;
                double px = param_.base2wheel_positions[i](0);
                double py = param_.base2wheel_positions[i](1);
                double phi = steer_angle(i);

                J_tilda.block(2*i,0,2,3) << -sin(phi)/b, cos(phi)/b, (px*cos(phi) + py*sin(phi))/b -1,
                                             cos(phi)/r, sin(phi)/r, (px*sin(phi) - py*cos(phi))/r;
            }
            return J_tilda;
        }
    } // namespace Mobile
} // namespace drc