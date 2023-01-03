/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

#include <ros-control-crtk-ros-hw/crtk_ros_hardware_interface.h>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace ros_control_crtk {

    CallbackReturn crtkROSHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
    {
        if (hardware_interface::ActuatorInterface::on_init(info) != CallbackReturn::SUCCESS) {
            return CallbackReturn::ERROR;
        }
        
        init();
        RCLCPP_INFO(rclcpp::get_logger("crtkROSHardwareInterface"), "Loaded crtk_hardware_interface. ...");
        
        return CallbackReturn::SUCCESS;
    }

    void crtkROSHardwareInterface::init(void)
    {
        // add subscriber to get joint state from crtk node
        m_measured_js_subscriber =
            m_node_handle->create_subscription<sensor_msgs::msg::JointState>("measured_js",
                                                                             1,
                                                                             std::bind(&crtkROSHardwareInterface::measured_js_callback,
                                                                                       this,
                                                                                       std::placeholders::_1));
        // add publishers to send servo commands to crtk node
        m_servo_jp_publisher = m_node_handle->create_publisher<sensor_msgs::msg::JointState>("servo_jp", 1);
    }


    void crtkROSHardwareInterface::measured_js_callback(const sensor_msgs::msg::JointState & measured_js)
    {
        // check if we had already found the crtk controller
        if (!m_crtk_node_found) {
            initialize_from_crtk_node(measured_js);
        }
        // preserve local copy for controllers
        copy_measured_js_from_crtk_node(measured_js);
    }

    void crtkROSHardwareInterface::initialize_from_crtk_node(const sensor_msgs::msg::JointState & measured_js)
    {
        // let's assume everything is going to work
        m_crtk_node_found = true;

        // check that we have a valid vector of names
        if (measured_js.name.size() == 0) {
            RCLCPP_FATAL(rclcpp::get_logger("crtk_hardware_interface"), "measured_js must have a valid vector of joint names");
            m_crtk_node_found = false;
        }
        m_number_of_joints = measured_js.name.size();
        m_measured_js.name.resize(m_number_of_joints);
        // copy names only the first time, we don't support name/size change at runtime
        std::copy(measured_js.name.begin(), measured_js.name.end(), m_measured_js.name.begin());
        // resize all other vectors assuming sizes are correct
        // -- position
        {
            const size_t position_size = measured_js.position.size();
            if (position_size == m_number_of_joints) {
                m_measured_js.position.resize(m_number_of_joints);
            } else if (position_size != 0) {
                RCLCPP_FATAL(rclcpp::get_logger("crtk_hardware_interface"), "measured_js name and position vectors must have the same size");
                m_crtk_node_found = false;
            }
        }
        // velocity
        {
            const size_t velocity_size = measured_js.velocity.size();
            if (velocity_size == m_number_of_joints) {
                m_measured_js.velocity.resize(m_number_of_joints);
            } else if (velocity_size != 0) {
                RCLCPP_FATAL(rclcpp::get_logger("crtk_hardware_interface"), "measured_js name and velocity vectors must have the same size");
                m_crtk_node_found = false;
            }
        }
        // effort
        {
            const size_t effort_size = measured_js.effort.size();
            if (effort_size == m_number_of_joints) {
                m_measured_js.effort.resize(m_number_of_joints);
            } else if (effort_size != 0) {
                RCLCPP_FATAL(rclcpp::get_logger("crtk_hardware_interface"), "measured_js name and effort vectors must have the same size");
                m_crtk_node_found = false;
            }
        }

        // copy joint state
        if (!m_crtk_node_found) {
            m_number_of_joints = 0;
            m_measured_js.name.resize(0);
            m_measured_js.position.resize(0);
            m_measured_js.velocity.resize(0);
            m_measured_js.effort.resize(0);
        } else {

            // resize servo objects too
            m_servo_jp.name.resize(m_number_of_joints);
            m_servo_jp.position.resize(m_number_of_joints);
            m_servo_jp.velocity.resize(0);
            m_servo_jp.effort.resize(0);
            m_servo_jv.name.resize(m_number_of_joints);
            m_servo_jv.position.resize(0);
            m_servo_jv.velocity.resize(m_number_of_joints);
            m_servo_jv.effort.resize(0);
            // copy names only the first time, we don't support name/size change at runtime
            std::copy(m_measured_js.name.begin(), m_measured_js.name.end(), m_servo_jp.name.begin());
            std::copy(m_measured_js.name.begin(), m_measured_js.name.end(), m_servo_jv.name.begin());

            // make the first copy from measured joint state
            copy_measured_js_from_crtk_node(measured_js);
        }
    }

    void crtkROSHardwareInterface::copy_measured_js_from_crtk_node(const sensor_msgs::msg::JointState & measured_js)
    {
        // -- position
        {
            const size_t position_size = measured_js.position.size();
            if (position_size != 0) {
                if (position_size == m_measured_js.position.size()) {
                    std::copy(measured_js.position.begin(), measured_js.position.end(),
                              m_measured_js.position.begin());
                } else {
                    RCLCPP_FATAL(rclcpp::get_logger("crtk_hardware_interface"), "measured_js name and position vectors must have the same size");
                }
            }
        }
        // -- velocity
        {
            const size_t velocity_size = measured_js.velocity.size();
            if (velocity_size != 0) {
                if (velocity_size == m_measured_js.velocity.size()) {
                    std::copy(measured_js.velocity.begin(), measured_js.velocity.end(),
                              m_measured_js.velocity.begin());
                } else {
                    RCLCPP_FATAL(rclcpp::get_logger("crtk_hardware_interface"), "measured_js name and velocity vectors must have the same size");
                }
            }
        }
        // -- effort
        {
            const size_t effort_size = measured_js.effort.size();
            if (effort_size != 0) {
                if (effort_size == m_measured_js.effort.size()) {
                    std::copy(measured_js.effort.begin(), measured_js.effort.end(),
                              m_measured_js.effort.begin());
                } else {
                    RCLCPP_FATAL(rclcpp::get_logger("crtk_hardware_interface"), "measured_js name and effort vectors must have the same size");
                }
            }
        }
    }

    std::vector<StateInterface> crtkROSHardwareInterface::export_state_interfaces()
    {
        std::vector<StateInterface> state_interfaces;
        for (std::size_t i = 0; i < m_number_of_joints; ++i) {
            RCLCPP_INFO(rclcpp::get_logger("crtkROSHardwareInterface"), "Adding position, velocity and effort state interfaces: %s", info_.joints[i].name.c_str());
            state_interfaces.emplace_back(StateInterface(m_measured_js.name[i], hardware_interface::HW_IF_POSITION, &m_measured_js.position[i]));
            state_interfaces.emplace_back(StateInterface(m_measured_js.name[i], hardware_interface::HW_IF_VELOCITY, &m_measured_js.velocity[i]));
            state_interfaces.emplace_back(StateInterface(m_measured_js.name[i], hardware_interface::HW_IF_EFFORT, &m_measured_js.effort[i]));
        }
        return state_interfaces;
    }

    std::vector<CommandInterface> crtkROSHardwareInterface::export_command_interfaces()
    {
        std::vector<CommandInterface> command_interfaces;
        for (std::size_t i = 0; i < m_number_of_joints; ++i) {
            RCLCPP_INFO(rclcpp::get_logger("crtkROSHardwareInterface"), "Adding position, velocity and effort command interfaces: %s", info_.joints[i].name.c_str());
            command_interfaces.emplace_back(CommandInterface(m_measured_js.name[i], hardware_interface::HW_IF_POSITION, &m_servo_jp.position[i]));
            command_interfaces.emplace_back(CommandInterface(m_measured_js.name[i], hardware_interface::HW_IF_VELOCITY, &m_servo_jv.velocity[i]));
        }
        return command_interfaces;
    }

    hardware_interface::return_type crtkROSHardwareInterface::read(void)
    {
        // maybe we should add some code to check that we're still getting some data from the crtk node
        // estimating rate of measured_js topic might also be needed

        // ADD a return code here!
    }

    hardware_interface::return_type crtkROSHardwareInterface::write(void)
    {
        // if (velocity_interface_running_) {
        //     robot_->setSpeed(cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5],  max_vel_change_*125);
        // } else if (position_interface_running_) {
        //     robot_->servoj(mServoJP);
        // }
    }

    hardware_interface::return_type prepare_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                                const std::vector<std::string>& stop_interfaces)
    {

        // we can always stop a controller
        return hardware_interface::return_type::OK;
    }


    hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                                const std::vector<std::string>& stop_interfaces)
    {

        return hardware_interface::return_type::OK;
    }

} // namespace


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ros_control_crtk::crtkROSHardwareInterface, hardware_interface::ActuatorInterface)
