/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

#include <crtk_ros2_hw/crtk_ros2_hardware_interface.h>

namespace crtk_ros2_hw {

    CallbackReturn crtkROSHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
    {

        if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
        {
            return CallbackReturn::ERROR;
        }
        m_node_handle = std::make_shared<rclcpp::Node>("hardware_interface_read_write");
        executor.add_node(m_node_handle);
        m_measured_js_subscriber = m_node_handle->create_subscription<sensor_msgs::msg::JointState>("/PSM1/measured_js",
                                                                             1,
                                                                             std::bind(&crtkROSHardwareInterface::measured_js_callback,
                                                                                       this,
                                                                                       std::placeholders::_1));

        m_servo_jp_publisher = m_node_handle->create_publisher<sensor_msgs::msg::JointState>("/PSM1/setpoint_js", 1);
        first_message_rx = false;
        m_number_of_joints = info_.joints.size();
        
        // RCLCPP_FATAL(rclcpp::get_logger("crtkROSHardwareInterface"),"The type is %s",typeid(info_.joints[1].name.c_str()).name());
        // std::cout<<typeid(info_.joints[1].name.c_str()).name()<<std::endl;

        hw_states_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_states_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_states_accelerations_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_commands_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        

        // Checks if the ROS2 control configuration matches the Hardware Interface configuration for command and state
        for (const hardware_interface::ComponentInfo & joint : info_.joints)
        {
                if (joint.command_interfaces.size() != 1)
                {
                RCLCPP_FATAL(
                    rclcpp::get_logger("crtkROSHardwareInterface"),
                    "Joint '%s' has %zu command interfaces. 1 expected.", joint.name.c_str(),
                    joint.command_interfaces.size());
                return CallbackReturn::ERROR;
                }

                if (!(joint.command_interfaces[0].name == hardware_interface::HW_IF_POSITION))
                {
                RCLCPP_FATAL(
                    rclcpp::get_logger("crtkROSHardwareInterface"),
                    "Joint '%s' has %s command interface. Expected %s & %s.", joint.name.c_str(),
                    joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                return CallbackReturn::ERROR;
                }

                if (joint.state_interfaces.size() != 1)
                {
                RCLCPP_FATAL(
                    rclcpp::get_logger("crtkROSHardwareInterface"),
                    "Joint '%s'has %zu state interfaces. 3 expected.", joint.name.c_str(),
                    joint.state_interfaces.size());
                return CallbackReturn::ERROR;
                }

                if (!(joint.state_interfaces[0].name == hardware_interface::HW_IF_POSITION ))
                {
                RCLCPP_FATAL(
                    rclcpp::get_logger("crtkROSHardwareInterface"),
                    "Joint '%s' has %s state interface. Expected %s.", joint.name.c_str(),
                    joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                return CallbackReturn::ERROR;
                }
        }
        
        RCLCPP_INFO(rclcpp::get_logger("crtkROSHardwareInterface"), "Loaded crtk_hardware_interface. ...");
        
        return CallbackReturn::SUCCESS;
    }

    
    CallbackReturn crtkROSHardwareInterface::on_activate(const rclcpp_lifecycle::State & previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("Initializing"),"Activating hardware interface");
        return CallbackReturn::SUCCESS;

    }

    CallbackReturn crtkROSHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("Initializing"),"Deactivating hardware interface");
        return CallbackReturn::SUCCESS;
    }

    std::vector<StateInterface> crtkROSHardwareInterface::export_state_interfaces()
    {
        RCLCPP_INFO(rclcpp::get_logger("Initializing"),"Exporting state_interfaces");
        RCLCPP_INFO(rclcpp::get_logger("Initializing"),"Expected Number of joints: %d, #joints",info_.joints.size(),m_number_of_joints);
        std::vector<StateInterface> state_interfaces;

        for (std::size_t i = 0; i < m_number_of_joints; ++i) {
            RCLCPP_INFO(rclcpp::get_logger("crtkROSHardwareInterface"), "Adding position state interfaces %s", info_.joints[i].name.c_str());
            state_interfaces.emplace_back(StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_positions_[i]));
            // state_interfaces.emplace_back(StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocities_[i]));
            // state_interfaces.emplace_back(StateInterface(info_.joints[i].name, hardware_interface::HW_IF_ACCELERATION, &hw_states_accelerations_[i]));
        }
        return state_interfaces;
    }

    std::vector<CommandInterface> crtkROSHardwareInterface::export_command_interfaces()
    {
        RCLCPP_INFO(rclcpp::get_logger("Initializing"),"Exporting command_interfaces");
        std::vector<CommandInterface> command_interfaces;
        for (std::size_t i = 0; i < m_number_of_joints; ++i) {
            RCLCPP_INFO(rclcpp::get_logger("crtkROSHardwareInterface"), "Adding position command interface: %s", info_.joints[i].name.c_str());
            command_interfaces.emplace_back(CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]));
        }
        return command_interfaces;
    }

    hardware_interface::return_type crtkROSHardwareInterface::read(void)
    {
        
        // maybe we should add some code to check that we're still getting some data from the crtk node
        // estimating rate of measured_js topic might also be needed
        executor.spin_some();

        // ADD a return code here!
        if(first_message_rx)
        {        
            for(std::size_t i = 0; i < m_number_of_joints;++i)
            {
                hw_states_positions_[i] = m_measured_js.position[i];
                hw_states_velocities_[i] = m_measured_js.velocity[i];
                hw_states_accelerations_[i] = m_measured_js.effort[i];
            }
        }
        else
        {
            // RCLCPP_INFO(rclcpp::get_logger("Read"),"Hardware is not talking to us!");
            for(std::size_t i = 0; i < m_number_of_joints;++i)
            {
                hw_states_positions_[i] = 0.0;
                hw_states_velocities_[i] = 0.0;
                hw_states_accelerations_[i] = 0.0;
            }
            hw_states_positions_[2]=120.0;
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type crtkROSHardwareInterface::write(void)
    {        
        executor.spin_some();
        if(first_message_rx)
        {
            m_servo_jp.name.resize(m_number_of_joints);
            m_servo_jp.position.resize(m_number_of_joints);
            m_servo_jp.velocity.resize(m_number_of_joints);
            m_servo_jp.effort.resize(m_number_of_joints);

            // Shift the name matching in first read and confirmation to on_init
            std::copy(m_measured_js.name.begin(), m_measured_js.name.end(),m_servo_jp.name.begin()); 
            
            for(std::size_t i = 0; i < m_number_of_joints;++i)
            {
                m_servo_jp.position[i] = hw_commands_positions_[i];
                m_servo_jp.velocity[i] = 0.0;
                m_servo_jp.effort[i] = 0.0;
            }
            m_servo_jp_publisher->publish(m_servo_jp);
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type crtkROSHardwareInterface::prepare_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                                const std::vector<std::string>& stop_interfaces)
    {
        
        // we can always stop a controller
        return hardware_interface::return_type::OK;
    }


    hardware_interface::return_type crtkROSHardwareInterface::perform_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                                const std::vector<std::string>& stop_interfaces)
    {
        return hardware_interface::return_type::OK;
    }

    void crtkROSHardwareInterface::measured_js_callback(const sensor_msgs::msg::JointState & measured_js)
    {
        
        bool checks[4]= {false,false,false,false};
        // Check to see if the number of joints in the message matches the joints in controller
        // for(int i = 0; i < measured_js.name.size();++i)
        // {
        //     RCLCPP_INFO(rclcpp::get_logger("crtkROSHardwareInterface"),"Joint name is %s and controlled joint is %s.", measured_js.name[i].c_str(),info_.joints[i].name.c_str());
        // }


        if(measured_js.name.size() == m_number_of_joints )
        {
            m_measured_js.name.resize(m_number_of_joints);
            m_servo_jp.name.resize(m_number_of_joints);

            std::copy(measured_js.name.begin(), measured_js.name.end(), m_measured_js.name.begin());
            std::copy(measured_js.name.begin(), measured_js.name.end(), m_servo_jp.name.begin());
            checks[0] = true;
        
            // Check for size of the position measurement is same as the joint list
            if(measured_js.position.size() == m_number_of_joints || measured_js.position.size() == m_number_of_joints+1)
            {
                m_measured_js.position.resize(m_number_of_joints);
                m_servo_jp.name.resize(m_number_of_joints);
                std::copy(measured_js.position.begin(), measured_js.position.end(), m_measured_js.position.begin());
                checks[1] = true;
            }
            else
            {
                RCLCPP_FATAL(rclcpp::get_logger("crtk_hardware_interface"), "the measurement joint position size is not same of joint size");
            }

            // Check for size of the velocity measurement is same as the joint list
            if(measured_js.velocity.size() == m_number_of_joints || measured_js.velocity.size() == m_number_of_joints+1)
            {
                m_measured_js.velocity.resize(m_number_of_joints);
                m_servo_jp.name.resize(m_number_of_joints);
                std::copy(measured_js.velocity.begin(), measured_js.velocity.end(), m_measured_js.velocity.begin());
                checks[2] = true;
            }
            else
            {
                RCLCPP_FATAL(rclcpp::get_logger("crtk_hardware_interface"), "the measurement joint velocity size is not same of joint size");
            }

            // Check for size of the effort measurement is same as the joint list
            if(measured_js.effort.size() == m_number_of_joints || measured_js.effort.size() == m_number_of_joints+1)
            {
                m_measured_js.effort.resize(m_number_of_joints);
                m_servo_jp.name.resize(m_number_of_joints);
                std::copy(measured_js.effort.begin(), measured_js.effort.end(), m_measured_js.effort.begin());
                checks[3] = true;
            }
            else
            {
                RCLCPP_FATAL(rclcpp::get_logger("crtk_hardware_interface"), "the measurement joint effort size is not same of joint size");
            }
            if(!first_message_rx)
            {
            first_message_rx = checks[0] && checks[1] && checks[2] && checks[3];
            }
        }
    }


} // namespace


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(crtk_ros2_hw::crtkROSHardwareInterface, hardware_interface::SystemInterface)
