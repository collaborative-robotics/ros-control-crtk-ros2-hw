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
        m_setpoint_js_subscriber = m_node_handle->create_subscription<sensor_msgs::msg::JointState>("/PSM1/measured_js",
                                                                             1,
                                                                             std::bind(&crtkROSHardwareInterface::measured_js_callback,
                                                                                       this,
                                                                                       std::placeholders::_1));
        // m_operating_state_subscriber = m_node_handle->create_subscription<crtk_msgs::msg::OperatingState>("/PSM1/operating_state",
        //                                                                      1,
        //                                                                      std::bind(&crtkROSHardwareInterface::operating_state_callback,
        //                                                                                this,
        //                                                                                std::placeholders::_1));


        m_servo_jp_publisher = m_node_handle->create_publisher<sensor_msgs::msg::JointState>("/PSM1/move_jp", 1);
        m_state_command_publisher = m_node_handle->create_publisher<crtk_msgs::msg::StringStamped>("/PSM1/state_command", 1);

        first_message_rx = false;
        homed_ = false;
        ready_= false;
        m_number_of_joints = info_.joints.size();
        


        hw_states_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
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

    CallbackReturn crtkROSHardwareInterface::on_configure(const rclcpp_lifecycle::State & previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("ConfigurecrtkROSHardwareInteface"), "Configuring ...please wait...");
        for (uint i = 0; i < hw_states_positions_.size(); i++)
            {
                hw_states_positions_[i] = 0;
                hw_commands_positions_[i] = 0;
            }
        RCLCPP_INFO(rclcpp::get_logger("ConfigurecrtkROSHardwareInteface"), "Successfully configured!");
        return CallbackReturn::SUCCESS;
    }

    std::vector<StateInterface> crtkROSHardwareInterface::export_state_interfaces()
    {
        RCLCPP_INFO(rclcpp::get_logger("Initializing"),"Exporting state_interfaces");
        std::vector<StateInterface> state_interfaces;

        for (std::size_t i = 0; i < info_.joints.size(); ++i) {
            RCLCPP_INFO(rclcpp::get_logger("crtkROSHardwareInterface"), "Adding position state interfaces %s", info_.joints[i].name.c_str());
            state_interfaces.emplace_back(StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_positions_[i]));
        }
        return state_interfaces;
    }

       std::vector<CommandInterface> crtkROSHardwareInterface::export_command_interfaces()
    {
        RCLCPP_INFO(rclcpp::get_logger("Initializing"),"Exporting command_interfaces");
        std::vector<CommandInterface> command_interfaces;
        for (std::size_t i = 0; i < info_.joints.size(); ++i) {
            RCLCPP_INFO(rclcpp::get_logger("crtkROSHardwareInterface"), "Adding position command interface: %s", info_.joints[i].name.c_str());
            command_interfaces.emplace_back(CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]));
        }
        return command_interfaces;
    }

    
    CallbackReturn crtkROSHardwareInterface::on_activate(const rclcpp_lifecycle::State & previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("Activation"),"Activating hardware interface");
        for (uint i = 0; i < hw_states_positions_.size(); i++)
        {
            hw_commands_positions_[i] = hw_states_positions_[i];
        }
        hw_commands_positions_[2]= 0.12;
        return CallbackReturn::SUCCESS;

    }

    CallbackReturn crtkROSHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("Deactivation"),"Deactivating hardware interface");
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type crtkROSHardwareInterface::read(void)
    {
        
        // maybe we should add some code to check that we're still getting some data from the crtk node
        // estimating rate of measured_js topic might also be needed
        executor.spin_some();

        // ADD a return code here!
        // if(ready_ && homed_)
        if(first_message_rx)
        {        
            for(std::size_t i = 0; i < info_.joints.size();++i)
            {
                hw_states_positions_[i] = m_setpoint_jp.position[i];
            }
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type crtkROSHardwareInterface::write(void)
    {        
        executor.spin_some();
        // if(ready_ && homed_)
        if(first_message_rx)
        {
            m_servo_jp.name.resize(0);
            m_servo_jp.position.resize(info_.joints.size());
            m_servo_jp.velocity.resize(0);
            m_servo_jp.effort.resize(0);

            // Shift the name matching in first read and confirmation to on_init
            // std::copy(m_setpoint_jp.name.begin(), m_setpoint_jp.name.end(),m_servo_jp.name.begin()); 
            
            for(std::size_t i = 0; i < info_.joints.size();++i)
            {
                RCLCPP_INFO(rclcpp::get_logger("WriteInterface"), "Got command %.5f for joint %d!",hw_commands_positions_[i],i);
                m_servo_jp.position[i] = hw_commands_positions_[i];
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
        if(measured_js.name.size() == info_.joints.size() )
        {
            m_setpoint_jp.position.resize(info_.joints.size());
            std::copy(measured_js.position.begin(), measured_js.position.end(), m_setpoint_jp.position.begin());
            first_message_rx = true;
        }
    }

    // void crtkROSHardwareInterface::operating_state_callback(const crtk_msgs::msg::OperatingState & current_state)
    // {
    //     std::string state_enable = "ENABLE";
    //     bool call_homing = false;
    //     std::string state(current_state.state.c_str());
    //     RCLCPP_INFO(rclcpp::get_logger("Operating state"),"Current State %s and ready state %d",state,current_state.is_homed);
    //     if(state != state_enable)
    //     {
    //         ready_ = false;
    //         call_homing = true;
    //     }
    //     else
    //     {
    //         ready_ = true;
    //     }
    //     if( current_state.is_homed != homed_)
    //     {
    //         homed_ = current_state.is_homed;
    //         call_homing = true;
    //     }
    //     // if(call_homing)
    //     // {
    //     //     configure_state_command.string = "home";
    //     //     m_state_command_publisher->publish(configure_state_command);
    //     // }
    // }
} // namespace


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(crtk_ros2_hw::crtkROSHardwareInterface, hardware_interface::SystemInterface)
