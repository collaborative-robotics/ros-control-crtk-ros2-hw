/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

#ifndef _ros_control_crtk_ros_hardware_interface_h
#define _ros_control_crtk_ros_hardware_interface_h

#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include <sensor_msgs/msg/joint_state.hpp>
#include <crtk_msgs/msg/operating_state.hpp>
#include <crtk_msgs/msg/string_stamped.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>

#include <rclcpp/clock.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/rclcpp.hpp>

#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>


namespace crtk_ros2_hw {

    using StateInterface = hardware_interface::StateInterface;
    using CommandInterface = hardware_interface::CommandInterface;
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class crtkROSHardwareInterface: public hardware_interface::SystemInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(crtkROSHardwareInterface);

        crtkROSHardwareInterface(void){};
        ~crtkROSHardwareInterface(){};
        
        CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
        CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
        std::vector<StateInterface> export_state_interfaces() override;
        std::vector<CommandInterface> export_command_interfaces() override;
        hardware_interface::return_type prepare_command_mode_switch(const std::vector<std::string>& start_interfaces, const std::vector<std::string>& stop_interfaces) override;
        hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string>& start_interfaces, const std::vector<std::string>& stop_interfaces) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
        hardware_interface::return_type read(void) override;
        hardware_interface::return_type write(void) override;

    private:
        void measured_js_callback(const sensor_msgs::msg::JointState & measured_js);
        // void operating_state_callback(const crtk_msgs::msg::OperatingState & measured_js);

        // virtual void initialize_from_crtk_node(const sensor_msgs::msg::JointState & measured_js);
        // virtual void copy_measured_js_from_crtk_node(const sensor_msgs::msg::JointState & measured_js);

        bool m_crtk_node_found = false;
        std::shared_ptr<rclcpp::Node> m_node_handle = nullptr;
        rclcpp::executors::SingleThreadedExecutor executor;


        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_setpoint_js_subscriber = nullptr;
        rclcpp::Subscription<crtk_msgs::msg::OperatingState>::SharedPtr m_operating_state_subscriber = nullptr;
        rclcpp::Publisher<crtk_msgs::msg::StringStamped>::SharedPtr m_state_command_publisher = nullptr;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_servo_jp_publisher = nullptr;

        std::size_t m_number_of_joints;

        //flag that first measurement has been recieved
        bool first_message_rx;
        bool homed_;
        bool ready_;
        
        //list of joints to contorl
        std::vector<std::string> controlled_joints;

        //Variables to store commands and states
        std::vector<double> hw_commands_positions_;

        std::vector<double> hw_states_positions_;
        std::vector<double> hw_states_velocities_;

        // Measured states
        sensor_msgs::msg::JointState m_setpoint_jp; // joint state
        sensor_msgs::msg::JointState m_servo_jp;    // commanded servo joint (velocity mode)
        crtk_msgs::msg::StringStamped configure_state_command; // Variable for sending the configure state command.
    };

}

#endif // _ros_control_crtk_ros_hardware_interface_h
