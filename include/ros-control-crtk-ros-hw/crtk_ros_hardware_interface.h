/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

#ifndef _ros_control_crtk_ros_hardware_interface_h
#define _ros_control_crtk_ros_hardware_interface_h

#include <memory>
#include <string>
#include <vector>

#include <controller_manager/controller_manager.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <hardware_interface/actuator_interface.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>


namespace ros_control_crtk {

    using StateInterface = hardware_interface::StateInterface;
    using CommandInterface = hardware_interface::CommandInterface;

    class crtkROSHardwareInterface: public hardware_interface::ActuatorInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(crtkROSHardwareInterface);

        CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
        //hardware_interface::return_type on_activate(const rclcpp_lifecycle::State & previous_state) override;
        //hardware_interface::return_type on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

        hardware_interface::return_type prepare_command_mode_switch(const std::vector<std::string>& start_interfaces, const std::vector<std::string>& stop_interfaces) override;
        hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string>& start_interfaces, const std::vector<std::string>& stop_interfaces) override;
        hardware_interface::return_type read(void) override;
        hardware_interface::return_type write(void) override;

        std::vector<StateInterface> export_state_interfaces() override;
        std::vector<CommandInterface> export_command_interfaces() override;

    protected:
        void init(void);
        void measured_js_callback(const sensor_msgs::msg::JointState & measured_js);
        virtual void initialize_from_crtk_node(const sensor_msgs::msg::JointState & measured_js);
        virtual void copy_measured_js_from_crtk_node(const sensor_msgs::msg::JointState & measured_js);

        bool m_crtk_node_found = false;
        std::shared_ptr<rclcpp::Node> m_node_handle = nullptr;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_measured_js_subscriber = nullptr;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_servo_jp_publisher = nullptr;

        std::size_t m_number_of_joints;
        sensor_msgs::msg::JointState m_measured_js; // joint state
        sensor_msgs::msg::JointState m_servo_jp;    // commanded/servo joint position
        sensor_msgs::msg::JointState m_servo_jv;
    };

}

#endif // _ros_control_crtk_ros_hardware_interface_h
