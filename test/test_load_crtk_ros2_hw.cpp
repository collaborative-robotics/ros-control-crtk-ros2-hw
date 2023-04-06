#include <gmock/gmock.h>

#include <cmath>
#include <string>
#include <unordered_map>

#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "hardware_interface/types/lifecycle_state_names.hpp"
#include "ros2_control_test_assets/components_urdfs.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

class TestGenericSystem : public ::testing::Test
{
protected:
  void SetUp() override
  {
    hardware_system_2dof_ =
      R"(
  <ros2_control name="GenericSystem2dof" type="system">
    <hardware>
      <plugin>crtk_ros2_hw/crtkROSHardwareInterface</plugin>
    </hardware>
    <joint name="joint1">
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
      <param name="initial_position">1.57</param>
    </joint>
    <joint name="joint2">
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
      <param name="initial_position">0.7854</param>
    </joint>
  </ros2_control>
)";
  }
    std::string hardware_system_2dof_;
};

TEST_F(TestGenericSystem, load_generic_system_2dof)
{
  auto urdf = ros2_control_test_assets::urdf_head + hardware_system_2dof_ +
              ros2_control_test_assets::urdf_tail;
  ASSERT_NO_THROW(hardware_interface::ResourceManager rm(urdf));
}

