#pragma once

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>

#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>
#include <angles/angles.h>
#include <std_msgs/Int32.h>
#include <ros_control_single_motor/Float_Floats.h>

class MotorHardwareInterface : public hardware_interface::RobotHW
{
    public:
        MotorHardwareInterface(ros::NodeHandle& nh);
        ~MotorHardwareInterface();
        void init();
        void update(const ros::TimerEvent& e);
        void read();
        void write(ros::Duration elapsed_time);
        ros::Publisher pub;
        ros::ServiceClient client;
        std_msgs::Int32 joint_pub;
        ros_control_single_motor::Float_Floats joint_read;

    protected:
        hardware_interface::JointStateInterface _jnt_state_interface;
        hardware_interface::PositionJointInterface _position_jnt_interface;
        hardware_interface::EffortJointInterface _effort_jnt_interface;

        joint_limits_interface::PositionJointSaturationInterface _position_joint_saturation_interface;

        int _num_joints;
        std::string _joint_name;
        double _joint_position;
        double _joint_velocity;
        double _joint_effort;

        double _joint_position_command;
        double _joint_effort_command;
        double _joint_velocity_command;
        
        ros::NodeHandle _nh;
        ros::Timer _non_realtime_loop;
        ros::Duration _elapsed_time;
        double _loop_hz;
        boost::shared_ptr<controller_manager::ControllerManager> _controller_manager;
};