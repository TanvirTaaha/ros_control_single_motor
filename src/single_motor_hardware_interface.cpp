#include <ros_control_single_motor/motor_hw.h>

MotorHardwareInterface::MotorHardwareInterface(ros::NodeHandle &nh) : _nh(nh)
{
    init();
    _controller_manager.reset(new controller_manager::ControllerManager(this, _nh));

    _loop_hz = 5;
    ros::Duration update_period = ros::Duration(1.0 / _loop_hz);

    pub = _nh.advertise<std_msgs::Int32>("/ros_control_to_arduino", 10);
    client = _nh.serviceClient<ros_control_single_motor::Float_Floats>("/read_joint_state");

    _non_realtime_loop = _nh.createTimer(update_period, &MotorHardwareInterface::update, this);
}

MotorHardwareInterface::~MotorHardwareInterface()
{
}

void MotorHardwareInterface::init()
{
    _joint_name = "joint1";

    hardware_interface::JointStateHandle jnt_state_handle(_joint_name, &_joint_position, &_joint_velocity, &_joint_effort);
    _jnt_state_interface.registerHandle(jnt_state_handle);

    hardware_interface::JointHandle joint_position_handle(jnt_state_handle, &_joint_position_command);
    _position_jnt_interface.registerHandle(joint_position_handle);

    // hardware_interface::JointHandle joint_velocity_handle(jnt_state_handle, &_joint_velocity_command);
    // _effort_jnt_interface.registerHandle(joint_velocity_handle);

    // hardware_interface::JointHandle joint_effort_handle(jnt_state_handle, &_joint_effort_command);
    // _effort_jnt_interface.registerHandle(joint_effort_handle);

    joint_limits_interface::JointLimits limits;
    joint_limits_interface::getJointLimits("joint1", _nh, limits);
    joint_limits_interface::PositionJointSaturationHandle position_limits_handle(joint_position_handle, limits);
    _position_joint_saturation_interface.registerHandle(position_limits_handle);
    /*
    If you have more joints then,

        joint_name_= "joint2"

    // Create joint state interface
        hardware_interface::JointStateHandle jointStateHandle2(joint_name_, &joint_position_2, &joint_velocity_2, &joint_effort_2);
        joint_state_interface_.registerHandle(jointStateHandle);

    //create the position/velocity/effort interface according to your actuator
        hardware_interface::JointHandle jointPositionHandle2(jointStateHandle2, &joint_position_command_2);
        position_joint_interface_.registerHandle(jointPositionHandle2);

        hardware_interface::JointHandle jointVelocityHandle2(jointStateHandle2, &joint_velocity_command_2);
        effort_joint_interface_.registerHandle(jointVelocityHandle2);

        hardware_interface::JointHandle jointEffortHandle2(jointStateHandle2, &joint_effort_command_2);
        effort_joint_interface_.registerHandle(jointEffortHandle2);

    //create joint limit interface.
        joint_limits_interface::getJointLimits("joint2", nh_, limits);
        joint_limits_interface::EffortJointSaturationHandle jointLimitsHandle2(jointEffortHandle2, limits);
        effortJointSaturationInterface.registerHandle(jointLimitsHandle2);

        Repeat same for other joints
    */

    registerInterface(&_jnt_state_interface);
    registerInterface(&_position_jnt_interface);
    registerInterface(&_effort_jnt_interface);
    registerInterface(&_position_joint_saturation_interface);
}

void MotorHardwareInterface::update(const ros::TimerEvent &e)
{
    _elapsed_time = ros::Duration(e.current_real - e.last_real);
    read();
    _controller_manager->update(ros::Time::now(), _elapsed_time);
    write(_elapsed_time);
}

void MotorHardwareInterface::read()
{
    joint_read.request.req = 1.0;
    if (client.call(joint_read))
    {
        _joint_position = angles::from_degrees(joint_read.response.res[0]);
        _joint_velocity = angles::from_degrees(joint_read.response.res[1]);
        ROS_DEBUG("Current pos:%.2lf, vel:%.2lf", _joint_position, _joint_velocity);
    }
    else
    {
        _joint_position = 0.0;
        _joint_velocity = 0.0;
    }
}

void MotorHardwareInterface::write(ros::Duration elapsed_time)
{
    _position_joint_saturation_interface.enforceLimits(elapsed_time);
    joint_pub.data = angles::to_degrees(_joint_position_command);
    
    ROS_DEBUG("Commanded position: rad: %0.2lf, deg:%d", _joint_position_command, joint_pub.data);
    pub.publish(joint_pub);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "single_motor_hardware_interface");
    ros::NodeHandle nh;
    ros::MultiThreadedSpinner spinner(2); // Multiple threads for controller service callback and for the Service client callback used to get the feedback from ardiuno
    MotorHardwareInterface robot(nh);
    spinner.spin();
    
    return 0;
}