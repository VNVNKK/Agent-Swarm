// 中文说明：控制映射实现，输出控制指令
#include "control_command_mapper.h"
#include <tf/transform_datatypes.h>

namespace agent_swarm
{

void ControlCommandMapper::init(ros::NodeHandle &nh, const std::string &agent_name, int agent_id, int agent_type)
{
    agent_type_ = agent_type;
    std::string prefix = "/" + agent_name + std::to_string(agent_id);
    uav_pub_ = nh.advertise<sunray_msgs::UAVControlCMD>(prefix + "/sunray/uav_control_cmd", 10);
    ugv_pub_ = nh.advertise<sunray_msgs::UGVControlCMD>(prefix + "/sunray_ugv/ugv_control_cmd", 10);
}

void ControlCommandMapper::publishUavHover()
{
    sunray_msgs::UAVControlCMD cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.cmd = sunray_msgs::UAVControlCMD::Hover;
    uav_pub_.publish(cmd);
}

void ControlCommandMapper::publishUgvHold()
{
    sunray_msgs::UGVControlCMD cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.cmd = sunray_msgs::UGVControlCMD::HOLD;
    ugv_pub_.publish(cmd);
}

void ControlCommandMapper::publishHover()
{
    if (agent_type_ == 0)
    {
        publishUavHover();
    }
    else
    {
        publishUgvHold();
    }
}

void ControlCommandMapper::publishTakeoff()
{
    if (agent_type_ != 0)
    {
        publishUgvHold();
        return;
    }
    sunray_msgs::UAVControlCMD cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.cmd = sunray_msgs::UAVControlCMD::Takeoff;
    uav_pub_.publish(cmd);
}

void ControlCommandMapper::publishLand()
{
    if (agent_type_ != 0)
    {
        publishUgvHold();
        return;
    }
    sunray_msgs::UAVControlCMD cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.cmd = sunray_msgs::UAVControlCMD::Land;
    uav_pub_.publish(cmd);
}

void ControlCommandMapper::publishReturnHome()
{
    if (agent_type_ != 0)
    {
        publishUgvHold();
        return;
    }
    sunray_msgs::UAVControlCMD cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.cmd = sunray_msgs::UAVControlCMD::Return;
    uav_pub_.publish(cmd);
}

void ControlCommandMapper::publishPosTarget(const geometry_msgs::Pose &target_pose)
{
    if (agent_type_ == 0)
    {
        sunray_msgs::UAVControlCMD cmd;
        cmd.header.stamp = ros::Time::now();
        cmd.cmd = sunray_msgs::UAVControlCMD::XyzPosYaw;
        cmd.desired_pos[0] = target_pose.position.x;
        cmd.desired_pos[1] = target_pose.position.y;
        cmd.desired_pos[2] = target_pose.position.z;
        cmd.desired_yaw = tf::getYaw(target_pose.orientation);
        uav_pub_.publish(cmd);
    }
    else
    {
        sunray_msgs::UGVControlCMD cmd;
        cmd.header.stamp = ros::Time::now();
        cmd.cmd = sunray_msgs::UGVControlCMD::POS_CONTROL_ENU;
        cmd.desired_pos[0] = target_pose.position.x;
        cmd.desired_pos[1] = target_pose.position.y;
        cmd.desired_yaw = tf::getYaw(target_pose.orientation);
        ugv_pub_.publish(cmd);
    }
}

void ControlCommandMapper::publishFromOrca(const sunray_msgs::OrcaCmd &orca)
{
    if (agent_type_ == 0)
    {
        sunray_msgs::UAVControlCMD cmd;
        cmd.header.stamp = ros::Time::now();
        if (orca.state == sunray_msgs::OrcaCmd::STOP)
        {
            cmd.cmd = sunray_msgs::UAVControlCMD::Hover;
        }
        else if (orca.state == sunray_msgs::OrcaCmd::RUN)
        {
            cmd.cmd = sunray_msgs::UAVControlCMD::XyVelZPosYaw;
            cmd.desired_vel[0] = orca.linear[0];
            cmd.desired_vel[1] = orca.linear[1];
            cmd.desired_pos[0] = orca.goal_pos[0];
            cmd.desired_pos[1] = orca.goal_pos[1];
            cmd.desired_pos[2] = orca.goal_pos[2];
            cmd.desired_yaw = orca.goal_yaw;
        }
        else if (orca.state == sunray_msgs::OrcaCmd::ARRIVED)
        {
            cmd.cmd = sunray_msgs::UAVControlCMD::XyzPosYaw;
            cmd.desired_pos[0] = orca.goal_pos[0];
            cmd.desired_pos[1] = orca.goal_pos[1];
            cmd.desired_pos[2] = orca.goal_pos[2];
            cmd.desired_yaw = orca.goal_yaw;
        }
        else
        {
            cmd.cmd = sunray_msgs::UAVControlCMD::Hover;
        }
        uav_pub_.publish(cmd);
    }
    else
    {
        sunray_msgs::UGVControlCMD cmd;
        cmd.header.stamp = ros::Time::now();
        if (orca.state == sunray_msgs::OrcaCmd::STOP)
        {
            cmd.cmd = sunray_msgs::UGVControlCMD::HOLD;
        }
        else if (orca.state == sunray_msgs::OrcaCmd::RUN)
        {
            cmd.cmd = sunray_msgs::UGVControlCMD::VEL_CONTROL_ENU;
            cmd.desired_vel[0] = orca.linear[0];
            cmd.desired_vel[1] = orca.linear[1];
            cmd.desired_pos[0] = orca.goal_pos[0];
            cmd.desired_pos[1] = orca.goal_pos[1];
            cmd.desired_yaw = orca.goal_yaw;
        }
        else if (orca.state == sunray_msgs::OrcaCmd::ARRIVED)
        {
            cmd.cmd = sunray_msgs::UGVControlCMD::POS_CONTROL_ENU;
            cmd.desired_pos[0] = orca.goal_pos[0];
            cmd.desired_pos[1] = orca.goal_pos[1];
            cmd.desired_yaw = orca.goal_yaw;
        }
        else
        {
            cmd.cmd = sunray_msgs::UGVControlCMD::HOLD;
        }
        ugv_pub_.publish(cmd);
    }
}

} // namespace agent_swarm
