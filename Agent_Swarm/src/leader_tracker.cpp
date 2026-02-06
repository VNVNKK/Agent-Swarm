// 中文说明：Leader 追踪实现，订阅并缓存 Leader 位姿
#include "leader_tracker.h"

namespace agent_swarm
{

void LeaderTracker::init(ros::NodeHandle &nh, int leader_id)
{
    if (leader_id > 100)
    {
        int ugv_id = leader_id - 100;
        std::string topic = "/ugv" + std::to_string(ugv_id) + "/sunray_ugv/ugv_state";
        sub_ = nh.subscribe<sunray_msgs::UGVState>(topic, 10, &LeaderTracker::ugvCallback, this);
    }
    else
    {
        std::string topic = "/uav" + std::to_string(leader_id) + "/sunray/uav_state";
        sub_ = nh.subscribe<sunray_msgs::UAVState>(topic, 10, &LeaderTracker::uavCallback, this);
    }
}

void LeaderTracker::uavCallback(const sunray_msgs::UAVState::ConstPtr &msg)
{
    leader_pose_.position.x = msg->position[0];
    leader_pose_.position.y = msg->position[1];
    leader_pose_.position.z = msg->position[2];
    leader_pose_.orientation = msg->attitude_q;
    last_stamp_ = msg->header.stamp;
    has_pose_ = true;
}

void LeaderTracker::ugvCallback(const sunray_msgs::UGVState::ConstPtr &msg)
{
    leader_pose_.position.x = msg->position[0];
    leader_pose_.position.y = msg->position[1];
    leader_pose_.position.z = 0.0;
    leader_pose_.orientation = msg->attitude_q;
    last_stamp_ = msg->header.stamp;
    has_pose_ = true;
}

bool LeaderTracker::getLeaderPose(geometry_msgs::Pose &pose_out) const
{
    if (!has_pose_)
    {
        return false;
    }
    pose_out = leader_pose_;
    return true;
}

bool LeaderTracker::isFresh(double timeout_sec) const
{
    if (!has_pose_)
    {
        return false;
    }
    return (ros::Time::now() - last_stamp_).toSec() <= timeout_sec;
}

} // namespace agent_swarm
