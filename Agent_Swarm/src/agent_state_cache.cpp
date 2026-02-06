// 中文说明：状态缓存实现，发布 /orca/agent_state
#include "agent_state_cache.h"
#include <boost/bind.hpp>

namespace agent_swarm
{

void AgentStateCache::init(ros::NodeHandle &nh, int agent_num, int agent_type, const std::string &agent_name)
{
    agent_num_ = agent_num;
    agent_type_ = agent_type;
    agent_name_ = agent_name;

    for (int i = 0; i < agent_num_; ++i)
    {
        int id = i + 1;
        std::string prefix = "/" + agent_name_ + std::to_string(id);
        if (agent_type_ == 0)
        {
            std::string topic = prefix + "/sunray/uav_state";
            subs_[id] = nh.subscribe<sunray_msgs::UAVState>(topic, 10,
                                                            boost::bind(&AgentStateCache::uavCallback, this, _1, id));
        }
        else
        {
            std::string topic = prefix + "/sunray_ugv/ugv_state";
            subs_[id] = nh.subscribe<sunray_msgs::UGVState>(topic, 10,
                                                            boost::bind(&AgentStateCache::ugvCallback, this, _1, id));
        }
        pubs_[id] = nh.advertise<nav_msgs::Odometry>(prefix + "/orca/agent_state", 10);
    }
}

void AgentStateCache::uavCallback(const sunray_msgs::UAVState::ConstPtr &msg, int idx)
{
    nav_msgs::Odometry odom;
    odom.header = msg->header;
    odom.pose.pose.position.x = msg->position[0];
    odom.pose.pose.position.y = msg->position[1];
    odom.pose.pose.position.z = msg->position[2];
    odom.pose.pose.orientation = msg->attitude_q;
    odom.twist.twist.linear.x = msg->velocity[0];
    odom.twist.twist.linear.y = msg->velocity[1];
    odom.twist.twist.linear.z = msg->velocity[2];
    agent_state_[idx] = odom;
}

void AgentStateCache::ugvCallback(const sunray_msgs::UGVState::ConstPtr &msg, int idx)
{
    nav_msgs::Odometry odom;
    odom.header = msg->header;
    odom.pose.pose.position.x = msg->position[0];
    odom.pose.pose.position.y = msg->position[1];
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = msg->attitude_q;
    odom.twist.twist.linear.x = msg->velocity[0];
    odom.twist.twist.linear.y = msg->velocity[1];
    agent_state_[idx] = odom;
}

void AgentStateCache::publishOrcaStates()
{
    for (const auto &kv : agent_state_)
    {
        int id = kv.first;
        auto it = pubs_.find(id);
        if (it != pubs_.end())
        {
            it->second.publish(kv.second);
        }
    }
}

} // namespace agent_swarm
