// 中文说明：状态缓存实现，发布 /orca/agent_state
#include "agent_state_cache.h"
#include <boost/bind.hpp>

namespace agent_swarm
{

void AgentStateCache::init(ros::NodeHandle &nh, int agent_num, int agent_type, const std::string &agent_name,
                           int leader_id)
{
    agent_num_ = agent_num;
    agent_type_ = agent_type;
    agent_name_ = agent_name;
    leader_id_ = leader_id;

    int leader_index = (leader_id > 100) ? (leader_id - 100) : leader_id;
    bool leader_is_ugv = (leader_id > 100);

    for (int i = 0; i < agent_num_; ++i)
    {
        int id = i + 1;
        // ORCA agent_state 始终使用 agent_name 前缀
        std::string orca_prefix = "/" + agent_name_ + std::to_string(id);
        pubs_[id] = nh.advertise<nav_msgs::Odometry>(orca_prefix + "/orca/agent_state", 10);

        if (leader_is_ugv && id == leader_index)
        {
            // UGV Leader：硬编码 /ugv 前缀（与 LeaderTracker 一致）
            std::string ugv_prefix = "/ugv" + std::to_string(id);
            std::string topic = ugv_prefix + "/sunray_ugv/ugv_state";
            subs_[id] = nh.subscribe<sunray_msgs::UGVState>(
                topic, 10, boost::bind(&AgentStateCache::ugvCallback, this, _1, id));
        }
        else
        {
            std::string prefix = "/" + agent_name_ + std::to_string(id);
            if (agent_type_ == 0)
            {
                std::string topic = prefix + "/sunray/uav_state";
                subs_[id] = nh.subscribe<sunray_msgs::UAVState>(
                    topic, 10, boost::bind(&AgentStateCache::uavCallback, this, _1, id));
            }
            else
            {
                std::string topic = prefix + "/sunray_ugv/ugv_state";
                subs_[id] = nh.subscribe<sunray_msgs::UGVState>(
                    topic, 10, boost::bind(&AgentStateCache::ugvCallback, this, _1, id));
            }
        }
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
