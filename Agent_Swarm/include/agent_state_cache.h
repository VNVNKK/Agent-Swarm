// 中文说明：状态缓存接口，订阅并标准化多机状态
#pragma once

#include <map>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sunray_msgs/UAVState.h>
#include <sunray_msgs/UGVState.h>

namespace agent_swarm
{

// 代理状态缓存
class AgentStateCache
{
  public:
    // 初始化订阅与发布
    void init(ros::NodeHandle &nh, int agent_num, int agent_type, const std::string &agent_name);
    // 获取当前缓存
    const std::map<int, nav_msgs::Odometry> &states() const
    {
        return agent_state_;
    }
    // 发布到 ORCA 的 agent_state
    void publishOrcaStates();

  private:
    // UAV 状态回调
    void uavCallback(const sunray_msgs::UAVState::ConstPtr &msg, int idx);
    // UGV 状态回调
    void ugvCallback(const sunray_msgs::UGVState::ConstPtr &msg, int idx);

    int agent_num_{1};
    int agent_type_{0};
    std::string agent_name_{"uav"};
    std::map<int, nav_msgs::Odometry> agent_state_{};
    std::map<int, ros::Subscriber> subs_{};
    std::map<int, ros::Publisher> pubs_{};
};

} // namespace agent_swarm
