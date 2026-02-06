// 中文说明：AgentSwarm 节点实现，驱动编队计算与控制输出
#include "agent_swarm_node.h"

namespace agent_swarm
{

AgentSwarmNode::AgentSwarmNode(ros::NodeHandle &nh) : nh_(nh)
{
    // 读取参数
    nh_.param<int>("agent_id", agent_id_, 1);
    nh_.param<int>("agent_num", agent_num_, 1);
    nh_.param<int>("agent_type", agent_type_, 0);
    nh_.param<int>("leader_id", leader_id_, 1);
    nh_.param<std::string>("agent_name", agent_name_, std::string("uav"));
    nh_.param<std::string>("formation_policy", policy_name_, std::string("ring"));
    nh_.param<double>("spacing", spacing_, 1.0);
    nh_.param<bool>("use_fixed_altitude", use_fixed_altitude_, true);
    nh_.param<double>("fixed_altitude", fixed_altitude_, 1.0);
    nh_.param<double>("spacing_min", spacing_min_, 0.3);
    nh_.param<double>("spacing_max", spacing_max_, 5.0);
    nh_.param<double>("spacing_scale_up", spacing_scale_up_, 1.2);
    nh_.param<double>("spacing_scale_down", spacing_scale_down_, 0.8);
    nh_.param<double>("leader_timeout", leader_timeout_, 1.0);
    nh_.param<double>("orca_timeout", orca_timeout_, 1.0);
    nh_.param<bool>("leader_publish_goal", leader_publish_goal_, true);

    // 初始化策略
    custom_policy_ = std::make_shared<CustomPolicy>();
    if (policy_name_ == "custom")
    {
        policy_ = custom_policy_;
    }
    else
    {
        policy_ = policy_factory_.create(policy_name_);
    }

    // 初始化各模块
    leader_tracker_.init(nh_, leader_id_);
    state_cache_.init(nh_, agent_num_, agent_type_, agent_name_);
    goal_dispatcher_.init(nh_, agent_name_, agent_id_);
    orca_client_.init(nh_, agent_name_, agent_id_);
    control_mapper_.init(nh_, agent_name_, agent_id_, agent_type_);

    // 订阅自身 UAV 状态与发布 UAV setup（仅 UAV 有效）
    if (agent_type_ == 0)
    {
        std::string self_prefix = "/" + agent_name_ + std::to_string(agent_id_);
        uav_state_sub_ = nh_.subscribe<sunray_msgs::UAVState>(self_prefix + "/sunray/uav_state", 10,
                                                              &AgentSwarmNode::uavStateCb, this);
        uav_setup_pub_ = nh_.advertise<sunray_msgs::UAVSetup>(self_prefix + "/sunray/setup", 10);
    }

    // 订阅外部编队指令
    formation_cmd_sub_ =
        nh_.subscribe<sunray_msgs::Formation>("/sunray/formation_cmd", 10, &AgentSwarmNode::onFormationCmd, this);
    formation_cmd_ground_sub_ = nh_.subscribe<sunray_msgs::Formation>("/sunray/formation_cmd/ground", 10,
                                                                      &AgentSwarmNode::onFormationCmd, this);
    formation_offsets_sub_ = nh_.subscribe<sunray_msgs::FormationOffsets>("/sunray/formation_offsets", 10,
                                                                          &AgentSwarmNode::formationOffsetsCb, this);
    // 订阅 Leader 目标点
    leader_goal_sub_ =
        nh_.subscribe<geometry_msgs::PoseStamped>("/sunray/leader_goal", 10, &AgentSwarmNode::leaderGoalCb, this);

    // 定时器频率
    double goal_rate = 20.0;
    double control_rate = 20.0;
    double state_pub_rate = 20.0;
    nh_.param<double>("goal_rate", goal_rate, 20.0);
    nh_.param<double>("control_rate", control_rate, 20.0);
    nh_.param<double>("state_pub_rate", state_pub_rate, 20.0);

    // 定时回调
    goal_timer_ = nh_.createTimer(ros::Duration(1.0 / goal_rate), &AgentSwarmNode::goalTimerCb, this);
    control_timer_ = nh_.createTimer(ros::Duration(1.0 / control_rate), &AgentSwarmNode::controlTimerCb, this);
    state_pub_timer_ = nh_.createTimer(ros::Duration(1.0 / state_pub_rate), &AgentSwarmNode::statePublishTimerCb, this);
}

void AgentSwarmNode::onFormationCmd(const sunray_msgs::Formation::ConstPtr &msg)
{
    // 更新状态机
    state_machine_.onFormationCmd(*msg);
    // 收到起飞命令时记录起飞起始时间
    if (msg->cmd == sunray_msgs::Formation::TAKEOFF)
    {
        takeoff_active_ = true;
        takeoff_start_time_ = ros::Time::now();
    }
    // 编队切换触发“到位后悬停”
    if (msg->cmd == sunray_msgs::Formation::FORMATION)
    {
        formation_change_active_ = true;
    }
    // 使用 name 作为策略名称
    if (msg->cmd == sunray_msgs::Formation::FORMATION && !msg->name.empty())
    {
        // 动作指令：扩散/聚拢
        if (msg->name == "expand")
        {
            spacing_ = std::min(spacing_ * spacing_scale_up_, spacing_max_);
        }
        else if (msg->name == "contract")
        {
            spacing_ = std::max(spacing_ * spacing_scale_down_, spacing_min_);
        }
        else
        {
            policy_name_ = msg->name;
        }
        if (policy_name_ == "custom")
        {
            policy_ = custom_policy_;
        }
        else
        {
            policy_ = policy_factory_.create(policy_name_);
        }
    }
}

void AgentSwarmNode::goalTimerCb(const ros::TimerEvent &)
{
    bool leader_ok = leader_tracker_.isFresh(leader_timeout_);
    bool orca_ok = orca_client_.isFresh(orca_timeout_);
    SwarmState state = state_machine_.effectiveState(leader_ok, orca_ok);
    if (state != SwarmState::FORMATION)
    {
        return;
    }

    geometry_msgs::Pose leader_pose;
    if (!leader_tracker_.getLeaderPose(leader_pose))
    {
        return;
    }
    geometry_msgs::Pose ref_pose = leader_pose;
    if (leader_goal_active_)
    {
        ref_pose = leader_goal_;
    }

    FormationContext ctx;
    ctx.agent_id = agent_id_;
    ctx.agent_num = agent_num_;
    ctx.leader_id = leader_id_;
    ctx.spacing = spacing_;

    geometry_msgs::Pose target;
    if (policy_ && policy_->computeTarget(ref_pose, ctx, target))
    {
        if (use_fixed_altitude_)
        {
            target.position.z = fixed_altitude_;
        }
        last_goal_ = target;
        last_goal_valid_ = true;
        goal_dispatcher_.publishGoal(target, true);
    }
    else if (leader_publish_goal_)
    {
        int leader_index = (leader_id_ > 100) ? (leader_id_ - 100) : leader_id_;
        if (ctx.agent_id == leader_index)
        {
            if (leader_goal_active_)
            {
                if (use_fixed_altitude_)
                {
                    leader_goal_.position.z = fixed_altitude_;
                }
                last_goal_ = leader_goal_;
                last_goal_valid_ = true;
                goal_dispatcher_.publishGoal(leader_goal_, true);
            }
            else
            {
                if (use_fixed_altitude_)
                {
                    leader_pose.position.z = fixed_altitude_;
                }
                last_goal_ = leader_pose;
                last_goal_valid_ = true;
                goal_dispatcher_.publishGoal(leader_pose, true);
            }
        }
    }
}

void AgentSwarmNode::controlTimerCb(const ros::TimerEvent &)
{
    bool leader_ok = leader_tracker_.isFresh(leader_timeout_);
    bool orca_ok = orca_client_.isFresh(orca_timeout_);
    SwarmState state = state_machine_.effectiveState(leader_ok, orca_ok);

    if (state == SwarmState::HOVER)
    {
        control_mapper_.publishHover();
        return;
    }
    if (state == SwarmState::TAKEOFF)
    {
        handleTakeoff();
        return;
    }
    if (state == SwarmState::LAND)
    {
        control_mapper_.publishLand();
        return;
    }
    if (state == SwarmState::RETURN_HOME)
    {
        control_mapper_.publishReturnHome();
        return;
    }

    sunray_msgs::OrcaCmd orca_cmd;
    if (orca_client_.getLastCmd(orca_cmd))
    {
        // 编队或移动到位后悬停
        if (orca_cmd.state == sunray_msgs::OrcaCmd::ARRIVED && (formation_change_active_ || leader_goal_active_) &&
            last_goal_valid_)
        {
            double dx = orca_cmd.goal_pos[0] - last_goal_.position.x;
            double dy = orca_cmd.goal_pos[1] - last_goal_.position.y;
            double dz = orca_cmd.goal_pos[2] - last_goal_.position.z;
            double dist2 = dx * dx + dy * dy + dz * dz;
            if (dist2 < 0.04)
            {
                formation_change_active_ = false;
                leader_goal_active_ = false;
                state_machine_.setRequestedState(SwarmState::HOVER);
                control_mapper_.publishHover();
                return;
            }
        }
        control_mapper_.publishFromOrca(orca_cmd);
    }
    else
    {
        control_mapper_.publishHover();
    }
}

void AgentSwarmNode::statePublishTimerCb(const ros::TimerEvent &)
{
    // 发布到 ORCA 的 agent_state
    state_cache_.publishOrcaStates();
}

void AgentSwarmNode::uavStateCb(const sunray_msgs::UAVState::ConstPtr &msg)
{
    uav_state_ = *msg;
    uav_state_ready_ = true;
}

void AgentSwarmNode::leaderGoalCb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    leader_goal_ = msg->pose;
    leader_goal_active_ = true;
    // 触发移动编队
    state_machine_.setRequestedState(SwarmState::FORMATION);
}

void AgentSwarmNode::formationOffsetsCb(const sunray_msgs::FormationOffsets::ConstPtr &msg)
{
    if (!custom_policy_)
    {
        return;
    }
    std::vector<Offset2D> offsets;
    offsets.reserve(msg->offsets.size());
    for (const auto &pt : msg->offsets)
    {
        offsets.push_back({pt.x, pt.y});
    }
    custom_policy_->setOffsets(offsets);
}

void AgentSwarmNode::handleTakeoff()
{
    if (agent_type_ != 0)
    {
        control_mapper_.publishHover();
        return;
    }
    if (!takeoff_active_)
    {
        takeoff_active_ = true;
        takeoff_start_time_ = ros::Time::now();
    }
    double time_diff = (ros::Time::now() - takeoff_start_time_).toSec();
    // 1) 切换到 CMD_CONTROL
    if (time_diff < 5.0)
    {
        if (!uav_state_ready_ || uav_state_.control_mode != sunray_msgs::UAVSetup::CMD_CONTROL)
        {
            sunray_msgs::UAVSetup setup;
            setup.header.stamp = ros::Time::now();
            setup.cmd = sunray_msgs::UAVSetup::SET_CONTROL_MODE;
            setup.control_mode = "CMD_CONTROL";
            uav_setup_pub_.publish(setup);
        }
        return;
    }
    // 2) 解锁
    if (time_diff >= 5.0 && time_diff < 10.0)
    {
        if (uav_state_ready_ && uav_state_.control_mode == sunray_msgs::UAVSetup::CMD_CONTROL && !uav_state_.armed)
        {
            sunray_msgs::UAVSetup setup;
            setup.header.stamp = ros::Time::now();
            setup.cmd = sunray_msgs::UAVSetup::ARM;
            uav_setup_pub_.publish(setup);
        }
        return;
    }
    // 3) 起飞指令
    if (time_diff >= 10.0 && time_diff < 15.0)
    {
        if (uav_state_ready_ && uav_state_.armed)
        {
            control_mapper_.publishTakeoff();
        }
        return;
    }
    // 4) 起飞流程结束，切回编队
    if (time_diff >= 15.0 && uav_state_ready_ && uav_state_.armed)
    {
        takeoff_active_ = false;
        state_machine_.setRequestedState(SwarmState::FORMATION);
    }
}

} // namespace agent_swarm

int main(int argc, char **argv)
{
    ros::init(argc, argv, "agent_swarm_node");
    ros::NodeHandle nh("~");

    agent_swarm::AgentSwarmNode node(nh);
    ros::spin();
    return 0;
}
