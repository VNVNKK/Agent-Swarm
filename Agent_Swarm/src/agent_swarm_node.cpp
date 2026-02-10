// 中文说明：AgentSwarm 节点实现，驱动编队计算与控制输出
#include "agent_swarm_node.h"
#include <tf/transform_datatypes.h>

namespace agent_swarm
{
namespace
{

double poseDist2(const geometry_msgs::Pose &a, const geometry_msgs::Pose &b)
{
    double dx = a.position.x - b.position.x;
    double dy = a.position.y - b.position.y;
    double dz = a.position.z - b.position.z;
    return dx * dx + dy * dy + dz * dz;
}

const char *toStateString(SwarmState state)
{
    switch (state)
    {
    case SwarmState::INIT:
        return "INIT";
    case SwarmState::TAKEOFF:
        return "TAKEOFF";
    case SwarmState::LAND:
        return "LAND";
    case SwarmState::HOVER:
        return "HOVER";
    case SwarmState::FORMATION:
        return "FORMATION";
    case SwarmState::ORCA_RETURN_HOME:
        return "ORCA_RETURN_HOME";
    default:
        return "UNKNOWN";
    }
}

const char *toCmdString(uint8_t cmd)
{
    switch (cmd)
    {
    case sunray_msgs::Formation::FORMATION:
        return "FORMATION";
    case sunray_msgs::Formation::HOVER:
        return "HOVER";
    case sunray_msgs::Formation::TAKEOFF:
        return "TAKEOFF";
    case sunray_msgs::Formation::LAND:
        return "LAND";
    case sunray_msgs::Formation::RETURN_HOME:
        return "RETURN_HOME";
    case sunray_msgs::Formation::SET_HOME:
        return "SET_HOME";
    default:
        return "UNKNOWN";
    }
}

} // namespace

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
    nh_.param<double>("goal_z_tolerance", goal_z_tolerance_, 0.2);
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
    state_cache_.init(nh_, agent_num_, agent_type_, agent_name_, leader_id_);
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
    ROS_INFO("Formation cmd: %s name=%s", toCmdString(msg->cmd), msg->name.c_str());
    // 更新状态机
    state_machine_.onFormationCmd(*msg);
    auto set_home_from_state = [this]() -> bool {
        if (agent_type_ != 0)
        {
            return false;
        }
        if (uav_state_ready_)
        {
            home_pose_.position.x = uav_state_.position[0];
            home_pose_.position.y = uav_state_.position[1];
            home_pose_.position.z = uav_state_.position[2];
            home_pose_.orientation = tf::createQuaternionMsgFromYaw(uav_state_.attitude[2]);
            home_set_ = true;
            return true;
        }
        if (last_goal_valid_)
        {
            home_pose_ = last_goal_;
            home_set_ = true;
            return true;
        }
        return false;
    };
    auto set_home_from_uav_home = [this]() -> bool {
        if (!uav_state_ready_ || agent_type_ != 0)
        {
            return false;
        }
        home_pose_.position.x = uav_state_.home_pos[0];
        home_pose_.position.y = uav_state_.home_pos[1];
        home_pose_.position.z = uav_state_.home_pos[2];
        home_pose_.orientation = tf::createQuaternionMsgFromYaw(uav_state_.home_yaw);
        home_set_ = true;
        return true;
    };
    // 收到起飞命令时记录起飞起始时间
    if (msg->cmd == sunray_msgs::Formation::TAKEOFF)
    {
        takeoff_active_ = true;
        takeoff_start_time_ = ros::Time::now();
    }
    // 编队切换触发“到位后悬停”
    if (msg->cmd == sunray_msgs::Formation::FORMATION)
    {
        formation_change_active_ = true;        // 失效旧目标，防止 controlTimerCb 用旧 ARRIVED 提前切 HOVER
        last_goal_valid_ = false;    }
    if (msg->cmd == sunray_msgs::Formation::SET_HOME)
    {
        if (!set_home_from_state())
        {
            ROS_WARN("SET_HOME ignored: no UAV state yet.");
        }
        else
        {
            ROS_INFO("Home set: [%.2f, %.2f, %.2f]", home_pose_.position.x, home_pose_.position.y,
                     home_pose_.position.z);
        }
        return;
    }
    if (msg->cmd == sunray_msgs::Formation::RETURN_HOME)
    {
        if (!home_set_ && !set_home_from_uav_home())
        {
            ROS_WARN("RETURN_HOME ignored: home not set.");
            state_machine_.setRequestedState(SwarmState::HOVER);
            return;
        }
        return_home_active_ = true;
        last_goal_valid_ = false;
        ROS_INFO("Return home start: [%.2f, %.2f, %.2f]", home_pose_.position.x, home_pose_.position.y,
                 home_pose_.position.z);
    }
    // 使用 name 作为策略名称
    if (msg->cmd == sunray_msgs::Formation::FORMATION && !msg->name.empty())
    {
        // 动作指令：扩散/聚拢（仅调整 spacing，不重建策略）
        if (msg->name == "expand")
        {
            spacing_ = std::min(spacing_ * spacing_scale_up_, spacing_max_);
            ROS_INFO("Spacing expand -> %.2f", spacing_);
        }
        else if (msg->name == "contract")
        {
            spacing_ = std::max(spacing_ * spacing_scale_down_, spacing_min_);
            ROS_INFO("Spacing contract -> %.2f", spacing_);
        }
        else
        {
            // 真正的阵型切换，重建策略
            policy_name_ = msg->name;
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
}

void AgentSwarmNode::goalTimerCb(const ros::TimerEvent &)
{
    bool leader_ok = leader_tracker_.isFresh(leader_timeout_);
    bool orca_ok = orca_client_.isFresh(orca_timeout_);
    SwarmState state = state_machine_.effectiveState(leader_ok, orca_ok);
    // 状态变化日志仅在 controlTimerCb 中打印，避免重复
    if (state != SwarmState::FORMATION && state != SwarmState::ORCA_RETURN_HOME)
    {
        return;
    }
    if (state == SwarmState::ORCA_RETURN_HOME)
    {
        if (!home_set_)
        {
            return;
        }
        geometry_msgs::Pose target = home_pose_;
        if (use_fixed_altitude_)
        {
            target.position.z = fixed_altitude_;
        }
        last_goal_ = target;
        last_goal_valid_ = true;
        goal_dispatcher_.publishGoal(target, true);
        ROS_DEBUG_THROTTLE(2.0, "ORCA_RETURN_HOME goal: [%.2f, %.2f, %.2f]", target.position.x, target.position.y,
                           target.position.z);
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
        ROS_DEBUG_THROTTLE(2.0, "Formation goal: [%.2f, %.2f, %.2f] (spacing=%.2f)", target.position.x,
                           target.position.y, target.position.z, spacing_);
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
                ROS_DEBUG_THROTTLE(2.0, "Leader goal passthrough: [%.2f, %.2f, %.2f]", leader_goal_.position.x,
                                   leader_goal_.position.y, leader_goal_.position.z);
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
                ROS_DEBUG_THROTTLE(2.0, "Leader pose passthrough: [%.2f, %.2f, %.2f]", leader_pose.position.x,
                                   leader_pose.position.y, leader_pose.position.z);
            }
        }
    }
}

void AgentSwarmNode::controlTimerCb(const ros::TimerEvent &)
{
    bool leader_ok = leader_tracker_.isFresh(leader_timeout_);
    bool orca_ok = orca_client_.isFresh(orca_timeout_);
    SwarmState state = state_machine_.effectiveState(leader_ok, orca_ok);
    if (!last_effective_state_valid_ || state != last_effective_state_)
    {
        ROS_INFO("Swarm state -> %s (leader_ok=%d orca_ok=%d)", toStateString(state), leader_ok ? 1 : 0,
                 orca_ok ? 1 : 0);
        last_effective_state_ = state;
        last_effective_state_valid_ = true;
    }

    // INIT 状态：无人机在地面，不发任何控制指令，等待 TAKEOFF
    if (state == SwarmState::INIT)
    {
        return;
    }
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
    sunray_msgs::OrcaCmd orca_cmd;
    if (orca_client_.getLastCmd(orca_cmd))
    {
        ROS_DEBUG_THROTTLE(2.0, "ORCA cmd state=%u goal=[%.2f, %.2f, %.2f] vel=[%.2f, %.2f]", orca_cmd.state,
                           orca_cmd.goal_pos[0], orca_cmd.goal_pos[1], orca_cmd.goal_pos[2], orca_cmd.linear[0],
                           orca_cmd.linear[1]);
        // 编队或移动到位后悬停
        if (orca_cmd.state == sunray_msgs::OrcaCmd::ARRIVED &&
            (formation_change_active_ || leader_goal_active_ || return_home_active_) && last_goal_valid_)
        {
            double dx = orca_cmd.goal_pos[0] - last_goal_.position.x;
            double dy = orca_cmd.goal_pos[1] - last_goal_.position.y;
            double dz = orca_cmd.goal_pos[2] - last_goal_.position.z;
            double dist2 = dx * dx + dy * dy + dz * dz;
            bool z_reached = true;
            if (agent_type_ == 0 && uav_state_ready_)
            {
                double z_err = std::fabs(uav_state_.position[2] - last_goal_.position.z);
                z_reached = z_err <= goal_z_tolerance_;
                ROS_DEBUG_THROTTLE(2.0, "Goal check: dist2=%.4f z_err=%.3f tol=%.3f", dist2, z_err,
                                   goal_z_tolerance_);
            }
            if (dist2 < 0.04 && z_reached)
            {
                formation_change_active_ = false;
                leader_goal_active_ = false;
                return_home_active_ = false;
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
    // 失效旧目标，防止 controlTimerCb 用旧 ARRIVED 提前切 HOVER
    last_goal_valid_ = false;
    // 故意用 leader_goal 的 Z 永久更新 fixed_altitude_，
    // 不固定高度时机群会上飘，因此始终需要有明确的 Z 目标值。
    // 这里让"最后一次 leader_goal 的高度"持续生效。
    if (use_fixed_altitude_ && agent_type_ == 0)
    {
        fixed_altitude_ = leader_goal_.position.z;
        ROS_INFO("Fixed altitude updated by leader_goal: %.2f", fixed_altitude_);
    }
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
    int phase = 0;
    if (time_diff < 5.0)
    {
        phase = 1;
    }
    else if (time_diff < 10.0)
    {
        phase = 2;
    }
    else if (time_diff < 15.0)
    {
        phase = 3;
    }
    else
    {
        phase = 4;
    }
    if (phase != last_takeoff_phase_)
    {
        ROS_INFO("Takeoff phase -> %d (t=%.2f)", phase, time_diff);
        last_takeoff_phase_ = phase;
    }
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
    // 3) 起飞指令（携带目标高度，确保所有 UAV 起飞到 fixed_altitude_）
    if (time_diff >= 10.0 && time_diff < 15.0)
    {
        if (uav_state_ready_ && uav_state_.armed)
        {
            control_mapper_.publishTakeoff(fixed_altitude_);
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
