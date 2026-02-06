// 中文说明：编队状态机实现，处理指令与安全兜底
#include "formation_state_machine.h"

namespace agent_swarm
{

FormationStateMachine::FormationStateMachine() = default;

void FormationStateMachine::onFormationCmd(const sunray_msgs::Formation &cmd)
{
    if (cmd.cmd == sunray_msgs::Formation::TAKEOFF)
    {
        requested_state_ = SwarmState::TAKEOFF;
    }
    else if (cmd.cmd == sunray_msgs::Formation::LAND)
    {
        requested_state_ = SwarmState::LAND;
    }
    else if (cmd.cmd == sunray_msgs::Formation::HOVER)
    {
        requested_state_ = SwarmState::HOVER;
    }
    else if (cmd.cmd == sunray_msgs::Formation::RETURN_HOME)
    {
        requested_state_ = SwarmState::RETURN_HOME;
    }
    else if (cmd.cmd == sunray_msgs::Formation::FORMATION)
    {
        requested_state_ = SwarmState::FORMATION;
    }
    else
    {
        requested_state_ = SwarmState::FORMATION;
    }
}

SwarmState FormationStateMachine::effectiveState(bool leader_ok, bool orca_ok) const
{
    if (requested_state_ == SwarmState::TAKEOFF || requested_state_ == SwarmState::LAND ||
        requested_state_ == SwarmState::HOVER || requested_state_ == SwarmState::RETURN_HOME)
    {
        return requested_state_;
    }
    // 安全兜底：leader 或 ORCA 超时则悬停
    if (!leader_ok || !orca_ok)
    {
        return SwarmState::HOVER;
    }
    return SwarmState::FORMATION;
}

void FormationStateMachine::setRequestedState(SwarmState state)
{
    requested_state_ = state;
}

} // namespace agent_swarm
