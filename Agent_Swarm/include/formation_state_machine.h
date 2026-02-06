// 中文说明：编队状态机接口，定义状态与状态切换
#pragma once

#include <sunray_msgs/Formation.h>

namespace agent_swarm
{

// 编队状态枚举
enum class SwarmState
{
    INIT = 0,
    TAKEOFF = 1,
    LAND = 2,
    HOVER = 3,
    FORMATION = 4,
    RETURN_HOME = 5
};

// 编队状态机
class FormationStateMachine
{
  public:
    FormationStateMachine();
    // 外部指令输入
    void onFormationCmd(const sunray_msgs::Formation &cmd);
    // 综合状态（含安全兜底）
    SwarmState effectiveState(bool leader_ok, bool orca_ok) const;
    // 请求状态
    SwarmState requestedState() const
    {
        return requested_state_;
    }
    // 直接设置状态
    void setRequestedState(SwarmState state);

  private:
    SwarmState requested_state_{SwarmState::FORMATION};
};

} // namespace agent_swarm
