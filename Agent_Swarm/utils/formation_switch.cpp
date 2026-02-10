// 中文说明：键盘控制节点，发布编队/动作/移动指令
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <limits>
#include <ros/ros.h>
#include <signal.h>
#include <sunray_msgs/Formation.h>
#include <tf/transform_datatypes.h>

using std::cin;
using std::cout;
using std::endl;

static void SigintHandler(int)
{
    ROS_INFO("[uav_command_pub] exit...");
    ros::shutdown();
}

// 主函数
int main(int argc, char **argv)
{
    ros::init(argc, argv, "uav_command_pub");
    ros::NodeHandle nh("~");
    signal(SIGINT, SigintHandler);

    ros::Publisher formation_pub = nh.advertise<sunray_msgs::Formation>("/sunray/formation_cmd", 10);
    ros::Publisher leader_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/sunray/leader_goal", 10);
    sunray_msgs::Formation formation_cmd;

    while (ros::ok())
    {
        ros::spinOnce();
        cout << "================ 编队键盘控制 ================" << endl;
        cout << "阵型指令:" << endl;
        cout << "  1  ring     (圆环阵型)" << endl;
        cout << "  4  line     (一字横队)" << endl;
        cout << "  5  column   (纵队)" << endl;
        cout << "  6  v_shape  (V形编队)" << endl;
        cout << "  7  wedge    (楔形编队)" << endl;
        cout << "动作指令:" << endl;
        cout << "  2  expand   (扩散)" << endl;
        cout << "  3  contract (聚拢)" << endl;
        cout << "控制指令:" << endl;
        cout << "  101 takeoff   起飞" << endl;
        cout << "  102 land      降落" << endl;
        cout << "  103 hover     悬停" << endl;
        cout << "  104 set_home  设置 home" << endl;
        cout << "  105 return    返航" << endl;
        cout << "移动指令:" << endl;
        cout << "  201 move leader  (输入 leader 目标位置)" << endl;
        cout << "请输入指令: ";

        int cmd = 0;
        cin >> cmd;

        if (!cin)
        {
            cin.clear();
            cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            cout << "输入无效，请输入整数指令" << endl;
            continue;
        }

        switch (cmd)
        {
        case 1:
            formation_cmd.cmd = sunray_msgs::Formation::FORMATION;
            formation_cmd.name = "ring";
            formation_pub.publish(formation_cmd);
            break;
        case 2:
            formation_cmd.cmd = sunray_msgs::Formation::FORMATION;
            formation_cmd.name = "expand";
            formation_pub.publish(formation_cmd);
            break;
        case 3:
            formation_cmd.cmd = sunray_msgs::Formation::FORMATION;
            formation_cmd.name = "contract";
            formation_pub.publish(formation_cmd);
            break;
        case 4:
            formation_cmd.cmd = sunray_msgs::Formation::FORMATION;
            formation_cmd.name = "line";
            formation_pub.publish(formation_cmd);
            break;
        case 5:
            formation_cmd.cmd = sunray_msgs::Formation::FORMATION;
            formation_cmd.name = "column";
            formation_pub.publish(formation_cmd);
            break;
        case 6:
            formation_cmd.cmd = sunray_msgs::Formation::FORMATION;
            formation_cmd.name = "v_shape";
            formation_pub.publish(formation_cmd);
            break;
        case 7:
            formation_cmd.cmd = sunray_msgs::Formation::FORMATION;
            formation_cmd.name = "wedge";
            formation_pub.publish(formation_cmd);
            break;
        case 101:
            formation_cmd.cmd = sunray_msgs::Formation::TAKEOFF;
            formation_cmd.name = "";
            formation_pub.publish(formation_cmd);
            break;
        case 102:
            formation_cmd.cmd = sunray_msgs::Formation::LAND;
            formation_cmd.name = "";
            formation_pub.publish(formation_cmd);
            break;
        case 103:
            formation_cmd.cmd = sunray_msgs::Formation::HOVER;
            formation_cmd.name = "";
            formation_pub.publish(formation_cmd);
            break;
        case 104:
            formation_cmd.cmd = sunray_msgs::Formation::SET_HOME;
            formation_cmd.name = "";
            formation_pub.publish(formation_cmd);
            break;
        case 105:
            formation_cmd.cmd = sunray_msgs::Formation::RETURN_HOME;
            formation_cmd.name = "";
            formation_pub.publish(formation_cmd);
            break;
        case 201: {
            geometry_msgs::PoseStamped goal;
            goal.header.stamp = ros::Time::now();
            goal.header.frame_id = "world";
            cout << "输入目标点 x y z yaw (空格分隔): ";
            double x, y, z, yaw;
            cin >> x >> y >> z >> yaw;
            goal.pose.position.x = x;
            goal.pose.position.y = y;
            goal.pose.position.z = z;
            goal.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
            leader_goal_pub.publish(goal);
            break;
        }
        default:
            cout << "无效指令" << endl;
            break;
        }

        ros::Duration(0.5).sleep();
    }

    return 0;
}
