#include "ros/ros.h"
// #include "bitbots_msgs/JointCommand.h"
#include "bitbots_msgs/KickResult.h"
#include "bitbots_msgs/KickAction.h"
#include "sensor_msgs/JointState.h"
#include <std_msgs/Bool.h>
#include <actionlib/server/simple_action_server.h>
#include <fstream>
#include <vector>
#include <string>
#include <iostream>

#define KICK_FILE_NAME "/home/nvidia/thmos_ws/src/kick.txt"
#define DYNUP_FRONT_FILE_NAME "/home/nvidia/thmos_ws/src/dynup_front.txt"
#define DYNUP_BACK_FILE_NAME "/home/nvidia/thmos_ws/src/dynup_back.txt"

typedef actionlib::SimpleActionServer<bitbots_msgs::KickAction> ActionServer;
std::string Id2Name (int id) {
    if (id == 1) {
        return "neck";
    }
    else if (id == 2) {
        return "head";
    }
    else if (id == 3) {
        return "L_arm_1";
    }
    else if (id == 4) {
        return "L_arm_2";
    }
    else if (id == 5) {
        return "L_arm_3";
    }
    else if (id == 6) {
        return "R_arm_1";
    }
    else if (id == 7) {
        return "R_arm_2";
    }
    else if (id == 8) {
        return "R_arm_3";
    }
    else if (id == 9) {
        return "L_leg_1";
    }
    else if (id == 10) {
        return "L_leg_2";
    }
    else if (id == 11) {
        return "L_leg_3";
    }
    else if (id == 12) {
        return "L_leg_4";
    }
    else if (id == 13) {
        return "L_leg_5";
    }
    else if (id == 14) {
        return "L_leg_6";
    }
    else if (id == 15) {
        return "R_leg_1";
    }
    else if (id == 16) {
        return "R_leg_2";
    }
    else if (id == 17) {
        return "R_leg_3";
    }
    else if (id == 18) {
        return "R_leg_4";
    }
    else if (id == 19) {
        return "R_leg_5";
    }
    else if (id == 20) {
        return "R_leg_6";
    }
    else {
        ROS_INFO ("Id is not valid.");
        return "Error";
    }
}

typedef enum {
    DEFAULT = 0,
    DYNUP_FRONT = 1,
    DYNUP_BACK = 2,
    KICK = 3
} AnimationState;
AnimationState mos_state = DEFAULT;

typedef struct {
    int motion_num;
    int rate;
    std::vector<std::vector<float>> args;

    void print() {
        printf ("%d %d\n", motion_num, rate);
        for (int i=0; i<motion_num; i++) {
            for (int j=0; j<20; j++) {
                printf("%f ", args[i][j]);
            }
            printf("\n");
        }
    }
} Arguments;

void publishMsg (ros::Publisher* pub, Arguments* args) 
{
    ros::Rate loop_rate (args -> rate);
    int id = 0;
    while (id < args -> motion_num) {
        // ROS_INFO("%d", id);
        // bitbots_msgs::JointCommand msg;
        sensor_msgs::JointState msg;
        for (int i=0; i<20; i++) {
            msg.name.push_back(Id2Name(i+1));
            msg.position.push_back(args -> args[id][i]);
        }
        pub -> publish(msg);
        id++;
        loop_rate.sleep();
    }
}

typedef struct {
    ros::Publisher kick_pub;
    ros::Publisher dynup_pub;
    ros::Publisher kick_start;
   
    Arguments* dynup_front_args;
    Arguments* dynup_back_args;
    Arguments* kick_args;
} Pubs;

void callback(const bitbots_msgs::KickGoalConstPtr &goal, ActionServer* as,Pubs* p)
{
    if (goal -> kick_speed == 1) {
        mos_state = DYNUP_FRONT;
        ROS_INFO("-------start dynup front-------");
        //Arguments* dynup_front_args = readArgs(DYNUP_FRONT_FILE_NAME);
        publishMsg(&(p->dynup_pub), p->dynup_front_args);
        ROS_INFO("-------finished dynup front-------");
    }
    else if (goal -> kick_speed == 2) {
        mos_state = DYNUP_BACK;
        ROS_INFO("-------start dynup back-------");
        //Arguments* dynup_back_args = readArgs(DYNUP_BACK_FILE_NAME);
        publishMsg(&(p->dynup_pub), p->dynup_back_args);
        ROS_INFO("-------finished dynup back-------");
    }
    else if (goal -> kick_speed == 3) {
        mos_state = KICK;
        ROS_INFO("-------start kicking-------");
        //Arguments* kick_args = readArgs(KICK_FILE_NAME);
        std_msgs::Bool start_msg;
        start_msg.data = true;
        p->kick_start.publish(start_msg);
        publishMsg(&(p->kick_pub), p->kick_args);
        ROS_INFO ("-----Done kicking ball!-----");
        start_msg.data = false;
        p->kick_start.publish(start_msg);
    }
    else {
        mos_state = DEFAULT;
        ROS_WARN("Action protocol failed");
    }
    as->setSucceeded();
}

Arguments* readArgs (std::string filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        ROS_INFO("open failed\n");
        return nullptr;
    }
    int motion_num;
    file >> motion_num;
    int rate;
    file >> rate;
    std::vector<std::vector<float>> all_args;
    int lines = 1;
    while (lines <= motion_num) {
        std::vector<float> args;
        for (int i=0; i<20; i++) {
            double tp;
            file >> tp;
            args.push_back(tp);
        }
        all_args.push_back(args);
        lines++;
    }
    file.close();
    Arguments* arg = new Arguments();
    arg -> motion_num = motion_num;
    arg -> rate = rate;
    arg -> args = all_args;
    return arg;
}

int main (int argc, char** argv) {
    Arguments* dynup_front_args = readArgs(DYNUP_FRONT_FILE_NAME);
    Arguments* dynup_back_args = readArgs(DYNUP_BACK_FILE_NAME);
    Arguments* kick_args = readArgs(KICK_FILE_NAME);
    if (!dynup_front_args || !dynup_back_args || !kick_args) {
        ROS_ERROR("Read args failed!");
        return 0;
    }

    ros::init(argc, argv, "thmos_animation_node");
    ros::NodeHandle n;
    ros::Publisher kick_pub = n.advertise<sensor_msgs::JointState>("kick_motor_goals", 1);
    ros::Publisher dynup_pub = n.advertise<sensor_msgs::JointState>("dynup_motor_goals", 1);
    ros::Publisher kick_start = n.advertise<std_msgs::Bool>("start_kick", 1);
    
    Pubs pubs = { kick_pub ,dynup_pub ,kick_start,dynup_front_args, dynup_back_args ,kick_args };
    ActionServer server(n, "thmos_animation", boost::bind(&callback, _1,&server,&pubs), false);
    server.start();
    ROS_INFO("-------start animation node-------");
    ros::spin();
    // while (ros::ok()) {
    //     bitbots_msgs::KickResult result;
    //     if (mos_state == DYNUP_FRONT) {
    //         publishMsg(&dynup_pub, dynup_front_args);
    //         ROS_INFO ("Done dynup front!");
    //         result.result = bitbots_msgs::KickResult::SUCCESS;
    //         server.setSucceeded(result);
    //     }
    //     else if (mos_state == DYNUP_BACK) {
    //         publishMsg(&dynup_pub, dynup_back_args);
    //         ROS_INFO ("Done dynup back!");
    //         result.result = bitbots_msgs::KickResult::SUCCESS;
    //         server.setSucceeded(result);
    //     }
    //     else if (mos_state == KICK) {
    //         std_msgs::Bool start_msg;
    //         start_msg.data = true;
    //         kick_start.publish(start_msg);
    //         publishMsg(&kick_pub, kick_args);
    //         ROS_INFO ("Done kicking ball!");
    //         start_msg.data = false;
    //         kick_start.publish(start_msg);
    //         result.result = bitbots_msgs::KickResult::SUCCESS;
    //         server.setSucceeded(result);
    //     }
    //     mos_state = DEFAULT;
    //     ros::spinOnce();
    // }
    return 0;
}
