#include "ros/package.h"
#include "ros/ros.h"
#include "bitbots_msgs/JointCommand.h"
#include "bitbots_msgs/DynUpResult.h"
#include "bitbots_msgs/DynUpAction.h"
#include <actionlib/server/simple_action_server.h>
#include <fstream>
#include <vector>
#include <string>
#include <iostream>

#define DYNUP_FRONT_FILE_NAME "/config/dynup_front.txt"
#define DYNUP_BACK_FILE_NAME "/config/dynup_back.txt"

typedef actionlib::SimpleActionServer<bitbots_msgs::DynUpAction> ActionServer;
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
    DYNUP_BACK = 2
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
        bitbots_msgs::JointCommand msg;
        for (int i=0; i<20; i++) {
            msg.joint_names.push_back(Id2Name(i+1));
            //joint orintation error bug fix
            if(i == 4 || i == 7){
              msg.positions.push_back(-args -> args[id][i]);
            }
            else{
              msg.positions.push_back(args -> args[id][i]);
            }
            msg.velocities.push_back(5.5);
        }
        pub -> publish(msg);
        id++;
        loop_rate.sleep();
    }
}

typedef struct {
    ros::Publisher dynup_pub;
    Arguments* dynup_front_args;
    Arguments* dynup_back_args;
} Pubs;

void callback(const bitbots_msgs::DynUpGoalConstPtr &goal, ActionServer* as,Pubs* p)
{
    if (goal -> direction == "front") {
        mos_state = DYNUP_FRONT;
        ROS_INFO("-------start dynup front-------");
        publishMsg(&(p->dynup_pub), p->dynup_front_args);
        ROS_INFO("-------finished dynup front-------");
    }
    else if (goal -> direction == "back") {
        mos_state = DYNUP_BACK;
        ROS_INFO("-------start dynup back-------");
        publishMsg(&(p->dynup_pub), p->dynup_back_args);
        ROS_INFO("-------finished dynup back-------");
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
    std::string packagePath = ros::package::getPath("thmos_dynup");
    std::string filePath_front = packagePath + DYNUP_FRONT_FILE_NAME;
    std::string filePath_back= packagePath + DYNUP_BACK_FILE_NAME;
    Arguments* dynup_front_args = readArgs(filePath_front);
    Arguments* dynup_back_args = readArgs(filePath_back);

    if (!dynup_front_args || !dynup_back_args) {
        ROS_ERROR("Read args failed!");
        return 0;
    }

    ros::init(argc, argv, "thmos_dynup_node");
    ros::NodeHandle n;
    ros::Publisher dynup_pub = n.advertise<bitbots_msgs::JointCommand>("dynup_motor_goals", 1);
    Pubs pubs = {dynup_pub , dynup_front_args, dynup_back_args};
    ActionServer server(n, "thmos_dynup", boost::bind(&callback, _1,&server,&pubs), false);
    server.start();
    ROS_INFO("-------start dynup node-------");
    ros::spin();
    return 0;
}
