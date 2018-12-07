#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <numeric>
#include <algorithm>
#include <map>

#include <ros/ros.h>
#include <ros/package.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib_msgs/GoalID.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <cv_bridge/cv_bridge.h>
#include "tf/transform_datatypes.h"
#include <yaml-cpp/yaml.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <turtle_pick/Object.h>
#include <turtle_pick/Objects.h>
#include <turtle_pick/DetectLocalize.h>
#include <dialogflow_ros/DialogflowResult.h>
#include <dialogflow_ros/DialogflowEvent.h>
#include <dialogflow_ros/DialogflowParameter.h>

class ObjectPickup
{
  public:
    ObjectPickup(ros::NodeHandle n) : nh_(n), move_base_ac_("/move_base", true)
    {
        // rgb_image_sub_ = nh_.subscribe("/camera/rgb/image_raw", 10, &ObjectLocalizer::rgbCB, this);
        // depth_image_sub_ = nh_.subscribe("/camera/depth/image_raw", 10, &ObjectLocalizer::depthCB, this);
        localize_client_ = nh_.serviceClient<turtle_pick::DetectLocalize>("/object_localize/detect_and_localize");
        robot_pose_sub_ = nh_.subscribe("/odom", 1, &ObjectPickup::pose_callback, this);
        arm_state_pub_ = nh_.advertise<sensor_msgs::JointState>("/turtlebot_arm/position_cmd", 10);
        velocity_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 10);
        objects_found_pub_ = nh_.advertise<dialogflow_ros::DialogflowEvent>("/dialogflow_client/requests/df_event", 10);
        // read in parameters

        std::string pkg_path = ros::package::getPath("turtle_pick");
        std::string config_path = pkg_path + "/config/params.yaml";
        YAML::Node parameters = YAML::LoadFile(config_path);
        const YAML::Node &arm_positions = parameters["arm_positions"];

        default_ = arm_positions["default"].as<std::vector<double>>();
        default2_ = arm_positions["default2"].as<std::vector<double>>();
        upright_ = arm_positions["upright"].as<std::vector<double>>();
        left_ = arm_positions["left"].as<std::vector<double>>();
        mid_ = arm_positions["mid"].as<std::vector<double>>();
        right_ = arm_positions["right"].as<std::vector<double>>();

        const YAML::Node &locations = parameters["locations"];
        for (YAML::const_iterator it = locations.begin(); it != locations.end(); ++it)
        {
            const YAML::Node &locations = *it;
            locationMap_.insert({locations["name"].as<std::string>(), locations["value"].as<std::vector<double>>()});
        }
    }

    // void depthCB(const sensor_msgs::ImageConstPtr &msg)
    // {
    //     depth_image_ = msg;
    // }

    // void rgbCB(const sensor_msgs::ImageConstPtr &msg)
    // {
    //     rgb_image_ = msg;
    // }

    void pose_callback(const nav_msgs::OdometryConstPtr &msg)
    {
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;
    }

    geometry_msgs::Quaternion calculate_quaternion(double normal[3])
    {
        tf::Vector3 axis_vector(normal[0], normal[1], 0);
        tf::Vector3 up_vector(1.0, 0.0, 0.0);
        tf::Vector3 right_vector = axis_vector.cross(up_vector);
        right_vector.normalized();
        tf::Quaternion q(right_vector, -1.0 * acos(axis_vector.dot(up_vector)));
        q.normalize();
        geometry_msgs::Quaternion orientation_quaternion;
        tf::quaternionTFToMsg(q, orientation_quaternion);
        return orientation_quaternion;
    }

    void move_arm(std::vector<double> position, int gripper, std::vector<double> velocity)
    {
        ROS_INFO("Moving arm");
        sensor_msgs::JointState goal;
        //goal.header.stamp = ros::Time::now();
        goal.name.push_back("joint_1");
        goal.name.push_back("joint_2");
        goal.name.push_back("joint_3");
        goal.name.push_back("gripper");
        goal.position.resize(velocity.size());
        goal.velocity.resize(velocity.size());
        goal.position = position;
        goal.position.push_back(gripper);
        goal.velocity = velocity;
        arm_state_pub_.publish(goal);
        ros::spinOnce();
        ros::Duration(5).sleep();
    }

    bool get_location()
    {
        detected_objects_.objects.clear();
        // reset arm position
        this->move_arm(default_, 380, {50, 50, 50, 50});
        // localize_srv_.request.depth_image = depth_image_;
        // localize_srv_.request.rgb_image = rgb_image_;
        if (localize_client_.call(localize_srv_))
        {
            detected_objects_ = localize_srv_.response.objects;
            dialogflow_ros::DialogflowEvent result_event;
            result_event.event_name = "objects_found";
            dialogflow_ros::DialogflowParameter dialog_parameter;
            dialog_parameter.param_name = "objects";
            if (detected_objects_.objects.size() > 0)
            {
                ROS_INFO("I see %d objects in front of me, those are:", int(detected_objects_.objects.size()));
                for (int i = 0; i < detected_objects_.objects.size(); i++)
                {
                    std::cout << i + 1 << ": " << detected_objects_.objects[i].Class << std::endl;
                    dialog_parameter.value.push_back(detected_objects_.objects[i].Class);
                }
                result_event.parameters.push_back(dialog_parameter);
                objects_found_pub_.publish(result_event);
                std::cout << "Which object do you want me to pick up?" << std::endl;
                return true;
            }
            else
            {
                ROS_INFO("No object detected");
                return false;
            }
        }
        else
        {
            ROS_INFO("Detection & Localization Failed...");
            return false;
        }
    }

    // bool pick_up(int index)
    // {
    //     ROS_INFO_STREAM("Ok, I'll pick up the " << detected_objects_.objects[index - 1].Class);
    //     move_base_msgs::MoveBaseGoal move_goal;
    //     float distance_x = detected_objects_.objects[index - 1].position.x - robot_x_;
    //     float distance_y = detected_objects_.objects[index - 1].position.y - robot_y_;
    //     float mag = sqrt(distance_x * distance_x + distance_y * distance_y);
    //     float normal_x = distance_x / mag;
    //     float normal_y = distance_y / mag;
    //     move_goal.target_pose.header.frame_id = "/odom";
    //     move_goal.target_pose.header.stamp = ros::Time::now();
    //     move_goal.target_pose.pose.position.x = detected_objects_.objects[index - 1].position.x - normal_x * 0.4;
    //     move_goal.target_pose.pose.position.y = detected_objects_.objects[index - 1].position.y - normal_y * 0.4;
    //     double normal[3] = {normal_x, normal_y, 0};
    //     move_goal.target_pose.pose.orientation = this->calculate_quaternion(normal);
    //     ROS_INFO("move_base goal %f, %f ", move_goal.target_pose.pose.position.x, move_goal.target_pose.pose.position.y);
    //     while (!move_base_ac_.waitForServer(ros::Duration(5.0)))
    //     {
    //         ROS_INFO("Waiting for the move_base action server to come up");
    //     }
    //     ROS_INFO("Sending goal");
    //     move_base_ac_.sendGoal(move_goal);
    //     move_base_ac_.waitForResult();
    //     if (move_base_ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    //     {
    //         ROS_INFO("Arrive");
    //         geometry_msgs::Twist tw;
    //         tw.linear.x = 0.05;
    //         std::vector<double> velocity{50, 50, 50, 50, 50};
    //         this->move_arm(default_2_, velocity);
    //         this->move_arm(hori_high_, velocity);
    //         ros::Time endTime = ros::Time::now() + ros::Duration(3.2);
    //         while (ros::Time::now() < endTime)
    //         {
    //             velocity_pub_.publish(tw);
    //         }
    //         this->move_arm(hori_high_close_, velocity);
    //         this->move_arm(default_2_, velocity);
    //         this->move_arm(default_, velocity);
    //     }
    //     else
    //     {
    //         ROS_INFO("Cannot not navigation to object...");
    //         return false;
    //     }
    //     return true;
    // }

    void pick_up(int index)
    {
        ROS_INFO_STREAM("Ok, I'll pick up the " << detected_objects_.objects[index].Class);
        // get the object location in the base_link frame
        geometry_msgs::PointStamped point_odom;
        geometry_msgs::PointStamped point_base;
        point_odom.header.frame_id = "odom";
        point_odom.point.x = detected_objects_.objects[index].position.x;
        point_odom.point.y = detected_objects_.objects[index].position.y;
        point_odom.point.z = 0;
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tf2_listener(tfBuffer);
        geometry_msgs::TransformStamped odom_to_base_link;
        odom_to_base_link = tfBuffer.lookupTransform("base_link", "odom", ros::Time(0), ros::Duration(3.0));
        tf2::doTransform(point_odom, point_base, odom_to_base_link);
        geometry_msgs::Twist tw;
        tw.linear.x = 0.1;
        // float move_time = (point_base.point.x - 0.25) / 0.1;
        float move_time = 4.5;
        ROS_INFO("x: %f, y %f, time: %f", point_base.point.x, point_base.point.y, move_time);
        this->move_arm(default2_, 380, {50, 50, 50, 50});
        std::vector<double> select;
        // pick left
        if (point_base.point.y > 0.03)
        {
            select = left_;
            select[0] = point_base.point.y * 500 + 218;
        }

        // pick right
        else if (point_base.point.y < -0.03)
        {
            select = right_;
            select[0] = point_base.point.y * 700 + 218;
        }
        // pick mid
        else
        {
            select = mid_;
        }
        this->move_arm(select, 380, {50, 50, 50, 50});
        ros::Time endTime = ros::Time::now() + ros::Duration(move_time);
        while (ros::Time::now() < endTime)
        {
            velocity_pub_.publish(tw);
        }
        tw.linear.x = 0;
        velocity_pub_.publish(tw);
        ros::Duration(1).sleep();
        if (detected_objects_.objects[index].Class.compare("snack") == 0)
        {
            this->move_arm(select, 710, {50, 50, 50, 50});
            this->move_arm(default2_, 710, {50, 50, 50, 50});
            this->move_arm(default_, 710, {50, 50, 50, 50});
        }
        else
        {
            this->move_arm(select, 610, {50, 50, 50, 50});
            this->move_arm(default2_, 610, {50, 50, 50, 50});
            this->move_arm(default_, 610, {50, 50, 50, 50});
        }
        // move back
        tw.linear.x = -0.1;
        endTime = ros::Time::now() + ros::Duration(move_time);
        while (ros::Time::now() < endTime)
        {
            velocity_pub_.publish(tw);
        }
        return;
    }

    int get_index(turtle_pick::Objects objects, std::string object)
    {
        std::vector<std::string> objects_list;
        objects_list.clear();
        for (int i; i < objects.objects.size(); i++)
        {
            objects_list.push_back(objects.objects[i].Class);
        }
        auto it = std::find(objects_list.begin(), objects_list.end(), object);
        if (it == objects_list.end())
        {
            ROS_INFO("Couldn't find. Repeat");
            return -1;
        }
        else
        {
            auto index = std::distance(objects_list.begin(), it);
            return index;
        }
    }

    void dialogflow_watcher()
    {
        ROS_INFO("Waiting for voice command...");
        boost::shared_ptr<dialogflow_ros::DialogflowResult const> sharedPtr;
        sharedPtr = ros::topic::waitForMessage<dialogflow_ros::DialogflowResult>("/dialogflow_client/results", ros::Duration(30));
        if (sharedPtr == NULL)
        {
            ROS_INFO("No Voice Command Received");
            return;
        }
        else
            voice_command_ = *sharedPtr;

        if (voice_command_.action.compare("objects_found") == 0)
        {
            ROS_INFO("Command Received: Detecting & Localizing...");
            this->get_location();
            while (true)
            {
                boost::shared_ptr<dialogflow_ros::DialogflowResult const> sharedPtr;
                sharedPtr = ros::topic::waitForMessage<dialogflow_ros::DialogflowResult>("/dialogflow_client/results", ros::Duration(50));
                if (sharedPtr == NULL)
                {
                    ROS_INFO("No Voice Command Received");
                    return;
                }
                else
                    voice_command_ = *sharedPtr;
                int index = this->get_index(detected_objects_, voice_command_.parameters[0].value[0]);
                if (index != -1)
                {
                    ROS_INFO("Command Received: Picking...");
                    this->pick_up(index);
                    break;
                }
                else
                    continue;
            }
        }
        else if (voice_command_.action.compare("move_to") == 0)
        {
            ROS_INFO("Command Received: Moving...");
            move_base_msgs::MoveBaseGoal move_goal;
            move_goal.target_pose.header.frame_id = "/map";
            move_goal.target_pose.header.stamp = ros::Time::now();
            move_goal.target_pose.pose.position.x = locationMap_[voice_command_.parameters[0].value[0]][0];
            move_goal.target_pose.pose.position.y = locationMap_[voice_command_.parameters[0].value[0]][1];
            move_goal.target_pose.pose.orientation.x = 0;
            move_goal.target_pose.pose.orientation.y = 0;
            move_goal.target_pose.pose.orientation.z = locationMap_[voice_command_.parameters[0].value[0]][2];
            move_goal.target_pose.pose.orientation.w = locationMap_[voice_command_.parameters[0].value[0]][3];
            while (!move_base_ac_.waitForServer(ros::Duration(5.0)))
            {
                ROS_INFO("Waiting for the move_base action server to come up");
            }
            ROS_INFO("Sending goal");
            move_base_ac_.sendGoal(move_goal);
            move_base_ac_.waitForResult();
            if (move_base_ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("Arrive");
            }
        }
        else
        {
            ROS_INFO("I don't understand...");
        }
        ROS_INFO("DONE");
        return;
    }

  private:
    ros::NodeHandle nh_;
    ros::ServiceClient localize_client_;
    ros::Subscriber robot_pose_sub_;
    ros::Publisher arm_state_pub_, velocity_pub_, objects_found_pub_;
    dialogflow_ros::DialogflowResult voice_command_;

    turtle_pick::Objects detected_objects_;
    turtle_pick::DetectLocalize localize_srv_;
    float robot_x_, robot_y_;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_ac_;
    std::vector<double> default_, default2_, upright_, left_, mid_, right_;
    std::map<std::string, std::vector<double>> locationMap_;
};

int main(int argc, char **argv)
{
    int index;
    ros::init(argc, argv, "object_pickup");
    ros::NodeHandle n("~");

    ObjectPickup object_pickup(n);
    ros::Duration(2).sleep();

    while (ros::ok())
    {
        object_pickup.dialogflow_watcher();
    }

    return 0;
}