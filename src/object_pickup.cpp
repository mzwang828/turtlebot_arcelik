#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <numeric>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include <actionlib_msgs/GoalID.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Pose.h>
#include <cv_bridge/cv_bridge.h>

#include <turtle_pick/Object.h>
#include <turtle_pick/Objects.h>
#include <turtle_pick/DetectLocalize.h>

class ObjectPickup
{
  public:
    ObjectPickup(ros::NodeHandle n) : nh_(n), move_base_ac_("/move_base", true)
    {
        // rgb_image_sub_ = nh_.subscribe("/camera/rgb/image_raw", 10, &ObjectLocalizer::rgbCB, this);
        // depth_image_sub_ = nh_.subscribe("/camera/depth/image_raw", 10, &ObjectLocalizer::depthCB, this);
        localize_client_ = nh_.serviceClient<turtle_pick::DetectLocalize>("/object_localize/detect_and_localize");
    }

    // void depthCB(const sensor_msgs::ImageConstPtr &msg)
    // {
    //     depth_image_ = msg;
    // }

    // void rgbCB(const sensor_msgs::ImageConstPtr &msg)
    // {
    //     rgb_image_ = msg;
    // }

    bool get_location()
    {
        // localize_srv_.request.depth_image = depth_image_;
        // localize_srv_.request.rgb_image = rgb_image_;
        if (localize_client_.call(localize_srv_))
        {
            detected_objects_ = localize_srv_.response.objects;
            if (detected_objects_.objects.size() > 0)
            {
                ROS_INFO("I see %d objects in front of me, those are:", int(detected_objects_.objects.size()));
                for (int i = 0; i < detected_objects_.objects.size(); i++)
                {
                    std::cout << i + 1 << ": " << detected_objects_.objects[i].Class << std::endl;
                }
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
            ROS_INFO("Failed call Detect&Localize service...");
            return false;
        }
    }

    void pick_up(int index)
    {
        move_base_msgs::MoveBaseGoal move_goal;
        move_goal.target_pose.header.frame_id = "/map";
        move_goal.target_pose.header.stamp = ros::Time::now();
        move_goal.target_pose.pose.position.x = detected_objects_.objects[index - 1].position.x;
        move_goal.target_pose.pose.position.y = detected_objects_.objects[index - 1].position.y;
        move_goal.target_pose.pose.orientation.w = 1.0;
        ROS_INFO("move_base goal %f, %f ", move_goal.target_pose.pose.position.x, move_goal.target_pose.pose.position.y);
        while (!move_base_ac_.waitForServer(ros::Duration(5.0)))
        {
            ROS_INFO("Waiting for the move_base action server to come up");
        }
        ROS_INFO("Sending goal");
        move_base_ac_.sendGoal(move_goal);
        move_base_ac_.waitForResult();
        if (move_base_ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            // TODO: Pick up
            ROS_INFO("Arrive");
        }
    }

  private:
    ros::NodeHandle nh_;
    ros::ServiceClient localize_client_;
    // ros::Subscriber rgb_image_sub_, depth_image_sub_;
    // sensor_msgs::Image rgb_image_, depth_image_;

    turtle_pick::Objects detected_objects_;
    turtle_pick::DetectLocalize localize_srv_;

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_ac_;
};

int main(int argc, char **argv)
{
    int index;
    ros::init(argc, argv, "object_pickup");
    ros::NodeHandle n("~");

    ObjectPickup object_pickup(n);
    // ros:spinOnce();

    if (object_pickup.get_location())
    {
        std::cin >> index;
        object_pickup.pick_up(index);
    }
    return 0;
}