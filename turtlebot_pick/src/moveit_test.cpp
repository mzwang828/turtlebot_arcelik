#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

void move_robot()
{
    ros::AsyncSpinner spinner(4); // important
    spinner.start();
    //sleep(10.0);

    moveit::planning_interface::MoveGroupInterface group("arm");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup *joint_model_group =
        group.getCurrentState()->getJointModelGroup("arm");
    group.setPlannerId("RRTConnect");
    // Create a publisher for visualizing plans in Rviz.
    moveit_msgs::DisplayTrajectory display_trajectory;
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
    visual_tools.deleteAllMarkers();

    // Getting Basic Information
    // We can print the name of the reference frame for this robot.
    ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());

    // We can also print the name of the end-effector link for this group.
    ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

    // Planning to a Pose goal
    geometry_msgs::Pose target_pose = group.getCurrentPose().pose;
    //target_pose.orientation.w = 1;
    target_pose.position.x = target_pose.position.x;
    target_pose.position.y = target_pose.position.y;
    target_pose.position.z = target_pose.position.z;
    ROS_INFO("position %f, %f, %f", target_pose.position.x, target_pose.position.y, target_pose.position.z);
    //group.setPoseReferenceFrame("base_footprint");
    group.setPoseTarget(target_pose);

    ROS_INFO("Reference frame: %s", group.getPoseReferenceFrame().c_str());

    // Now, we call the planner to compute the plan and visualize it.
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);
    bool success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    /* Sleep to give Rviz time to visualize the plan. */

    // Visualizing plans
    visual_tools.publishAxisLabeled(target_pose, "grab_pose");
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();

    sleep(10.0);

    // move the robot
    ROS_INFO("Attention: moving the arm");
    // gripper_action(0.0); // open the gripper
    group.move();
    // gripper_action(0.75*FINGER_MAX); // close the gripper
}

int main(int argc, char **argv)
{
    int index;
    ros::init(argc, argv, "move_it_test");
    ros::NodeHandle n("~");

    move_robot();

    return 0;
}