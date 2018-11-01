#include <ros/ros.h>
#include <iostream>
#include <fstream>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Int8.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/CheckForObjectsAction.h>

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>

#include <turtle_pick/Object.h>
#include <turtle_pick/Objects.h>
#include <turtle_pick/DetectLocalize.h>

class ObjectLocalizer
{
  public:
    ObjectLocalizer(ros::NodeHandle n) : nh_(n), depth_img_cv_(new cv_bridge::CvImage), check_for_objects_ac_("/darknet_ros/check_for_objects/", true)
    {
        info_sub_ = nh_.subscribe("/camera/rgb/camera_info", 10, &ObjectLocalizer::infoCB, this);
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("objects_marker", 10);
        location_pub_ = nh_.advertise<turtle_pick::Objects>("objects_location", 10);
        localize_server_ = nh_.advertiseService("detect_and_localize", &ObjectLocalizer::detect_localize_CB, this);
    }

    void infoCB(const sensor_msgs::CameraInfoConstPtr &msg)
    {
        model1_.fromCameraInfo(msg);
    }

    geometry_msgs::Point get_coordinate(int pixel_x, int pixel_y)
    {
        // get coordinate of object in camera frame
        cv::Point3d coordiante_camera_frame;
        cv::Point2d pixel_point(pixel_x, pixel_y);
        float depth = depth_img_cv_->image.at<short int>(pixel_point);
        cv::Point3d xyz = model1_.projectPixelTo3dRay(pixel_point);
        cv::Point3d coordinate = xyz * depth;
        if (depth > 0.01)
        {
            coordiante_camera_frame = coordinate / 1000;
        }
        // get coordiante of object in odom frame
        tf::TransformListener listener;
        tf::StampedTransform stampedtransform;
        listener.waitForTransform("/odom", "/camera_rgb_optical_frame", ros::Time::now(), ros::Duration(3.0));
        listener.lookupTransform("/odom", "/camera_rgb_optical_frame", ros::Time(0), stampedtransform);
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(coordiante_camera_frame.x, coordiante_camera_frame.y, coordiante_camera_frame.z));
        tf::Transform ntransform;
        ntransform = stampedtransform * transform;
        geometry_msgs::Point object_location;
        object_location.x = ntransform.getOrigin().x();
        object_location.y = ntransform.getOrigin().y();
        object_location.z = ntransform.getOrigin().z(); // xyz is in /odom frame
        return object_location;
    }

    bool marker_publish(turtle_pick::Objects objects_to_be_pub)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/odom";
        marker.header.stamp = ros::Time();
        marker.ns = "detected_objects";
        marker.action = visualization_msgs::Marker::DELETEALL;
        marker_pub_.publish(marker);
        for (int i = 0; i < objects_to_be_pub.objects.size(); i++)
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "/odom";
            marker.header.stamp = ros::Time();
            marker.ns = "detected_objects";
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.id = i;
            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker.scale.z = 0.2;
            marker.text = objects_to_be_pub.objects[i].Class;
            marker.pose.position = objects_to_be_pub.objects[i].position;
            // Points are green
            marker.color.g = 1.0f;
            marker.color.a = 1.0;
            marker_pub_.publish(marker);
        }
        ROS_INFO("Object(s) Location Published!");
        return true;
    }

    bool detect_localize_CB(turtle_pick::DetectLocalize::Request &req,
                            turtle_pick::DetectLocalize::Response &res)
    {
        // get latest rgb image and depth image
        boost::shared_ptr<sensor_msgs::Image const> sharedPtr;
        sharedPtr = ros::topic::waitForMessage<sensor_msgs::Image>("/camera/rgb/image_raw", ros::Duration(10));
        if (sharedPtr == NULL)
        {
            ROS_INFO("No RGB image received");
            return false;
        }
        else
            rgb_image_ = *sharedPtr;
        sharedPtr = ros::topic::waitForMessage<sensor_msgs::Image>("/camera/depth/image_raw", ros::Duration(10));
        if (sharedPtr == NULL)
        {
            ROS_INFO("No depth image received");
            return false;
        }
        else
        {
            depth_image_ = *sharedPtr;
            depth_img_cv_ = cv_bridge::toCvCopy(depth_image_, sensor_msgs::image_encodings::TYPE_16UC1);
        }
        // call YOLO action and localize detection results
        darknet_ros_msgs::CheckForObjectsGoal yolo_ac_goal;
        yolo_ac_goal.image = rgb_image_;
        check_for_objects_ac_.sendGoal(yolo_ac_goal);
        check_for_objects_ac_.waitForResult();
        if (check_for_objects_ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ros::Duration(2).sleep();
            // darknet_ros_msgs::CheckForObjectsResult yolo_ac_result = *check_for_objects_ac_.getResult();
            // use topic message instead of action result; action result has a weird delay
            boost::shared_ptr<darknet_ros_msgs::BoundingBoxes const> sharedPtr;
            sharedPtr = ros::topic::waitForMessage<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes", ros::Duration(10));
            if (sharedPtr == NULL)
            {
                ROS_INFO("No Detection result received");
                return false;
            }
            else
                yolo_results_ = *sharedPtr;

            // yolo_results_ = yolo_ac_result.bounding_boxes;
            detected_objects_.objects.resize(0);
            int size = yolo_results_.bounding_boxes.size();
            int pixel_x, pixel_y;
            turtle_pick::Object single_object;
            ROS_INFO("%d objects detected, now localizing", size);
            for (int i = 0; i < size; i++)
            {
                pixel_x = (yolo_results_.bounding_boxes[i].xmin + yolo_results_.bounding_boxes[i].xmax) / 2;
                pixel_y = (yolo_results_.bounding_boxes[i].ymin + yolo_results_.bounding_boxes[i].ymax) / 2;
                single_object.position = this->get_coordinate(pixel_x, pixel_y);
                single_object.Class = yolo_results_.bounding_boxes[i].Class;
                detected_objects_.objects.push_back(single_object);
            }
            // publish marker for visulization
            if (size > 0)
            {
                if (!this->marker_publish(detected_objects_))
                {
                    ROS_INFO("Failed publishing RViz markers");
                    return false;
                }
            }
            // publish location msgs
            location_pub_.publish(detected_objects_);
            res.objects = detected_objects_;
            return true;
        }
        else
        {
            ROS_INFO("Failed call YOLO detection action");
            return false;
        }
    }

  private:
    ros::NodeHandle nh_;
    ros::Subscriber info_sub_;
    ros::ServiceServer localize_server_;
    ros::Publisher location_pub_, marker_pub_;
    image_geometry::PinholeCameraModel model1_;
    sensor_msgs::Image rgb_image_, depth_image_;

    darknet_ros_msgs::BoundingBoxes yolo_results_;
    cv_bridge::CvImagePtr depth_img_cv_;

    turtle_pick::Objects detected_objects_;

    actionlib::SimpleActionClient<darknet_ros_msgs::CheckForObjectsAction> check_for_objects_ac_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_localize");
    ros::NodeHandle n("~");
    ObjectLocalizer object_localizer(n);

    ROS_INFO("Objects Detect and Localize Service Initialized!");
    ros::spin();
    return 0;
}
