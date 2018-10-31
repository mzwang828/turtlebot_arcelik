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

#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>

#include <turtle_pick/Object.h>
#include <turtle_pick/Objects.h>

class ObjectLocalizer
{
  public:
    ObjectLocalizer(ros::NodeHandle n) : nh_(n), yolo_msgs_(new darknet_ros_msgs::BoundingBoxes), depth_msgs_(new sensor_msgs::Image), depth_img_cv_(new cv_bridge::CvImage)
    {
        info_sub_ = nh_.subscribe("/camera/rgb/camera_info", 10, &ObjectLocalizer::infoCB, this);
        depth_image_sub_ = nh_.subscribe("/camera/depth_registered/image_raw", 10, &ObjectLocalizer::depthCB, this);
        yolo_sub_ = nh_.subscribe<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes", 1, &ObjectLocalizer::yoloCB, this);
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("objects_marker", 10);
        location_pub_ = nh_.advertise<turtle_pick::Objects>("objects_location", 10);
    }

    void infoCB(const sensor_msgs::CameraInfoConstPtr &msg)
    {
        model1_.fromCameraInfo(msg);
    }

    void yoloCB(const darknet_ros_msgs::BoundingBoxesConstPtr &msg)
    {
        yolo_msgs_ = msg;
    }

    void depthCB(const sensor_msgs::ImageConstPtr &msg)
    {
        depth_img_cv_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
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
        // get coordiante of object in map frame
        tf::TransformListener listener;
        tf::StampedTransform stampedtransform;
        // listener.waitForTransform("/map", "/camera_rgb_optical_frame", ros::Time::now(), ros::Duration(3.0));
        // listener.lookupTransform("/map", "/camera_rgb_optical_frame", ros::Time(0), stampedtransform);
        listener.waitForTransform("/camera_rgb_optical_frame", "/camera_rgb_optical_frame", ros::Time::now(), ros::Duration(3.0));
        listener.lookupTransform("/camera_rgb_optical_frame", "/camera_rgb_optical_frame", ros::Time(0), stampedtransform);
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(coordiante_camera_frame.x, coordiante_camera_frame.y, coordiante_camera_frame.z));
        tf::Transform ntransform;
        ntransform = stampedtransform * transform;
        geometry_msgs::Point object_location;
        object_location.x = ntransform.getOrigin().x();
        object_location.y = ntransform.getOrigin().y();
        object_location.z = ntransform.getOrigin().z(); // xyz is in /map frame
        return object_location;
    }

    bool marker_publish(turtle_pick::Objects objects_to_be_pub)
    {
        for (int i = 0; i < objects_to_be_pub.objects.size(); i++)
        {
            visualization_msgs::Marker marker;
            // marker.header.frame_id = "/map";
            marker.header.frame_id = "/camera_rgb_optical_frame";
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

    void pub_locations()
    {
        detected_objects.objects.resize(0);
        int size = yolo_msgs_->bounding_boxes.size();
        int pixel_x, pixel_y;
        turtle_pick::Object single_object;
        ROS_INFO("%d objects detected, now localizing", size);
        for (int i = 0; i < size; i++)
        {
            pixel_x = (yolo_msgs_->bounding_boxes[i].xmin + yolo_msgs_->bounding_boxes[i].xmax) / 2;
            pixel_y = (yolo_msgs_->bounding_boxes[i].ymin + yolo_msgs_->bounding_boxes[i].ymax) / 2;
            single_object.position = this->get_coordinate(pixel_x, pixel_y);
            single_object.Class = yolo_msgs_->bounding_boxes[i].Class;
            detected_objects.objects.push_back(single_object);
        }
        // publish marker for visulization
        if (size > 0)
        {
            if (!this->marker_publish(detected_objects))
            {
                ROS_INFO("Failed publishing RViz markers");
            }
        }
        // publish location msgs
        location_pub_.publish(detected_objects);
    }

  private:
    ros::NodeHandle nh_;
    ros::Subscriber info_sub_, depth_image_sub_, yolo_sub_, yolo_number_sub_;
    ros::Publisher location_pub_, marker_pub_;
    image_geometry::PinholeCameraModel model1_;

    darknet_ros_msgs::BoundingBoxesConstPtr yolo_msgs_;
    sensor_msgs::ImageConstPtr depth_msgs_;
    cv_bridge::CvImagePtr depth_img_cv_;

    turtle_pick::Objects detected_objects;

    std::vector<cv::Point3d> coordinate_camera_;

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_localizing");
    ros::NodeHandle n("~");
    ObjectLocalizer object_localizer(n);
    ros::Rate loop_rate(20); // 10 Hz
    ros::Duration(2).sleep();
    ros::spinOnce();
    ROS_INFO("Objects Localizing Initialized!");
    while (ros::ok())
    {
        object_localizer.pub_locations();
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
