#include "monocular-slam-node.hpp"

#include<opencv2/core/core.hpp>




using std::placeholders::_1;

MonocularSlamNode::MonocularSlamNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2")
{
    m_SLAM = pSLAM;
    // std::cout << "slam changed" << std::endl;
    m_image_subscriber = this->create_subscription<ImageMsg>(
        "camera",
        10,
        std::bind(&MonocularSlamNode::GrabImage, this, std::placeholders::_1));
    std::cout << "slam changed" << std::endl;

    // Create a tf broadcaster to broadcast the camera pose
    m_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    // create a static tf broadcaster to broadcast the telloBase_link to camera_link
    m_static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);

    m_pointcloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_in", 10);
    timer_ = this->create_wall_timer(std::chrono::seconds(5), std::bind(&MonocularSlamNode::PublishMapPointsAsPointCloud, this));
}

MonocularSlamNode::~MonocularSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void MonocularSlamNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    // Copy the ros image message to cv::Mat.
    try
    {
        m_cvImPtr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    Sophus::SE3f Tcw = m_SLAM->TrackMonocular(m_cvImPtr->image, Utility::StampToSec(msg->header.stamp));
    //this->BroadcastCameraTransform(Tcw);
}

void MonocularSlamNode::BroadcastCameraTransform(Sophus::SE3f Tcw)
{
    Sophus::SE3f Twc = Tcw.inverse();

    Eigen::Vector3f t = Twc.translation();
    Eigen::Quaternionf q(Twc.rotationMatrix());

    // Rotate the camera -90 degrees around the x axis, 90 degrees around the y axis
    Eigen::Quaternionf q_cam_rot = Eigen::AngleAxisf(-M_PI/2, Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f::UnitY()) * Eigen::Quaternionf::Identity();

    // Rotate the translation vector
    t = q_cam_rot * t;

    q_cam_rot = q_cam_rot * q;

    // Create a transform from world to camera
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = rclcpp::Clock().now();
    transform_stamped.header.frame_id = "map";
    transform_stamped.child_frame_id = "camera_depth_frame";
    transform_stamped.transform.translation.x = t.x();
    transform_stamped.transform.translation.y = t.y();
    transform_stamped.transform.translation.z = t.z();
    transform_stamped.transform.rotation.x = q_cam_rot.x();
    transform_stamped.transform.rotation.y = q_cam_rot.y();
    transform_stamped.transform.rotation.z = q_cam_rot.z();
    transform_stamped.transform.rotation.w = q_cam_rot.w();


    // Rotate the IMU -90 degrees around the y axis relative to the camera
    Eigen::Quaternionf q_imu_rot = Eigen::AngleAxisf(-M_PI/2, Eigen::Vector3f::UnitY()) * Eigen::Quaternionf::Identity();
    

    // Create a static transform from telloCamera to telloIMU
    geometry_msgs::msg::TransformStamped static_transform_stamped;
    static_transform_stamped.header.stamp = rclcpp::Clock().now();
    static_transform_stamped.header.frame_id = "camera_depth_frame";
    static_transform_stamped.child_frame_id = "imu";
    static_transform_stamped.transform.translation.x = 0.0;
    static_transform_stamped.transform.translation.y = -0.0028;
    static_transform_stamped.transform.translation.z = -0.043;
    static_transform_stamped.transform.rotation.x = q_imu_rot.x();
    static_transform_stamped.transform.rotation.y = q_imu_rot.y();
    static_transform_stamped.transform.rotation.z = q_imu_rot.z();
    static_transform_stamped.transform.rotation.w = q_imu_rot.w();



    // Send the transform
    //m_tf_broadcaster_->sendTransform(transform_stamped);
    
    // Send the static transform
    //m_static_tf_broadcaster_->sendTransform(static_transform_stamped);
}

/*
NOTE: This code is developed in cooperation with Ole Løkken
*/

void MonocularSlamNode::PublishMapPointsAsPointCloud()
{
    // Source: https://github.com/appliedAI-Initiative/orb_slam_2_ros/blob/master/ros/src/Node.cc#L232
    try 
    {
        auto map_points = m_SLAM->GetTrackedMapPoints(); 

        if (map_points.empty()) {
            std::cout << "Map point vector is empty!" << std::endl;
            return; // Exit if no map points
        }

        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header.stamp = rclcpp::Clock().now();
        cloud.header.frame_id = "camera_depth_frame";
        cloud.height = 1; // single row
        cloud.width = map_points.size();
        cloud.is_bigendian = false;
        cloud.is_dense = true; // no invalid points
        cloud.point_step = 3 * sizeof(float); // 3 floats per point
        cloud.row_step = cloud.point_step * cloud.width;
        cloud.fields.resize(3);

        std::string channel_id[] = {"x", "y", "z"};
        for (int i = 0; i < 3; i++) {
            cloud.fields[i].name = channel_id[i];
            cloud.fields[i].offset = i * sizeof(float);
            cloud.fields[i].count = 1;
            cloud.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
        }

        cloud.data.resize(cloud.row_step * cloud.height);

        for (size_t i = 0; i < cloud.width; i++) {
            if (map_points.at(i) && map_points.at(i)->nObs >= 2) {
                Eigen::Vector3f coords = map_points.at(i)->GetWorldPos();
                std::array<float, 3> data_array = {
                    coords[2],  // Use z for x
                    -coords[0], // Use -x for y
                    -coords[1]  // Use -y for z
                };

                std::memcpy(&cloud.data[i * cloud.point_step], data_array.data(), cloud.point_step);
            }
        }

        m_pointcloud_pub->publish(cloud);
    }

    catch (const std::exception& e) 
    {
        std::cerr << "Exception caught: " << e.what() << std::endl;
    }

}