#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <CameraSystem.h>
#include <DepthCamera.h>

class VoxelSDKNode : public rclcpp::Node
{
public:
  VoxelSDKNode() : Node("voxelsdk_node")
  {
    // Initialize Publisher
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("voxel/depth/pointcloud", 10);
    
    // Setup Voxel Camera System
    sys_.reset(new Voxel::CameraSystem());
    const std::vector<Voxel::DevicePtr> &devices = sys_->scan();
    
    if (devices.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "No Voxel Depth Cameras discovered. Waiting...");
      return;
    }

    camera_ = sys_->connect(devices[0]);
    if (!camera_)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to connect to the Depth Camera.");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Successfully connected to Voxel camera: %s", camera_->id().c_str());

    // Register callback for Voxel SDK frames
    camera_->registerCallback(Voxel::DepthCamera::FRAME_XYZI_POINT_CLOUD_FRAME,
      [this](Voxel::DepthCamera &dc, const Voxel::Frame &frame, Voxel::DepthCamera::FrameType type) {
        this->frameCallback(dc, frame, type);
      });

    camera_->start();
  }

  ~VoxelSDKNode()
  {
    if (camera_ && camera_->isRunning())
    {
      camera_->stop();
      camera_->wait();
    }
  }

private:
  void frameCallback(Voxel::DepthCamera & /*dc*/, const Voxel::Frame &frame, Voxel::DepthCamera::FrameType type)
  {
    if (type != Voxel::DepthCamera::FRAME_XYZI_POINT_CLOUD_FRAME) return;

    const Voxel::PointCloudFrame *pcFrame = dynamic_cast<const Voxel::PointCloudFrame *>(&frame);
    if (!pcFrame) return;

    // Convert Voxel point cloud to ROS 2 PointCloud2 message
    auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
    msg->header.stamp = this->now();
    msg->header.frame_id = "voxel_camera_link";

    msg->height = 1; // Unordered point cloud
    msg->width = pcFrame->points.size();
    msg->is_dense = false;
    msg->is_bigendian = false;

    sensor_msgs::PointCloud2Modifier modifier(*msg);
    modifier.setPointCloud2FieldsByString(2, "xyz", "intensity");
    modifier.resize(msg->width);

    sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_i(*msg, "intensity");

    for (size_t i = 0; i < pcFrame->points.size(); ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_i)
    {
      const Voxel::IntensityPoint &pt = pcFrame->points[i];
      *iter_x = pt.x;
      *iter_y = pt.y;
      *iter_z = pt.z;
      *iter_i = pt.i;
    }

    // Publish point cloud
    publisher_->publish(std::move(msg));
  }

  std::unique_ptr<Voxel::CameraSystem> sys_;
  Voxel::DepthCameraPtr camera_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VoxelSDKNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
