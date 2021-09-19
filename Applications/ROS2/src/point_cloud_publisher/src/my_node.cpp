#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {

      // publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", 10);
      timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      // // PointCloud2Modifier handles allocating the data size, point_step and offset, given the height, weight and signature ("xyz") of the payload 
      sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();

      point_cloud_msg->header.frame_id = "lidar_front";
      rclcpp::Time t = this->now();
      point_cloud_msg->header.stamp.sec = t.seconds();
      point_cloud_msg->header.stamp.nanosec = t.nanoseconds();

      point_cloud_msg->is_dense = true;
      point_cloud_msg->is_bigendian = false;
      point_cloud_msg->width = 22;
      point_cloud_msg->height = 1;

      sensor_msgs::PointCloud2Modifier pcd_modifier(*point_cloud_msg);
      pcd_modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                                           "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                                           "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                                           "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);


      // sensor_msgs::PointCloud2Modifier pcd_modifier(*point_cloud_msg);
      // pcd_modifier.setPointCloud2FieldsByString(1, "xyz");      

      sensor_msgs::PointCloud2Iterator<float> iter_x(*point_cloud_msg, "x");
      sensor_msgs::PointCloud2Iterator<float> iter_y(*point_cloud_msg, "y");
      sensor_msgs::PointCloud2Iterator<float> iter_z(*point_cloud_msg, "z");
      sensor_msgs::PointCloud2Iterator<float> iter_I(*point_cloud_msg, "intensity");

      for(int i = 0; i < static_cast<int>(point_cloud_msg->width); i++, ++iter_x, ++iter_y, ++iter_z, ++iter_I) {
        *iter_x = i;
        *iter_y = i;
        *iter_z = i;
        *iter_I = i*i*i;
      }

      RCLCPP_INFO(this->get_logger(), "Publishing");
      publisher_->publish(*point_cloud_msg);
    }


    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    size_t count_;
  };

  int main(int argc, char * argv[])
  {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
  }