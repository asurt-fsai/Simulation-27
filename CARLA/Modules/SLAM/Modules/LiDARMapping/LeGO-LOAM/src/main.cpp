#include "rclcpp/rclcpp.hpp"
#include "featureAssociation.h"
#include "imageProjection.h"
#include "mapOptimization.h"
#include "transformFusion.h"
#include "StatusPublisher.h"
#include <cmath>
#include <nav_msgs/msg/odometry.hpp>

int main(int argc, char** argv) {
  Channel<ProjectionOut> projection_out_channel(true);
  Channel<AssociationOut> association_out_channel(false);

  rclcpp::init(argc, argv);

  // Create a dedicated node for status publishing
  auto status_node = std::make_shared<rclcpp::Node>("legoloam_status_node");
  StatusPublisher status("/status/legoloam", status_node);
  status.starting();  // Publish "Starting" immediately

  // This process owns historical keyframes and an incremental pose graph.
  // Terminate cleanly when CARLA rewinds or teleports the hero; the module
  // supervisor may restart a fresh graph if the user still has SLAM enabled.
  // Only the preexisting bridge odometry is observed; no simulator actions.
  double last_odom_stamp = -1.0;
  double last_odom_x = 0.0;
  double last_odom_y = 0.0;
  auto odom_guard = status_node->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 5, [&](nav_msgs::msg::Odometry::SharedPtr odom) {
        const double stamp = odom->header.stamp.sec +
                             odom->header.stamp.nanosec * 1e-9;
        const double x = odom->pose.pose.position.x;
        const double y = odom->pose.pose.position.y;
        if (last_odom_stamp >= 0.0) {
          const bool rewound = stamp + 0.25 < last_odom_stamp;
          const bool teleported =
              stamp >= last_odom_stamp &&
              stamp - last_odom_stamp < 2.0 &&
              std::hypot(x - last_odom_x, y - last_odom_y) > 8.0;
          if (rewound || teleported) {
            RCLCPP_ERROR(status_node->get_logger(),
                         "CARLA pose/time discontinuity; resetting LiDAR "
                         "SLAM by ending this process");
            rclcpp::shutdown();
            return;
          }
        }
        last_odom_stamp = stamp;
        last_odom_x = x;
        last_odom_y = y;
      });

  // Create the LOAM processing nodes
  auto IP = std::make_shared<ImageProjection>("image_projection", projection_out_channel);
  auto FA = std::make_shared<FeatureAssociation>("feature_association", projection_out_channel, association_out_channel);
  auto MO = std::make_shared<MapOptimization>("map_optimization", association_out_channel);
  auto TF = std::make_shared<TransformFusion>("transform_fusion");

  RCLCPP_INFO(IP->get_logger(), "\033[1;32m---->\033[0m ImageProjection Started.");
  RCLCPP_INFO(FA->get_logger(), "\033[1;32m---->\033[0m FeatureAssociation Started.");
  RCLCPP_INFO(MO->get_logger(), "\033[1;32m---->\033[0m MapOptimization Started.");
  RCLCPP_INFO(TF->get_logger(), "\033[1;32m---->\033[0m TransformFusion Started.");

  status.ready();  // Publish "Ready" after setup

  // Add a timer to continuously publish "Running" status
  auto running_timer = status_node->create_wall_timer(
    std::chrono::milliseconds(50),  // ✅ Correct way for 0.1 seconds
    [&status]() {
      status.running();
    }
  );


  // Multi-threaded executor with 4 threads
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(IP);
  executor.add_node(FA);
  executor.add_node(MO);
  executor.add_node(TF);
  executor.add_node(status_node);  // Include status heartbeat node

  executor.spin();

  rclcpp::shutdown();
  return 0;
}
