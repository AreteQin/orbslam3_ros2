// Created by qin on 09/02/25.
// Modified by Assistant on 2025-02-10 to use TF instead of subscribing to /position/camera_pose.

#include <matplotlibcpp.h>
namespace plt = matplotlibcpp;

#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/channel_float32.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <glog/logging.h>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <pangolin/pangolin.h>

#include <fstream>
#include <iomanip>
#include <limits>
#include <vector>

// Constants for WGS84 ellipsoid
const double a = 6378137.0;            // Semi-major axis (meters)
const double e2 = 0.00669437999014;      // Square of eccentricity
const double b = 6356752.314245;         // Semi-minor axis (meters)

template <typename T>
void GPS2ECEF(const T lat, const T lon, const T alt, T& X, T& Y, T& Z)
{
  T phi    = lat * M_PI / 180.0;
  T lambda = lon * M_PI / 180.0;
  T x_z_squar = a * a * b * b / (b * b + a * a * pow(tan(phi), 2));
  T x_z = sqrt(x_z_squar);
  T R = sqrt(x_z_squar * (1 + tan(phi) * tan(phi)));
  Z = (R + alt) * sin(phi);
  X = (R + alt) * cos(phi) * cos(lambda);
  Y = (R + alt) * cos(phi) * sin(lambda);
}

template <typename T>
void ECEF2GPS(const T X, const T Y, const T Z, T& lat, T& lon, T& alt)
{
  lon = atan2(Y, X) * 180 / M_PI;
  lat = atan2(Z, sqrt(X * X + Y * Y)) * 180 / M_PI;
  T phi    = lat * M_PI / 180.0;
  T lambda = lon * M_PI / 180.0;
  T x_z_squar = a * a * b * b / (b * b + a * a * pow(tan(phi), 2));
  T R = sqrt(x_z_squar * (1 + tan(phi) * tan(phi)));
  alt = sqrt(X * X + Y * Y + Z * Z) - R;
}

class GeoPositioning : public rclcpp::Node
{
public:
  // The constructor sets up publishers, subscribers, and the TF listener.
  GeoPositioning()
  : Node("geo_positioning"),
    origin_frame_index(800)
  {
    // Publishers remain unchanged.
    real_scale_pub_ = this->create_publisher<sensor_msgs::msg::ChannelFloat32>("/position/real_scale", 10);
    fire_spots_GPS_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/position/fire_spots_GPS", 10);

    // Initialize message_filters subscribers for fire spots and GPS.
    fire_spots_sub_.subscribe(this, "/position/fire_spots", rmw_qos_profile_default);
    GPS_sub_.subscribe(this, "/dji_osdk_ros/gps_position", rmw_qos_profile_default);

    // Note: We removed the camera pose subscriber since we now use TF.

    // Initialize TF buffer and listener.
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Set up synchronizer using an approximate time policy for two topics.
    // Define a new sync policy for fire_spots and GPS only.
    typedef message_filters::sync_policies::ApproximateTime<
      geometry_msgs::msg::PoseArray,
      sensor_msgs::msg::NavSatFix
    > MySyncPolicy2;

    sync2_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy2>>(
              MySyncPolicy2(10), fire_spots_sub_, GPS_sub_);
    sync2_->registerCallback(std::bind(&GeoPositioning::callback, this,
                                      std::placeholders::_1,
                                      std::placeholders::_2));
  }

  // Accessor functions.
  int size() { return camera_trajectory_SLAM_.size(); }
  std::vector<geometry_msgs::msg::PoseStamped>* getCameraPosesSLAM() { return &camera_trajectory_SLAM_; }
  std::vector<sensor_msgs::msg::NavSatFix>* getCameraPosesGPS() { return &camera_trajectory_GPS_; }
  geometry_msgs::msg::PoseArray* getFireSpotsGPS() { return &fire_spots_GPS_; }
  std::vector<Eigen::Vector3d>* getCameraPosesGPSCalculated() { return &camera_trajectory_GPS_calculated_; }
  std::vector<double>* getRealScales() { return &real_scales_; }
  std::vector<Eigen::Vector3d> getAverageFireSpotsGPS() { return fire_spots_GPS_average_; }

  const int origin_frame_index;

private:
  // Publishers.
  rclcpp::Publisher<sensor_msgs::msg::ChannelFloat32>::SharedPtr real_scale_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr fire_spots_GPS_pub_;

  // Message filters subscribers for fire spots and GPS.
  message_filters::Subscriber<geometry_msgs::msg::PoseArray> fire_spots_sub_;
  message_filters::Subscriber<sensor_msgs::msg::NavSatFix> GPS_sub_;

  // Synchronizer for two topics.
  typedef message_filters::sync_policies::ApproximateTime<
    geometry_msgs::msg::PoseArray,
    sensor_msgs::msg::NavSatFix
  > MySyncPolicy2;
  std::shared_ptr<message_filters::Synchronizer<MySyncPolicy2>> sync2_;

  // TF2 buffer and listener for getting the transform from "map" to "H20T".
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Data storage.
  std::vector<geometry_msgs::msg::PoseStamped> camera_trajectory_SLAM_;
  std::vector<sensor_msgs::msg::NavSatFix> camera_trajectory_GPS_;
  geometry_msgs::msg::PoseArray fire_spots_SLAM_;
  geometry_msgs::msg::PoseArray fire_spots_GPS_;
  std::vector<Eigen::Vector3d> camera_trajectory_GPS_calculated_, fire_spots_GPS_average_;
  std::vector<double> real_scales_;
  double scales_sum_ = 0;

  // Callback invoked when synchronized messages are received.
  // Now we only receive fire spots and GPS fixes.
  void callback(const geometry_msgs::msg::PoseArray::ConstSharedPtr& fire_spots_msg,
                const sensor_msgs::msg::NavSatFix::ConstSharedPtr& GPS_msg)
  {
    // Look up the transform from "map" to "H20T" using TF.
    geometry_msgs::msg::TransformStamped transformStamped;
    try {
      // Use the GPS message header stamp (or rclcpp::Time(0) for latest) as the lookup time.
      transformStamped = tf_buffer_->lookupTransform("map", "H20T", GPS_msg->header.stamp);
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Could not transform from map to H20T: %s", ex.what());
      return;
    }
    // Convert the transform into a PoseStamped.
    geometry_msgs::msg::PoseStamped camera_pose_msg;
    camera_pose_msg.header = transformStamped.header;
    camera_pose_msg.pose.position.x = transformStamped.transform.translation.x;
    camera_pose_msg.pose.position.y = transformStamped.transform.translation.y;
    camera_pose_msg.pose.position.z = transformStamped.transform.translation.z;
    camera_pose_msg.pose.orientation    = transformStamped.transform.rotation;

    // Store the latest camera pose (obtained from TF) and GPS fix.
    camera_trajectory_SLAM_.push_back(camera_pose_msg);
    camera_trajectory_GPS_.push_back(*GPS_msg);

    LOG(INFO) << "Total number of camera poses: " << camera_trajectory_SLAM_.size() << ".";

    // If there are fewer than two camera poses, we cannot compute a time interval.
    if (camera_trajectory_SLAM_.size() < 2)
      return;

    double time_interval = rclcpp::Time(camera_trajectory_SLAM_.back().header.stamp).seconds() -
                           rclcpp::Time(camera_trajectory_SLAM_[camera_trajectory_SLAM_.size() - 2].header.stamp).seconds();
    LOG(INFO) << "The time consumed by SLAM: " << time_interval << " seconds.";

    // Wait until we have enough frames.
    if (camera_trajectory_SLAM_.size() < origin_frame_index * 1.1)
      return;

    // Compute the SLAM distance between the origin frame and the last frame.
    Eigen::Vector3d diff(
      camera_trajectory_SLAM_.back().pose.position.x - camera_trajectory_SLAM_[origin_frame_index].pose.position.x,
      camera_trajectory_SLAM_.back().pose.position.y - camera_trajectory_SLAM_[origin_frame_index].pose.position.y,
      camera_trajectory_SLAM_.back().pose.position.z - camera_trajectory_SLAM_[origin_frame_index].pose.position.z);
    double distance = diff.norm();

    double x1, y1, z1, x2, y2, z2;
    GPS2ECEF(camera_trajectory_GPS_[origin_frame_index].latitude,
             camera_trajectory_GPS_[origin_frame_index].longitude,
             camera_trajectory_GPS_[origin_frame_index].altitude, x1, y1, z1);
    GPS2ECEF(camera_trajectory_GPS_.back().latitude,
             camera_trajectory_GPS_.back().longitude,
             camera_trajectory_GPS_.back().altitude, x2, y2, z2);
    double distance_lat = x2 - x1;
    double distance_lon = y2 - y1;
    double distance_alt = z2 - z1;
    double distance_ECEF = sqrt(distance_lat * distance_lat +
                                distance_lon * distance_lon +
                                distance_alt * distance_alt);

    // Calculate and average the real scale.
    double real_scale = distance_ECEF / distance;
    real_scales_.push_back(real_scale);
    scales_sum_ += real_scale;
    real_scale = scales_sum_ / real_scales_.size();

    sensor_msgs::msg::ChannelFloat32 real_scale_msg;
    real_scale_msg.name = "real_scale";
    real_scale_msg.values.push_back(real_scale);
    real_scale_pub_->publish(real_scale_msg);

    // Calculate the rotation matrix using the cross product and Rodrigues formula.
    Eigen::Vector3d a = real_scale * Eigen::Vector3d(
      camera_trajectory_SLAM_.back().pose.position.x - camera_trajectory_SLAM_[origin_frame_index].pose.position.x,
      camera_trajectory_SLAM_.back().pose.position.y - camera_trajectory_SLAM_[origin_frame_index].pose.position.y,
      camera_trajectory_SLAM_.back().pose.position.z - camera_trajectory_SLAM_[origin_frame_index].pose.position.z);
    Eigen::Vector3d b(distance_lat, distance_lon, distance_alt);

    Eigen::Vector3d v = a.cross(b);
    double s = v.norm();
    double c = a.dot(b);
    Eigen::Matrix3d nx;
    nx << 0, -v[2], v[1],
          v[2], 0, -v[0],
         -v[1], v[0], 0;
    Eigen::Matrix3d r = Eigen::Matrix3d::Identity() + nx + nx * nx * ((1 - c) / (s * s));

    Eigen::Vector3d camera_pose_ECEF_calculated = r * a;
    camera_pose_ECEF_calculated = camera_pose_ECEF_calculated.normalized() * a.norm() + Eigen::Vector3d(x1, y1, z1);
    double drone_lon, drone_lat, drone_alt;
    ECEF2GPS(camera_pose_ECEF_calculated[0],
             camera_pose_ECEF_calculated[1],
             camera_pose_ECEF_calculated[2],
             drone_lat, drone_lon, drone_alt);
    Eigen::Vector3d camera_pose_GPS_calculated(drone_lat, drone_lon, drone_alt);
    camera_trajectory_GPS_calculated_.push_back(camera_pose_GPS_calculated);

    // Calculate fire spot positions in GPS coordinates.
    for (const geometry_msgs::msg::Pose& fire_spot : fire_spots_msg->poses)
    {
      Eigen::Vector3d fire_spot_SLAM(
          fire_spot.position.x - camera_trajectory_SLAM_[origin_frame_index].pose.position.x,
          fire_spot.position.y - camera_trajectory_SLAM_[origin_frame_index].pose.position.y,
          fire_spot.position.z - camera_trajectory_SLAM_[origin_frame_index].pose.position.z);
      double fire_norm = fire_spot_SLAM.norm();
      Eigen::Vector3d fire_ecef = r * fire_spot_SLAM;
      fire_ecef = fire_ecef.normalized() * real_scales_.back() * fire_norm + Eigen::Vector3d(x1, y1, z1);
      // Temporary variables to receive GPS values.
      double temp_lat, temp_lon, temp_alt;
      ECEF2GPS(fire_ecef[0], fire_ecef[1], fire_ecef[2], temp_lat, temp_lon, temp_alt);
      geometry_msgs::msg::Pose pose;
      // Adjust with small offsets as in your original code.
      pose.position.x = temp_lat + 0.000035;
      pose.position.y = temp_lon - 0.00004;
      pose.position.z = temp_alt - 19;
      fire_spots_GPS_.poses.push_back(pose);
    }
    fire_spots_GPS_pub_->publish(fire_spots_GPS_);

    // Calculate the average of the last 600 fire spots.
    if (fire_spots_GPS_.poses.size() > 600 && camera_trajectory_GPS_calculated_.size() > 120)
    {
      double latitude  = 0;
      double longitude = 0;
      double altitude  = 0;
      for (size_t i = fire_spots_GPS_.poses.size() - 600; i < fire_spots_GPS_.poses.size(); i++)
      {
        latitude  += fire_spots_GPS_.poses[i].position.x;
        longitude += fire_spots_GPS_.poses[i].position.y;
        altitude  += fire_spots_GPS_.poses[i].position.z;
      }
      latitude  /= 600;
      longitude /= 600;
      altitude  /= 600;
      Eigen::Vector3d fire_spot_GPS_average(latitude, longitude, altitude);
      fire_spots_GPS_average_.push_back(fire_spot_GPS_average);
    }
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto geo_positioning_node = std::make_shared<GeoPositioning>();

  // Instead of ros::spinOnce in a loop, we use rclcpp::spin_some.
  while (rclcpp::ok())
  {
    rclcpp::spin_some(geo_positioning_node);
    if (geo_positioning_node->size() > 1700)
    {
      break;
    }
    // Clear the fire spots for the next iteration.
    geo_positioning_node->getFireSpotsGPS()->poses.clear();
  }

  // --- Plotting and data export using matplotlibcpp and std::ofstream ---

  std::vector<double> longitude, latitude;
  for (const auto& drone_GPS : *geo_positioning_node->getCameraPosesGPS())
  {
    longitude.push_back(drone_GPS.longitude);
    latitude.push_back(drone_GPS.latitude);
  }
  plt::plot(longitude, latitude, "bo");
  longitude.clear();
  latitude.clear();
  latitude.push_back(45.458047453);
  longitude.push_back(-73.932875451);
  latitude.push_back(45.458046062);
  longitude.push_back(-73.932882159);
  latitude.push_back(45.458050887);
  longitude.push_back(-73.932884099);
  latitude.push_back(45.458049803);
  longitude.push_back(-73.932890364);
  plt::plot(longitude, latitude, "ro");

  double latitude_avg  = 0;
  double longitude_avg = 0;
  for (size_t i = 0; i < latitude.size(); i++)
  {
    latitude_avg  += latitude[i];
    longitude_avg += longitude[i];
  }
  latitude_avg  /= latitude.size();
  longitude_avg /= longitude.size();

  longitude.clear();
  latitude.clear();
  int points_kept = 6000;
  for (size_t i = geo_positioning_node->getFireSpotsGPS()->poses.size() - points_kept;
       i < geo_positioning_node->getFireSpotsGPS()->poses.size(); i++)
  {
    longitude.push_back(geo_positioning_node->getFireSpotsGPS()->poses[i].position.y);
    latitude.push_back(geo_positioning_node->getFireSpotsGPS()->poses[i].position.x);
  }
  plt::plot(longitude, latitude, "go");
  longitude.clear();
  latitude.clear();
  for (const auto& drone_GPS : *geo_positioning_node->getCameraPosesGPSCalculated())
  {
    longitude.push_back(drone_GPS[1]);
    latitude.push_back(drone_GPS[0]);
  }
  plt::plot(longitude, latitude, "yo");
  plt::show();

  std::vector<double> ALEs, t;
  for (size_t i = 0; i < geo_positioning_node->getAverageFireSpotsGPS().size(); i++)
  {
    double latitude_diff  = geo_positioning_node->getAverageFireSpotsGPS()[i][0] - latitude_avg;
    double longitude_diff = geo_positioning_node->getAverageFireSpotsGPS()[i][1] - longitude_avg;
    double ATE = sqrt(latitude_diff * latitude_diff + longitude_diff * longitude_diff);
    ALEs.push_back(ATE);
    t.push_back(i);
  }
  plt::plot(t, ALEs);

  std::vector<double> ATEs;
  t.clear();
  int sizes_difference = geo_positioning_node->getCameraPosesGPS()->size() - geo_positioning_node->getCameraPosesGPSCalculated()->size();
  for (size_t i = 0; i < geo_positioning_node->getCameraPosesGPSCalculated()->size(); i++)
  {
    t.push_back(i);
    double latitude_diff  = geo_positioning_node->getCameraPosesGPS()->at(i + sizes_difference).latitude -
                            geo_positioning_node->getCameraPosesGPSCalculated()->at(i)[0];
    double longitude_diff = geo_positioning_node->getCameraPosesGPS()->at(i + sizes_difference).longitude -
                            geo_positioning_node->getCameraPosesGPSCalculated()->at(i)[1];
    double ATE = sqrt(latitude_diff * latitude_diff + longitude_diff * longitude_diff);
    ATEs.push_back(ATE);
  }
  plt::plot(t, ATEs);
  plt::show();

  std::ofstream file;
  file.open("01_fire_spots_GPS.txt");
  for (size_t i = geo_positioning_node->getFireSpotsGPS()->poses.size() - points_kept;
       i < geo_positioning_node->getFireSpotsGPS()->poses.size(); i++)
  {
    file << std::setprecision(std::numeric_limits<double>::digits10 + 1)
         << geo_positioning_node->getFireSpotsGPS()->poses[i].position.x << " "
         << geo_positioning_node->getFireSpotsGPS()->poses[i].position.y << " "
         << geo_positioning_node->getFireSpotsGPS()->poses[i].position.z << std::endl;
  }
  file.close();

  file.open("02_camera_poses_GPS.txt");
  for (const auto& camera_GPS : *geo_positioning_node->getCameraPosesGPS())
  {
    file << std::setprecision(std::numeric_limits<double>::digits10 + 1)
         << camera_GPS.latitude << " " << camera_GPS.longitude << " " << camera_GPS.altitude << std::endl;
  }
  file.close();

  file.open("03_camera_poses_GPS_calculated.txt");
  for (const auto& camera_GPS_calculated : *geo_positioning_node->getCameraPosesGPSCalculated())
  {
    file << std::setprecision(std::numeric_limits<double>::digits10 + 1)
         << camera_GPS_calculated[0] << " " << camera_GPS_calculated[1] << " " << camera_GPS_calculated[2] << std::endl;
  }
  file.close();

  LOG(INFO) << "The number of real scales: " << geo_positioning_node->getRealScales()->size() << ".";
  plt::hist(*geo_positioning_node->getRealScales(), 100);
  plt::show();

  file.open("04_real_scales.txt");
  for (double scale : *geo_positioning_node->getRealScales())
  {
    file << std::setprecision(std::numeric_limits<double>::digits10 + 1) << scale << std::endl;
  }
  file.close();

  file.open("05_ALE.txt");
  for (double ALE : ALEs)
  {
    file << std::setprecision(std::numeric_limits<double>::digits10 + 1) << ALE << std::endl;
  }
  file.close();

  file.open("06_ATE.txt");
  for (double ATE : ATEs)
  {
    file << std::setprecision(std::numeric_limits<double>::digits10 + 1) << ATE << std::endl;
  }
  file.close();

  file.open("07_average_fire_spots_GPS.txt");
  for (const auto& fire_spot_GPS_average : geo_positioning_node->getAverageFireSpotsGPS())
  {
    file << std::setprecision(std::numeric_limits<double>::digits10 + 1)
         << fire_spot_GPS_average[0] << " " << fire_spot_GPS_average[1] << " " << fire_spot_GPS_average[2] << std::endl;
  }
  file.close();

  double time_period = rclcpp::Time(geo_positioning_node->getCameraPosesGPS()->back().header.stamp).seconds() -
                       rclcpp::Time(geo_positioning_node->getCameraPosesGPS()->at(1000).header.stamp).seconds();
  LOG(INFO) << "The whole ATE time period: " << time_period << " seconds.";

  time_period = rclcpp::Time(geo_positioning_node->getCameraPosesGPS()->back().header.stamp).seconds() -
                rclcpp::Time(geo_positioning_node->getCameraPosesGPS()->at(1128).header.stamp).seconds();
  LOG(INFO) << "The whole ALE time period: " << time_period << " seconds.";

  rclcpp::shutdown();
  return 0;
}
