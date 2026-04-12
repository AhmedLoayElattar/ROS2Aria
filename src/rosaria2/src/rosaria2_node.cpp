/**
 * @brief ROS2 node that interfaces with mobile robot bases via AriaCoda.
 *
 * Port of the ROS1 rosaria package to ROS2 Jazzy.
 * Publishes odometry, bumpers, sonar, battery, and laser data.
 * Subscribes to cmd_vel.
 * Provides enable/disable motor services.
 * Uses ROS2 parameters for runtime configuration (replaces
 * dynamic_reconfigure).
 */

#include <Aria/ArGripper.h>
#include <Aria/ArRobotConfigPacketReader.h>
#include <Aria/Aria.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud_conversion.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "rosaria2/msg/bumper_state.hpp"

#include "rosaria2/laser_publisher.hpp"

#include <cmath>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

class RosAriaNode : public rclcpp::Node {
public:
  RosAriaNode();
  ~RosAriaNode() override;

  int Setup();
  void publish();

private:
  void cmdvel_cb(const geometry_msgs::msg::Twist::SharedPtr msg);
  void cmdvel_watchdog();
  void sonar_check_subscribers();
  void check_connection();

  // Services
  void
  enable_motors_cb(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                   std::shared_ptr<std_srvs::srv::Empty::Response> response);
  void disable_motors_cb(
      const std::shared_ptr<std_srvs::srv::Empty::Request> request,
      std::shared_ptr<std_srvs::srv::Empty::Response> response);

  // Gripper service callbacks
  void grip_open_cb(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                    std::shared_ptr<std_srvs::srv::Empty::Response>);
  void grip_close_cb(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                     std::shared_ptr<std_srvs::srv::Empty::Response>);
  void grip_lift_up_cb(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                       std::shared_ptr<std_srvs::srv::Empty::Response>);
  void grip_lift_down_cb(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                         std::shared_ptr<std_srvs::srv::Empty::Response>);
  void gripper_store_cb(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                        std::shared_ptr<std_srvs::srv::Empty::Response>);
  void gripper_deploy_cb(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                         std::shared_ptr<std_srvs::srv::Empty::Response>);

  // Parameter callback
  rcl_interfaces::msg::SetParametersResult
  on_parameter_change(const std::vector<rclcpp::Parameter> &parameters);

  void readParameters();

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pose_pub_;
  rclcpp::Publisher<rosaria2::msg::BumperState>::SharedPtr bumpers_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr sonar_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      sonar_pointcloud2_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr voltage_pub_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr recharge_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr state_of_charge_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motors_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr gripper_state_pub_;

  // Subscriber
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdvel_sub_;

  // Services
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr enable_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr disable_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr grip_open_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr grip_close_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr grip_lift_up_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr grip_lift_down_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr gripper_store_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr gripper_deploy_srv_;

  // Timers
  rclcpp::TimerBase::SharedPtr cmdvel_watchdog_timer_;
  rclcpp::TimerBase::SharedPtr sonar_timer_;
  rclcpp::TimerBase::SharedPtr reconnect_timer_;

  // TF
  std::shared_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster_;

  // Parameters
  std::string serial_port_;
  int serial_baud_;
  bool debug_aria_;
  std::string aria_log_filename_;
  bool publish_aria_lasers_;
  std::string frame_id_odom_;
  std::string frame_id_base_link_;
  std::string frame_id_bumper_;
  std::string frame_id_sonar_;
  double cmdvel_timeout_secs_;

  // Robot calibration
  int TicksMM_, DriftFactor_, RevCount_;

  // ARIA objects
  ArRobotConnector *conn_;
  ArLaserConnector *laser_connector_;
  ArRobot *robot_;
  ArFunctorC<RosAriaNode> my_publish_cb_;
  ArGripper *gripper_;

  // State
  rclcpp::Time veltime_;
  std_msgs::msg::Int8 recharge_state_;
  std_msgs::msg::Bool motors_state_;
  bool sonar_enabled_;
  bool publish_sonar_;
  bool publish_sonar_pointcloud2_;
  bool published_motors_state_;

  nav_msgs::msg::Odometry position_;
  rosaria2::msg::BumperState bumpers_;

  // Parameter callback handle
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      param_cb_handle_;
};

RosAriaNode::RosAriaNode()
    : Node("ROSaria2"), TicksMM_(-1), DriftFactor_(-99999), RevCount_(-1),
      conn_(nullptr), laser_connector_(nullptr), robot_(nullptr),
      my_publish_cb_(this, &RosAriaNode::publish), gripper_(nullptr),
      sonar_enabled_(false), publish_sonar_(false),
      publish_sonar_pointcloud2_(false), published_motors_state_(false) {
  // Declare parameters with defaults
  this->declare_parameter<std::string>(
      "port", "/dev/serial/by-id/usb-1a86_USB2.0-Ser_-if00-port0");
  this->declare_parameter<int>("baud", 0);
  this->declare_parameter<bool>("debug_aria", false);
  this->declare_parameter<std::string>("aria_log_filename", "Aria.log");
  this->declare_parameter<bool>("publish_aria_lasers", false);
  this->declare_parameter<std::string>("odom_frame", "odom");
  this->declare_parameter<std::string>("base_link_frame", "base_link");
  this->declare_parameter<std::string>("bumpers_frame", "bumpers");
  this->declare_parameter<std::string>("sonar_frame", "sonar");
  this->declare_parameter<double>("cmd_vel_timeout", 0.6);

  // Calibration parameters (set to 0 = use robot default)
  this->declare_parameter<int>("TicksMM", -1);
  this->declare_parameter<int>("DriftFactor", -99999);
  this->declare_parameter<int>("RevCount", -1);

  // Acceleration/deceleration parameters (in m/s^2 or rad/s^2, 0.0 = use robot
  // default)
  this->declare_parameter<double>("trans_accel", 0.0);
  this->declare_parameter<double>("trans_decel", 0.0);
  this->declare_parameter<double>("rot_accel", 0.0);
  this->declare_parameter<double>("rot_decel", 0.0);
  this->declare_parameter<double>("lat_accel", 0.0);
  this->declare_parameter<double>("lat_decel", 0.0);

  // Read parameters
  this->get_parameter("port", serial_port_);
  this->get_parameter("baud", serial_baud_);
  this->get_parameter("debug_aria", debug_aria_);
  this->get_parameter("aria_log_filename", aria_log_filename_);
  this->get_parameter("publish_aria_lasers", publish_aria_lasers_);
  this->get_parameter("odom_frame", frame_id_odom_);
  this->get_parameter("base_link_frame", frame_id_base_link_);
  this->get_parameter("bumpers_frame", frame_id_bumper_);
  this->get_parameter("sonar_frame", frame_id_sonar_);
  this->get_parameter("cmd_vel_timeout", cmdvel_timeout_secs_);

  RCLCPP_INFO(this->get_logger(), "RosAria2: port: [%s]", serial_port_.c_str());
  if (serial_baud_ != 0) {
    RCLCPP_INFO(this->get_logger(), "RosAria2: serial baud rate %d",
                serial_baud_);
  }

  // Create publishers
  pose_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("pose", 1000);
  bumpers_pub_ =
      this->create_publisher<rosaria2::msg::BumperState>("bumper_state", 1000);
  sonar_pub_ =
      this->create_publisher<sensor_msgs::msg::PointCloud>("sonar", 50);
  sonar_pointcloud2_pub_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("sonar_pointcloud2",
                                                            50);
  voltage_pub_ =
      this->create_publisher<std_msgs::msg::Float64>("battery_voltage", 1000);
  recharge_state_pub_ = this->create_publisher<std_msgs::msg::Int8>(
      "battery_recharge_state", rclcpp::QoS(5).transient_local());
  state_of_charge_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "battery_state_of_charge", 100);
  motors_state_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "motors_state", rclcpp::QoS(5).transient_local());

  recharge_state_.data = -2;
  motors_state_.data = false;

  // Services
  enable_srv_ = this->create_service<std_srvs::srv::Empty>(
      "enable_motors", std::bind(&RosAriaNode::enable_motors_cb, this,
                                 std::placeholders::_1, std::placeholders::_2));
  disable_srv_ = this->create_service<std_srvs::srv::Empty>(
      "disable_motors",
      std::bind(&RosAriaNode::disable_motors_cb, this, std::placeholders::_1,
                std::placeholders::_2));

  gripper_state_pub_ =
      this->create_publisher<std_msgs::msg::Int8>("gripper_state", 100);

  grip_open_srv_ = this->create_service<std_srvs::srv::Empty>(
      "grip_open", std::bind(&RosAriaNode::grip_open_cb, this,
                             std::placeholders::_1, std::placeholders::_2));
  grip_close_srv_ = this->create_service<std_srvs::srv::Empty>(
      "grip_close", std::bind(&RosAriaNode::grip_close_cb, this,
                              std::placeholders::_1, std::placeholders::_2));
  grip_lift_up_srv_ = this->create_service<std_srvs::srv::Empty>(
      "grip_lift_up", std::bind(&RosAriaNode::grip_lift_up_cb, this,
                                std::placeholders::_1, std::placeholders::_2));
  grip_lift_down_srv_ = this->create_service<std_srvs::srv::Empty>(
      "grip_lift_down",
      std::bind(&RosAriaNode::grip_lift_down_cb, this, std::placeholders::_1,
                std::placeholders::_2));
  gripper_store_srv_ = this->create_service<std_srvs::srv::Empty>(
      "gripper_store", std::bind(&RosAriaNode::gripper_store_cb, this,
                                 std::placeholders::_1, std::placeholders::_2));
  gripper_deploy_srv_ = this->create_service<std_srvs::srv::Empty>(
      "gripper_deploy",
      std::bind(&RosAriaNode::gripper_deploy_cb, this, std::placeholders::_1,
                std::placeholders::_2));

  veltime_ = this->now();

  // TF broadcaster
  odom_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

RosAriaNode::~RosAriaNode() {
  if (robot_) {
    robot_->disableMotors();
    robot_->disableSonar();
    robot_->stopRunning();
    robot_->waitForRunExit();
  }
  Aria::shutdown();
}

void RosAriaNode::readParameters() {
  robot_->lock();

  this->get_parameter("TicksMM", TicksMM_);
  if (TicksMM_ > 0) {
    RCLCPP_INFO(this->get_logger(),
                "Setting robot TicksMM from ROS Parameter: %d", TicksMM_);
    robot_->comInt(93, TicksMM_);
  } else {
    TicksMM_ = robot_->getOrigRobotConfig()->getTicksMM();
    RCLCPP_INFO(this->get_logger(), "This robot's TicksMM parameter: %d",
                TicksMM_);
  }

  this->get_parameter("DriftFactor", DriftFactor_);
  if (DriftFactor_ != -99999) {
    RCLCPP_INFO(this->get_logger(),
                "Setting robot DriftFactor from ROS Parameter: %d",
                DriftFactor_);
    robot_->comInt(89, DriftFactor_);
  } else {
    DriftFactor_ = robot_->getOrigRobotConfig()->getDriftFactor();
    RCLCPP_INFO(this->get_logger(), "This robot's DriftFactor parameter: %d",
                DriftFactor_);
  }

  this->get_parameter("RevCount", RevCount_);
  if (RevCount_ > 0) {
    RCLCPP_INFO(this->get_logger(),
                "Setting robot RevCount from ROS Parameter: %d", RevCount_);
    robot_->comInt(88, RevCount_);
  } else {
    RevCount_ = robot_->getOrigRobotConfig()->getRevCount();
    RCLCPP_INFO(this->get_logger(), "This robot's RevCount parameter: %d",
                RevCount_);
  }

  robot_->unlock();
}

rcl_interfaces::msg::SetParametersResult RosAriaNode::on_parameter_change(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  if (!robot_) {
    result.successful = false;
    result.reason = "Robot not connected";
    return result;
  }

  robot_->lock();

  for (const auto &param : parameters) {
    if (param.get_name() == "TicksMM" && param.as_int() > 0) {
      RCLCPP_INFO(this->get_logger(), "Setting TicksMM: %d -> %ld", TicksMM_,
                  param.as_int());
      TicksMM_ = static_cast<int>(param.as_int());
      robot_->comInt(93, TicksMM_);
    } else if (param.get_name() == "DriftFactor" && param.as_int() != -99999) {
      RCLCPP_INFO(this->get_logger(), "Setting DriftFactor: %d -> %ld",
                  DriftFactor_, param.as_int());
      DriftFactor_ = static_cast<int>(param.as_int());
      robot_->comInt(89, DriftFactor_);
    } else if (param.get_name() == "RevCount" && param.as_int() > 0) {
      RCLCPP_INFO(this->get_logger(), "Setting RevCount: %d -> %ld", RevCount_,
                  param.as_int());
      RevCount_ = static_cast<int>(param.as_int());
      robot_->comInt(88, RevCount_);
    } else if (param.get_name() == "trans_accel") {
      int value = static_cast<int>(param.as_double() * 1000);
      if (value > 0) {
        RCLCPP_INFO(this->get_logger(), "Setting TransAccel: %d", value);
        robot_->setTransAccel(value);
      }
    } else if (param.get_name() == "trans_decel") {
      int value = static_cast<int>(param.as_double() * 1000);
      if (value > 0) {
        RCLCPP_INFO(this->get_logger(), "Setting TransDecel: %d", value);
        robot_->setTransDecel(value);
      }
    } else if (param.get_name() == "rot_accel") {
      int value = static_cast<int>(param.as_double() * 180.0 / M_PI);
      if (value > 0) {
        RCLCPP_INFO(this->get_logger(), "Setting RotAccel: %d", value);
        robot_->setRotAccel(value);
      }
    } else if (param.get_name() == "rot_decel") {
      int value = static_cast<int>(param.as_double() * 180.0 / M_PI);
      if (value > 0) {
        RCLCPP_INFO(this->get_logger(), "Setting RotDecel: %d", value);
        robot_->setRotDecel(value);
      }
    } else if (param.get_name() == "lat_accel") {
      int value = static_cast<int>(param.as_double() * 1000);
      if (value > 0 && robot_->getAbsoluteMaxLatAccel() > 0) {
        RCLCPP_INFO(this->get_logger(), "Setting LatAccel: %d", value);
        robot_->setLatAccel(value);
      }
    } else if (param.get_name() == "lat_decel") {
      int value = static_cast<int>(param.as_double() * 1000);
      if (value > 0 && robot_->getAbsoluteMaxLatDecel() > 0) {
        RCLCPP_INFO(this->get_logger(), "Setting LatDecel: %d", value);
        robot_->setLatDecel(value);
      }
    }
  }

  robot_->unlock();
  return result;
}

int RosAriaNode::Setup() {
  robot_ = new ArRobot();
  ArArgumentBuilder *args = new ArArgumentBuilder();
  ArArgumentParser *argparser = new ArArgumentParser(args);
  argparser->loadDefaultArguments();

  // Parse port: if it contains ':', treat as hostname:tcpport
  size_t colon_pos = serial_port_.find(":");
  if (colon_pos != std::string::npos) {
    args->add("-remoteHost");
    args->add(serial_port_.substr(0, colon_pos).c_str());
    args->add("-remoteRobotTcpPort");
    args->add(serial_port_.substr(colon_pos + 1).c_str());
  } else {
    args->add("-robotPort %s", serial_port_.c_str());
  }

  if (serial_baud_ != 0) {
    args->add("-robotBaud %d", serial_baud_);
  }

  if (debug_aria_) {
    args->add("-robotLogPacketsReceived");
    args->add("-robotLogPacketsSent");
    args->add("-robotLogVelocitiesReceived");
    args->add("-robotLogMovementSent");
    args->add("-robotLogMovementReceived");
    ArLog::init(ArLog::File, ArLog::Verbose, aria_log_filename_.c_str(), true);
  }

  // Connect to the robot
  conn_ = new ArRobotConnector(argparser, robot_);
  if (!conn_->connectRobot()) {
    RCLCPP_ERROR(this->get_logger(),
                 "RosAria2: ARIA could not connect to robot! "
                 "(Check port parameter, permissions, or errors above)");
    return 1;
  }

  if (publish_aria_lasers_) {
    laser_connector_ = new ArLaserConnector(argparser, robot_, conn_);
  }

  // Load robot-specific parameters
  if (!Aria::parseArgs()) {
    RCLCPP_ERROR(this->get_logger(),
                 "RosAria2: ARIA error parsing startup parameters!");
    return 1;
  }

  readParameters();

  // Register parameter change callback (replaces dynamic_reconfigure)
  param_cb_handle_ = this->add_on_set_parameters_callback(std::bind(
      &RosAriaNode::on_parameter_change, this, std::placeholders::_1));

  // Enable motors, disable sonar by default
  robot_->enableMotors();
  robot_->disableSonar();

  // Register ARIA sensor interpretation callback
  robot_->addSensorInterpTask("ROSPublishingTask", 100, &my_publish_cb_);

  // Initialize bumpers
  bumpers_.front_bumpers.resize(robot_->getNumFrontBumpers(), false);
  bumpers_.rear_bumpers.resize(robot_->getNumRearBumpers(), false);

  // Initialize Gripper
  gripper_ = new ArGripper(robot_);

  // Run ArRobot background processing thread
  robot_->runAsync(true);

  // Connect to lasers if enabled
  if (publish_aria_lasers_) {
    RCLCPP_INFO(
        this->get_logger(),
        "Connecting to laser(s) configured in ARIA parameter file(s)...");
    if (!laser_connector_->connectLasers()) {
      RCLCPP_FATAL(this->get_logger(), "Error connecting to laser(s)...");
      return 1;
    }

    robot_->lock();
    const std::map<int, ArLaser *> *lasers = robot_->getLaserMap();
    RCLCPP_INFO(this->get_logger(), "There are %lu connected lasers",
                lasers->size());
    for (auto i = lasers->begin(); i != lasers->end(); ++i) {
      ArLaser *l = i->second;
      int ln = i->first;
      std::string tfname("laser");
      if (lasers->size() > 1 || ln > 1) {
        tfname += std::to_string(ln);
      }
      tfname += "_frame";
      RCLCPP_INFO(
          this->get_logger(),
          "Creating publisher for laser #%d named %s with tf frame name %s", ln,
          l->getName(), tfname.c_str());
      new LaserPublisher(l, shared_from_this(), true, tfname);
    }
    robot_->unlock();
    RCLCPP_INFO(this->get_logger(), "Done creating laser publishers");
  }

  // Subscribe to cmd_vel
  cmdvel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 1,
      std::bind(&RosAriaNode::cmdvel_cb, this, std::placeholders::_1));

  // cmd_vel watchdog timer
  if (cmdvel_timeout_secs_ > 0.0) {
    cmdvel_watchdog_timer_ =
        this->create_wall_timer(std::chrono::milliseconds(100),
                                std::bind(&RosAriaNode::cmdvel_watchdog, this));
  }

  // Timer to periodically check sonar subscriber count
  sonar_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&RosAriaNode::sonar_check_subscribers, this));

  // Timer to periodically check connection
  reconnect_timer_ = this->create_wall_timer(
      std::chrono::seconds(1), std::bind(&RosAriaNode::check_connection, this));

  RCLCPP_INFO(this->get_logger(), "RosAria2: Setup complete");
  return 0;
}

void RosAriaNode::check_connection() {
  if (robot_ && !robot_->isConnected()) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Robot connection lost! Attempting to reconnect...");

    robot_->stopRunning();
    robot_->waitForRunExit();

    if (conn_->connectRobot()) {
      RCLCPP_INFO(this->get_logger(), "Reconnected to robot.");

      readParameters();
      robot_->enableMotors();
      if (sonar_enabled_) {
        robot_->enableSonar();
      } else {
        robot_->disableSonar();
      }

      robot_->runAsync(true);
    }
  }
}

void RosAriaNode::sonar_check_subscribers() {
  bool want_sonar = (sonar_pub_->get_subscription_count() > 0);
  bool want_pc2 = (sonar_pointcloud2_pub_->get_subscription_count() > 0);

  if ((want_sonar || want_pc2) && !sonar_enabled_) {
    robot_->lock();
    robot_->enableSonar();
    robot_->unlock();
    sonar_enabled_ = true;
  } else if (!want_sonar && !want_pc2 && sonar_enabled_) {
    robot_->lock();
    robot_->disableSonar();
    robot_->unlock();
    sonar_enabled_ = false;
  }
  publish_sonar_ = want_sonar;
  publish_sonar_pointcloud2_ = want_pc2;
}

void RosAriaNode::publish() {
  // Called from ArRobot background thread via SensorInterpTask
  ArPose pos = robot_->getPose();

  // Odometry
  tf2::Quaternion q;
  q.setRPY(0, 0, pos.getTh() * M_PI / 180.0);

  position_.pose.pose.position.x = pos.getX() / 1000.0;
  position_.pose.pose.position.y = pos.getY() / 1000.0;
  position_.pose.pose.position.z = 0.0;
  position_.pose.pose.orientation = tf2::toMsg(q);

  position_.twist.twist.linear.x = robot_->getVel() / 1000.0;
  position_.twist.twist.linear.y = robot_->getLatVel() / 1000.0;
  position_.twist.twist.angular.z = robot_->getRotVel() * M_PI / 180.0;

  position_.header.frame_id = frame_id_odom_;
  position_.child_frame_id = frame_id_base_link_;
  position_.header.stamp = this->now();
  pose_pub_->publish(position_);

  // TF: odom -> base_link
  geometry_msgs::msg::TransformStamped odom_trans;
  odom_trans.header.stamp = this->now();
  odom_trans.header.frame_id = frame_id_odom_;
  odom_trans.child_frame_id = frame_id_base_link_;
  odom_trans.transform.translation.x = pos.getX() / 1000.0;
  odom_trans.transform.translation.y = pos.getY() / 1000.0;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = tf2::toMsg(q);
  odom_broadcaster_->sendTransform(odom_trans);

  // Bumpers
  int stall = robot_->getStallValue();
  unsigned char front_bumpers = static_cast<unsigned char>(stall >> 8);
  unsigned char rear_bumpers = static_cast<unsigned char>(stall);

  bumpers_.header.frame_id = frame_id_bumper_;
  bumpers_.header.stamp = this->now();

  for (unsigned int i = 0; i < robot_->getNumFrontBumpers(); i++) {
    bumpers_.front_bumpers[i] = (front_bumpers & (1 << (i + 1))) != 0;
  }
  unsigned int num_rear = robot_->getNumRearBumpers();
  for (unsigned int i = 0; i < num_rear; i++) {
    bumpers_.rear_bumpers[i] = (rear_bumpers & (1 << (num_rear - i))) != 0;
  }
  bumpers_pub_->publish(bumpers_);

  // Battery voltage
  std_msgs::msg::Float64 battery_voltage;
  battery_voltage.data = robot_->getRealBatteryVoltageNow();
  voltage_pub_->publish(battery_voltage);

  // State of charge
  if (robot_->haveStateOfCharge()) {
    std_msgs::msg::Float32 soc;
    soc.data = robot_->getStateOfCharge() / 100.0f;
    state_of_charge_pub_->publish(soc);
  }

  // Recharge state
  char s = robot_->getChargeState();
  if (s != recharge_state_.data) {
    RCLCPP_INFO(this->get_logger(), "Publishing new recharge state %d.", s);
    recharge_state_.data = s;
    recharge_state_pub_->publish(recharge_state_);
  }

  // Motors state
  bool e = robot_->areMotorsEnabled();
  if (e != motors_state_.data || !published_motors_state_) {
    RCLCPP_INFO(this->get_logger(), "Publishing new motors state %d.", e);
    motors_state_.data = e;
    motors_state_pub_->publish(motors_state_);
    published_motors_state_ = true;
  }

  // Gripper state
  if (gripper_) {
    std_msgs::msg::Int8 g_state;
    g_state.data = gripper_->getGripState();
    gripper_state_pub_->publish(g_state);
  }

  // Sonar
  if (publish_sonar_ || publish_sonar_pointcloud2_) {
    sensor_msgs::msg::PointCloud cloud;
    cloud.header.stamp = this->now();
    cloud.header.frame_id = frame_id_sonar_;

    for (int i = 0; i < robot_->getNumSonar(); i++) {
      ArSensorReading *reading = robot_->getSonarReading(i);
      if (!reading) {
        RCLCPP_WARN(this->get_logger(), "Did not receive a sonar reading.");
        continue;
      }
      geometry_msgs::msg::Point32 p;
      p.x = reading->getLocalX() / 1000.0f;
      p.y = reading->getLocalY() / 1000.0f;
      p.z = 0.0f;
      cloud.points.push_back(p);
    }

    if (publish_sonar_pointcloud2_) {
      sensor_msgs::msg::PointCloud2 cloud2;
      sensor_msgs::convertPointCloudToPointCloud2(cloud, cloud2);
      sonar_pointcloud2_pub_->publish(cloud2);
    }

    if (publish_sonar_) {
      sonar_pub_->publish(cloud);
    }
  }
}

void RosAriaNode::enable_motors_cb(
    const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/) {
  RCLCPP_INFO(this->get_logger(), "Enable motors request.");
  robot_->lock();
  if (robot_->isEStopPressed()) {
    RCLCPP_WARN(this->get_logger(), "Enable motors requested, but E-Stop is "
                                    "pressed. Motors will not enable.");
  }
  robot_->enableMotors();
  robot_->unlock();
}

void RosAriaNode::disable_motors_cb(
    const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/) {
  RCLCPP_INFO(this->get_logger(), "Disable motors request.");
  robot_->lock();
  robot_->disableMotors();
  robot_->unlock();
}

void RosAriaNode::grip_open_cb(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>) {
  RCLCPP_INFO(this->get_logger(), "Gripper open request");
  if (gripper_)
    gripper_->gripOpen();
}
void RosAriaNode::grip_close_cb(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>) {
  RCLCPP_INFO(this->get_logger(), "Gripper close request");
  if (gripper_)
    gripper_->gripClose();
}
void RosAriaNode::grip_lift_up_cb(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>) {
  RCLCPP_INFO(this->get_logger(), "Gripper lift up request");
  if (gripper_)
    gripper_->liftUp();
}
void RosAriaNode::grip_lift_down_cb(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>) {
  RCLCPP_INFO(this->get_logger(), "Gripper lift down request");
  if (gripper_)
    gripper_->liftDown();
}
void RosAriaNode::gripper_store_cb(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>) {
  RCLCPP_INFO(this->get_logger(), "Gripper store request");
  if (gripper_)
    gripper_->gripperStore();
}
void RosAriaNode::gripper_deploy_cb(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>) {
  RCLCPP_INFO(this->get_logger(), "Gripper deploy request");
  if (gripper_)
    gripper_->gripperDeploy();
}

void RosAriaNode::cmdvel_cb(const geometry_msgs::msg::Twist::SharedPtr msg) {
  veltime_ = this->now();
  RCLCPP_INFO(this->get_logger(), "New speed: [%.2f, %.2f] (%.3f)",
              msg->linear.x * 1e3, msg->angular.z, veltime_.seconds());

  robot_->lock();
  robot_->setVel(msg->linear.x * 1e3);
  if (robot_->hasLatVel()) {
    robot_->setLatVel(msg->linear.y * 1e3);
  }
  robot_->setRotVel(msg->angular.z * 180.0 / M_PI);
  robot_->unlock();
}

void RosAriaNode::cmdvel_watchdog() {
  if ((this->now() - veltime_).seconds() > cmdvel_timeout_secs_) {
    robot_->lock();
    robot_->setVel(0.0);
    if (robot_->hasLatVel()) {
      robot_->setLatVel(0.0);
    }
    robot_->setRotVel(0.0);
    robot_->unlock();
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  Aria::init();

  auto node = std::make_shared<RosAriaNode>();

  if (node->Setup() != 0) {
    RCLCPP_FATAL(node->get_logger(), "RosAria2: Node setup failed...");
    rclcpp::shutdown();
    return -1;
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
