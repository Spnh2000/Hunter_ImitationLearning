#include <algorithm>
#include <cmath>
#include <limits>

#include "hunter_msgs/msg/episode_event.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "ros_gz_interfaces/msg/contacts.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace hunter_control
{

// Detects episode-terminating crashes LIVE, so that:
//   * the recorder bags the fact rather than the ingredients for re-deriving it
//     offline (re-derivation would make the world SDF a permanent dependency of
//     every dataset -- change the derivation, silently relabel old data), and
//   * Step 4's DAgger loop can terminate and respawn, which cannot wait for a
//     label computed next week.
//
// Two independent signals, OR-ed. The Gazebo contact sensor is primary; a
// minimum-range check on /scan is the backstop, because contact sensors can
// miss a glancing wall scrape. Which one fired is recorded, so the redundancy
// can be shown to be earning its keep rather than assumed to.
class EpisodeMonitor : public rclcpp::Node
{
public:
  EpisodeMonitor()
  : rclcpp::Node("episode_monitor")
  {
    // Default matches dataset_spec.SCAN_CONTACT_M. The value is stored in the
    // HDF5 attrs because a threshold is part of what `terminals` MEANS -- data
    // labelled at 0.4 m and data labelled at 0.3 m are not the same data.
    scan_contact_m_ = declare_parameter<double>("scan_contact_m", 0.4);
    // Below the sensor minimum (0.15 m) readings are not trustworthy, so they
    // are ignored rather than treated as an infinitely close wall.
    scan_min_valid_m_ = declare_parameter<double>("scan_min_valid_m", 0.16);
    // A crash stays a crash: once latched, no further events until reset. A
    // robot resting against a wall would otherwise emit an event per tick.
    latch_ = declare_parameter<bool>("latch_until_reset", true);

    event_pub_ = create_publisher<hunter_msgs::msg::EpisodeEvent>(
      "/episode/event", rclcpp::QoS(10).reliable());

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      std::bind(&EpisodeMonitor::scanCallback, this, std::placeholders::_1));

    contact_sub_ = create_subscription<ros_gz_interfaces::msg::Contacts>(
      "/bumper_contacts", rclcpp::QoS(10),
      std::bind(&EpisodeMonitor::contactCallback, this, std::placeholders::_1));

    reset_srv_ = create_service<std_srvs::srv::Trigger>(
      "/episode/reset",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        fired_ = false;
        contact_count_ = 0;
        res->success = true;
        res->message = "episode monitor re-armed";
        RCLCPP_INFO(get_logger(), "re-armed");
      });

    RCLCPP_INFO(
      get_logger(), "episode_monitor ready: scan_contact=%.2f m, latch=%s",
      scan_contact_m_, latch_ ? "true" : "false");
  }

private:
  void contactCallback(const ros_gz_interfaces::msg::Contacts::SharedPtr msg)
  {
    // Level signal: how many contact points right now. Evaluated on the scan
    // tick so both signals are judged against the same instant.
    contact_count_ = static_cast<uint32_t>(msg->contacts.size());
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // Evaluate on the SCAN tick, not the contact tick: the dataset is at scan
    // rate, so a terminal flag that lands between scans has no row to attach to.
    double min_r = std::numeric_limits<double>::infinity();
    for (const float r : msg->ranges) {
      if (std::isfinite(r) && r >= scan_min_valid_m_ && r < min_r) {
        min_r = r;
      }
    }

    const bool by_contact = contact_count_ > 0;
    const bool by_scan = std::isfinite(min_r) && min_r < scan_contact_m_;

    uint8_t trigger = hunter_msgs::msg::EpisodeEvent::TRIGGER_NONE;
    if (by_contact && by_scan) {
      trigger = hunter_msgs::msg::EpisodeEvent::TRIGGER_BOTH;
    } else if (by_contact) {
      trigger = hunter_msgs::msg::EpisodeEvent::TRIGGER_CONTACT;
    } else if (by_scan) {
      trigger = hunter_msgs::msg::EpisodeEvent::TRIGGER_SCAN_PROXIMITY;
    }

    const bool crashed = (trigger != hunter_msgs::msg::EpisodeEvent::TRIGGER_NONE);
    const bool emit_terminal = crashed && !(latch_ && fired_);

    hunter_msgs::msg::EpisodeEvent ev;
    ev.header.stamp = msg->header.stamp;   // the scan's stamp, so the converter
    ev.header.frame_id = msg->header.frame_id;  // can join on it exactly
    ev.terminal = emit_terminal;
    ev.trigger = trigger;
    ev.min_scan_m = std::isfinite(min_r) ? static_cast<float>(min_r) : msg->range_max;
    ev.contact_count = contact_count_;
    event_pub_->publish(ev);

    if (emit_terminal) {
      fired_ = true;
      RCLCPP_WARN(
        get_logger(), "TERMINAL: trigger=%u min_scan=%.3f m contacts=%u",
        trigger, ev.min_scan_m, contact_count_);
    }
  }

  double scan_contact_m_;
  double scan_min_valid_m_;
  bool latch_;
  bool fired_{false};
  uint32_t contact_count_{0};

  rclcpp::Publisher<hunter_msgs::msg::EpisodeEvent>::SharedPtr event_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr contact_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
};

}  // namespace hunter_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hunter_control::EpisodeMonitor>());
  rclcpp::shutdown();
  return 0;
}
