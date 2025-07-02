#pragma once

#include "behaviortree_cpp/decorator_node.h"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

#include "rclcpp/rclcpp.hpp"

#include <algorithm>
#include <chrono>
#include <ctime> 
#include "rebet/system_attribute_value.hpp"
#include "rebet/qr_node.hpp"

/**
 * @brief The QRNode is used to constrain the action nodes it decorates.
 */


using namespace BT;
using namespace rebet;
using LaserScan = sensor_msgs::msg::LaserScan;
using Range = sensor_msgs::msg::Range;
using BatteryState = sensor_msgs::msg::BatteryState;
using KeyValue = diagnostic_msgs::msg::KeyValue;

class NoObjectsNearbyQR : public TaskLevelQR
{
  public:
    NoObjectsNearbyQR(const std::string& name, const NodeConfig& config) : TaskLevelQR(name, config, QualityAttribute::Safety)
    {
      _last_timestamp = BT::Timestamp();
      getInput(IN_THRESHOLD, _proximity_threshold);
    }

    //Every behavior tree node needs to define this static method.
    static PortsList providedPorts()
    {
      PortsList base_ports = TaskLevelQR::providedPorts();

      PortsList child_ports =  {
              InputPort<LaserScan>(IN_LASER_SCAN,"The LaserScan message origination from the robot's LIDAR"),
              InputPort<float>(IN_THRESHOLD,"How close is too close for the requirement"),
      };

      child_ports.merge(base_ports);

      return child_ports;
    }

    virtual void calculate_measure() override
    {
      if(auto timestamp = getInputStamped(IN_LASER_SCAN,_laser_msg))
      {
        
        if(timestamp.value().seq > _last_timestamp.seq)
        {
          float laser_min = 0.30; //Mirte's lidar can see itself.
          float laser_max = _laser_msg.range_max;

          float nearest_object = laser_max*2; //Anything above laser_max should work

          for (float const & laser_dist : _laser_msg.ranges)
          {
            if(laser_dist < laser_max && laser_dist > laser_min)
            {
              nearest_object = (laser_dist < nearest_object) ? laser_dist : nearest_object;
            }
          }

          float fitted_nearest = (nearest_object - laser_min) / (laser_max - laser_min);
          _metric = std::clamp(fitted_nearest,0.0f,1.0f);
          output_metric();

          
          std::string status = _metric < _proximity_threshold ? "VIOLATED" : "OK"; 

          setOutput(QR_STATUS,status); //This output port is defined in the parent class.
        }
      }

    }
    private:
      static constexpr const char* IN_LASER_SCAN = "laserScan";
      static constexpr const char* IN_THRESHOLD = "nearbyPercentage";
      BT::Timestamp _last_timestamp;

      LaserScan _laser_msg;
      float _proximity_threshold;
};


class InTheWayQR : public TaskLevelQR
{
  public:
    InTheWayQR(const std::string& name, const NodeConfig& config) : TaskLevelQR(name, config, QualityAttribute::Safety)
    {
      _last_timestamp = BT::Timestamp();
    }

    //Every behavior tree node needs to define this static method.
    static PortsList providedPorts()
    {
      PortsList base_ports = TaskLevelQR::providedPorts();

      PortsList child_ports =  {
              InputPort<Range>(IN_LEFT_RANGE,"The LaserScan message origination from the robot's LIDAR"),
              InputPort<Range>(IN_RIGHT_RANGE,"The LaserScan message origination from the robot's LIDAR"),
      };

      child_ports.merge(base_ports);

      return child_ports;
    }

    virtual void calculate_measure() override
    {
      if(auto timestamp = getInputStamped(IN_LEFT_RANGE,_rear_left_msg))
      {
        getInput(IN_RIGHT_RANGE,_rear_right_msg);
        if(timestamp.value().seq > _last_timestamp.seq)
        {
          std::cout << "InTheWayQR: " << _rear_left_msg.min_range << " " << _rear_right_msg.min_range << std::endl;
          
          float left_distance = _rear_left_msg.range;
          float right_distance = _rear_right_msg.range;

          // Define a threshold for determining if the robot is in the way
          float obstacle_threshold = 0.5; // Example threshold in meters

          // Check if either rear sensor detects an object within the threshold
          bool in_the_way = (left_distance < obstacle_threshold) || (right_distance < obstacle_threshold);

          // Update the metric and status based on the detection
          _metric = in_the_way ? 0.0f : 1.0f; // 0.0 if in the way, 1.0 otherwise
          output_metric();

          std::string status = in_the_way ? "VIOLATED" : "OK";
          setOutput(QR_STATUS, status); // This output port is defined in the parent class.

          // Insert your logic here to determine if the robot is the way.
        }

      }
    }
    private:
      static constexpr const char* IN_LEFT_RANGE = "leftRearRange";
      static constexpr const char* IN_RIGHT_RANGE = "rightRearRange";
      BT::Timestamp _last_timestamp;

      Range _rear_right_msg;
      Range _rear_left_msg;

};

class CPULimitQR : public TaskLevelQR
{
  public:
    CPULimitQR(const std::string& name, const NodeConfig& config) : TaskLevelQR(name, config, QualityAttribute::Performance)
    {
      _last_timestamp = BT::Timestamp();
    }

    //Every behavior tree node needs to define this static method.
    static PortsList providedPorts()
    {
      PortsList base_ports = TaskLevelQR::providedPorts();

      PortsList child_ports =  {
              InputPort<KeyValue>(IN_CPU,"A key value pair which gives the average CPU load over the past minute"),
              InputPort<BatteryState>(IN_BATTERY,"A BatteryState message which gives the current battery level and power details of the robot"),
              InputPort<float>(IN_THRESHOLD,"The load limit"),
      };

      child_ports.merge(base_ports);

      return child_ports;
    }

    virtual void calculate_measure() override
    {
      getInput(IN_CPU, cpu_msg);
      getInput(IN_BATTERY, battery_msg);

      std::cout << "CPULimitQR: CPU load is " << cpu_msg.value << std::endl;
      float cpu_load = std::stof(cpu_msg.value); // Convert CPU load from string to float
      float battery_percentage = battery_msg.percentage; // Get battery percentage
      float cpu_threshold = 0.2f;

      getInput(IN_THRESHOLD, cpu_threshold); // Retrieve the CPU load threshold

      // Check if the CPU load exceeds the threshold
      bool cpu_overloaded = cpu_load > cpu_threshold;

      // Update the metric based on CPU load
      _metric = cpu_overloaded ? 0.0f : 1.0f; // 0.0 if overloaded, 1.0 otherwise
      output_metric();

      // Determine the status based on CPU load
      std::string status = cpu_overloaded ? "VIOLATED" : "OK";
      setOutput(QR_STATUS, status); // This output port is defined in the parent class.
    }
    private:
      static constexpr const char* IN_BATTERY = "batteryState";
      static constexpr const char* IN_CPU = "cpuMessage";
      static constexpr const char* IN_THRESHOLD = "cpuPercentage";
      static constexpr const char* IN_WATT_THRESHOLD = "wattThreshold";
      BT::Timestamp _last_timestamp;

      KeyValue cpu_msg;
      BatteryState battery_msg;

};