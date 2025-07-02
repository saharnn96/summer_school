#pragma once

#include "rclcpp/rclcpp.hpp"

#include "builtin_interfaces/msg/time.hpp"
#include <algorithm>
#include <chrono>
#include <ctime> 
#include "rebet/system_attribute_value.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "rebet_school/adaptations.hpp"
#include "rebet/rebet_utilities.hpp"

class SimpleAdaptMaxVelocity : public AdaptNavToPose
{
  public:

    SimpleAdaptMaxVelocity(const std::string& name, const NodeConfig& config) : AdaptNavToPose(name, config)
    {
    }

    static PortsList providedPorts()
    {
      PortsList base_ports = AdaptNavToPose::providedPorts();

      PortsList child_ports =  {
        InputPort<std::string>(NEARBY_IN,"status of nearby QR, whether violated or not"),
      };
      child_ports.merge(base_ports);

      return child_ports;
    }

    // When this method returns true, adaptations will be performed.
    bool evaluate_condition() override
    {

        if(!AdaptNavToPose::evaluate_condition()) {
            return false;
        }
        std::string intheway_status;
        double proximity_metric;

        if (!getInput(INTHEWAY_IN, intheway_status) || !getInput(PROXIMITY_IN, proximity_metric)) {
          return false;
        }

        // Adaptation plan based on the QR statuses and proximity metric
        if (nearby_status == "VIOLATED" && intheway_status == "VIOLATED") {
          if (proximity_metric < 50.0) {
            // If both QRs are violated and the proximity is less than 50%, decrease max velocity
            return decrease_max_velocity();
          } else {
            // If both QRs are violated but proximity is greater than or equal to 50%, maintain current velocity
            return false;
          }
        } else if (nearby_status == "VIOLATED") {
          // If only Nearby QR is violated, decrease max velocity
          return decrease_max_velocity();
        } else if (intheway_status == "VIOLATED") {
          // If only InTheWay QR is violated, decrease max velocity
          return decrease_max_velocity();
        } else {
          // If neither QR is violated, increase max velocity
          return increase_max_velocity();
        }
        // std::string nearby_status;
        // if(!getInput(NEARBY_IN, nearby_status)) {
        //     return false;
        // }
        
        // if(nearby_status == "VIOLATED"){
        //     // This returns true if it is possible to lower the max velocity.
        //     return decrease_max_velocity();
        // }
       
        // // If the nearby QRs status is not violated, we can increase the velocity.
        // return increase_max_velocity();
    }

    static constexpr const char* NEARBY_IN = "in_nearby_qr";
};

class ComplexAdaptMaxVelocity : public AdaptNavToPose
{
  public:

    ComplexAdaptMaxVelocity(const std::string& name, const NodeConfig& config) : AdaptNavToPose(name, config)
    {
    }

    static PortsList providedPorts()
    {
      PortsList base_ports = AdaptNavToPose::providedPorts();

      PortsList child_ports =  {
        InputPort<std::string>(NEARBY_IN,"status of nearby QR, whether violated or not"),
        InputPort<std::string>(INTHEWAY_IN, "status of in the way QR, whether violated or not"),
        InputPort<double>(PROXIMITY_IN, "how close as a percentage of the lidar range is the nearest object"),
      };
      child_ports.merge(base_ports);

      return child_ports;
    }

    // When this method returns true, adaptations will be performed.
    bool evaluate_condition() override
    {
      RCLCPP_INFO(node_->get_logger(), "ComplexAdaptMaxVelocity evaluate_condition called");
      if(!AdaptNavToPose::evaluate_condition()) {
          return false;
      }

      std::string nearby_status;
      if(!getInput(NEARBY_IN, nearby_status)) {
          return false;
      }


        // Insert more complex logic here which considers multiple quality requirements.
        
    }

    static constexpr const char* NEARBY_IN = "in_nearby_qr";
    static constexpr const char* INTHEWAY_IN = "in_intheway_qr";
    static constexpr const char* PROXIMITY_IN = "in_proximity";

};

class AdaptPlanner : public AdaptNavToPose
{
  public:

    AdaptPlanner(const std::string& name, const NodeConfig& config) : AdaptNavToPose(name, config)
    {
      // Set the default path planner to NavFn since it uses less CPU.
      change_path_planner(PathPlanner::NavFn);
    }

    static PortsList providedPorts()
    {
      PortsList base_ports = AdaptNavToPose::providedPorts();

      PortsList child_ports =  {
        InputPort<std::string>(CPU_IN,"status of CPU QR, whether violated or not"),
      };
      child_ports.merge(base_ports);

      return child_ports;
    }

    // When this method returns true, adaptations will be performed.
    bool evaluate_condition() override
    {
        if(!AdaptNavToPose::evaluate_condition()) {
            return false;
        }


        //These are your adaptation options.
        //return change_path_planner(PathPlanner::SMAC);
        //return change_path_planner(PathPlanner::NavFn);

        //SMAC uses more CPU than NavFn.
        return false;
    }

    static constexpr const char* CPU_IN = "in_cpu_limit_qr";
};