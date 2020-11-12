#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <cstdlib>
#include <unordered_map>

#include <sam_msgs/CircuitStatusStamped.h>
#include <sam_msgs/CircuitStatusStampedArray.h>
// #include <sam_msgs/KeyFloatTimedArray.h>
#include <diagnostic_msgs/DiagnosticArray.h>

// -------------------------------- CircuitStatusJoiner --------------------------------
class CircuitStatusJoiner {
  std::unordered_map<unsigned, sam_msgs::CircuitStatusStamped> circuit_registry;
  ros::Publisher CircuitStatusPublisher;
  ros::Subscriber CircuitStatusSubscriber;
  ros::Timer timer1;
  unsigned pub_freq;

  void callbackPublish(const ros::TimerEvent&)
  {
    // ROS_INFO("Callback 1 triggered");
    sam_msgs::CircuitStatusStampedArray msg;
    for(int i = 0; i < 100; i++)
    {
      if (circuit_registry.find(i) != circuit_registry.end())
        {
          msg.array.push_back(circuit_registry[i]);
        }
    }
    CircuitStatusPublisher.publish(msg);
  }

  void callbackSubscriber(const sam_msgs::CircuitStatusStamped& msg) {
    // ROS_INFO("New message received. id: %d", msg.circuit.circuit_id);
    circuit_registry[msg.circuit.circuit_id] = msg;
  }

  public:
  CircuitStatusJoiner(ros::NodeHandle *nh) 
    : pub_freq(2)
    {
    CircuitStatusPublisher = nh->advertise<sam_msgs::CircuitStatusStampedArray>("circuit_status_array", 10);    
    CircuitStatusSubscriber = nh->subscribe("circuit_status", 30, 
        &CircuitStatusJoiner::callbackSubscriber, this);

    timer1 = nh->createTimer(ros::Duration(1.0/pub_freq), &CircuitStatusJoiner::callbackPublish, this);
  }
};

// -------------------------------- SystemStatusMonitor --------------------------------
class SystemStatusMonitor {
  diagnostic_msgs::DiagnosticArray lastMessage;
  // std::list<rosgraph_msgs::Log> mainLogList;
  std::map<unsigned, sam_msgs::CircuitStatusStamped> circuit_registrsy;
  std::unordered_map<unsigned, sam_msgs::CircuitStatusStamped> circuit_registry;
  ros::Publisher DiagnosticsPublisher;
  ros::Subscriber DiagnosticsSubscriber;
  ros::Timer timer1;
  unsigned pub_freq;

  // void callbackPublish(const ros::TimerEvent&)
  // {
  //   // ROS_INFO("Callback 1 triggered");
  //   sam_msgs::CircuitStatusStampedArray msg;
  //   for(int i = 0; i < 100; i++)
  //   {
  //     if (circuit_registry.find(i) != circuit_registry.end())
  //       {
  //         msg.array.push_back(circuit_registry[i]);
  //       }
  //   }
  //   DiagnosticsPublisher.publish(msg);
  // }

  void callbackDiagnosticsSubscriber(const diagnostic_msgs::DiagnosticArray& msg) {
    // ROS_INFO("New message received. id: %d", msg.circuit.circuit_id);
    // circuit_registry[msg.circuit.circuit_id] = msg;
    if(strcmp(msg.status[0].name.c_str(), "jetson_stats board status") == 0)
    {
      // ROS_INFO("About Jetson :)");
      lastMessage = msg;
      DiagnosticsPublisher.publish(msg);
    }
  }

  public:
  SystemStatusMonitor(ros::NodeHandle *nh) 
    : pub_freq(2)
    {
    DiagnosticsPublisher = nh->advertise<diagnostic_msgs::DiagnosticArray>("jetson_diagnostics", 1);    
    DiagnosticsSubscriber = nh->subscribe("/diagnostics", 30, 
        &SystemStatusMonitor::callbackDiagnosticsSubscriber, this);

    // timer1 = nh->createTimer(ros::Duration(1.0/pub_freq), &SystemStatusMonitor::callbackPublish, this);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "message_joiner");

  ros::NodeHandle nh;

  // int pub_frequency;
  // ros::param::param<int>("pub_frequency", pub_frequency, 2);

  ROS_INFO("Message joiner started");

  CircuitStatusJoiner nc = CircuitStatusJoiner(&nh);
  SystemStatusMonitor ns = SystemStatusMonitor(&nh);

  ros::spin();

  return 0;
}