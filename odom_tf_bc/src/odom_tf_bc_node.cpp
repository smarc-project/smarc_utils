#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>


class OdomTfBC{

public:
    OdomTfBC(ros::NodeHandle &nh): nh_(&nh){
        std::string gt_topic;
        start_bc_ = false;

        nh_->param<std::string>((ros::this_node::getName() + "/gt_topic"), gt_topic, "/gt_pose");
        nh_->param<std::string>((ros::this_node::getName() + "/parent_frame"), parent_frame_, "/world");
        nh_->param<std::string>((ros::this_node::getName() + "/child_frame"), child_frame_, "/odom");
        ros::Subscriber initial_pose_subs = nh.subscribe(gt_topic, 1, &OdomTfBC::getInitialPoseCB, this);
    }

    void broadcastOdomTf(){
        ros::Rate rate(30.0);
        while (nh_->ok()){
            if(start_bc_){
                transform_.setOrigin(tf::Vector3(
                                         initial_odom_ps_->pose.pose.position.x,
                                         initial_odom_ps_->pose.pose.position.y,
                                         initial_odom_ps_->pose.pose.position.z));
                transform_.setRotation(tf::Quaternion(initial_odom_ps_->pose.pose.orientation.x,
                                                      initial_odom_ps_->pose.pose.orientation.y,
                                                      initial_odom_ps_->pose.pose.orientation.z,
                                                      initial_odom_ps_->pose.pose.orientation.w));
                br_.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), parent_frame_, child_frame_));
            }
            else{
                ros::spinOnce();
            }
            rate.sleep();
        }
    }

    void getInitialPoseCB(const nav_msgs::OdometryPtr &gt_pose){
        initial_odom_ps_ = gt_pose;
        start_bc_ = true;
    }

private:
    bool start_bc_;
    std::string parent_frame_;
    std::string child_frame_;
    nav_msgs::OdometryPtr initial_odom_ps_;
    ros::NodeHandle* nh_;
    tf::TransformBroadcaster br_;
    tf::Transform transform_;
};



int main(int argc, char** argv){
  ros::init(argc, argv, "odom_tf_bc_node");
  ros::NodeHandle nh;

  OdomTfBC world_odom_bc = OdomTfBC(nh);
  world_odom_bc.broadcastOdomTf();


  return 0;
}
