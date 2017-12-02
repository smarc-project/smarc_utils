#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>


class OdomTfBC{

public:
    OdomTfBC(ros::NodeHandle &nh): nh_(&nh){
        start_bc_ = false;

        nh_->param<std::string>((ros::this_node::getName() + "/gt_topic"), gt_topic_, "/pose_gt");
        nh_->param<std::string>((ros::this_node::getName() + "/parent_frame"), parent_frame_, "/world");
        nh_->param<std::string>((ros::this_node::getName() + "/child_frame"), child_frame_, "/odom");
        initial_pose_subs_ = nh_->subscribe(gt_topic_, 1, &OdomTfBC::getInitialPoseCB, this);
    }

    void broadcastOdomTf(){
        ros::Rate rate(30.0);
        while (ros::ok()){
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
                ROS_DEBUG("Tf world --> odom not being broadcasted");
                ros::spinOnce();
            }
            rate.sleep();
        }
    }

    void getInitialPoseCB(const nav_msgs::OdometryPtr &gt_pose){
        if(start_bc_ == false){
            initial_odom_ps_ = gt_pose;
        }
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
    std::string gt_topic_;
    ros::Subscriber initial_pose_subs_;
};



int main(int argc, char** argv){
  ros::init(argc, argv, "odom_tf_bc_node");
  ros::NodeHandle nh;

  OdomTfBC world_odom_bc = OdomTfBC(nh);
  world_odom_bc.broadcastOdomTf();


  return 0;
}
