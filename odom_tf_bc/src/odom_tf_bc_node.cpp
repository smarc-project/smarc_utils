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
        tf::TransformBroadcaster br;
        tf::Transform transform;

        while (ros::ok()){
            if(start_bc_){
                // Construct transform world --> odom
                transform.setOrigin(tf::Vector3(
                                         initial_odom_ps_->pose.pose.position.x,
                                         initial_odom_ps_->pose.pose.position.y,
                                         initial_odom_ps_->pose.pose.position.z));
                tf::Quaternion q_auv;
                tf::quaternionMsgToTF(initial_odom_ps_->pose.pose.orientation, q_auv);
                transform.setRotation(q_auv);
                // Broadcast
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), parent_frame_, child_frame_));
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
