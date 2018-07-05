#include <stdlib.h>
#include <string>
#include <time.h>
#include <vector>
#include <cmath>
#include <map>

#include <ros/ros.h>
#include <ros/console.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>



namespace Steering{

class SeekFlee{

private:

      ros::NodeHandle nh;
      ros::Publisher pub;
      ros::Subscriber sub;
//      ros::Subscriber vel_sub;
      ros::Subscriber r0_sub;
//      ros::Subscriber r1_sub;
      ros::Time goal_reached_time_;

      tf::TransformListener* tf_listener_;
      tf::StampedTransform player_transform_;
      tf::StampedTransform robot_transform_;
      tf::StampedTransform player_robot_transform_;

      nav_msgs::Odometry base_odom_;
      boost::mutex odom_lock_;
      bool on_simulation_;
      bool holonomic = true;
      bool turn_in_place_first = false;


public:

  double min_dist_to_robot;


  SeekFlee(){

      tf_listener_ = new tf::TransformListener();
      if (!nh.getParam("/planner_node/simulation", on_simulation_)){
          ROS_ERROR("BEHAVIOR MANAGER: could not read 'simulation' from rosparam!");
          exit(-1);
      }
      auto now = ros::Time(0);
      bool stopped();
      goal_reached_time_ = ros::Time::now();
      ros::Rate rate(10);
      if (on_simulation_){
            r0_sub=nh.subscribe<nav_msgs::Odometry>("robot_1/odom", 1, boost::bind(&SeekFlee::odomCallback, this, _1));
            pub=nh.advertise<geometry_msgs::Twist>("robot_0/cmd_vel", 50);
            if (!nh.getParam("/planner_node/min_dist_to_tower", min_dist_to_robot)){
              ROS_ERROR("BEHAVIOR MANAGER: could not read 'min_dist_to_tower' from rosparam!");
              exit(-1);
            }
      }
      else{
        exit(-1);
      }
  }

  ~SeekFlee(void){
    delete tf_listener_;
  }


  inline double sign(double n){
    return n < 0.0 ? -1.0 : 1.0;
  }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){

  	ROS_INFO("Seq: [%d]", msg->header.seq);
  	ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);
  	ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  	ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
  }

  bool getTransform(std::string target, std::string source, ros::Time &time, tf::StampedTransform &result){
      try{
          tf_listener_->waitForTransform(target.c_str(), time, source.c_str(), time, "/map", ros::Duration(0.20));
          tf_listener_->lookupTransform(target.c_str(), source.c_str(), time, result);
          return true;
      } catch (const std::exception& ex){
          ROS_WARN("ERROR when getting transform: %s",ex.what());
          return false;
      }
  }

  double headingDiff(double x, double y, double pt_x, double pt_y, double heading){
      double v1_x = x - pt_x;
      double v1_y = y - pt_y;
      double v2_x = cos(heading);
      double v2_y = sin(heading);

      double perp_dot = v1_x * v2_y - v1_y * v2_x;
      double dot = v1_x * v2_x + v1_y * v2_y;

      //get the signed angle
      double vector_angle = atan2(perp_dot, dot);

      return -1.0 * vector_angle;
  }


  bool TargetReached(){

        float x_dist = player_robot_transform_.getOrigin().x(), y_dist = player_robot_transform_.getOrigin().y();
        return (pow(x_dist, 2)+ pow(y_dist, 2) < pow(min_dist_to_robot, 2));
  }

  void publishVelocity(){

        ros::Rate rate(10);

        while(ros::ok()){
            ROS_DEBUG("SeekFlee: robot pose %f %f ==> %f", robot_transform_.getOrigin().x(), robot_transform_.getOrigin().y(), tf::getYaw(robot_transform_.getRotation()));
            ROS_DEBUG("SeekFlee: player pose %f %f ==> %f", player_transform_.getOrigin().x(), player_transform_.getOrigin().y(), tf::getYaw(player_transform_.getRotation()));

            geometry_msgs::Twist diff = diff2D(player_transform_, robot_transform_);
            ROS_DEBUG("SeekFlee: diff %f %f ==> %f", diff.linear.x, diff.linear.y, diff.angular.z);

            geometry_msgs::Twist limit_vel = limitTwist(diff);

            geometry_msgs::Twist test_vel = limit_vel;
            pub.publish(diff);

          rate.sleep();

        }

  }

  bool stopped(){
    //copy over the odometry information
    nav_msgs::Odometry base_odom;
    {
      boost::mutex::scoped_lock lock(odom_lock_);
      base_odom = base_odom_;
    }

    return fabs(base_odom.twist.twist.angular.z) <= 1e-4
      && fabs(base_odom.twist.twist.linear.x) <= 1e-4
      && fabs(base_odom.twist.twist.linear.y) <= 1e-4;
  }

  geometry_msgs::Twist diff2D(const tf::StampedTransform& transform1, const tf::StampedTransform& transform2)
  {
    geometry_msgs::Twist res;
    tf::Transform diff = transform2.inverse() * transform1;
    res.linear.x = diff.getOrigin().x();
    res.linear.y = diff.getOrigin().y();
    res.angular.z = tf::getYaw(diff.getRotation());

    if(true || (fabs(res.linear.x) <= 0.02 && fabs(res.linear.y) <= 0.02))
      return res;

      double yaw_diff = headingDiff(transform1.getOrigin().x(), transform1.getOrigin().y(),
          transform2.getOrigin().x(), transform2.getOrigin().y(), tf::getYaw(transform2.getRotation()));

      //we'll also check if we can move more effectively backwards
      double neg_yaw_diff = headingDiff(transform1.getOrigin().x(), transform1.getOrigin().y(),
          transform2.getOrigin().x(), transform2.getOrigin().y(), M_PI + tf::getYaw(transform2.getRotation()));

      //check if its faster to just back up
    /*  if(fabs(neg_yaw_diff) < fabs(yaw_diff)){
        ROS_DEBUG("Negative is better: %.2f", neg_yaw_diff);
        yaw_diff = neg_yaw_diff;
      }*/

      //compute the desired quaterion
      tf::Quaternion rot_diff = tf::createQuaternionFromYaw(yaw_diff);
      tf::Quaternion rot = transform2.getRotation() * rot_diff;
      tf::Transform new_transform = transform1;
      new_transform.setRotation(rot);

      diff = transform2.inverse() * new_transform;
      res.linear.x = diff.getOrigin().x();
      res.linear.y = diff.getOrigin().y();
      res.angular.z = tf::getYaw(diff.getRotation());
      return res;
    }


  geometry_msgs::Twist limitTwist(const geometry_msgs::Twist& twist)
  {
    geometry_msgs::Twist res = twist;
    res.linear.x *= 2.0;
    res.linear.y *= 2.0;
    res.angular.z *= 2.0;

    if (turn_in_place_first && fabs(twist.angular.z) > 0.17)
    {
      res.linear.x = 0;
      res.linear.y = 0;
      if (fabs(res.angular.z) < 0.0) res.angular.z = 0.0 * sign(res.angular.z);
      return res;
    }


    //velocity limits
    double lim = sqrt(res.linear.x * res.linear.x + res.linear.y * res.linear.y) / 0.9;
    if (lim > 1.0)
    {
      res.linear.x /= lim;
      res.linear.y /= lim;
    }
    if (fabs(res.angular.z) > 1.4) res.angular.z = 1.4 * sign(res.angular.z);
    if (fabs(res.angular.z) < 0.0) res.angular.z = 0.0 * sign(res.angular.z);

    if(fabs(res.linear.x) < 0.0 && fabs(res.linear.y) < 0.0){
      if (fabs(res.angular.z) < 0.0) res.angular.z = 0.0 * sign(res.angular.z);
    }

    ROS_DEBUG("Angular command %f", res.angular.z);
    return res;
  }

void publish_vel(){
          int i;
          bool a = true;
          bool b = true;
          bool c = true;
        //  ros::Rate rate(100);
          while(ros::ok()){
            auto now = ros::Time(0);

            b = getTransform("/map", "robot_0/base_link", now, robot_transform_);
            a = getTransform("/map", "robot_1/base_link", now, player_transform_);
            c = getTransform("/robot_0/base_link","robot_1/base_link", now, player_robot_transform_);
            /*
              this is desired velocity calculation....and published velocity is the one desired.
              since, earlier current_vel of robot_0 = 0 , therefore  desired_vel = nearly same as that of player.

              now i've got t

            */
            geometry_msgs::Twist res = diff2D(player_transform_,robot_transform_);
            geometry_msgs::Twist limit_vel = limitTwist(res);

            float x_dist = player_robot_transform_.getOrigin().x(), y_dist = player_robot_transform_.getOrigin().y();
//(pow(x_dist, 2)+ pow(y_dist, 2) < pow(min_dist_to_robot, 2))
// flee if radius decreases to limit, seek if radius increases than limit

            if(pow(x_dist, 2)+ pow(y_dist, 2) < pow(3, 2)){       //If inside perimeter : flee

                limit_vel.linear.x = limit_vel.linear.x * -1;
                limit_vel.linear.y = limit_vel.linear.y * -1;
                limit_vel.angular.z = limit_vel.angular.z * 0.5;
                pub.publish(limit_vel);
                }
            else{                                                 //else Seek
                limit_vel.linear.x = limit_vel.linear.x * 1;
                limit_vel.linear.y = limit_vel.linear.y * 1;
                limit_vel.angular.z = limit_vel.angular.z * 0.5;
                pub.publish(limit_vel);
              }

            }
          }

  };

}


int main(int argc, char** argv){
    ros::init(argc, argv, "steering_node");
    ros::NodeHandle nh;
    ros::Rate rate(10);
    Steering::SeekFlee seek_flee;

       while(ros::ok()){
         seek_flee.publish_vel();
        ros::spinOnce();
        // Wait until it's time for another iteration.
        rate.sleep();
    }
    ros::spin();
    return 0;
}
