#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class TwistFilter
{
public:
  TwistFilter()
  {
    // Initialize ROS node and subscribers/publishers
    nh_ = ros::NodeHandle("~");
    sub_ = nh_.subscribe("/spacenav/twist", 1, &TwistFilter::twistCallback, this);
    pub_ = nh_.advertise<geometry_msgs::Twist>("/filtered_twist", 1);
    zero_sent_ = false;
  }

  void twistCallback(const geometry_msgs::Twist::ConstPtr& msg)
  {
    // Filter zero values
    if (msg->linear.x == 0 && msg->linear.y == 0 && msg->linear.z == 0 &&
        msg->angular.x == 0 && msg->angular.y == 0 && msg->angular.z == 0)
    {
      // Check if zero value has already been sent
      if (zero_sent_)
        return;
      else
        zero_sent_ = true;
    }
    else
    {
      zero_sent_ = false;
    }

    // Publish the filtered twist
    pub_.publish(msg);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  bool zero_sent_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "twist_filter");
  TwistFilter filter;
  ros::spin();

  return 0;
}