#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>

#include <string>

enum ArbiterState
{
    NO_SELECTION,
    STOP,
    NAVIGATION,
    TELEOP
};

class CmdVelArbiterNode
{
 private:
    ros::NodeHandle _nh;
    ros::Publisher _outPub;
    ros::Subscriber _inSub;

    ros::ServiceServer switchToStopService;
    ros::ServiceServer switchToNavigationService;
    ros::ServiceServer switchToTeleopService;

    ArbiterState _arbiterState;
    std::string _navTopic;
    std::string _teleopTopic;

 public:
    explicit CmdVelArbiterNode(ros::NodeHandle *nh);

    void subscribeToNavigation();
    void subscribeToTeleop();
    bool switchToStop();
    bool switchToNavigation();
    bool switchToTeleop();

    bool switchToStopSrvCb(std_srvs::Empty::Request &req,
        std_srvs::Empty::Response &res);
    bool switchToNavigationSrvCb(std_srvs::Empty::Request &req,
        std_srvs::Empty::Response &res);
    bool switchToTeleopSrvCb(std_srvs::Empty::Request &req,
        std_srvs::Empty::Response &res);

    void inCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void publishStopToCmdVel();
    void run();
};
