#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>

enum ArbiterState
{
    NO_SELECTION,
    NAVIGATION,
    TELEOP
};

class CmdVelArbiter
{
 private:
    ros::NodeHandle _nh;
    ros::Publisher _outPub;
    ros::Subscriber _inSub;

    ros::ServiceServer switchToNavigationService;
    ros::ServiceServer switchToTeleopService;

    ArbiterState _arbiterState;
    std::string _navTopic;
    std::string _teleopTopic;

 public:
    explicit CmdVelArbiter(ros::NodeHandle *nh):
    _nh(*nh),
    _arbiterState(NO_SELECTION)
    {
        // Out topic
        _outPub = _nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

        // Read topic parameters
        _navTopic = "/cmd_vel_navigation";
        _teleopTopic = "/cmd_vel_teleop";

        std::string startingState;
        _nh.param<std::string>("starting_state", startingState, "teleop");

        if (startingState == "navigation")
        {
            switchToNavigation();
        }
        else if (startingState == "teleop")
        {
            switchToTeleop();
        }
        else
        {
            ROS_ERROR_STREAM(
                "Unsuported starting_state provided. Exiting...");
            exit(0);
        }

        // Switching services
        switchToNavigationService = _nh.advertiseService(
            "switch_to_navigation",
            &CmdVelArbiter::switchToNavigationSrvCb, this);
        switchToTeleopService = _nh.advertiseService(
            "switch_to_teleop",
            &CmdVelArbiter::switchToTeleopSrvCb, this);
    }

    void subscribeToNavigation()
    {
        _inSub = _nh.subscribe(
            _navTopic, 1, &CmdVelArbiter::inCallback, this);
        ROS_INFO_STREAM(
            "cmd_vel switched to navigation (topic: " << _navTopic << ")");
    }

    void subscribeToTeleop()
    {
        _inSub = _nh.subscribe(
            _teleopTopic, 1, &CmdVelArbiter::inCallback, this);
        ROS_INFO_STREAM(
            "cmd_vel switched to teleop (topic: " << _teleopTopic << ")");
    }

    bool switchToNavigation()
    {
        if (_arbiterState != NAVIGATION)
        {
            _inSub.shutdown();
            _arbiterState = NAVIGATION;
            subscribeToNavigation();
        }
        return true;
    }

    bool switchToTeleop()
    {
        if (_arbiterState != TELEOP)
        {
            _inSub.shutdown();
            _arbiterState = TELEOP;
            subscribeToTeleop();
        }
        return true;
    }

    bool switchToNavigationSrvCb(std_srvs::Empty::Request &req,
        std_srvs::Empty::Response &res)
    {
        return switchToNavigation();
    }

    bool switchToTeleopSrvCb(std_srvs::Empty::Request &req,
        std_srvs::Empty::Response &res)
    {
        return switchToTeleop();
    }

    void inCallback(const geometry_msgs::Twist::ConstPtr& msg)
    {
        _outPub.publish(msg);
    }

    void run()
    {
        ros::spin();
    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "cmd_vel_arbiter_node");
    ros::NodeHandle nh("~");
    CmdVelArbiter arbiter = CmdVelArbiter(&nh);
    arbiter.run();
}
