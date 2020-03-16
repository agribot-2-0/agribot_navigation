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
        CmdVelArbiter(ros::NodeHandle *nh):
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
                subscribeToNavigation();
            }
            else if (startingState == "teleop")
            {
                subscribeToTeleop();
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
                &CmdVelArbiter::switchToNavigation, this);
            switchToTeleopService = _nh.advertiseService(
                "switch_to_teleop", &CmdVelArbiter::switchToTeleop, this);
        }

        void subscribeToNavigation()
        {
            _inSub = _nh.subscribe(
                _navTopic, 1, &CmdVelArbiter::inCallback, this);
            ROS_INFO_STREAM(
                "cmd_vel switched to navigation (topic: " << _navTopic << ")");
            _arbiterState = NAVIGATION;
        }

        void subscribeToTeleop()
        {
            _inSub = _nh.subscribe(
                _teleopTopic, 1, &CmdVelArbiter::inCallback, this);
            ROS_INFO_STREAM(
                "cmd_vel switched to teleop (topic: " << _teleopTopic << ")");
            _arbiterState = TELEOP;
        }

        bool switchToNavigation(std_srvs::Empty::Request &req,
                                std_srvs::Empty::Response &res)
        {
            if (_arbiterState != NAVIGATION)
            {
                _inSub.shutdown();
                subscribeToNavigation();
            }
            return true;
        }

        bool switchToTeleop(std_srvs::Empty::Request &req,
                            std_srvs::Empty::Response &res)
        {
            if (_arbiterState != TELEOP)
            {
                _inSub.shutdown();
                subscribeToTeleop();
            }
            return true;
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
