#pragma once

# include "rclcpp/rclcpp.hpp"
# include "comedilib.hpp"
# include "omnimagnet.hpp"
# include "omnimagnet_interfaces/msg/error_message.hpp"
# include "omnimagnet_interfaces/msg/finished_message.hpp"
# include "omnimagnet_interfaces/srv/single_magnet_constant.hpp"
# include "omnimagnet_interfaces/srv/single_magnet_rotation.hpp"
# include "omnimagnet_interfaces/srv/multi_magnet_constant.hpp"
# include "omnimagnet_interfaces/srv/multi_magnet_rotation.hpp"
# include "omnimagnet_interfaces/srv/driver_reset.hpp"

# include <chrono>
# include <vector>
# include <map>

/*****************************************************
omnimagnet_driver.hpp   defines a class which inherits from the Ros2 node class. This class is used
    is used to control the omnimagnet system in Jake Abbott's Magnetic Manipulation Lab using Ros2
    communication protocols. Cuurrent operational parameters allow for the control of either a single
    or multiple magnets, which can collectively drive independent constant or independent rotating magentic
    dipoles. Additional features can be added using the Ros2 service framework along with an appropriate callback.

    Operation is currently single-threaded, with all magnets updated simultaneously with a given time step.

    TODO: Change fixed constants to parameterized files for easy updating
    TODO: Implement callbacks for additional control modes
    TODO: Account for rotation with defined phase offset between magnets
    TODO: Update omnimagnet.cpp for more error-handling, including current saturation and failure to write

	Includes:
        rclcpp/rclcpp.hpp
        comedilib.hpp
		omnimagnet.hpp
        omnimagnet_interfaces/
    Inherits:
        rclcpp::Node

Ver 0.1 by Tyler Wilcox, April 2026
tyler.c.wilcox@utah.edu		
*****************************************************/
class OmnimagnetDriverNode : public rclcpp::Node {

public:
    
    OmnimagnetDriverNode();
    void shutdown();

private:
    // Containers
    std::map<int, OmniMagnet> omnimagnets;
    std::vector<OmniMagnet*> activeMagnets;

    // Ros2 Agents
    rclcpp::Publisher<omnimagnet_interfaces::msg::ErrorMessage>::SharedPtr errorPublisher;
    rclcpp::Publisher<omnimagnet_interfaces::msg::FinishedMessage>::SharedPtr finishedPublisher;

    rclcpp::Service<omnimagnet_interfaces::srv::SingleMagnetRotation>::SharedPtr smrServer;
    rclcpp::Service<omnimagnet_interfaces::srv::SingleMagnetConstant>::SharedPtr smcServer;
    rclcpp::Service<omnimagnet_interfaces::srv::MultiMagnetConstant>::SharedPtr mmcServer;
    rclcpp::Service<omnimagnet_interfaces::srv::MultiMagnetRotation>::SharedPtr mmrServer;
    rclcpp::Service<omnimagnet_interfaces::srv::DriverReset>::SharedPtr resetServer;
    
    rclcpp::TimerBase::SharedPtr timeoutTimer;
    rclcpp::TimerBase::SharedPtr spinTimer;
    rclcpp::TimerBase::SharedPtr durationTimer;

    std::chrono::_V2::system_clock::time_point startTime;

    // D2A card
    comedi_t *D2A;
    int subdev;
    int chan;
    int range;
    int aref;
    lsampl_t maxdata25{16383};
    lsampl_t maxdata26{16383};

    // Vectors

    Eigen::Vector3d offVector;

    /******* FUNCTIONS *******/
    void setupHardware();
    void setupMagnets();

    // Timer callbacks
    void spinCallback();
    void timeoutCallback();
    void durationCallback();

    // Server callbacks
    void smcCallback(
        const omnimagnet_interfaces::srv::SingleMagnetConstant::Request::SharedPtr,
        const omnimagnet_interfaces::srv::SingleMagnetConstant::Response::SharedPtr
    );
    void smrCallback(
        const omnimagnet_interfaces::srv::SingleMagnetRotation::Request::SharedPtr,
        const omnimagnet_interfaces::srv::SingleMagnetRotation::Response::SharedPtr
    );
    void mmcCallback(
        const omnimagnet_interfaces::srv::MultiMagnetConstant::Request::SharedPtr,
        const omnimagnet_interfaces::srv::MultiMagnetConstant::Response::SharedPtr
    );
    void mmrCallback(
        const omnimagnet_interfaces::srv::MultiMagnetRotation::Request::SharedPtr,
        const omnimagnet_interfaces::srv::MultiMagnetRotation::Response::SharedPtr
    );
    void resetCallback(
        const omnimagnet_interfaces::srv::DriverReset::Request::SharedPtr,
        const omnimagnet_interfaces::srv::DriverReset::Response::SharedPtr
    );

};