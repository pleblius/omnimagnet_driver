#pragma once

#include "rclcpp/rclcpp.hpp"
#include "comedilib.hpp"
#include "omnimagnet.hpp"
#include "commands.hpp"
#include "omnimagnet_interfaces/msg/error_message.hpp"
#include "omnimagnet_interfaces/msg/finished_message.hpp"
#include "omnimagnet_interfaces/srv/single_magnet_constant.hpp"
#include "omnimagnet_interfaces/srv/single_magnet_rotation.hpp"
#include "omnimagnet_interfaces/srv/multi_magnet_constant.hpp"
#include "omnimagnet_interfaces/srv/multi_magnet_rotation.hpp"
#include "omnimagnet_interfaces/srv/driver_reset.hpp"

#include "omnimagnet_interfaces/srv/single_current_constant.hpp"
#include "omnimagnet_interfaces/srv/single_current_rotation.hpp"
#include "omnimagnet_interfaces/srv/multi_current_constant.hpp"
#include "omnimagnet_interfaces/srv/multi_current_rotation.hpp"

#include <chrono>
#include <vector>
#include <map>
#include <stdexcept>

/*****************************************************
omnimagnet_driver.hpp   defines a class which inherits from the Ros2 node class. This class is used
    is used to control the omnimagnet system in Jake Abbott's Magnetic Manipulation Lab using Ros2
    communication protocols. Cuurrent operational parameters allow for the control of either a single
    or multiple magnets, which can collectively drive independent constant or independent rotating magentic
    dipoles. Additional features can be added using the Ros2 service framework along with an appropriate callback.

    Operation is currently single-threaded, with all magnets updated simultaneously with a given time step.

    TODO: Implement callbacks for additional control modes
    TODO: Update omnimagnet.cpp for more error-handling, including current saturation and failure to write

	Includes:
        rclcpp/rclcpp.hpp
        comedilib.hpp
		omnimagnet.hpp
        omnimagnet_interfaces/
    Inherits:
        rclcpp::Node

Ver 1.0 by Tyler Wilcox, August 2026
tyler.c.wilcox@utah.edu		
*****************************************************/

struct ParameterException : public std::runtime_error {
    using std::runtime_error::runtime_error;
};

enum class DriverMode {
    OFF,
    IDLE,
    RUNNING,
    NEW_COMMAND,
    RESET
};

class OmnimagnetDriverNode : public rclcpp::Node {

public:
    
    OmnimagnetDriverNode();
    void shutdown();

private:
    // Operating Parameters
    const static std::size_t maxMagnets_{6};
    double timeout_;

    // Thread Managers
    std::atomic<DriverMode> runMode_{DriverMode::OFF};

    std::thread controlThread_;
    std::mutex commandMutex_;



    // Containers
    std::map<int, OmniMagnet> omnimagnets_;
    std::vector<ActiveMagnetCommand> activeCommands_;

    // Ros2 Agents
    rclcpp::Publisher<omnimagnet_interfaces::msg::ErrorMessage>::SharedPtr errorPublisher_;
    rclcpp::Publisher<omnimagnet_interfaces::msg::FinishedMessage>::SharedPtr finishedPublisher_;

    rclcpp::Service<omnimagnet_interfaces::srv::SingleMagnetRotation>::SharedPtr smrServer_;
    rclcpp::Service<omnimagnet_interfaces::srv::SingleMagnetConstant>::SharedPtr smcServer_;
    rclcpp::Service<omnimagnet_interfaces::srv::MultiMagnetConstant>::SharedPtr mmcServer_;
    rclcpp::Service<omnimagnet_interfaces::srv::MultiMagnetRotation>::SharedPtr mmrServer_;

    rclcpp::Service<omnimagnet_interfaces::srv::SingleCurrentConstant>::SharedPtr sccServer_;
    rclcpp::Service<omnimagnet_interfaces::srv::SingleCurrentRotation>::SharedPtr scrServer_;
    rclcpp::Service<omnimagnet_interfaces::srv::MultiCurrentConstant>::SharedPtr mccServer_;
    rclcpp::Service<omnimagnet_interfaces::srv::MultiCurrentRotation>::SharedPtr mcrServer_;

    rclcpp::Service<omnimagnet_interfaces::srv::DriverReset>::SharedPtr resetServer_;
    
    rclcpp::TimerBase::SharedPtr timeoutTimer_;
    rclcpp::TimerBase::SharedPtr durationTimer_;

    // D2A card
    std::string device_;
    comedi_t *card_;
    int subDevice_;
    int channel_;
    int range_;
    int aref_;
    int minSample_;
    int maxSample_;
    double minVoltage_;
    double maxVoltage_;
    double minCurrent_;
    double maxCurrent_;
    std::vector<int64_t> inhibPins_;
    double inhibVolt_;

    /******* FUNCTIONS *******/
    void declareParameters();
    void declareMagnetParameters(std::size_t, const std::array<int, 3>&);
    MagnetConfig loadMagnetConfig(std::size_t) const;
    void buildTimers();
    void resetDurationTimer(double);
    void buildPublishers();
    void buildServices();
    void setupHardware();
    void setupMagnets();
    lsampl_t currentD2A(double);
    int runCurrent(const Eigen::Vector3d&, const OmniMagnet&);

    // Timer callbacks
    void timeoutCallback();
    void durationCallback();

    // Server callbacks
    void resetCallback(
        [[maybe_unused]] const omnimagnet_interfaces::srv::DriverReset::Request::SharedPtr,
        const omnimagnet_interfaces::srv::DriverReset::Response::SharedPtr
    );

    // Dipole-driven callbacks
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

    // Current-driven callbacks
    void sccCallback(
        const omnimagnet_interfaces::srv::SingleCurrentConstant::Request::SharedPtr,
        const omnimagnet_interfaces::srv::SingleCurrentConstant::Response::SharedPtr
    );
    void scrCallback(
        const omnimagnet_interfaces::srv::SingleCurrentRotation::Request::SharedPtr,
        const omnimagnet_interfaces::srv::SingleCurrentRotation::Response::SharedPtr
    );
    void mccCallback(
        const omnimagnet_interfaces::srv::MultiCurrentConstant::Request::SharedPtr,
        const omnimagnet_interfaces::srv::MultiCurrentConstant::Response::SharedPtr
    );
    void mcrCallback(
        const omnimagnet_interfaces::srv::MultiCurrentRotation::Request::SharedPtr,
        const omnimagnet_interfaces::srv::MultiCurrentRotation::Response::SharedPtr
    );


    void controlLoop();
    void resetCommands(std::vector<ActiveMagnetCommand>&);
    void loadNewCommands(std::vector<ActiveMagnetCommand>&);
    void runCommands(std::vector<ActiveMagnetCommand>&, double);

    bool systemIsBusy();
};

