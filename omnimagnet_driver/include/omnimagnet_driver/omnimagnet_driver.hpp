#pragma once

#include "rclcpp/rclcpp.hpp"
#include "comedilib.hpp"
#include "omnimagnet.hpp"
#include "omnimagnet_interfaces/msg/error_message.hpp"
#include "omnimagnet_interfaces/msg/finished_message.hpp"
#include "omnimagnet_interfaces/srv/single_magnet_constant.hpp"
#include "omnimagnet_interfaces/srv/single_magnet_rotation.hpp"
#include "omnimagnet_interfaces/srv/multi_magnet_constant.hpp"
#include "omnimagnet_interfaces/srv/multi_magnet_rotation.hpp"
#include "omnimagnet_interfaces/srv/driver_reset.hpp"

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

/**
 * @brief Set of orthonormal vectors spanning a 2D rotation plane in 3D space.
 * 
 * The two vectors u and v are unit length, mutually orthogonal, and together define the orientation of a 2D rotation plane in 3D space.
 */
struct Basis {
    /**
     * @brief Default constructor returns <1, 0, 0> and <0, 1, 0>.
     */
    Basis() {
        u_ << 1, 0, 0;
        v_ << 0, 1, 0;
    }

	/**
     * @brief Constructor that takes two 3x1 vectors to construct a basis set.
     * 
     * Constructs an orthonormal basis spanning the plane defined by the two input vectors using Gram-Schmidt.
     * The first unit vector will coincide with the first input vector.
     * 
     * @param u The first 3x1 Eigen vector.
     * @param v The second 3x1 Eigen vector.
     * @param tolerance Tolerance used to detect zero and parallel vectors. (Default = 1e-6)
     * 
     * @throws std::invalid_argument if either vector is zero or if the vectors are parallel.
     * 
     * @note The input vectors do not need to be normalized.
     */
	Basis(const Eigen::Vector3d &u, const Eigen::Vector3d &v, const double tolerance = 1e-6) {
        // Check for zero vectors
        if (u.norm() < tolerance || v.norm() < tolerance) {
            throw std::invalid_argument("Vectors cannot be zero vectors.");
        }

        u_ = u.normalized();

        const Eigen::Vector3d v_ortho = v - u * u.dot(v);

        // Check for parallel vectors
        if (v_ortho.norm() < tolerance) {
            throw std::invalid_argument("Cannot generate a basis from parallel vectors.");
        }

		v_ = v_ortho.normalized();
	}

    /**
     * @brief Gets the 'u' basis vector.
     * 
     * @return 3x1 Eigen vector 'u'.
     */
    [[nodiscard]] const Eigen::Vector3d& u() const noexcept {
        return u_;
    }

    /**
     * @brief Gets the 'v' basis vector.
     * 
     * @return 3x1 Eigen vector 'v'.
     */
    [[nodiscard]] const Eigen::Vector3d& v() const noexcept{
        return v_;
    }

private:
    Eigen::Vector3d u_;
	Eigen::Vector3d v_;
};

struct ActiveMagnetCommand
{
    OmniMagnet* omni;
    double freq;
    double strength;
    double offset;
    Basis basis;
    Eigen::Vector3d vector;
};

struct MagnetConfig
{
    int id;
    bool enabled;

    double wire_width;
    double inner_wire_length;
    double mid_wire_length;
    double outer_wire_length;
    double core_size;

    int64_t inner_channel;
    int64_t mid_channel;
    int64_t outer_channel;

    bool estimate;
    std::vector<double> frame;
};

struct ParameterException : public std::runtime_error {
    using std::runtime_error::runtime_error;
};

class OmnimagnetDriverNode : public rclcpp::Node {

public:
    
    OmnimagnetDriverNode();
    void shutdown();

private:
    // Operating Parameters
    const static std::size_t maxMagnets{6};

    double defaultDuration_{30.0};
    double timeout_{300.0};

    std::thread controlThread;
    std::atomic<bool> experimentRunning{false};
    std::atomic<bool> controlThreadRunning{false};

    std::mutex commandMutex;

    // Containers
    std::map<int, OmniMagnet> omnimagnets;
    std::array<ActiveMagnetCommand, maxMagnets> activeCommands;
    std::atomic<std::size_t> activeCommandCount{0};

    // Ros2 Agents
    rclcpp::Publisher<omnimagnet_interfaces::msg::ErrorMessage>::SharedPtr errorPublisher;
    rclcpp::Publisher<omnimagnet_interfaces::msg::FinishedMessage>::SharedPtr finishedPublisher;

    rclcpp::Service<omnimagnet_interfaces::srv::SingleMagnetRotation>::SharedPtr smrServer;
    rclcpp::Service<omnimagnet_interfaces::srv::SingleMagnetConstant>::SharedPtr smcServer;
    rclcpp::Service<omnimagnet_interfaces::srv::MultiMagnetConstant>::SharedPtr mmcServer;
    rclcpp::Service<omnimagnet_interfaces::srv::MultiMagnetRotation>::SharedPtr mmrServer;
    rclcpp::Service<omnimagnet_interfaces::srv::DriverReset>::SharedPtr resetServer;
    
    rclcpp::TimerBase::SharedPtr timeoutTimer;
    rclcpp::TimerBase::SharedPtr durationTimer;

    std::chrono::_V2::steady_clock::time_point startTime;

    // D2A card
    comedi_t *D2A;
    lsampl_t maxdata1{16383};
    lsampl_t maxdata2{16383};

    // Vectors

    Eigen::Vector3d offVector;

    /******* FUNCTIONS *******/
    void declareParameters();
    void declareMagnetParameters(std::size_t, const std::array<int, 3>&);
    void loadParameters();
    MagnetConfig loadMagnetConfig(std::size_t) const;
    void buildTimers();
    void resetDurationTimer(const double);
    void buildPublishers();
    void buildServices();
    void setupHardware();
    void setupMagnets();

    // Timer callbacks
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
        [[maybe_unused]] const omnimagnet_interfaces::srv::DriverReset::Request::SharedPtr,
        const omnimagnet_interfaces::srv::DriverReset::Response::SharedPtr
    );

    void controlLoop();

    // Parameters
    const std::vector<double> identityFrame_{
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0
    };
};

