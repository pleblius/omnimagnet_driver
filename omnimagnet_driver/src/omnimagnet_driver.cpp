#include "rclcpp/rclcpp.hpp"
#include "comedilib.hpp"
#include "../include/omnimagnet_driver/omnimagnet.hpp"
#include "../include/omnimagnet_driver/omnimagnet_driver.hpp"
#include "../include/omnimagnet_driver/commands.hpp"

#include <atomic>
#include <thread>
#include <iomanip>
#include <any>

// Helper functions
namespace {
    /**
     * @brief Generates a namespace string for a given magnet index.
     * 
     * This function constructs a namespace string in the format "magnets.magnet_{index}" where 
     * {index} is replaced by the provided index.
     * 
     * @param index The index of the magnet.
     * 
     * @return The namespace string.
     */
    std::string magnetNamespace(std::size_t index) {
        return "magnets.magnet_" + std::to_string(index);
    }

    /**
     * @brief Helper function to handle command errors.
     * 
     * This function is used to handle errors that arise from faulty command requests.
     * It uses the logger to log the warning message, 
     * sets the error state and error message in the service response, 
     * and resets the provided timeout timer.
     * 
     * @param logger The logger to use for logging the warning message.
     * @param logMessage The warning message to log.
     * @param response The response object to set the error state.
     * @param responseMessage The error message to set in the response.
     * @param timeoutTimer The timer to reset after handling the error.
     */
    template<typename ResponseType>
    void commandError(
        const rclcpp::Logger& logger,
        const std::string& logMessage,
        const std::shared_ptr<ResponseType>& response,
        const std::string& responseMessage,
        rclcpp::TimerBase::SharedPtr& timeoutTimer
    ) {
        RCLCPP_WARN(logger, "%s", logMessage.c_str());

        response->error = true;
        response->error_desc = responseMessage;

        timeoutTimer->reset();
    }

    /**
     * @brief Helper function to handle system errors.
     * 
     * This function is used to handle system errors that arise from hardware issues 
     * or other system-level problems.
     * It uses the logger to log the error message, 
     * including the error from the comedi library (assumes comedi_errno is set and unchanged), 
     * sets the error state and error message in the error publisher,
     * and optionally shuts down the system if specified.
     * 
     * @param logger The logger to use for logging the error message.
     * @param logMessage The error message to log.
     * @param comediError The error message from the comedi library.
     * @param errorPublisher The publisher used to publish the error message.
     * @param errorDescription The error description to publish.
     * @param shutdown Whether to shutdown the system after handling the error.
     */
    void systemError(
        const rclcpp::Logger& logger,
        const std::string& logmessage,
        const std::string& comediError,
        const rclcpp::Publisher<omnimagnet_interfaces::msg::ErrorMessage>::SharedPtr& errorPublisher,
        const std::string& errorDescription,
        const bool shutdown
    ) {
        RCLCPP_ERROR(logger, "%s", logmessage.c_str());
        comedi_perror(comediError.c_str());

        auto msg = omnimagnet_interfaces::msg::ErrorMessage();
        msg.error_desc = errorDescription;
        msg.shutdown = shutdown;
        errorPublisher->publish(msg);

        if (shutdown) {
            rclcpp::shutdown();
        }
    }

    /**
     * @brief Constructs a Basis struct orthogonal to the provided vector.
     * 
     * This function takes an axis vector and constructs a 2D plane orthogonal to that vector. It then
     * generates an orthonormal basis defining that rotation plane.
     * 
     * @param axis The axis vector to construct the basis from.
     * 
     * @return A Basis struct containing the orthonormal basis vectors u and v.
     * 
     * @note This method is deterministic, but to maintain numerical stability small changes to the input may produce large changes in the resulting basis vectors
     * even though the rotation plane remains identical.
     */
    Basis makeBasisFromRotationVector(const Eigen::Vector3d& axis) {
        Eigen::Vector3d n = axis.normalized();

        // Choose initial vector to cross with based on which it is least parallel to
        Eigen::Vector3d initVec =
            (std::abs(n.x()) < std::abs(n.y()))
                ? Eigen::Vector3d::UnitX()
                : Eigen::Vector3d::UnitY();

        Eigen::Vector3d u = n.cross(initVec);
        Eigen::Vector3d v = n.cross(u);

        return Basis(u, v);
    }

	/**
	 * @brief Maps a value from one range to another.
	 * 
	 * This function takes an input value and maps it from the range [val_min, val_max] to the range [range_min, range_max].
	 * If the input value is outside the input range, it will be clamped to the nearest boundary of the output range.
	 * 
	 * @param value The input value to be mapped.
	 * @param val_min The minimum value of the input range.
	 * @param val_max The maximum value of the input range.
	 * @param range_min The minimum value of the output range.
	 * @param range_max The maximum value of the output range.
	 * 
	 * @return The mapped value in the output range.
	 * 
	 * @tparam inType The type of the input value.
	 * @tparam outType The type of the output value.
	 * 
	 * @throws std::invalid_argument if the input range has zero width.
	 * 
	 * @note Generic types must support arithmetic operations and type casting to double.
	 */
	template <typename inType, typename outType>
	outType mapRange(inType value, inType val_min, inType val_max, outType range_min, outType range_max) {
		if (val_min == val_max) {
			throw std::invalid_argument("Input range has zero width.");
		}

		if (value >= val_max)
			return range_max;
		if (value <= val_min)
			return range_min;

		return static_cast<outType>(
			static_cast<double>(value - val_min) * 
			static_cast<double>(range_max - range_min) / 
			static_cast<double>(val_max - val_min) + 
			range_min
		);
	}

    /**
     * @brief Checks if a vector is valid for driver use. Checks for zero vectors and non-finite components.
     * 
     * @param vector The 3x1 vector to check for validity.
     * @param output An error string if the vector is invalid.
     * 
     * @return true if the vector is valid.
     */
    bool validVector(const Eigen::Vector3d& vector, std::string& output) {
        // Check for NaN
        if (!vector.allFinite()) {
            output = "User passed non-finite vector.";

            return false;
        }

        // Check for zero vectors
        if (vector.isApprox(Eigen::Vector3d::Zero())) {
            output = "User passed zero vector.";

            return false;
        }

        return true;
    }

    /**
     * @brief Checks if a magnet ID is contained in the magnet map.
     * 
     * @param id Magnet id.
     * @param map Map of IDs to magnet objects.
     * @param output Contains error string if vector is invalid.
     * 
     * @return true if the id is valid.
     */
    bool checkID(
        int id,
        const std::map<int, OmniMagnet>& map,
        std::string& output
    ) {
        if (map.count(id) == 0) {
            output = "Magnet ID " + std::to_string(id) + " not found.";

            return false;
        }

        return true;
    }

    /**
     * @brief Checks an vector of magnet IDs for validity.
     * 
     * @param ids Vector containing list of magnet IDs.
     * @param map Mapping of integer ids to magnet objects.
     * @param output Contains error string if vector is invalid.
     * 
     * @return true if ID vector is valid.
     */
    bool checkIDs(
        const std::vector<uint64_t>& ids,
        const std::map<int, OmniMagnet>& map,
        std::string& output
    ) {
        if (ids.size() < 1) {
            output = "No magnets selected.";

            return false;
        }

        // Check if map contains id
        for (auto& id : ids) {
            if (!checkID(id, map, output))
                return false;
        }

        return true;
    }

    /**
     * @brief Checks if each vector has a valid number of elements, either exactly 1 element or the same number of elements as the ids vector.
     * 
     * @param ids Vector of magnet ids.
     * @param output Contains error string if check fails.
     * @param vecs Variadic argument of all vectors to check.
     * 
     * @return true if all vectors are a valid size.
     */
    template <typename... Vecs>
    bool checkMultipleVectors(
        const std::vector<uint64_t>& ids,
        std::string& output,
        const Vecs&... vecs
    ) {
        bool isValid =
            ((vecs.size() == 1 || vecs.size() == ids.size()) && ...);

        if (!isValid)
            output = "Vector size mismatch.";

        return isValid;
    }
}

/**
 * @brief Main function for the Omnimagnet Driver Node.
 * 
 * This function initializes the ROS2 node, creates an instance of the OmnimagnetDriverNode,
 * binds the shutdown method to the signal handler for safe program exit, and starts spinning the node.
 * 
 * @param argc The number of command line arguments.
 * @param argv The array of command line arguments.
 * 
 * @return int Returns 0 on successful execution.
 */
int main (int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<OmnimagnetDriverNode>();
    
    // Bind shutdown method to signal handler for safe program exit
    rclcpp::on_shutdown(
        std::bind(&OmnimagnetDriverNode::shutdown, node)
    );

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}

/**
 * @brief Constructor for the OmnimagnetDriverNode class. Inherits from the rclcpp::Node class.
 * 
 * This constructor initializes the OmnimagnetDriverNode, declares and loads parameters,
 * builds timers, publishers, and services, sets up the hardware and magnets,
 * and starts the control loop in a separate thread.
 */
OmnimagnetDriverNode::OmnimagnetDriverNode() : 
    rclcpp::Node ("omnimagnet_driver") {

    declareParameters();
    buildTimers();
    buildPublishers();
    buildServices();
    setupHardware();
    setupMagnets();

    controlThreadRunning_.store(true, std::memory_order_release);
    controlThread_ = std::thread(&OmnimagnetDriverNode::controlLoop, this);
}

/**
 * @brief Sets up the hardware for the OmnimagnetDriverNode.
 * 
 * This function initializes the D2A device, sets the amplifier inhibitors, and logs the completion of hardware setup.
 * It retrieves the necessary parameters from the ROS2 parameter server, including subdevice, analog reference, D2A device, inhibitor percent, and inhibitor pins.
 * It also checks for errors during the hardware setup process and publishes error messages if any issues arise.
 * If the hardware setup is successful, it logs a message indicating that the hardware setup is complete.
 * If any errors occur during the hardware setup, it logs the error and shuts down the ROS2 node.
 */
void OmnimagnetDriverNode::setupHardware() {
    subDevice_  = this->get_parameter("hardware.subdevice").as_int();
    aref_       = this->get_parameter("hardware.analog_reference").as_int();
    range_      = this->get_parameter("hardware.range").as_int();
    channel_    = this->get_parameter("hardware.channel").as_int();
    device_     = this->get_parameter("hardware.device").as_string();
    inhibVolt_  = this->get_parameter("hardware.inhibitor.voltage").as_double();
    inhibPins_  = this->get_parameter("hardware.inhibitor.pins").as_integer_array();
    minSample_  = this->get_parameter("hardware.min_sample").as_int();
    maxSample_  = this->get_parameter("hardware.max_sample").as_int();
    minVoltage_ = this->get_parameter("hardware.min_voltage").as_double();
    maxVoltage_ = this->get_parameter("hardware.max_voltage").as_double();
    minCurrent_ = this->get_parameter("hardware.min_current").as_double();
    maxCurrent_ = this->get_parameter("hardware.max_current").as_double();

    // Open the D2A device
    card_ = comedi_open(device_.c_str());
    if (card_ == nullptr) {
        systemError(
            this->get_logger(),
            "Failed to open D2A:",
            "comedi_open",
            errorPublisher_,
            "Failed to open D2A device.\n",
            true
        );

        return;
    }

    // Setting amplifier inhibitors at 5V. Default pins are 25 & 26

    for (const auto& pin : inhibPins_) {
        auto sample = mapRange<double, lsampl_t>(inhibVolt_, minVoltage_, maxVoltage_, minSample_, maxSample_);
        auto retVal = comedi_data_write(card_, subDevice_, pin, range_, aref_, sample);

        // If failed to enable pin, send system error and shutdown system
        if (retVal < 0) {
            systemError(
                this->get_logger(),
                "Failed to set enable pin " + std::to_string(pin) + ":",
                "comedi_data_write",
                errorPublisher_,
                "Failed to set enable pin " + std::to_string(pin) + ".\n",
                true
            );

            return;
        }
    }

    RCLCPP_INFO(this->get_logger(), "Hardware setup complete. Power supply enabled.");
}

/**
 * @brief Sets up the magnets for the OmnimagnetDriverNode.
 * 
 * This function iterates through the maximum number of magnets, loading their configurations from parameters.
 * For each enabled magnet, it initializes an OmniMagnet object with the specified properties and frame.
 * It also updates the mapping and sets the D2A maximum value for each magnet.
 * If any magnet configuration is invalid or if there are duplicate magnet IDs, it throws a runtime error.
 * Finally, it logs the completion of magnet setup and the number of enabled magnets.
 * 
 * @throws ParameterException if a magnet configuration is invalid or if there are duplicate magnet IDs.
 */
void OmnimagnetDriverNode::setupMagnets() {
    // Max magnets is currently hard-coded as a static constant in omnimagnet_driver.hpp, 
    // to allow for fixed-size command arrays to increase performance. This might be unnecessary. 
    for (std::size_t i = 1; i <= maxMagnets_; ++i) {
        const MagnetConfig config = loadMagnetConfig(i);

        // Skip if magnet is set to disabled
        if (!config.enabled)
            continue;

        // Tries to insert magnet into map with desired ID
        auto [iterator, inserted] =
            omnimagnets_.try_emplace(config.id);

        // Check if the same ID is loaded twice
        if (!inserted) {
            throw ParameterException(
                "Duplicate magnet ID: " + std::to_string(config.id)
            );
        }

        OmniMagnet& magnet = iterator->second;

        try {
            magnet.setProp(config);
        }
        catch (const std::invalid_argument& err) {
            throw ParameterException("Invalid frame for Magnet " + std::to_string(config.id) + ": " + err.what());
        }

        RCLCPP_INFO(this->get_logger(), "Configured magnet %d from parameter index %zu", config.id, i);
    }

    RCLCPP_INFO(this->get_logger(), "Magnet setup complete: %zu magnet(s) enabled.", omnimagnets_.size());
}

/**
 * @brief Shuts down the OmnimagnetDriverNode, turning off all magnets and releasing resources.
 * 
 * This function is called during the shutdown process of the ROS2 node.
 * It ensures that all magnets are turned off and that the D2A device is properly released.
 * 
 * It first checks if the shutdown process has already been initiated to prevent multiple shutdown attempts.
 * It then stops the control thread and waits for it to finish.
 * After that, it attempts to turn off all magnets by setting their current to zero.
 * If any magnet fails to turn off, it logs an error message.
 * Finally, it attempts to shut down the D2A pins and logs the result.
 * If any errors occur during the shutdown process,
 * it logs the error messages and advises the user to use the emergency stop if necessary.
 */
void OmnimagnetDriverNode::shutdown() {
    // Currently unnecessary, as shutdown is only called from the main thread. Implemented preemptively in case of future changes.
    static std::atomic_bool already_shutdown{false};

    if (already_shutdown.exchange(true)) {
        return;
    }
    already_shutdown.store(true, std::memory_order_release);

    std::cout << "Beginning Shutdown" << std::endl;

    // Stop the control thread and wait for it to finish.
    controlThreadRunning_.store(false, std::memory_order_release);

    if (controlThread_.joinable()) {
        controlThread_.join();
    }

    // D2A should have been initialized, but check for safety.
    if (card_ == nullptr) {
        std::cerr << "D2A was null during shutdown:" << std::endl <<
        "Hardware may not have been properly initialized" << std::endl;

        return;
    }

    // Turn off all magnets. Log any failures, but continue to attempt to turn off all magnets.
    bool magnetFail = false;
    for (auto & [id, magnet] : omnimagnets_) {
        int retval = runCurrent(Eigen::Vector3d::Zero(), magnet);

        if (retval < 0) {
            std::cerr << "Magnet " << id << 
                " failed to turn off." << std::endl;
            magnetFail = true;
        }
        else {
            std::cout << "Magnet " << id << 
            " turned off." << std::endl;
        }
    }

    if (!magnetFail) {
        std::cout << "All magnets turned off." << std::endl;
    }
    else {
        std::cerr << "Failed to shut down all magnets." << std::endl;
    }

    // Shut down amplifier enable pins. Logs failures and continues.
    bool pinFail = false;

    for (const auto& pin : inhibPins_) {
        auto zeroVoltage = mapRange<double, lsampl_t>(0.0, minVoltage_, maxVoltage_, minSample_, maxSample_);
        int retval = comedi_data_write(card_, subDevice_, pin, range_, aref_, zeroVoltage);

        if (retval < 0) {
            std::cerr << "Failed to shut down enable pin " << std::to_string(pin) << std::endl;
            pinFail = true;
        }
    }

    if (!pinFail) {
        std::cout << "Successfully shut down power supply enable pins." << std::endl;
    }
    else {
        std::cerr << "Failed to shut down power supply enable pins, use emergency stop!" << std::endl;
    }
}

/*************** TIMER CALLBACKS ***************/

/** @brief Callback for handling timeout events.
  * 
  * This function is called when a timeout event occurs.
  * It logs an error message, publishes to the error publisher, and shuts down the system.
  */
void OmnimagnetDriverNode::timeoutCallback() {
    RCLCPP_WARN(this->get_logger(), "System timed out. Shutting down.");
    
    auto msg = omnimagnet_interfaces::msg::ErrorMessage();

    msg.error_desc = "System timed out.";
    msg.shutdown = true;

    errorPublisher_->publish(msg);

    rclcpp::shutdown();
}

/** @brief Callback for handling end-of-duration events.
  * 
  * This function is called when the Node's duration monitor expires, 
  * indicating that the current request has been completed.
  * It cancels the duration timer, resets the timeout timer,
  * stops the experiment, sets the current command count to 0,
  * turns off all magnets, and publishes a FinishedMessage with the timestamp of completion.
  * If a magnet fails to shut down, it logs an error message and shuts down the system.
  */
void OmnimagnetDriverNode::durationCallback() {
    // Delete timer until new order is received
    durationTimer_->cancel();

    auto msg = omnimagnet_interfaces::msg::FinishedMessage();

    // Current timestamp
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);

    std::ostringstream oss;
    oss << "Run finished at: "
        << std::put_time(std::localtime(&now_c), "%Y-%m-%d %H:%M:%S");

    msg.msg = oss.str();

    RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());

    finishedPublisher_->publish(msg);

    resetCommand_.store(true, std::memory_order_release);

    // Reset timeout timer to wait for next command
    timeoutTimer_->reset();
}

/*************** SERVER CALLBACKS ***************/

/**
 * @brief Callback for handling SingleMagnetConstant service requests.
 * 
 * This function is called when a SingleMagnetConstant service request is received.
 * If a command is already in progress, it will log a warning and ignor 
 * It cancels the timeout timer and retrieves the request parameters, including the magnet ID, dipole strength, and dipole vector.
 * It checks if the specified magnet ID is valid, and if not, it logs a warning and returns an error response.
 * It also checks if the dipole vector is valid (non-zero and finite), and if not, it logs a warning and returns an error response.
 * If all checks pass, it sets up the active command for the specified magnet, starts the experiment, and creates a duration timer for the specified duration.
 * It logs the details of the operation and publishes a FinishedMessage when the operation is complete.
 */
void OmnimagnetDriverNode::smcCallback(
    const omnimagnet_interfaces::srv::SingleMagnetConstant::Request::SharedPtr request,
    const omnimagnet_interfaces::srv::SingleMagnetConstant::Response::SharedPtr response
    ) {
    // Ignore command if another is currently running
    if (systemIsBusy()) {
        commandError(
            this->get_logger(),
            "Received command while executing previous command. Disregarding.",
            response,
            "Command sent while another is being executed. New command ignored.",
            timeoutTimer_
        );

        return;
    }

    // Cancel the timeout timer to prevent it from triggering during the operation
    timeoutTimer_->cancel();

    auto id = request->omnimagnet;
    auto strength = request->dipole_strength;
    auto vector = request->dipole_vec;

    std::string errorString;

    // Check if the specified magnet ID is valid
    if (!checkID(id, omnimagnets_, errorString)) {
        commandError(
            this->get_logger(),
            errorString,
            response,
            errorString,
            timeoutTimer_
        );

        return;
    }
    auto& magnet = omnimagnets_.at(id);

    auto duration = request->duration;
    if (duration <= 0.0) {
        commandError(
            this->get_logger(),
            "Duration is non-positive: " + std::to_string(duration),
            response,
            "Duration is non-positive: " + std::to_string(duration),
            timeoutTimer_
        );
        return;
    }

    Eigen::Vector3d dipoleVector;
    dipoleVector << vector.x, vector.y, vector.z;

    if (!validVector(dipoleVector, errorString)) {
        commandError(
            this->get_logger(),
            errorString,
            response,
            errorString,
            timeoutTimer_
        );

        return;
    }

    // Normalize dipole vector to ensure it has a unit length
    dipoleVector.normalize();

    ConstantDipoleCommand command = {&magnet, strength, dipoleVector};

    // Set up the active command for the specified magnet using a lock guard to ensure thread safety
    {
        std::lock_guard<std::mutex> lock(commandMutex_);
        activeCommands_.push_back(command);
        newCommand_.store(true, std::memory_order_release);
    }

    resetDurationTimer(duration);

    RCLCPP_INFO(this->get_logger(), 
        "Beginning Operation\n"
        "Single Magnet\n"
        "Mode: Constant Dipole\n"
        "Duration %.3f s\n",
        duration
    );

    RCLCPP_INFO(this->get_logger(), 
        "Magnet: %lu\n"
        "Dipole: <%.3f, %.3f, %.3f>\n"
        "Strength: %.2f",
        id, vector.x, vector.y, vector.z, strength
    );
}

/**
 * @brief Callback for handling SingleMagnetRotation service requests.
 * 
 * This function is called when a SingleMagnetRotation service request is received.
 * It cancels the timeout timer and retrieves the request parameters, including the magnet ID, dipole strength, rotation vector, phase offset, and rotation frequency.
 * It checks if the specified magnet ID is valid, and if not, it logs a warning and returns an error response.
 * It also checks if the rotation vector is valid (non-zero and finite), and if not, it logs a warning and returns an error response.
 * If all checks pass, it sets up the active command for the specified magnet, starts the experiment, and creates a duration timer for the specified duration.
 * It logs the details of the operation and publishes a FinishedMessage when the operation is complete.
 */
void OmnimagnetDriverNode::smrCallback(
    const omnimagnet_interfaces::srv::SingleMagnetRotation::Request::SharedPtr request,
    const omnimagnet_interfaces::srv::SingleMagnetRotation::Response::SharedPtr response
) {
    // Ignore command if another is currently running
    if (systemIsBusy()) {
        commandError(
            this->get_logger(),
            "Received command while executing previous command. Disregarding.",
            response,
            "Command sent while another is being executed. New command ignored.",
            timeoutTimer_
        );

        return;
    }

    // Cancel the timeout timer to prevent it from triggering during the operation
    timeoutTimer_->cancel();

    auto id = request->omnimagnet;
    auto strength = request->dipole_strength;
    auto rotationVector = request->rotation_vector;
    auto offset = request->phase_offset;
    auto phaseOffset = offset * M_PI / 180.0; // Converted to radians
    auto freq = request->rotation_freq;
    auto duration = request->duration;
    
    std::string errorString;

    // Check if the specified magnet ID is valid
    if (!checkID(id, omnimagnets_, errorString)) {
        commandError(
            this->get_logger(),
            errorString,
            response,
            errorString,
            timeoutTimer_
        );

        return;
    }
    auto& magnet = omnimagnets_.at(id);

    // Check if duration is positive
    if (duration <= 0.0) {
        commandError(
            this->get_logger(),
            "Duration is non-positive: " + std::to_string(duration),
            response,
            "Duration is non-positive: " + std::to_string(duration),
            timeoutTimer_
        );
        return;
    }
    
    Eigen::Vector3d rotVec;
    rotVec << rotationVector.x, rotationVector.y, rotationVector.z;

    if (!validVector(rotVec, errorString)) {
        commandError(
            this->get_logger(),
            errorString,
            response,
            errorString,
            timeoutTimer_
        );

        return;
    }

    Basis rotationPlane = makeBasisFromRotationVector(rotVec);
    RotatingDipoleCommand command = {&magnet, freq, strength, phaseOffset, rotationPlane};

    // Set up the active command for the specified magnet using a lock guard to ensure thread safety
    {
        std::lock_guard<std::mutex> lock (commandMutex_);
        activeCommands_.push_back(command);
        newCommand_.store(true, std::memory_order_release);
    }

    resetDurationTimer(duration);

    RCLCPP_INFO(this->get_logger(), 
        "Beginning Operation\n"
        "Single Magnet\n"
        "Mode: Rotating Dipole\n"
        "Duration %.3f s\n",
        duration
    );

    RCLCPP_INFO(this->get_logger(), 
        "Magnet: %lu\n"
        "Rotation: <%.3f, %.3f, %.3f>\n"
        "Strength: %.2f\n"
        "Frequency: %.2f\n"
        "Offset: %.2f\n",
        id, rotationVector.x, rotationVector.y, rotationVector.z, strength, freq, offset
    );
}

/**
 * @brief Callback for handling MultiMagnetConstant service requests.
 * 
 * This function is called when a MultiMagnetConstant service request is received.
 * It cancels the timeout timer and retrieves the request parameters, including the magnet IDs, dipole strengths, and dipole vectors.
 * It checks for size mismatches in the request parameters and validates each magnet ID.
 * It also checks if the dipole vectors are valid (non-zero and finite), and if not, it logs a warning and returns an error response.
 * If all checks pass, it sets up the active commands for the specified magnets, starts the experiment, and creates a duration timer for the specified duration.
 * It logs the details of the operation and publishes a FinishedMessage when the operation is complete.
 */
void OmnimagnetDriverNode::mmcCallback(
    const omnimagnet_interfaces::srv::MultiMagnetConstant::Request::SharedPtr request,
    const omnimagnet_interfaces::srv::MultiMagnetConstant::Response::SharedPtr response
) {
    // Ignore command if another is currently running
    if (systemIsBusy()) {
        commandError(
            this->get_logger(),
            "Received command while executing previous command. Disregarding.",
            response,
            "Command sent while another is being executed. New command ignored.",
            timeoutTimer_
        );

        return;
    }

    // Cancel the timeout timer to prevent it from triggering during the operation
    timeoutTimer_->cancel();
    
    auto ids = request->omnimagnets;
    auto strengths = request->dipole_strengths;
    auto vectors = request->dipole_vecs;

    
    std::vector<ActiveMagnetCommand> commandList;
    
    std::string errorString;
    
    if (!checkIDs(ids, omnimagnets_, errorString)) {
        commandError(
            this->get_logger(),
            errorString,
            response,
            errorString,
            timeoutTimer_
        );
        
        return;
    }
    
    if (!checkMultipleVectors(
        ids,
        errorString,
        strengths,
        vectors
    )) {
        commandError(
            this->get_logger(),
            errorString,
            response,
            errorString,
            timeoutTimer_
        );

        return;
    }

    // Optional argument
    auto duration = request->duration;
    if (duration <= 0.0) {
        commandError(
            this->get_logger(),
            "Duration is non-positive: " + std::to_string(duration),
            response,
            "Duration is non-positive: " + std::to_string(duration),
            timeoutTimer_
        );

        return;
    }

    std::stringstream logString;

    logString 
        << "Beginning Operation" << std::endl
        << "Multiple Magnets" << std::endl
        << "Mode: Constant Dipole" << std::endl
        << "Duration: " << duration << " s" << std::endl;

    // Set up active commands for each magnet
    for (std::size_t i = 0; i < ids.size(); ++i) {
        uint64_t id;
        omnimagnet_interfaces::msg::Vector3 vector;
        double strength;

        id = ids[i];

        // If only one vector is provided, use it for all magnets; otherwise, use the corresponding vector for each magnet
        vector = (vectors.size() == 1)
            ? vectors[0]
            : vectors[i];

        strength = (strengths.size() == 1)
            ? strengths[0]
            : strengths[i];

        auto& magnet = omnimagnets_.at(id);
        
        Eigen::Vector3d dipoleVector;
        dipoleVector << vector.x, vector.y, vector.z;

        if (!validVector(dipoleVector, errorString)) {
            commandError(
                this->get_logger(),
                errorString,
                response,
                errorString,
                timeoutTimer_
            );

            return;
        }

        // Normalize dipole vector to ensure it has a unit length
        dipoleVector.normalize();

        logString 
            << "Magnet: " << id << std::endl
            << "Dipole: <"
                << vector.x << ", "
                << vector.y << ", "
                << vector.z << ">" << std::endl
            << "Strength: " << strength << std::endl;

        ConstantDipoleCommand command = {&magnet, strength, dipoleVector};

        // Add command to the temporary command list
        commandList.push_back(command);
    }

    // Store commands for control thread
    {
        std::lock_guard<std::mutex> lock(commandMutex_);
        activeCommands_ = std::move(commandList);
        newCommand_.store(true, std::memory_order_release);
    }

    resetDurationTimer(duration);

    RCLCPP_INFO(this->get_logger(), "%s", logString.str().c_str());
}

/**
 * @brief Callback for handling MultiMagnetRotation service requests.
 * 
 * This function is called when a MultiMagnetRotation service request is received.
 * It cancels the timeout timer and retrieves the request parameters, including the magnet IDs, dipole strengths, rotation vectors, phase offsets, and rotation frequencies.
 * It checks for size mismatches in the request parameters and validates each magnet ID.
 * It also checks if the rotation vectors are valid (non-zero and finite), and if not, it logs a warning and returns an error response.
 * If all checks pass, it sets up the active commands for the specified magnets, starts the experiment, and creates a duration timer for the specified duration.
 * It logs the details of the operation and publishes a FinishedMessage when the operation is complete
 */
void OmnimagnetDriverNode::mmrCallback(
    const omnimagnet_interfaces::srv::MultiMagnetRotation::Request::SharedPtr request,
    const omnimagnet_interfaces::srv::MultiMagnetRotation::Response::SharedPtr response
) {
    // Ignore command if another is currently running
    if (systemIsBusy()) {
        commandError(
            this->get_logger(),
            "Received command while executing previous command. Disregarding.",
            response,
            "Command sent while another is being executed. New command ignored.",
            timeoutTimer_
        );

        return;
    }
    
    // Cancel the timeout timer to prevent it from triggering during the operation
    timeoutTimer_->cancel();

    auto ids = request->omnimagnets;
    auto rotationVectors = request->rotation_vectors;
    auto freqs = request->rotation_freqs;
    auto strengths = request->dipole_strengths;
    auto offsets = request->phase_offsets;
    auto duration = request->duration;

    std::vector<ActiveMagnetCommand> commandList;

    std::string errorString;

    if (!checkIDs(ids, omnimagnets_, errorString)) {
        commandError(
            this->get_logger(),
            errorString,
            response,
            errorString,
            timeoutTimer_
        );

        return;
    }

    if (!checkMultipleVectors(
        ids,
        errorString,
        rotationVectors,
        freqs,
        strengths,
        offsets    
    )) {
        commandError(
            this->get_logger(),
            errorString,
            response,
            errorString,
            timeoutTimer_
        );

        return;
    }

    if (duration <= 0.0) {
        commandError(
            this->get_logger(),
            "Duration is non-positive: " + std::to_string(duration),
            response,
            "Duration is non-positive: " + std::to_string(duration),
            timeoutTimer_
        );
        return;
    }

    std::stringstream logString;

    logString 
        << "Beginning Operation" << std::endl
        << "Multiple Magnets" << std::endl
        << "Mode: Rotating Dipole" << std::endl
        << "Duration: " << duration << " s" << std::endl;

    // Set up active commands for each magnet
    for (std::size_t i = 0; i < ids.size(); ++i) {
        uint64_t id;
        omnimagnet_interfaces::msg::Vector3 rotationVector;
        double strength;
        double offset;
        double freq;

        id = ids[i];

        // For each list, if only one entry is provided, apply it to all, otherwise use corresponding entry
        rotationVector = (rotationVectors.size() == 1)
            ? rotationVectors[0]
            : rotationVectors[i];

        strength = (strengths.size() == 1)
            ? strengths[0]
            : strengths[i];

        freq = (freqs.size() == 1)
            ? freqs[0]
            : freqs[i];

        offset = (offsets.size() == 1)
            ? offsets[0]
            : offsets[i];

        // Convert to radians
        auto phaseOffset = offset * M_PI / 180.0;

        auto& magnet = omnimagnets_.at(id);
        
        // Convert vector to Eigen Vector3d
        Eigen::Vector3d rotVec;
        rotVec << rotationVector.x, rotationVector.y, rotationVector.z;

        if (!validVector(rotVec, errorString)) {
            commandError(
                this->get_logger(),
                errorString,
                response,
                errorString,
                timeoutTimer_
            );

            return;
        }

        // Normalize rotation vector to ensure it has a unit length
        rotVec.normalize();

        Basis rotationPlane = makeBasisFromRotationVector(rotVec);
        RotatingDipoleCommand command = {&magnet, freq, strength, phaseOffset, rotationPlane};

        // Add to temporary command list
        commandList.push_back(command);
    

        logString
            << "Magnet: " << id << std::endl
            << "Rotation: <"
                << rotationVector.x << ", "
                << rotationVector.y << ", "
                << rotationVector.z << ">" << std::endl
            << "Strength: " << strength << std::endl
            << "Frequency: " << freq << std::endl
            << "Offset: " << offset << std::endl;
    }

    // Send command list to control thread
    {
        std::lock_guard<std::mutex> lock (commandMutex_);
        activeCommands_ = std::move(commandList);
        newCommand_.store(true, std::memory_order_release);
    }

    resetDurationTimer(duration);

    RCLCPP_INFO(this->get_logger(), "%s", logString.str().c_str());
}

/**
 * @brief Callback for Ros2 single current constant service.
 */
void OmnimagnetDriverNode::sccCallback(
    const omnimagnet_interfaces::srv::SingleCurrentConstant::Request::SharedPtr request,
    const omnimagnet_interfaces::srv::SingleCurrentConstant::Response::SharedPtr response
) {
    // Ignore command if another is currently running
    if (systemIsBusy()) {
        commandError(
            this->get_logger(),
            "Received command while executing previous command. Disregarding.",
            response,
            "Command sent while another is being executed. New command ignored.",
            timeoutTimer_
        );

        return;
    }

    // Cancel the timeout timer to prevent it from triggering during the operation
    timeoutTimer_->cancel();

    auto id = request->omnimagnet;
    auto strength = request->current_strength;
    auto vector = request->current_vec;

    std::string errorString;

    // Check if the specified magnet ID is valid
    if (!checkID(id, omnimagnets_, errorString)) {
        commandError(
            this->get_logger(),
            errorString,
            response,
            errorString,
            timeoutTimer_
        );

        return;
    }
    auto& magnet = omnimagnets_.at(id);

    auto duration = request->duration;
    if (duration <= 0.0) {
        commandError(
            this->get_logger(),
            "Duration is non-positive: " + std::to_string(duration),
            response,
            "Duration is non-positive: " + std::to_string(duration),
            timeoutTimer_
        );
        return;
    }

    Eigen::Vector3d currentVector;
    currentVector << vector.x, vector.y, vector.z;

    if (!validVector(currentVector, errorString)) {
        commandError(
            this->get_logger(),
            errorString,
            response,
            errorString,
            timeoutTimer_
        );

        return;
    }

    currentVector.normalize();
    ConstantCurrentCommand command = {&magnet, strength, currentVector};

    // Set up the active command for the specified magnet using a lock guard to ensure thread safety
    {
        std::lock_guard<std::mutex> lock(commandMutex_);
        activeCommands_.push_back(command);
        newCommand_.store(true, std::memory_order_release);
    }

    resetDurationTimer(duration);

    RCLCPP_INFO(this->get_logger(), 
        "Beginning Operation\n"
        "Single Magnet\n"
        "Mode: Constant Current\n"
        "Duration %.3f s\n",
        duration
    );

    RCLCPP_INFO(this->get_logger(), 
        "Magnet: %lu\n"
        "Current: <%.3f, %.3f, %.3f>\n"
        "Strength: %.2f",
        id, vector.x, vector.y, vector.z, strength
    );
}

/**
 * @brief Callback for Ros2 single current rotation service.
 */
void OmnimagnetDriverNode::scrCallback(
    const omnimagnet_interfaces::srv::SingleCurrentRotation::Request::SharedPtr request,
    const omnimagnet_interfaces::srv::SingleCurrentRotation::Response::SharedPtr response
) {
    // Ignore command if another is currently running
    if (systemIsBusy()) {
        commandError(
            this->get_logger(),
            "Received command while executing previous command. Disregarding.",
            response,
            "Command sent while another is being executed. New command ignored.",
            timeoutTimer_
        );

        return;
    }

    // Cancel the timeout timer to prevent it from triggering during the operation
    timeoutTimer_->cancel();

    auto id = request->omnimagnet;
    auto strength = request->current_strength;
    auto rotationVector = request->rotation_vector;
    auto offset = request->phase_offset;
    auto phaseOffset = offset * M_PI / 180.0; // Converted to radians
    auto freq = request->rotation_freq;
    auto duration = request->duration;
    
    std::string errorString;

    // Check if the specified magnet ID is valid
    if (!checkID(id, omnimagnets_, errorString)) {
        commandError(
            this->get_logger(),
            errorString,
            response,
            errorString,
            timeoutTimer_
        );

        return;
    }
    auto& magnet = omnimagnets_.at(id);

    // Check if duration is positive
    if (duration <= 0.0) {
        commandError(
            this->get_logger(),
            "Duration is non-positive: " + std::to_string(duration),
            response,
            "Duration is non-positive: " + std::to_string(duration),
            timeoutTimer_
        );
        return;
    }
    
    Eigen::Vector3d rotVec;
    rotVec << rotationVector.x, rotationVector.y, rotationVector.z;

    if (!validVector(rotVec, errorString)) {
        commandError(
            this->get_logger(),
            errorString,
            response,
            errorString,
            timeoutTimer_
        );

        return;
    }

    Basis rotationPlane = makeBasisFromRotationVector(rotVec);
    RotatingCurrentCommand command = {&magnet, freq, strength, phaseOffset, rotationPlane};

    // Set up the active command for the specified magnet using a lock guard to ensure thread safety
    {
        std::lock_guard<std::mutex> lock (commandMutex_);
        activeCommands_.push_back(command);
        newCommand_.store(true, std::memory_order_release);
    }

    resetDurationTimer(duration);

    RCLCPP_INFO(this->get_logger(), 
        "Beginning Operation\n"
        "Single Magnet\n"
        "Mode: Rotating Current\n"
        "Duration %.3f s\n",
        duration
    );

    RCLCPP_INFO(this->get_logger(), 
        "Magnet: %lu\n"
        "Rotation: <%.3f, %.3f, %.3f>\n"
        "Strength: %.2f\n"
        "Frequency: %.2f\n"
        "Offset: %.2f\n",
        id, rotationVector.x, rotationVector.y, rotationVector.z, strength, freq, offset
    );
}

/**
 * @brief Callback for Ros2 Multi-current constant service.
 */
void OmnimagnetDriverNode::mccCallback(
    const omnimagnet_interfaces::srv::MultiCurrentConstant::Request::SharedPtr request,
    const omnimagnet_interfaces::srv::MultiCurrentConstant::Response::SharedPtr response
) {
    // Ignore command if another is currently running
    if (systemIsBusy()) {
        commandError(
            this->get_logger(),
            "Received command while executing previous command. Disregarding.",
            response,
            "Command sent while another is being executed. New command ignored.",
            timeoutTimer_
        );

        return;
    }

    // Cancel the timeout timer to prevent it from triggering during the operation
    timeoutTimer_->cancel();
    
    auto ids = request->omnimagnets;
    auto strengths = request->current_strengths;
    auto vectors = request->current_vecs;

    
    std::vector<ActiveMagnetCommand> commandList;
    
    std::string errorString;
    
    if (!checkIDs(ids, omnimagnets_, errorString)) {
        commandError(
            this->get_logger(),
            errorString,
            response,
            errorString,
            timeoutTimer_
        );
        
        return;
    }
    
    if (!checkMultipleVectors(
        ids,
        errorString,
        strengths,
        vectors
    )) {
        commandError(
            this->get_logger(),
            errorString,
            response,
            errorString,
            timeoutTimer_
        );

        return;
    }

    auto duration = request->duration;
    if (duration <= 0.0) {
        commandError(
            this->get_logger(),
            "Duration is non-positive: " + std::to_string(duration),
            response,
            "Duration is non-positive: " + std::to_string(duration),
            timeoutTimer_
        );

        return;
    }

    std::stringstream logString;

    logString 
        << "Beginning Operation" << std::endl
        << "Multiple Magnets" << std::endl
        << "Mode: Constant Current" << std::endl
        << "Duration: " << duration << " s" << std::endl;

    // Set up active commands for each magnet
    for (std::size_t i = 0; i < ids.size(); ++i) {
        uint64_t id;
        omnimagnet_interfaces::msg::Vector3 vector;
        double strength;

        id = ids[i];

        // If only one vector is provided, use it for all magnets; otherwise, use the corresponding vector for each magnet
        vector = (vectors.size() == 1)
            ? vectors[0]
            : vectors[i];

        strength = (strengths.size() == 1)
            ? strengths[0]
            : strengths[i];

        auto& magnet = omnimagnets_.at(id);
        
        Eigen::Vector3d currentVector;
        currentVector << vector.x, vector.y, vector.z;

        if (!validVector(currentVector, errorString)) {
            commandError(
                this->get_logger(),
                errorString,
                response,
                errorString,
                timeoutTimer_
            );

            return;
        }

        // Normalize dipole vector to ensure it has a unit length
        currentVector.normalize();

        logString 
            << "Magnet: " << id << std::endl
            << "Current: <"
                << vector.x << ", "
                << vector.y << ", "
                << vector.z << ">" << std::endl
            << "Strength: " << strength << std::endl;

        ConstantCurrentCommand command = {&magnet, strength, currentVector};

        // Add command to the temporary command list
        commandList.push_back(command);
    }

    // Store commands for control thread
    {
        std::lock_guard<std::mutex> lock(commandMutex_);
        activeCommands_ = std::move(commandList);
        newCommand_.store(true, std::memory_order_release);
    }

    resetDurationTimer(duration);

    RCLCPP_INFO(this->get_logger(), "%s", logString.str().c_str());
}

/**
 * @brief Callback for Ros2 Multi-current rotation service
 */
void OmnimagnetDriverNode::mcrCallback(
    const omnimagnet_interfaces::srv::MultiCurrentRotation::Request::SharedPtr request,
    const omnimagnet_interfaces::srv::MultiCurrentRotation::Response::SharedPtr response
) {
    // Ignore command if another is currently running
    if (systemIsBusy()) {
        commandError(
            this->get_logger(),
            "Received command while executing previous command. Disregarding.",
            response,
            "Command sent while another is being executed. New command ignored.",
            timeoutTimer_
        );

        return;
    }
    
    // Cancel the timeout timer to prevent it from triggering during the operation
    timeoutTimer_->cancel();

    auto ids = request->omnimagnets;
    auto rotationVectors = request->rotation_vectors;
    auto freqs = request->rotation_freqs;
    auto strengths = request->current_strengths;
    auto offsets = request->phase_offsets;
    auto duration = request->duration;

    std::vector<ActiveMagnetCommand> commandList;

    std::string errorString;

    if (!checkIDs(ids, omnimagnets_, errorString)) {
        commandError(
            this->get_logger(),
            errorString,
            response,
            errorString,
            timeoutTimer_
        );

        return;
    }

    if (!checkMultipleVectors(
        ids,
        errorString,
        rotationVectors,
        freqs,
        strengths,
        offsets    
    )) {
        commandError(
            this->get_logger(),
            errorString,
            response,
            errorString,
            timeoutTimer_
        );

        return;
    }

    if (duration <= 0.0) {
        commandError(
            this->get_logger(),
            "Duration is non-positive: " + std::to_string(duration),
            response,
            "Duration is non-positive: " + std::to_string(duration),
            timeoutTimer_
        );
        return;
    }

    std::stringstream logString;

    logString 
        << "Beginning Operation" << std::endl
        << "Multiple Magnets" << std::endl
        << "Mode: Rotating Current" << std::endl
        << "Duration: " << duration << " s" << std::endl;

    // Set up active commands for each magnet
    for (std::size_t i = 0; i < ids.size(); ++i) {
        uint64_t id;
        omnimagnet_interfaces::msg::Vector3 rotationVector;
        double strength;
        double offset;
        double freq;

        id = ids[i];

        // For each list, if only one entry is provided, apply it to all, otherwise use corresponding entry
        rotationVector = (rotationVectors.size() == 1)
            ? rotationVectors[0]
            : rotationVectors[i];

        strength = (strengths.size() == 1)
            ? strengths[0]
            : strengths[i];

        freq = (freqs.size() == 1)
            ? freqs[0]
            : freqs[i];

        offset = (offsets.size() == 1)
            ? offsets[0]
            : offsets[i];

        // Convert to radians
        auto phaseOffset = offset * M_PI / 180.0;

        auto& magnet = omnimagnets_.at(id);
        
        // Convert vector to Eigen Vector3d
        Eigen::Vector3d rotVec;
        rotVec << rotationVector.x, rotationVector.y, rotationVector.z;

        if (!validVector(rotVec, errorString)) {
            commandError(
                this->get_logger(),
                errorString,
                response,
                errorString,
                timeoutTimer_
            );

            return;
        }

        // Normalize rotation vector to ensure it has a unit length
        rotVec.normalize();

        Basis rotationPlane = makeBasisFromRotationVector(rotVec);
        RotatingCurrentCommand command = {&magnet, freq, strength, phaseOffset, rotationPlane};

        // Add to temporary command list
        commandList.push_back(command);
    

        logString
            << "Magnet: " << id << std::endl
            << "Rotation: <"
                << rotationVector.x << ", "
                << rotationVector.y << ", "
                << rotationVector.z << ">" << std::endl
            << "Strength: " << strength << std::endl
            << "Frequency: " << freq << std::endl
            << "Offset: " << offset << std::endl;
    }

    // Send command list to control thread
    {
        std::lock_guard<std::mutex> lock (commandMutex_);
        activeCommands_ = std::move(commandList);
        newCommand_.store(true, std::memory_order_release);
    }

    resetDurationTimer(duration);

    RCLCPP_INFO(this->get_logger(), "%s", logString.str().c_str());
}


/**
 * @brief Callback for handling DriverReset service requests.
 * 
 * This function is called when a DriverReset service request is received.
 * It stops any ongoing experiment, resets the active command count, cancels the duration timer if it exists, and turns off all magnets.
 * If any magnet fails to turn off, it logs an error message and shuts down the system.
 * It also resets the timeout timer to wait for the next command and returns a successful response.
 */
void OmnimagnetDriverNode::resetCallback(
    [[maybe_unused]] const omnimagnet_interfaces::srv::DriverReset::Request::SharedPtr request,
    const omnimagnet_interfaces::srv::DriverReset::Response::SharedPtr response
) {
    // Cancel the duration timer if it exists
    if (durationTimer_) {
        durationTimer_->cancel();
    }

    RCLCPP_INFO(this->get_logger(), "System reset by command.");

    resetCommand_.store(true, std::memory_order_release);

    // Reset the timeout timer to wait for the next command
    timeoutTimer_->reset();

    response->status = true;
}

/****************** THREAD LOOP ************************/

/**
 * @brief Control loop for the OmnimagnetDriverNode.
 * 
 * This function runs in a separate thread and continuously checks if an experiment is running.
 * If an experiment is running, it calculates the current time and updates the currents of the active magnets based on their respective commands (constant or rotating dipoles).
 * The loop runs at a frequency specified by the "timing.control_frequency_hz" parameter and sleeps until the next control cycle.
 */
void OmnimagnetDriverNode::controlLoop() {
    using clock = std::chrono::steady_clock;

    // Get the control frequency from the parameters and calculate the control period
    double control_hz = this->get_parameter(
        "timing.control_frequency_hz").as_double();

    const auto period =
        std::chrono::duration_cast<clock::duration>(
            std::chrono::duration<double>(1. / control_hz)
        );

    auto startTime = clock::now();
    auto next = clock::now();

    std::vector<ActiveMagnetCommand> localCommands;

    // Main control loop that runs while the control thread is active
    while (controlThreadRunning_.load(std::memory_order_acquire)) {
        next += period;
        
        if (resetCommand_.load(std::memory_order_acquire)) {
            
            for (const auto& command : localCommands) {
                const auto* magnet = commandMagnet(command);
                
                if (runCurrent(Eigen::Vector3d::Zero(), *magnet) < 0) {
                    systemError(
                        this->get_logger(),
                        "Failed to shut down magnet " + std::to_string(magnet->ID()) + ". Shutting down.",
                        "comedi_data_write",
                        errorPublisher_,
                        "Failed to shut down magnet " + std::to_string(magnet->ID()) + ". Shutting down.",
                        true
                    );
                }
            }
            
            localCommands.clear();
            resetCommand_.store(false, std::memory_order_release);
            experimentRunning_.store(false, std::memory_order_release);
        }

        if (newCommand_.load(std::memory_order_acquire)) {
            // Reset start time
            startTime = clock::now();

            // Copy commands to local environment
            {
                std::lock_guard<std::mutex> lock(commandMutex_);
                localCommands = std::move(activeCommands_);
                activeCommands_.clear();
            }

            // Set experiment flag to true
            experimentRunning_.store(true, std::memory_order_release);

            // Reset newCommand flag
            newCommand_.store(false, std::memory_order_release);
        }

        if (experimentRunning_.load(std::memory_order_acquire)) {
            // Calculate the elapsed time since the start of the experiment
            double t = std::chrono::duration<double>(clock::now() - startTime).count();
            
            // Update the currents of the active magnets based on their respective commands (constant or rotating dipoles)
            for (const auto& command : localCommands) {
                const auto current = currentAtTime(command, t);
                const auto* magnet = commandMagnet(command);

                if (runCurrent(current, *magnet) < 0) {
                    systemError(
                        this->get_logger(),
                        "Failed to write current to magnet " + std::to_string(magnet->ID()),
                        "comedi_data_write",
                        errorPublisher_,
                        "Failed to write current to magnet " + std::to_string(magnet->ID()) + ". Shutting down.",
                        true
                    );
                }
            }
        }

        // Sleep until the next control cycle to maintain the desired control frequency
        std::this_thread::sleep_until(next);
    }
}

/***************** ROS Builders *****************/
/**
 * @brief Declares parameters for the OmnimagnetDriverNode.
 * 
 * This function declares various parameters for the OmnimagnetDriverNode, including hardware settings, timing configurations, and default parameters for each magnet (1-6).
 * It uses the declare_parameter method to declare parameters with their default values.
 * The parameters include hardware device, subdevice, channel, range, analog reference, inhibitor settings, 
 * timing timeout, control frequency, 
 * and magnet-specific parameters such as ID, enabled status, wire width, wire lengths, core size, channels, 
 * estimation method, and frame.
 * The function also calls the declareMagnetParameters helper function to declare parameters for each magnet with default channels.
 */
void OmnimagnetDriverNode::declareParameters() {
    // Hardware
    this->declare_parameter<std::string>(
        "hardware.device", "/dev/comedi0"
    );
    this->declare_parameter<int>(
        "hardware.subdevice", 0
    );
    this->declare_parameter<int>(
        "hardware.channel", 10
    );
    this->declare_parameter<int>(
        "hardware.range", 0
    );
    this->declare_parameter<int>(
        "hardware.analog_reference", 0
    );
    this->declare_parameter<std::vector<int>>(
        "hardware.inhibitor.pins", {25, 26}
    );
    this->declare_parameter<double>(
        "hardware.inhibitor.voltage", 5.0
    );

    // Ranges
    this->declare_parameter<int>(
        "hardware.min_sample", 0
    );
    this->declare_parameter<int>(
        "hardware.max_sample", 16383
    );
    this->declare_parameter<double>(
        "hardware.min_voltage", -10.0
    );
    this->declare_parameter<double>(
        "hardware.max_voltage", 10.0
    );
    this->declare_parameter<double>(
        "hardware.min_current", -15.0
    );
    this->declare_parameter<double>(
        "hardware.max_current", 15.0
    );

    // Timing
    this->declare_parameter<double>(
        "timing.timeout_seconds", 300.
    );
    this->declare_parameter<double>(
        "timing.control_frequency_hz", 1000.
    );

    // Default channels for magnets 1-6
    constexpr std::array<std::array<int, 3>, maxMagnets_> default_channels{{
        {{2, 0, 18}},
        {{3, 11, 19}},
        {{4, 12, 20}},
        {{5, 13, 21}},
        {{6, 14, 22}},
        {{7, 15, 23}}
    }};

    // Declare parameters for each magnet (1-6) using the default channels
    for (std::size_t i = 0; i < maxMagnets_; ++i) {
        declareMagnetParameters(i + 1, default_channels[i]);
    }
}

/**
 * @brief Declares parameters for a specific magnet.
 * 
 * This function declares the default parameters for the magnet at the given index, using the provided default channels.
 * 
 * @param index The index of the magnet for which to declare parameters.
 * @param default_channels The default channels for the magnet, specified as an array of three integers (inner, mid, outer).
 */
void OmnimagnetDriverNode::declareMagnetParameters(
    const std::size_t index,
    const std::array<int, 3>& default_channels
) {
    std::string prefix = magnetNamespace(index);

    // ID
    this->declare_parameter<int>(
        prefix + ".id",
        static_cast<int>(index)
    );

    // Enabled
    this->declare_parameter<bool>(
        prefix + ".enabled",
        false
    );

    // Width
    this->declare_parameter<double>(
        prefix + ".wire_width",
        .00135
    );

    // Length (inner, mid, outer)
    this->declare_parameter<double>(
        prefix + ".wire_lengths.inner",
        121.0
    );
    
    this->declare_parameter<double>(
        prefix + ".wire_lengths.mid",
        122.0
    );

    this->declare_parameter<double>(
        prefix + ".wire_lengths.outer",
        132.0
    );

    // Core_size
    this->declare_parameter<double>(
        prefix + ".core_size",
        17.0
    );

    // Channels (inner, mid, outer)
    this->declare_parameter<int>(
        prefix + ".channels.inner",
        default_channels[0]
    );

    this->declare_parameter<int>(
        prefix + ".channels.mid",
        default_channels[1]
    );

    this->declare_parameter<int>(
        prefix + ".channels.outer",
        default_channels[2]
    );

    // Estimation method
    this->declare_parameter<bool>(
        prefix + ".estimate",
        true
    );

    // Frame
    this->declare_parameter<std::vector<double>>(
        prefix + ".frame",
        {
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        }
    );
}

/**
 * @brief Loads the configuration for a specific magnet.
 * 
 * This function retrieves the values of the parameters for the magnet at the given index from the ROS parameter server and stores them in a MagnetConfig struct.
 * 
 * @param index The index of the magnet for which to load configuration.
 * @return The configuration for the specified magnet.
 */
MagnetConfig OmnimagnetDriverNode::loadMagnetConfig(const std::size_t index) const {
    std::string prefix = magnetNamespace(index);

    MagnetConfig config;

    config.id = 
        this->get_parameter(prefix + ".id").as_int();

    config.enabled = 
        this->get_parameter(prefix + ".enabled").as_bool();

    config.wireWidth =
        this->get_parameter(prefix + ".wire_width").as_double();

    config.innerWireLength =
        this->get_parameter(prefix + ".wire_lengths.inner").as_double();

    config.midWireLength =
        this->get_parameter(prefix + ".wire_lengths.mid").as_double();

    config.outerWireLength =
        this->get_parameter(prefix + ".wire_lengths.outer").as_double();

    config.coreSize =
        this->get_parameter(prefix + ".core_size").as_double();

    config.innerChannel =
        this->get_parameter(prefix + ".channels.inner").as_int();

    config.midChannel = 
        this->get_parameter(prefix + ".channels.mid").as_int();

    config.outerChannel =
        this->get_parameter(prefix + ".channels.outer").as_int();

    config.estimate =
        this->get_parameter(prefix + ".estimate").as_bool();

    config.frame = 
        this->get_parameter(prefix + ".frame").as_double_array();

    return config;
}

/**
 * @brief Builds the timers for the OmnimagnetDriverNode.
 * 
 * This function creates two timers: a timeout timer and a duration timer.
 * The duration timer is initially canceled and will be started when an experiment is run.
 */
void OmnimagnetDriverNode::buildTimers() {
    timeout_ = this->get_parameter("timing.timeout_seconds").as_double();

    timeoutTimer_ = this->create_wall_timer(
        std::chrono::duration<double>(timeout_),
        std::bind(&OmnimagnetDriverNode::timeoutCallback, this)
    );

    durationTimer_ = this->create_wall_timer(
        std::chrono::duration<double>(0.0),
        std::bind(&OmnimagnetDriverNode::durationCallback, this)
    );
    durationTimer_->cancel(); // Hold timer until experiment run
}

/**
 * @brief Resets the driver's duration timer
 * 
 * Takes the durationTimer pointer and creates a new wall timer for it with the assigned duration, binding it to the appropriate callback.
 * 
 * @param duration The duration of the new timer.
 * 
 * @note If the duration timer is somehow in progress when this function is invoked, it will cancel and then reset it.
 * Any operations in progress will not be canceled.
 */
void OmnimagnetDriverNode::resetDurationTimer(const double duration) {
    if (durationTimer_) {
        durationTimer_->cancel();
    }

    durationTimer_ = this->create_wall_timer(
        std::chrono::duration<double>(duration),
        std::bind(&OmnimagnetDriverNode::durationCallback, this)
    );
}

/**
 * @brief Builds the publishers for the OmnimagnetDriverNode.
 * 
 * This function creates two publishers: an error publisher and a finished publisher. The error publisher is used to publish error messages, 
 * while the finished publisher is used to publish messages when an experiment is finished.
 */
void OmnimagnetDriverNode::buildPublishers() {
    this->errorPublisher_ = 
        this->create_publisher<omnimagnet_interfaces::msg::ErrorMessage>("driver_errors", 10);
    this->finishedPublisher_ = 
        this->create_publisher<omnimagnet_interfaces::msg::FinishedMessage>("driver_finished", 10);
}

/**
 * @brief Builds the services for the OmnimagnetDriverNode.
 * 
 * This function creates four services: single_magnet_constant, single_magnet_rotation, multi_magnet_constant, and multi_magnet_rotation.
 * Each service is associated with a callback function that handles the corresponding service request.
 * The services allow clients to send requests to control the omnimagnets in different modes (constant or rotating dipoles) for single or multiple magnets.
 */
void OmnimagnetDriverNode::buildServices() {
    smcServer_ = this->create_service<omnimagnet_interfaces::srv::SingleMagnetConstant>(
        "single_magnet_constant",
        std::bind(
            &OmnimagnetDriverNode::smcCallback,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );
    smrServer_ = this->create_service<omnimagnet_interfaces::srv::SingleMagnetRotation>(
        "single_magnet_rotation",
        std::bind(
            &OmnimagnetDriverNode::smrCallback,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );
    mmcServer_ = this->create_service<omnimagnet_interfaces::srv::MultiMagnetConstant>(
        "multi_magnet_constant",
        std::bind(
            &OmnimagnetDriverNode::mmcCallback,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );
    mmrServer_ = this->create_service<omnimagnet_interfaces::srv::MultiMagnetRotation>(
        "multi_magnet_rotation",
        std::bind(
            &OmnimagnetDriverNode::mmrCallback,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

    sccServer_ = this->create_service<omnimagnet_interfaces::srv::SingleCurrentConstant>(
        "single_current_constant",
        std::bind(
            &OmnimagnetDriverNode::sccCallback,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );
    scrServer_ = this->create_service<omnimagnet_interfaces::srv::SingleCurrentRotation>(
        "single_current_rotation",
        std::bind(
            &OmnimagnetDriverNode::scrCallback,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );
    mccServer_ = this->create_service<omnimagnet_interfaces::srv::MultiCurrentConstant>(
        "multi_current_constant",
        std::bind(
            &OmnimagnetDriverNode::mccCallback,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );
    mcrServer_ = this->create_service<omnimagnet_interfaces::srv::MultiCurrentRotation>(
        "multi_current_rotation",
        std::bind(
            &OmnimagnetDriverNode::mcrCallback,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

    resetServer_ = this->create_service<omnimagnet_interfaces::srv::DriverReset>(
        "reset_driver",
        std::bind(&OmnimagnetDriverNode::resetCallback,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );
}

/***************** Setting Current *****************/

/**
 * @brief Converts a current value to the corresponding D2A value for the OmniMagnet object.
 * 
 * This method takes a current value (double) as input and calculates the corresponding D2A value (lsampl_t).
 * The conversion is based on the maximum D2A value set for the OmniMagnet object.
 * 
 * @param current A double representing the desired current value in A.
 * 
 * @return An lsampl_t representing the corresponding D2A value.
 */
lsampl_t OmnimagnetDriverNode::currentD2A(double current) {
	// Range was interpreted from original code.
	return mapRange<double, lsampl_t>(current, minCurrent_, maxCurrent_, minSample_, maxSample_);
}

/**
 * @brief Sets the current for the OmniMagnet object.
 * 
 * This method sets the current values for an OmniMagnet object and send the current request to the corresponding D2A pins.
 * Current is represented as a 3x1 Eigen vector, where each component corresponds to the current in the x, y, and z directions.
 * 
 * @param current A 3x1 Eigen vector representing the desired current values.
 * @param magnet The omnimagnet being written to.
 * 
 * @return An integer indicating the success or failure of the operation. 
 * A value of 1 indicates success, while a negative value indicates an error in writing to the D2A channels.
 * 
 * @note If any of the writes fails for any reason, all components will be reset to 0 current.
 */
int OmnimagnetDriverNode::runCurrent(const Eigen::Vector3d& current, const OmniMagnet& magnet) {
    lsampl_t xCurrent = currentD2A(current[0]);
    lsampl_t yCurrent = currentD2A(current[1]);
    lsampl_t zCurrent = currentD2A(current[2]);

    auto pins = magnet.pinNumbers();

	// Write the D2A values to the corresponding channels using comedi_data_write. 
	// Check for errors after each write
    int retval;
    retval = comedi_data_write(card_, subDevice_, pins[0], range_, aref_, xCurrent);
    if (retval < 0)
		goto fail_state1;

    retval = comedi_data_write(card_, subDevice_, pins[1], range_, aref_, yCurrent);
    if (retval < 0)
		goto fail_state2;

    retval = comedi_data_write(card_, subDevice_, pins[2], range_, aref_, zCurrent);
    if (retval < 0)
		goto fail_state3;

	return 1;

	// If any write fails, reset the previous channels to 0 to avoid leaving the magnet in an undefined state.
	fail_state3:
		comedi_data_write(card_, subDevice_, pins[2], range_, aref_, currentD2A(0));
	fail_state2:
		comedi_data_write(card_, subDevice_, pins[1], range_, aref_, currentD2A(0));
	fail_state1:
		comedi_data_write(card_, subDevice_, pins[0], range_, aref_, currentD2A(0));
		
		return retval;
}

/**
 * @brief Checks if the system is running a previous command.
 * 
 * @return true if the system is already running or processing a command.
 */
bool OmnimagnetDriverNode::systemIsBusy() {
    return 
        experimentRunning_.load(std::memory_order_acquire) ||
        newCommand_.load(std::memory_order_acquire);
}