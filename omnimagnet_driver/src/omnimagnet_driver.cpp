# include "rclcpp/rclcpp.hpp"
# include "../include/omnimagnet_driver/omnimagnet.hpp"
# include "comedilib.hpp"
# include "../include/omnimagnet_driver/omnimagnet_driver.hpp"

# include <atomic>
# include <chrono>
# include <thread>
# include <vector>
# include <map>
# include <iomanip>

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
}

/**
 * @brief Main function for the Omnimagnet Driver Node.
 * 
 * This function initializes the ROS2 node, creates an instance of the OmnimagnetDriverNode,
 * binds the shutdown method to the signal handler for safe program exit, and starts spinning the node.
 * 
 * @param argc The number of command line arguments.
 * @param argv The array of command line arguments.
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
    Node ("omnimagnet_driver") {

    offVector << 0.0, 0.0, 0.0;

    declareParameters();
    loadParameters();
    buildTimers();
    buildPublishers();
    buildServices();
    setupHardware();
    setupMagnets();

    controlThreadRunning.store(true, std::memory_order_release);
    controlThread = std::thread(&OmnimagnetDriverNode::controlLoop, this);
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
    auto subdev = this->get_parameter("hardware.subdevice").as_int();
    auto aref   = this->get_parameter("hardware.analog_reference").as_int();
    auto device = this->get_parameter("hardware.device").as_string();
    auto pct    = this->get_parameter("hardware.inhibitor.percent").as_double();
    // Currently unused - Parameters were used in previous implementations, unsure of ultimate purpose, potential deletion candidates
    // auto chan   = this->get_parameter("hardware.channel").as_int();  // Currently unused - unsure of purpose
    // auto range  = this->get_parameter("hardware.range").as_int();    // Currently unused - comedi_get_maxdata() is used to determine range
    
    auto inhibs = this->get_parameter("hardware.inhibitor.pins").as_integer_array();
    // This check might be redundant, as the parameter is declared with a size of 2, but it is kept for safety.
    // If hardware changes, this might need to be updated.
    if (inhibs.size() != 2) {
        RCLCPP_ERROR(this->get_logger(), "Need to specify two inhibitor pins.");
        rclcpp::shutdown();
    }
    auto inhbp1 = inhibs.at(0); 
    auto inhbp2 = inhibs.at(1);

    // Open the D2A device
    D2A = comedi_open(device.c_str());
    if(D2A == nullptr) {
        systemError(
            this->get_logger(),
            "Failed to open D2A:",
            "comedi_open",
            errorPublisher,
            "Failed to open D2A device.\n",
            true
        );
    }

    // Setting amplifier inhibitors at 75%. Default pins are 25 & 26
    this->maxdata1  = comedi_get_maxdata(D2A, subdev, inhbp1);
    this->maxdata2  = comedi_get_maxdata(D2A, subdev, inhbp2);
    lsampl_t inhib1 = maxdata1 * pct;
    lsampl_t inhib2 = maxdata2 * pct;

    // Inhibitor pin 1
    int r25 = comedi_data_write(D2A, subdev, inhbp1, 0, aref, inhib1);
    if (r25 < 0) {
        systemError(
            this->get_logger(),
            "Failed to inhibit pin " + std::to_string(inhbp1) + ":",
            "comedi_data_write",
            errorPublisher,
            "Failed to set inhibitor on D2A pin" + std::to_string(inhbp1) + ".\n",
            true
        );

        return;
    }

    // Inhibitor pin 2
    int r26 = comedi_data_write(D2A, subdev, inhbp2, 0, aref, inhib2);
    if (r26 < 0) {
        systemError(
            this->get_logger(),
            "Failed to inhibit pin " + std::to_string(inhbp2) + ":",
            "comedi_data_write",
            errorPublisher,
            "Failed to set inhibitor on D2A pin" + std::to_string(inhbp2) + ".\n",
            true
        );

        return;
    }

    RCLCPP_INFO(this->get_logger(), "Hardware setup complete.");
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
 * @throws std::runtime_error if a magnet configuration is invalid or if there are duplicate magnet IDs.
 * 
 * TODO: Consider replacing runtime throw with a more graceful error handling mechanism, such as logging the error and continuing with the next magnet.
 */
void OmnimagnetDriverNode::setupMagnets() {
    // Max magnets is currently hard-coded as a static constant in omnimagnet_driver.hpp, 
    // to allow for fixed-size command arrays to increase performance. This might be unnecessary. 
    for (std::size_t i =1; i < maxMagnets; ++i) {
        const MagnetConfig config = loadMagnetConfig(i);

        if (!config.enabled)
            continue;

        // Frame must be a 3x3 matrix, so it should have exactly 9 values
        if (config.frame.size() != 9) {
            throw std::runtime_error(
                magnetNamespace(i) + ".frame must contain exactly 9 values (3x3 matrix)"
            );
        }

        auto [iterator, inserted] =
            omnimagnets.try_emplace(config.id);

        // Check if the same ID is loaded twice
        if (!inserted) {
            throw std::runtime_error(
                "Duplicate magnet ID: " + std::to_string(config.id)
            );
        }

        OmniMagnet& magnet = iterator->second;

        magnet.SetProp(
            config.wire_width,
            config.inner_wire_length,
            config.mid_wire_length,
            config.outer_wire_length,
            config.core_size,
            config.inner_channel,
            config.mid_channel,
            config.outer_channel,
            config.estimate,
            D2A
        );

        magnet.SetFrame(config.frame);
        magnet.UpdateMapping();
        magnet.setD2AMax(this->maxdata1);
        magnet.ID = config.id;

        RCLCPP_INFO(this->get_logger(), "Configured magnet %d from parameter index %zu", config.id, i);
    }

    RCLCPP_INFO(this->get_logger(), "Magnet setup complete: %zu magnet(s) enabled.", omnimagnets.size());
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
    experimentRunning.store(false, std::memory_order_release);
    controlThreadRunning.store(false, std::memory_order_release);
    if (controlThread.joinable()) {
        controlThread.join();
    }

    // D2A should have been initialized, but check for safety.
    if (D2A == nullptr) {
        std::cerr << "D2A was null during shutdown:" << std::endl <<
        "Hardware may not have been properly initialized" << std::endl;

        return;
    }

    // Turn off all magnets. Log any failures, but continue to attempt to turn off all magnets.
    bool magnetFail = false;
    for (auto & [id, omni] : omnimagnets) {
        int retval = omni.SetCurrent(offVector);

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

    // Shut down amplifier using inhibitors. Logs failures and continues.
    auto inhibs = this->get_parameter("hardware.inhibitor.pins").as_integer_array();
    auto inhbp1 = inhibs.at(0); 
    auto inhbp2 = inhibs.at(1);
    auto subdev = this->get_parameter("hardware.subdevice").as_int();
    auto ground = this->get_parameter("hardware.analog_reference").as_int();

    bool pinFail = false;
    int retval = comedi_data_write(D2A, subdev, inhbp1, 0, ground, maxdata1*2./4.);
    if (retval < 0) {
        std::cerr << "Failed to shut down D2A Pin " << std::to_string(inhbp1) << std::endl;
        pinFail = true;
    }
    retval = comedi_data_write(D2A, subdev, inhbp2, 0, ground, maxdata2*2./4.);
    if (retval < 0) {
        std::cerr << "Failed to shut down D2A Pin " << std::to_string(inhbp2) << std::endl;
        pinFail = true;
    }

    if (!pinFail) {
        std::cout << "Successfully shut down write pins." << std::endl;
    }
    else {
        std::cerr << "Failed to shut down write pins, use emergency stop!" << std::endl;
    }
}

/*************** TIMER CALLBACKS ***************/

/** @brief Callback for handling timeout events.
  * 
  * This function is called when a timeout event occurs.
  * It logs an error message and shuts down the system.
  * 
  * TODO: systemError might be too aggressive, consider changing to a warning and a shutdown.
  */
void OmnimagnetDriverNode::timeoutCallback() {
    systemError(
        this->get_logger(),
        "Connection timed out.",
        "timeoutCallback",
        errorPublisher,
        "Controller timed out waiting for command. Shutting down.",
        true
    );
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
    durationTimer->cancel();

    // End experiment and reset command count
    experimentRunning.store(false, std::memory_order_release);
    activeCommandCount.store(0, std::memory_order_release);

    // Shutdown magnets
    for (auto& [id, omni] : omnimagnets) {
        int retval = omni.SetCurrent(offVector);
        if (retval <= 0) {
            systemError(
                this->get_logger(),
                "Failed to shut down magnet " + std::to_string(id) + ":",
                "omni.SetCurrent",
                errorPublisher,
                "Failed to shut down magnet" + std::to_string(id) + ".\n",
                true
            );
        }
    }

    auto msg = omnimagnet_interfaces::msg::FinishedMessage();

    // Current timestamp
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);

    std::ostringstream oss;
    oss << "Run finished at: "
        << std::put_time(std::localtime(&now_c), "%Y-%m-%d %H:%M:%S");

    msg.msg = oss.str();

    RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());

    finishedPublisher->publish(msg);

    // Reset timeout timer to wait for next command
    timeoutTimer->reset();
}

/*************** SERVER CALLBACKS ***************/

/**
 * @brief Callback for handling SingleMagnetConstant service requests.
 * 
 * This function is called when a SingleMagnetConstant service request is received.
 * It checks if an experiment is already running, and if so, it logs a warning and returns an error response.
 * If no experiment is running, it cancels the timeout timer and retrieves the request parameters, including the magnet ID, dipole strength, and dipole vector.
 * It checks if the specified magnet ID is valid, and if not, it logs a warning and returns an error response.
 * It also checks if the dipole vector is valid (non-zero and finite), and if not, it logs a warning and returns an error response.
 * If all checks pass, it sets up the active command for the specified magnet, starts the experiment, and creates a duration timer for the specified duration.
 * It logs the details of the operation and publishes a FinishedMessage when the operation is complete.
 */
void OmnimagnetDriverNode::smcCallback(
    const omnimagnet_interfaces::srv::SingleMagnetConstant::Request::SharedPtr request,
    const omnimagnet_interfaces::srv::SingleMagnetConstant::Response::SharedPtr response
    ) 
{
    // Check if an experiment is already running
    if (experimentRunning.load(std::memory_order_acquire)) {
        commandError(
            this->get_logger(), 
            "Server tried to start experiment while already in operation.", 
            response,
            "Operation in progress. Please reset before invoking another operation.",
            timeoutTimer
        );

        return;
    }

    // Cancel the timeout timer to prevent it from triggering during the operation
    timeoutTimer->cancel();

    auto id = request->omnimagnet;
    auto strength = request->dipole_strength;
    auto vector = request->dipole_vec;

    // Check if the specified magnet ID is valid
    if (omnimagnets.count(id) == 0) {
        // TODO: Replace with commandError to avoid code duplication
        RCLCPP_WARN(this->get_logger(), "Omnimagnet %lu not found: ", id);

        response->error = true;
        response->error_desc = "Invalid magnet ID.";

        timeoutTimer->reset();
        return;
    }

    // Optional duration argument
    auto duration = request->duration;
    if (duration <= 0.0) {
        RCLCPP_INFO(this->get_logger(), "0 or negative duration passed.");
        return;
    }

    OmniMagnet& omni = omnimagnets[id];

    Eigen::Vector3d dipole_vector;
    dipole_vector << vector.x, vector.y, vector.z;

    // Check if the dipole vector is valid (non-zero and finite)
    if (dipole_vector.norm() < 1e-8) {
        commandError(
            this->get_logger(), 
            "User attempted to pass zero vector.", 
            response,
            "Zero rotation vector passed.",
            timeoutTimer
        );

        return;
    }
    if (!dipole_vector.allFinite()) {
        commandError(
            this->get_logger(), 
            "User attempted to pass NaN.", 
            response,
            "NaN component.",
            timeoutTimer
        );

        return;
    }

    // Normalize dipole vector to ensure it has a unit length
    dipole_vector.normalize();

    // Set up the active command for the specified magnet using a lock guard to ensure thread safety
    {
        std::lock_guard<std::mutex> lock(commandMutex);
        activeCommands[0] = {&omni, 0, strength, 0, makeBasis(dipole_vector), dipole_vector};
    }

    // Start the experiment and set the active command count to 1
    startTime = std::chrono::steady_clock::now();
    activeCommandCount.store(1, std::memory_order_release);
    experimentRunning.store(true, std::memory_order_release);

    // Create a duration timer for the specified duration, canceling any existing duration timer if it exists
    // (This should not happen, but is a safety check)
    if (durationTimer) {
        durationTimer->cancel();
    }
    durationTimer = this->create_wall_timer(
        std::chrono::duration<double>(duration),
        std::bind(&OmnimagnetDriverNode::durationCallback, this)
    );

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
 * It checks if an experiment is already running, and if so, it logs a warning and returns an error response.
 * If no experiment is running, it cancels the timeout timer and retrieves the request parameters, including the magnet ID, dipole strength, rotation vector, phase offset, and rotation frequency.
 * It checks if the specified magnet ID is valid, and if not, it logs a warning and returns an error response.
 * It also checks if the rotation vector is valid (non-zero and finite), and if not, it logs a warning and returns an error response.
 * If all checks pass, it sets up the active command for the specified magnet, starts the experiment, and creates a duration timer for the specified duration.
 * It logs the details of the operation and publishes a FinishedMessage when the operation is complete.
 */
void OmnimagnetDriverNode::smrCallback(
    const omnimagnet_interfaces::srv::SingleMagnetRotation::Request::SharedPtr request,
    const omnimagnet_interfaces::srv::SingleMagnetRotation::Response::SharedPtr response
) {
    // Check if an experiment is already running
    // TODO: Replace with a commandError to avoid code duplication
    if (experimentRunning.load(std::memory_order_acquire)) {
        RCLCPP_WARN(this->get_logger(), "Server tried to start experiment while already in operation.");

        response->error = true;
        response->error_desc = "Operation in progress. Please reset before invoking another operation.";

        timeoutTimer->reset();
        return;
    }

    // Cancel the timeout timer to prevent it from triggering during the operation
    timeoutTimer->cancel();

    auto id = request->omnimagnet;
    auto strength = request->dipole_strength;
    auto rotationVector = request->rotation_vector;
    auto offset = request->phase_offset;
    auto phaseOffset = offset * M_PI / 180.0;
    auto freq = request->rotation_freq;

    // Check if the specified magnet ID is valid
    if (omnimagnets.count(id) == 0) {
        // TODO: Replace with commandError to avoid code duplication
        RCLCPP_WARN(this->get_logger(), "Omnimagnet %lu not found: ", id);

        response->error = true;
        response->error_desc = "Invalid magnet ID: " + std::to_string(id);

        timeoutTimer->reset();
        return;
    }

    // Optional argument
    auto duration = request->duration;
    if (duration <= 0.0) {
        RCLCPP_INFO(this->get_logger(), "0 or negative duration passed.");
        return;
    }

    auto omni = &omnimagnets[id];
    
    Eigen::Vector3d rotVec;
    rotVec << rotationVector.x, rotationVector.y, rotationVector.z;

    // Check if the rotation vector is valid (non-zero and finite)
    if (rotVec.norm() < 1e-8) {
        commandError(
            this->get_logger(), 
            "User attempted to pass zero vector.", 
            response,
            "Zero rotation vector passed.",
            timeoutTimer
        );

        return;
    }
    if (!rotVec.allFinite()) {
        commandError(
            this->get_logger(), 
            "User attempted to pass NaN.", 
            response,
            "NaN component.",
            timeoutTimer
        );

        return;
    }

    // Normalize rotation vector to ensure it has a unit length
    rotVec.normalize();

    // Set up the active command for the specified magnet using a lock guard to ensure thread safety
    {
        std::lock_guard<std::mutex> lock (commandMutex);
        activeCommands[0] = {omni, freq, strength, phaseOffset, makeBasis(rotVec), rotVec};
    }

    // Start the experiment and set the active command count to 1
    startTime = std::chrono::steady_clock::now();
    activeCommandCount.store(1, std::memory_order_release);
    experimentRunning.store(true, std::memory_order_release);

    // Create a duration timer for the specified duration, canceling any existing duration timer if it exists
    // (This should not happen, but is a safety check)
    // TODO: Consider moving this to a separate function to avoid code duplication 
    if (durationTimer) {
        durationTimer->cancel();
    }
    durationTimer = this->create_wall_timer(
        std::chrono::duration<double>(duration),
        std::bind(&OmnimagnetDriverNode::durationCallback, this)
    );

    RCLCPP_INFO(this->get_logger(), 
        "Beginning Operation\n"
        "Single Magnet\n"
        "Mode: Rotating Dipole\n"
        "Duration %.3f s\n",
        duration
    );

    RCLCPP_INFO(this->get_logger(), 
        "Magnet: %lu\n"
        "Dipole: <%.3f, %.3f, %.3f>\n"
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
 * It checks if an experiment is already running, and if so, it logs a warning and returns an error response.
 * If no experiment is running, it cancels the timeout timer and retrieves the request parameters, including the magnet IDs, dipole strengths, and dipole vectors.
 * It checks for size mismatches in the request parameters and validates each magnet ID.
 * It also checks if the dipole vectors are valid (non-zero and finite), and if not, it logs a warning and returns an error response.
 * If all checks pass, it sets up the active commands for the specified magnets, starts the experiment, and creates a duration timer for the specified duration.
 * It logs the details of the operation and publishes a FinishedMessage when the operation is complete.
 */
void OmnimagnetDriverNode::mmcCallback(
    const omnimagnet_interfaces::srv::MultiMagnetConstant::Request::SharedPtr request,
    const omnimagnet_interfaces::srv::MultiMagnetConstant::Response::SharedPtr response
)
{
    // Check if an experiment is already running
    if (experimentRunning.load(std::memory_order_acquire)) {
        // TODO: Replace with a commandError to avoid code duplication
        RCLCPP_WARN(this->get_logger(), "Server tried to start experiment while already in operation.");

        response->error = true;
        response->error_desc = "Operation in progress. Please reset before invoking another operation.";

        timeoutTimer->reset();
        return;
    }

    // Cancel the timeout timer to prevent it from triggering during the operation
    timeoutTimer->cancel();
    
    auto ids = request->omnimagnets;
    auto strengths = request->dipole_strengths;
    auto vectors = request->dipole_vecs;

    // Check for size mismatches
    if (ids.size() < 1) {
        commandError(
            this->get_logger(), 
            "No omnimagnets selected.", 
            response,
            "No omnimagnets selected.",
            timeoutTimer
        );
    }
    if (ids.size() > maxMagnets) {
        commandError(
            this->get_logger(), 
            "Too many magnets selected.", 
            response,
            "Too many magnets selected.",
            timeoutTimer
        );
    }
    if (strengths.size() != 1 && strengths.size() != ids.size()) {
        commandError(
            this->get_logger(), 
            "Strength size mismatch.", 
            response,
            "Dipole strengths list size mismatch.",
            timeoutTimer
        );

        return;
    }
    if (vectors.size() != 1 && vectors.size() != ids.size()) {
        commandError(
            this->get_logger(), 
            "Vector size mismatch.", 
            response,
            "Dipole vectors list size mismatch.",
            timeoutTimer
        );

        return;
    }

    for (auto& id : ids) {
        if (omnimagnets.count(id) == 0) {
            commandError(
                this->get_logger(), 
                "Omnimagnet not found.", 
                response,
                "Invalid magnet ID: " + std::to_string(id),
                timeoutTimer
            );
    
            timeoutTimer->reset();
            return;
        }
    }

    // Optional argument
    auto duration = request->duration;
    if (duration <= 0.0) {
        RCLCPP_INFO(this->get_logger(), "0 or negative duration passed.");
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
        if (vectors.size() == 1)
            vector = vectors[0];
        else
            vector = vectors[i];

        if (strengths.size() == 1)
            strength = strengths[0];
        else
            strength = strengths[i];

        auto& omni = omnimagnets[id];
        
        Eigen::Vector3d dipole_vector;
        dipole_vector << vector.x, vector.y, vector.z;

        // Check if the dipole vector is valid (non-zero and finite)
        if (dipole_vector.norm() < 1e-8) {
            commandError(
                this->get_logger(), 
                "User attempted to pass zero vector.", 
                response,
                "Zero rotation vector passed.",
                timeoutTimer
            );

            return;
        }
        if (!dipole_vector.allFinite()) {
            commandError(
                this->get_logger(), 
                "User attempted to pass NaN.", 
                response,
                "NaN component.",
                timeoutTimer
            );

            return;
        }

        // Normalize dipole vector to ensure it has a unit length
        dipole_vector.normalize();

        logString 
            << "Magnet: " << id << std::endl
            << "Dipole: <"
                << vector.x << ", "
                << vector.y << ", "
                << vector.z << ">" << std::endl
            << "Strength: " << strength << std::endl;

        // Set up the active command for the specified magnet using a lock guard to ensure thread safety
        {
            std::lock_guard<std::mutex> lock(commandMutex);
            activeCommands[i] = {&omni, 0, strength, 0, makeBasis(dipole_vector), dipole_vector};
        }
    }

    RCLCPP_INFO(this->get_logger(), "%s", logString.str().c_str());

    // Start the experiment and set the active command count to the number of magnets
    startTime = std::chrono::steady_clock::now();
    activeCommandCount.store(ids.size(), std::memory_order_release);
    experimentRunning.store(true, std::memory_order_release);

    // Create a duration timer for the specified duration, canceling any existing duration timer if it exists
    // (This should not happen, but is a safety check)
    // TODO: Consider moving this to a separate function to avoid code duplication
    if (durationTimer) {
        durationTimer->cancel();
    }
    durationTimer = this->create_wall_timer(
        std::chrono::duration<double>(duration),
        std::bind(&OmnimagnetDriverNode::durationCallback, this)
    );
}

/**
 * @brief Callback for handling MultiMagnetRotation service requests.
 * 
 * This function is called when a MultiMagnetRotation service request is received.
 * It checks if an experiment is already running, and if so, it logs a warning and returns an error response.
 * If no experiment is running, it cancels the timeout timer and retrieves the request parameters, including the magnet IDs, dipole strengths, rotation vectors, phase offsets, and rotation frequencies.
 * It checks for size mismatches in the request parameters and validates each magnet ID.
 * It also checks if the rotation vectors are valid (non-zero and finite), and if not, it logs a warning and returns an error response.
 * If all checks pass, it sets up the active commands for the specified magnets, starts the experiment, and creates a duration timer for the specified duration.
 * It logs the details of the operation and publishes a FinishedMessage when the operation is complete
 */
void OmnimagnetDriverNode::mmrCallback(
    const omnimagnet_interfaces::srv::MultiMagnetRotation::Request::SharedPtr request,
    const omnimagnet_interfaces::srv::MultiMagnetRotation::Response::SharedPtr response
) {
    // Check if an experiment is already running
    if (experimentRunning.load(std::memory_order_acquire)) {
        // TODO: Replace with a commandError to avoid code duplication
        RCLCPP_WARN(this->get_logger(), "Server tried to start experiment while already in operation.");

        response->error = true;
        response->error_desc = "Operation in progress. Please reset before invoking another operation.";

        timeoutTimer->reset();
        return;
    }

    // Cancel the timeout timer to prevent it from triggering during the operation
    timeoutTimer->cancel();

    auto ids = request->omnimagnets;
    auto rotationVectors = request->rotation_vectors;
    auto freqs = request->rotation_freqs;
    auto strengths = request->dipole_strengths;
    auto offsets = request->phase_offsets;

    // Check for size mismatches
    if (ids.size() < 1) {
        commandError(
            this->get_logger(), 
            "No omnimagnets selected.", 
            response,
            "No omnimagnets selected.",
            timeoutTimer
        );

        return;
    }
    if (ids.size() > maxMagnets) {
        commandError(
            this->get_logger(), 
            "Too many magnets selected.", 
            response,
            "Too many magnets selected.",
            timeoutTimer
        );

        return;
    }
    if (strengths.size() != 1 && strengths.size() != ids.size()) {
        commandError(
            this->get_logger(), 
            "Strength size mismatch.", 
            response,
            "Dipole strengths list size mismatch.",
            timeoutTimer
        );

        return;
    }
    if (freqs.size() != 1 && freqs.size() != ids.size()) {
        commandError(
            this->get_logger(), 
            "Frequency size mismatch.", 
            response,
            "Frequency list size mismatch.",
            timeoutTimer
        );

        return;
    }
    if (offsets.size() != 1 && offsets.size() != ids.size()) {
        commandError(
            this->get_logger(), 
            "Offset size mismatch.", 
            response,
            "Offset list size mismatch.",
            timeoutTimer
        );

        return;
    }
    if (rotationVectors.size() != 1 && rotationVectors.size() != ids.size()) {
        commandError(
            this->get_logger(), 
            "Rotation vectors size mismatch.", 
            response,
            "Rotation vectors list size mismatch.",
            timeoutTimer
        );

        return;
    }

    for (auto& id : ids) {
        if (omnimagnets.count(id) == 0) {
            commandError(
                this->get_logger(), 
                "Omnimagnet not found.", 
                response,
                "Invalid magnet ID: " + std::to_string(id),
                timeoutTimer
            );

            return;
        }
    }

    // Optional duration argument
    auto duration = request->duration;
    if (duration <= 0.0) {
        RCLCPP_INFO(this->get_logger(), "0 or negative duration passed.");
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

        // If only one rotation vector is provided, use it for all magnets; otherwise, use the corresponding rotation vector for each magnet
        if (rotationVectors.size() == 1)
            rotationVector = rotationVectors[0];
        else
            rotationVector = rotationVectors[i];

        if (strengths.size() == 1)
            strength = strengths[0];
        else
            strength = strengths[i];
            
        if (freqs.size() == 1)
            freq = freqs[0];
        else
            freq = freqs[i];

        if (offsets.size() == 1)
            offset = offsets[0];
        else
            offset = offsets[i];

        // Convert to radians
        auto phaseOffset = offset * M_PI / 180.0;

        auto omni = &omnimagnets[id];
        
        Eigen::Vector3d rotVec;
        rotVec << rotationVector.x, rotationVector.y, rotationVector.z;

        // Check if the rotation vector is valid (non-zero and finite)
        if (rotVec.norm() < 1e-8) {
            commandError(
                this->get_logger(),
                "Zero rotation vector.",
                response,
                "Zero rotation vector passed.",
                timeoutTimer
            );

            return;
        }
        if (!rotVec.allFinite()) {
            commandError(
                this->get_logger(),
                "Invalid rotation vector.",
                response,
                "NaN component in rotation vector.",
                timeoutTimer
            );

            return;
        }

        // Normalize rotation vector to ensure it has a unit length
        rotVec.normalize();
        
        // Set up the active command for the specified magnet using a lock guard to ensure thread safety
        {
            std::lock_guard<std::mutex> lock (commandMutex);
            activeCommands[i] = {omni, freq, strength, phaseOffset, makeBasis(rotVec), rotVec};
        }

        logString
            << "Magnet: " << id << std::endl
            << "Dipole: <"
                << rotationVector.x << ", "
                << rotationVector.y << ", "
                << rotationVector.z << ">" << std::endl
            << "Strength: " << strength << std::endl
            << "Frequency: " << freq << std::endl
            << "Offset: " << offset << std::endl;
    }

    RCLCPP_INFO(this->get_logger(), logString.str().c_str());

    // Start the experiment and set the active command count to the number of magnets
    startTime = std::chrono::steady_clock::now();
    activeCommandCount.store(ids.size(), std::memory_order_release);
    experimentRunning.store(true, std::memory_order_release);

    // Create a duration timer for the specified duration, canceling any existing duration timer if it exists
    // (This should not happen, but is a safety check)
    if (durationTimer) {
        durationTimer->cancel();
    }
    durationTimer = this->create_wall_timer(
        std::chrono::duration<double>(duration),
        std::bind(&OmnimagnetDriverNode::durationCallback, this)
    );
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
    // Stop any ongoing experiment and reset the active command count
    experimentRunning.store(false, std::memory_order_release);
    activeCommandCount.store(0, std::memory_order_release);

    // Cancel the duration timer if it exists
    if (durationTimer) {
        durationTimer->cancel();
    }

    RCLCPP_INFO(this->get_logger(), "System reset by command.");

    // Turn off all magnets and check for failures
    for (auto & [id, magnet] : omnimagnets) {
        if (magnet.SetCurrent(offVector) < 1) {
            // TODO: use systemError to avoid code duplication
            auto msg = omnimagnet_interfaces::msg::ErrorMessage();
            msg.error_desc = "Failed to turn off magnet " + std::to_string(id);
            msg.shutdown = true;
            errorPublisher->publish(msg);

            RCLCPP_WARN(this->get_logger(), "Failed to shut down magnet %d.\nShutting down.", id);

            rclcpp::shutdown();
        } 
    }

    // Reset the timeout timer to wait for the next command
    timeoutTimer->reset();

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
    auto period = std::chrono::duration<double>(1. / control_hz);

    auto next = clock::now();

    // Main control loop that runs while the control thread is active
    while (controlThreadRunning.load(std::memory_order_acquire)) {
        auto now = clock::now();

        // Ensure that the next control cycle is in the future, adjusting if necessary
        while (next <= now) {
            next += std::chrono::duration_cast<clock::duration>(period);
        }
        
        // If an experiment is running, update the currents of the active magnets based on their respective commands
        if (experimentRunning.load(std::memory_order_acquire)) {
            // Calculate the elapsed time since the start of the experiment
            double t = std::chrono::duration<double>(now - startTime).count();

            // Copy the active commands to a local array to avoid holding the lock while updating currents
            std::array<ActiveMagnetCommand, maxMagnets> localCommands;

            // Get the current count of active commands using relaxed memory order
            size_t localCount = activeCommandCount.load(std::memory_order_relaxed);

            // Copy the active commands to the local array while holding the lock to ensure thread safety
            {
                std::lock_guard<std::mutex> lock(commandMutex);
                std::copy_n(activeCommands.begin(), localCount, localCommands.begin());
            }
            
            // Update the currents of the active magnets based on their respective commands (constant or rotating dipoles)
            for (size_t i = 0; i < localCount; ++i) {
                const auto& command = localCommands[i];

                // Constant dipole command (frequency = 0) or rotating dipole command (frequency > 0)
                if (command.freq == 0){
                    command.omni->SetCurrent(command.omni->Dipole2Current(command.strength * command.vector));
                }
                else {
                    // Calculate the angle theta based on the frequency, elapsed time, and phase offset
                    double theta = 2.0 * M_PI * command.freq * t + command.offset;
                    Eigen::Vector3d dipole = command.strength * 
                        (std::cos(theta) * command.basis.u +
                         std::sin(theta) * command.basis.v);
    
                    command.omni->SetCurrent(command.omni->Dipole2Current(dipole));
                }
            }
        }

        // Sleep until the next control cycle to maintain the desired control frequency
        std::this_thread::sleep_until(next);
    }
}


/***************** HELPERS **********************/

/**
 * @brief Constructs a Basis struct from a given axis vector.
 * 
 * This function takes an axis vector and constructs a 2D plane with basis vectors u and v 
 * that are orthogonal to the provided axis using gram-schmidt orthogonalization. 
 * The basis vectors are normalized and can be used for rotating dipole commands.
 * 
 * @param axis The axis vector to construct the basis from.
 * @return A Basis struct containing the orthogonal basis vectors u and v.
 * 
 * TODO: Consider moving to anonymous namespace to avoid polluting the global namespace, as this function is only used within this file.
 */
Basis OmnimagnetDriverNode::makeBasis(const Eigen::Vector3d& axis) {
    Eigen::Vector3d n = axis.normalized();

    Eigen::Vector3d initVec =
        (std::abs(n.x()) < 0.9)
            ? Eigen::Vector3d::UnitX()
            : Eigen::Vector3d::UnitY();

    Eigen::Vector3d u = n.cross(initVec).normalized();
    Eigen::Vector3d v = n.cross(u).normalized();

    return Basis(u, v);
}

/***************** ROS Builders *****************/
/**
 * @brief Declares parameters for the OmnimagnetDriverNode.
 * 
 * This function declares various parameters for the OmnimagnetDriverNode, including hardware settings, timing configurations, and default parameters for each magnet (1-6).
 * It uses the declare_parameter method to declare parameters with their default values.
 * The parameters include hardware device, subdevice, channel, range, analog reference, inhibitor settings, 
 * timing timeout, default duration, control frequency, 
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
    this->declare_parameter<int>(
        "hardware.inhibitor.enabled", false
    );
    this->declare_parameter<std::vector<int>>(
        "hardware.inhibitor.pins", {25, 26}
    );
    this->declare_parameter<double>(
        "hardware.inhibitor.percent", .75
    );

    // Timing
    this->declare_parameter<double>(
        "timing.timeout_seconds", 300.
    );
    this->declare_parameter<double>(
        "timing.default_duration_seconds", 30.0
    );
    this->declare_parameter<double>(
        "timing.control_frequency_hz", 1000.
    );

    // Default channels for magnets 1-6
    constexpr std::array<std::array<int, 3>, maxMagnets> default_channels{{
        {{2, 0, 18}},
        {{3, 11, 19}},
        {{4, 12, 20}},
        {{5, 13, 21}},
        {{6, 14, 22}},
        {{7, 15, 23}}
    }};

    // Declare parameters for each magnet (1-6) using the default channels
    for (std::size_t i = 0; i < maxMagnets; ++i) {
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
    std::size_t index,
    const std::array<int, 3>& default_channels)
{
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
        prefix + "wire_lengths.mid",
        122.0
    );

    this->declare_parameter<double>(
        prefix + "wire_lengths.outer",
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
        identityFrame_
    );
}

/**
 * @brief Loads software parameters for the OmnimagnetDriverNode.
 * 
 * This function retrieves the values of the timing parameters "timing.default_duration_seconds" and "timing.timeout_seconds" from the ROS parameter server and stores them in the member variables defaultDuration_ and timeout_, respectively.
 */
void OmnimagnetDriverNode::loadParameters() {
    defaultDuration_ = this->get_parameter("timing.default_duration_seconds").as_double();
    timeout_ = this->get_parameter("timing.timeout_seconds").as_double();
}

/**
 * @brief Loads the configuration for a specific magnet.
 * 
 * This function retrieves the values of the parameters for the magnet at the given index from the ROS parameter server and stores them in a MagnetConfig struct.
 * 
 * @param index The index of the magnet for which to load configuration.
 * @return The configuration for the specified magnet.
 */
MagnetConfig OmnimagnetDriverNode::loadMagnetConfig(std::size_t index) const {
    std::string prefix = magnetNamespace(index);

    MagnetConfig config;

    config.id = 
        this->get_parameter(prefix + ".id").as_int();

    config.enabled = 
        this->get_parameter(prefix + ".enabled").as_bool();

    config.wire_width =
        this->get_parameter(prefix + ".wire_width").as_double();

    config.inner_wire_length =
        this->get_parameter(prefix + ".wire_lengths.inner").as_double();

    config.mid_wire_length =
        this->get_parameter(prefix + "wire_lengths.mid").as_double();

    config.outer_wire_length =
        this->get_parameter(prefix + ".wire_lengths.outer").as_double();

    config.core_size =
        this->get_parameter(prefix + ".core_size").as_double();

    config.inner_channel =
        this->get_parameter(prefix + ".channels.inner").as_int();

    config.mid_channel = 
        this->get_parameter(prefix + ".channels.mid").as_int();

    config.outer_channel =
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
 * This function creates two timers: a timeout timer and a duration timer. The timeout timer is set to trigger after the specified timeout duration, while the duration timer is set to trigger after the specified default duration.
 * The duration timer is initially canceled and will be started when an experiment is run.
 */
void OmnimagnetDriverNode::buildTimers() {
    this->timeoutTimer = this->create_wall_timer(
        std::chrono::duration<double>(timeout_),
        std::bind(&OmnimagnetDriverNode::timeoutCallback, this)
    );

    this->durationTimer = this->create_wall_timer(
        std::chrono::duration<double>(defaultDuration_),
        std::bind(&OmnimagnetDriverNode::durationCallback, this)
    );
    this->durationTimer->cancel(); // Hold timer until experiment run
}

/**
 * @brief Builds the publishers for the OmnimagnetDriverNode.
 * 
 * This function creates two publishers: an error publisher and a finished publisher. The error publisher is used to publish error messages, 
 * while the finished publisher is used to publish messages when an experiment is finished.
 */
void OmnimagnetDriverNode::buildPublishers() {
    this->errorPublisher = 
        this->create_publisher<omnimagnet_interfaces::msg::ErrorMessage>("driver_errors", 10);
    this->finishedPublisher = 
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
        smcServer = this->create_service<omnimagnet_interfaces::srv::SingleMagnetConstant>(
            "single_magnet_constant",
            std::bind(&OmnimagnetDriverNode::smcCallback,
                this,
                std::placeholders::_1,
                std::placeholders::_2
            )
        );
        smrServer = this->create_service<omnimagnet_interfaces::srv::SingleMagnetRotation>(
            "single_magnet_rotation",
            std::bind(&OmnimagnetDriverNode::smrCallback,
                this,
                std::placeholders::_1,
                std::placeholders::_2
            )
        );
        mmcServer = this->create_service<omnimagnet_interfaces::srv::MultiMagnetConstant>(
            "multi_magnet_constant",
            std::bind(&OmnimagnetDriverNode::mmcCallback,
                this,
                std::placeholders::_1,
                std::placeholders::_2
            )
        );
        mmrServer = this->create_service<omnimagnet_interfaces::srv::MultiMagnetRotation>(
            "multi_magnet_rotation",
            std::bind(&OmnimagnetDriverNode::mmrCallback,
                this,
                std::placeholders::_1,
                std::placeholders::_2
            )
        );
        resetServer = this->create_service<omnimagnet_interfaces::srv::DriverReset>(
            "reset_driver",
            std::bind(&OmnimagnetDriverNode::resetCallback,
                this,
                std::placeholders::_1,
                std::placeholders::_2
            )
        );
}