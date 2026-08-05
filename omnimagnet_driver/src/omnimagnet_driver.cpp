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
    // Returns a string representing the namespace of a given magnet index
    std::string magnetNamespace(std::size_t index) {
        return "magnets.magnet_" + std::to_string(index);
    }

    // Error handler for input command errors
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
 * @brief Sets up D2A for magnet operation.
 * 
 * TODO:
 * 
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
    if (inhibs.size() != 2) {
        RCLCPP_ERROR(this->get_logger(), "Need to specify two inhibitor pins.");
        rclcpp::shutdown();
    }
    auto inhbp1 = inhibs.at(0); 
    auto inhbp2 = inhibs.at(1);

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

    // Setting amplifier inhibitors at 75%. Pins are 25&26
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
 * @brief Loads omnimagnets with parameters for operation.
 * 
 * TODO:
 * 
 */
void OmnimagnetDriverNode::setupMagnets() {
    for (std::size_t i =1; i < maxMagnets; ++i) {
        const MagnetConfig config = loadMagnetConfig(i);

        if (!config.enabled)
            continue;

        if (config.frame.size() != 9) {
            throw std::runtime_error(
                magnetNamespace(i) + ".frame must contain exactly 9 values (3x3 matrix)"
            );
        }

        auto [iterator, inserted] =
            omnimagnets.try_emplace(config.id);

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
 * @brief Turns off system for safe shutdown.
 * 
 * TODO:
 * 
 */
void OmnimagnetDriverNode::shutdown() {
    static std::atomic_bool already_shutdown{false};

    if (already_shutdown.exchange(true)) {
        return;
    }
    std::cout << "Beginning Shutdown" << std::endl;

    controlThreadRunning.store(false, std::memory_order_release);
    if (controlThread.joinable()) {
        controlThread.join();
    }

    if (D2A == nullptr) {
        std::cerr << "D2A was null during shutdown:" << std::endl <<
        "Hardware may not have been properly initialized" << std::endl;

        return;
    }

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


void OmnimagnetDriverNode::durationCallback() {
    // Delete timer until new order is received
    durationTimer->cancel();

    experimentRunning.store(false, std::memory_order_release);
    activeCommandCount.store(0, std::memory_order_release);

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

    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);

    std::ostringstream oss;
    oss << "Run finished at: "
        << std::put_time(std::localtime(&now_c), "%Y-%m-%d %H:%M:%S");

    msg.msg = oss.str();

    RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());

    finishedPublisher->publish(msg);

    // Wait for new command or timeout
    timeoutTimer->reset();
}

/*************** SERVER CALLBACKS ***************/

void OmnimagnetDriverNode::smcCallback(
    const omnimagnet_interfaces::srv::SingleMagnetConstant::Request::SharedPtr request,
    const omnimagnet_interfaces::srv::SingleMagnetConstant::Response::SharedPtr response
    ) 
{
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

    timeoutTimer->cancel();

    auto id = request->omnimagnet;
    auto strength = request->dipole_strength;
    auto vector = request->dipole_vec;

    if (omnimagnets.count(id) == 0) {
        RCLCPP_WARN(this->get_logger(), "Omnimagnet %lu not found: ", id);

        response->error = true;
        response->error_desc = "Invalid magnet ID.";

        timeoutTimer->reset();
        return;
    }

    // Optional argument
    auto duration = request->duration;
    if (duration <= 0.0) {
        RCLCPP_INFO(this->get_logger(), "0 or negative duration passed.");
        return;
    }

    OmniMagnet& omni = omnimagnets[id];

    Eigen::Vector3d dipole_vector;
    dipole_vector << vector.x, vector.y, vector.z;

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

    dipole_vector.normalize();

    {
        std::lock_guard<std::mutex> lock(commandMutex);
        activeCommands[0] = {&omni, 0, strength, 0, makeBasis(dipole_vector), dipole_vector};
    }

    startTime = std::chrono::steady_clock::now();
    activeCommandCount.store(1, std::memory_order_release);
    experimentRunning.store(true, std::memory_order_release);

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


void OmnimagnetDriverNode::smrCallback(
    const omnimagnet_interfaces::srv::SingleMagnetRotation::Request::SharedPtr request,
    const omnimagnet_interfaces::srv::SingleMagnetRotation::Response::SharedPtr response
) {
    if (experimentRunning.load(std::memory_order_acquire)) {
        RCLCPP_WARN(this->get_logger(), "Server tried to start experiment while already in operation.");

        response->error = true;
        response->error_desc = "Operation in progress. Please reset before invoking another operation.";

        timeoutTimer->reset();
        return;
    }

    timeoutTimer->cancel();

    auto id = request->omnimagnet;
    auto strength = request->dipole_strength;
    auto rotationVector = request->rotation_vector;
    auto offset = request->phase_offset;
    auto phaseOffset = offset * M_PI / 180.0;
    auto freq = request->rotation_freq;

    if (omnimagnets.count(id) == 0) {
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

    rotVec.normalize();

    {
        std::lock_guard<std::mutex> lock (commandMutex);
        activeCommands[0] = {omni, freq, strength, phaseOffset, makeBasis(rotVec), rotVec};
    }

    startTime = std::chrono::steady_clock::now();
    activeCommandCount.store(1, std::memory_order_release);
    experimentRunning.store(true, std::memory_order_release);

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


void OmnimagnetDriverNode::mmcCallback(
    const omnimagnet_interfaces::srv::MultiMagnetConstant::Request::SharedPtr request,
    const omnimagnet_interfaces::srv::MultiMagnetConstant::Response::SharedPtr response
)
{
    if (experimentRunning.load(std::memory_order_acquire)) {
        RCLCPP_WARN(this->get_logger(), "Server tried to start experiment while already in operation.");

        response->error = true;
        response->error_desc = "Operation in progress. Please reset before invoking another operation.";

        timeoutTimer->reset();
        return;
    }

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

    // Iterate through lists
    for (std::size_t i = 0; i < ids.size(); ++i) {
        uint64_t id;
        omnimagnet_interfaces::msg::Vector3 vector;
        double strength;

        id = ids[i];

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

        dipole_vector.normalize();

        logString 
            << "Magnet: " << id << std::endl
            << "Dipole: <"
                << vector.x << ", "
                << vector.y << ", "
                << vector.z << ">" << std::endl
            << "Strength: " << strength << std::endl;

        {
            std::lock_guard<std::mutex> lock(commandMutex);
            activeCommands[i] = {&omni, 0, strength, 0, makeBasis(dipole_vector), dipole_vector};
        }
    }

    RCLCPP_INFO(this->get_logger(), "%s", logString.str().c_str());

    startTime = std::chrono::steady_clock::now();
    activeCommandCount.store(ids.size(), std::memory_order_release);
    experimentRunning.store(true, std::memory_order_release);

    if (durationTimer) {
        durationTimer->cancel();
    }
    durationTimer = this->create_wall_timer(
        std::chrono::duration<double>(duration),
        std::bind(&OmnimagnetDriverNode::durationCallback, this)
    );
}


void OmnimagnetDriverNode::mmrCallback(
    const omnimagnet_interfaces::srv::MultiMagnetRotation::Request::SharedPtr request,
    const omnimagnet_interfaces::srv::MultiMagnetRotation::Response::SharedPtr response
) {
    if (experimentRunning.load(std::memory_order_acquire)) {
        RCLCPP_WARN(this->get_logger(), "Server tried to start experiment while already in operation.");

        response->error = true;
        response->error_desc = "Operation in progress. Please reset before invoking another operation.";

        timeoutTimer->reset();
        return;
    }

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

    // Iterate through lists
    for (std::size_t i = 0; i < ids.size(); ++i) {
        uint64_t id;
        omnimagnet_interfaces::msg::Vector3 rotationVector;
        double strength;
        double offset;
        double freq;

        id = ids[i];

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

        rotVec.normalize();
        
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

    startTime = std::chrono::steady_clock::now();
    activeCommandCount.store(ids.size(), std::memory_order_release);
    experimentRunning.store(true, std::memory_order_release);

    if (durationTimer) {
        durationTimer->cancel();
    }
    durationTimer = this->create_wall_timer(
        std::chrono::duration<double>(duration),
        std::bind(&OmnimagnetDriverNode::durationCallback, this)
    );
}


void OmnimagnetDriverNode::resetCallback(
    [[maybe_unused]] const omnimagnet_interfaces::srv::DriverReset::Request::SharedPtr request,
    const omnimagnet_interfaces::srv::DriverReset::Response::SharedPtr response
) {
    experimentRunning.store(false, std::memory_order_release);
    activeCommandCount.store(0, std::memory_order_release);

    if (durationTimer) {
        durationTimer->cancel();
    }

    RCLCPP_INFO(this->get_logger(), "System reset by command.");

    for (auto & [id, magnet] : omnimagnets) {
        if (magnet.SetCurrent(offVector) < 1) {
            auto msg = omnimagnet_interfaces::msg::ErrorMessage();
            msg.error_desc = "Failed to turn off magnet " + std::to_string(id);
            msg.shutdown = true;
            errorPublisher->publish(msg);

            RCLCPP_WARN(this->get_logger(), "Failed to shut down magnet %d.\nShutting down.", id);

            rclcpp::shutdown();
        } 
    }

    timeoutTimer->reset();

    response->status = true;
}

/****************** THREAD LOOP ************************/

void OmnimagnetDriverNode::controlLoop() {
    using clock = std::chrono::steady_clock;

    double control_hz = this->get_parameter(
        "timing.control_frequency_hz").as_double();
    auto period = std::chrono::duration<double>(1. / control_hz);

    auto next = clock::now();

    while (controlThreadRunning.load(std::memory_order_acquire)) {
        auto now = clock::now();
        while (next < now) {
            next += std::chrono::duration_cast<clock::duration>(period);
        }
        
        if (experimentRunning.load(std::memory_order_acquire)) {
            double t = std::chrono::duration<double>(now - startTime).count();
            std::array<ActiveMagnetCommand, maxMagnets> localCommands;
            size_t localCount = activeCommandCount.load(std::memory_order_relaxed);
            {
                std::lock_guard<std::mutex> lock(commandMutex);
                std::copy_n(activeCommands.begin(), localCount, localCommands.begin());
            }
            
            for (size_t i = 0; i < localCount; ++i) {
                const auto& command = localCommands[i];

                if (command.freq == 0){
                    command.omni->SetCurrent(command.omni->Dipole2Current(command.strength * command.vector));
                }
                else {
                    double theta = 2.0 * M_PI * command.freq * t + command.offset;
                    Eigen::Vector3d dipole = command.strength * 
                        (std::cos(theta) * command.basis.u +
                         std::sin(theta) * command.basis.v);
    
                    command.omni->SetCurrent(command.omni->Dipole2Current(dipole));
                }
            }
        }

        std::this_thread::sleep_until(next);
    }
}


/***************** HELPERS **********************/

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

    for (std::size_t i = 0; i < maxMagnets; ++i) {
        declareMagnetParameters(i + 1, default_channels[i]);
    }
}

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

void OmnimagnetDriverNode::loadParameters() {
    defaultDuration_ = this->get_parameter("timing.default_duration_seconds").as_double();
    timeout_ = this->get_parameter("timing.timeout_seconds").as_double();
}

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

void OmnimagnetDriverNode::buildPublishers() {
    this->errorPublisher = 
        this->create_publisher<omnimagnet_interfaces::msg::ErrorMessage>("driver_errors", 10);
    this->finishedPublisher = 
        this->create_publisher<omnimagnet_interfaces::msg::FinishedMessage>("driver_finished", 10);
}

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