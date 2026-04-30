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
    
    omnimagnets = std::map<int, OmniMagnet>();

    offVector << 0.0, 0.0, 0.0;

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
    // TODO: Convert this into a .yaml file to load config
    subdev = 0;
    chan = 10;
    range = 0;
    aref = AREF_GROUND;

    D2A = comedi_open("/dev/comedi0");
    if(D2A == nullptr) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open D2A:");
        comedi_perror("comedi_open");

        auto msg = omnimagnet_interfaces::msg::ErrorMessage();
        msg.error_desc = "Failed to open D2A device.\n";
        msg.shutdown = true;
        errorPublisher->publish(msg);

        std::exit(1);
    }

    // Setting amplifier inhibitors at 75%. Pins are 25&26
    this->maxdata25  = comedi_get_maxdata(D2A, subdev, 25);
    this->maxdata26  = comedi_get_maxdata(D2A, subdev, 26);
    lsampl_t inhib25 = maxdata25 * .75;
    lsampl_t inhib26 = maxdata26 * .75;

    int r25 = comedi_data_write(D2A, subdev, 25, 0, aref, inhib25);
    if (r25 < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to inhibit pin 25!");
        comedi_perror("Pin25");

        auto msg = omnimagnet_interfaces::msg::ErrorMessage();
        msg.error_desc = "Failed to set inhibitor on D2A pin 25.\n";
        msg.shutdown = true;
        errorPublisher->publish(msg);

        std::exit(1);
    }

    int r26 = comedi_data_write(D2A, subdev, 26, 0, aref, inhib26);
    if (r26 < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to inhibit pin 26!");
        comedi_perror("Pin26");
        
        auto msg = omnimagnet_interfaces::msg::ErrorMessage();
        msg.error_desc = "Failed to set inhibitor on D2A pin 26.\n";
        msg.shutdown = true;
        errorPublisher->publish(msg);

        std::exit(1);
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
    // TODO: Replace this with parameterization and configuration loading
    omnimagnets[0].SetProp(1.35/1000.0 , 121, 122, 132, 17, 2, 0, 18, true, D2A);     // Omni #1, left upper 
    omnimagnets[1].SetProp(1.35/1000.0 , 121, 122, 132, 17, 3, 11, 19, true, D2A);    // Omni #2, center upper 
    omnimagnets[2].SetProp(1.35/1000.0 , 121, 122, 132, 17, 4, 12, 20, true, D2A);    // Omni #3, right upper 
    omnimagnets[3].SetProp(1.35/1000.0 , 121, 122, 132, 17, 5, 13, 21, true, D2A);     // Omni #4, right lower 
    omnimagnets[4].SetProp(1.35/1000.0 , 121, 122, 132, 17, 6, 14, 22, true, D2A);   // Omni #5, left lower
    // omnimagnets[5].SetProp(1.35/1000.0 , 121, 122, 132, 17, 7, 15, 23, true, D2A); omnimagnets[5].UpdateMapping();  // spare (potentially for Super Omni), DON'T TURN ON TILL CONNECTED

    for (auto & [id, magnet] : omnimagnets) {
        magnet.UpdateMapping();

        magnet.setD2AMax(this->maxdata25);

        magnet.ID = id;
    }

    RCLCPP_INFO(this->get_logger(), "Magnet setup complete.");
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

    bool pinFail = false;
    int retval = comedi_data_write(D2A, subdev, 25, 0, AREF_GROUND, 16383.0*2./4.);
    if (retval < 0) {
        std::cerr << "Failed to shut down D2A Pin 25!" << std::endl;
        pinFail = true;
    }
    retval = comedi_data_write(D2A, subdev, 26, 0, AREF_GROUND, 16383.0*2./4.);
    if (retval < 0) {
        std::cerr << "Failed to shut down D2A Pin 26!" << std::endl;
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
    RCLCPP_WARN(this->get_logger(), "Connection timed out.");

    auto msg = omnimagnet_interfaces::msg::ErrorMessage();

    msg.error_desc = "Controller timed out waiting for command. Shutting down.";
    msg.shutdown   = true;

    errorPublisher->publish(msg);

    rclcpp::shutdown();
}


void OmnimagnetDriverNode::durationCallback() {
    // Delete timer until new order is received
    durationTimer->cancel();

    experimentRunning.store(false, std::memory_order_release);
    activeCommandCount.store(0, std::memory_order_release);

    for (auto& [id, omni] : omnimagnets) {
        int retval = omni.SetCurrent(offVector);
        if (retval <= 0) {
            auto msg = omnimagnet_interfaces::msg::ErrorMessage();
            msg.error_desc = "Failed to shut down magnet" + std::to_string(id);
            msg.shutdown = true;

            errorPublisher->publish(msg);

            rclcpp::shutdown();
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
        RCLCPP_WARN(this->get_logger(), "Server tried to start experiment while already in operation.");

        response->error = true;
        response->error_desc = "Operation in progress. Please reset before invoking another operation.";

        timeoutTimer->reset();
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
        duration = 30.0;
    }

    OmniMagnet& omni = omnimagnets[id];

    Eigen::Vector3d dipole_vector;
    dipole_vector << vector.x, vector.y, vector.z;
    if (dipole_vector.norm() < 1e-8) {
        RCLCPP_WARN(this->get_logger(), "User attempted to pass zero vector.");

        response->error = true;
        response->error_desc = "Zero dipole vector.";

        timeoutTimer->reset();
        return;
    }
    if (!dipole_vector.allFinite()) {
        RCLCPP_WARN(this->get_logger(), "User attempted to pass NaN.");

        response->error = true;
        response->error_desc = "NaN component.";

        timeoutTimer->reset();
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
        "Duration %.1f s\n",
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
    auto freq = request->rotation_freq;
    auto duration = request->duration;

    if (omnimagnets.count(id) == 0) {
        RCLCPP_WARN(this->get_logger(), "Omnimagnet %lu not found: ", id);

        response->error = true;
        response->error_desc = "Invalid magnet ID: " + std::to_string(id);

        timeoutTimer->reset();
        return;
    }

    auto omni = &omnimagnets[id];
    
    Eigen::Vector3d rotVec;
    rotVec << rotationVector.x, rotationVector.y, rotationVector.z;
    if (rotVec.norm() < 1e-8) {
        RCLCPP_WARN(this->get_logger(), "User attempted to pass zero vector.");

        response->error = true;
        response->error_desc = "Zero rotation vector passed.";

        timeoutTimer->reset();
        return;
    }
    if (!rotVec.allFinite()) {
        RCLCPP_WARN(this->get_logger(), "User attempted to pass NaN.");

        response->error = true;
        response->error_desc = "NaN component.";

        timeoutTimer->reset();
        return;
    }
    rotVec.normalize();
    {
        std::lock_guard<std::mutex> lock (commandMutex);
        activeCommands[0] = {omni, freq, strength, offset, makeBasis(rotVec), rotVec};
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
        "Duration %.1f s\n",
        duration
    );

    RCLCPP_INFO(this->get_logger(), 
        "Magnet: %lu\n"
        "Dipole: <%.3f, %.3f, %.3f>\n"
        "Strength: %.2f\n"
        "Frequency: %.2f",
        id, rotationVector.x, rotationVector.y, rotationVector.z, strength, freq
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
        RCLCPP_WARN(this->get_logger(), "No omnimagnets selected.");

        response->error = true;
        response->error_desc = "No omnimagnets selected.";

        timeoutTimer->reset();
        return;
    }
    if (ids.size() > maxMagnets) {
        RCLCPP_WARN(this->get_logger(), "Too many magnets selected.");

        response->error = true;
        response->error_desc = "Too many magnets selected.";

        timeoutTimer->reset();
        return;
    }
    if (strengths.size() != 1 && strengths.size() != ids.size()) {
        RCLCPP_WARN(this->get_logger(), "Request size mismatch.");

        response->error = true;
        response->error_desc = "Dipole strengths list size mismatch.";

        timeoutTimer->reset();
        return;
    }

    if (vectors.size() != 1 && vectors.size() != ids.size()) {
        RCLCPP_WARN(this->get_logger(), "Request size mismatch.");

        response->error = true;
        response->error_desc = "Dipole vectors list size mismatch.";

        timeoutTimer->reset();
        return;
    }

    for (auto& id : ids) {
        if (omnimagnets.count(id) == 0) {
            RCLCPP_WARN(this->get_logger(), "Omnimagnet %lu not found: ", id);

            response->error = true;
            response->error_desc = "Invalid magnet ID: " + std::to_string(id);
    
            timeoutTimer->reset();
            return;
        }
    }

    // Optional argument
    auto duration = request->duration;
    if (duration <= 0.0) {
        duration = 30.0;
        // TODO: Parameterize default duration
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
            RCLCPP_WARN(this->get_logger(), "User attempted to pass zero vector.");

            response->error = true;
            response->error_desc = "Zero dipole vector.";

            timeoutTimer->reset();
            return;
        }
        if (!dipole_vector.allFinite()) {
            RCLCPP_WARN(this->get_logger(), "User attempted to pass NaN.");

            response->error = true;
            response->error_desc = "NaN component.";

            timeoutTimer->reset();
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
    auto freq = request->rotation_freq;
    auto strengths = request->dipole_strengths;
    auto offsets = request->phase_offsets;

    // Check for size mismatches
    if (ids.size() < 1) {
        RCLCPP_WARN(this->get_logger(), "No omnimagnets selected.");

        response->error = true;
        response->error_desc = "No omnimagnets selected.";

        timeoutTimer->reset();
        return;
    }
    if (ids.size() > maxMagnets) {
        RCLCPP_WARN(this->get_logger(), "Too many magnets selected.");

        response->error = true;
        response->error_desc = "Too many magnets selected.";

        timeoutTimer->reset();
        return;
    }
    if (strengths.size() != 1 && strengths.size() != ids.size()) {
        RCLCPP_WARN(this->get_logger(), "Request size mismatch.");

        response->error = true;
        response->error_desc = "Dipole strengths list size mismatch.";

        timeoutTimer->reset();
        return;
    }

    if (rotationVectors.size() != 1 && rotationVectors.size() != ids.size()) {
        RCLCPP_WARN(this->get_logger(), "Request size mismatch.");

        response->error = true;
        response->error_desc = "Rotation vectors list size mismatch.";

        timeoutTimer->reset();
        return;
    }

    if (offsets.size() != 1 && offsets.size() != ids.size()) {
        RCLCPP_WARN(this->get_logger(), "Request size mismatch.");

        response->error = true;
        response->error_desc = "Phase offsets list size mismatch.";
        
        timeoutTimer->reset();
        return;
    }

    for (auto& id : ids) {
        if (omnimagnets.count(id) == 0) {
            RCLCPP_WARN(this->get_logger(), "Omnimagnet %lu not found: ", id);

            response->error = true;
            response->error_desc = "Invalid magnet ID: " + std::to_string(id);
    
            timeoutTimer->reset();
            return;
        }
    }

    // Optional argument
    auto duration = request->duration;
    if (duration <= 0.0) {
        duration = 30.0;
        // TODO: Parameterize default duration
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

        id = ids[i];

        if (rotationVectors.size() == 1)
            rotationVector = rotationVectors[0];
        else
            rotationVector = rotationVectors[i];

        if (strengths.size() == 1)
            strength = strengths[0];
        else
            strength = strengths[i];

        if (offsets.size() == 1)
            offset = offsets[0];
        else
            offset = offsets[i];

        auto omni = &omnimagnets[id];
        
        Eigen::Vector3d rotVec;
        rotVec << rotationVector.x, rotationVector.y, rotationVector.z;
        if (rotVec.norm() < 1e-8) {
            RCLCPP_WARN(this->get_logger(), "User attempted to pass zero vector.");

            response->error = true;
            response->error_desc = "Zero rotation vector passed.";

            timeoutTimer->reset();
            return;
        }
        if (!rotVec.allFinite()) {
            RCLCPP_WARN(this->get_logger(), "User attempted to pass NaN.");

            response->error = true;
            response->error_desc = "NaN component.";

            timeoutTimer->reset();
            return;
        }
        rotVec.normalize();
        
        {
            std::lock_guard<std::mutex> lock (commandMutex);
            activeCommands[i] = {omni, freq, strength, offset, makeBasis(rotVec), rotVec};
        }
        logString
            << "Magnet: " << id << std::endl
            << "Dipole: <"
                << rotationVector.x << ", "
                << rotationVector.y << ", "
                << rotationVector.z << ">" << std::endl
            << "Strength: " << strength << std::endl
            << "Frequency: " << freq << std::endl;
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

    // TODO: Parameterize
    constexpr double control_hz = 1200.0;
    const auto period = std::chrono::duration<double>(1. / control_hz);

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
void OmnimagnetDriverNode::buildTimers() {
    this->timeoutTimer = this->create_wall_timer(
        std::chrono::duration<double>(30.),
        std::bind(&OmnimagnetDriverNode::timeoutCallback, this)
    );

    this->durationTimer = this->create_wall_timer(
        std::chrono::duration<double>(10.0),
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