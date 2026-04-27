# include "rclcpp/rclcpp.hpp"
# include "../include/omnimagnet_driver/omnimagnet.hpp"
# include "comedilib.hpp"
# include "../include/omnimagnet_driver/omnimagnet_driver.hpp"

# include <atomic>
# include <chrono>
# include <thread>
# include <vector>
# include <map>

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

    activeMagnets = std::vector<OmniMagnet*>();

    offVector << 0.0, 0.0, 0.0;

    running = false;
    strength = 0;
    freq = 0;
    startTime = std::chrono::steady_clock::now();

    // Timers
    timeoutTimer = this->create_wall_timer(
        std::chrono::duration<double>(30.),
        std::bind(&OmnimagnetDriverNode::timeoutCallback, this)
    );

    durationTimer = this->create_wall_timer(
        std::chrono::duration<double>(10.0),
        std::bind(&OmnimagnetDriverNode::durationCallback, this)
    );
    durationTimer->cancel(); // Hold timer until prompted

    spinTimer = this->create_wall_timer(
        std::chrono::duration<double>(1./1200.),
        std::bind(&OmnimagnetDriverNode::spinCallback, this)
    );
    spinTimer->cancel(); // Hold timer until prompted

    // Publishers
    errorPublisher = this->create_publisher<omnimagnet_interfaces::msg::ErrorMessage>("driver_errors", 10);
    finishedPublisher = this->create_publisher<omnimagnet_interfaces::msg::FinishedMessage>("driver_finished", 10);

    // Servers
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

    setupHardware();
    setupMagnets();
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
    int i = 1;
    for (auto & [id, omni] : omnimagnets) {
        int retval = omni.SetCurrent(offVector);

        if (retval < 0) {
            RCLCPP_WARN(this->get_logger(), "Magnet %d failed to turn off.", i);

            auto msg = omnimagnet_interfaces::msg::ErrorMessage();
            msg.error_desc = "Failed to disable current on magnet " + std::to_string(i) + ". Continuing shutdown.\n";
            msg.shutdown = false;
            errorPublisher->publish(msg);
        }
        else {
            RCLCPP_INFO(this->get_logger(), "Magnet %d turned off.", i);
        }

        ++i;
    }

    RCLCPP_INFO(this->get_logger(), "All magnets turned off.");

    bool fail = false;
    int retval = comedi_data_write(D2A, subdev, 25, 0, AREF_GROUND, 16383.0*2./4.);
    if (retval < 0) {
        RCLCPP_WARN(this->get_logger(), "Failed to shut down D2A Pin 25!");
        fail = true;

        auto msg = omnimagnet_interfaces::msg::ErrorMessage();
        msg.error_desc = "Failed to shut down pin 25.\n";
        msg.shutdown = false;
        errorPublisher->publish(msg);
    }
    retval = comedi_data_write(D2A, subdev, 26, 0, AREF_GROUND, 16383.0*2./4.);
    if (retval < 0) {
        RCLCPP_WARN(this->get_logger(), "Failed to shut down D2A Pin 26!");
        fail = true;

        auto msg = omnimagnet_interfaces::msg::ErrorMessage();
        msg.error_desc = "Failed to shut down pin 26.\n";
        msg.shutdown = false;
        errorPublisher->publish(msg);
    }

    if (!fail) {
        RCLCPP_INFO(this->get_logger(), "Successfully shut down hardware.");
    }
    else {
        RCLCPP_ERROR(this->get_logger(), "Failed to shut down hardware, use emergency stop!");

        auto msg = omnimagnet_interfaces::msg::ErrorMessage();
        msg.error_desc = "Failed to properly shut down hardware! Use Emergency Stop!";
        msg.shutdown = true;
        errorPublisher->publish(msg);
    }
}

/*************** TIMER CALLBACKS ***************/

/**
 * @brief Runs omnimagnets at set frequency to guarantee synchronicity.
 * 
 * TODO:
 * 
 */
void OmnimagnetDriverNode::spinCallback() {

    // Get delta-time
    auto currentTime = std::chrono::steady_clock::now();
    double t = std::chrono::duration<double>(currentTime - startTime).count();

    // Calculate new dipoles
    for (auto & omni : activeMagnets) {
        omni->theta = 2.0 * M_PI * omni->freq * t + omni->offset;
        omni->dipole = (std::cos(omni->theta) * omni->rotBasis.u +
                    std::sin(omni->theta) * omni->rotBasis.v) * omni->strength;
    }

    // Send currents to each omnimagnet
    for (auto & omni : activeMagnets) {
        int retval = omni->SetCurrent(omni->dipole);

        if (retval < 0) {
            RCLCPP_WARN(this->get_logger(), "Failed to write current to magnet %d", omni->ID);
            
            auto msg = omnimagnet_interfaces::msg::ErrorMessage();
            msg.error_desc = "Failed to write current to magnet " + std::to_string(omni->ID);
            msg.shutdown = false;
            errorPublisher->publish(msg);

            timeoutTimer->reset();

            for (auto& active : activeMagnets) {
                retval = active->SetCurrent(offVector);
                if (retval < 1) {
                    RCLCPP_WARN(this->get_logger(), "Failed to shut off magnet %d", active->ID);
                    auto msg = omnimagnet_interfaces::msg::ErrorMessage();
                    msg.error_desc = "Failed to shutdown manget " + std::to_string(omni->ID);
                    msg.shutdown = true;
                    errorPublisher->publish(msg);

                    rclcpp::shutdown();
                }
            }
            return;
        }
    }
}


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
    spinTimer->cancel();

    for (auto& omni : activeMagnets) {
        int retval = omni->SetCurrent(offVector);
        if (retval <= 0) {
            auto msg = omnimagnet_interfaces::msg::ErrorMessage();
            msg.error_desc = "Failed to shut down magnet" + std::to_string(omni->ID);
            msg.shutdown = true;

            errorPublisher->publish(msg);

            rclcpp::shutdown();
        }
    }

    activeMagnets.clear();

    auto msg = omnimagnet_interfaces::msg::FinishedMessage();

    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);

    std::ostringstream oss;
    oss << "Run finished at: "
        << std::put_time(std::localtime(&now_c), "%Y-%m-%d %H:%M:%S");

    msg.msg = oss.str();

    RCLCPP_INFO(this->get_logger(), "Run finished at: %s", oss.str().c_str());

    finishedPublisher->publish(msg);

    // Wait for new command or timeout
    timeoutTimer->reset();
    running = false;
}

/*************** SERVER CALLBACKS ***************/

void OmnimagnetDriverNode::smcCallback(
    const omnimagnet_interfaces::srv::SingleMagnetConstant::Request::SharedPtr request,
    const omnimagnet_interfaces::srv::SingleMagnetConstant::Response::SharedPtr response
    ) 
{
    if (running) {
        RCLCPP_WARN(this->get_logger(), "Server tried to start experiment while already in operation.");

        response->error = true;
        response->error_desc = "Operation in progress. Please reset before invoking another operation.";

        timeoutTimer->reset();
        return;
    }
    running = true;

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

    int retval = omni.SetCurrent(omni.Dipole2Current(strength*dipole_vector));
    if (retval < 0) {
        RCLCPP_WARN(this->get_logger(), "Failed to write current to magnet %lu", id);

        auto msg = omnimagnet_interfaces::msg::ErrorMessage();
        msg.error_desc = "Failed to write current to magnet " + std::to_string(id);
        msg.shutdown = false;
        errorPublisher->publish(msg);

        response->error = true;
        response->error_desc = "Failed to set current.";
        
        timeoutTimer->reset();
        return;
    }

    activeMagnets.push_back(&omni);

    this->create_wall_timer(
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
    if (running) {
        RCLCPP_WARN(this->get_logger(), "Server tried to start experiment while already in operation.");

        response->error = true;
        response->error_desc = "Operation in progress. Please reset before invoking another operation.";

        timeoutTimer->reset();
        return;
    }
    running = true;
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

    auto &omni = omnimagnets[id];

    omni.freq = freq;
    omni.strength = strength;
    omni.offset = offset;
    
    Eigen::Vector3d rotVec;
    rotVec << rotationVector.x, rotationVector.y, rotationVector.z;
    if (rotVec.norm() < 1e-8) {
        RCLCPP_WARN(this->get_logger(), "User attempted to pass zero vector.");

        response->error = true;
        response->error_desc = "Zero rotation vector passed.";

        timeoutTimer->reset();
        return;
    }

    rotVec.normalize();

    omni.rotVector = rotVec;
    omni.rotBasis = makeBasis(rotVec);

    activeMagnets.push_back(&omni);

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

    startTime = std::chrono::steady_clock::now();
    spinTimer->reset();
}


void OmnimagnetDriverNode::mmcCallback(
    const omnimagnet_interfaces::srv::MultiMagnetConstant::Request::SharedPtr request,
    const omnimagnet_interfaces::srv::MultiMagnetConstant::Response::SharedPtr response
)
{
    if (running) {
        RCLCPP_WARN(this->get_logger(), "Server tried to start experiment while already in operation.");

        response->error = true;
        response->error_desc = "Operation in progress. Please reset before invoking another operation.";

        timeoutTimer->reset();
        return;
    }
    running = true;

    timeoutTimer->cancel();
    
    auto ids = request->omnimagnets;
    auto strengths = request->dipole_strengths;
    auto vectors = request->dipole_vecs;

    // Check for size mismatch
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

    RCLCPP_INFO(this->get_logger(), 
        "Beginning Operation\n"
        "Multiple Magnets\n"
        "Mode: Constant Dipole\n"
        "Duration %.1f s\n",
        duration
    );

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

        RCLCPP_INFO(this->get_logger(), 
            "Magnet: %lu\n"
            "Dipole: <%.3f, %.3f, %.3f>\n"
            "Strength: %.2f",
            id, vector.x, vector.y, vector.z, strength
        );

        int retval = omni.SetCurrent(omni.Dipole2Current(strength*dipole_vector));
        if (retval < 0) {
            RCLCPP_WARN(this->get_logger(), "Failed to write current to magnet %lu", id);
            
            auto msg = omnimagnet_interfaces::msg::ErrorMessage();
            msg.error_desc = "Failed to write current to magnet " + std::to_string(id);
            msg.shutdown = false;
            errorPublisher->publish(msg);

            response->error = true;
            response->error_desc = "Failed to set current in magnet " + std::to_string(id);

            timeoutTimer->reset();

            for (auto& active : activeMagnets) {
                active->SetCurrent(offVector);
            }
            return;
        }

        activeMagnets.push_back(&omni);
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
    if (running) {
        RCLCPP_WARN(this->get_logger(), "Server tried to start experiment while already in operation.");

        response->error = true;
        response->error_desc = "Operation in progress. Please reset before invoking another operation.";

        timeoutTimer->reset();
        return;
    }
    running = true;

    timeoutTimer->cancel();

    auto ids = request->omnimagnets;
    auto rotationVectors = request->rotation_vectors;
    auto freq = request->rotation_freq;
    auto strengths = request->dipole_strengths;
    auto offsets = request->phase_offsets;

    // Check for size mismatch
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

    RCLCPP_INFO(this->get_logger(), 
        "Beginning Operation\n"
        "Multiple Magnets\n"
        "Mode: Rotating Dipole\n"
        "Duration %.1f s\n",
        duration
    );

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

        auto& omni = omnimagnets[id];
        
        Eigen::Vector3d rotVec;
        rotVec << rotationVector.x, rotationVector.y, rotationVector.z;
        if (rotVec.norm() < 1e-8) {
            RCLCPP_WARN(this->get_logger(), "User attempted to pass zero vector.");

            response->error = true;
            response->error_desc = "Zero rotation vector passed.";

            timeoutTimer->reset();
            return;
        }
        rotVec.normalize();
        
        omni.freq = freq;
        omni.strength = strength;
        omni.offset = offset;
        omni.rotVector = rotVec;
        omni.rotBasis = makeBasis(rotVec);

        RCLCPP_INFO(this->get_logger(), 
            "Magnet: %lu\n"
            "Dipole: <%.3f, %.3f, %.3f>\n"
            "Strength: %.2f\n"
            "Frequency: %.2f",
            id, rotationVector.x, rotationVector.y, rotationVector.z, strength, freq
        );

        activeMagnets.push_back(&omni);
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
    durationTimer->cancel();
    spinTimer->cancel();

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

    activeMagnets.clear();
    timeoutTimer->reset();
    running = false;

    response->status = true;
}

/***************** HELPERS **********************/
Basis OmnimagnetDriverNode::makeBasis(const Eigen::Vector3d& axis) {
    Eigen::Vector3d n = axis.normalized();

    Eigen::Vector3d initVec =
        (std::abs(n.x()) < 0.9)
            ? Eigen::Vector3d::UnitX()
            : Eigen::Vector3d::UnitY();

    Eigen::Vector3d u = axis.cross(initVec).normalized();
    Eigen::Vector3d v = axis.cross(u).normalized();

    return Basis(u, v);
}