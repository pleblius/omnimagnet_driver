# include "rclcpp/rclcpp.hpp"
# include "../include/omnimagnet_driver/omnimagnet.hpp"
# include "comedilib.hpp"
# include "../include/omnimagnet_driver/omnimagnet_driver.hpp"

# include <atomic>
# include <chrono>
# include <thread>
# include <vector>
# include <chrono>
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
        std::chrono::duration<double>(20.0),
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
    auto currentTime = std::chrono::high_resolution_clock::now();
    auto deltaTime   = currentTime - startTime;

    // Calculate new dipoles
    for (auto & omni : activeMagnets) {
        
    }

    // Send currents to each omnimagnet
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
    RCLCPP_INFO(this->get_logger(), "Operation complete.");

    // Delete timer until new order is received
    durationTimer->cancel();

    for (auto& omni : activeMagnets) {
        int retval = omni->SetCurrent(this->offVector);
        if (retval <= 0) {
            auto msg = omnimagnet_interfaces::msg::ErrorMessage();
            msg.error_desc = "Failed to shut down magnet" + std::to_string(omni->ID);
        }
    }

    activeMagnets.clear();

    auto msg = omnimagnet_interfaces::msg::FinishedMessage();


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

}


void OmnimagnetDriverNode::mmcCallback(
    const omnimagnet_interfaces::srv::MultiMagnetConstant::Request::SharedPtr request,
    const omnimagnet_interfaces::srv::MultiMagnetConstant::Response::SharedPtr response
)
{
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

    this->create_wall_timer(
        std::chrono::duration<double>(duration),
        std::bind(&OmnimagnetDriverNode::durationCallback, this)
    );


}


void OmnimagnetDriverNode::mmrCallback(
    const omnimagnet_interfaces::srv::MultiMagnetRotation::Request::SharedPtr request,
    const omnimagnet_interfaces::srv::MultiMagnetRotation::Response::SharedPtr response
) {

}


void OmnimagnetDriverNode::resetCallback(
    const omnimagnet_interfaces::srv::DriverReset::Request::SharedPtr request,
    const omnimagnet_interfaces::srv::DriverReset::Response::SharedPtr response
) {

}