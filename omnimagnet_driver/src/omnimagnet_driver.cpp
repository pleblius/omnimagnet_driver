# include "rclcpp/rclcpp.hpp"
# include "../include/omnimagnet_driver/omnimagnet.hpp"
# include "comedilib.hpp"
# include "../include/omnimagnet_driver/omnimagnet_driver.hpp"

# include <atomic>
# include <chrono>
# include <thread>
# include <vector>

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
    
    this->omnimagnets = std::vector<OmniMagnet>(5);

    this->off << 0.0, 0.0, 0.0;

    this->setupHardware();
    this->setupMagnets();
}

/**
 * @brief Sets up D2A for magnet operation.
 * 
 * TODO:
 * 
 */
void OmnimagnetDriverNode :: setupHardware() {
    // TODO: Convert this into a .yaml file to load config
    subdev = 0;
    chan = 10;
    range = 0;
    aref = AREF_GROUND;

    D2A = comedi_open("/dev/comedi0");
    if(D2A == nullptr) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open D2A:");
        comedi_perror("comedi_open");
        // TODO: Add error publisher
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
        // TODO: Add error publisher
        std::exit(1);
    }

    int r26 = comedi_data_write(D2A, subdev, 26, 0, aref, inhib26);
    if (r26 < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to inhibit pin 26!");
        comedi_perror("Pin26");
        // TODO: Add error publisher
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
void OmnimagnetDriverNode :: setupMagnets() {
    // TODO: Replace this with parameterization and configuration loading
    omnimagnets[0].SetProp(1.35/1000.0 , 121, 122, 132, 17, 2, 0, 18, true, D2A);     // Omni #1, left upper 
    omnimagnets[1].SetProp(1.35/1000.0 , 121, 122, 132, 17, 3, 11, 19, true, D2A);    // Omni #2, center upper 
    omnimagnets[2].SetProp(1.35/1000.0 , 121, 122, 132, 17, 4, 12, 20, true, D2A);    // Omni #3, right upper 
    omnimagnets[3].SetProp(1.35/1000.0 , 121, 122, 132, 17, 5, 13, 21, true, D2A);     // Omni #4, right lower 
    omnimagnets[4].SetProp(1.35/1000.0 , 121, 122, 132, 17, 6, 14, 22, true, D2A);   // Omni #5, left lower
    // omnimagnets[5].SetProp(1.35/1000.0 , 121, 122, 132, 17, 7, 15, 23, true, D2A); omnimagnets[5].UpdateMapping();  // spare (potentially for Super Omni), DON'T TURN ON TILL CONNECTED

    for (auto & omni : omnimagnets) {
        omni.UpdateMapping();
    }

    RCLCPP_INFO(this->get_logger(), "Magnet setup complete.");
}

/**
 * @brief Turns off system for safe shutdown.
 * 
 * TODO:
 * 
 */
void OmnimagnetDriverNode :: shutdown() {
    int i = 1;
    for (auto & omni : omnimagnets) {
        omni.SetCurrent(off);
        RCLCPP_INFO(this->get_logger(), "Magnet %d turned off.", i++);
    }

    RCLCPP_INFO(this->get_logger(), "All magnets turned off.");

    bool fail = false;
    int retval = comedi_data_write(D2A, subdev, 25, 0, AREF_GROUND, 16383.0*2./4.);
    if (retval < 0) {
        RCLCPP_WARN(this->get_logger(), "Failed to shut down D2A Pin 25!");
        fail = true;
    }
    retval = comedi_data_write(D2A, subdev, 26, 0, AREF_GROUND, 16383.0*2./4.);
    if (retval < 0) {
        RCLCPP_WARN(this->get_logger(), "Failed to shut down D2A Pin 26!");
        fail = true;
    }

    if (!fail) {
        RCLCPP_INFO(this->get_logger(), "Successfully shut down hardware.");
    }
}