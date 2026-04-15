# include "rclcpp/rclcpp.hpp"
# include "../include/omnimagnet_driver/omnimagnet.hpp"
# include "comedilib.hpp"

# include <atomic>
# include <chrono>
# include <thread>
# include <vector>

class OmnimagnetDriverNode : public rclcpp::Node
{
    public:
    OmnimagnetDriverNode() : Node("omnimagnet_driver") {
        omnimagnets = std::vector<OmniMagnet>(5);

        off << 0.0, 0.0, 0.0;

        this->setupHardware();
        this->setupMagnets();

        x_timer = this->create_wall_timer(std::chrono::duration<double>(5.0),
            std::bind(&OmnimagnetDriverNode::xTimer, this));
        y_timer = this->create_wall_timer(std::chrono::duration<double>(5.0),
            std::bind(&OmnimagnetDriverNode::yTimer, this));
        z_timer = this->create_wall_timer(std::chrono::duration<double>(5.0),
            std::bind(&OmnimagnetDriverNode::zTimer, this));

        y_timer->cancel();
        z_timer->cancel();
    }

    void shutdown();

    private:
    int64_t num_magnets;
    std::vector<OmniMagnet> omnimagnets;

    // D2A card
    comedi_t *D2A;
    int subdev;
    int chan;
    int range;
    int aref;

    Eigen::Vector3d off;

    rclcpp::TimerBase::SharedPtr x_timer;
    rclcpp::TimerBase::SharedPtr y_timer;
    rclcpp::TimerBase::SharedPtr z_timer;

    void setupHardware();
    void setupMagnets();

    void xTimer();
    void yTimer();
    void zTimer();
};

int main (int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OmnimagnetDriverNode>();
    rclcpp::on_shutdown(
        std::bind(&OmnimagnetDriverNode::shutdown, node)
    );

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}

void OmnimagnetDriverNode :: setupHardware() {
    // TODO: Convert this into a .yaml file to load config
    subdev = 0;
    chan = 10;
    range = 0;
    aref = AREF_GROUND;

    D2A = comedi_open("/dev/comedi0");
    if(D2A == nullptr) {
        RCLCPP_WARN(this->get_logger(), "Failed to open D2A:");
        comedi_perror("comedi_open");
        std::exit(1);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Activating the amplifiers inhibits. Pins are 25&26   --- DON'T MODIFIY ---
    lsampl_t maxdata25 = comedi_get_maxdata(D2A, subdev, 25);
    lsampl_t maxdata26 = comedi_get_maxdata(D2A, subdev, 26);

    lsampl_t v25 = maxdata25 * 3 / 4;
    lsampl_t v26 = maxdata26 * 3 / 4;

    int r25 = comedi_data_write(D2A, subdev, 25, 0, aref, v25);
    int r26 = comedi_data_write(D2A, subdev, 26, 0, aref, v26);

    std::cout << "ch25 init val = " << v25 << ", ret = " << r25 << std::endl;
    std::cout << "ch26 init val = " << v26 << ", ret = " << r26 << std::endl;

    RCLCPP_INFO(this->get_logger(), "Hardware setup complete.");
}

void OmnimagnetDriverNode :: setupMagnets() {
    // TODO: Replace this with parameteriation and configuration loading
    omnimagnets[0].SetProp(1.35/1000.0 , 121, 122, 132, 17, 2, 0, 18, true, D2A); omnimagnets[0].UpdateMapping();      // Omni #1, left upper 
    omnimagnets[1].SetProp(1.35/1000.0 , 121, 122, 132, 17, 3, 11, 19, true, D2A); omnimagnets[1].UpdateMapping();     // Omni #2, center upper 
    omnimagnets[2].SetProp(1.35/1000.0 , 121, 122, 132, 17, 4, 12, 20, true, D2A); omnimagnets[2].UpdateMapping();     // Omni #3, right upper 
    omnimagnets[3].SetProp(1.35/1000.0 , 121, 122, 132, 17, 5, 13, 21, true, D2A); omnimagnets[3].UpdateMapping();     // Omni #4, right lower 
    omnimagnets[4].SetProp(1.35/1000.0 , 121, 122, 132, 17, 6, 14, 22, true, D2A); omnimagnets[4].UpdateMapping();     // Omni #5, left lower
    // omnimagnets[5].SetProp(1.35/1000.0 , 121, 122, 132, 17, 7, 15, 23, true, D2A); omnimagnets[5].UpdateMapping();  // spare (potentially for Super Omni), DON'T TURN ON TILL CONNECTED

    RCLCPP_INFO(this->get_logger(), "Magnet setup complete.");
}

void OmnimagnetDriverNode :: shutdown() {
    int i = 0;
    for (auto &omni : omnimagnets) {
        omni.SetCurrent(off);
        RCLCPP_INFO(this->get_logger(), "Magnet %d turned off.", ++i);
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

void OmnimagnetDriverNode :: xTimer() {
    Eigen::Vector3d current;

    current << 10.0, 0.0, 0.0;

    auto & magnet = omnimagnets[4];

    magnet.SetCurrent(current);

    x_timer->cancel();
    y_timer->reset();
}

void OmnimagnetDriverNode :: yTimer() {
    Eigen::Vector3d current;

    current << 0.0, 10.0, 0.0;

    auto & magnet = omnimagnets[4];

    magnet.SetCurrent(current);

    y_timer->cancel();
    z_timer->reset();
}

void OmnimagnetDriverNode :: zTimer() {
        Eigen::Vector3d current;

    current << 0.0, 0.0, 10.0;

    auto & magnet = omnimagnets[4];

    magnet.SetCurrent(current);

    z_timer->cancel();
    x_timer->reset();
}