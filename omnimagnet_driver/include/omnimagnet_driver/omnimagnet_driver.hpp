#pragma once

# include "rclcpp/rclcpp.hpp"
# include "comedilib.hpp"
# include "omnimagnet.hpp"
# include <vector>

class OmnimagnetDriverNode : public rclcpp::Node {

public:
    
    OmnimagnetDriverNode();
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
    lsampl_t maxdata25{16383};
    lsampl_t maxdata26{16383};

    Eigen::Vector3d off;

    void setupHardware();
    void setupMagnets();
};