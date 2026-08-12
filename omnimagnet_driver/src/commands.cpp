#include "../include/omnimagnet_driver/commands.hpp"

Eigen::Vector3d ConstantDipoleCommand::currentAtTime([[maybe_unused]] double time) const {
    return omni->dipoleToCurrent(strength * vector);
}

Eigen::Vector3d ConstantCurrentCommand::currentAtTime([[maybe_unused]] double time) const {
    return strength * vector;
}

Eigen::Vector3d RotatingDipoleCommand::currentAtTime(double time) const {
    double theta = 2.0 * M_PI * freq * time + offset;

    Eigen::Vector3d dipole = strength * (
        std::cos(theta) * basis.u() +
        std::sin(theta) * basis.v()
    );

    return omni->dipoleToCurrent(dipole);
}

Eigen::Vector3d RotatingCurrentCommand::currentAtTime(double time) const {
    double theta = 2.0 * M_PI * freq * time + offset;

    Eigen::Vector3d current = strength * (
        std::cos(theta) * basis.u() +
        std::sin(theta) * basis.v()
    );

    return current;
}

Eigen::Vector3d currentAtTime(const ActiveMagnetCommand& command, double time) {
    return std::visit(
        [time](const auto& cmd) {
            return cmd.currentAtTime(time);
        },
        command
    );
}

const OmniMagnet* commandMagnet(const ActiveMagnetCommand& command) {
    return std::visit(
        [](const auto& cmd) {
            return cmd.omni;
        },
        command
    );
}