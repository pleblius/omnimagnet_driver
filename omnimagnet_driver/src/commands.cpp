#include "../include/omnimagnet_driver/commands.hpp"

/**
 * @brief Gets the current for a constant-dipole command.
 */
Eigen::Vector3d ConstantDipoleCommand::currentAtTime([[maybe_unused]] double time) const {
    return omni->dipoleToCurrent(strength * vector);
}

/**
 * @brief Gets the current for a constant-current command.
 */
Eigen::Vector3d ConstantCurrentCommand::currentAtTime([[maybe_unused]] double time) const {
    return strength * vector;
}

/**
 * @brief Gets the required current for a rotating dipole command for the elapsed time.
 */
Eigen::Vector3d RotatingDipoleCommand::currentAtTime(double time) const {
    double theta = 2.0 * M_PI * freq * time + offset;

    Eigen::Vector3d dipole = strength * (
        std::cos(theta) * basis.u() +
        std::sin(theta) * basis.v()
    );

    return omni->dipoleToCurrent(dipole);
}

/**
 * @brief Gets the required current for a rotating current command for the elapsed time.
 */
Eigen::Vector3d RotatingCurrentCommand::currentAtTime(double time) const {
    double theta = 2.0 * M_PI * freq * time + offset;

    Eigen::Vector3d current = strength * (
        std::cos(theta) * basis.u() +
        std::sin(theta) * basis.v()
    );

    return current;
}

/**
 * @brief Gets the required current for the command variant.
 */
Eigen::Vector3d currentAtTime(const ActiveMagnetCommand& command, double time) {
    return std::visit(
        [time](const auto& cmd) {
            return cmd.currentAtTime(time);
        },
        command
    );
}

/**
 * @brief Gets the magnet ID for the command variant.
 */
const OmniMagnet* commandMagnet(const ActiveMagnetCommand& command) {
    return std::visit(
        [](const auto& cmd) {
            return cmd.omni;
        },
        command
    );
}