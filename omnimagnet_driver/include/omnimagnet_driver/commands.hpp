#pragma once

#include "omnimagnet.hpp"

#include <variant>

/**
 * @brief Set of orthonormal vectors spanning a 2D rotation plane in 3D space.
 * 
 * The two vectors u and v are unit length, mutually orthogonal, and together define the orientation of a 2D rotation plane in 3D space.
 * 
 * @note Basis cannot be updated once created.
 */
class Basis {

public:
    /**
     * @brief Default constructor generates a basis with <1, 0, 0> and <0, 1, 0>.
     */
    Basis() {
        u_ << 1, 0, 0;
        v_ << 0, 1, 0;
    }

	/**
     * @brief Constructor that takes two 3x1 vectors to construct a basis set.
     * 
     * Constructs an orthonormal basis spanning the plane defined by the two input vectors using Gram-Schmidt.
     * The first unit vector will coincide with the first input vector.
     * 
     * @param u The first 3x1 Eigen vector.
     * @param v The second 3x1 Eigen vector.
     * 
     * @throws std::invalid_argument if either vector is zero or if the vectors are parallel.
     * 
     * @note The input vectors do not need to be normalized to call this function.
     */
	Basis(const Eigen::Vector3d &u, const Eigen::Vector3d &v) {
        // Check for zero vectors
        if (u.isApprox(Eigen::Vector3d::Zero()) || v.isApprox(Eigen::Vector3d::Zero())) {
            throw std::invalid_argument("Vectors cannot be zero vectors.");
        }

        auto u1 = u;
        auto proj = u * v.dot(u)/u.dot(u);
        auto u2 = v - proj;
        
        // Check for parallel vectors
        if (u2.isApprox(Eigen::Vector3d::Zero())) {
            throw std::invalid_argument("Vectors cannot be parallel.");
        }

		u_ = u1.normalized();
        v_ = u2.normalized();
	}

    /**
     * @brief Gets the 'u' basis vector.
     * 
     * @return 3x1 Eigen vector 'u'.
     */
    [[nodiscard]] const Eigen::Vector3d& u() const noexcept {
        return u_;
    }

    /**
     * @brief Gets the 'v' basis vector.
     * 
     * @return 3x1 Eigen vector 'v'.
     */
    [[nodiscard]] const Eigen::Vector3d& v() const noexcept{
        return v_;
    }

private:
    Eigen::Vector3d u_;
	Eigen::Vector3d v_;
};

struct ConstantDipoleCommand {
    OmniMagnet* omni;
    double strength;
    Eigen::Vector3d vector;
    
    Eigen::Vector3d currentAtTime(double time) const;
};

struct ConstantCurrentCommand {
    OmniMagnet* omni;
    double strength;
    Eigen::Vector3d vector;

    Eigen::Vector3d currentAtTime(double time) const;
};

struct RotatingDipoleCommand {
    OmniMagnet* omni;
    double freq;
    double strength;
    double offset;
    Basis basis;

    Eigen::Vector3d currentAtTime(double time) const;
};

struct RotatingCurrentCommand {
    OmniMagnet* omni;
    double freq;
    double strength;
    double offset;
    Basis basis;

    Eigen::Vector3d currentAtTime(double time) const;
};

using ActiveMagnetCommand = std::variant<
    ConstantDipoleCommand,
    ConstantCurrentCommand,
    RotatingDipoleCommand,
    RotatingCurrentCommand
>;

Eigen::Vector3d currentAtTime(const ActiveMagnetCommand& command, double time);

const OmniMagnet* commandMagnet(const ActiveMagnetCommand& command);