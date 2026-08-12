#include <iostream>
#include "../include/omnimagnet_driver/omnimagnet.hpp"
#include "comedilib.hpp"

// Helper Functions
namespace {
	/**
	 * @brief Checks if a given matrix is a valid rotation matrix.
	 * 
	 * This function checks if the provided 3x3 matrix is a valid rotation matrix by verifying that its determinant is close to 1, 
	 * its transpose is equal to its inverse, and it is orthogonal.
	 * 
	 * @param matrix The 3x3 matrix to be checked.
	 * @param tolerance The tolerance for numerical comparisons (default is 1e-6).
	 * 
	 * @return true if the matrix is a valid rotation matrix, false otherwise.
	 * 
	 * @throws std::invalid_argument if the tolerance is not positive.
	 */
	bool isValidRotationMatrix(const Eigen::Matrix3d& matrix, const double tolerance = 1e-6) {
		if (tolerance <= 0) {
			throw std::invalid_argument("Tolerance must be positive.");
		}

		// Check if the determinant is close to 1
		double det = matrix.determinant();
		if (std::abs(det - 1.0) > tolerance) {
			return false;
		}

		// Check if the transpose is equal to the inverse
		Eigen::Matrix3d transpose = matrix.transpose();
		Eigen::Matrix3d inverse = matrix.inverse();
		if (!transpose.isApprox(inverse, tolerance)) {
			return false;
		}

		// Check orthogonality: R^T*R should be the identity matrix
		Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
		if (!(transpose * matrix).isApprox(identity, tolerance)) {
			return false;
		}

		return true;
	}
}

/**
 * @brief Default constructor for the OmniMagnet class.
 * 
 * This constructor initializes the OmniMagnet object with default values for its member variables.
 * It sets the current vector to zero and sets the mapping and frame matrices as the identity matrix.
 */
OmniMagnet::OmniMagnet() {
	current_ = Eigen::Vector3d::Zero();
	mapping_ = Eigen::Matrix3d::Identity();
	frame_ = Eigen::Matrix3d::Identity();
};

/**
 * @brief Construct that sets magnet properties.
 * 
 * Uses a MagnetConfig struct to bulk-set the properties of the OmniMagnet, then updates the mapping matrix.
 * 
 * @param config MagnetConfig struct containing magnet properties.
 * 
 * @throws std::invalid_argument if config.frame cannot be converted to a valid rotation matrix.
 */
OmniMagnet::OmniMagnet(const MagnetConfig& config)
	:	id_{config.id},
		wireWidth_{config.wireWidth},
		innerWireLength_{config.innerWireLength},
		midWireLength_{config.midWireLength},
		outerWireLength_{config.outerWireLength},
		coreSize_{config.coreSize},
		estimate_{config.estimate}
{
	cardPinNumbers_ = std::vector<int>{
		config.innerChannel,
		config.midChannel,
		config.outerChannel
	};

	setFrame(config.frame);

	updateMapping();
}

/**
 * @brief Sets the magnet's properties
 * 
 * Uses a MagnetConfig struct to bulk-set the properties of the OmniMagnet object, then updates the mapping matrix.
 * 
 * @param config MagnetConfig struct containing magnet properties.
 * 
 * @throws std::invalid_argument if config.frame cannot be converted to a valid rotation matrix. 
 */
void OmniMagnet::setProp(const MagnetConfig& config) {
	id_ = config.id;

	wireWidth_ = config.wireWidth;
	innerWireLength_ = config.innerWireLength;
	midWireLength_ = config.midWireLength;
	outerWireLength_ = config.outerWireLength;
	coreSize_ = config.coreSize;
	
	estimate_ = config.estimate;
	
	cardPinNumbers_ = std::vector<int>{
		config.innerChannel,
		config.midChannel,
		config.outerChannel
	};

	setFrame(config.frame);

	updateMapping();
}

/**
 * @brief Sets the omnimagnet's id.
 * 
 * @param id The new magnet id.
 */
void OmniMagnet::setID(int id) {
	id_ = id;
}

/**
 * @brief Gets the omnimagnet's id.
 * 
 * @return ID.
 */
[[nodiscard]] int OmniMagnet::ID() const {
	return id_;
}

/**
 * @brief Sets the magnet's pin numbers.
 * 
 * @param numbers A vector containing the pin numbers to be assigned.
 */
void OmniMagnet::setPinNumbers(const std::vector<int>& numbers) {
	cardPinNumbers_ = numbers;
}

/**
 * @brief Gets the magnet's pin numbers.
 * 
 * @return A vector containing the pin numbers.
 */
const std::vector<int>& OmniMagnet::pinNumbers() const {
	return cardPinNumbers_;
}

/**
 * @brief Sets the frame of reference for the OmniMagnet object.
 * 
 * This method sets the frame of reference for the OmniMagnet object using a 3x3 Eigen matrix.
 * The frame is used to transform the magnet's actual X-Y-Z output frame to the idealized frame described in frames/default.png
 * 
 * @param frame A 3x3 Eigen matrix representing the frame of reference.
 * 
 * @throws std::invalid_argument if the input matrix is not a valid rotation matrix.
 */
void OmniMagnet::setFrame(const Eigen::Matrix3d frame) {
	if (!isValidRotationMatrix(frame)) {
		throw std::invalid_argument("Input frame is not a valid rotation matrix.");
	}

	frame_ = frame;
}

/**
 * @brief Sets the frame of reference for the OmniMagnet object using a vector of doubles.
 * 
 * This method sets the frame of reference for the OmniMagnet object using a vector of doubles.
 * The vector must contain exactly 9 elements, which will be interpreted as a 3x3 matrix in row-major order. 
 * If the vector does not contain exactly 9 elements, it produces and error and returns without modifying the frame.
 * The frame is used to transform the magnet's actual X-Y-Z output frame to the idealized frame described in frames/default.png
 * 
 * @param list A vector of doubles representing the frame of reference in row-major order.
 * 
 * @throws std::invalid_argument if list cannot be converted to 3x3 matrix or if matrix is not a valid rotation matrix.
 * 
 * @note The vector must contain exactly 9 elements to be interpreted as a 3x3 matrix.
 */
void OmniMagnet::setFrame(const std::vector<double>& list) {
	if (list.size() != 9) {
		throw std::invalid_argument("List does not have 9 entries.");
	}

	Eigen::Matrix3d matrix = Eigen::Map<const Eigen::Matrix3d, Eigen::RowMajor>(list.data());

	if (!isValidRotationMatrix(matrix)) {
		throw std::invalid_argument("Matrix is not a valid rotation matrix.");
	}

	frame_ = matrix;
}

/**
 * @brief Retrieves the frame of reference for the OmniMagnet object.
 * 
 * This method returns the current frame of reference for the OmniMagnet object as a 3x3 Eigen matrix.
 * The frame is used to transform the magnet's actual X-Y-Z output frame to the idealized frame described in frames/default.png
 * 
 * @return A 3x3 Eigen matrix representing the frame of reference.
 */
const Eigen::Matrix3d& OmniMagnet::frame() const {
	return frame_;
}

/**
 * @brief Updates the mapping matrix for the OmniMagnet object.
 * 
 * This method generates the mapping matrix (3x3) that maps dipole moments (in Am^2) to current densities (in A/m^2).
 * The mapping is based on the wire dimensions, core size, and orientation of the magnet.
 * If the estimation flag is set, it calculates the mapping matrix using a predefined constant.
 * The mapping matrix is then decomposed using complete orthogonal decomposition for later use in solving for current densities.
 * 
 * TODO: Rewrite to be more intuitive and more dynamic, as the mapping constant is currently hardcoded and may need to be adjusted based on experimental calibration.
 */
void OmniMagnet::updateMapping() /* Generates the mapping (3X3) matrix*/ 
{
	if (estimate_) {
		constexpr double mapping_constant = .05145 * 2 * .825 * .115 * .115 * .115 * .115;
		mapping_ = Eigen::MatrixXd::Identity(3,3) * mapping_constant;
		decomp_  = mapping_.completeOrthogonalDecomposition();
	}
	else {
		// No alternative methods were provided in original code, so this is a placeholder for future development
		std::cerr << "No method, use the estimate method" << std::endl;
	}
};

/**
 * @brief Retrieves the mapping matrix for the OmniMagnet object.
 * 
 * This method returns the current mapping matrix (3x3) that maps dipole moments (in Am^2) to current densities (in A/m^2).
 * 
 * @return A 3x3 Eigen matrix representing the mapping matrix.
 */
const Eigen::Matrix3d& OmniMagnet::mapping() const {
	return mapping_;	
};

/**
 * @brief Sets the current for the OmniMagnet object.
 * 
 * @param current A 3x1 Eigen vector representing the desired current values.
 */
void OmniMagnet::setCurrent(Eigen::Vector3d current) {
    current_ = current;
}

/**
 * @brief Retrieves the current values for the OmniMagnet object.
 * 
 * This method returns the current values (3x1) for the OmniMagnet object.
 * The current is represented as a 3x1 Eigen vector, where each component corresponds to the current in the x, y, and z directions.
 * 
 * @return A 3x1 Eigen vector representing the current values.
 */
const Eigen::Vector3d& OmniMagnet::current() const {
	return current_;
}

/**
 * @brief Converts a dipole moment to the corresponding current values for the OmniMagnet object.
 * 
 * This method takes a dipole moment (3x1) as input and calculates the corresponding current values (3x1) needed to achieve that dipole moment.
 * The mapping matrix is used to perform the conversion, and the result is scaled by the wire area to obtain the current values.
 * 
 * @param dipole A 3x1 Eigen vector representing the desired dipole moment in Am^2.
 * 
 * @return A 3x1 Eigen vector representing the corresponding current values in A.
 */
Eigen::Vector3d OmniMagnet::dipoleToCurrent(Eigen::Vector3d dipole) const {
	return decomp_.solve(frame_.transpose() * dipole) * (wireWidth_ * wireWidth_);
}

// Below functions are commented out because they are not used in the current implementation, but may be useful for future development or debugging purposes.

// [[maybe_unused]]
// static Eigen::MatrixXd AshkanPseudoinverse(Eigen::MatrixXd A, double singularMinToMaxRatio)
// {
//     Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV ); // computes the SVD
//     Eigen::MatrixXd S_inv(svd.matrixV().cols(),svd.matrixU().rows());
//     S_inv.setZero();
//     for( int i = 0; i < std::min(A.rows(),A.cols()); i++)
//     {
//         double val = 0;
//         if( svd.singularValues()[i] > svd.singularValues()[0]*singularMinToMaxRatio )// threashold singular values anything less than 1/1000 of the max is set to 0
//             val = 1.0 / svd.singularValues()[i];
//         S_inv(i,i) = val;
//     }
//     Eigen::MatrixXd answer;
//     answer = svd.matrixV()*(S_inv*(svd.matrixU().transpose()));
//     return answer;
// };

// [[maybe_unused]]
// static Eigen::MatrixXd Pseudoinverse(Eigen::MatrixXd A)
// {
//     Eigen::MatrixXd B = A.completeOrthogonalDecomposition().pseudoInverse();
//     return B;
// };