#include <iostream>
#include "../include/omnimagnet_driver/omnimagnet.hpp"
#include "comedilib.hpp"

namespace {
	/**
	 * @brief Maps a value from one range to another.
	 * 
	 * This function takes an input value and maps it from the range [val_min, val_max] to the range [range_min, range_max].
	 * If the input value is outside the input range, it will be clamped to the nearest boundary of the output range.
	 * 
	 * @param value The input value to be mapped.
	 * @param val_min The minimum value of the input range.
	 * @param val_max The maximum value of the input range.
	 * @param range_min The minimum value of the output range.
	 * @param range_max The maximum value of the output range.
	 * 
	 * @return The mapped value in the output range.
	 * 
	 * @tparam inType The type of the input value.
	 * @tparam outType The type of the output value.
	 * 
	 * @throws std::invalid_argument if the input range has zero width.
	 * 
	 * @note Generic types must support arithmetic operations and type casting to double.
	 */
	template <typename inType, typename outType>
	outType map_range(inType value, inType val_min, inType val_max, outType range_min, outType range_max) {
		if (val_min == val_max) {
			throw std::invalid_argument("Input range has zero width.");
		}

		if (value >= val_max)
			return range_max;
		if (value <= val_min)
			return range_min;

		return static_cast<outType>(
			static_cast<double>(value - val_min) * 
			static_cast<double>(range_max - range_min) / 
			static_cast<double>(val_max - val_min) + 
			range_min
		);
	}

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
 * It sets the current vector to zero, initializes the mapping matrix to zero, and sets the frame to the identity matrix.
 */
OmniMagnet::OmniMagnet(){
	this->current_ << 	0.0, 0.0 ,0.0;

	this->mapping_ << 	0.0, 0.0 ,0.0,
						0.0, 0.0 ,0.0,
						0.0, 0.0 ,0.0;
						
	frame_ = Eigen::Matrix3d::Identity();
};

/**
 * @brief Constructs an OmniMagnet object with specified properties.
 * 
 * This constructor initializes an OmniMagnet object with the given wire dimensions, core size, pin numbers, estimation flag, and a pointer to the comedi_t card.
 * It calls the default constructor to initialize member variables and then sets the properties using the SetProp method.
 * 
 * @param wirewidth The width of the wire in meters.
 * @param wirelenin The length of the inner wire in meters.
 * @param wirelenmid The length of the middle wire in meters.
 * @param wirelenout The length of the outer wire in meters.
 * @param coresize The size of the core in meters.
 * @param pinin The pin number for the inner wire.
 * @param pinmid The pin number for the middle wire.
 * @param pinout The pin number for the outer wire.
 * @param estimate A boolean flag indicating whether to use estimation for mapping.
 * @param card A pointer to the comedi_t card for hardware communication.
 * 
 * TODO: Consider adding validation for the input parameters to ensure they are within acceptable ranges.
 * TODO: Consider adding error handling for the comedi_t card pointer to ensure it is valid before use.
 * TODO: Consider rewriting so there aren't so many parameters, as it may be difficult to maintain and understand. Perhaps use a struct or class to encapsulate the properties.
 */
OmniMagnet::OmniMagnet(
	double wirewidth,
	double wirelenin,
	double wirelenmid,
	double wirelenout,
	double coresize,
	int pinin,
	int pinmid,
	int pinout,
	bool estimate,
	comedi_t *card
) : OmniMagnet() {
 	SetProp(wirewidth, wirelenin, wirelenmid, wirelenout, coresize, pinin, pinmid, pinout, estimate, card);
};

/**
 * @brief Sets the properties of the OmniMagnet object.
 * 
 * This method sets the wire dimensions, core size, pin numbers, estimation flag, and the comedi_t card pointer for the OmniMagnet object.
 * It updates the member variables accordingly.
 * 
 * Sets the default orientation to 0 degrees.
 * 
 * TODO: Rewrite entirely
 * 
 * @param wirewidth The width of the wire in meters.
 * @param wirelenin The length of the inner wire in meters.
 * @param wirelenmid The length of the middle wire in meters.
 * @param wirelenout The length of the outer wire in meters.
 * @param coresize The size of the core in meters.
 * @param pinin The pin number for the inner wire.
 * @param pinmid The pin number for the middle wire.
 * @param pinout The pin number for the outer wire.
 * @param estimate A boolean flag indicating whether to use estimation for mapping.
 * @param card A pointer to the comedi_t card for hardware communication.
 */
void OmniMagnet::SetProp(
	double wirewidth,
	double wirelenin,
	double wirelenmid,
	double wirelenout,
	double coresize,
	int pinin,
	int pinmid,
	int pinout, 
	bool estimate,
	comedi_t *card
) 
{
	this->card_ = card;
	this->wire_width = wirewidth;
	this->wire_len_in = wirelenin;
	this->wire_len_mid = wirelenmid;
	this->wire_len_out = wirelenout;
	this->core_size = coresize;
	this->orientation_ = 0.0;

	D2A_pin_number << pinin, pinmid, pinout;
	estimate_ = estimate;
};

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
void OmniMagnet::SetFrame(const Eigen::Matrix3d frame) {
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
void OmniMagnet::SetFrame(const std::vector<double>& list) {
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
Eigen::Matrix3d OmniMagnet::GetFrame() {
	return frame_;
}

/**
 * @brief Updates the mapping matrix for the OmniMagnet object.
 * 
 * This method generates the mapping matrix (3x3) that maps dipole moments (in Am^2) to current densities (in A/m^2).
 * The mapping is based on the wire dimensions, core size, and orientation of the magnet.
 * If the estimation flag is set, it calculates the mapping matrix using a predefined constant and applies the orientation rotation.
 * The mapping matrix is then decomposed using complete orthogonal decomposition for later use in solving for current densities.
 * 
 * TODO: Rewrite to be more intuitive and more dynamic, as the mapping constant is currently hardcoded and may need to be adjusted based on experimental calibration.
 */
void OmniMagnet::UpdateMapping() /* Generates the mapping (3X3) matrix*/ 
{
	if (estimate_) {
		// Generates mapping of dipole (Am^2) to current density (A/m^2)
		// Equation (13) from Omnimagnet Paper
		// float mapping_constant = 51.45 * 2 * 0.825; 	// Tuned value to map current to desired dipole strength 
		
		// Rewritten from original code to allow compile-time solution
		constexpr double mapping_constant = .05145 * 2 * .825 * .115 * .115 * .115 * .115;

		// mapping_ = Eigen::MatrixXd::Identity(3,3)*((mapping_constant * pow (10.0, -3.0) * pow(0.115,4)));
		mapping_ = Eigen::MatrixXd::Identity(3,3) * mapping_constant;

		// mapping_ = Eigen::MatrixXd::Identity(3,3)*((51.45 * pow (10.0, -3.0) * (0.115))/(wire_width*wire_width));

		// I'm not sure why this is included because orientation is always set to 0
		// Included because it was in the original code, but I don't think it does anything
		// Consider removing it if it is not needed, as it may be confusing to future developers
		axis_rot_Z = (Eigen::AngleAxisd(orientation_*M_PI/180.0, Eigen::Vector3d::UnitZ()));
		axis_rot_Z.normalize();

		mapping_ = (axis_rot_Z * mapping_);
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
Eigen::Matrix3d OmniMagnet::GetMapping()
{
	return mapping_;	
};

/**
 * @brief Sets the current for the OmniMagnet object.
 * 
 * This method sets the current values for the OmniMagnet object and updates the corresponding D2A signal rates.
 * Current is represented as a 3x1 Eigen vector, where each component corresponds to the current in the x, y, and z directions.
 * 
 * @param current A 3x1 Eigen vector representing the desired current values.
 * 
 * @return An integer indicating the success or failure of the operation. 
 * A value of 1 indicates success, while a negative value indicates an error in writing to the D2A channels.
 */
int OmniMagnet::SetCurrent(Eigen::Vector3d current)
{
	// Store current as a member variable for later retrieval. Possibly unneeded
    this->current_ = current;

    lsampl_t d0 = CurrentD2A(current[0]);
    lsampl_t d1 = CurrentD2A(current[1]);
    lsampl_t d2 = CurrentD2A(current[2]);

	// Write the D2A values to the corresponding channels using comedi_data_write. 
	// Check for errors after each write
    int retval;
    retval = comedi_data_write(card_, subdev, D2A_pin_number[0], 0, AREF_GROUND, d0);
    if (retval < 0)
		goto fail_state1;

    retval = comedi_data_write(card_, subdev, D2A_pin_number[1], 0, AREF_GROUND, d1);
    if (retval < 0)
		goto fail_state2;

    retval = comedi_data_write(card_, subdev, D2A_pin_number[2], 0, AREF_GROUND, d2);
    if (retval < 0)
		goto fail_state3;

	return 1;

	// If any write fails, reset the previous channels to 0 to avoid leaving the magnet in an undefined state.
	fail_state3:
		comedi_data_write(card_, subdev, D2A_pin_number[2], 0, AREF_GROUND, CurrentD2A(0));
	fail_state2:
		comedi_data_write(card_, subdev, D2A_pin_number[1], 0, AREF_GROUND, CurrentD2A(0));
	fail_state1:
		comedi_data_write(card_, subdev, D2A_pin_number[0], 0, AREF_GROUND, CurrentD2A(0));
		
		return retval;
}

/**
 * @brief Retrieves the current values for the OmniMagnet object.
 * 
 * This method returns the current values (3x1) for the OmniMagnet object.
 * The current is represented as a 3x1 Eigen vector, where each component corresponds to the current in the x, y, and z directions.
 * 
 * @return A 3x1 Eigen vector representing the current values.
 */
Eigen::Vector3d OmniMagnet::GetCurrent()
{
	return current_;
};

/**
 * @brief Retrieves the orientation of the OmniMagnet object.
 * 
 * This method returns the current orientation of the OmniMagnet object in degrees.
 * The orientation is represented as a double value, where 0 degrees corresponds to the default orientation.
 * 
 * @return A double representing the orientation in degrees.
 * 
 * @note The orientation seems to correspond to a rotation about the Z-axis, but doesn't account for other frame rotations.
 * Considering removing entirely and relying on the frame_ matrix to handle all rotations, as it is more general and flexible.
 */
double OmniMagnet::GetOrientation()
{
	return orientation_;	
};

/**
 * @brief Sets the orientation of the OmniMagnet object.
 * 
 * This method sets the orientation of the OmniMagnet object in degrees.
 * The orientation is represented as a double value, where 0 degrees corresponds to the default orientation.
 * The current mapping matrix is updated to reflect the new orientation.
 * 
 * @param orientation A double representing the desired orientation in degrees.
 * 
 * @note The orientation seems to correspond to a rotation about the Z-axis, but doesn't account for other frame rotations.
 * Considering removing entirely and relying on the frame_ matrix to handle all rotations, as it is more general and flexible.
 */
void OmniMagnet::SetOrientation( double orientation)
{
	orientation_ = orientation;

	UpdateMapping();
};

/**
 * @brief Sets the maximum D2A value for the OmniMagnet object.
 * 
 * This method sets the maximum D2A value for the OmniMagnet object.
 * 
 * @param maxRate A lsampl_t representing the desired maximum D2A value.
 */
void OmniMagnet::setD2AMax(lsampl_t maxRate) {
	this->d2a_max = maxRate;
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
Eigen::Vector3d  OmniMagnet::Dipole2Current(Eigen::Vector3d dipole) {
	return decomp_.solve(frame_.transpose() * dipole) * (wire_width * wire_width);
};

/**
 * @brief Converts a current value to the corresponding D2A value for the OmniMagnet object.
 * 
 * This method takes a current value (double) as input and calculates the corresponding D2A value (lsampl_t).
 * The conversion is based on the maximum D2A value set for the OmniMagnet object.
 * 
 * @param current A double representing the desired current value in A.
 * 
 * @return A lsampl_t representing the corresponding D2A value.
 */
lsampl_t OmniMagnet::CurrentD2A(double current) {
	// Old calculation from original code, but it was not dynamic and had magic numbers. 
	// It was also not clear what the units were, so it was hard to understand what was going on.

    // // std::cout<<(16383.0/(30.0))*(current+15.0)<<"\n";
    // return (16383.0/(30.0))*(current+15.0);

	// Rewrote to a mapping function to allow for more dynamic scaling and to avoid magic numbers.
	// Range was interpreted from original code.
	return map_range<double, lsampl_t>(current, -15.0, 15.0, 0, this->d2a_max);
};

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