#include <iostream>
#include "../include/omnimagnet_driver/omnimagnet.hpp"
#include "comedilib.hpp"

namespace {
	template <typename inType, typename outType>
	outType map_range(inType value, inType val_min, inType val_max, outType range_min, outType range_max) {
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
}

// No return type for constructor
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
)
{
	OmniMagnet();
 	SetProp(wirewidth, wirelenin, wirelenmid, wirelenout, coresize, pinin, pinmid, pinout, estimate, card);
};


OmniMagnet::OmniMagnet(){
	this->current_ << 	0.0, 0.0 ,0.0;

	this->mapping_ << 	0.0, 0.0 ,0.0,
						0.0, 0.0 ,0.0,
						0.0, 0.0 ,0.0;
						
	frame_ = Eigen::Matrix3d::Identity();
};


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

void OmniMagnet::SetFrame(const Eigen::Matrix3d frame) {
	frame_ = frame;
}

void OmniMagnet::SetFrame(const std::vector<double>& list) {
	if (list.size() != 9) {
		std::cerr << "Cannot set omnimagnet frame, requires list of 9 doubles." << std::endl;
		return;
	}

	frame_ = Eigen::Map<const Eigen::Matrix3d, Eigen::RowMajor>(list.data());
}

Eigen::Matrix3d OmniMagnet::GetFrame() {
	return frame_;
}


void OmniMagnet::UpdateMapping() /* Generates the mapping (3X3) matrix*/ 
{
	if (estimate_) {
		// Generates mapping of dipole (Am^2) to current density (A/m^2)
		// Equation (13) from Omnimagnet Paper
		// float mapping_constant = 51.45 * 2 * 0.825; 	// Tuned value to map current to desired dipole strength 
		
		constexpr float mapping_constant = .05145 * 2 * .825 * .115 * .115 * .115 * .115;

		// mapping_ = Eigen::MatrixXd::Identity(3,3)*((mapping_constant * pow (10.0, -3.0) * pow(0.115,4)));
		mapping_ = Eigen::MatrixXd::Identity(3,3) * mapping_constant;

		// mapping_ = Eigen::MatrixXd::Identity(3,3)*((51.45 * pow (10.0, -3.0) * (0.115))/(wire_width*wire_width));

		axis_rot_Z = (Eigen::AngleAxisd(orientation_*M_PI/180.0, Eigen::Vector3d::UnitZ()));
		axis_rot_Z.normalize();

		mapping_ = (axis_rot_Z * mapping_);
		decomp_  = mapping_.completeOrthogonalDecomposition();
	}
	else {
		std::cout<<"No method, use the estimate method";
	}
};


Eigen::Matrix3d OmniMagnet::GetMapping()
{
	return mapping_;	
};


int OmniMagnet::SetCurrent(Eigen::Vector3d current)
{
    this->current_ = current;

    lsampl_t d0 = CurrentD2A(current_[0]);
    lsampl_t d1 = CurrentD2A(current_[1]);
    lsampl_t d2 = CurrentD2A(current_[2]);

    int retval;
    retval = comedi_data_write(card_, subdev, D2A_pin_number[0], 0, AREF_GROUND, d0);
    if (retval < 0)
		return retval;

    retval = comedi_data_write(card_, subdev, D2A_pin_number[1], 0, AREF_GROUND, d1);
    if (retval < 0) {
		comedi_data_write(card_, subdev, D2A_pin_number[0], 0, AREF_GROUND, CurrentD2A(0));
		return retval;
	}

    retval = comedi_data_write(card_, subdev, D2A_pin_number[2], 0, AREF_GROUND, d2);
    if (retval < 0) {
		comedi_data_write(card_, subdev, D2A_pin_number[0], 0, AREF_GROUND, CurrentD2A(0));
		comedi_data_write(card_, subdev, D2A_pin_number[1], 0, AREF_GROUND, CurrentD2A(0));
		return retval;
	}

	return 1;
}


Eigen::Vector3d OmniMagnet::GetCurrent()
{
	return current_;
};


double OmniMagnet::GetOrientation()
{
	return orientation_;	
};


void OmniMagnet::SetOrientation( double orientation)
{
	orientation_ = orientation;
	UpdateMapping();
};


void OmniMagnet::setD2AMax(lsampl_t maxRate) {
	this->d2a_max = maxRate;
}


Eigen::Vector3d  OmniMagnet::Dipole2Current(Eigen::Vector3d dipole) /* 
For a given dipole (3X1), calculates the needed current (3X1). The mapping matrix needs to get updated the Omni moves*/
{
	Eigen::Vector3d result;
	result = decomp_.solve(frame_.transpose() * dipole);	// calculates the needed current density for desired dipole 
	result = result*((wire_width*wire_width));							// calculates current needed
	return result;
};

/**
 * @brief Maps current values to D2A signal rates
 * 
 * TODO:
 * 
 */
lsampl_t OmniMagnet::CurrentD2A(double current)   //Converts the current to D2A values:
{
    // // std::cout<<(16383.0/(30.0))*(current+15.0)<<"\n";
    // return (16383.0/(30.0))*(current+15.0);

	// Rewrote to be more intuitive
	return map_range<double, lsampl_t>(current, -15.0, 15.0, 0, this->d2a_max);
};

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