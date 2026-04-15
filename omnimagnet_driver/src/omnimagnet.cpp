#include <iostream>
#include "../include/omnimagnet_driver/omnimagnet.hpp"
#include "comedilib.hpp"

// No return type for constructor
OmniMagnet::OmniMagnet(double wirewidth, double wirelenin, double wirelenmid,	double wirelenout, double coresize, int pinin, int pinmid, int pinout,bool estimate, comedi_t *card)
{
	// wire_guage = wireguage;
	// wire_len_in = wirelenin;
	// wire_len_mid = wirelenmid;
	// wire_len_out = wirelenout;
	// core_size = coresize;
	// current_ << 0.0, 0.0 ,0.0;
 	SetProp(wirewidth, wirelenin, wirelenmid, wirelenout, coresize, pinin, pinout, pinout, estimate, card);

};

OmniMagnet::OmniMagnet(){
};

void OmniMagnet::SetProp(double wirewidth, double wirelenin, double wirelenmid,	double wirelenout, double coresize, int pinin, int pinmid, int pinout, bool estimate, comedi_t *card) 
{
	card_ = card;
	wire_width = wirewidth;
	wire_len_in = wirelenin;
	wire_len_mid = wirelenmid;
	wire_len_out = wirelenout;
	core_size = coresize;
	orientation_ = 0.0;
	current_ << 0.0, 0.0 ,0.0;
	mapping_ << 0.0, 0.0 ,0.0,
				0.0, 0.0 ,0.0,
				0.0, 0.0 ,0.0;
	D2A_pin_number << pinin, pinmid, pinout;
	estimate_ = estimate;
	SetCurrent(current_);
};

void OmniMagnet::UpdateMapping() /* Generates the mapping (3X3) matrix*/ 
{
	if (estimate_) {
		// std::cout<<"here";

		// Generates mapping of dipole (Am^2) to current density (A/m^2)
		// Equation (13) from Omnimagnet Paper
		float mapping_constant = 51.45 *2 * 0.825; 	// Tuned value to map current to desired dipole strength 
		mapping_ = Eigen::MatrixXd::Identity(3,3)*((mapping_constant * pow (10.0, -3.0) * pow(0.115,4)));
		// mapping_ = Eigen::MatrixXd::Identity(3,3)*((51.45 * pow (10.0, -3.0) * (0.115))/(wire_width*wire_width));
		axis_rot_Z = (Eigen::AngleAxisd(orientation_*M_PI/180.0, Eigen::Vector3d::UnitZ()));
		// std::cout<<"Res_rot:    \n"<<axis_rot_Z * mapping_<<std::endl;
		axis_rot_Z.normalize();
		// std::cout<<"Res_rot:    \n"<<axis_rot_Z * mapping_<<std::endl;
		mapping_ = (axis_rot_Z * mapping_);
		
		//Eigen::Vector3d max_dipole = mapping_ *  
		//std::cout<<mapping_<<std::endl;
	}
	else{
		std::cout<<"No method, use the estimate method!!!!!";
	}
};


Eigen::Matrix3d OmniMagnet::GetMapping()
{
	return mapping_;	
};

void OmniMagnet::SetCurrent(Eigen::Vector3d current)
{
    current_ = current;

    lsampl_t d0 = CurrentD2A(current_[0]);
    lsampl_t d1 = CurrentD2A(current_[1]);
    lsampl_t d2 = CurrentD2A(current_[2]);

    std::cout << "currents: " << current_[0] << ", " << current_[1] << ", " << current_[2] << std::endl;
    std::cout << "dac vals: " << d0 << ", " << d1 << ", " << d2 << std::endl;
    std::cout << "channels: " << D2A_pin_number[0] << ", " << D2A_pin_number[1] << ", " << D2A_pin_number[2] << std::endl;

    int retval;
    retval = comedi_data_write(card_, subdev, D2A_pin_number[0], 0, AREF_GROUND, d0);
    std::cout << "ret0 = " << retval << std::endl;

    retval = comedi_data_write(card_, subdev, D2A_pin_number[1], 0, AREF_GROUND, d1);
    std::cout << "ret1 = " << retval << std::endl;

    retval = comedi_data_write(card_, subdev, D2A_pin_number[2], 0, AREF_GROUND, d2);
    std::cout << "ret2 = " << retval << std::endl;
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


Eigen::Vector3d  OmniMagnet::Dipole2Current(Eigen::Vector3d dipole) /* 
For a given dipole (3X1), calculates the needed current (3X1). The mapping matrix needs to get updated the Omni moves*/
{
	Eigen::Vector3d result;
	result = mapping_.completeOrthogonalDecomposition().solve(dipole);	// calculates the needed current density for desired dipole 
	result = result*((wire_width*wire_width));							// calculates current needed
	// std::cout<< "The requested dipole:     \n" << dipole << std::endl<< "The generated dipole:     \n"<<  mapping_ * result<< std::endl;	
	// std::cout <<"result :     \n"<< result << std::endl;
	return result;
};


Eigen::Vector3d OmniMagnet::GetPosision(){};
Eigen::Vector3d OmniMagnet::GetTemperature(){};
Eigen::Vector3d OmniMagnet::GetPower(){};
void OmniMagnet::SetTemperature(Eigen::Vector3d temperature_){};
void OmniMagnet::SetPower(Eigen::Vector3d power_){};
void OmniMagnet::SetPosision( Eigen::Vector3d posision_ ){};
// void OmniMagnet::SetOrientation( Eigen::Matrix3f orientation_ ){};
Eigen::Vector3d OmniMagnet::GetDipole(Eigen::Vector3d target_loc){};





void OmniMagnet::RotatingDipole(Eigen::Vector3d init_dipole, Eigen::Vector3d axis_rot, double freq_, int dur)
{
	std::chrono::duration<double> test_duration = std::chrono::milliseconds(dur);
	current_time = std::chrono::high_resolution_clock::now();
	ref_time = current_time; 
	std::cout<<"\nStart_rot!!!!!\n";
	while ( current_time - ref_time <test_duration){
		current_time = std::chrono::high_resolution_clock::now();
		// std::cout<<"------\n";
		//std::cout<< ((Eigen::AngleAxisd(( 2.0 * M_PI * ((current_time - ref_time).count()/1000000000.0) * freq_), axis_rot)) * init_dipole) <<"\n";
		SetCurrent(Dipole2Current((Eigen::AngleAxisd(( 2.0 * M_PI * ((current_time - ref_time).count()/1000000000.0) * freq_), axis_rot)) * init_dipole));
	};
	std::cout<<"End rot!!!!!\n";
	// std::cout<<"here";
};
