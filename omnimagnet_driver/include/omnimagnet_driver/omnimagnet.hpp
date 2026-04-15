#ifndef OMNIMAGNET_H
#define OMNIMAGNET_H
/*****************************************************
OmniMagnet.h  (requires omnimagnet.cpp)  defines a class which
    takes in an Omnimagnet props: 
		OmniMagnet(double wire_guage, double wire_len_in, double wire_len_mid,	double wire_len_out, double core_size;);
	Inherits:
		type.hpp


Omnimangnet local frame: out:Z, mid:Y, in:X
Ver 1.0 by Ashkan July-2019		
*****************************************************/
#include "type.hpp" // defines the matrix types.

class OmniMagnet {
	private:
		int subdev = 0;     /* change this to your input subdevice */
		int chan = 10;      /* change this to your channel */
		int range = 16383;      /* more on this later */
		int aref = AREF_GROUND; /* more on this later */
		unsigned int subdevice = 0;
		int res;
		comedi_t* card_;
		bool estimate_;
		double wire_width;
		double wire_len_in;
		double wire_len_mid;
		double wire_len_out;
		double core_size;
		double orientation_;
		Eigen::Matrix3d axis_rot_Z;
		Eigen::Vector3d temperature_;
		Eigen::Vector3d power_;
		Eigen::Vector3d posision_;
		Eigen::Vector3d current_;
		Eigen::Vector3d current_density;
		Eigen::Vector3d D2A_pin_number;
		// Eigen::Matrix3f orientation_;
		Eigen::Matrix3d mapping_;     // local maping 
		Eigen::Vector3d max_current;
		// std::chrono::milliseconds ref_time;
		std::chrono::high_resolution_clock::time_point ref_time, current_time;
	public:
		OmniMagnet();
		OmniMagnet(double wire_width, double wire_len_in, double wire_len_mid, double wire_len_out, double core_size, int pinin, int pinmid, int pinout,bool estimate, comedi_t * card);
		void SetProp(double wire_width, double wire_len_in, double wire_len_mid, double wire_len_out, double core_size, int pinin, int pinmid, int pinout,bool estimate, comedi_t *card);
		void SetCurrent(Eigen::Vector3d current_);
		void SetTemperature(Eigen::Vector3d temperature_);
		void SetPower(Eigen::Vector3d power_);
		void SetPosision(Eigen::Vector3d posision_ );
		// void SetOrientation( Eigen::Matrix3f orientation_ );
		void SetOrientation( double orientation);
		void UpdateMapping();
		double GetOrientation();
		Eigen::Vector3d GetCurrent();
		Eigen::Vector3d GetTemperature();
		Eigen::Vector3d GetPower();
		Eigen::Vector3d GetPosision();
		Eigen::Vector3d GetDipole(Eigen::Vector3d target_loc);
		Eigen::Matrix3d GetMapping();
		Eigen::Vector3d Dipole2Current(Eigen::Vector3d dipole_);
		double max_dipole_mag;
		void RotatingDipole(Eigen::Vector3d init_dipole, Eigen::Vector3d axis_rot, double freq, int dur);
};
#endif // OMNIMAGNET_H