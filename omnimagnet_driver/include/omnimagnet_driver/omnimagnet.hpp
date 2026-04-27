#ifndef OMNIMAGNET_H
#define OMNIMAGNET_H

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/SVD>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/QR> 
#include <comedilib.hpp>
#include <chrono>

/*****************************************************
OmniMagnet.hpp  (requires omnimagnet.cpp)  defines a class which
    takes in an Omnimagnet props: 
		OmniMagnet(double wire_guage, double wire_len_in, double wire_len_mid,	double wire_len_out, double core_size;);

	TODO: Error checking
	TODO: Eliminate magic numbers
	TODO: Add URDF parameterization for reference frame adjustments

Omnimangnet local frame: out:Z, mid:Y, in:X
Ver 1.0 by Ashkan July-2019
Update  by Tyler Wilcox April 2026
	Moved type.hpp functions into omnimagnet.hpp
*****************************************************/

/* Set of orthonormal vectors representing a rotation plane */
struct Basis {
	Eigen::Vector3d u;
	Eigen::Vector3d v;

	// Takes linearly independent vectors and creates an orthonormal set of bases
	Basis(const Eigen::Vector3d &u_, const Eigen::Vector3d &v_) {
		u = u_.normalized();

		v = (v_ - u * u.dot(v_)).normalized();
	}

	Basis() {}
};

class OmniMagnet {
	private:
		int subdev = 0;     /* change this to your input subdevice */
		int chan = 10;      /* change this to your channel */
		lsampl_t d2a_max = 16383;      /* more on this later */
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
		Eigen::Matrix3d mapping_;
		Eigen::Vector3d max_current;
		std::chrono::high_resolution_clock::time_point ref_time, current_time;
	public:
		int ID{0};
		double freq;
		double strength;
		double offset;
		double theta;
    	Eigen::Vector3d dipole;
		Eigen::Vector3d rotVector;
		Basis rotBasis;

		OmniMagnet();
		OmniMagnet(double wire_width, double wire_len_in, double wire_len_mid, double wire_len_out, double core_size, int pinin, int pinmid, int pinout,bool estimate, comedi_t * card);
		void SetProp(double wire_width, double wire_len_in, double wire_len_mid, double wire_len_out, double core_size, int pinin, int pinmid, int pinout,bool estimate, comedi_t *card);

		int SetCurrent(Eigen::Vector3d current_);
		void SetOrientation( double orientation);
		void UpdateMapping();
		double GetOrientation();
		void setD2AMax(lsampl_t);
		Eigen::Vector3d GetCurrent();
		Eigen::Matrix3d GetMapping();
		Eigen::Vector3d Dipole2Current(Eigen::Vector3d dipole_);
		lsampl_t CurrentD2A(double);
		double max_dipole_mag;
		void RotatingDipole(Eigen::Vector3d init_dipole, Eigen::Vector3d axis_rot, double freq, int dur);
		// [[maybe_unused]] static Eigen::MatrixXd AshkanPseudoinverse(Eigen::MatrixXd, double);
		// [[maybe_unused]] static Eigen::MatrixXd Pseudoinverse(Eigen::MatrixXd);

		template <typename inType, typename outType>
		static outType map_range(inType value, inType val_min, inType val_max, outType range_min, outType range_max) {
			if (value > val_max)
				return range_max;
			if (value < val_min)
				return range_min;

			return (value - val_min) * (range_max - range_min) / (val_max - val_min) + range_min;
		}
};
#endif // OMNIMAGNET_H