#ifndef OMNIMAGNET_H
#define OMNIMAGNET_H

#include <eigen3/Eigen/Dense>
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
Updateed by Tyler Wilcox August 2026
	Moved type.hpp functions into omnimagnet.hpp
	Rewrote most functionality to be more modular
	Eliminated unused functions and class members
	Added more robust error checking and handling
	Replaced magic numbers with principled functionality
*****************************************************/

/**
 * @brief Struct to hold magnet configuration parameters
 */
struct MagnetConfig
{
    int id;
    bool enabled;

    double wireWidth;
    double innerWireLength;
    double midWireLength;
    double outerWireLength;
    double coreSize;

    int innerChannel;
    int midChannel;
    int outerChannel;

    bool estimate;
    std::vector<double> frame;
};

/**
 * @brief Software representation of an omnimagnet.
 */
class OmniMagnet {
	private:
		int id_;

		double wireWidth_;
		double innerWireLength_;
		double midWireLength_;
		double outerWireLength_;
		double coreSize_;

		bool estimate_;

		std::vector<int> cardPinNumbers_;

		Eigen::Vector3d current_;
		Eigen::Matrix3d mapping_;
		Eigen::Matrix3d frame_;

		Eigen::CompleteOrthogonalDecomposition<Eigen::Matrix3d> decomp_;

	public:
		OmniMagnet();
		OmniMagnet(const MagnetConfig&);

		void setProp(const MagnetConfig&);

		void setID(int);
		int ID() const;

		void setPinNumbers(const std::vector<int>&);
		const std::vector<int>& pinNumbers() const;
		
		void setFrame(const Eigen::Matrix3d);
		void setFrame(const std::vector<double>&);
		const Eigen::Matrix3d& frame() const;

		void setCurrent(Eigen::Vector3d);
		const Eigen::Vector3d& current() const;
		
		void updateMapping();
		const Eigen::Matrix3d& mapping() const;
		
		Eigen::Vector3d dipoleToCurrent(Eigen::Vector3d dipole) const;

		// [[maybe_unused]] static Eigen::MatrixXd AshkanPseudoinverse(Eigen::MatrixXd, double);
		// [[maybe_unused]] static Eigen::MatrixXd Pseudoinverse(Eigen::MatrixXd);
};
#endif // OMNIMAGNET_H