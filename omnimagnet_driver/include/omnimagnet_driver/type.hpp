#ifndef TYPE_H
#define TYPE_H
/*****************************************************
type.h  (requires eigen library)  defines matrix types: 

    Inherits:
		eigen
Ver 1.0 by Ashkan July-2019		
*****************************************************/

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/SVD>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/QR> 
#include <comedilib.hpp>
#include <chrono>

Eigen::MatrixXd AshkanPseudoinverse(Eigen::MatrixXd A, double singularMinToMaxRatio);
Eigen::MatrixXd Pseudoinverse(Eigen::MatrixXd A);
double CurrentD2A(double current);
#endif //TYPE_H