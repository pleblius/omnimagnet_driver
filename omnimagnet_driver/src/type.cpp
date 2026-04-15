#include "../include/omnimagnet_driver/type.hpp"



Eigen::MatrixXd AshkanPseudoinverse(Eigen::MatrixXd A, double singularMinToMaxRatio)
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV ); // computes the SVD
    Eigen::MatrixXd S_inv(svd.matrixV().cols(),svd.matrixU().rows());
    S_inv.setZero();
    for( int i = 0; i < std::min(A.rows(),A.cols()); i++)
    {
        double val = 0;
        if( svd.singularValues()[i] > svd.singularValues()[0]*singularMinToMaxRatio )// threashold singular values anything less than 1/1000 of the max is set to 0
            val = 1.0 / svd.singularValues()[i];
        S_inv(i,i) = val;
    }
    Eigen::MatrixXd answer;
    answer = svd.matrixV()*(S_inv*(svd.matrixU().transpose()));
    return answer;
};


Eigen::MatrixXd Pseudoinverse(Eigen::MatrixXd A)
{
    Eigen::MatrixXd B = A.completeOrthogonalDecomposition().pseudoInverse();
    return B;
};

double CurrentD2A(double current)   //Converts the current to D2A values:
{
    // std::cout<<(16383.0/(30.0))*(current+15.0)<<"\n";
    return (16383.0/(30.0))*(current+15.0);
};