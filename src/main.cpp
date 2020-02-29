#include <iostream>
#include <../../eigen-3.3.7/Eigen/Dense>
#include <math.h>

class HomoTran4d
{
public:
    HomoTran4d(double thtXDeg, double thtYDeg, double thtZDeg, Eigen::Vector3d T) 
    : m_thetaX{thtXDeg*3.1415926/180}, 
      m_thetaY{thtYDeg*3.1415926/180}, 
      m_thetaZ{thtZDeg*3.1415926/180},
      m_T{T}
    {
        double a = m_thetaX;        

        m_RX <<    1,       0,       0,
                   0,  cos(a), -sin(a),
                   0,  sin(a),  cos(a);

        a = m_thetaY;
        m_RY <<  cos(a),  0,  sin(a),
                      0,  1,       0,
                -sin(a),  0,  cos(a);             

        a = m_thetaZ;
        m_RZ << cos(a), -sin(a), 0,
                sin(a),  cos(a), 0,
                0,            0, 1;
        
        m_R33 = m_RX*m_RY*m_RZ;
        m_H.setIdentity();
        m_H.block<3,3>(0,0) = m_R33;
        m_H.block<3,1>(0,3) = m_T;    
    }
    ~HomoTran4d()
    {};

    double           m_thetaX;//Rotation in deg around Y axis
    double           m_thetaY;//Rotation in deg around Y axis
    double           m_thetaZ;//Rotation in deg around Z axis
    
    Eigen::Matrix3d m_RX;//3 by 3 rotation matrix
    Eigen::Matrix3d m_RY;//3 by 3 rotation matrix
    Eigen::Matrix3d m_RZ;//3 by 3 rotation matrix
    Eigen::Matrix3d m_R33;//3 by 3 rotation matrix
    Eigen::Vector3d m_T;
    Eigen::Matrix4d m_H;//4 by 4 homogenous matrix
};

using namespace Eigen;
// using Matrix4f = Matrix<float, 4, 4>;
int main()
{
    std::cout << "Begin Eigen playground" << std::endl;
    // MatrixXd m(2,2);
    // m(0,0) = 3;
    // m(1,0) = 2.5;
    // m(0,1) = -1;
    // m(1,1) = m(1,0) + m(0,1);
    // std::cout << m << std::endl;
    Eigen::Vector3d T = {5,0,10};
    HomoTran4d R1(0, 0, 45, T);
    std::cout << R1.m_thetaX << std::endl;
    std::cout << R1.m_RX << std::endl;
    std::cout << R1.m_RX.inverse() << std::endl;

    std::cout << R1.m_H << std::endl;
    std::cout << R1.m_H.inverse() << std::endl;
    
    return 0;
}
