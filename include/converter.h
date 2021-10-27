#ifndef CONVERTER_H_
#define CONVERTER_H_

#include <cmath>
#include <Eigen/Core>
#include <Eigen/Dense>

namespace bev_reloca{


    void rotToAngle(const Eigen::Matrix3d R, double angles[3]);
    
    void angleToRot(Eigen::Matrix3d& R, const double angles[3]);

    void rot_to_angle( const double R[9],double angles[3]);
    
    void angle_to_rot(double R[9], const double angles[3]);
    

}//namespace bev_loca



#endif //#ifndef CONVERTER_H_