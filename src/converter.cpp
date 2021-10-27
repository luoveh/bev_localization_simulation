#include "converter.h"
#include <iostream>
namespace bev_reloca{

    void rotToAngle(const Eigen::Matrix3d R, double angles[3])
    {
        angles[0] = atan2(R(2,1), R(2,2));

        angles[1] = atan2(-R(2,0), sqrt(R(2,1)*R(2,1)+R(2,2)*R(2,2)));
        angles[2] = atan2(R(1,0), R(0,0));
    }

    void angleToRot(Eigen::Matrix3d& R, const double angles[3])
    {
        double cx = cos(angles[0]), cy = cos(angles[1]), cz = cos(angles[2]);
        double sx = sin(angles[0]), sy = sin(angles[1]), sz = sin(angles[2]);
        
        R(0,0) = cy*cz;	R(0,1) = -cx*sz + sx*sy*cz;	R(0,2) = sx*sz + cx*sy*cz;
        R(1,0) = cy*sz;	R(1,1) = cx*cz + sx*sy*sz;	R(1,2) = -sx*cz + cx*sy*sz;
        R(2,0) = -sy;		R(2,1) = sx*cy;			R(2,2) = cx*cy;
    }

    void rot_to_angle( const double R[9],double angles[3])
    {
        angles[0] = atan2(R[7],R[8]);
        angles[1] = atan2(-R[6],sqrt(R[7]*R[7] + R[8]*R[8]));
        angles[2] = atan2(R[3],R[0]);
    }

    void angle_to_rot(double R[9], const double angles[3])
    {
        double cx = cos(angles[0]), cy = cos(angles[1]), cz = cos(angles[2]);
        double sx = sin(angles[0]), sy = sin(angles[1]), sz = sin(angles[2]);
        R[0] = cy*cz;	R[1] = -cx*sz + sx*sy*cz;	R[2] = sx*sz + cx*sy*cz;
        R[3] = cy*sz;	R[4] = cx*cz + sx*sy*sz;	R[5] = -sx*cz + cx*sy*sz;
        R[6] = -sy;		R[7] = sx*cy;			R[8] = cx*cy;
    }

}//namespace bev_reloca