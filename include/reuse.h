#ifndef REUSE_H_
#define REUSE_H_
#include <vector>
#include <sys/time.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

namespace bev_reloca{

    #define ToPixel(x) x*8+70
    
    #define xPercepToPixel(x) x*15+280
    #define yPercepToPixel(x) -x*15+280

    //perception element type
    enum ELE_TYPE
    {
        SLOT_CORNER = 0,
        LANE,
        LONG_WHITE_LINE,
        ARROW,
        ZEBRA,
        SPEEDBUMP
    };

    //line element，two end points and label
    typedef struct _hd_point{
        Eigen::Vector3d point;
        int label;
    } hd_point;

    //line element，two end points and label
    typedef struct _hd_line{
        Eigen::Vector3d point1;
        Eigen::Vector3d point2;
        int label;
    } hd_line;

    //veh pose, car center and eular angles
    typedef struct _pose{
        Eigen::Vector3d center;
        Eigen::Vector3d eular;
    } b_pose;

}//namespace bev_reloca

#endif //#ifndef REUSE_H_