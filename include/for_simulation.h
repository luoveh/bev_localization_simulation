#ifndef FOR_SIMULATION_H_
#define FOR_SIMULATION_H_

#include "reuse.h"

using namespace std;

namespace bev_reloca{

    b_pose addPoseNoise(b_pose curr_gt_pose);
    void getBevRect(float bev_rect[4][3], float car_icon[4][3] );
    Eigen::Matrix4d getDeltaPose(vector<b_pose>& trjs, int pose_count);
    void drawCurrPose(b_pose curr_pose, cv::Mat& img,  float bev_rect[4][3], float car_icon[4][3]);


}//namespace bev_reloca


#endif //#ifndef FOR_SIMULATION_H_