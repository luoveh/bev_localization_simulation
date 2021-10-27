#include <iostream>
#include "hd_map.h"
#include "bev_relocalizer.h"
#include "converter.h"
#include "for_simulation.h"

using namespace std;
using namespace bev_reloca;

float bev_rect[4][3];
float car_icon[4][3];

int main()
{
    int pose_count = 0;
    b_pose init_pose;
    Eigen::Matrix4d delta_pose;
    cv::Mat map = cv::Mat(1000, 2200, CV_32FC3, cv::Scalar(255, 255, 255));

    //create bev HD map
    hd_map g_map;
    g_map.getMap();
    g_map.display(map);
    cv::imwrite("map.jpg", map);

    //create bev relocalizer
    bev_relocalizer relocalizer;

    getBevRect(bev_rect, car_icon);
    while(true)
    {
        if(pose_count >= g_map.trjs_.size()) 
        {
            break;
        }

        cv::Mat vehicle_bev = map.clone();
        cv::Mat perception = cv::Mat(600, 600, CV_32FC3, cv::Scalar(255, 255, 255));
        cv::Mat result = cv::Mat(600, 600, CV_32FC3, cv::Scalar(255, 255, 255));

        b_pose curr_gt_pose = g_map.trjs_[pose_count];  
        if(0 == pose_count)
        {
            init_pose = addPoseNoise(curr_gt_pose);   //init_pose is gt_pose added a noise
        }
        else
        {
            init_pose = relocalizer.getRelocaPose();   //init pose is last frame's pose
        }
        
        delta_pose = getDeltaPose(g_map.trjs_, pose_count);             //delta pose between last and current frame, given by DR 
        drawCurrPose(init_pose, vehicle_bev, bev_rect, car_icon);   
        
        relocalizer.process(g_map, init_pose,  curr_gt_pose, delta_pose, perception, result);

        pose_count++;

        cv::imshow("vehicle", vehicle_bev);
        cv::imshow("perception", perception);
        cv::imshow("result", result);
        cv::waitKey();
        
    } 

    return 0;
}


