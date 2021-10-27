#include "for_simulation.h"
#include "converter.h"
#include "bev_reloca_config.h"

namespace bev_reloca{

b_pose addPoseNoise(b_pose curr_gt_pose)
{
    b_pose pose_with_noise;
    //位置误差3m
    pose_with_noise.center(0,0) = curr_gt_pose.center(0,0) + 3*(rand()/double(RAND_MAX));
    pose_with_noise.center(1,0) = curr_gt_pose.center(1,0) + 3*(rand()/double(RAND_MAX));
    pose_with_noise.center(2,0) = 0;

    //eular Z value误差3度
    pose_with_noise.eular(0,0) = curr_gt_pose.eular(0,0);
    pose_with_noise.eular(1,0) = curr_gt_pose.eular(1,0);
    float theta = curr_gt_pose.eular(2,0)*180/PI;
    theta = theta+ 3*(rand()/double(RAND_MAX));
    theta = theta*PI/180;
    pose_with_noise.eular(2,0) = theta;

    return pose_with_noise;

}

Eigen::Matrix4d getDeltaPose(vector<b_pose>& trjs, int pose_count)
{
    Eigen::Matrix4d delta_pose = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d last_R, curr_R;
    Eigen::Matrix4d last_T, curr_T;

    if(pose_count == 0)
        return delta_pose;
    
    b_pose last_p = trjs[pose_count-1];
    b_pose curr_p = trjs[pose_count];

    //std::cout<<"the "<<pose_count<<" frame-----------"<<std::endl;
    //std::cout<<"last_p = ("<<last_p.center(0,0)<<", "<<last_p.center(1,0)<<", "<<last_p.center(2,0)<<") ("<<last_p.eular(0,0)<<", "<<last_p.eular(1,0)<<", "<<last_p.eular(2,0)<<")"<<std::endl;
    //std::cout<<"curr_p = ("<<curr_p.center(0,0)<<", "<<curr_p.center(1,0)<<", "<<curr_p.center(2,0)<<") ("<<curr_p.eular(0,0)<<", "<<curr_p.eular(1,0)<<", "<<curr_p.eular(2,0)<<")"<<std::endl;

    double last_angles[3] = {last_p.eular(0,0), last_p.eular(1,0), last_p.eular(2,0)};
    double curr_angles[3] = {curr_p.eular(0,0), curr_p.eular(1,0), curr_p.eular(2,0)};

    angleToRot(last_R, last_angles);
    angleToRot(curr_R, curr_angles);

    last_T << last_R(0,0), last_R(0,1), last_R(0,2), last_p.center(0,0),
                        last_R(1,0), last_R(1,1), last_R(1,2), last_p.center(1,0),
                        last_R(2,0), last_R(2,1), last_R(2,2), last_p.center(2,0),
                        0, 0, 0, 1;
    
    curr_T << curr_R(0,0), curr_R(0,1), curr_R(0,2), curr_p.center(0,0),
                        curr_R(1,0), curr_R(1,1), curr_R(1,2), curr_p.center(1,0),
                        curr_R(2,0), curr_R(2,1), curr_R(2,2), curr_p.center(2,0),
                        0, 0, 0, 1;

    delta_pose = curr_T.inverse()*last_T;

    //std::cout<<"delta_pose = "<<std::endl;
    //std::cout<<delta_pose<<std::endl;

    return delta_pose;    

}

void drawCurrPose(b_pose curr_pose, cv::Mat& img, float bev_rect[4][3], float car_icon[4][3] )
{
    Eigen::Matrix3d R;
    Eigen::Vector3d t(curr_pose.center(0,0), curr_pose.center(1,0), curr_pose.center(2,0));
    double angles[3] = {curr_pose.eular(0,0), curr_pose.eular(1,0), curr_pose.eular(2,0)};
    angleToRot(R, angles);

   /*  std::cout<<"当前位姿 ："<<std::endl;
    std::cout<<"坐标：("<<curr_pose.center(0,0)<<", "<<curr_pose.center(1,0)<<")"<<std::endl;
    std::cout<<"欧拉角：("<<curr_pose.eular(0,0)<<", "<<curr_pose.eular(1,0)<<", "<<curr_pose.eular(2,0)<<")"<<std::endl; */

    for(int i = 0; i<4; i++)
    {
        int j = (i+1)%4;

        Eigen::Vector3d bev_vertex1(bev_rect[i][0], bev_rect[i][1], bev_rect[i][2]);
        Eigen::Vector3d bev_vertex2(bev_rect[j][0], bev_rect[j][1], bev_rect[j][2]);
        

        Eigen::Vector3d icon_vertex1(car_icon[i][0], car_icon[i][1], car_icon[i][2]);
        Eigen::Vector3d icon_vertex2(car_icon[j][0], car_icon[j][1], car_icon[j][2]);

        Eigen::Vector3d c_bev_vertex1 = R*bev_vertex1 + t;
        Eigen::Vector3d c_bev_vertex2 = R*bev_vertex2 + t;
        
        Eigen::Vector3d c_icon_vertex1 = R*icon_vertex1 + t;
        Eigen::Vector3d c_icon_vertex2 = R*icon_vertex2 + t;

        cv::Point2d c_bev_p1(ToPixel(c_bev_vertex1(0,0)), ToPixel(-c_bev_vertex1(1,0)));
        cv::Point2d c_bev_p2(ToPixel(c_bev_vertex2(0,0)), ToPixel(-c_bev_vertex2(1,0)));

        cv::Point2d c_icon_p1(ToPixel(c_icon_vertex1(0,0)), ToPixel(-c_icon_vertex1(1,0)));
        cv::Point2d c_icon_p2(ToPixel(c_icon_vertex2(0,0)), ToPixel(-c_icon_vertex2(1,0)));

        //std::cout<<"c_bev_p1 = ("<<c_bev_p1.x<<", "<<c_bev_p1.y<<")"<<std::endl;

        cv::line(img, c_bev_p1, c_bev_p2, cv::Scalar(0,0,0),1 );
        if(i == 0)
            cv::line(img, c_icon_p1, c_icon_p2, cv::Scalar(255,0,0),2 );
        else 
            cv::line(img, c_icon_p1, c_icon_p2, cv::Scalar(0,0,0),1 );
    }
}


void getBevRect(float bev_rect[4][3], float car_icon[4][3] )
{
    float bev_vertex_x1 = BEV_RANGE_LENGTH/2 + CAR_LENGTH/2 - CAR_REAR_TO_CENTER;
    float bev_vertex_x2 = bev_vertex_x1 - BEV_RANGE_LENGTH;
    float bev_vetex_y = BEV_RANGE_WIDTH/2;

    bev_rect[0][0] = bev_vertex_x1;  bev_rect[0][1] = -bev_vetex_y;  bev_rect[0][2] = 0;
    bev_rect[1][0] = bev_vertex_x1;  bev_rect[1][1] =   bev_vetex_y;  bev_rect[1][2] = 0;
    bev_rect[2][0] = bev_vertex_x2;  bev_rect[2][1] =   bev_vetex_y;  bev_rect[2][2] = 0;
    bev_rect[3][0] = bev_vertex_x2;  bev_rect[3][1] = -bev_vetex_y;  bev_rect[3][2] = 0;

    float car_icon_x1 =  CAR_LENGTH - CAR_REAR_TO_CENTER;
    float car_icon_x2 =  -CAR_REAR_TO_CENTER;
    float car_icon_y = CAR_WIDTH/2;

    car_icon[0][0] = car_icon_x1;  car_icon[0][1] = -car_icon_y;  car_icon[0][2] = 0;
    car_icon[1][0] = car_icon_x1;  car_icon[1][1] =   car_icon_y;  car_icon[1][2] = 0;
    car_icon[2][0] = car_icon_x2;  car_icon[2][1] =   car_icon_y;  car_icon[2][2] = 0;
    car_icon[3][0] = car_icon_x2;  car_icon[3][1] = -car_icon_y;  car_icon[3][2] = 0;

}

} //namespace bev_reloca