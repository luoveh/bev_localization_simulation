#ifndef BEV_RELOCALIZER_H_
#define BEV_RELOCALIZER_H_

#include "reuse.h"
#include "pcl_lib.h"

namespace bev_reloca{

    class local_perception;
    class hd_map;
    class bev_frame;

    class bev_relocalizer
    {
        public:
            bev_relocalizer();
            

            /*******************************************************************************
            | Function name: process
            | param:
            |				param1: IN, const hd_map&, 
            |				param2: IN/OUT type, data type, 
            |				param3: IN/OUT type, data type,
            |               param3: IN/OUT type, data type,
            | return value: value type, value definition
            ******************************************************************************/
            int process(const hd_map& g_map, b_pose init_pose, b_pose gt_pose, Eigen::Matrix4d delta_pose, cv::Mat& p, cv::Mat& result);

            void getLocalMap(const hd_map& g_map, b_pose init_pose);
            void bevPoseOptimize(b_pose init_pose, b_pose gt_pose);

            b_pose getCurrPose();
            b_pose getRelocaPose();

            void displayLocalMapInVehCord(cv::Mat& p, b_pose init_pose);
            void displayRelocaResult(cv::Mat& p);

        private:
            float pos_error_ = 5;
            hd_map *local_map_ = nullptr;

            local_perception * local_perceptions_ = nullptr;
            b_pose pose_;

            Eigen::Matrix4f delta_T_ = Eigen::Matrix4f::Identity ();  //优化得到的初始位姿与真实位姿的相对变换
            Eigen::Matrix3d reloca_R_;                                                        //重定位得到的最终位姿
            Eigen::Vector3d reloca_t_;

            PointCloud::Ptr local_cloud_;
            PointCloud::Ptr local_map_cloud_;

            ofstream loca_result_;

        
    };

}//namespace bev_reloca


#endif  //#ifndef BEV_RELOCALIZER_H_