#ifndef LOCAL_PERCEPTION_H_
#define LOCAL_PERCEPTION_H_

#include <queue>
#include "reuse.h"
#include "pcl_lib.h"
using namespace std;

namespace bev_reloca{

    class bev_frame;

    class local_perception
    {
        public:
            local_perception();


            /*******************************************************************************
            | Function name: updateCloud
            | Function discreption: function receive current frame(curr_frame) and relative 
            |       pose between last frame and current frame. first transform last 4 frames to 
            |       current frame, then use PCL outlier remove method to remove  error classes.
            | param:
            |				param1: IN, relative pose between current frame and last frame
            |				param2: IN, current frame 
            | return value: 
            ******************************************************************************/
            int updateCloud(Eigen::Matrix4d delta_pose, bev_frame*  curr_frame);


            /*******************************************************************************
            | Function name: updatePerception
            | Function discreption: function receive current frame(curr_frame) and relative 
            |       pose between last frame and current frame. first transform last 4 frames to 
            |       current frame, then .
            | param:
            |				param1: IN, relative pose between current frame and last frame
            |				param2: IN, current frame 
            | return value: 
            ******************************************************************************/
            int updatePerception(Eigen::Matrix4d delta_pose, bev_frame*  curr_frame);

            void getNewPerception(Eigen::Matrix4d delta_pose, bev_frame* frame);
            double calcLineDistance(hd_line line1, hd_line line2);
            void fusePerceptions();
            void removeInvalidPerceptions();
            void generateCloud();

            PointCloud::Ptr getLocalFusedCloud();

            double getMinDistanceToCam(Eigen::Vector3d point);
            Eigen::Vector3d getCordNoise(double error);


        private:
            queue<bev_frame * > frames_;
            queue<Eigen::Matrix4d> relative_poses_;

            vector<hd_point> slot_corners_;
            vector<hd_line> lanes_;
            vector<hd_line> long_white_lines_;
            vector<hd_line> arrows_;
            vector<hd_line> zebras_;
            vector<hd_line> speedbumps_;

            vector<int> invalid_slot_corners_;
            vector<int> invalid_lanes_;
            vector<int> invalid_long_white_lines_;
            vector<int> invalid_arrows_;
            vector<int> invalid_zebras_;
            vector<int> invalid_speedbumps_;

            PointCloud::Ptr local_cloud_;
            PointCloud::Ptr local_cloud_fused_;

            double radius_;
            int min_neighbors_;

            bool first_frame_ = true;

    };



}//namespace bev_reloca




#endif //#ifndef LOCAL_PERCEPTION_H_