#ifndef BEV_FRAME_H_
#define BEV_FRAME_H_
#include <atomic>
#include "pcl_lib.h"
#include "reuse.h"

using namespace std;
namespace bev_reloca{

    class hd_map;

    class bev_frame
    {
        public:
            bev_frame(const hd_map& g_map, b_pose gt_pose);

            void getBevRect();
            bool isInBev(float x, float y);
            bool isInRect(float rect[4][3], float x, float y);
            std::vector<std::vector<cv::Point2f>> projectTwoPointLine2(Eigen::Vector3d point1, Eigen::Vector3d point2);

            void getPerception(const hd_map& g_map);
            void getSlotCorners(const hd_map& g_map);
            void getLanes(const hd_map& g_map);
            void getLongWhiteLines(const hd_map& g_map);
            void getArrows(const hd_map& g_map);
            void getZebras(const hd_map& g_map);
            void getSpeedbumps(const hd_map& g_map);

            double getMinDistanceToCam(Eigen::Vector3d point);
            Eigen::Vector3d getCordNoise(double error);
            void generatePointCloud();

            PointCloud::Ptr getCloud();

            void display(cv::Mat& p);
    

            b_pose gt_pose_;
            Eigen::Matrix3d gt_R_;
            Eigen::Vector3d gt_t_;
            vector<hd_point> slot_corners_;
            vector<hd_line> lanes_;
            vector<hd_line> long_white_lines_;
            vector<hd_line> arrows_;
            vector<hd_line> zebras_;
            vector<hd_line> speedbumps_;

            unsigned int id_;
        
        private:

            static std::atomic<unsigned int> next_id_;

            float bev_rect_[4][3];
            float car_icon_[4][3];

            PointCloud::Ptr percep_cloud_;

    };

}//namespace bev_reloca



#endif //#ifndef BEV_FRAME_H_