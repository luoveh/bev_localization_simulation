#include "bev_relocalizer.h"
#include "local_perception.h"
#include "bev_frame.h"
#include "hd_map.h"
#include "converter.h"
#include "bev_reloca_config.h"

namespace bev_reloca{

    bev_relocalizer::bev_relocalizer():local_cloud_(new PointCloud), local_map_cloud_(new PointCloud)
    {
        local_map_ = new hd_map();
        local_perceptions_ = new local_perception();

        loca_result_.open("traj.csv", ios::in|ios::out|ios::binary|ios::trunc);    
    }


    int bev_relocalizer::process(const hd_map& g_map, b_pose init_pose, b_pose gt_pose, Eigen::Matrix4d delta_pose, cv::Mat& p, cv::Mat& result)
    {
        
        bev_frame*  curr_frame = new bev_frame(g_map, gt_pose);
        curr_frame->display(p);
        curr_frame->display(result);

        getLocalMap(g_map, init_pose);
        displayLocalMapInVehCord(p, init_pose);

        //local_perceptions_->updateCloud(delta_pose, curr_frame);
        local_perceptions_->updatePerception(delta_pose, curr_frame);

        local_cloud_ = local_perceptions_->getLocalFusedCloud();     //已经在车辆坐标系下了

        //std::cout<<"-------------------"<<curr_frame->id_<<"---------------------"<<std::endl;
        bevPoseOptimize(init_pose, gt_pose);
        displayRelocaResult(result);

        loca_result_ << gt_pose.center(0,0) <<", "<< gt_pose.center(1,0)<<", "<<reloca_t_(0, 0)<<", "<<reloca_t_(1, 0)<<std::endl;


        b_pose reloc_pose = getRelocaPose();
        
        /* std::cout<<"gt_pose = ("<<gt_pose.eular(0,0)<<", "<<gt_pose.eular(1,0)<<", "<<gt_pose.eular(2,0)<<") ("<<gt_pose.center(0,0)<<", "<<gt_pose.center(1,0)<<")"<<std::endl;
        std::cout<<"init pose = ("<<init_pose.eular(0,0)<<", "<<init_pose.eular(1,0)<<", "<<init_pose.eular(2,0)<<") ("<<init_pose.center(0,0)<<", "<<init_pose.center(1,0)<<")"<<std::endl;
        std::cout<<"reloc_pose = ("<<reloc_pose.eular(0,0)<<", "<<reloc_pose.eular(1,0)<<", "<<reloc_pose.eular(2,0)<<") ("<<reloc_pose.center(0,0)<<", "<<reloc_pose.center(1,0)<<")"<<std::endl;
        std::cout<<"reloca_R :"<<std::endl;
        std::cout<<reloca_R_<<std::endl; */


        return 0;
    }


    void bev_relocalizer::bevPoseOptimize(b_pose init_pose, b_pose gt_pose)
    {
        Eigen::Matrix3d R_init, R_;
        Eigen::Matrix4f T_noise = Eigen::Matrix4f::Identity();
        Eigen::Matrix4f T_gt = Eigen::Matrix4f::Identity();

        PointCloud::Ptr perception_cloud(new PointCloud);
        PointCloud::Ptr local_map_cloud(new PointCloud);
        
        double init_angles[3] = {init_pose.eular(0,0), init_pose.eular(1,0), init_pose.eular(2,0)};
        double gt_angles[3] = {gt_pose.eular(0,0), gt_pose.eular(1,0), gt_pose.eular(2,0)};
        angleToRot(R_init, init_angles);
        angleToRot(R_, gt_angles);

        T_noise << R_init(0,0), R_init(0,1), R_init(0,2),  init_pose.center(0,0),
                               R_init(1,0), R_init(1,1), R_init(1,2),  init_pose.center(1,0),
                               R_init(2,0), R_init(2,1), R_init(2,2),  init_pose.center(2,0),
                               0, 0, 0, 1;
       /* T_gt<<R_(0, 0), R_(0, 1), R_(0, 2), gt_pose.center(0, 0),
                      R_(1, 0), R_(1, 1), R_(1, 2), gt_pose.center(1, 0),
                      R_(2, 0), R_(2, 1), R_(2, 2), gt_pose.center(2, 0),
                      0, 0, 0, 1;*/
        //std::cout<<"init R: "<<std::endl;
        //std::cout<<R_init<<std::endl;

        //bev感知转换到车辆坐标系下
        pcl::transformPointCloud (*local_cloud_, *perception_cloud, T_gt.inverse());

        //local_map点云按pose_with_noise投影到车辆坐标系下
        pcl::transformPointCloud (*local_map_cloud_, *local_map_cloud, T_noise.inverse());

        //保存
        PointCloud::Ptr out_put(new PointCloud);
        *out_put += *perception_cloud;
        *out_put += *local_map_cloud;

        /* stringstream percep_map;
        percep_map<<"out_put.pcd";
        if(out_put->width>0)
            pcl::io::savePCDFile (percep_map.str (), *out_put, true); */


        //ICP 迭代匹配
        PointCloud::Ptr final_cloud(new PointCloud);
        PointCloud::Ptr perception_cloud_aligned(new PointCloud);
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;  //创建ICP的实例类
        icp.setInputSource(perception_cloud);
        icp.setInputTarget(local_map_cloud);
        icp.setMaxCorrespondenceDistance(5);    //有效关联点之间的最大距离m
        icp.setTransformationEpsilon(1e-6); 
        icp.setEuclideanFitnessEpsilon(0.001); 
        icp.setMaximumIterations(100);   
        icp.align(*final_cloud);
        delta_T_ = icp.getFinalTransformation ();                 //将source点转换到target点

        //计算重定位的位姿
        Eigen::Matrix4f reloca_T = T_noise*delta_T_;
        reloca_R_ << reloca_T(0, 0), reloca_T(0, 1), reloca_T(0, 2), 
                                    reloca_T(1, 0), reloca_T(1, 1), reloca_T(1, 2), 
                                    reloca_T(2, 0), reloca_T(2, 1), reloca_T(2, 2);

        reloca_t_ << reloca_T(0, 3), reloca_T(1, 3), reloca_T(2, 3);
        
        pcl::transformPointCloud (*perception_cloud, *perception_cloud_aligned, delta_T_);

        //保存
        PointCloud::Ptr out_put2(new PointCloud);
        *out_put2 += *perception_cloud_aligned;
        *out_put2 += *local_map_cloud;

        /* stringstream frame;
        frame<<"frame_aligned.pcd";
        if(out_put2->width>0)
            pcl::io::savePCDFile (frame.str (), *out_put2, true); */

    }

    b_pose bev_relocalizer::getRelocaPose()
    {
        b_pose final_pose;
        double eular[3];
        rotToAngle(reloca_R_, eular);

        final_pose.eular(0,0) = eular[0];
        final_pose.eular(1,0) = eular[1];
        final_pose.eular(2,0) = eular[2];
    
        final_pose.center(0,0) = reloca_t_(0, 0);
        final_pose.center(1,0) = reloca_t_(1, 0);
        final_pose.center(2,0) = reloca_t_(2, 0);

        return final_pose;
    
    }

    void bev_relocalizer::getLocalMap(const hd_map& g_map, b_pose init_pose)
    {
        local_map_->clear();
        local_map_cloud_->points.clear();
        float bev_vertex_x1 = BEV_RANGE_LENGTH/2 + CAR_LENGTH/2 - CAR_REAR_TO_CENTER;
        float bev_vetex_y = BEV_RANGE_WIDTH/2;


        float radius = pos_error_ + sqrt(bev_vertex_x1*bev_vertex_x1 + bev_vetex_y*bev_vetex_y);
        //slot_corners
        PointT point;
        for(auto pt = g_map.slot_corners_.begin(); pt!=g_map.slot_corners_.end(); pt++)
        {
            float x = (pt->point(0,0) - init_pose.center(0,0));
            float y = (pt->point(1,0) - init_pose.center(1,0));

            if(sqrt(x*x + y*y) < radius)
            {
                point.x = pt->point(0,0);
                point.y = pt->point(1,0);
                point.z = SLOT_CORNER_Z; 

                local_map_->slot_corners_.push_back(*pt);
                local_map_cloud_->points.push_back(point);
            }
        }

        //lanes
        for(auto pt = g_map.lanes_.begin(); pt!=g_map.lanes_.end(); pt++)
        {
            float x1 = (pt->point1(0,0) - init_pose.center(0,0));
            float y1 = (pt->point1(1,0) - init_pose.center(1,0));

            float x2 = (pt->point2(0,0) - init_pose.center(0,0));
            float y2 = (pt->point2(1,0) - init_pose.center(1,0));

            if(sqrt(x1*x1 + y1*y1) < radius || sqrt(x2*x2 + y2*y2) < radius)
            {
                local_map_->lanes_.push_back(*pt);

                double v_x =  pt->point2(0,0) -  pt->point1(0,0);
                double v_y =  pt->point2(1,0) -  pt->point1(1,0);
                double line_length = sqrt(v_x*v_x + v_y*v_y);
                v_x = v_x/line_length;
                v_y = v_y/line_length;
                int pt_num = static_cast<int>(line_length/INTERVAL);
                
                for(int i = 0; i < pt_num; i++)
                {
                    point.x = pt->point1(0,0) + i*INTERVAL*v_x;
                    point.y = pt->point1(1,0) + i*INTERVAL*v_y;
                    point.z = LANE_Z;
                    local_map_cloud_->points.push_back(point);
                }

            }
        }

        //long white lines
        for(auto pt = g_map.long_white_lines_.begin(); pt!=g_map.long_white_lines_.end(); pt++)
        {
            float x1 = (pt->point1(0,0) - init_pose.center(0,0));
            float y1 = (pt->point1(1,0) - init_pose.center(1,0));

            float x2 = (pt->point2(0,0) - init_pose.center(0,0));
            float y2 = (pt->point2(1,0) - init_pose.center(1,0));

            if(sqrt(x1*x1 + y1*y1) < radius || sqrt(x2*x2 + y2*y2) < radius)
            {
                local_map_->long_white_lines_.push_back(*pt);

                double v_x =  pt->point2(0,0) -  pt->point1(0,0);
                double v_y =  pt->point2(1,0) -  pt->point1(1,0);
                double line_length = sqrt(v_x*v_x + v_y*v_y);
                v_x = v_x/line_length;
                v_y = v_y/line_length;
                int pt_num = static_cast<int>(line_length/INTERVAL);
                
                for(int i = 0; i < pt_num; i++)
                {
                    point.x = pt->point1(0,0) + i*INTERVAL*v_x;
                    point.y = pt->point1(1,0) + i*INTERVAL*v_y;
                    point.z = LONG_WHITE_LINE_Z;
                    local_map_cloud_->points.push_back(point);
                }

            }
        }
        /* for(auto pt = g_map.long_white_lines_.begin(); pt!=g_map.long_white_lines_.end(); pt++)
        {
            float x1 = (pt->point1(0,0) - init_pose.center(0,0));
            float y1 = (pt->point1(1,0) - init_pose.center(1,0));

            float x2 = (pt->point2(0,0) - init_pose.center(0,0));
            float y2 = (pt->point2(1,0) - init_pose.center(1,0));

            float delta_x = pt->point2(0,0) - pt->point1(0,0);
            float delta_y = pt->point2(1,0) - pt->point1(1,0);
            double line_length = sqrt(delta_x*delta_x + delta_y*delta_y);

            if(sqrt(x1*x1 + y1*y1) < line_length && sqrt(x2*x2 + y2*y2) < line_length)
            {
                local_map_->long_white_lines_.push_back(*pt);

                double v_x = delta_x/line_length;
                double v_y = delta_y/line_length;
                int pt_num = static_cast<int>(line_length/INTERVAL);
                
                for(int i = 0; i < pt_num; i++)
                {
                    point.x = pt->point1(0,0) + i*INTERVAL*v_x;
                    point.y = pt->point1(1,0) + i*INTERVAL*v_y;
                    point.z = LONG_WHITE_LINE_Z;
                    local_map_cloud_->points.push_back(point);
                }

            }
        } */

        //arrows
        for(auto pt = g_map.arrows_.begin(); pt!=g_map.arrows_.end(); pt++)
        {
            float x1 = (pt->point1(0,0) - init_pose.center(0,0));
            float y1 = (pt->point1(1,0) - init_pose.center(1,0));

            float x2 = (pt->point2(0,0) - init_pose.center(0,0));
            float y2 = (pt->point2(1,0) - init_pose.center(1,0));

            if(sqrt(x1*x1 + y1*y1) < radius || sqrt(x2*x2 + y2*y2) < radius)
            {
                local_map_->arrows_.push_back(*pt);

                double v_x =  pt->point2(0,0) -  pt->point1(0,0);
                double v_y =  pt->point2(1,0) -  pt->point1(1,0);
                double line_length = sqrt(v_x*v_x + v_y*v_y);
                v_x = v_x/line_length;
                v_y = v_y/line_length;
                int pt_num = static_cast<int>(line_length/INTERVAL);
                
                for(int i = 0; i < pt_num; i++)
                {
                    point.x = pt->point1(0,0) + i*INTERVAL*v_x;
                    point.y = pt->point1(1,0) + i*INTERVAL*v_y;
                    point.z = ARROW_Z;
                    local_map_cloud_->points.push_back(point);
                }
            }
        }

        //zebras
        for(auto pt = g_map.zebras_.begin(); pt!=g_map.zebras_.end(); pt++)
        {
            float x1 = (pt->point1(0,0) - init_pose.center(0,0));
            float y1 = (pt->point1(1,0) - init_pose.center(1,0));

            float x2 = (pt->point2(0,0) - init_pose.center(0,0));
            float y2 = (pt->point2(1,0) - init_pose.center(1,0));

            if(sqrt(x1*x1 + y1*y1) < radius || sqrt(x2*x2 + y2*y2) < radius)
            {
                local_map_->zebras_.push_back(*pt);

                double v_x =  pt->point2(0,0) -  pt->point1(0,0);
                double v_y =  pt->point2(1,0) -  pt->point1(1,0);
                double line_length = sqrt(v_x*v_x + v_y*v_y);
                v_x = v_x/line_length;
                v_y = v_y/line_length;
                int pt_num = static_cast<int>(line_length/INTERVAL);
                
                for(int i = 0; i < pt_num; i++)
                {
                    point.x = pt->point1(0,0) + i*INTERVAL*v_x;
                    point.y = pt->point1(1,0) + i*INTERVAL*v_y;
                    point.z = ZEBRA_Z;
                    local_map_cloud_->points.push_back(point);
                }
            }
        }

        //speedbumps
        for(auto pt = g_map.speedbumps_.begin(); pt!=g_map.speedbumps_.end(); pt++)
        {
            float x1 = (pt->point1(0,0) - init_pose.center(0,0));
            float y1 = (pt->point1(1,0) - init_pose.center(1,0));

            float x2 = (pt->point2(0,0) - init_pose.center(0,0));
            float y2 = (pt->point2(1,0) - init_pose.center(1,0));

            if(sqrt(x1*x1 + y1*y1) < radius || sqrt(x2*x2 + y2*y2) < radius)
            {
                local_map_->speedbumps_.push_back(*pt);

                double v_x =  pt->point2(0,0) -  pt->point1(0,0);
                double v_y =  pt->point2(1,0) -  pt->point1(1,0);
                double line_length = sqrt(v_x*v_x + v_y*v_y);
                v_x = v_x/line_length;
                v_y = v_y/line_length;
                int pt_num = static_cast<int>(line_length/INTERVAL);
                
                for(int i = 0; i < pt_num; i++)
                {
                    point.x = pt->point1(0,0) + i*INTERVAL*v_x;
                    point.y = pt->point1(1,0) + i*INTERVAL*v_y;
                    point.z = SPEEDBUMP_Z;
                    local_map_cloud_->points.push_back(point);
                }
            }
        }

        local_map_cloud_->height = 1;
        local_map_cloud_->width = local_map_cloud_->points.size();
        local_map_cloud_->is_dense = true;  
        
        /* stringstream percep_map;
        percep_map<<"local_map_gt.pcd";
        pcl::io::savePCDFile (percep_map.str (), *local_map_cloud_, true);  */

    }

    b_pose bev_relocalizer::getCurrPose()
    {
        return pose_;
    }


    void bev_relocalizer::displayLocalMapInVehCord(cv::Mat& p, b_pose init_pose)
    {
        Eigen::Matrix3d R_init;
        
        double angles[3] = {init_pose.eular(0,0), init_pose.eular(1,0), init_pose.eular(2,0)};
        angleToRot(R_init, angles);

        Eigen::Matrix3d R = R_init.transpose();
        Eigen::Vector3d t = -R*init_pose.center;

         //slot_corners
        cv::Point2d slot_corner;
        for(auto pt = local_map_->slot_corners_.begin(); pt!=local_map_->slot_corners_.end(); pt++)
        {
            Eigen::Vector3d pt3 = R* pt->point + t;
            slot_corner.x = xPercepToPixel(pt3(0,0));
            slot_corner.y = yPercepToPixel(pt3(1,0));
            cv::circle(p, slot_corner, 1, cv::Scalar(0, 0, 0), 2);
        }

        cv::Point2d end_p1;
        cv::Point2d end_p2;
        //显示车道线
        for(auto lane = local_map_->lanes_.begin(); lane!=local_map_->lanes_.end(); lane++)
        {
            Eigen::Vector3d pt3_1 = R* lane->point1 + t;
            Eigen::Vector3d pt3_2 = R* lane->point2 + t;

            end_p1.x =xPercepToPixel(pt3_1(0,0));
            end_p1.y = yPercepToPixel(pt3_1(1,0));

            end_p2.x = xPercepToPixel(pt3_2(0,0));
            end_p2.y = yPercepToPixel(pt3_2(1,0));

            cv::line(p, end_p1, end_p2, cv::Scalar(0, 0 , 0), 2);
        }


         //显示车道线
        for(auto lane = local_map_->long_white_lines_.begin(); lane!=local_map_->long_white_lines_.end(); lane++)
        {
            Eigen::Vector3d pt3_1 = R* lane->point1 + t;
            Eigen::Vector3d pt3_2 = R* lane->point2 + t;

            end_p1.x =xPercepToPixel(pt3_1(0,0));
            end_p1.y = yPercepToPixel(pt3_1(1,0));

            end_p2.x = xPercepToPixel(pt3_2(0,0));
            end_p2.y = yPercepToPixel(pt3_2(1,0));

            cv::line(p, end_p1, end_p2, cv::Scalar(0, 0 , 0), 1);
        }



        //显示箭头
        for(auto lane = local_map_->arrows_.begin(); lane!=local_map_->arrows_.end(); lane++)
        {
            Eigen::Vector3d pt3_1 = R* lane->point1 + t;
            Eigen::Vector3d pt3_2 = R* lane->point2 + t;

            end_p1.x =xPercepToPixel(pt3_1(0,0));
            end_p1.y = yPercepToPixel(pt3_1(1,0));

            end_p2.x = xPercepToPixel(pt3_2(0,0));
            end_p2.y = yPercepToPixel(pt3_2(1,0));

            cv::line(p, end_p1, end_p2, cv::Scalar(0, 0 , 0), 2);
        }

        //显示斑马线
        for(auto lane = local_map_->zebras_.begin(); lane!=local_map_->zebras_.end(); lane++)
        {
            Eigen::Vector3d pt3_1 = R* lane->point1 + t;
            Eigen::Vector3d pt3_2 = R* lane->point2 + t;

            end_p1.x =xPercepToPixel(pt3_1(0,0));
            end_p1.y = yPercepToPixel(pt3_1(1,0));

            end_p2.x = xPercepToPixel(pt3_2(0,0));
            end_p2.y = yPercepToPixel(pt3_2(1,0));

            cv::line(p, end_p1, end_p2, cv::Scalar(0, 0 , 0), 2);
        }

        //显示减速带
        for(auto lane = local_map_->speedbumps_.begin(); lane!=local_map_->speedbumps_.end(); lane++)
        {
            Eigen::Vector3d pt3_1 = R* lane->point1 + t;
            Eigen::Vector3d pt3_2 = R* lane->point2 + t;

            end_p1.x =xPercepToPixel(pt3_1(0,0));
            end_p1.y = yPercepToPixel(pt3_1(1,0));

            end_p2.x = xPercepToPixel(pt3_2(0,0));
            end_p2.y = yPercepToPixel(pt3_2(1,0));

            cv::line(p, end_p1, end_p2, cv::Scalar(0, 0 , 0), 2);
        }
    }


    void bev_relocalizer::displayRelocaResult(cv::Mat& p)
    {
        b_pose delta_pose = getRelocaPose();
        displayLocalMapInVehCord(p, delta_pose);
    }


}//namespace bev_reloca
