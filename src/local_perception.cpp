#include "local_perception.h"
#include "bev_reloca_config.h"
#include "bev_frame.h"

namespace bev_reloca{


    local_perception::local_perception() : local_cloud_(new PointCloud), local_cloud_fused_(new PointCloud)
    {

    }


    int local_perception::updateCloud(Eigen::Matrix4d delta_pose, bev_frame*  curr_frame)
    {
        local_cloud_->points.clear();
        local_cloud_->width = local_cloud_->points.size();
        local_cloud_fused_->points.clear();
        local_cloud_fused_->width = local_cloud_fused_->points.size();

        PointCloud::Ptr last_frame_cloud(new PointCloud);
        PointCloud::Ptr last_cloud_aligned(new PointCloud);
        Eigen::Matrix4d relative_pose = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d poses[LOCAL_FRAMES_NUM];

        for(int i=0; i<LOCAL_FRAMES_NUM; i++) 
            poses[i] = Eigen::Matrix4d::Identity();

        frames_.push(curr_frame);
        relative_poses_.push(delta_pose);
        while(frames_.size()>LOCAL_FRAMES_NUM)
        {
            frames_.pop();
            //TO DO: release frame
            relative_poses_.pop();
        }

        if(first_frame_)
        {
            relative_poses_.pop();
            first_frame_ = false;
        }    
        
        //caculate relative poses
        for(int i=0; i<relative_poses_.size(); i++)
        {
            Eigen::Matrix4d delta_pose = relative_poses_.front();
            relative_poses_.pop();

            for(int j = 0; j<=i; j++)
            {
                poses[j] = delta_pose*poses[j];
            }
            relative_poses_.push(delta_pose);
        }
      
        //transfer last several frames to current frame
        for(int i = 0; i<frames_.size(); i++)
        {
            last_cloud_aligned->points.clear();

            bev_frame * last_frame = frames_.front();
            frames_.pop();
            frames_.push(last_frame);
            last_frame_cloud = last_frame->getCloud();

            pcl::transformPointCloud (*last_frame_cloud, *last_cloud_aligned, poses[i]);
            *local_cloud_ += * last_cloud_aligned;
        }
        local_cloud_->height = 1;
        local_cloud_->width = local_cloud_->points.size();
        local_cloud_->is_dense = true;  

         if(local_cloud_->width>0)
        {
            if(frames_.size() < LOCAL_FRAMES_NUM)
                min_neighbors_ = static_cast<int>(frames_.size()/2);
            else
            {
                min_neighbors_ = static_cast<int>(LOCAL_FRAMES_NUM/2)+1;
            }

            //remove wrong detected class
            pcl::RadiusOutlierRemoval<PointT> outliers_rem;
            outliers_rem.setInputCloud(local_cloud_);
            outliers_rem.setRadiusSearch(0.1);
            outliers_rem.setMinNeighborsInRadius(min_neighbors_);
            outliers_rem.filter(*local_cloud_fused_);
            local_cloud_fused_->height = 1;
            local_cloud_fused_->width = local_cloud_fused_->points.size();
            local_cloud_fused_->is_dense = true;  

            /* stringstream percep_map;
            percep_map<<"local.pcd";
            pcl::io::savePCDFile (percep_map.str (), *local_cloud_, true);  */
        }

        /* stringstream local_map;
        local_map<<"local_fused.pcd";
        if(local_cloud_fused_->width>0)
            pcl::io::savePCDFile (local_map.str (), *local_cloud_fused_, true);  */

    }


    int local_perception::updatePerception(Eigen::Matrix4d delta_pose, bev_frame*  curr_frame)
    {
        slot_corners_.clear();
        lanes_.clear();
        long_white_lines_.clear();
        arrows_.clear();
        zebras_.clear();
        speedbumps_.clear();

        Eigen::Matrix4d relative_pose = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d poses[LOCAL_FRAMES_NUM];

        for(int i=0; i<LOCAL_FRAMES_NUM; i++) 
            poses[i] = Eigen::Matrix4d::Identity();

        frames_.push(curr_frame);
        relative_poses_.push(delta_pose);
        while(frames_.size()>LOCAL_FRAMES_NUM)
        {
            frames_.pop();
            //release frame
            relative_poses_.pop();
        }

        if(first_frame_)
        {
            relative_poses_.pop();
            first_frame_ = false;
        }    
        
        for(int i=0; i<relative_poses_.size(); i++)
        {
            Eigen::Matrix4d delta_pose = relative_poses_.front();
            relative_poses_.pop();

            for(int j = 0; j<=i; j++)
            {
                poses[j] = delta_pose*poses[j];
            }
            relative_poses_.push(delta_pose);
        }

        //transfer last LOCAL_FRAMES_NUM-1 frames to current frame
        for(int i = 0; i<frames_.size(); i++)
        {
            bev_frame * last_frame = frames_.front();
            frames_.pop();
            frames_.push(last_frame);

            getNewPerception( poses[i], last_frame);
        }

        generateCloud();

    }


    void local_perception::getNewPerception(Eigen::Matrix4d delta_pose, bev_frame* frame)
    {
        Eigen::Matrix3d R;
        Eigen::Vector3d t;
        hd_point point_temp;
        hd_line line_temp;

        double min_distance_to_cam;
        double error;
        Eigen::Vector3d  cord_noise;
        Eigen::Vector3d  point1_temp, point2_temp;

        R << delta_pose(0,0), delta_pose(0,1), delta_pose(0,2),
                  delta_pose(1,0), delta_pose(1,1), delta_pose(1,2),
                  delta_pose(2,0), delta_pose(2,1), delta_pose(2,2);

        t << delta_pose(0,3), delta_pose(1,3), delta_pose(2,3);

        for(auto corner = frame->slot_corners_.begin(); corner != frame->slot_corners_.end(); corner++)
        {
            min_distance_to_cam = getMinDistanceToCam(Eigen::Vector3d(corner->point));
            error = ERROR_COEF[0]*min_distance_to_cam*min_distance_to_cam+ERROR_COEF[1]*min_distance_to_cam+ERROR_COEF[2];
            cord_noise = getCordNoise(error);

            point1_temp = corner->point+cord_noise;

            point_temp.point = R * point1_temp + t;
            point_temp.label = SLOT_CORNER;
            slot_corners_.push_back(point_temp);
        }

        line_temp.label = LANE;
        for(auto line = frame->lanes_.begin(); line != frame->lanes_.end(); line++)
        {
            min_distance_to_cam = getMinDistanceToCam(line->point1);
            error = ERROR_COEF[0]*min_distance_to_cam*min_distance_to_cam+ERROR_COEF[1]*min_distance_to_cam+ERROR_COEF[2];
            cord_noise = getCordNoise(error);

            point1_temp = line->point1+cord_noise;

            min_distance_to_cam = getMinDistanceToCam(line->point2);
            error = ERROR_COEF[0]*min_distance_to_cam*min_distance_to_cam+ERROR_COEF[1]*min_distance_to_cam+ERROR_COEF[2];
            cord_noise = getCordNoise(error);

            point2_temp = line->point2+cord_noise;

            line_temp.point1 = R * point1_temp + t;
            line_temp.point2 = R * point2_temp + t;
            lanes_.push_back(line_temp);
        }

        line_temp.label = LONG_WHITE_LINE;
        for(auto line = frame->long_white_lines_.begin(); line != frame->long_white_lines_.end(); line++)
        {
            min_distance_to_cam = getMinDistanceToCam(line->point1);
            error = ERROR_COEF[0]*min_distance_to_cam*min_distance_to_cam+ERROR_COEF[1]*min_distance_to_cam+ERROR_COEF[2];
            cord_noise = getCordNoise(error);

            point1_temp = line->point1+cord_noise;

            min_distance_to_cam = getMinDistanceToCam(line->point2);
            error = ERROR_COEF[0]*min_distance_to_cam*min_distance_to_cam+ERROR_COEF[1]*min_distance_to_cam+ERROR_COEF[2];
            cord_noise = getCordNoise(error);

            point2_temp = line->point2+cord_noise;

            line_temp.point1 = R * point1_temp + t;
            line_temp.point2 = R * point2_temp + t;
            long_white_lines_.push_back(line_temp);
        }



         line_temp.label = ARROW;
        for(auto line = frame->arrows_.begin(); line != frame->arrows_.end(); line++)
        {
            min_distance_to_cam = getMinDistanceToCam(line->point1);
            error = ERROR_COEF[0]*min_distance_to_cam*min_distance_to_cam+ERROR_COEF[1]*min_distance_to_cam+ERROR_COEF[2];
            cord_noise = getCordNoise(error);

            point1_temp = line->point1+cord_noise;

            min_distance_to_cam = getMinDistanceToCam(line->point2);
            error = ERROR_COEF[0]*min_distance_to_cam*min_distance_to_cam+ERROR_COEF[1]*min_distance_to_cam+ERROR_COEF[2];
            cord_noise = getCordNoise(error);

            point2_temp = line->point2+cord_noise;

            line_temp.point1 = R * point1_temp + t;
            line_temp.point2 = R * point2_temp + t;
            arrows_.push_back(line_temp);
        }

         line_temp.label = ZEBRA;
        for(auto line = frame->zebras_.begin(); line != frame->zebras_.end(); line++)
        {
            min_distance_to_cam = getMinDistanceToCam(line->point1);
            error = ERROR_COEF[0]*min_distance_to_cam*min_distance_to_cam+ERROR_COEF[1]*min_distance_to_cam+ERROR_COEF[2];
            cord_noise = getCordNoise(error);

            point1_temp = line->point1+cord_noise;

            min_distance_to_cam = getMinDistanceToCam(line->point2);
            error = ERROR_COEF[0]*min_distance_to_cam*min_distance_to_cam+ERROR_COEF[1]*min_distance_to_cam+ERROR_COEF[2];
            cord_noise = getCordNoise(error);

            point2_temp = line->point2+cord_noise;

            line_temp.point1 = R * point1_temp + t;
            line_temp.point2 = R * point2_temp + t;
            zebras_.push_back(line_temp);
        }

        line_temp.label = SPEEDBUMP;
        for(auto line = frame->speedbumps_.begin(); line != frame->speedbumps_.end(); line++)
        {
            min_distance_to_cam = getMinDistanceToCam(line->point1);
            error = ERROR_COEF[0]*min_distance_to_cam*min_distance_to_cam+ERROR_COEF[1]*min_distance_to_cam+ERROR_COEF[2];
            cord_noise = getCordNoise(error);

            point1_temp = line->point1+cord_noise;

            min_distance_to_cam = getMinDistanceToCam(line->point2);
            error = ERROR_COEF[0]*min_distance_to_cam*min_distance_to_cam+ERROR_COEF[1]*min_distance_to_cam+ERROR_COEF[2];
            cord_noise = getCordNoise(error);

            point2_temp = line->point2+cord_noise;

            line_temp.point1 = R * point1_temp + t;
            line_temp.point2 = R * point2_temp + t;
            speedbumps_.push_back(line_temp);
        }

    }

     double local_perception::calcLineDistance(hd_line line1, hd_line line2)
     {
        double result = 0;
        double middle_point_dis = 0;
        double end_point1_to_line = 0;
        double end_point2_to_line = 0;

        Eigen::Vector3d middle_point1 = 0.5*(line1.point1+line1.point2);
        Eigen::Vector3d middle_point2 = 0.5*(line2.point1+line2.point2);

        middle_point_dis = (middle_point1 - middle_point2).norm();   //线段中心点的距离

        //line1端点到line2的距离
        end_point1_to_line = ((line1.point1-line2.point1).cross(line2.point2-line2.point1)).norm()/(line2.point2-line2.point1).norm();
        end_point2_to_line = ((line1.point2-line2.point1).cross(line2.point2-line2.point1)).norm()/(line2.point2-line2.point1).norm();

        result = middle_point_dis+end_point1_to_line+end_point2_to_line;

        return result;
     }


     void local_perception::removeInvalidPerceptions()
     {
        int id_to_be_deleted = 0;  
        sort(invalid_slot_corners_.begin(), invalid_slot_corners_.end());
        for(int i = invalid_slot_corners_.size()-1; i>=0;  i-- )
        {
            id_to_be_deleted = invalid_slot_corners_[i];
            slot_corners_.erase(slot_corners_.begin()+id_to_be_deleted);
        }

        sort(invalid_lanes_.begin(), invalid_lanes_.end());
        for(int i = invalid_lanes_.size()-1; i>=0;  i-- )
        {
            id_to_be_deleted = invalid_lanes_[i];
            lanes_.erase(lanes_.begin()+id_to_be_deleted);
        }

        sort(invalid_arrows_.begin(), invalid_arrows_.end());
        for(int i = invalid_arrows_.size()-1; i>=0;  i-- )
        {
            id_to_be_deleted = invalid_arrows_[i];
            arrows_.erase(arrows_.begin()+id_to_be_deleted);
        }

        sort(invalid_zebras_.begin(), invalid_zebras_.end());
        for(int i = invalid_zebras_.size()-1; i>=0;  i-- )
        {
            id_to_be_deleted = invalid_zebras_[i];
            zebras_.erase(zebras_.begin()+id_to_be_deleted);
        }

         sort(invalid_speedbumps_.begin(), invalid_speedbumps_.end());
        for(int i = invalid_speedbumps_.size()-1; i>=0;  i-- )
        {
            id_to_be_deleted = invalid_speedbumps_[i];
            speedbumps_.erase(speedbumps_.begin()+id_to_be_deleted);
        }

     }


    void local_perception::fusePerceptions()
    {
        int count = 0;

        hd_point point1, point2;
        hd_line line1, line2;
        double distance = 0;

        invalid_slot_corners_.clear();
        invalid_lanes_.clear();
        invalid_arrows_.clear();
        invalid_zebras_.clear();
        invalid_speedbumps_.clear();

        if(frames_.size() < LOCAL_FRAMES_NUM)
            min_neighbors_ = static_cast<int>(frames_.size()/2);
        else
        {
            min_neighbors_ = static_cast<int>(LOCAL_FRAMES_NUM/2)+1;
        }
        
        //slot corners
        for(int i = 0; i<slot_corners_.size(); i++)
        {
            count = 1;
            point1 = slot_corners_[i];
            for(int j = 0; j < slot_corners_.size(); j++)
            {
                if( i == j)
                    continue;

                point2 = slot_corners_[j];
                distance = sqrt((point1.point.x()-point2.point.x())*(point1.point.x()-point2.point.x()) + (point1.point.y()-point2.point.y())*(point1.point.y()-point2.point.y()));
                if(distance<SLOT_CORNER_FUSE_DIS)
                    count++;
            }

            if(count<min_neighbors_)
                invalid_slot_corners_.push_back(i);
        }

        //lanes
        for(int i = 0; i < lanes_.size(); i++)
        {
            count = 1;
            line1 = lanes_[i];
            for(int j = 0; j<lanes_.size(); j++)
            {
                if(i == j)
                    continue;
                
                line2 = lanes_[j];
                distance = calcLineDistance(line1, line2);
                if(distance < LINE_FUSE_DIS)
                    count++;
            }

            if(count<min_neighbors_)
                invalid_lanes_.push_back(i);
        }

        //arrows
        for(int i = 0; i < arrows_.size(); i++)
        {
            count = 1;
            line1 = arrows_[i];
            for(int j = 0; j<arrows_.size(); j++)
            {
                if(i == j)
                    continue;
                
                line2 = arrows_[j];
                distance = calcLineDistance(line1, line2);
                if(distance < LINE_FUSE_DIS)
                    count++;
            }

            if(count<min_neighbors_)
                invalid_arrows_.push_back(i);
        }

        //speedbumps
        for(int i = 0; i < speedbumps_.size(); i++)
        {
            count = 1;
            line1 = speedbumps_[i];
            for(int j = 0; j<speedbumps_.size(); j++)
            {
                if(i == j)
                    continue;
                
                line2 = speedbumps_[j];
                distance = calcLineDistance(line1, line2);
                if(distance < LINE_FUSE_DIS)
                    count++;
            }

            if(count<min_neighbors_)
                invalid_speedbumps_.push_back(i);
        }

        //zebras
        for(int i = 0; i < zebras_.size(); i++)
        {
            count = 1;
            line1 = zebras_[i];
            for(int j = 0; j<zebras_.size(); j++)
            {
                if(i == j)
                    continue;
                
                line2 = zebras_[j];
                distance = calcLineDistance(line1, line2);
                if(distance < LINE_FUSE_DIS)
                    count++;
            }

            if(count<min_neighbors_)
                invalid_zebras_.push_back(i);
        }

        removeInvalidPerceptions();

    }


    void local_perception::generateCloud()
    {
        PointT point;
        double v_x = 0;
        double v_y = 0;
        double line_length = 0;
        int pt_num = 0;

        fusePerceptions();
        local_cloud_fused_->points.clear();

        //slot_corners
        for(auto corner = slot_corners_.begin(); corner!=slot_corners_.end(); corner++)
        {
            point.x = corner->point(0,0);
            point.y = corner->point(1,0);
            point.z = -SLOT_CORNER_Z;                                                  //车辆坐标系，Z值取负值
            local_cloud_fused_->points.push_back(point);
        }

         //lanes
        for(auto  line = lanes_.begin(); line!=lanes_.end(); line++)
        {
            v_x = line->point2(0,0) - line->point1(0,0);
            v_y = line->point2(1,0) - line->point1(1,0);
            line_length = sqrt(v_x*v_x + v_y*v_y);
            v_x = v_x/line_length;
            v_y = v_y/line_length;

            pt_num = static_cast<int>(line_length/INTERVAL);

            for(int i = 0; i<pt_num; i++)
            {
                point.x = line->point1(0,0) + i*INTERVAL*v_x;
                point.y = line->point1(1,0) + i*INTERVAL*v_y;
                point.z = -LANE_Z;
                local_cloud_fused_->points.push_back(point);
            }
        }

        //long white lines
        for(auto  line = long_white_lines_.begin(); line!=long_white_lines_.end(); line++)
        {
            v_x = line->point2(0,0) - line->point1(0,0);
            v_y = line->point2(1,0) - line->point1(1,0);
            line_length = sqrt(v_x*v_x + v_y*v_y);
            v_x = v_x/line_length;
            v_y = v_y/line_length;

            pt_num = static_cast<int>(line_length/INTERVAL);

            for(int i = 0; i<pt_num; i++)
            {
                point.x = line->point1(0,0) + i*INTERVAL*v_x;
                point.y = line->point1(1,0) + i*INTERVAL*v_y;
                point.z = -LONG_WHITE_LINE_Z;
                local_cloud_fused_->points.push_back(point);
            }
        }

           //arrows
        for(auto  line = arrows_.begin(); line!=arrows_.end(); line++)
        {
            v_x = line->point2(0,0) - line->point1(0,0);
            v_y = line->point2(1,0) - line->point1(1,0);
            line_length = sqrt(v_x*v_x + v_y*v_y);
            v_x = v_x/line_length;
            v_y = v_y/line_length;

            pt_num = static_cast<int>(line_length/INTERVAL);

            for(int i = 0; i<pt_num; i++)
            {
                point.x = line->point1(0,0) + i*INTERVAL*v_x ;
                point.y = line->point1(1,0) + i*INTERVAL*v_y ;
                point.z = -ARROW_Z;
                local_cloud_fused_->points.push_back(point);
            }
        }

           //zebras
        for(auto  line = zebras_.begin(); line!=zebras_.end(); line++)
        {
            v_x = line->point2(0,0) - line->point1(0,0);
            v_y = line->point2(1,0) - line->point1(1,0);
            line_length = sqrt(v_x*v_x + v_y*v_y);
            v_x = v_x/line_length;
            v_y = v_y/line_length;

            pt_num = static_cast<int>(line_length/INTERVAL);

            for(int i = 0; i<pt_num; i++)
            {
                point.x = line->point1(0,0) + i*INTERVAL*v_x ;
                point.y = line->point1(1,0) + i*INTERVAL*v_y ;
                point.z = -ZEBRA_Z;
                local_cloud_fused_->points.push_back(point);
            }
        }

           //speedbumps
        for(auto  line = speedbumps_.begin(); line!=speedbumps_.end(); line++)
        {
            v_x = line->point2(0,0) - line->point1(0,0);
            v_y = line->point2(1,0) - line->point1(1,0);
            line_length = sqrt(v_x*v_x + v_y*v_y);
            v_x = v_x/line_length;
            v_y = v_y/line_length;

            pt_num = static_cast<int>(line_length/INTERVAL);

            for(int i = 0; i<pt_num; i++)
            {
                point.x = line->point1(0,0) + i*INTERVAL*v_x;
                point.y = line->point1(1,0) + i*INTERVAL*v_y;
                point.z = -SPEEDBUMP_Z;
                local_cloud_fused_->points.push_back(point);
            }
        }

        local_cloud_fused_->height = 1;
        local_cloud_fused_->width = local_cloud_fused_->points.size();
        local_cloud_fused_->is_dense = true;  

        /* stringstream local_map;
        local_map<<"local_fused.pcd";
        if(local_cloud_fused_->width>0)
            pcl::io::savePCDFile (local_map.str (), *local_cloud_fused_, true);  */

    }

    PointCloud::Ptr local_perception::getLocalFusedCloud()
    {
        return local_cloud_fused_;
    }


    double local_perception::getMinDistanceToCam(Eigen::Vector3d point)
    {
        double min_distance = 20;
        double distance = 0;
        for(int i = 0; i<4; i++)
        {
            distance = sqrt((point(0,0)-CAM_CENTER[i][0])*(point(0,0)-CAM_CENTER[i][0]) + (point(1,0)-CAM_CENTER[i][1])*(point(1,0)-CAM_CENTER[i][1]));
            if(distance<min_distance)
            {
                min_distance = distance;
            }
        }

        return distance;

    }

    Eigen::Vector3d local_perception::getCordNoise(double error)
    {
        Eigen::Vector3d noise;
        double angle = 0;
        
        struct timeval time1;
        gettimeofday(&time1, NULL);
        unsigned int seed1 = (((long)time1.tv_sec)*1000+(long)time1.tv_usec/1000)%200;
        srand(seed1);

        angle = rand()/double(RAND_MAX) * 2*PI;
        
        noise.x() = error*sin(angle);
        noise.y() = error*cos(angle);
        noise.z() = 0;

        return noise;

    }


}//namespace bev_reloca