#include "bev_frame.h"
#include "hd_map.h"
#include "converter.h"

namespace bev_reloca{

    std::atomic<unsigned int> bev_frame::next_id_{0};

    bev_frame::bev_frame(const hd_map& g_map, b_pose gt_pose) : id_(next_id_++), percep_cloud_(new PointCloud)
    {
        gt_pose_ = gt_pose;                            //gt pose
        double angles[3] = {gt_pose_.eular(0,0), gt_pose_.eular(1,0), gt_pose_.eular(2,0)};
        angleToRot(gt_R_, angles);              //gt pose R
        gt_t_(0, 0) = gt_pose_.center(0,0);  //gt pose t
        gt_t_(1, 0) = gt_pose_.center(1,0);
        gt_t_(2, 0) = gt_pose_.center(2,0);

        getBevRect();
        getPerception(g_map);
    }

    void bev_frame::getPerception(const hd_map& g_map)
    {
        getSlotCorners(g_map);
        getLanes(g_map);
        getLongWhiteLines(g_map);
        getArrows(g_map);
        getZebras(g_map);
        getSpeedbumps(g_map);

        generatePointCloud();
    }

    void bev_frame::getSlotCorners(const hd_map& g_map)
    {
        slot_corners_.clear();
        Eigen::Matrix3d R = gt_R_.transpose();
        Eigen::Vector3d t = -R*gt_t_;

        float perception_rate = 0;
        struct timeval time1;

        hd_point p_temp;
        p_temp.label = SLOT_CORNER;
        p_temp.point(2,0) = 0;

        gettimeofday(&time1, NULL);
        unsigned int seed1 = (((long)time1.tv_sec)*1000+(long)time1.tv_usec/1000)%200;
        srand(seed1);

        for(auto pt = g_map.slot_corners_.begin(); pt!=g_map.slot_corners_.end(); pt++)
        {
            float x = (pt->point(0,0) - gt_pose_.center(0,0));
            float y = (pt->point(1,0) - gt_pose_.center(1,0));
            if(abs(x) > 20 || abs(y)>20) continue;

            perception_rate = rand()/double(RAND_MAX);
            if(perception_rate <= loss_rate_[0]) continue;     //percetion loss

            //std::cout<<"slot in g map: ("<<pt->point(0,0)<<", "<<pt->point(1,0)<<")"<<std::endl;
            Eigen::Vector3d point = R*pt->point + t;  //transfer to vehicle coord
            //std::cout<<"slot in vehicle: ("<<point(0,0)<<", "<<point(1,0)<<")"<<std::endl;


            if (isInRect(bev_rect_, point(0, 0), point(1,0)) && !isInRect(car_icon_,  point(0, 0), point(1,0)))
            {
                p_temp.point(0,0) = point(0,0);
                p_temp.point(1,0) = point(1,0);
                slot_corners_.push_back(p_temp);
            }
        }

    }
    

     void bev_frame::getLanes(const hd_map& g_map)
     {
        lanes_.clear();
        Eigen::Matrix3d R = gt_R_.transpose();
        Eigen::Vector3d t = -R*gt_t_;

        float perception_rate = 0;
        float pricise_rate = 0;
        float error_class = 0;
        struct timeval time1;

        hd_line line_temp;
        line_temp.label = LANE;
        line_temp.point1(2,0) = 0;
        line_temp.point2(2,0) = 0;

        gettimeofday(&time1, NULL);
        unsigned int seed1 = (((long)time1.tv_sec)*1000+(long)time1.tv_usec/1000)%200;
        srand(seed1);

        for(auto pt = g_map.lanes_.begin(); pt!=g_map.lanes_.end(); pt++)
        {
            if(abs(pt->point1(0,0) - gt_pose_.center(0,0)) > 20 || abs(pt->point1(1,0) - gt_pose_.center(1,0)) > 20 ||
                abs(pt->point2(0,0) - gt_pose_.center(0,0)) > 20 || abs(pt->point2(1,0) - gt_pose_.center(1,0)) > 20)
                continue;

            std::vector<std::vector<cv::Point2f>> lines = projectTwoPointLine2(pt->point1, pt->point2);
            if(!lines.empty())
            {
                for(auto line = lines.begin(); line!= lines.end(); line++)
                {

                    if(sqrt(((*line)[0].x - (*line)[1].x)*((*line)[0].x-(*line)[1].x) + ((*line)[0].y-(*line)[1].y)*((*line)[0].y-(*line)[1].y)) < 0.5)
                        continue;

                    //-------------------------------------------------------------------
                    perception_rate = rand()/double(RAND_MAX);
                    if(perception_rate <= loss_rate_[1]) continue; //loss
                    //------------------------------------------------------------------

                    line_temp.point1(0,0) = (*line)[0].x;
                    line_temp.point1(1,0) = (*line)[0].y;

                    line_temp.point2(0,0) = (*line)[1].x;
                    line_temp.point2(1,0) = (*line)[1].y;
                    //------------------------------------------------------------------------------------
                    hd_line line_t;
                    Eigen::Vector3d point3 = R*line_temp.point1 + t;   //投影到车辆坐标系
                    Eigen::Vector3d point4 = R*line_temp.point2 + t;
                    line_t.point1(2,0) = 0;  line_t.point1(0,0) = point3(0,0);  line_t.point1(1,0) = point3(1,0); 
                    line_t.point2(2,0) = 0;  line_t.point2(0,0) = point4(0,0);  line_t.point2(1,0) = point4(1,0); 
                    
                    pricise_rate = rand()/double(RAND_MAX);
                    error_class = rand()/double(RAND_MAX);
                    if(pricise_rate <= false_rate_[1][1])   //误检
                    {
                        if(error_class <= false_rate_[1][2])  //误分为arrow
                        {
                            line_t.label = ARROW;
                            arrows_.push_back(line_t);
                        }
                        else if(error_class <= false_rate_[1][2]+false_rate_[1][3])  //误分为zebra
                        {
                            line_t.label = ZEBRA;
                            zebras_.push_back(line_t);
                        }
                        else  //误分为speedbump
                        {
                            line_t.label = SPEEDBUMP;
                            speedbumps_.push_back(line_t);
                        }

                    }
                    else
                    {
                        line_t.label = LANE;
                        lanes_.push_back(line_t);
                    }
                    //-------------------------------------------------------------------------------------
                }
            }
        }

    }


    void bev_frame::getLongWhiteLines(const hd_map& g_map)
    {
        long_white_lines_.clear();
        Eigen::Matrix3d R = gt_R_.transpose();
        Eigen::Vector3d t = -R*gt_t_;

        float perception_rate = 0;
        float pricise_rate = 0;
        float error_class = 0;
        struct timeval time1;

        hd_line line_temp;
        line_temp.label = LONG_WHITE_LINE;
        line_temp.point1(2,0) = 0;
        line_temp.point2(2,0) = 0;

        gettimeofday(&time1, NULL);
        unsigned int seed1 = (((long)time1.tv_sec)*1000+(long)time1.tv_usec/1000)%200;
        srand(seed1);

        for(auto pt = g_map.long_white_lines_.begin(); pt!=g_map.long_white_lines_.end(); pt++)
        {
            /* double delta_x = pt->point1(0,0)-pt->point2(0,0);
            double delta_y = pt->point1(1,0)-pt->point2(1,0);
            double line_length = sqrt(delta_x*delta_x + delta_y*delta_y);

            if(abs(pt->point1(0,0) - gt_pose_.center(0,0)) > line_length || abs(pt->point1(1,0) - gt_pose_.center(1,0)) > line_length ||
                abs(pt->point2(0,0) - gt_pose_.center(0,0)) > line_length || abs(pt->point2(1,0) - gt_pose_.center(1,0)) > line_length)
                continue; */

            if(abs(pt->point1(0,0) - gt_pose_.center(0,0)) > 20 || abs(pt->point1(1,0) - gt_pose_.center(1,0)) > 20 ||
                abs(pt->point2(0,0) - gt_pose_.center(0,0)) > 20 || abs(pt->point2(1,0) - gt_pose_.center(1,0)) > 20)
                continue;

            std::vector<std::vector<cv::Point2f>> lines = projectTwoPointLine2(pt->point1, pt->point2);
            if(!lines.empty())
            {
                for(auto line = lines.begin(); line!= lines.end(); line++)
                {

                    if(sqrt(((*line)[0].x - (*line)[1].x)*((*line)[0].x-(*line)[1].x) + ((*line)[0].y-(*line)[1].y)*((*line)[0].y-(*line)[1].y)) < 0.5)
                        continue;

                    //-------------------------------------------------------------------
                    perception_rate = rand()/double(RAND_MAX);
                    if(perception_rate <= 0) continue; //loss
                    //------------------------------------------------------------------

                    line_temp.point1(0,0) = (*line)[0].x;
                    line_temp.point1(1,0) = (*line)[0].y;

                    line_temp.point2(0,0) = (*line)[1].x;
                    line_temp.point2(1,0) = (*line)[1].y;

                    //std::cout<<"line in g map: ("<<line_temp.point1(0,0)<<", "<<line_temp.point1(1,0)<<"), ("<<line_temp.point2(0,0)<<", "<<line_temp.point2(1,0)<<")"<<std::endl;
                    //------------------------------------------------------------------------------------
                    hd_line line_t;
                    Eigen::Vector3d point3 = R*line_temp.point1 + t;   //投影到车辆坐标系
                    Eigen::Vector3d point4 = R*line_temp.point2 + t;
                    line_t.point1(2,0) = 0;  line_t.point1(0,0) = point3(0,0);  line_t.point1(1,0) = point3(1,0); 
                    line_t.point2(2,0) = 0;  line_t.point2(0,0) = point4(0,0);  line_t.point2(1,0) = point4(1,0); 

                    //std::cout<<"line in vehicle: ("<<line_t.point1(0,0)<<", "<<line_t.point1(1,0)<<"), ("<<line_t.point2(0,0)<<", "<<line_t.point2(1,0)<<")"<<std::endl;
                    
                    pricise_rate = rand()/double(RAND_MAX);
                    error_class = rand()/double(RAND_MAX);
                    if(pricise_rate <= 0)   //误检
                    {
                        if(error_class <= false_rate_[1][2])  //误分为arrow
                        {
                            line_t.label = ARROW;
                            arrows_.push_back(line_t);
                        }
                        else if(error_class <= false_rate_[1][2]+false_rate_[1][3])  //误分为zebra
                        {
                            line_t.label = ZEBRA;
                            zebras_.push_back(line_t);
                        }
                        else  //误分为speedbump
                        {
                            line_t.label = SPEEDBUMP;
                            speedbumps_.push_back(line_t);
                        }

                    }
                    else
                    {
                        line_t.label = LONG_WHITE_LINE;
                        long_white_lines_.push_back(line_t);
                    }
                    //-------------------------------------------------------------------------------------
                }
            }
        }
    }


    void bev_frame::getArrows(const hd_map& g_map)
    {
        arrows_.clear();
        Eigen::Matrix3d R = gt_R_.transpose();
        Eigen::Vector3d t = -R*gt_t_;

        float perception_rate = 0;
        float pricise_rate = 0;
        float error_class = 0;
        struct timeval time1;

        hd_line line_temp;
        line_temp.label = ARROW;
        line_temp.point1(2,0) = 0;
        line_temp.point2(2,0) = 0;

        gettimeofday(&time1, NULL);
        unsigned int seed1 = (((long)time1.tv_sec)*1000+(long)time1.tv_usec/1000)%200;
        srand(seed1);

        for(auto pt = g_map.arrows_.begin(); pt!=g_map.arrows_.end(); pt++)
        {
            if(abs(pt->point1(0,0) - gt_pose_.center(0,0)) > 20 || abs(pt->point1(1,0) - gt_pose_.center(1,0)) > 20 ||
                abs(pt->point2(0,0) - gt_pose_.center(0,0)) > 20 || abs(pt->point2(1,0) - gt_pose_.center(1,0)) > 20)
                continue;

            std::vector<std::vector<cv::Point2f>> lines = projectTwoPointLine2(pt->point1, pt->point2);
            if(!lines.empty())
            {
                for(auto line = lines.begin(); line!= lines.end(); line++)
                {

                    if(sqrt(((*line)[0].x - (*line)[1].x)*((*line)[0].x-(*line)[1].x) + ((*line)[0].y-(*line)[1].y)*((*line)[0].y-(*line)[1].y)) < 0.5)
                        continue;

                    //-------------------------------------------------------------------
                    perception_rate = rand()/double(RAND_MAX);
                    if(perception_rate <= loss_rate_[2]) continue; //loss
                    //------------------------------------------------------------------

                    line_temp.point1(0,0) = (*line)[0].x;
                    line_temp.point1(1,0) = (*line)[0].y;

                    line_temp.point2(0,0) = (*line)[1].x;
                    line_temp.point2(1,0) = (*line)[1].y;
                    //------------------------------------------------------------------------------------
                    hd_line line_t;
                    Eigen::Vector3d point3 = R*line_temp.point1 + t;
                    Eigen::Vector3d point4 = R*line_temp.point2 + t;
                    line_t.point1(2,0) = 0;  line_t.point1(0,0) = point3(0,0);  line_t.point1(1,0) = point3(1,0); 
                    line_t.point2(2,0) = 0;  line_t.point2(0,0) = point4(0,0);  line_t.point2(1,0) = point4(1,0); 
                    
                    pricise_rate = rand()/double(RAND_MAX);
                    error_class = rand()/double(RAND_MAX);
                    if(pricise_rate <= false_rate_[2][2])   //误检
                    {
                        if(error_class <= false_rate_[2][1])  //误分为lane
                        {
                            line_t.label = LANE;
                            lanes_.push_back(line_t);
                        }
                        else if(error_class <= false_rate_[2][1]+false_rate_[2][3])  //误分为zebra
                        {
                            line_t.label = ZEBRA;
                            zebras_.push_back(line_t);
                        }
                        else  //误分为speedbump
                        {
                            line_t.label = SPEEDBUMP;
                            speedbumps_.push_back(line_t);
                        }

                    }
                    else
                    {
                        line_t.label = ARROW;
                        arrows_.push_back(line_t);
                    }
                    //-------------------------------------------------------------------------------------
                }
            }
        }

    }

    void bev_frame::getZebras(const hd_map& g_map)
    {
        zebras_.clear();
        Eigen::Matrix3d R = gt_R_.transpose();
        Eigen::Vector3d t = -R*gt_t_;

        float perception_rate = 0;
        float pricise_rate = 0;
        float error_class = 0;
        struct timeval time1;

        hd_line line_temp;
        line_temp.label = ZEBRA;
        line_temp.point1(2,0) = 0;
        line_temp.point2(2,0) = 0;

        gettimeofday(&time1, NULL);
        unsigned int seed1 = (((long)time1.tv_sec)*1000+(long)time1.tv_usec/1000)%200;
        srand(seed1);

        for(auto pt = g_map.zebras_.begin(); pt!=g_map.zebras_.end(); pt++)
        {
            if(abs(pt->point1(0,0) - gt_pose_.center(0,0)) > 20 || abs(pt->point1(1,0) - gt_pose_.center(1,0)) > 20 ||
                abs(pt->point2(0,0) - gt_pose_.center(0,0)) > 20 || abs(pt->point2(1,0) - gt_pose_.center(1,0)) > 20)
                continue;

            std::vector<std::vector<cv::Point2f>> lines = projectTwoPointLine2(pt->point1, pt->point2);
            if(!lines.empty())
            {
                for(auto line = lines.begin(); line!= lines.end(); line++)
                {

                    if(sqrt(((*line)[0].x - (*line)[1].x)*((*line)[0].x-(*line)[1].x) + ((*line)[0].y-(*line)[1].y)*((*line)[0].y-(*line)[1].y)) < 0.5)
                        continue;

                    //-------------------------------------------------------------------
                    perception_rate = rand()/double(RAND_MAX);
                    if(perception_rate <= loss_rate_[3]) continue; //loss
                    //------------------------------------------------------------------

                    line_temp.point1(0,0) = (*line)[0].x;
                    line_temp.point1(1,0) = (*line)[0].y;

                    line_temp.point2(0,0) = (*line)[1].x;
                    line_temp.point2(1,0) = (*line)[1].y;
                    //------------------------------------------------------------------------------------
                    hd_line line_t;
                    Eigen::Vector3d point3 = R*line_temp.point1 + t;
                    Eigen::Vector3d point4 = R*line_temp.point2 + t;
                    line_t.point1(2,0) = 0;  line_t.point1(0,0) = point3(0,0);  line_t.point1(1,0) = point3(1,0); 
                    line_t.point2(2,0) = 0;  line_t.point2(0,0) = point4(0,0);  line_t.point2(1,0) = point4(1,0); 
                    
                    pricise_rate = rand()/double(RAND_MAX);
                    error_class = rand()/double(RAND_MAX);
                    if(pricise_rate <= false_rate_[3][3])   //误检
                    {
                        if(error_class <= false_rate_[3][1])  //误分为lane
                        {
                            line_t.label = LANE;
                            lanes_.push_back(line_t);
                        }
                        else if(error_class <= false_rate_[3][1]+false_rate_[3][2])  //误分为arrow
                        {
                            line_t.label = ARROW;
                            arrows_.push_back(line_t);
                        }
                        else  //误分为speedbump
                        {
                            line_t.label = SPEEDBUMP;
                            speedbumps_.push_back(line_t);
                        }

                    }
                    else
                    {
                        line_t.label = ZEBRA;
                        zebras_.push_back(line_t);
                    }
                    //-------------------------------------------------------------------------------------
                }
            }
        }
    }


    void bev_frame::getSpeedbumps(const hd_map& g_map)
    {
        speedbumps_.clear();
        Eigen::Matrix3d R = gt_R_.transpose();
        Eigen::Vector3d t = -R*gt_t_;

        float perception_rate = 0;
        float pricise_rate = 0;
        float error_class = 0;
        struct timeval time1;

        hd_line line_temp;
        line_temp.label = SPEEDBUMP;
        line_temp.point1(2,0) = 0;
        line_temp.point2(2,0) = 0;

        gettimeofday(&time1, NULL);
        unsigned int seed1 = (((long)time1.tv_sec)*1000+(long)time1.tv_usec/1000)%200;
        srand(seed1);

        for(auto pt = g_map.speedbumps_.begin(); pt!=g_map.speedbumps_.end(); pt++)
        {
            if(abs(pt->point1(0,0) - gt_pose_.center(0,0)) > 20 || abs(pt->point1(1,0) - gt_pose_.center(1,0)) > 20 ||
                abs(pt->point2(0,0) - gt_pose_.center(0,0)) > 20 || abs(pt->point2(1,0) - gt_pose_.center(1,0)) > 20)
                continue;

            std::vector<std::vector<cv::Point2f>> lines = projectTwoPointLine2(pt->point1, pt->point2);
            if(!lines.empty())
            {
                for(auto line = lines.begin(); line!= lines.end(); line++)
                {

                    if(sqrt(((*line)[0].x - (*line)[1].x)*((*line)[0].x-(*line)[1].x) + ((*line)[0].y-(*line)[1].y)*((*line)[0].y-(*line)[1].y)) < 0.5)
                        continue;

                    //-------------------------------------------------------------------
                    perception_rate = rand()/double(RAND_MAX);
                    if(perception_rate <= loss_rate_[4]) continue; //loss
                    //------------------------------------------------------------------

                    line_temp.point1(0,0) = (*line)[0].x;
                    line_temp.point1(1,0) = (*line)[0].y;

                    line_temp.point2(0,0) = (*line)[1].x;
                    line_temp.point2(1,0) = (*line)[1].y;
                    //------------------------------------------------------------------------------------
                    hd_line line_t;
                    Eigen::Vector3d point3 = R*line_temp.point1 + t;
                    Eigen::Vector3d point4 = R*line_temp.point2 + t;
                    line_t.point1(2,0) = 0;  line_t.point1(0,0) = point3(0,0);  line_t.point1(1,0) = point3(1,0); 
                    line_t.point2(2,0) = 0;  line_t.point2(0,0) = point4(0,0);  line_t.point2(1,0) = point4(1,0); 
                    
                    pricise_rate = rand()/double(RAND_MAX);
                    error_class = rand()/double(RAND_MAX);
                    if(pricise_rate <= false_rate_[4][4])   //误检
                    {
                        if(error_class <= false_rate_[4][1])  //误分为lane
                        {
                            line_t.label = LANE;
                            lanes_.push_back(line_t);
                        }
                        else if(error_class <= false_rate_[4][1]+false_rate_[4][2])  //误分为arrow
                        {
                            line_t.label = ARROW;
                            arrows_.push_back(line_t);
                        }
                        else  //误分为zebra
                        {
                            line_t.label = ZEBRA;
                            zebras_.push_back(line_t);
                        }

                    }
                    else
                    {
                        line_t.label = SPEEDBUMP;
                        speedbumps_.push_back(line_t);
                    }
                    //-------------------------------------------------------------------------------------
                }
            }
        }
    }


    bool bev_frame::isInBev(float x, float y)
    {
        Eigen::Matrix3d R = gt_R_.transpose();
        Eigen::Vector3d t = -R*gt_t_;

        Eigen::Vector3d point = R * Eigen::Vector3d(x, y, 0) + t;
        return (isInRect(bev_rect_, point(0, 0), point(1,0)) && !isInRect(car_icon_,  point(0, 0), point(1,0)));
    }

    bool bev_frame::isInRect(float rect[4][3], float x, float y)
    {
        if(x < rect[0][0] && x>rect[2][0] && y <rect[1][1] && y>rect[0][1])
            return true;
        else
            return false;
    }

     void bev_frame::getBevRect()
    {
        float bev_vertex_x1 = BEV_RANGE_LENGTH/2 + CAR_LENGTH/2 - CAR_REAR_TO_CENTER;
        float bev_vertex_x2 = bev_vertex_x1 - BEV_RANGE_LENGTH;
        float bev_vetex_y = BEV_RANGE_WIDTH/2;

        bev_rect_[0][0] = bev_vertex_x1;  bev_rect_[0][1] = -bev_vetex_y;  bev_rect_[0][2] = 0;
        bev_rect_[1][0] = bev_vertex_x1;  bev_rect_[1][1] =   bev_vetex_y;  bev_rect_[1][2] = 0;
        bev_rect_[2][0] = bev_vertex_x2;  bev_rect_[2][1] =   bev_vetex_y;  bev_rect_[2][2] = 0;
        bev_rect_[3][0] = bev_vertex_x2;  bev_rect_[3][1] = -bev_vetex_y;  bev_rect_[3][2] = 0;

        float car_icon_x1 =  CAR_LENGTH - CAR_REAR_TO_CENTER;
        float car_icon_x2 =  -CAR_REAR_TO_CENTER;
        float car_icon_y = CAR_WIDTH/2;

        car_icon_[0][0] = car_icon_x1;  car_icon_[0][1] = -car_icon_y;  car_icon_[0][2] = 0;
        car_icon_[1][0] = car_icon_x1;  car_icon_[1][1] =   car_icon_y;  car_icon_[1][2] = 0;
        car_icon_[2][0] = car_icon_x2;  car_icon_[2][1] =   car_icon_y;  car_icon_[2][2] = 0;
        car_icon_[3][0] = car_icon_x2;  car_icon_[3][1] = -car_icon_y;  car_icon_[3][2] = 0;

    }

    std::vector<std::vector<cv::Point2f>> bev_frame::projectTwoPointLine2(Eigen::Vector3d point1, Eigen::Vector3d point2)
    {
        float low_end_x, low_end_y, high_end_x, high_end_y;
        float k;
        cv::Point2f qury_pt;
        std::vector<cv::Point2f> qury_points;
        std::vector<std::vector<cv::Point2f>> result;

        //按x值小的为低点
        if(point1(0,0) < point2(0,0))
        {
            low_end_x = point1(0,0);   low_end_y = point1(1,0);
            high_end_x = point2(0,0); high_end_y = point2(1,0);
        }
        else
        {
            low_end_x = point2(0,0);   low_end_y = point2(1,0);
            high_end_x = point1(0,0); high_end_y = point1(1,0);
        }

        //将线离散为点
        if(abs(low_end_x-high_end_x)<0.01)
        {
            qury_pt.x = (low_end_x+high_end_x)/2;
            float low_y = low_end_y < high_end_y? low_end_y : high_end_y;
            float high_y = low_end_y > high_end_y? low_end_y : high_end_y;
            for(float y = low_y; y<=high_y; y+=0.05)
            {
                qury_pt.y = y;
                qury_points.push_back(qury_pt);
            }
        }
        else
        {
            k = (high_end_y - low_end_y)/(high_end_x - low_end_x);
            for(float x = low_end_x; x <high_end_x; x+=0.05 )
            {
                qury_pt.x = x;
                qury_pt.y = k*(x - low_end_x) + low_end_y;
                qury_points.push_back(qury_pt);
            }
        }

        //遍历点，找到成像线段端点
        bool last_point_in_bev = false;
        bool start_pt_found = false;
        bool end_pt_found = false;
         cv::Point2f start_pt, end_pt, last_pt;
        for(auto pt = qury_points.begin(); pt!=qury_points.end(); pt++)
        {
            if(isInBev(pt->x, pt->y))
            {
                if(!last_point_in_bev)
                {
                    last_point_in_bev = true;
                    start_pt = *pt;
                    last_pt = *pt;
                    start_pt_found = true;
                }
                else
                {
                    last_pt = *pt;
                    if(pt ==qury_points.end()-1)
                    {
                        end_pt = last_pt;
                        end_pt_found = true;
                    }
                    else
                        continue;
                }
            }
            else
            {
                if(!last_point_in_bev) 
                    continue;
                else
                {
                    last_point_in_bev = false;
                    end_pt = last_pt;
                    end_pt_found = true;
                }
            }

            if(start_pt_found && end_pt_found)
            {
                std::vector<cv::Point2f> line;
                line.push_back(start_pt);
                line.push_back(end_pt);
                result.push_back(line);

                start_pt_found = false;
                end_pt_found = false;
            }

        }

        return result;
    }


    void bev_frame::display(cv::Mat& p)
    {

        //显示车位角点
        cv::Point2d slot_corner;
        for(auto corner = slot_corners_.begin(); corner!=slot_corners_.end(); corner++)
        {
            slot_corner.x = xPercepToPixel(corner->point(0,0));
            slot_corner.y = yPercepToPixel(corner->point(1,0));
            cv::circle(p, slot_corner, 1, cv::Scalar(255, 0, 0), 2);
        }


        cv::Point2d end_p1;
        cv::Point2d end_p2;
        //显示车道线
        for(auto lane = lanes_.begin(); lane!=lanes_.end(); lane++)
        {
            end_p1.x =xPercepToPixel(lane->point1(0,0));
            end_p1.y = yPercepToPixel(lane->point1(1,0));

            end_p2.x = xPercepToPixel(lane->point2(0,0));
            end_p2.y = yPercepToPixel(lane->point2(1,0));

            cv::line(p, end_p1, end_p2, cv::Scalar(0, 180 , 255), 2);
        }

        //显示长白线
        for(auto line = long_white_lines_.begin(); line!=long_white_lines_.end(); line++)
        {
            end_p1.x =xPercepToPixel(line->point1(0,0));
            end_p1.y = yPercepToPixel(line->point1(1,0));

            end_p2.x = xPercepToPixel(line->point2(0,0));
            end_p2.y = yPercepToPixel(line->point2(1,0));

            cv::line(p, end_p1, end_p2, cv::Scalar(255, 0 , 255), 1);
        }

        //显示箭头
        for(auto lane = arrows_.begin(); lane!=arrows_.end(); lane++)
        {
            end_p1.x =xPercepToPixel(lane->point1(0,0));
            end_p1.y = yPercepToPixel(lane->point1(1,0));

            end_p2.x = xPercepToPixel(lane->point2(0,0));
            end_p2.y = yPercepToPixel(lane->point2(1,0));

            cv::line(p, end_p1, end_p2, cv::Scalar(0, 180 , 0), 2);
        }

        //显示斑马线
        for(auto lane = zebras_.begin(); lane!=zebras_.end(); lane++)
        {
            end_p1.x =xPercepToPixel(lane->point1(0,0));
            end_p1.y = yPercepToPixel(lane->point1(1,0));

            end_p2.x = xPercepToPixel(lane->point2(0,0));
            end_p2.y = yPercepToPixel(lane->point2(1,0));

            cv::line(p, end_p1, end_p2, cv::Scalar(0, 0 , 220), 1);
        }

        //显示减速带
        for(auto lane = speedbumps_.begin(); lane!=speedbumps_.end(); lane++)
        {
            end_p1.x =xPercepToPixel(lane->point1(0,0));
            end_p1.y = yPercepToPixel(lane->point1(1,0));

            end_p2.x = xPercepToPixel(lane->point2(0,0));
            end_p2.y = yPercepToPixel(lane->point2(1,0));

            cv::line(p, end_p1, end_p2, cv::Scalar(255, 0 , 220), 3);
        }

        //画bev rect和car icon
        for(int i = 0; i<4; i++)
        {
            int j = (i+1)%4;

            Eigen::Vector3d bev_vertex1(bev_rect_[i][0], bev_rect_[i][1], bev_rect_[i][2]);
            Eigen::Vector3d bev_vertex2(bev_rect_[j][0], bev_rect_[j][1], bev_rect_[j][2]);

            Eigen::Vector3d icon_vertex1(car_icon_[i][0], car_icon_[i][1], car_icon_[i][2]);
            Eigen::Vector3d icon_vertex2(car_icon_[j][0], car_icon_[j][1], car_icon_[j][2]);


            cv::Point2d c_bev_p1(xPercepToPixel(bev_vertex1(0,0)), xPercepToPixel(bev_vertex1(1,0)));
            cv::Point2d c_bev_p2(xPercepToPixel(bev_vertex2(0,0)), xPercepToPixel(bev_vertex2(1,0)));

            cv::Point2d c_icon_p1(xPercepToPixel(icon_vertex1(0,0)), xPercepToPixel(icon_vertex1(1,0)));
            cv::Point2d c_icon_p2(xPercepToPixel(icon_vertex2(0,0)), xPercepToPixel(icon_vertex2(1,0)));

            cv::line(p, c_bev_p1, c_bev_p2, cv::Scalar(0,0,0),1 );
            cv::line(p, c_icon_p1, c_icon_p2, cv::Scalar(0,0,0),1 );
        }

        //displayLocalMap(p);

    }


    void bev_frame::generatePointCloud()
    {
        PointT point;
        double v_x = 0;
        double v_y = 0;
        double line_length = 0;
        int pt_num = 0;
        double min_distance_to_cam = 0;
        double error = 0;
        Eigen::Vector3d cord_noise;

        percep_cloud_->points.clear();

        //slot_corners
        for(auto corner = slot_corners_.begin(); corner!=slot_corners_.end(); corner++)
        {
            min_distance_to_cam = getMinDistanceToCam(corner->point);
            error = ERROR_COEF[0]*min_distance_to_cam*min_distance_to_cam+ERROR_COEF[1]*min_distance_to_cam+ERROR_COEF[2];
            cord_noise = getCordNoise(error);

            point.x = corner->point(0,0) + cord_noise(0,0);
            point.y = corner->point(1,0) + cord_noise(1,0);;
            point.z = -SLOT_CORNER_Z;     //将点从世界坐标系转到车辆坐标系的时候，Z值取负值
            percep_cloud_->points.push_back(point);
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
                min_distance_to_cam = getMinDistanceToCam(Eigen::Vector3d(line->point1(0,0) + i*INTERVAL*v_x,  line->point1(1,0) + i*INTERVAL*v_y, 0));
                error = ERROR_COEF[0]*min_distance_to_cam*min_distance_to_cam+ERROR_COEF[1]*min_distance_to_cam+ERROR_COEF[2];
                cord_noise = getCordNoise(error);

                point.x = line->point1(0,0) + i*INTERVAL*v_x + cord_noise(0,0);
                point.y = line->point1(1,0) + i*INTERVAL*v_y + cord_noise(1,0);
                point.z = -LANE_Z;
                percep_cloud_->points.push_back(point);
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
                min_distance_to_cam = getMinDistanceToCam(Eigen::Vector3d(line->point1(0,0) + i*INTERVAL*v_x,  line->point1(1,0) + i*INTERVAL*v_y, 0));
                error = ERROR_COEF[0]*min_distance_to_cam*min_distance_to_cam+ERROR_COEF[1]*min_distance_to_cam+ERROR_COEF[2];
                cord_noise = getCordNoise(error);

                point.x = line->point1(0,0) + i*INTERVAL*v_x + cord_noise(0,0);
                point.y = line->point1(1,0) + i*INTERVAL*v_y + cord_noise(1,0);
                point.z = -ARROW_Z;
                percep_cloud_->points.push_back(point);
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
                min_distance_to_cam = getMinDistanceToCam(Eigen::Vector3d(line->point1(0,0) + i*INTERVAL*v_x,  line->point1(1,0) + i*INTERVAL*v_y, 0));
                error = ERROR_COEF[0]*min_distance_to_cam*min_distance_to_cam+ERROR_COEF[1]*min_distance_to_cam+ERROR_COEF[2];
                cord_noise = getCordNoise(error);

                point.x = line->point1(0,0) + i*INTERVAL*v_x + cord_noise(0,0);
                point.y = line->point1(1,0) + i*INTERVAL*v_y + cord_noise(1,0);
                point.z = -ZEBRA_Z;
                percep_cloud_->points.push_back(point);
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
                min_distance_to_cam = getMinDistanceToCam(Eigen::Vector3d(line->point1(0,0) + i*INTERVAL*v_x,  line->point1(1,0) + i*INTERVAL*v_y, 0));
                error = ERROR_COEF[0]*min_distance_to_cam*min_distance_to_cam+ERROR_COEF[1]*min_distance_to_cam+ERROR_COEF[2];
                cord_noise = getCordNoise(error);

                point.x = line->point1(0,0) + i*INTERVAL*v_x + cord_noise(0,0);
                point.y = line->point1(1,0) + i*INTERVAL*v_y + cord_noise(1,0);
                point.z = -SPEEDBUMP_Z;
                percep_cloud_->points.push_back(point);
            }
        }

        percep_cloud_->height = 1;
        percep_cloud_->width = percep_cloud_->points.size();
        percep_cloud_->is_dense = true;  

        //stringstream percep_map;
        //percep_map<<"percep.pcd";
        //pcl::io::savePCDFile (percep_map.str (), *percep_cloud_, true); 

    }


    PointCloud::Ptr bev_frame::getCloud()
    { 
         return percep_cloud_ ;
    }

    double bev_frame::getMinDistanceToCam(Eigen::Vector3d point)
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

    Eigen::Vector3d bev_frame::getCordNoise(double error)
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

}// namespace bev_reloca