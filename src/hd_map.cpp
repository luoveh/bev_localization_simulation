#include "hd_map.h"
#include "converter.h"

namespace bev_reloca{

    hd_map::hd_map()
    {
        std::cout<<"g map..."<<endl;
    }


    int hd_map::getMap()
    {
        loadSlotCorners();
        
        loadLanes();

        loadLongWhiteLines();
        
        loadArrows();
        
        loadZebras();
        
        loadSpeedbumps();
        
        loadTrj();

        return 0;
    }

    void hd_map::clear()
    {
        slot_corners_.clear();
        lanes_.clear();
        long_white_lines_.clear();
        arrows_.clear();
        zebras_.clear();
        speedbumps_.clear();
        trjs_.clear();
    }

    int hd_map::loadSlotCorners()
    {
        float x = 0;
        hd_point point_temp;
        point_temp.label = SLOT_CORNER;
        point_temp.point(2,0) = 0;

        point_temp.point(1,0) = -1.8;
        for(int i = 0; i<9; i++)
        {
            point_temp.point(0,0) = -4.784+i*3;
            slot_corners_.push_back(point_temp);
        }

        for(int i = 0; i<7; i++)
        {
            point_temp.point(0,0) = 147.143+i*2.7;
            slot_corners_.push_back(point_temp);
        }

        point_temp.point(0,0) = 205.353;
        for(int i = 0; i< 12; i++)
        {
            point_temp.point(1,0) = -(6.53+i*2.5);
            slot_corners_.push_back(point_temp);
        }

        point_temp.point(0,0) = 205.353;
        for(int i = 0; i< 15; i++)
        {
            point_temp.point(1,0) = -(6.53+i*2.5);
            slot_corners_.push_back(point_temp);
        }


        point_temp.point(0,0) = 212.52;
        for(int i = 0; i< 12; i++)
        {
            point_temp.point(1,0) = -(6.53+i*2.5);
            slot_corners_.push_back(point_temp);
        }

        point_temp.point(1,0) = -41.68;
        for(int i = 0; i< 7; i++)
        {
            point_temp.point(0,0) = (205.503+i*2.5);
            slot_corners_.push_back(point_temp);
        }

        point_temp.point(0,0) = 223.177;
        for(int i = 0; i< 12; i++)
        {
            point_temp.point(1,0) = -(6.53+i*2.5);
            slot_corners_.push_back(point_temp);
        }

        point_temp.point(0,0) = 230.344;
        for(int i = 0; i< 22; i++)
        {
            point_temp.point(1,0) = -(6.53+i*2.5);
            slot_corners_.push_back(point_temp);
        }

        point_temp.point(0,0) = 223.9077;
        for(int i = 0; i< 8; i++)
        {
            point_temp.point(1,0) = -(48.3201+i*2.5);
            slot_corners_.push_back(point_temp);
        }


        point_temp.point(0,0) = 223.9077;
        for(int i = 0; i< 7; i++)
        {
            point_temp.point(1,0) = -(74.7764+i*2.5);
            slot_corners_.push_back(point_temp);
        }


        point_temp.point(0,0) = 228.944;
        for(int i = 0; i< 2; i++)
        {
            point_temp.point(1,0) = -(65.49+i*2.5);
            slot_corners_.push_back(point_temp);
        }

        point_temp.point(0,0) = 228.944;
        for(int i = 0; i< 7; i++)
        {
            point_temp.point(1,0) = -(74.5508+i*2.5);
            slot_corners_.push_back(point_temp);
        }

        point_temp.point(0,0) = 170.776;
        for(int i = 0; i< 17; i++)
        {
            point_temp.point(1,0) = -(13.15+i*2.5);
            slot_corners_.push_back(point_temp);
        }

        point_temp.point(0,0) = 174.447;
        for(int i = 0; i< 18; i++)
        {
            point_temp.point(1,0) = -(13.15+i*2.5);
            slot_corners_.push_back(point_temp);
        }

        point_temp.point(0,0) = 184.047;
        for(int i = 0; i< 18; i++)
        {
            point_temp.point(1,0) = -(13.15+i*2.5);
            slot_corners_.push_back(point_temp);
        }

        point_temp.point(0,0) = 187.747;
        for(int i = 3; i< 18; i++)
        {
            point_temp.point(1,0) = -(13.15+i*2.5);
            slot_corners_.push_back(point_temp);
        }

        point_temp.point(0,0) = 196.462;
        for(int i = 0; i< 12; i++)
        {
            point_temp.point(1,0) = -(45.65-i*2.5);
            slot_corners_.push_back(point_temp);
        }


        

        

        return 0;
    }


    int hd_map::loadLanes()
    {
        hd_line lane_temp;
        lane_temp.label = LANE;
        lane_temp.point1(2,0) = 0;
        lane_temp.point2(2,0) = 0;

        //lane 0
        lane_temp.point1(1,0) = 1.5;
        lane_temp.point2(1,0) = 1.5;
        lane_temp.point1(0,0) = 0;
        lane_temp.point2(0,0) = 3.6;
        lanes_.push_back(lane_temp);

        //lane 1
        lane_temp.point1(0,0) = 8.58;
        lane_temp.point2(0,0) = 12.117;
        lanes_.push_back(lane_temp);

        //lane 2
        lane_temp.point1(0,0) = 17.568;
        lane_temp.point2(0,0) = 21.075;
        lanes_.push_back(lane_temp);

         //lane 3
        lane_temp.point1(0,0) = 26.145;
        lane_temp.point2(0,0) = 29.727;
        lanes_.push_back(lane_temp);

         //lane 4
        lane_temp.point1(0,0) = 35.218;
        lane_temp.point2(0,0) = 38.852;
        lanes_.push_back(lane_temp);

         //lane 5
        lane_temp.point1(0,0) = 43.968;
        lane_temp.point2(0,0) = 47.272;
        lanes_.push_back(lane_temp);

         //lane 6
        lane_temp.point1(0,0) = 52.49;
        lane_temp.point2(0,0) = 55.893;
        lanes_.push_back(lane_temp);

         //lane 7
        lane_temp.point1(0,0) = 61.736;
        lane_temp.point2(0,0) = 65.203;
        lanes_.push_back(lane_temp);

         //lane 8
        lane_temp.point1(0,0) = 70.69;
        lane_temp.point2(0,0) = 74.149;
        lanes_.push_back(lane_temp);

         //lane 9
        lane_temp.point1(0,0) = 79.853;
        lane_temp.point2(0,0) = 83.028;
        lanes_.push_back(lane_temp);

         //lane 10
        lane_temp.point1(0,0) = 88.821;
        lane_temp.point2(0,0) = 92.562;
        lanes_.push_back(lane_temp);

         //lane 11
        lane_temp.point1(0,0) = 98.105;
        lane_temp.point2(0,0) = 101.909;
        lanes_.push_back(lane_temp);

         //lane 12
        lane_temp.point1(0,0) = 107.532;
        lane_temp.point2(0,0) = 111.218;
        lanes_.push_back(lane_temp);

         //lane 13
        lane_temp.point1(0,0) = 116.944;
        lane_temp.point2(0,0) = 120.279;
        lanes_.push_back(lane_temp);

        //lane 14
        lane_temp.point1(0,0) = 125.348;
        lane_temp.point2(0,0) = 128.593;
        lanes_.push_back(lane_temp);

        //lane 15
        lane_temp.point1(0,0) = 133.714;
        lane_temp.point2(0,0) = 136.95;
        lanes_.push_back(lane_temp);

        //lane 16
        lane_temp.point1(0,0) = 142.076;
        lane_temp.point2(0,0) = 145.171;
        lanes_.push_back(lane_temp);

        //lane 17
        lane_temp.point1(0,0) = 150.217;
        lane_temp.point2(0,0) = 153.382;
        lanes_.push_back(lane_temp);

        //lane 18
        lane_temp.point1(0,0) = 158.438;
        lane_temp.point2(0,0) = 161.664;
        lanes_.push_back(lane_temp);


        //lane 19
        lane_temp.point1(0,0) = 185.8;
        lane_temp.point2(0,0) = 189;
        lanes_.push_back(lane_temp);


         //lane 20
        lane_temp.point1(0,0) = 195;
        lane_temp.point2(0,0) = 198.2;
        lanes_.push_back(lane_temp);

         //lane 21
        lane_temp.point1(0,0) = 204.2;
        lane_temp.point2(0,0) = 207.4;
        lanes_.push_back(lane_temp);


        //lane 22
        lane_temp.point1(0,0) = 208.937;
        lane_temp.point2(0,0) = 208.937;
        for(int i = 0; i<5; i++)
        {
            lane_temp.point1(1,0) = -(6.5+i*6.5);
            lane_temp.point2(1,0) = -(9.7+i*6.5);
            lanes_.push_back(lane_temp);
        }


        //lane 23
        lane_temp.point1(0,0) = 226.761;
        lane_temp.point2(0,0) = 226.761;
        for(int i = 0; i<13; i++)
        {
            lane_temp.point1(1,0) = -(6.5+i*6.5);
            lane_temp.point2(1,0) = -(9.7+i*6.5);
            lanes_.push_back(lane_temp);
        }
       
        return 0;

    }


    int hd_map::loadLongWhiteLines()
    {
        hd_line line_temp;
        line_temp.label = LONG_WHITE_LINE;
        line_temp.point1(2,0) = 0;
        line_temp.point2(2,0) = 0;

        line_temp.point1(1,0) = -1.5;
        line_temp.point2(1,0) = -1.5;

        for(int i = 0; i<40; i++)
        {
            line_temp.point1(0,0) = -4.784+i*4;
            line_temp.point2(0,0) = -2+i*4;
            long_white_lines_.push_back(line_temp);
        }


    }


    int hd_map::loadArrows()
    {
        hd_line arrow_temp;
        arrow_temp.label = ARROW;

        arrow_temp.point1(2,0) = 0;
        arrow_temp.point2(2,0) = 0;

        // 0
        arrow_temp.point1(1,0) = -2.5;
        arrow_temp.point2(1,0) = -2.5;

        arrow_temp.point1(0,0) = 95;
        arrow_temp.point2(0,0) = 98;

        arrows_.push_back(arrow_temp);


        //1
        float R1 = 21.125;  //m
        float R2 = 4;
        float alpha = atan2(9+R2, R1);
        arrow_temp.point1(0,0) = 162+R1*sin(alpha/2);
        arrow_temp.point1(1,0) = -(-R1*cos(alpha/2)+R1+1);

        arrow_temp.point2(0,0) = 162+R1*sin(alpha);
        arrow_temp.point2(1,0) = -(-R1*cos(alpha)+R1+1);
        arrows_.push_back(arrow_temp);

        //2
        arrow_temp.point1(0,0) = 190+R1*sin(-alpha);
        arrow_temp.point1(1,0) = -(-R1*cos(-alpha)+R1+1);

        arrow_temp.point2(0,0) = 190+R1*sin(-alpha/2);
        arrow_temp.point2(1,0) = -(-R1*cos(-alpha/2)+R1+1);
        arrows_.push_back(arrow_temp);


        //3
        arrow_temp.point1(0,0) = 197.855;
        arrow_temp.point1(1,0) = -1.5;

        arrow_temp.point2(0,0) = 200.855;
        arrow_temp.point2(1,0) = -1.5;
        arrows_.push_back(arrow_temp);


        //4
        arrow_temp.point1(0,0) = 210.52;
        arrow_temp.point1(1,0) = -6.5;

        arrow_temp.point2(0,0) = 210.52;
        arrow_temp.point2(1,0) = -9.5;
        arrows_.push_back(arrow_temp);

        return 0;
    }

    int hd_map::loadZebras()
    {
        hd_line zebra_temp;
        zebra_temp.label = ZEBRA;

        zebra_temp.point1(2,0) = 0;
        zebra_temp.point2(2,0) = 0;

        //zebra 0
        zebra_temp.point1(0,0) = 57.8;
        zebra_temp.point2(0,0) = 59.591;
        for(int i = 0; i < 10; i++)
        {
            zebra_temp.point1(1,0) = -(0.876 - i * 0.572);
            zebra_temp.point2(1,0) = -(0.876 - i * 0.572);
            zebras_.push_back(zebra_temp);
        }

        //zebra 1
        zebra_temp.point1(0,0) = 85.085;
        zebra_temp.point2(0,0) = 86.602;
        for(int i = 0; i < 13; i++)
        {
            zebra_temp.point1(1,0) = -(-1.459 - i * 0.58);
            zebra_temp.point2(1,0) = -(-1.459 - i * 0.58);
            zebras_.push_back(zebra_temp);
        }
        for(int i = 1; i < 17; i++)
        {
            zebra_temp.point1(1,0) = -(-1.459 + i * 0.58);
            zebra_temp.point2(1,0) = -(-1.459 + i * 0.58);
            zebras_.push_back(zebra_temp);
        }

        //zebra 2
        zebra_temp.point1(0,0) = 113.461;
        zebra_temp.point2(0,0) = 115.002;
        for(int i = 0; i<12; i++)
        {
            zebra_temp.point1(1,0) = -(1.346 - i * 0.615);
            zebra_temp.point2(1,0) = -(1.346 - i * 0.615);
            zebras_.push_back(zebra_temp);
        }

        //zebra 3
        zebra_temp.point1(0,0) = 138.7;
        zebra_temp.point2(0,0) = 140.29;
        for(int i = 0; i<20; i++)
        {
            zebra_temp.point1(1,0) = -(-5.265 + i * 0.603);
            zebra_temp.point2(1,0) = -(-5.265 + i * 0.603);
            zebras_.push_back(zebra_temp);
        }

        //zebra 4
        zebra_temp.point1(0,0) = 191.682;
        zebra_temp.point2(0,0) = 193.166;
        for(int i = 0; i<8; i++)
        {
            //zebra_temp.point1(1,0) = 0.448 + i * 0.603;
            zebra_temp.point1(1,0) = -(-0.758 + i * 0.603);
            zebra_temp.point2(1,0) = -(-0.758 + i * 0.603);
            zebras_.push_back(zebra_temp);
        }
        for(int i = 1; i<8; i++)
        {
            zebra_temp.point1(1,0) =  -(-0.758 - i * 0.603);
            zebra_temp.point2(1,0) =  -(-0.758 - i * 0.603);
            zebras_.push_back(zebra_temp);
        }

         //zebra 5
        zebra_temp.point1(1,0) = -13.587;
        zebra_temp.point2(1,0) = -15.138;
        for(int i = 0; i<8; i++)
        {
            zebra_temp.point1(0,0) =210.62 - i * 0.644;
            zebra_temp.point2(0,0) = 210.62 - i * 0.644;
            zebras_.push_back(zebra_temp);
        }

        zebra_temp.point1(1,0) = -55.65;
        zebra_temp.point2(1,0) = -57.25;
        for(int i = 0; i<7; i++)
        {
            zebra_temp.point1(0,0) =174.187 - i * 0.52;
            zebra_temp.point2(0,0) = 174.187 - i * 0.52;
            zebras_.push_back(zebra_temp);
        }

        zebra_temp.point1(1,0) = -55.65;
        zebra_temp.point2(1,0) = -57.25;
        for(int i = 0; i<8; i++)
        {
            zebra_temp.point1(0,0) =193.02 + i * 0.52;
            zebra_temp.point2(0,0) = 193.02 + i * 0.52;
            zebras_.push_back(zebra_temp);
        }

        zebra_temp.point1(0,0) = 223.177;
        zebra_temp.point2(0,0) = 224.677;
        for(int i = 0; i<16; i++)
        {
            zebra_temp.point1(1,0) = -(37.48 + i * 0.52);
            zebra_temp.point2(1,0) = -(37.48 + i * 0.52);
            zebras_.push_back(zebra_temp);
        }

        zebra_temp.point1(0,0) = 212.52;
        zebra_temp.point2(0,0) = 214.02;
        for(int i = 1; i<13; i++)
        {
            zebra_temp.point1(1,0) = (-3.463+ i * 0.603);
            zebra_temp.point2(1,0) = (-3.463 + i * 0.603);
            zebras_.push_back(zebra_temp);
        }


        return 0;

    }


    int hd_map::loadSpeedbumps()
    {
        hd_line sp_temp;
        sp_temp.label = SPEEDBUMP;
        sp_temp.point1(2,0) = 0;
        sp_temp.point2(2,0) = 0;

        sp_temp.point1(0,0) = 36.5;  sp_temp.point1(1,0) = 3.7;
        sp_temp.point2(0,0) = 36.5;  sp_temp.point2(1,0) = -3.7;
        //speedbumps_.push_back(sp_temp);

        sp_temp.point1(0,0) = 50.3;  sp_temp.point1(1,0) = 17.5;
        sp_temp.point2(0,0) = 57.7;  sp_temp.point2(1,0) = 17.5;
        //speedbumps_.push_back(sp_temp);

        sp_temp.point1(0,0) = 50.3;  sp_temp.point1(1,0) = 52.5;
        sp_temp.point2(0,0) = 57.7;  sp_temp.point2(1,0) = 52.5;
        //speedbumps_.push_back(sp_temp);

        return 0;
    }


    int hd_map::loadTrj()
    {
        b_pose pose_temp;
        pose_temp.center(2,0) = 0;
        pose_temp.eular(0,0) = 0;
        pose_temp.eular(1,0) = 0;
        pose_temp.eular(2,0) = 0;

        //第一段直行
        for(float x = 0; x<162; x+=0.5)
        {
            pose_temp.center(0,0) = x+0.0*(rand()/double(RAND_MAX));      //每次运行产生固定随机噪声
            pose_temp.center(1,0) = 0.0*(rand()/double(RAND_MAX));

            /* pose_temp.center(0,0) = x;      //
            pose_temp.center(1,0) = -2; */
            trjs_.push_back(pose_temp);
        }

        float R1 = 21.125;  //m
        float R2 = 4;
        float alpha = atan2(9+R2, R1);
        //第一段圆弧
        for(float theta = 0; theta < alpha; theta+=0.0375)
        {
            pose_temp.center(0,0) = R1*sin(theta)+162+0.0*(rand()/double(RAND_MAX));
            pose_temp.center(1,0) = -(-R1*cos(theta)+R1+0.0*(rand()/double(RAND_MAX)));
            trjs_.push_back(pose_temp);
        }

        //第二段圆弧
        for(float theta = -alpha; theta<alpha; theta+=0.0375)
        {
            pose_temp.center(0,0) = R2*sin(theta)+176+0.0*(rand()/double(RAND_MAX));
            pose_temp.center(1,0) = -(R2*cos(theta)+0.0*(rand()/double(RAND_MAX)));
            trjs_.push_back(pose_temp);
        }

        //第三段圆弧
        for(float theta = alpha; theta>0; theta-=0.0375)
        {
            pose_temp.center(0,0) = -R1*sin(theta)+190+0.0*(rand()/double(RAND_MAX));
            pose_temp.center(1,0) = -(-R1*cos(theta)+R1+0.0*(rand()/double(RAND_MAX)));
            trjs_.push_back(pose_temp);
        }

        //第二段直行
        float R3 = 6.29;
        pose_temp.center(1,0) = -0.0*(rand()/double(RAND_MAX));
        for(float x = 190; x<(207.145-R3); x+=0.5)
        {
            pose_temp.center(0,0) = x+0.0*(rand()/double(RAND_MAX));
            trjs_.push_back(pose_temp);
        }

        //第四段圆弧
        for(float theta = 0; theta < PI/2; theta+=0.0375)
        {
            pose_temp.center(0,0) = R3*sin(theta)+(207.145-R3)+0.0*(rand()/double(RAND_MAX));
            pose_temp.center(1,0) = -(-R3*cos(theta)+R3+0.0*(rand()/double(RAND_MAX)));
            trjs_.push_back(pose_temp);
        }

        //第三段直行
        double R5 = 5.375; //相连的下一段圆弧的半径
        pose_temp.center(0,0) =207.145+ 0.0*(rand()/double(RAND_MAX));
        for(float y = R3; y<39.58-R5; y+=0.5)
        {
            pose_temp.center(1,0) = -(y+0.0*(rand()/double(RAND_MAX)));
            trjs_.push_back(pose_temp);
        }


        //第5段圆弧
        for(float theta = 0; theta < PI/2; theta+=0.0375)
        {
            pose_temp.center(0,0) = R5*(1-cos(theta))+(207.145)+0.0*(rand()/double(RAND_MAX));
            pose_temp.center(1,0) = -(39.58-R5+R5*sin(theta)+0.0*(rand()/double(RAND_MAX)));
            trjs_.push_back(pose_temp);
        }

        //第4段直行
        double R6 = 5.375; //相连的下一段圆弧的半径
        pose_temp.center(1,0) =-(39.58+0.0*(rand()/double(RAND_MAX)));
        for(float x = 207.145+R5; x<225.9077-R6; x+=0.5)
        {
            pose_temp.center(0,0) = (x+0.0*(rand()/double(RAND_MAX)));
            trjs_.push_back(pose_temp);
        }

        //第6段圆弧
        for(float theta = 0; theta < PI/2; theta+=0.0375)
        {
            pose_temp.center(0,0) = R6*(sin(theta))+(225.9077-R6)+0.0*(rand()/double(RAND_MAX));
            pose_temp.center(1,0) = -(39.58+R6*(1-cos(theta))+0.0*(rand()/double(RAND_MAX)));
            trjs_.push_back(pose_temp);
        }

        //第5段直行
        double R7 = 5.375; //相连的下一段圆弧的半径
        pose_temp.center(0,0) =(225.9077+0.0*(rand()/double(RAND_MAX)));
        for(float y = 39.58+R6; y<39.58+R6+50; y+=0.5)
        {
            pose_temp.center(1,0) = -(y+0.0*(rand()/double(RAND_MAX)));
            trjs_.push_back(pose_temp);
        }


        float k = 0;
        float theta = 0;
        Eigen::Matrix3d R_initial;
        R_initial << 1, 0, 0, 0, -1, 0, 0, 0, -1;
        Eigen::Vector3d t_initial;
        t_initial << 0, 0, 0;

        double eular[3];

        for(vector<b_pose>::iterator pt = trjs_.begin(); pt!=trjs_.end(); pt++)
        {
            if(pt == trjs_.end()-1)
            {
                pt->eular(0,0) =  eular[0];
                pt->eular(1,0) =  eular[1];
                pt->eular(2,0) =  eular[2];
                break;
            }
            pose_temp = *pt;
            b_pose pose_temp2 = *(pt+1);

            if(pose_temp2.center(0,0)-pose_temp.center(0,0) ==0)
            {
                if(pose_temp2.center(1,0)-pose_temp.center(1,0) ==0)
                    theta = 0;
                else
                {
                    if(pose_temp2.center(1,0)-pose_temp.center(1,0)<0)
                        theta = -PI/2;
                    else
                        theta = PI/2;
                }
            }
            else{
                theta = atan2(pose_temp2.center(1,0)-pose_temp.center(1,0), pose_temp2.center(0,0)-pose_temp.center(0,0));
            }

            Matrix3d R;
            R << cos(theta), sin(theta), 0, 
                    -sin(theta), cos(theta), 0,  
                    0, 0, 1;

            Vector3d t;
            t << pose_temp.center(0,0) ,  -pose_temp.center(1,0)-2, 0;

            Matrix3d R_c2w = R_initial*R;
            Vector3d t_c2w = R_initial*t + t_initial;

            rotToAngle(R_c2w, eular);

            pt->eular(0,0) =  eular[0];
            pt->eular(1,0) =  eular[1];
            pt->eular(2,0) =  eular[2];
        }

    }



    void hd_map::display(cv::Mat p)
    {
        //显示车位角点
        cv::Point2d slot_corner;
        for(auto corner = slot_corners_.begin(); corner!=slot_corners_.end(); corner++)
        {
            slot_corner.x = ToPixel(corner->point(0,0));
            slot_corner.y = ToPixel(-corner->point(1,0));
            cv::circle(p, slot_corner, 1, cv::Scalar(255, 0, 0), 2);
        }


        cv::Point2d end_p1;
        cv::Point2d end_p2;
        //显示车道线
        for(auto lane = lanes_.begin(); lane!=lanes_.end(); lane++)
        {
            end_p1.x =ToPixel(lane->point1(0,0));
            end_p1.y = ToPixel(-lane->point1(1,0));

            end_p2.x = ToPixel(lane->point2(0,0));
            end_p2.y = ToPixel(-lane->point2(1,0));

            cv::line(p, end_p1, end_p2, cv::Scalar(0, 180 , 255), 2);
        }


        //显示长白线
        for(auto line = long_white_lines_.begin(); line!=long_white_lines_.end(); line++)
        {
            end_p1.x =ToPixel(line->point1(0,0));
            end_p1.y = ToPixel(-line->point1(1,0));

            end_p2.x = ToPixel(line->point2(0,0));
            end_p2.y = ToPixel(-line->point2(1,0));

            cv::line(p, end_p1, end_p2, cv::Scalar(255, 0 , 255), 1);
        }

        //显示箭头
        for(auto lane = arrows_.begin(); lane!=arrows_.end(); lane++)
        {
            end_p1.x =ToPixel(lane->point1(0,0));
            end_p1.y = ToPixel(-lane->point1(1,0));

            end_p2.x = ToPixel(lane->point2(0,0));
            end_p2.y = ToPixel(-lane->point2(1,0));

            cv::line(p, end_p1, end_p2, cv::Scalar(0, 180 , 0), 2);
        }

        //显示斑马线
        for(auto lane = zebras_.begin(); lane!=zebras_.end(); lane++)
        {
            end_p1.x =ToPixel(lane->point1(0,0));
            end_p1.y = ToPixel(-lane->point1(1,0));

            end_p2.x = ToPixel(lane->point2(0,0));
            end_p2.y = ToPixel(-lane->point2(1,0));

            cv::line(p, end_p1, end_p2, cv::Scalar(0, 0 , 220), 1);
        }

        //显示减速带
        for(auto lane = speedbumps_.begin(); lane!=speedbumps_.end(); lane++)
        {
            end_p1.x =ToPixel(lane->point1(0,0));
            end_p1.y = ToPixel(-lane->point1(1,0));

            end_p2.x = ToPixel(lane->point2(0,0));
            end_p2.y = ToPixel(-lane->point2(1,0));

            cv::line(p, end_p1, end_p2, cv::Scalar(255, 0 , 220), 3);
        }

        //显示路径
        cv::Point2d trj;
        for(auto corner = trjs_.begin(); corner!=trjs_.end(); corner++)
        {
            trj.x = ToPixel(corner->center(0,0));
            trj.y = ToPixel(-corner->center(1,0));
            cv::circle(p, trj, 1, cv::Scalar(255, 255, 0), 1);
        }

    }



}//namespace bev_reloca

