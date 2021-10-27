#ifndef HD_MAP_H_
#define HD_MAP_H_

#include "bev_reloca_config.h"
#include "reuse.h"

using namespace std;
using namespace Eigen;

namespace bev_reloca{

    class hd_map
    {
        public:
            hd_map();
            int getMap();
            int loadSlotCorners();
            int loadLanes();
            int loadLongWhiteLines();
            int loadArrows();
            int loadZebras();
            int loadSpeedbumps();
            int loadTrj();
            void display(cv::Mat p);
            void clear();

        //private:
            vector<hd_point> slot_corners_;
            vector<hd_line> lanes_;
            vector<hd_line> long_white_lines_;
            vector<hd_line> arrows_;
            vector<hd_line> zebras_;
            vector<hd_line> speedbumps_;
            vector<b_pose> trjs_;
    };

}//namespace bev_reloca
#endif //#ifndef HD_MAP_H_