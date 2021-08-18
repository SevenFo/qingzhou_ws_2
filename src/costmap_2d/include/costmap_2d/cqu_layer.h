#ifndef CQU_COSTMAP_LAYER_H
#define CQU_COSTMAP_LAYER_H



#include <ros/ros.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <message_filters/subscriber.h>


namespace costmap_2d
{
    class CQULayer: public CostmapLayer
    {
        public:
            CQULayer();
            virtual ~CQULayer();
            virtual void onInitialize();
            virtual void activate();
            virtual void deactivate();
            virtual void reset();

            virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                                        double* max_x, double* max_y);
            virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

            virtual void matchSize();
        
        protected:
            float startpointx, endpointx,startpointy,endpointy;
            std::string global_frame_;
            bool first_map_only_,subscribe_to_updates_,track_unknown_space_,use_maximum_,trinary_costmap_;
            int lethal_threshold_, unknown_cost_value_;

            void CalculateLine(const int &map_start_point_x, const int &map_start_point_y, const int &map_end_point_x, const int &map_end_point_y, costmap_2d::Costmap2D &master_grid);
    };
}

#endif