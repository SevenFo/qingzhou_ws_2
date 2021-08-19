#include <cqu_layer/cqu_layer.h>
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(cqu_costmap::CQULayer, costmap_2d::Layer)
namespace cqu_costmap
{
CQULayer::CQULayer()
{

}
CQULayer::~CQULayer(){}

void CQULayer::onInitialize()
{
    ros::NodeHandle nh("~/" + name_), g_nh;
    current_ = true;


    origin_x_ = 0;
    origin_y_ = 0;
    resolution_ = 50;

    global_frame_ = layered_costmap_->getGlobalFrameID();

    std::string map_topic;
    nh.param("map_topic", map_topic, std::string("map"));
    nh.param("first_map_only", first_map_only_, false);
    nh.param("subscribe_to_updates", subscribe_to_updates_, false);
    nh.param("start_point_x", startpointx, 0.0f);
    nh.param("end_point_x", endpointx, 0.0f);
    nh.param("start_point_y", startpointy, 0.0f);
    nh.param("end_point_y", endpointy, 0.0f);
    nh.param("track_unknown_space", track_unknown_space_, true);
    nh.param("use_maximum", use_maximum_, false);

    int temp_lethal_threshold, temp_unknown_cost_value;
    nh.param("lethal_cost_threshold", temp_lethal_threshold, int(100));
    nh.param("unknown_cost_value", temp_unknown_cost_value, int(-1));
    nh.param("trinary_costmap", trinary_costmap_, true);

    lethal_threshold_ = std::max(std::min(temp_lethal_threshold, 100), 0);
    unknown_cost_value_ = temp_unknown_cost_value;
    ROS_WARN("cqu layer inited");
}
void CQULayer::matchSize()
{
    Costmap2D* master = layered_costmap_->getCostmap();
    resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
              master->getOriginX(), master->getOriginY());
}
void CQULayer::activate()
{
  onInitialize();
}
//更新 要更新的范围
void CQULayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                                        double* max_x, double* max_y)
{

    ROS_INFO_NAMED("cqu_costmap", "CQUCOSTMAP:costmap call me to update bounds, but I dont");
    ROS_INFO_STREAM("[update bounds] robot_x:" << robot_x << " robot_y:" << robot_y << " robot_yaw:" << robot_yaw << " min_x:" << *min_x << " mixy:" << *min_y << " max_x:" << *max_x << " max_y:" << *max_y);
    return;
}
void CQULayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
    ROS_INFO_STREAM_NAMED("cqu_costmap", "CQUCOSTMAP: UPDATE COSTS min_i:"<<min_i<<" min_j:"<<min_j<<" maxi:"<<max_i<<" maxj:"<<max_j);
    unsigned int mapstartx, mapstarty,mapendx,mapendy;
    this->worldToMap(startpointx, startpointy,mapstartx, mapstarty);
    this->worldToMap(endpointx, endpointy,mapendx, mapendy);
    ROS_INFO_STREAM("WORLD TO MAP TEST world x:" << startpointx << " >> " << mapstartx << " y" << startpointy << " >> " << mapstarty);
    // ROS_INFO_STREAM_NAMED("cqu_costmap", "CQUCOSTMAP: UPDATE COSTS map_start_x:" << mapstartx << " map_start_y:" << mapstarty << " map_end_x:" << mapendx << " map_end_y:" << mapendy);
    // CalculateLine(mapstartx, mapstarty, mapendx, mapendy, master_grid);
    
    return;
}
void CQULayer::reset()
{
    onInitialize();
}

void CQULayer::CalculateLine(const int &map_start_point_x, const int &map_start_point_y, const int &map_end_point_x, const int &map_end_point_y, costmap_2d::Costmap2D &master_grid)
{
    double k = (map_end_point_y-map_start_point_y)/(map_start_point_y-map_start_point_x);
    double b = map_start_point_y - (k * map_start_point_x);
    for (int i = map_start_point_x; i < map_end_point_x;i++)
    {
        int index = this->getIndex(i, k * i + b);
        master_grid.setCost(i, k * i + b, costmap_2d::LETHAL_OBSTACLE);
    }
}
}
