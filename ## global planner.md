## global planner
### global planner core
1. `void initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id);`
   1. 初始化规划器，获取一些东西
   2. name:规划器的名字
   3. costmap代价地图
   4. frame_id 地图的参考坐标系

   5. 从代价地图获取cell的x值和y值，cell的大小
   6. xxx
2. `makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, double tolerance,std::vector<geometry_msgs::PoseStamped>& plan);`
   1. 制定路径
   2. start goal 起始点和目标点
   3. tolerance goal的容忍度
   4. plan数组

   5. 获得坐标的起始点，
   6. 清除机器人所在地方的代价值
   7. 通过planner（A start Dstart）计算Potentials
   8. 通过Potential计算路径，grid或者gredient
 
3. `computePotential(const geometry_msgs::Point& world_point);`
   1. 从某个点开始计算路径的势
4. `bool getPlanFromPotential(double start_x, double start_y, double end_x, double end_y,const geometry_msgs::PoseStamped& goal,std::vector<geometry_msgs::PoseStamped>& plan);`
   1. 
