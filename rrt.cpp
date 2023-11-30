//header 파일에서 메시지 타입 또는 패키지 가져오기
#include "rrt/rrt.h" 
#include <rrt/occupancy_grid.h> 
#include <rrt/CSVReader.h>
#include <visualization_msgs/Marker.h> 
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <rrt/spline.h>
#include <tf/transform_listener.h>
using namespace std;


//파라미터
const float MARGIN = 0.18;
const float DETECTED_OBS_MARGIN = 0.2;
const int MAX_ITER = 1200;
const int MIN_ITER = 1000;	
const double X_SAMPLE_RANGE = 3;
const double Y_SAMPLE_RANGE = 3;
const double STD = 1.5;   // standard deviation for normal distribution
const double GOAL_THRESHOLD = 0.15;
const double STEER_RANGE = 0.3;
const float SCAN_RANGE = 3.0; //3m을 스캔하도록 파라미터 설정
const float GOAL_AHEAD_DIST = 2.5; //3.5; //
const float LOOK_AHEAD_DIST = 2.0;
const float P_GAIN = 0.3;
const float SPEED = 2.7;
const float NEAR_RANGE = 1.0;


// waypoint의 csv파일 저장
const string file_name = "/home/cstl/f1tenth_ws/src/rrt/csv/sim_draw_case1.csv"; 


// 사용하는 토픽 이름 지정
const string pose_topic = "/pf/pose/odom"; //실제 작동시에 particle filter안의 odom 토픽 사용함.
const string scan_topic = "/scan";
const string drive_topic = "/nav";


// 번외 
//const string pose_topic = "/odom"; //시뮬레이션 상의 odom
//const string pose_topic = "vesc/odom";
//const string drive_topic = "/vesc/high_level/ackermann_cmd_mux/input/nav_1";


RRT::~RRT() {
    ROS_INFO("RRT shutting down");
}
RRT::RRT(ros::NodeHandle &nh): nh_(nh), gen_((std::random_device())()) {
    // ROS subscribers
    odom_sub_ = nh_.subscribe(pose_topic, 10, &RRT::odom_callback, this);
    scan_sub_ = nh_.subscribe(scan_topic, 10, &RRT::scan_callback, this);
    // ROS publishers
    path1_ = nh_.advertise<nav_msgs::Path>("path1", 1);
    waypath_ = nh_.advertise<nav_msgs::Path>("waypath", 1);//eun
    path_pub_ = nh_.advertise<visualization_msgs::Marker>("path_found", 1);
    map_update_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("map_updated", 5);
    drive_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 1);
    obstacle_viz_pub_ = nh_.advertise<visualization_msgs::Marker>("obstacles_inflated", 1);
    tree_viz_pub_ = nh_.advertise<visualization_msgs::Marker>("rrt_tree", 1);
    pos_sp_viz_pub_ = nh_.advertise<visualization_msgs::Marker>("pos_sp_", 1);
    goal_viz_pub_ = nh_.advertise<visualization_msgs::Marker>("goal", 1);
    tree_nodes_pub_ = nh_.advertise<visualization_msgs::Marker>("tree_nodes", 1);
    tree_branches_pub_ = nh_.advertise<visualization_msgs::Marker>("tree_branches", 1);
    // 기존 rrt 샘플점을 구할 때 사용하였으나, 현재는 사용하지 않음
    std::uniform_real_distribution<double> unit_dist(-1.0,1.0);
    uni_dist_ = unit_dist;
    //솔직히 필요없음
    std_msgs::ColorRGBA red; red.r =1.0; red.a=1.0;
    std_msgs::ColorRGBA green; green.g =1.0; green.a=1.0;
    std_msgs::ColorRGBA blue; blue.b =1.0; blue.a=1.0;
    //rviz상에서 goal의 지점을 초록색으로 고정하여 표현하고싶어 쓴 코드(없어도 됨)
    pos_sp_viz = new MarkerVisualizer(pos_sp_viz_pub_, "pos_sp", "laser", red, 0.2, visualization_msgs::Marker::SPHERE);
    goal_viz = new MarkerVisualizer(goal_viz_pub_, "goal", "map", green, 0.3, visualization_msgs::Marker::SPHERE);
    // rrt tree와 branch 시각화 
    tree_nodes.header.frame_id = tree_branch.header.frame_id = "map"; //tree 노드와 branch의 좌표 프레임을 map으로 설정. 
    tree_nodes.ns = "nodes"; tree_branch.ns = "branch"; //시각화 메시지의 네임스페이스를 node, branch로 설정함. 메시지 그룹 구별함.
    tree_nodes.action = tree_branch.action = visualization_msgs::Marker::ADD; // 시각화 메시지의 액션을 추가 할수 있는 경우를 대비해서 만듦
    tree_nodes.pose.orientation.w = tree_branch.pose.orientation.w = 1.0;
    tree_nodes.id = 5; tree_branch.id = 6; //메시지의 고유id 를 각각 5,6으로 설정
    tree_nodes.type = visualization_msgs::Marker::POINTS; //node의 메시지 유형을 point로 설정
    tree_branch.type = visualization_msgs::Marker::LINE_LIST;//branch의 메시지 유형을 line_list로 설정
    tree_nodes.scale.x = tree_nodes.scale.y = tree_nodes.scale.z = 0.05; //node의 x,y,z스케일을 0.05
    tree_branch.scale.x = 0.01; //branch의 x 스케일 0.01
    tree_nodes.color = red; tree_branch.color = blue; //색깔 
    // rrt 코드 진행 순서
    init_occupancy_grid();
    visualize_map();
    load_waypoints(file_name);
    reset_goal();
    ROS_INFO("Created new RRT Object.");
    last_time_ = ros::Time::now().toSec();
    // 파라미터
    nh_.param<float>("/rrt_node/SPEED", SPEED, 5.0);
    nh_.param<float>("/rrt_node/MARGIN", MARGIN, 0.25);
    nh_.param<float>("/rrt_node/DETECTED_OBS_MARGIN",  DETECTED_OBS_MARGIN, 0.2);
    nh_.param<float>("/rrt_node/P_GAIN", P_GAIN, 0.3);
    nh_.param<float>("/rrt_node/SCAN_RANGE", SCAN_RANGE, 3.5);
    nh_.param<float>("/rrt_node/LOOK_AHEAD_DIST", LOOK_AHEAD_DIST, 0.5);
    nh_.param<float>("/rrt_node/MAX_DECELARATION", MAX_DECELARATION, 0.7*SPEED);
}


//////////////////////////////////////////////////////// 함수 정의//////////////////////////////////////////////////////


// map 데이터를 초기화하고 확장된 map을 사용가능한 상태로 설정// 해당 map정보는 경로 계획, 장애물 회피와 같은 작업할때 사용할거임
void RRT::init_occupancy_grid(){
    boost::shared_ptr<nav_msgs::OccupancyGrid const> map_ptr;
    map_ptr = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("map", ros::Duration(5.0)); // map 토픽에서 5초동안 지도 정보, 경로 계획 및 시각화에 사용되는 메시지를 받음
    if (map_ptr == nullptr){ROS_INFO("No map received");} //메시지가 정상적으로 수신되지않으면 no map received로 출력
    else{
        map_ = *map_ptr;
        map_updated_ = map_; //map update
        ROS_INFO("Map received");
    }
    ROS_INFO("Initializing occupancy grid for map ...");  //map의 grid 정보를 초기화한다고 알림
    occupancy_grid::inflate_map(map_, MARGIN); // "occupancy_grid" 라이브러리를 사용하여 지도 정보를 확장함. MARGIN은 확장할 여백을 지정하는 파라미터
    //이로써 장애물 주변의 안전한 경로를 계획히기 위한 작업이 수행됨. 
    ROS_INFO("occupancy_grid_OK");
}

// map에 있는 장애물 시각화
void RRT::visualize_map(){
    visualization_msgs::Marker dots;
    dots.header.stamp = ros::Time::now();
    dots.header.frame_id = "map"; //지도상에서 시각화 할것이다~
    dots.id = 0;
    dots.ns = "obstacle"; //name space을 "장애물"로 하겠다~
    dots.type = visualization_msgs::Marker::POINTS;
    dots.scale.x = dots.scale.y = 0.04; //점의 크기를 0.04로 하겠다.
    dots.scale.z = 0.04;
    dots.action = visualization_msgs::Marker::ADD;
    dots.pose.orientation.w = 1.0; //점의 방향성을 나타내는 퀴터니언 w를 1.0으로 설정
    dots.color.r = 1.0; //점색깔 빨간색
    dots.color.a = 1.0; //불투명도 1.0
    dots.lifetime = ros::Duration();
    for (int i=0; i<map_.data.size(); i++){
        if (int(map_.data.at(i))==100){
            geometry_msgs::Point p;
            p.x = occupancy_grid::ind2x(map_, i);
            p.y = occupancy_grid::ind2y(map_, i);
            p.z = 0.0;
            dots.points.push_back(p);
        }
    }
    obstacle_viz_pub_.publish(dots);
    ROS_INFO("visualize_map_OK");
}


// csv 파일로 저장된 waypoint을 가져오는 함수
void RRT::load_waypoints(std::string file_name){
    waypoints_.clear(); //초기화
    CSVReader reader(file_name); //csv파일 읽기 프로그램
    // Get the data from CSV File
    std::vector<std::vector<std::string> > dataList = reader.getData(); //vector형태로 csv 데이터읽기
    // Print the content of row by row on screen
    for(std::vector<std::string> vec : dataList){
        geometry_msgs::Point wp; //wp 변수에 x,y,z 정의
        wp.x = std::stof(vec.at(0)); 
        wp.y = std::stof(vec.at(1));
        wp.z = std::stof(vec.at(3)); // eunji 
        waypoints_.push_back(wp); //다른 함수에서도 변수 wp사용 할 수있도록 push_back
    }
    ROS_INFO("load_waypoints_OK");
}

//레이저 스캔 데이터를 처리하고 map을 업데이트하는 역할
void RRT::scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg) {
    using namespace occupancy_grid;
    //ROS_INFO("scan_callback_s");
    tf::StampedTransform tf_stamped;
    listener_.lookupTransform("/map", "/laser", ros::Time(0), tf_stamped);
    tf_.setOrigin(tf_stamped.getOrigin());
    tf_.setRotation(tf_stamped.getRotation());

    //only reset map when the car has made enough progress
//    if(ros::Time::now().toSec() - last_time_ > 5.0){
//        map_updated_ = map_; // might be expensive to copy
//        last_time_ = ros::Time::now().toSec();
//    }
    map_updated_ = map_; // might be expensive to copy
    float angle_min = msg->angle_min;
    float angle_increment = msg->angle_increment;

    for (int i = 0; i < msg->ranges.size(); ++i) {
        float range = msg->ranges.at(i);
        if (range > SCAN_RANGE) {
            continue;
        }
        if (!isnan(range) && !isinf(range)) {
            float angle = angle_min + angle_increment * i;
            tf::Vector3 pos_in_car(range*cos(angle), range*sin(angle), 0.0);
            tf::Vector3 pos_in_map = tf_ * pos_in_car;
            if (!is_xy_occupied(map_, pos_in_map.x(), pos_in_map.y())){
                inflate_cell(map_updated_, xy2ind(map_updated_, pos_in_map.x(), pos_in_map.y()), DETECTED_OBS_MARGIN, 100);
            }
        }
    }
    // free the cells in which the car occupies (dynamic layer)
    inflate_cell(map_updated_, xy2ind(map_updated_, car_pose_.position.x, car_pose_.position.y), 0.25, 0);
    map_update_pub_.publish(map_updated_);
    //ROS_INFO("scan_callback_OK");
}


// n1이 n2보다 작으면 true로 반환함. 
bool comp_cost(Node& n1, Node& n2){
    return n1.cost < n2.cost;
}
//sub된걸 callback해야함 
void RRT::odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
    //ROS_INFO("odom_callbac_s");
    // The pose callback when subscribed to particle filter's inferred pose
    // The RRT main loop happens here
    car_pose_ = odom_msg->pose.pose;
    Node start_node;
    start_node.x = car_pose_.position.x;
    start_node.y = car_pose_.position.y;
    start_node.parent = 0;
    start_node.cost = 0.0;
    start_node.is_root = true;
    //Get Goal
    get_current_goal();
    std::vector<Node> tree;
    std::vector<Node> nodes_near_goal;
    tree.clear();
    tree.push_back(start_node);
    /***  RRT* main loop ***/
    //가장 가까운 waypoint에서 rrt node와 tree찾기
    for (int iter=0; iter<MAX_ITER; iter++){
        vector<double> sampled_point = sample();
        int nearest_ind = nearest(tree, sampled_point);
        Node new_node = steer(tree.at(nearest_ind), sampled_point);
        if (!check_collision(tree.at(nearest_ind), new_node)){
            vector<int> nodes_near = near(tree, new_node);
            tree.push_back(new_node);

            /** connect new_node to the node in the neighborhood with the minimum cost **/
            int min_cost_node_ind = nearest_ind;
            float min_cost = tree.at(nearest_ind).cost + line_cost(tree.at(nearest_ind), new_node);
            for (int i=0; i<nodes_near.size(); i++){
                if(!check_collision(tree.at(nodes_near.at(i)), new_node)){
                    float cost = tree.at(nodes_near.at(i)).cost + line_cost(tree.at(nodes_near.at(i)), new_node);
                    if(cost < min_cost) {
                       min_cost_node_ind = nodes_near.at(i);
                       min_cost = cost;
                   }
                }
            }

            tree.back().is_root = false;
            tree.back().cost = min_cost;
            // add edge
            tree.back().parent = min_cost_node_ind;
            tree.at(min_cost_node_ind).children.push_back(tree.size()-1);

            /** Rewiring **/
            int rewire_count = 0; //49:5
            for (int j=0; j<int(nodes_near.size()); j++) {
                float new_cost = tree.back().cost + line_cost(new_node, tree.at(nodes_near.at(j)));
                if (new_cost < tree.at(nodes_near.at(j)).cost) {
                    if (!check_collision(tree.at(nodes_near.at(j)), new_node)) {
                        // rewire: update parent, cost and costs of all children;
                        float cost_change = new_cost - tree.at(nodes_near.at(j)).cost;
                        tree.at(nodes_near.at(j)).cost = new_cost;
                        // assign new_node to be its new parent
                        int old_parent = tree.at(nodes_near.at(j)).parent;
                        tree.at(nodes_near.at(j)).parent = tree.size() - 1;
                        tree.back().children.push_back(nodes_near.at(j));
                        // remove it from its old parent's children list
                        vector<int>::iterator start = tree.at(old_parent).children.begin();
                        vector<int>::iterator end = tree.at(old_parent).children.end();
                        tree.at(old_parent).children.erase(remove(start, end, nodes_near.at(j)), end);
                        // update_children_cost(tree, nodes_near.at(j), cost_change); // optional(expensive)
                    }
                }
                rewire_count ++;
            }
            //cout<<"rewire: "<<rewire_count<<endl;
            if (is_goal(tree.back(), waypoints_.at(curr_goal_ind_).x, waypoints_.at(curr_goal_ind_).y)){
                nodes_near_goal.push_back(tree.back());
            }
        }
        /** check if goal reached and recover path with the minimum cost**/
        if(iter>MIN_ITER && !nodes_near_goal.empty()){
            Node best = *min_element(nodes_near_goal.begin(), nodes_near_goal.end(), comp_cost);
            vector<Node> path_found = find_path(tree, nodes_near_goal.back());

            visualization_msgs::Marker path_dots;
            path_dots.header.frame_id = "map";
            path_dots.id = 20;
            path_dots.ns = "path";
            path_dots.type = visualization_msgs::Marker::POINTS;
            path_dots.scale.x = path_dots.scale.y = path_dots.scale.z = 0.08;
            path_dots.action = visualization_msgs::Marker::ADD;
            path_dots.pose.orientation.w = 1.0;
            path_dots.color.g = 0.0;
            path_dots.color.r = 1.0;
            path_dots.color.a = 1.0;

            for (int i=0; i<path_found.size(); i++){
                geometry_msgs::Point p;
                p.x = path_found.at(i).x;
                p.y = path_found.at(i).y;
                path_dots.points.push_back(p);
            }
            double RRT_INTERVAL = 0.2;
            vector<geometry_msgs::Point> path_processed;
            for (int i=0; i< path_dots.points.size()-1; i++){
                path_processed.push_back(path_dots.points[i]);
                double dist = sqrt(pow(path_dots.points[i+1].x-path_dots.points[i].x, 2)
                                   +pow(path_dots.points[i+1].y-path_dots.points[i].y, 2));
                if (dist < RRT_INTERVAL) continue;
                int num = static_cast<int>(ceil(dist/RRT_INTERVAL));
                for(int j=1; j< num; j++){
                    geometry_msgs::Point p;
                    p.x = path_dots.points[i].x + j*((path_dots.points[i+1].x - path_dots.points[i].x)/num);
                    p.y = path_dots.points[i].y + j*((path_dots.points[i+1].y - path_dots.points[i].y)/num);
                    path_processed.push_back(p);
                }
            }    
            // rrt 정보 받기
            nav_msgs::Path path1;
            geometry_msgs::PoseStamped pose;

            //	    boost::shared_ptr<nav_msgs::Odometry const> pose_ptr;
            //          pose_ptr = ros::topic::waitForMessage<nav_msgs::Odometry>(pose_topic, ros::Duration(5.0));
            nav_msgs::Odometry msg;

            pose.header.stamp = ros::Time::now();
            pose.header.frame_id = "odom"; //path

            pose.pose.position = msg.pose.pose.position;
            pose.pose.orientation = msg.pose.pose.orientation;

            path1.header.stamp = ros::Time::now();
            path1.header.frame_id = "map";

            for (int i=0; i<path_found.size(); i++){
                geometry_msgs::Point p;
                pose.pose.position.x = path_found.at(i).x;
                pose.pose.position.y = path_found.at(i).y;
//                path_dots.points.push_back(p);
                path1.poses.push_back(pose);
            }            
	    
//	        path1.poses.push_back(pose); 
            path1_.publish(path1);

            // waypoint 받아오기
            nav_msgs::Path waypath;
            //	    boost::shared_ptr<nav_msgs::Odometry const> pose_ptr;
            //          pose_ptr = ros::topic::waitForMessage<nav_msgs::Odometry>(pose_topic, ros::Duration(5.0));
            //geometry_msgs::PoseStamped pose;
            //nav_msgs::Odometry msg;
            pose.header.stamp = ros::Time::now();
            pose.header.frame_id = "odom"; //path

            pose.pose.position = msg.pose.pose.position;
            pose.pose.orientation = msg.pose.pose.orientation;

            waypath.header.stamp = ros::Time::now();
            waypath.header.frame_id = "map";
            //load_waypath(way_path_dots);
            boost::shared_ptr<nav_msgs::Odometry const> pose_ptr;
            pose_ptr = ros::topic::waitForMessage<nav_msgs::Odometry>(pose_topic, ros::Duration(5.0));
            int min_ind= find_closest_waypoint(waypoints_, pose_ptr->pose.pose);
            if (min_ind < curr_goal_ind_){
                for (int i=min_ind; i<= curr_goal_ind_; i++){
                    pose.pose.position.x = waypoints_.at(i).x;
                    pose.pose.position.y = waypoints_.at(i).y;
                    pose.pose.position.z = waypoints_.at(i).z;
                    waypath.poses.push_back(pose);      
                }
                waypath_.publish(waypath);   
            }
            else{
                for (int i=min_ind; i<=waypoints_.size()-1; i++){
                    pose.pose.position.x = waypoints_.at(i).x;
                    pose.pose.position.y = waypoints_.at(i).y;
                    pose.pose.position.z = waypoints_.at(i).z;
                    waypath.poses.push_back(pose);     
                }
                for (int i=0; i<=curr_goal_ind_; i++){
                    pose.pose.position.x = waypoints_.at(i).x;
                    pose.pose.position.y = waypoints_.at(i).y;
                    pose.pose.position.z = waypoints_.at(i).z;
                    waypath.poses.push_back(pose);
                }     
                waypath_.publish(waypath);
            }     
 	        //waypoint.poses.push_back(pose); 
            //waypath_.publish(waypath); //eunj
            ////////////////////////////////////
            path_dots.points = path_processed;
            path_pub_.publish(path_dots);
            //track_path(waypath);  //path1
            visualize_tree(tree);
            //ROS_INFO("path found");
            break;
        }
    }

    if (nodes_near_goal.empty()){
        ROS_INFO("Couldn't find a path");
    }
    //ROS_INFO("odom_callback_OK");
}
// 피타고라스 계산법    
double calculate_dist2(double x1, double x2, double y1, double y2){
    // considering that doing sqrt is expensive
    return pow(x1-x2, 2) + pow(y1-y2,2);  //roots
}
//현재 시점에서 목표점을 찾도록 하는 함수
void RRT::get_current_goal(){
    //ROS_INFO("get_current_goal_s");
    float dist_to_goal2 = calculate_dist2(waypoints_.at(curr_goal_ind_).x, car_pose_.position.x,
                              waypoints_.at(curr_goal_ind_).y,  car_pose_.position.y);
    // goal out of range, reset goal
    if (dist_to_goal2 > pow(GOAL_AHEAD_DIST, 2)){
        reset_goal();
    }
    if (occupancy_grid::is_xy_occupied(map_updated_, waypoints_.at(curr_goal_ind_).x, waypoints_.at(curr_goal_ind_).y)){
        advance_goal();
    }
    // enough progress made, advance goal
    else if(dist_to_goal2 < pow(GOAL_AHEAD_DIST*0.75, 2)){
        advance_goal();
    }
    //ROS_INFO("get_current_goal_OK");
}
// 목표지점이 이상할때 다시 목표지점을 찾도록 하는 함수
void RRT::reset_goal(){
    boost::shared_ptr<nav_msgs::Odometry const> pose_ptr;
    pose_ptr = ros::topic::waitForMessage<nav_msgs::Odometry>(pose_topic, ros::Duration(5.0));


    if(pose_ptr== nullptr){ROS_INFO("Failed to receive car's pose");}
    else{
        //ROS_INFO("reset_goal_OK");
        int closest_ind = find_closest_waypoint(waypoints_, pose_ptr->pose.pose);
        //ROS_INFO("same min_dist: %i\n", closest_ind);

        //ROS_INFO("reset_goal_OK");
        ///////////////////////////////////////
        float closest_dist2 = calculate_dist2(waypoints_.at(closest_ind).x, pose_ptr->pose.pose.position.x,
                                              waypoints_.at(closest_ind).y, pose_ptr->pose.pose.position.y);
        //ROS_INFO("dist value: %f\n", closest_dist2);
        ///////////////////////////////////////
        //if (closest_dist2 > pow(GOAL_AHEAD_DIST, 2)){
          //  throw "Couldn't find a goal in range. Reposition the car somewhere near the waypoints";
        //}
        // advance from closest waypoint until one that is around 0.9 GOAL_AHEAD_DIST away
        curr_goal_ind_ = closest_ind;
        advance_goal();
    }
    //ROS_INFO("reset_goal_OK");
}
// 최종 목표지점을 더블체크하는 함수. 설정한 goal_ahead 길이의 0.9만큼으로 goal을 잡는다.  
void RRT::advance_goal(){
    // advance in waypoints from current goal to a point that is around 0.9*MAX_GOAL_AHEAD distance ahead
    using namespace occupancy_grid;
    int curr_ind = curr_goal_ind_;
    if (curr_ind >= waypoints_.size()){ curr_ind = 0;}
    float pose_x = car_pose_.position.x;
    float pose_y = car_pose_.position.y;
    float curr_dist2 = calculate_dist2(waypoints_.at(curr_ind).x, pose_x, waypoints_.at(curr_ind).y, pose_y);
    ROS_INFO("curr_dist: %f\n", curr_dist2);

    while(curr_dist2 < pow(GOAL_AHEAD_DIST*0.9, 2)
      || is_xy_occupied(map_updated_, waypoints_.at(curr_ind).x, waypoints_.at(curr_ind).y)){
        curr_dist2 = calculate_dist2(waypoints_.at(curr_ind).x, pose_x, waypoints_.at(curr_ind).y, pose_y);
        curr_ind++;
        if (curr_ind >= waypoints_.size()) {curr_ind = 0;}
        if (curr_dist2 > pow(GOAL_AHEAD_DIST, 2) && is_xy_occupied(map_updated_, waypoints_.at(curr_ind).x, waypoints_.at(curr_ind).y)){
            break;
        }
    }
    curr_goal_ind_ = max(0, curr_ind-1);
    ROS_INFO("adv goal OK");
}
//가장 가까운 waypoint찾기
int RRT::find_closest_waypoint(const vector<geometry_msgs::Point>& waypoints, const geometry_msgs::Pose& pose){
    float min_dist2 = 100000.0;
    int min_ind;
    for (int i=0; i<waypoints.size(); i++){
        float dist2 = calculate_dist2(waypoints.at(i).x, pose.position.x, waypoints.at(i).y, pose.position.y);
        if (dist2 < min_dist2){
            min_dist2 = dist2;
            min_ind = i;
        }
    }
    ROS_INFO("min_dist: %f\n", min_dist2);
    ROS_INFO("i value: %i\n", min_ind);
    ROS_INFO("find_closest_waypoint_OK");
    return min_ind;
}
//rrt node와 branch를 생성
// rrt의 임의 sample점 생성
std::vector<double> RRT::sample() {
    // This method returns a sampled point from the free space

    std::vector<double> sampled_point;

   // double x = car_pose_.position.x + uni_dist_(gen_)*X_SAMPLE_RANGE;
   // double y = car_pose_.position.y + uni_dist_(gen_)*Y_SAMPLE_RANGE;
    std::normal_distribution<double> norm_dist_x(0.6*waypoints_.at(curr_goal_ind_).x+0.4*car_pose_.position.x, STD);
    std::normal_distribution<double> norm_dist_y(0.6*waypoints_.at(curr_goal_ind_).y+0.4*car_pose_.position.y, STD);
    double x = norm_dist_x(gen_);
    double y = norm_dist_y(gen_);

    // sample recursively until one in the free space gets returned
    if (!occupancy_grid::is_xy_occupied(map_updated_,x, y)){
        sampled_point.push_back(x);
        sampled_point.push_back(y);
        return sampled_point;
    }
    else{
        return sample();
    }
}
//p_nearest 찾는 함수
int RRT::nearest(std::vector<Node> &tree, std::vector<double> &sampled_point) {
    // This method returns the nearest node on the tree to the sampled point
    // Args:
    //     tree (std::vector<Node>): the current RRT tree
    //     sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //     nearest_node (int): index of nearest node on the tree

    int nearest_node = 0;
    double min_dist = 100000.0;
    for (int i=0; i<int(tree.size()); i++){
        double dist = calculate_dist2(tree.at(i).x, sampled_point[0], tree.at(i).y, sampled_point[1]);
        if (dist<min_dist){
            nearest_node = i;
            min_dist = dist;
        }
    }
    return nearest_node;
}
// 새로운 노드 생성하도록 하는 함수 
Node RRT::steer(Node &nearest_node, std::vector<double> &sampled_point) {
    // The function steer:(x,y)->z returns a point such that z is “closer”
    // to y than x is. The point z returned by the function steer will be
    // such that z minimizes ||z−y|| while at the same time maintaining
    //||z−x|| <= max_expansion_dist, for a prespecified max_expansion_dist > 0

    // basically, expand the tree towards the sample point (within a max dist)

    // Args:
    //    nearest_node (Node): nearest node on the tree to the sampled point
    //    sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //    new_node (Node): new node created from steering
    Node new_node;
    double dist = sqrt(calculate_dist2(nearest_node.x, sampled_point[0], nearest_node.y, sampled_point[1]));

    new_node.x = nearest_node.x + min(STEER_RANGE, dist)*(sampled_point[0]-nearest_node.x)/dist;
    new_node.y = nearest_node.y + min(STEER_RANGE, dist)*(sampled_point[1]-nearest_node.y)/dist;

    return new_node;
}
// node와 node끼리 연결선 만들때 장애물이 있는지 파악하는 함수
bool RRT::check_collision(Node &nearest_node, Node &new_node) {
    // This method returns a boolean indicating if the path between the
    // nearest node and the new node created from steering is collision free

    bool collision = false;
    using namespace occupancy_grid;
    int x_cell_diff = abs(ceil((nearest_node.x-new_node.x)/map_updated_.info.resolution));
    int y_cell_diff = abs(ceil((nearest_node.y-new_node.y)/map_updated_.info.resolution));

    double dt = 1.0/max(x_cell_diff, y_cell_diff);
    double t = 0.0;
    for (int i=0;i<= max(x_cell_diff, y_cell_diff); i++){
        double x = nearest_node.x + t*(new_node.x-nearest_node.x);
        double y = nearest_node.y + t*(new_node.y-nearest_node.y);
        if (is_xy_occupied(map_updated_, x, y)){
            collision = true;
            break;
        }
        t+=dt;
    }
    return collision;
}

bool RRT::is_goal(Node &latest_added_node, double goal_x, double goal_y) {
    // This method checks if the latest node added to the tree is close
    // enough
    return calculate_dist2(goal_x, latest_added_node.x, goal_y, latest_added_node.y) < pow(GOAL_THRESHOLD, 2);
}
//목표 지점까지 path tree만들기
std::vector<Node> RRT::find_path(std::vector<Node> &tree, Node& node) {
    // This method traverses the tree from the node that has been determined
    // as goal
    std::vector<Node> found_path;
    Node current = node;
    while (!current.is_root){
        found_path.push_back(current);
        current = tree.at(current.parent);
    }
    found_path.push_back(current); // add start node
    reverse(found_path.begin(), found_path.end());
    return found_path;
}
// RRT* methods // 최적의 노드 찾기
double RRT::line_cost(Node &n1, Node &n2) {
    // This method returns the cost of the straight line path between two nodes

    return sqrt(calculate_dist2(n1.x, n2.x, n1.y, n2.y));
}

std::vector<int> RRT::near(std::vector<Node> &tree, Node& node) {
    // This method returns the set of Nodes in the neighborhood of a node.
    // Returns:
    //   neighborhood (std::vector<int>): the index of the nodes in the neighborhood
    std::vector<int> neighborhood;
    neighborhood.clear();
    for (int i=0; i<tree.size(); i++){
        if (line_cost(tree.at(i), node) < NEAR_RANGE){
            neighborhood.push_back(i);
        }
    }
    return neighborhood;
}
//rrt의 노드와 트리 시각화하도록 하는 함수 
void RRT::visualize_tree(vector<Node>& tree){
    // plot goal first
    geometry_msgs::Pose goal_pose;
    goal_pose.orientation.w = 1.0;
    goal_pose.position.x = waypoints_.at(curr_goal_ind_).x;
    goal_pose.position.y = waypoints_.at(curr_goal_ind_).y;
    goal_viz->set_pose(goal_pose);
    goal_viz->publish_marker();
    // plot tree
    tree_nodes.points.clear();
    tree_branch.points.clear();

    for (int i=0; i<tree.size(); i++){
        geometry_msgs::Point p;
        p.x = tree.at(i).x; p.y = tree.at(i).y;
        tree_nodes.points.push_back(p);
        for (int j=0; j<tree.at(i).children.size(); j++){
            tree_branch.points.push_back(p);
            geometry_msgs::Point p_child;
            p_child.x = tree.at(tree.at(i).children.at(j)).x;
            p_child.y = tree.at(tree.at(i).children.at(j)).y;
            tree_branch.points.push_back(p_child);
        }
    }
    tree_branches_pub_.publish(tree_branch);
    tree_nodes_pub_.publish(tree_nodes);
}
// 가장가까운 waypoint을 따라가도록 하는 함수 
void RRT::track_path(const nav_msgs::Path& path){
    //use pure pursuit to track the path planned by RRT
    int i =0;
    while (i<path.poses.size()-1){
        float x = path.poses.at(i).pose.position.x;
        float y = path.poses.at(i).pose.position.y;
        float x_car = car_pose_.position.x;
        float y_car = car_pose_.position.y;
        if (calculate_dist2(x, x_car, y, y_car) > pow(LOOK_AHEAD_DIST, 2)){
            break;
        }
        i++;
    }
    //calculate setpoint for pure pursuit to track
    tf::Vector3 p1(path.poses.at(i).pose.position.x, path.poses.at(i).pose.position.y, 0.0);
    tf::Vector3 p2(path.poses.at(max(0,i-1)).pose.position.x, path.poses.at(max(0,i-1)).pose.position.y, 0.0);
    pos_sp_ = tf_.inverse() * ((p1 + p2) / 2.0);
    float curvature = 2*abs(pos_sp_.getY())/(LOOK_AHEAD_DIST * LOOK_AHEAD_DIST);

    // publish drive cmds
    float steering_cmd =  P_GAIN * curvature;
    ROS_INFO("steering_cmd: %f\n", steering_cmd);
    if (pos_sp_.getY()<0){steering_cmd *= -1.0;}
    publish_cmd(steering_cmd);

    // visualize setpoint for tracking
    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x = pos_sp_.x(); pose.position.y = pos_sp_.y();
    pos_sp_viz->set_pose(pose);
    pos_sp_viz->publish_marker();
}
//steering에 따라 속도 다르게 내보내기
void RRT::publish_cmd(float steering_cmd){
    steering_cmd = min(steering_cmd, 0.41f);
    steering_cmd = max(steering_cmd, -0.41f);
    ROS_INFO("steering_cmdp: %f\n", steering_cmd);
    double speed = SPEED;
    if(abs(steering_cmd)>0.1){
        // speed -= (steering_cmd-0.1)/(0.41-0.28)*MAX_DECELARATION;
        speed = SPEED - abs(steering_cmd)*10;
        if (speed > 1){
            speed = speed;
            }
        else{
            speed = 1;
            }
    }
    cout<<"speed: "<<speed<<endl;
    ackermann_msgs::AckermannDriveStamped ack_msg;
    ack_msg.header.stamp = ros::Time::now();
    ack_msg.drive.speed = speed;
    ack_msg.drive.steering_angle = steering_cmd;
    ack_msg.drive.steering_angle_velocity = 1.0;
    drive_pub_.publish(ack_msg);
}
//자식 노드 update
void RRT::update_children_cost(vector<Node>& tree, int root_node_ind, float cost_change){
    if (tree.at(root_node_ind).children.empty()){
        return;
    }
    else{
        for (int i=1; i<tree.at(root_node_ind).children.size(); i++){
            tree.at(tree.at(root_node_ind).children.at(i)).cost += cost_change;
            update_children_cost(tree, tree.at(root_node_ind).children.at(i), cost_change);
        }
    }
}
