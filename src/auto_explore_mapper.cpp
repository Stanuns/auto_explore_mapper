//
// Created by Wei.Sun on 08/19/24
//
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <array>
#include <filesystem>
#include <fstream>
#include <cstdio>

#include <iostream>

#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "map_msgs/msg/occupancy_grid_update.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/occ_grid_values.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "std_msgs/std_msgs/msg/color_rgba.hpp"
#include "nav2_map_server/map_mode.hpp"
#include "nav2_map_server/map_saver.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/utils.h"

#include "robot_interfaces/msg/auto_explore_mapping_state.hpp"
#include "robot_interfaces/msg/auto_explore_mapping_trigger.hpp"


using std::placeholders::_1;
using sensor_msgs::msg::Range;
using geometry_msgs::msg::PoseWithCovarianceStamped;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::Point;
using nav_msgs::msg::OccupancyGrid;
using nav2_msgs::action::NavigateToPose;
using map_msgs::msg::OccupancyGridUpdate;
using visualization_msgs::msg::MarkerArray;
using visualization_msgs::msg::Marker;
using std_msgs::msg::ColorRGBA;
using nav2_costmap_2d::Costmap2D;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::FREE_SPACE;
using std::to_string;
using std::abs;
using std::chrono::milliseconds;
using namespace std::chrono_literals;
using namespace std;
using namespace rclcpp;
using namespace rclcpp_action;
using namespace nav2_map_server;
using std::chrono::steady_clock;

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class AutoExploreMapper : public Node {
public:
    AutoExploreMapper()
            : Node("auto_explore_mapper") {
        state_publisher_ = create_publisher<robot_interfaces::msg::AutoExploreMappingState>("/auto_explore_mapping/state", 10); 
        amstate.state = -1;
        timer_ = this->create_wall_timer(
            500ms, std::bind(&AutoExploreMapper::AEMStateTimerCallback, this));   

        RCLCPP_INFO(get_logger(), "AutoExploreMapper started...");
        // node_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});

        pose_subscription_ = create_subscription<PoseWithCovarianceStamped>(
                "/pose", 10, bind(&AutoExploreMapper::PoseTopicCallback, this, _1));

        map_subscription_ = create_subscription<OccupancyGrid>(
                "/map", 10, bind(&AutoExploreMapper::UpdateFullMap, this, _1));

        markerArrayPublisher_ = create_publisher<MarkerArray>("/frontiers", 10);
        nav2_action_client_ = rclcpp_action::create_client<NavigateToPose>(
                this,
                "/navigate_to_pose");

        trigger_subscription_ = create_subscription<robot_interfaces::msg::AutoExploreMappingTrigger>(
                "/auto_explore_mapping/trigger", 1, bind(&AutoExploreMapper::trigger, this, _1));

        tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
            get_node_base_interface(),
            get_node_timers_interface());
        tf2_buffer_->setCreateTimerInterface(timer_interface);
        tfL_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
        
        amstate.state = 0;
        // while(!nav2_action_client_->wait_for_action_server(std::chrono::seconds(5))){
        //     RCLCPP_INFO(get_logger(), "Navigation action server not available after waiting");
        // }
        
        // RCLCPP_INFO(get_logger(), "AutoExploreMapper nav2_action_client_");

        // declare_parameter("map_path", rclcpp::PARAMETER_STRING);
        // get_parameter("map_path", mapPath_);

        pre_goal.pose.pose.position.x = 0;
        pre_goal.pose.pose.position.y = 0;

        aep_trigger = -1; //初始化
    }

private:
    // rclcpp::Node::SharedPtr node_;
    const double MIN_FRONTIER_DENSITY = 0.3; //0.3
    const double MIN_DISTANCE_TO_FRONTIER = 1.0;
    const int MIN_FREE_THRESHOLD = 4; //4 减少该参数， 增加选取边界点数量
    Costmap2D costmap_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav2_action_client_;
    Publisher<MarkerArray>::SharedPtr markerArrayPublisher_;
    Publisher<robot_interfaces::msg::AutoExploreMappingState>::SharedPtr state_publisher_;
    int pre_marker_size = 0;
    Subscription<OccupancyGrid>::SharedPtr map_subscription_;
    bool isExploring_ = false;
    // int markerId_;
    string mapPath_;

    int checkFrontierEmpty = 0;
    NavigateToPose::Goal pre_goal;
    bool isReturnInitialPoint = false;

    robot_interfaces::msg::AutoExploreMappingState amstate;
    // bool isFullStart = false;
    std::shared_ptr<tf2_ros::TransformListener> tfL_{nullptr};
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;

    Subscription<PoseWithCovarianceStamped>::SharedPtr pose_subscription_;
    PoseWithCovarianceStamped::UniquePtr pose_;

    int aep_trigger;
    Subscription<robot_interfaces::msg::AutoExploreMappingTrigger>::SharedPtr trigger_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;

    array<unsigned char, 256> costTranslationTable_ = initTranslationTable();

    static array<unsigned char, 256> initTranslationTable() {
        array<unsigned char, 256> cost_translation_table{};

        // lineary mapped from [0..100] to [0..255]
        for (size_t i = 0; i < 256; ++i) {
            cost_translation_table[i] =
                    static_cast<unsigned char>(1 + (251 * (i - 1)) / 97);
        }

        // special values:
        cost_translation_table[0] = FREE_SPACE;
        cost_translation_table[99] = 253;
        cost_translation_table[100] = LETHAL_OBSTACLE;
        cost_translation_table[static_cast<unsigned char>(-1)] = NO_INFORMATION;

        return cost_translation_table;
    }

    struct Frontier {
        Point centroid;
        vector<Point> points;
        string getKey() const{to_string(centroid.x) + "," + to_string(centroid.y);}
    };

    void AEMStateTimerCallback(){
        amstate.header.stamp = this->get_clock()->now(); 
        state_publisher_->publish(amstate);    

        if(aep_trigger == 1 && amstate.state == 0){
            //开始自动建图
            amstate.state = 1;
            aep_trigger = -1;
        }else if(aep_trigger == 3 && amstate.state == 2){
            //停止自动建图
            Stop();
            aep_trigger = -1;
        }else if(aep_trigger == 5 && amstate.state == 2){
            //保存地图
            SaveMap();  
            aep_trigger = -1;     
        }
    }

    void trigger(const robot_interfaces::msg::AutoExploreMappingTrigger::SharedPtr tr_){
        int trig_temp = tr_->to_state;
        if(trig_temp == 1 || trig_temp == 3 || trig_temp == 5){
            aep_trigger = trig_temp;
        }  
    }

    void PoseTopicCallback(PoseWithCovarianceStamped::UniquePtr pose) {
        pose_ = move(pose);
        // RCLCPP_INFO(get_logger(), "PoseTopicCallback...");
    }

    void UpdateFullMap(OccupancyGrid::UniquePtr occupancyGrid) {
        if(!(amstate.state == 1) && !(amstate.state == 2)){
            return;
        }

        if (pose_ == nullptr) { return; }
        RCLCPP_INFO(get_logger(), "update full map start...");
        const auto occupancyGridInfo = occupancyGrid->info;
        unsigned int size_in_cells_x = occupancyGridInfo.width;
        unsigned int size_in_cells_y = occupancyGridInfo.height;
        double resolution = occupancyGridInfo.resolution;
        double origin_x = occupancyGridInfo.origin.position.x;
        double origin_y = occupancyGridInfo.origin.position.y;

        // // debug
        // ofstream outputFile;
        // outputFile.open("DataMap_CG_082701.txt",ios::app); 
        // int maxData = -300; //max = 100
        // int minData = 300; //min = -1;
        // for(int i = 0; i< size_in_cells_x * size_in_cells_y; i++){
        //     if(occupancyGrid->data[i] > maxData){
        //         maxData = occupancyGrid->data[i];
        //     }
        //     if(occupancyGrid->data[i] < minData){
        //         minData = occupancyGrid->data[i];
        //     }
        //     outputFile <<  (int) occupancyGrid->data[i] << "\n";
        // }
        // outputFile.close();

        RCLCPP_INFO(get_logger(), "received full new map, resizing to: %d, %d", size_in_cells_x,
                    size_in_cells_y);
        costmap_.resizeMap(size_in_cells_x,
                           size_in_cells_y,
                           resolution,
                           origin_x,
                           origin_y);

        // lock as we are accessing raw underlying map
        auto *mutex = costmap_.getMutex();
        lock_guard<Costmap2D::mutex_t> lock(*mutex);

        // fill map with data
        unsigned char *costmap_data = costmap_.getCharMap();
        size_t costmap_size = costmap_.getSizeInCellsX() * costmap_.getSizeInCellsY();
        RCLCPP_INFO(get_logger(), "full map update, %lu values", costmap_size);
        for (size_t i = 0; i < costmap_size && i < occupancyGrid->data.size(); ++i) {
            //adapt to diversive grid map
            if(occupancyGrid->data[i] >= 55){
                occupancyGrid->data[i] = 100;
            }else if(occupancyGrid->data[i] < 40 && occupancyGrid->data[i] >= 10){
                occupancyGrid->data[i] = -1;
            }
            // else{
            //     occupancyGrid->data[i] = -1;
            // }

            auto cell_cost = static_cast<unsigned char>(occupancyGrid->data[i]);
            costmap_data[i] = costTranslationTable_[cell_cost];
        }

        // // debug
        // ofstream outputFile2;
        // outputFile2.open("DataMapCost_CG_082701.txt",ios::app); 
        // int maxDataCost = -300; //max = 255
        // int minDataCost = 300; //min = 0;
        // for(int i = 0; i< costmap_size; i++){
        //     if(costmap_data[i] > maxDataCost){
        //         maxDataCost = costmap_data[i];
        //     }
        //     if(costmap_data[i] < minDataCost){
        //         minDataCost = costmap_data[i];
        //     }
        //     outputFile2 <<  (int) costmap_data[i] << "\n";
        // }
        // outputFile2.close();

        Explore();
    }

    void DrawMarkers(const vector<Frontier> &frontiers) {
        // remove previous markers
        MarkerArray pre_markersMsg_;
        int pre_markerId_= 0 ;
        vector<Marker> &pre_mm_markers = pre_markersMsg_.markers;
        // ColorRGBA colour;
        // colour.r = 1;
        // colour.g = 1;
        // colour.b = 0;
        // colour.a = 1.0;
        for (int i = 0;i < pre_marker_size; i++) {
            Marker m;

            m.header.frame_id = "map";
            m.header.stamp = now();
            m.frame_locked = true;

            m.action = Marker::DELETE;
            m.ns = "frontiers";
            m.id = ++pre_markerId_;
            m.type = Marker::SPHERE;
            m.pose.position.x = 0;
            m.pose.position.y = 0;
            m.scale.x = 0.3;
            m.scale.y = 0.3;
            m.scale.z = 0.3;
            // m.color = colour;
            pre_mm_markers.push_back(m);           
        }
        markerArrayPublisher_->publish(pre_markersMsg_);

        // markersMsg_
        MarkerArray markersMsg_;
        int markerId_= 0 ;
        vector<Marker> &mm_markers = markersMsg_.markers;
        ColorRGBA colour;
        colour.r = 1;
        colour.g = 1;
        colour.b = 0;
        colour.a = 1.0;
        
        RCLCPP_INFO(get_logger(), "DrawMarkers:frontiers.size(): %d ", frontiers.size());

        for (const auto &frontier: frontiers) {
            // RCLCPP_INFO(get_logger(), "visualising: %f,%f ", frontier.centroid.x, frontier.centroid.y);

            Marker m;

            m.header.frame_id = "map";
            m.header.stamp = now();
            m.frame_locked = true;

            m.action = Marker::ADD;
            m.ns = "frontiers";
            m.id = ++markerId_;
            m.type = Marker::SPHERE;
            m.pose.position = frontier.centroid;
            m.scale.x = 0.3;
            m.scale.y = 0.3;
            m.scale.z = 0.3;
            m.color = colour;
            mm_markers.push_back(m);           
        }
        markerArrayPublisher_->publish(markersMsg_);

        pre_marker_size = mm_markers.size();
    }

    void ClearMarkers() {
        // for (auto &m: markersMsg_.markers) {
        //     m.action = Marker::DELETE;
        // }
        // markerArrayPublisher_->publish(markersMsg_);
        MarkerArray last_markersMsg_;
        int last_markerId_= 0 ;
        vector<Marker> &last_mm_markers = last_markersMsg_.markers;
        // ColorRGBA colour;
        // colour.r = 1;
        // colour.g = 1;
        // colour.b = 0;
        // colour.a = 1.0;
        for (int i = 0;i < pre_marker_size; i++) {
            Marker m;

            m.header.frame_id = "map";
            m.header.stamp = now();
            m.frame_locked = true;

            m.action = Marker::DELETE;
            m.ns = "frontiers";
            m.id = ++last_markerId_;
            m.type = Marker::SPHERE;
            m.pose.position.x = 0;
            m.pose.position.y = 0;
            m.scale.x = 0.3;
            m.scale.y = 0.3;
            m.scale.z = 0.3;
            // m.color = colour;
            last_mm_markers.push_back(m);           
        }
        markerArrayPublisher_->publish(last_markersMsg_);
    }

    void Stop() {
        amstate.state = 3;

        RCLCPP_INFO(get_logger(), "Stopped...");
        // pose_subscription_.reset();
        // map_subscription_.reset();
        nav2_action_client_->async_cancel_all_goals();
        SaveMap();
        // node_.reset();
        ClearMarkers();
        // markerArrayPublisher_.reset();

        // isFullStart = false;
        sleep(1);//sleep 1sec.
        amstate.state = 0;
    }

    void Explore() {
        if (isExploring_) { return; }
        auto frontiers = SearchFrontiers();
        //debug 
        RCLCPP_WARN(get_logger(), "---After SearchFrontiers---frontiers.size(): %d -----isExploring_:%d ", frontiers.size(),isExploring_);
        DrawMarkers(frontiers);
        // if (frontiers.empty()) {
        //     RCLCPP_WARN(get_logger(), "NO BOUNDARIES FOUND!!");
        //     Stop();
        //     return;
        // }
        //if(frontiers.empty()){
        if(frontiers.size()<=5){
            checkFrontierEmpty++;
            if(checkFrontierEmpty > 25){
                RCLCPP_WARN(get_logger(), "----------NO BOUNDARIES FOUND!!!!! and return to INITIAL POINT and SAVE MAP!!!-----------");
                //return to INITIAL POINT
                auto initial_goal = NavigateToPose::Goal();
                initial_goal.pose.pose.position.x = 0.0;
                initial_goal.pose.pose.position.y = 0.0;
                initial_goal.pose.pose.position.z = 0.0;
                initial_goal.pose.header.frame_id = "map";
                isReturnInitialPoint = true;
                NavigateToGoal(initial_goal);

                return;
            }
            RCLCPP_WARN(get_logger(), "---No frontiers can be searched!!, checkFrontierEmpty: %d---", checkFrontierEmpty);
            //sleep
            sleep(1);

            return;
        }
        checkFrontierEmpty = 0;

        amstate.state = 2;

        //对 frontiers[i].centroid 到机器人当前位姿 pose_ 进行权重排序，选取最合适的边界点
        vector<double>  evaulate_value_frontiers;
        for(auto &frontier: frontiers){
            double dis =  sqrt(pow((double(frontier.centroid.x) - double(pose_->pose.pose.position.x)), 2.0) +
                                           pow((double(frontier.centroid.y) - double(pose_->pose.pose.position.y)), 2.0)); 
            /***
             * [-M_PI, M_PI]
             *  */ 
            double yaw_frontier = atan2(double(frontier.centroid.y) - double(pose_->pose.pose.position.y), double(frontier.centroid.x) - double(pose_->pose.pose.position.x));
            double yaw_base_link = tf2::getYaw(pose_->pose.pose.orientation); //yaw： 0~M_PI   0~-M_PI
            double delta_yaw = yaw_frontier - yaw_base_link;
            double delta_yaw_p = fmod(delta_yaw+2*M_PI, 2*M_PI);
            if(delta_yaw_p > M_PI){
                delta_yaw_p = fmod(2*M_PI - delta_yaw_p, 2*M_PI);
            }
            //debug
            // RCLCPP_INFO(get_logger(), "---frontier:[%f,%f], dis: %f,---delta_yaw: %f,delta_yaw_p:%f,---yaw_frontier: %f, yaw_base_link: %f", frontier.centroid.x, frontier.centroid.y, dis, delta_yaw, delta_yaw_p, yaw_frontier, yaw_base_link);
            double alpha = 1.00; //权重参数
            double beta = 3.00; //权重参数
            double ev_frontier = alpha*dis + beta*delta_yaw_p; //越小越好

            // geometry_msgs::msg::PoseStamped pose_inBaseLink;
            // geometry_msgs::msg::PoseStamped pose_inMap;
            // pose_inMap.header = pose_->header;
            // pose_inMap.pose = pose_->pose.pose;
            // try{
            //     tf2_buffer_->transform(pose_inMap, pose_inBaseLink, "base_link", tf2::durationFromSec(1.0));
            // }
            // catch(...){
            //     return;
            // }
            // double yaw_inBaseLink = tf2::getYaw(pose_inBaseLink.pose.orientation);
            // RCLCPP_INFO(get_logger(), "------delta_yaw: %f yaw_inBaseLink: %f", delta_yaw, yaw_inBaseLink);
           
            evaulate_value_frontiers.push_back(ev_frontier);
        }
        int min_index = minElementIndexVector(evaulate_value_frontiers);

        
        const auto frontier = frontiers[min_index];
        auto goal = NavigateToPose::Goal();
        goal.pose.pose.position = frontier.centroid;

        //judge the distance of goal and pre_goal, if distance<0.8m, then const auto frontier = frontiers[(int)(frontiers.size()-1)/2];
        double dis_goal_pre_goal = sqrt(pow((double(goal.pose.pose.position.x) - double(pre_goal.pose.pose.position.x)), 2.0) +
                                           pow((double(goal.pose.pose.position.y) - double(pre_goal.pose.pose.position.y)), 2.0));
        if(dis_goal_pre_goal < 0.5){

            int frontiersSizeThreshold = 6;
            // int min_index2 = minElementIndexVector(evaulate_value_frontiers);
            if(frontiers.size() > frontiersSizeThreshold){

                for(int i = 0; i < frontiersSizeThreshold-3; i++){
                    frontiers.erase(frontiers.begin() + min_index);
                    evaulate_value_frontiers.erase(evaulate_value_frontiers.begin() + min_index);
                    min_index = minElementIndexVector(evaulate_value_frontiers);
                }

                goal.pose.pose.position = frontiers[min_index].centroid;
            }else{
                goal.pose.pose.position = frontiers[frontiers.size()-1].centroid;
            } 
        }

        // goal.pose.pose.orientation.w = 1.;
        goal.pose.header.frame_id = "map";
        RCLCPP_INFO(get_logger(), "Sending goal %f,%f", frontier.centroid.x, frontier.centroid.y);

        pre_goal.pose.pose.position = goal.pose.pose.position;       

        NavigateToGoal(goal);
        sleep(1);//等待NavigateToGoal
    }

    int minElementIndexVector(const vector<double> &vec){
        auto min_it = std::min_element(vec.begin(), vec.end());
        return std::distance(vec.begin(), min_it);
    }

    void NavigateToGoal(const NavigateToPose::Goal &goal){
        isExploring_ = true;

        while(!nav2_action_client_->wait_for_action_server(std::chrono::seconds(2))){
            RCLCPP_INFO(get_logger(), "Navigation action server not available after waiting");
        }
        RCLCPP_INFO(get_logger(), "--- go into NavigateToGoal,isExploring_:%d ---",isExploring_);  
        //send_goal()
        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback = [this](
                const GoalHandleNavigateToPose::SharedPtr &goal_handle) {

            RCLCPP_INFO(get_logger(), "--- go into goal_response_callback ---");  

            if (goal_handle) {
                RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for result");
                // RCLCPP_INFO(get_logger(), "---------Goal start, isExploring_: %d ",isExploring_);
            } else {
                RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
            }
        };

        send_goal_options.feedback_callback = [this](
                const GoalHandleNavigateToPose::SharedPtr &,
                const std::shared_ptr<const NavigateToPose::Feedback> &feedback) {
            if(feedback->distance_remaining < 2){
                RCLCPP_INFO(get_logger(), "Distance remaining: %f", feedback->distance_remaining);
            }   
        };

        send_goal_options.result_callback = [this](const GoalHandleNavigateToPose::WrappedResult &result) {        
            //RCLCPP_INFO(get_logger(), "---------Goal end wth a result, isExploring_: %d",isExploring_);
            RCLCPP_INFO(get_logger(), "--- go into result_callback ,isExploring_: %d---",isExploring_); 
            //SaveMap();
            //ClearMarkers();
            //Explore();
            switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(get_logger(), "Goal reached");
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(get_logger(), "Goal was aborted");
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_ERROR(get_logger(), "Goal was canceled");
                    break;
                default:
                    RCLCPP_ERROR(get_logger(), "Unknown result code");
                    break;
            }
            if(isReturnInitialPoint){
                Stop();
            }
            isExploring_ = false;
            isReturnInitialPoint = false;
        };
        nav2_action_client_->async_send_goal(goal, send_goal_options);
    }

    int SaveMap() {
        //refer to map_server.cpp
        std::shared_ptr<rclcpp::Node> map_saver_node = rclcpp::Node::make_shared("map_saver_client"); 
        rclcpp::Client<nav2_msgs::srv::SaveMap>::SharedPtr map_saver_client =
            map_saver_node->create_client<nav2_msgs::srv::SaveMap>("/map_saver/save_map");

        auto save_map_request = std::make_shared<nav2_msgs::srv::SaveMap::Request>();

        while (!map_saver_client->wait_for_service(2s)) {
            if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the SaveMap service. Exiting.");
            return -1;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "SaveMap service not available, waiting again...");
        }

        save_map_request->map_topic = "/map";
        rclcpp::Time now = map_saver_node->get_clock()->now(); 
        int t_s = (int)now.seconds() + 8*3600; //UTC+8: 8*3600
        long sod = t_s%(3600*24);
        int hh = sod/3600;
        long moh = sod%3600;
        int mm = moh/60;
        int ss = moh%60;
        save_map_request->map_url = "AutoExploreMap_"+std::to_string(hh)+std::to_string(mm)+std::to_string(ss);
        auto map_saver_result = map_saver_client->async_send_request(save_map_request);

        // Wait for the result.
        if (rclcpp::spin_until_future_complete(map_saver_node, map_saver_result) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Auto Save map Successfully");
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Auto Failed to save map");
            return -1;
        }
        return 0;
    }

    vector<unsigned int> Neighborhood8(unsigned int idx) {
        unsigned int mx, my;
        vector<unsigned int> out;
        costmap_.indexToCells(idx, mx, my);
        const int x = mx;
        const int y = my;
        const pair<int, int> directions[] = {
                pair(-1, -1),
                pair(-1, 1),
                pair(1, -1),
                pair(1, 1),
                pair(1, 0),
                pair(-1, 0),
                pair(0, 1),
                pair(0, -1)
        };
        for (const auto &d: directions) {
            int newX = x + d.first;
            int newY = y + d.second;
            if (newX > -1 && newX < costmap_.getSizeInCellsX() &&
                newY > -1 && newY < costmap_.getSizeInCellsY()) {
                out.push_back(costmap_.getIndex(newX, newY));
            }
        }
        return out;
    }

    vector<unsigned int> Neighborhood24(unsigned int idx) {
        unsigned int mx, my;
        vector<unsigned int> out;
        costmap_.indexToCells(idx, mx, my);
        const int x = mx;
        const int y = my;
        const pair<int, int> directions[] = {
                pair(-1, -1),
                pair(-1, 1),
                pair(1, -1),
                pair(1, 1),
                pair(1, 0),
                pair(-1, 0),
                pair(0, 1),
                pair(0, -1),
                pair(-2, -2),
                pair(-2, -1),
                pair(-2, 0),
                pair(-2, 1),
                pair(-2, 2),
                pair(-1, -2),
                pair(-1, 2),
                pair(0, -2),
                pair(0, 2),
                pair(1, -2),
                pair(1, 2),
                pair(2, -2),
                pair(2, -1),
                pair(2, 0),
                pair(2, 1),
                pair(2, 2),
        };
        for (const auto &d: directions) {
            int newX = x + d.first;
            int newY = y + d.second;
            if (newX > -1 && newX < costmap_.getSizeInCellsX() &&
                newY > -1 && newY < costmap_.getSizeInCellsY()) {
                out.push_back(costmap_.getIndex(newX, newY));
            }
        }
        return out;
    }

    bool IsAchievableFrontierCell(unsigned int idx,
                                  const vector<bool> &frontier_flag) {
        auto map = costmap_.getCharMap();
        // check that cell is unknown and not already marked as frontier
        if (map[idx] != NO_INFORMATION || frontier_flag[idx]) {
            return false;
        }

        //check there's enough free space for robot to move to frontier 
        int freeCount = 0;
        int occupiedCount = 0;
        for (unsigned int nbr: Neighborhood24(idx)) {
            if (map[nbr] == FREE_SPACE) {
                // if (++freeCount >= MIN_FREE_THRESHOLD) {
                //     return true;
                // }
                freeCount++;
            }
            if(map[nbr] == LETHAL_OBSTACLE){
                occupiedCount++;
            }

            if(freeCount >= MIN_FREE_THRESHOLD && occupiedCount < 1){
                return true;
            }

        }

        return false;
    }

    Frontier BuildNewFrontier(unsigned int neighborCell, vector<bool> &frontier_flag) {
        Frontier output;
        output.centroid.x = 0;
        output.centroid.y = 0;

        queue<unsigned int> bfs;
        bfs.push(neighborCell);

        while (!bfs.empty()) {
            unsigned int idx = bfs.front();
            bfs.pop();

            // try adding cells in 8-connected neighborhood to frontier
            for (unsigned int nbr: Neighborhood8(idx)) {
                // check if neighbour is a potential frontier cell
                if (IsAchievableFrontierCell(nbr, frontier_flag)) {
                    // mark cell as frontier
                    frontier_flag[nbr] = true;
                    unsigned int mx, my;
                    double wx, wy;
                    costmap_.indexToCells(nbr, mx, my);
                    costmap_.mapToWorld(mx, my, wx, wy);

                    Point point;
                    point.x = wx;
                    point.y = wy;
                    output.points.push_back(point);

                    // update centroid of frontier
                    output.centroid.x += wx;
                    output.centroid.y += wy;

                    bfs.push(nbr);
                }
            }
        }

        // average out frontier centroid
        output.centroid.x /= output.points.size();
        output.centroid.y /= output.points.size();
        return output;
    }

    vector<Frontier> SearchFrontiers() {
        vector<Frontier> frontier_list;
        const auto position = pose_->pose.pose.position;
        unsigned int mx, my;
        if (!costmap_.worldToMap(position.x, position.y, mx, my)) {
            RCLCPP_ERROR(get_logger(), "Robot out of costmap bounds, cannot search for frontiers");
            return frontier_list;
        }

        // make sure map is consistent and locked for duration of search
        lock_guard<Costmap2D::mutex_t> lock(*(costmap_.getMutex()));

        auto map_ = costmap_.getCharMap();
        auto size_x_ = costmap_.getSizeInCellsX();
        auto size_y_ = costmap_.getSizeInCellsY();

        // initialize flag arrays to keep track of visited and frontier cells
        vector<bool> frontier_flag(size_x_ * size_y_,
                                   false);
        vector<bool> visited_flag(size_x_ * size_y_,
                                  false);

        // initialize breadth first search
        queue<unsigned int> bfs;

        unsigned int pos = costmap_.getIndex(mx, my);
        RCLCPP_INFO(get_logger(), "costmap_.getIndex: %d", pos);

        bfs.push(pos);
        visited_flag[bfs.front()] = true;

        while (!bfs.empty()) {
            unsigned int idx = bfs.front();
            bfs.pop();

            for (unsigned nbr: Neighborhood8(idx)) {
                // add to queue all free, unvisited cells, use descending search in case
                // initialized on non-free cell
                if (map_[nbr] == FREE_SPACE && !visited_flag[nbr]) {
                    visited_flag[nbr] = true;
                    bfs.push(nbr);
                    // check if cell is new frontier cell (unvisited, NO_INFORMATION, free
                    // neighbour)
                } else if (IsAchievableFrontierCell(nbr, frontier_flag)) {
                    frontier_flag[nbr] = true;
                    const Frontier frontier = BuildNewFrontier(nbr, frontier_flag);

                    // double distance = sqrt(pow((double(frontier.centroid.x) - double(position.x)), 2.0) +
                    //                        pow((double(frontier.centroid.y) - double(position.y)), 2.0));
                    double distance = 0;
                    if(!isnan(frontier.centroid.x)){
                        distance = sqrt(pow((double(frontier.centroid.x) - double(position.x)), 2.0) +
                                           pow((double(frontier.centroid.y) - double(position.y)), 2.0));
                    }
                    //debug
                    // RCLCPP_INFO(get_logger(), "distance: %0.6f; position.x: %0.6f; position.y: %0.6f", distance, position.x, position.y);
                    
                    if (distance < MIN_DISTANCE_TO_FRONTIER) { continue; }
                    //debug
                    double frontier_density = frontier.points.size() * costmap_.getResolution();
                    // RCLCPP_INFO(get_logger(), "frontier_density: %0.6f", frontier_density);

                    //max of frontier.points.size() = 8
                    if (frontier.points.size() * costmap_.getResolution() >= MIN_FRONTIER_DENSITY) {
                        frontier_list.push_back(frontier);
                    }
                }
            }
        }

        return frontier_list;
    }

};

int main(int argc, char *argv[]) {
    init(argc, argv);
    spin(make_shared<AutoExploreMapper>());
    shutdown();
    return 0;
}
