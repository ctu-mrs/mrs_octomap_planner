#include <mrs_lib/batch_visualizer.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/transformer.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <mrs_modules_msgs/srv/path.hpp>
#include <astar_planner.hpp>
#include <iostream>
#include <memory>

namespace mrs_octomap_planner
{
  using OcTree_t          = octomap::OcTree;
  using OcTreeSharedPtr_t = std::shared_ptr<octomap::OcTree>;

  class MinimalOctomapPlanner : public rclcpp::Node
  {
  public:
    explicit MinimalOctomapPlanner(const rclcpp::NodeOptions & options);
    virtual void onInit();

  private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Clock::SharedPtr clock_;

    bool        is_initialized_ = false;

    // params
    double _safe_obstacle_distance_      = 0.0;
    double _distance_transform_distance_ = 0.0;
    double _planning_tree_resolution_    = 0.0;
    double _distance_penalty_            = 0.0;
    double _greedy_penalty_              = 0.0;
    double _timeout_threshold_           = 0.0;
    double _max_waypoint_distance_       = 0.0;
    double _min_altitude_                = 0.0;
    double _max_altitude_                = 0.0;
    double _scale_points_                = 0.0;
    double _scale_lines_                 = 0.0;
    double _min_path_length_             = 0.0;
    bool   _unknown_is_occupied_         = false;

    std::mutex                                mutex_octree_;
    std::shared_ptr<OcTree_t>                 octree_ = nullptr;
    std::string                               octree_frame_;
    std::shared_ptr<mrs_lib::BatchVisualizer> bv_planner_;

    mrs_lib::SubscriberHandler<octomap_msgs::msg::Octomap> sh_octomap_;

    void timeoutOctomap(const std::string& topic,
                        const rclcpp::Time&   last_msg);
    void callbackOctomap(const octomap_msgs::msg::Octomap::ConstSharedPtr msg);

    rclcpp::Service<mrs_modules_msgs::srv::Path>::SharedPtr service_server_get_path_;
    
    void callbackGetPath(const std::shared_ptr<mrs_modules_msgs::srv::Path::Request> req,
                         std::shared_ptr<mrs_modules_msgs::srv::Path::Response> res);

    std::unique_ptr<mrs_lib::Transformer> transformer_;

    std::optional<OcTreeSharedPtr_t> msgToMap(const octomap_msgs::msg::Octomap::ConstPtr octomap);

    rclcpp ::TimerBase::SharedPtr timer_init_;
  };

  void MinimalOctomapPlanner::onInit()
  {
    timer_init_->cancel();
    node_ = this->shared_from_this(); 
    clock_ = node_->get_clock();
  
    //rclcpp::Time::waitForValid();

    RCLCPP_INFO(this->get_logger(),"[MrsMinimalOctomapPlanner]: initializing");

    mrs_lib::ParamLoader param_loader(this->shared_from_this(), "MrsMinimalOctomapPlanner");

    param_loader.loadParam("safe_obstacle_distance", _safe_obstacle_distance_);
    param_loader.loadParam("distance_penalty", _distance_penalty_);
    param_loader.loadParam("greedy_penalty", _greedy_penalty_);
    param_loader.loadParam("planning_tree_resolution", _planning_tree_resolution_);
    param_loader.loadParam("unknown_is_occupied", _unknown_is_occupied_);
    param_loader.loadParam("distance_transform_distance", _distance_transform_distance_);
    param_loader.loadParam("max_waypoint_distance", _max_waypoint_distance_);
    param_loader.loadParam("min_altitude", _min_altitude_);
    param_loader.loadParam("max_altitude", _max_altitude_);
    param_loader.loadParam("timeout_threshold", _timeout_threshold_);
    param_loader.loadParam("min_path_length", _min_path_length_);

    param_loader.loadParam("viz/scale/points", _scale_points_);
    param_loader.loadParam("viz/scale/lines", _scale_lines_);

    if (!param_loader.loadedSuccessfully()) {
      RCLCPP_ERROR(this->get_logger(),"[MrsMinimalOctomapPlanner]: Could not load all parameters");
      rclcpp::shutdown();
    }

    auto callback_octomap = [this](const octomap_msgs::msg::Octomap::ConstSharedPtr msg) {
      this->callbackOctomap(msg);
    };

    mrs_lib::SubscriberHandlerOptions shopts;
    shopts.node                 = node_;
    shopts.node_name          = "MrsMinimalOctomapPlanner";
    shopts.no_message_timeout = mrs_lib::no_timeout;
    shopts.threadsafe         = true;
    shopts.autostart          = true;

    sh_octomap_ = mrs_lib::SubscriberHandler<octomap_msgs::msg::Octomap>(shopts,
                                                                   "~/octomap_in",
                                                                   callback_octomap);

    service_server_get_path_ = this->create_service<mrs_modules_msgs::srv::Path>(
            "get_path_in",
            std::bind(&MinimalOctomapPlanner::callbackGetPath, this,
                      std::placeholders::_1, std::placeholders::_2));

    transformer_ = std::make_unique<mrs_lib::Transformer>(node_);
    transformer_->setLookupTimeout(std::chrono::duration<double>(0.5));
    transformer_->retryLookupNewest(true);
    RCLCPP_INFO(this->get_logger(), "Initialized transformer.");

    bv_planner_ = std::make_shared<mrs_lib::BatchVisualizer>(node_, "visualize_planner", "");
    bv_planner_->setPointsScale(_scale_points_);
    bv_planner_->setLinesScale(_scale_lines_);

    is_initialized_ = true;

    RCLCPP_INFO(this->get_logger(),"[MrsMinimalOctomapPlanner]: initialized");
  }

  void MinimalOctomapPlanner::callbackOctomap(const octomap_msgs::msg::Octomap::ConstSharedPtr msg)
  {
    if (!is_initialized_) {
      return;
    }

    RCLCPP_INFO_ONCE(this->get_logger(),"[MrsMinimalOctomapPlanner]: getting octomap");

    std::optional<OcTreeSharedPtr_t> octree_local = msgToMap(msg);

    if (!octree_local) {
      RCLCPP_WARN_THROTTLE(this->get_logger(),*this->get_clock(),1000, "[MrsMinimalOctomapPlanner]: received map is empty!");
      return;
    }
      mrs_lib::set_mutexed(mutex_octree_,octree_local.value(), octree_);
      mrs_lib::set_mutexed(mutex_octree_,msg->header.frame_id, octree_frame_);
  }

  std::optional<OcTreeSharedPtr_t> MinimalOctomapPlanner::msgToMap(const octomap_msgs::msg::Octomap::ConstPtr octomap)
  {
    octomap::AbstractOcTree* abstract_tree;

    if (octomap->binary) {
      abstract_tree = octomap_msgs::binaryMsgToMap(*octomap);

    }
    else {
      abstract_tree = octomap_msgs::fullMsgToMap(*octomap);
      }

    if (!abstract_tree) {
      RCLCPP_WARN(this->get_logger(),"[MrsMinimalOctomapPlanner]: Octomap message is empty! can not convert to OcTree");
      return {};
    }
    else {
      return { OcTreeSharedPtr_t(dynamic_cast<OcTree_t*>(abstract_tree)) };
    }
  }

  void MinimalOctomapPlanner::timeoutOctomap(const std::string& topic,
                                             const rclcpp::Time&   last_msg)
  {
    if (!is_initialized_) {
      return;
    }

    if (!sh_octomap_.hasMsg()) {
      return;
    }

    RCLCPP_WARN_THROTTLE(this->get_logger(),*this->get_clock(),1000,"[MrsMinimalOctomapPlanner]: octomap timeout!");
  }

  void MinimalOctomapPlanner::callbackGetPath(const std::shared_ptr<mrs_modules_msgs::srv::Path::Request> req, std::shared_ptr<mrs_modules_msgs::srv::Path::Response> res)
  {
    if (!is_initialized_) {
      res->success = false;
      res->message = "node not initialized";
      return;
    }

    const bool got_octomap = sh_octomap_.hasMsg() && (clock_->now() - sh_octomap_.lastMsgTime()).seconds() < 2.0;

    if (!got_octomap) {
      RCLCPP_INFO_THROTTLE(this->get_logger(),*this->get_clock(),1000,
                        "[MrsMinimalOctomapPlanner]: waiting for data: octomap = %s",
                        got_octomap ? "TRUE" : "FALSE");
      res->success = false;
      return;
    }

    bv_planner_->setParentFrame(mrs_lib::get_mutexed(mutex_octree_, octree_frame_));
    mrs_octomap_planner::AstarPlanner planner = mrs_octomap_planner::AstarPlanner( 
                                                                                  this->shared_from_this(),
                                                                                  "MrsMinimalOctomapPlanner",
                                                                                  _safe_obstacle_distance_,
                                                                                  _safe_obstacle_distance_,
                                                                                  _distance_transform_distance_,
                                                                                  _planning_tree_resolution_,
                                                                                  _distance_penalty_,
                                                                                  _greedy_penalty_,
                                                                                  _timeout_threshold_,
                                                                                  _max_waypoint_distance_,
                                                                                  _min_altitude_,
                                                                                  _max_altitude_,
                                                                                  _unknown_is_occupied_,
                                                                                  bv_planner_);

    octomap::point3d plan_from, plan_to;
    plan_from.x() = req->start.x;
    plan_from.y() = req->start.y;
    plan_from.z() = req->start.z;

    plan_to.x() = req->end.x;
    plan_to.y() = req->end.y;
    plan_to.z() = req->end.z;

    OcTreeSharedPtr_t octree = mrs_lib::get_mutexed(mutex_octree_, octree_);

    auto path = planner.findPath(plan_from, plan_to, octree, _timeout_threshold_);

    // check path
    if (path.second) {
      // path until the end
      path.first.push_back(plan_to);
      std::stringstream ss;
      ss << "Found complete path of length = " << path.first.size();
      RCLCPP_INFO_STREAM(this->get_logger(),"[MrsMinimalOctomapPlanner]: " << ss.str());
      res->message = ss.str();
    }
    else {
      // no path at all
      if (path.first.size() < 2) {
        RCLCPP_WARN(this->get_logger(),"[MrsMinimalOctomapPlanner]: No path found");
        res->success = false;
        res->message = "No path found";
        res->path    = std::vector<geometry_msgs::msg::Point>();
        return;
      }
      
      // path not until the end but to a closer point
      std::stringstream ss;
      ss << "Incomplete path found of length = " << path.first.size();
      RCLCPP_INFO_STREAM(this->get_logger(),"[MrsMinimalOctomapPlanner]: " << ss.str());
      res->message = ss.str();

      double front_x = path.first.front().x();
      double front_y = path.first.front().y();
      double front_z = path.first.front().z();

      double back_x = path.first.back().x();
      double back_y = path.first.back().y();
      double back_z = path.first.back().z();

      double dist_path_start_to_end =
          sqrt(pow(front_x - back_x, 2) + pow(front_y - back_y, 2) + pow(front_z - back_z, 2));

      if (dist_path_start_to_end < _min_path_length_) {
        std::stringstream ss;
        ss << "Path too short, length: " << dist_path_start_to_end;
        RCLCPP_WARN_STREAM(this->get_logger(),"[MrsMinimalOctomapPlanner]: " << ss.str());
        res->message = ss.str();
      }
    }

    std::vector<geometry_msgs::msg::Point> tf_path;
    auto                              from_frame = mrs_lib::get_mutexed(mutex_octree_, octree_frame_);
    auto                              to_frame   = req->header.frame_id;
    auto                              ret        = transformer_->getTransform(from_frame, to_frame, clock_->now());

    if (!ret) {
      RCLCPP_ERROR(this->get_logger(), "[MrsMinimalOctomapPlanner]: Failed to transform path from %s to %s", from_frame.c_str(), to_frame.c_str());
      res->success = false;
      res->message = "transform unavailable";
      return;
    }

    auto tf = ret.value();

    for (auto& point : path.first) {

      geometry_msgs::msg::PointStamped tmp_pt;
      tmp_pt.header.stamp    = clock_->now();
      tmp_pt.header.frame_id = from_frame;
      tmp_pt.point.x         = point.x();
      tmp_pt.point.y         = point.y();
      tmp_pt.point.z         = point.z();

      auto transformed_point = transformer_->transform(tmp_pt, tf);

      if (!transformed_point) {
        RCLCPP_ERROR(this->get_logger(), "[MrsMinimalOctomapPlanner]: Failed to transform path point from %s to %s even when TF exists", from_frame.c_str(), to_frame.c_str());
        res->success = false;
        res->message = "point transform failed";
        return;
      }

      tf_path.push_back(transformed_point->point);
    }

    res->success         = true;
    res->header.stamp    = clock_->now();
    res->header.frame_id = to_frame;
    res->path            = tf_path;
    return;
  }



MinimalOctomapPlanner::MinimalOctomapPlanner(const rclcpp::NodeOptions& options) : rclcpp::Node("minimal_octomap_planner", options) {
  timer_init_ = this->create_wall_timer(std::chrono::duration<double>(0.1), std::bind(&MinimalOctomapPlanner::onInit, this));
}

}  // mrs_octomap_planner


#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(mrs_octomap_planner::MinimalOctomapPlanner)