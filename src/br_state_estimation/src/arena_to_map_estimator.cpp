#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>


#include <tf2_eigen/tf2_eigen.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <eigen3/Eigen/Geometry>
#include <grid_map_cv/GridMapCvConverter.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

#include <cmath>

using namespace std::chrono_literals;
using std::placeholders::_1;

class Arena2Map : public rclcpp::Node {
    public:
    Arena2Map(const std::string node_name="arena_to_map") : Node(node_name), initialized_(false) {
        tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
        transform.reset(new geometry_msgs::msg::TransformStamped());
        transform->header.frame_id = "arena";
        transform->child_frame_id = "map";
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        occupancy_grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", 10, std::bind(&Arena2Map::occupancyGridCallback, this, _1));
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("debug_image", 1);
    }
    private:
    void occupancyGridCallback(nav_msgs::msg::OccupancyGrid::ConstPtr map){
        geometry_msgs::msg::Transform new_tf;
        this->extractTfFromMap(map, new_tf);
        if(tfUpdateNecessary(new_tf, 0.1, M_PI * 5. / 180.)){
            this->transform->transform = new_tf;
            this->transform->header.stamp = this->get_clock()->now();
            tf_broadcaster_->sendTransform(*this->transform);
            RCLCPP_INFO(this->get_logger(), "Updated arena->map transform");
        }
        

    }

    bool extractTfFromMap(nav_msgs::msg::OccupancyGrid::ConstPtr map, geometry_msgs::msg::Transform& tf) {
        grid_map::GridMap grid_map;
        grid_map::GridMapRosConverter::fromOccupancyGrid(*map, "slam", grid_map);
        cv::Mat img, img_thr;
        grid_map::GridMapCvConverter::toImage<unsigned short, 1>(grid_map, "slam", 2, img);
        img.convertTo(img, CV_8U, 1 / 256.0);
        cv::rotate(img, img, cv::RotateFlags::ROTATE_90_CLOCKWISE);
        cv::threshold(img, img_thr, 200, 255, cv::THRESH_BINARY);
        

        std::vector<cv::Point> arena_points;
        for(size_t w = 0; w < img_thr.size().width; w++){
            for(size_t h = 0; h < img_thr.size().height; h++) {
                if(img_thr.at<unsigned char>(h, w) == 255){
                    arena_points.push_back(cv::Point(w, h));
                }
            }
        }
        

        cv::Point2f vtx[4];
        cv::RotatedRect box = cv::minAreaRect(arena_points);
        box.points(vtx);
        for(size_t i = 0; i < 4; i++ )
            cv::line(img_thr, vtx[i], vtx[(i+1)%4], cv::Scalar(150), 1, cv::LINE_AA);
        std::vector<Eigen::Vector2d> box_corners;
        Eigen::Vector2d offset(map->info.origin.position.x, map->info.origin.position.y);
        std::pair<size_t, double> best(0, std::numeric_limits<double>::max());
        for(size_t i = 0u; i < 4; i++){
            Eigen::Vector2d new_point(Eigen::Vector2d::Zero());
            new_point << vtx[i].x, img_thr.size().height - vtx[i].y;
            new_point *= map->info.resolution;
            new_point += offset;
            box_corners.push_back(new_point);
            if(!initialized_){
                // Select closest point to robot start pose (in cartesian sense)
                double metric = new_point.norm();
                if(metric < best.second){
                    best.first = i;
                    best.second = metric;
                }
            }
            else{
                // Select closest point to last estimate
                Eigen::Isometry3d transform_as_eigen = tf2::transformToEigen(transform->transform);
                double metric = (new_point - transform_as_eigen.inverse().translation().block<2,1>(0,0)).norm();
                if(metric < best.second){
                    best.first = i;
                    best.second = metric;
                }
            }
            
        }

        
        Eigen::Vector2d world_origin = box_corners[best.first];
        Eigen::Vector2d x_axis = box_corners[(best.first + 1) % 4] - world_origin;
        double yaw = std::atan2(x_axis.y(), x_axis.x()) - M_PI_2;

        // Workaround since stringstream as argument does not work directly (like in ROS1)
        std::stringstream debug_msg;
        debug_msg <<  "Pose of arena-frame in map-frame [" << world_origin.x() << "m, " << world_origin.y() << "m, " << yaw * 180. / M_PI<< "deg] (format: [x,y,yaw])";
        RCLCPP_DEBUG(this->get_logger(), debug_msg.str().c_str());
        tf.translation.x = world_origin.x();
        tf.translation.y = world_origin.y();
        if(initialized_){
            tf.translation.z = -1 * transform->transform.translation.z;
        }
        else {
            try{
                tf.translation.z = tf_buffer_->lookupTransform(map->header.frame_id, "base_link", tf2::TimePoint(0s)).transform.translation.z;
            }
            catch (tf2::TransformException & ex) {
                RCLCPP_WARN_SKIPFIRST_THROTTLE(
                    get_logger(), *get_clock(), (500ms).count(),
                    "cannot get necessary transform: %s", ex.what());
                return false;
            }
        }
            
        
        tf.rotation.w = std::cos(yaw / 2.0);
        tf.rotation.z = std::sin(yaw / 2.0);
        Eigen::Isometry3d transform_as_eigen = tf2::transformToEigen(tf);
        tf = tf2::eigenToTransform(transform_as_eigen.inverse()).transform;
        
        cv::Mat debug_image;
        cv::cvtColor(img, debug_image, cv::COLOR_GRAY2RGB);
        for(int i = 0; i < 4; i++ )
            cv::line(debug_image, vtx[i], vtx[(i+1)%4], cv::Scalar(255,0,0), 1, cv::LINE_AA);
        cv::drawMarker(debug_image, vtx[best.first], cv::Scalar(0,255,0), cv::MarkerTypes::MARKER_CROSS, 5);
        sensor_msgs::msg::Image::SharedPtr debug_image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", debug_image).toImageMsg();
        this->image_pub_->publish(*debug_image_msg);

        if(! initialized_) initialized_ = true;
        return true;

    }

    bool tfUpdateNecessary(const geometry_msgs::msg::Transform new_tf, const double pos_threshold, const double yaw_threshold){
        
        Eigen::Vector2d new_pos, current_pos;
        new_pos << new_tf.translation.x, new_tf.translation.y;
        current_pos << transform->transform.translation.x, transform->transform.translation.y;
        
        double next_yaw, current_yaw;
        next_yaw = 2. * std::atan2(new_tf.rotation.z, new_tf.rotation.w);
        current_yaw = 2. * std::atan2(transform->transform.rotation.z, transform->transform.rotation.w);
        
        auto constrainAngle = [&](double x) {
            x = fmod(x + M_PI, 2 * M_PI);
            if (x < 0)
                x += 2 * M_PI;
            return x - M_PI;
        };

        return std::fabs(constrainAngle(next_yaw - current_yaw)) > yaw_threshold | (new_pos - current_pos).norm() > pos_threshold;
    }

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_sub_;
    geometry_msgs::msg::TransformStamped::SharedPtr transform;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    std::mutex transform_mutex;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    bool initialized_;
    double z_offset_;


};


int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    Arena2Map::SharedPtr a2m(new Arena2Map);
    rclcpp::spin(a2m);
}