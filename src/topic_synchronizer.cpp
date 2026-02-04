#include <memory>
#include <string>
#include <vector>
#include <map>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "visualization_msgs/msg/marker.hpp"

class TopicHandlerBase {
public:
    virtual ~TopicHandlerBase() = default;
};

template<typename MsgT>
class TypedTopicHandler : public TopicHandlerBase {
public:
    TypedTopicHandler(
        rclcpp::Node* node,
        const std::string& topic_name, 
        const std::string& mode,
        std::shared_ptr<rclcpp::Time> start_time_ref,
        std::shared_ptr<bool> start_time_set_ref,
        int downsampling_factor)
    : node_(node), mode_(mode), start_time_(start_time_ref), start_time_set_(start_time_set_ref), downsampling_factor_(downsampling_factor), msg_counter_(0)
    {
    
        sub_ = node_->create_subscription<MsgT>(
            topic_name, 
            rclcpp::SensorDataQoS(), 
            std::bind(&TypedTopicHandler::callback, this, std::placeholders::_1));
        
        std::string out_topic = "/synced" + topic_name;
        pub_ = node_->create_publisher<MsgT>(out_topic, 10);

        if (topic_name == "/imu/data"){
            xsens_ = true;
            RCLCPP_INFO(node_->get_logger(), "Xsens IMU rilevato, applicando correzione asse Y.");
        }
        
        RCLCPP_INFO(node_->get_logger(), "Bridge creato: %s -> %s (Downsampling: %d)", topic_name.c_str(), out_topic.c_str(), downsampling_factor_);
    }

protected:
    void callback(std::shared_ptr<MsgT> msg) {
        msg_counter_++;
        if ((msg_counter_ % downsampling_factor_) != 0) {
            return; // Skip message
        }

        rclcpp::Time now = node_->now();

        if (mode_ == "now") {
            msg->header.stamp = now;
        } 
        else if (mode_ == "relative") {
            // il primo messaggio in assoluto definisce il tempo zero
            if (!(*start_time_set_)) {
                *start_time_ = now;
                *start_time_set_ = true;
                RCLCPP_INFO(node_->get_logger(), "Tempo zero inizializzato a: %f", start_time_->seconds());
            }

            rclcpp::Duration diff = now - *start_time_;
            
            msg->header.stamp = rclcpp::Time(diff.nanoseconds());
        }

        if constexpr (std::is_same<MsgT, sensor_msgs::msg::Imu>::value) if (xsens_) callback_xsens(msg);

        pub_->publish(*msg);
    }

    void callback_xsens(std::shared_ptr<sensor_msgs::msg::Imu> msg) {
        //msg->linear_acceleration.y *= -1.0;
        //msg->angular_velocity.y *= -1.0;
        //msg->orientation.y *= -1.0;
//
        //// Updating covariances for the Y axis flip
        //msg->linear_acceleration_covariance[1] *= -1.0; // xy
        //msg->linear_acceleration_covariance[3] *= -1.0; // yx
        //msg->linear_acceleration_covariance[5] *= -1.0; // yz
        //msg->linear_acceleration_covariance[7] *= -1.0; // zy
//
        //msg->angular_velocity_covariance[1] *= -1.0;
        //msg->angular_velocity_covariance[3] *= -1.0;
        //msg->angular_velocity_covariance[5] *= -1.0;
        //msg->angular_velocity_covariance[7] *= -1.0;
//
        //msg->orientation_covariance[1] *= -1.0;
        //msg->orientation_covariance[3] *= -1.0;
        //msg->orientation_covariance[5] *= -1.0;
        //msg->orientation_covariance[7] *= -1.0;

        // print ax, ay, az, wx, wy, wz
        RCLCPP_INFO(node_->get_logger(), "IMU Synced: ax: %f, ay: %f, az: %f, wx: %f, wy: %f, wz: %f",
            msg->linear_acceleration.x,
            msg->linear_acceleration.y,
            msg->linear_acceleration.z,
            msg->angular_velocity.x,
            msg->angular_velocity.y,
            msg->angular_velocity.z);

        // keep track of min and max values
        if (msg->angular_velocity.z < min_) min_ = msg->angular_velocity.z;
        if (msg->angular_velocity.z > max_) max_ = msg->angular_velocity.z;
        RCLCPP_INFO(node_->get_logger(), "IMU Z angular velocity min: %f, max: %f", min_, max_);
    }

    rclcpp::Node* node_;
    std::string mode_;
    
    std::shared_ptr<rclcpp::Time> start_time_;
    std::shared_ptr<bool> start_time_set_;
    
    typename rclcpp::Subscription<MsgT>::SharedPtr sub_;
    typename rclcpp::Publisher<MsgT>::SharedPtr pub_;
    
    int downsampling_factor_;
    uint64_t msg_counter_;

    bool xsens_ = false;
    float min_, max_;
};

class TopicSynchronizer : public rclcpp::Node {
public:
    TopicSynchronizer() : Node("topic_synchronizer") {
        RCLCPP_INFO(this->get_logger(), "Inizializzazione nodo TopicSynchronizer...");

        this->declare_parameter("timestamp_mode", "now");
        this->declare_parameter("topic_names", std::vector<std::string>{});
        this->declare_parameter("topic_types", std::vector<std::string>{});
        this->declare_parameter("downsampling_factors", std::vector<int64_t>{});

        std::string mode = this->get_parameter("timestamp_mode").as_string();
        std::vector<std::string> names = this->get_parameter("topic_names").as_string_array();
        std::vector<std::string> types = this->get_parameter("topic_types").as_string_array();
        std::vector<int64_t> downsampling = this->get_parameter("downsampling_factors").as_integer_array();

        // Se downsampling non è specificato, usa 1 per tutti
        if (downsampling.empty()) {
            downsampling.resize(names.size(), 1);
        }

        RCLCPP_INFO(this->get_logger(), "Modalità: %s. Trovati %zu topic da configurare.", mode.c_str(), names.size());

        if (names.size() != types.size() || names.size() != downsampling.size()) {
            RCLCPP_ERROR(this->get_logger(), "Errore: topic_names, topic_types e downsampling_factors devono avere la stessa dimensione!");
            return;
        }

        global_start_time_ = std::make_shared<rclcpp::Time>(0);
        global_start_time_set_ = std::make_shared<bool>(false);

        for (size_t i = 0; i < names.size(); ++i) {
            std::string name = names[i];
            std::string type = types[i];
            int factor = static_cast<int>(downsampling[i]);

            create_handler(name, type, mode, factor);
        }
    }

private:
    void create_handler(const std::string& name, const std::string& type, const std::string& mode, int factor) {
        std::shared_ptr<TopicHandlerBase> handler;

        if (type == "pcl") {
            handler = std::make_shared<TypedTopicHandler<sensor_msgs::msg::PointCloud2>>(
                this, name, mode, global_start_time_, global_start_time_set_, factor);
        } 
        else if (type == "imu") {
            handler = std::make_shared<TypedTopicHandler<sensor_msgs::msg::Imu>>(
                this, name, mode, global_start_time_, global_start_time_set_, factor);
        }
        else if (type == "marker") {
            handler = std::make_shared<TypedTopicHandler<visualization_msgs::msg::Marker>>(
                this, name, mode, global_start_time_, global_start_time_set_, factor);
        }
        else {
             RCLCPP_WARN(this->get_logger(), "Tipo sconosciuto '%s' per il topic '%s'. Ignorato.", type.c_str(), name.c_str());
             return;
        }

        handlers_.push_back(handler);
    }

    std::vector<std::shared_ptr<TopicHandlerBase>> handlers_;
    
    std::shared_ptr<rclcpp::Time> global_start_time_;
    std::shared_ptr<bool> global_start_time_set_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TopicSynchronizer>());
  rclcpp::shutdown();
  return 0;
}
