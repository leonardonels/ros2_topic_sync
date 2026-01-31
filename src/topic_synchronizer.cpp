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
        std::shared_ptr<bool> start_time_set_ref)
    : node_(node), mode_(mode), start_time_(start_time_ref), start_time_set_(start_time_set_ref)
    {
        sub_ = node_->create_subscription<MsgT>(
            topic_name, 
            rclcpp::SensorDataQoS(), 
            std::bind(&TypedTopicHandler::callback, this, std::placeholders::_1));

        std::string out_topic = "/synced" + topic_name;
        pub_ = node_->create_publisher<MsgT>(out_topic, 10);
        
        RCLCPP_INFO(node_->get_logger(), "Bridge creato: %s -> %s", topic_name.c_str(), out_topic.c_str());
    }

private:
    void callback(std::shared_ptr<MsgT> msg) {
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

        pub_->publish(*msg);
    }

    rclcpp::Node* node_;
    std::string mode_;
    
    std::shared_ptr<rclcpp::Time> start_time_;
    std::shared_ptr<bool> start_time_set_;
    
    typename rclcpp::Subscription<MsgT>::SharedPtr sub_;
    typename rclcpp::Publisher<MsgT>::SharedPtr pub_;
};

class TopicSynchronizer : public rclcpp::Node {
public:
    TopicSynchronizer() : Node("topic_synchronizer") {
        RCLCPP_INFO(this->get_logger(), "Inizializzazione nodo TopicSynchronizer...");

        this->declare_parameter("timestamp_mode", "now");
        this->declare_parameter("topic_names", std::vector<std::string>{});
        this->declare_parameter("topic_types", std::vector<std::string>{});

        std::string mode = this->get_parameter("timestamp_mode").as_string();
        std::vector<std::string> names = this->get_parameter("topic_names").as_string_array();
        std::vector<std::string> types = this->get_parameter("topic_types").as_string_array();

        RCLCPP_INFO(this->get_logger(), "Modalità: %s. Trovati %zu topic da configurare.", mode.c_str(), names.size());

        if (names.size() != types.size()) {
            RCLCPP_ERROR(this->get_logger(), "Errore: topic_names e topic_types devono avere la stessa dimensione!");
            return;
        }

        global_start_time_ = std::make_shared<rclcpp::Time>(0);
        global_start_time_set_ = std::make_shared<bool>(false);

        for (size_t i = 0; i < names.size(); ++i) {
            std::string name = names[i];
            std::string type = types[i];

            create_handler(name, type, mode);
        }
    }

private:
    void create_handler(const std::string& name, const std::string& type, const std::string& mode) {
        std::shared_ptr<TopicHandlerBase> handler;

        if (type == "pcl") {
            handler = std::make_shared<TypedTopicHandler<sensor_msgs::msg::PointCloud2>>(
                this, name, mode, global_start_time_, global_start_time_set_);
        } 
        else if (type == "imu") {
            handler = std::make_shared<TypedTopicHandler<sensor_msgs::msg::Imu>>(
                this, name, mode, global_start_time_, global_start_time_set_);
        }
        else if (type == "marker") {
            handler = std::make_shared<TypedTopicHandler<visualization_msgs::msg::Marker>>(
                this, name, mode, global_start_time_, global_start_time_set_);
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
