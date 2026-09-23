#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <vector>
#include <string>
#include <mutex>

class DrawLetter {
public:
    DrawLetter(std::shared_ptr<rclcpp::Node> node) : node_(node) {
        joint_names_ = {
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        };
        current_joints_ = {0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0};

        joint_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("/pen_marker", 10);

        // Cấu hình Nét mực chữ L
        marker_.header.frame_id = "base_link";
        marker_.ns = "letter_l";
        marker_.id = 0;
        marker_.type = visualization_msgs::msg::Marker::LINE_LIST;
        marker_.action = visualization_msgs::msg::Marker::ADD;
        marker_.scale.x = 0.015; // Nét mực dày 1.5 cm
        marker_.color.r = 1.0;   // Màu Vàng Neon rực rỡ
        marker_.color.g = 0.9;
        marker_.color.b = 0.0;
        marker_.color.a = 1.0;

        // Timer phát liên tục 50Hz để RViz không bao giờ bị mất nét vẽ
        timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&DrawLetter::publishLoop, this)
        );
    }

    void execute() {
        using moveit::planning_interface::MoveGroupInterface;
        rclcpp::sleep_for(std::chrono::seconds(2));

        MoveGroupInterface move_group(node_, "ur_manipulator");
        geometry_msgs::msg::Pose start_pose = move_group.getCurrentPose().pose;

        double h = 0.16;      // Chiều cao thân đứng chữ L: 16cm
        double w = 0.10;      // Chiều rộng nét đáy chữ L: 10cm
        double z_lift = 0.04; // Chiều cao nhấc bút: 4cm

        // P1: Đỉnh trên cùng của chữ L
        geometry_msgs::msg::Pose p1 = start_pose; 
        p1.position.x += h; 
        p1.position.y += w / 2.0;

        // P2: Góc vuông chữ L (giao giữa nét đứng và nét ngang)
        geometry_msgs::msg::Pose p2 = start_pose; 
        p2.position.x += 0.0; 
        p2.position.y += w / 2.0;

        // P3: Điểm kết thúc nét gạch đáy bên phải
        geometry_msgs::msg::Pose p3 = start_pose; 
        p3.position.x += 0.0; 
        p3.position.y -= w / 2.0;

        // ==========================================
        // QUY TRÌNH VẼ CHỮ L
        // ==========================================
        RCLCPP_INFO(node_->get_logger(), "[1] Nhấc bút bay đến đỉnh chữ L (p1)...");
        geometry_msgs::msg::Pose hover_p1 = p1; hover_p1.position.z += z_lift;
        planAndMove(move_group, {start_pose, hover_p1, p1});

        RCLCPP_INFO(node_->get_logger(), "[2] Hạ bút vẽ nét đứng (p1 -> p2)...");
        planAndMove(move_group, {p1, p2});
        addSegment(p1.position, p2.position); // Vẽ vệt mực nét đứng

        RCLCPP_INFO(node_->get_logger(), "[3] Vẽ tiếp nét ngang đáy (p2 -> p3)...");
        planAndMove(move_group, {p2, p3});
        addSegment(p2.position, p3.position); // Vẽ vệt mực nét ngang

        RCLCPP_INFO(node_->get_logger(), "[4] Nhấc bút kết thúc chữ L...");
        geometry_msgs::msg::Pose hover_p3 = p3; hover_p3.position.z += z_lift;
        planAndMove(move_group, {p3, hover_p3});

        RCLCPP_INFO(node_->get_logger(), ">> [XONG] ĐÃ HOÀN THÀNH 100%% VẼ CHỮ L! <<");
    }

private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::string> joint_names_;
    std::vector<double> current_joints_;
    visualization_msgs::msg::Marker marker_;
    std::mutex mutex_;

    void publishLoop() {
        sensor_msgs::msg::JointState js;
        js.header.stamp = node_->now();
        js.name = joint_names_;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            js.position = current_joints_;
        }
        joint_pub_->publish(js);

        {
            std::lock_guard<std::mutex> lock(mutex_);
            marker_.header.stamp = node_->now();
            marker_pub_->publish(marker_);
        }
    }

    void addSegment(const geometry_msgs::msg::Point& pt1, const geometry_msgs::msg::Point& pt2) {
        std::lock_guard<std::mutex> lock(mutex_);
        marker_.points.push_back(pt1);
        marker_.points.push_back(pt2);
    }

    void planAndMove(moveit::planning_interface::MoveGroupInterface& move_group,
                     const std::vector<geometry_msgs::msg::Pose>& waypoints) {
        moveit_msgs::msg::RobotTrajectory trajectory;
        double fraction = move_group.computeCartesianPath(waypoints, 0.005, 0.0, trajectory);
        if (fraction > 0.5 && !trajectory.joint_trajectory.points.empty()) {
            const auto& traj = trajectory.joint_trajectory;
            rclcpp::Time start_time = node_->now();

            for (size_t i = 0; i < traj.points.size(); ++i) {
                const auto& pt = traj.points[i];
                while ((node_->now() - start_time) < pt.time_from_start) {
                    rclcpp::sleep_for(std::chrono::milliseconds(5));
                }
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    for (size_t j = 0; j < traj.joint_names.size(); ++j) {
                        for (size_t k = 0; k < joint_names_.size(); ++k) {
                            if (traj.joint_names[j] == joint_names_[k]) {
                                current_joints_[k] = pt.positions[j];
                            }
                        }
                    }
                }
            }
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    auto move_group_node = rclcpp::Node::make_shared("draw_letter_node", node_options);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread spinner([&executor]() { executor.spin(); });

    DrawLetter drawer(move_group_node);
    drawer.execute();

    // =========================================================================
    // ĐOẠN QUAN TRỌNG: Bắt Node tiếp tục sống, KHÔNG CHO TỰ TẮT
    // =========================================================================
    RCLCPP_INFO(move_group_node->get_logger(), ">> ĐÃ VẼ XONG! Giữ nguyên trạng thái để RViz hiển thị mãi mãi...");
    while (rclcpp::ok()) {
        rclcpp::sleep_for(std::chrono::seconds(1)); // Vòng lặp giữ node luôn hoạt động
    }

    executor.cancel();
    if (spinner.joinable()) {
        spinner.join();
    }
    rclcpp::shutdown();
    return 0;
}