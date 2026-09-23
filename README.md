```markdown
## 🚀 Hướng dẫn cài đặt và khởi chạy

### 1. Cài đặt các gói phụ thuộc (Dependencies)
Mở terminal và đảm bảo đã cài đặt đầy đủ các gói của Universal Robots và MoveIt 2:
```bash
sudo apt update
sudo apt install ros-humble-ur ros-humble-moveit -y
```

### 2. Tải và Build Package
Đặt package vào thư mục `src` của ROS 2 workspace (ví dụ: `~/ros2_ws`):
```bash
cd ~/ros2_ws
colcon build --packages-select ur_draw_letter
source install/setup.bash
```

### 3. Khởi chạy chương trình
Chạy file launch tích hợp duy nhất để khởi động toàn bộ hệ thống:
```bash
ros2 launch ur_draw_letter draw.launch.py
```

### 4. Hiện đường chữ sau khi thực thi
Trong trường họp topic /pen_marker chưa hiện trong bảng bên trái, chọn Add -> Marker -> chọn topic /pen_marker
