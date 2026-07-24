# BÁO CÁO DỰ ÁN: HỆ THỐNG TAY MÁY PANDA GẮP BULONG DỰA TRÊN YOLOv8-OBB VÀ MoveIt 2

> Hệ thống mô phỏng tay máy Franka Emika Panda 7-DOF trong Gazebo, sử dụng camera + YOLOv8-OBB (Oriented Bounding Box) để phát hiện bulong, tính toán tọa độ và góc xoay, rồi lập quỹ đạo bằng planner OMPL (RRTConnect) của MoveIt 2 để gắp và di chuyển vật.

---

## MỤC LỤC

1. [Cấu trúc chi tiết dự án](#1-cấu-trúc-chi-tiết-dự-án)
2. [Lựa chọn tay máy Panda và so sánh với Unitree Z1](#2-lựa-chọn-tay-máy-panda-và-so-sánh-với-unitree-z1)
3. [Nguyên lý và cách hoạt động của code](#3-nguyên-lý-và-cách-hoạt-động-của-code)
4. [Thuật toán lập quỹ đạo OMPL (RRTConnect)](#4-thuật-toán-lập-quỹ-đạo-ompl-rrtconnect)
5. [Các plugin và controller sử dụng](#5-các-plugin-và-controller-sử-dụng)
6. [Thuật toán điều khiển gắp](#6-thuật-toán-điều-khiển-gắp)

---

## 1. CẤU TRÚC CHI TIẾT DỰ ÁN

Dự án là một ROS 2 workspace (dùng ROS 2 Jazzy) với 4 package chính trong `src/` và một module giao diện người dùng độc lập trong `UI/`.

### 1.1. Sơ đồ tổng thể workspace

```
moveit2_obb/
├── src/
│   ├── panda_moveit_config/     # Cấu hình MoveIt 2, launch, controller, script điều khiển
│   ├── robot_description/       # Mô tả robot (URDF) và camera
│   ├── yolov8_obb/              # Node YOLOv8-OBB (publisher + subscriber hiển thị)
│   └── yolov8_obb_msgs/         # Định nghĩa message tùy chỉnh cho kết quả nhận diện
├── UI/                          # Giao diện PyQt5 chọn bulong cần gắp
│   ├── bolt_selector.py         # Logic chính: hiển thị, tính tọa độ, publish target
│   └── bolt_selector_window.py  # Layout giao diện (sinh từ Qt Designer)
├── build/  install/  log/       # Sản phẩm build của colcon
```

### 1.2. Chi tiết từng package

**`panda_moveit_config/`** — Trái tim của hệ thống điều khiển:

| Thành phần | File | Vai trò |
|---|---|---|
| Script điều khiển | `scripts/arm_control_from_UI.py` | Node `commander` nhận `/target_point`, lập kế hoạch + thực thi chuỗi gắp |
| Launch tổng | `launch/moveit_gazebo_obb.py` | Khởi động Gazebo, spawn robot, MoveItPy, RViz, bridge, các controller |
| Cấu hình pipeline | `config/controller_setting.yaml` | Khai báo pipeline planner (**ompl**, stomp, pilz), chọn **ompl (RRTConnect)** làm mặc định |
| Controller ros2_control | `config/ros2_controllers.yaml` | Định nghĩa `panda_arm_controller`, `panda_hand_controller`, `joint_state_broadcaster` |
| Controller MoveIt | `config/moveit_controllers.yaml`, `gripper_moveit_controllers.yaml` | Ánh xạ MoveIt sang action FollowJointTrajectory / ParallelGripperCommand |
| Động học ngược | `config/kinematics.yaml` | Dùng `KDLKinematicsPlugin` cho nhóm `panda_arm` |
| Giới hạn khớp | `config/joint_limits.yaml` | Giới hạn vận tốc/gia tốc từng khớp |
| Mô hình ngữ nghĩa | `config/panda.srdf` | Định nghĩa nhóm `panda_arm`, `hand`, các trạng thái, cặp link tắt va chạm |
| Mô hình robot | `config/panda.urdf.xacro` | Ghép URDF Panda + camera + plugin Gazebo |

**`robot_description/`** — Mô tả hình học:
- `urdf/panda.urdf` — mô hình Panda gốc.
- `urdf/camera/camera.xacro` + `camera.gazebo` — camera gắn cố định trên `panda_link0`, publish topic `/image_raw`.

**`yolov8_obb/`** — Thị giác máy:
- `scripts/yolov8_obb_publisher.py` — chạy model YOLOv8-OBB (`best.pt`), phát hiện bulong dưới dạng hộp bao định hướng, publish lên `/Yolov8_Inference`.
- `scripts/yolov8_obb_subscriber.py` — vẽ kết quả nhận diện để kiểm tra trực quan.

**`yolov8_obb_msgs/`** — Định nghĩa 2 message:
- `InferenceResult`: `class_name` (tên lớp) + `coordinates` (8 số = 4 đỉnh của OBB).
- `Yolov8Inference`: header + mảng các `InferenceResult`.

### 1.3. Sơ đồ luồng dữ liệu (data flow) giữa các node

```
   [Gazebo Camera] --/image_raw--> [yolov8_obb_publisher] --/Yolov8_Inference--> [UI bolt_selector]
                                                                                        |
                                                                    (người dùng click chọn bulong)
                                                                                        |
                                                                        tính (x, y, góc xoay)
                                                                                        |
                                                                       --/target_point-->
                                                                                        |
                                                              [arm_control_from_UI: node commander]
                                                                                        |
                                                       MoveItPy → OMPL (RRTConnect) → quỹ đạo khớp
                                                                                        |
                                                    --FollowJointTrajectory / GripperCommand-->
                                                                                        |
                                                                [ros2_control ↔ Gazebo]
```

---

## 2. LỰA CHỌN TAY MÁY PANDA VÀ SO SÁNH VỚI UNITREE Z1

### 2.1. Tại sao chọn Franka Emika Panda

Dự án chọn Panda vì các lý do kỹ thuật gắn liền trực tiếp với bài toán gắp bulong bằng thị giác và MoveIt 2:

1. **7 bậc tự do (7-DOF) — có dư (redundancy).** Với 6 ràng buộc của một pose 3D (3 vị trí + 3 hướng), khớp thứ 7 tạo ra một không gian null-space vô hạn nghiệm động học ngược. Điều này cực kỳ có lợi cho planner như OMPL/RRTConnect: khi tay máy cần với xuống gắp bulong ở nhiều vị trí và nhiều góc xoay khác nhau trên bàn, robot dư bậc tự do dễ tránh kỳ dị (singularity), tránh va chạm và tìm được quỹ đạo hợp lệ dễ hơn nhiều so với robot 6-DOF.

2. **Hỗ trợ hệ sinh thái ROS 2 / MoveIt 2 hàng đầu.** Panda là robot "chuẩn" (reference robot) của MoveIt — hầu hết tutorial, config mẫu (SRDF, kinematics, controller) đều dựng sẵn cho Panda. Điều này thấy rõ trong dự án: toàn bộ `panda_moveit_config` kế thừa cấu trúc chuẩn của MoveIt.

3. **Mô hình mô phỏng chất lượng cao.** Có sẵn URDF/mesh, thông số quán tính, giới hạn khớp chính xác — cần thiết cho mô phỏng vật lý DART trong Gazebo (dự án dùng `gz-physics-dartsim-plugin`).

4. **Cảm biến mô-men ở cả 7 khớp.** Panda có torque sensor tại mỗi khớp, cho phép điều khiển tuân thủ (compliance) và gắp an toàn — quan trọng khi thao tác vật nhỏ như bulong.

5. **Gripper song song tích hợp sẵn.** Franka Hand là gripper 2 ngón song song, khớp với `parallel_gripper_action_controller` mà dự án dùng để đóng/mở gắp.

### 2.2. Bảng so sánh Panda vs Unitree Z1

> Các thông số dưới đây là số liệu công bố phổ biến của nhà sản xuất; giá trị có thể thay đổi theo phiên bản.

| Tiêu chí | Franka Emika Panda | Unitree Z1 | Ý nghĩa với dự án |
|---|---|---|---|
| **Số bậc tự do** | 7-DOF | 6-DOF | Panda có dư bậc tự do → planner dễ tránh va chạm/kỳ dị, quỹ đạo mượt hơn |
| **Tầm với (reach)** | ~855 mm | ~740 mm | Panda phủ vùng làm việc rộng hơn trên bàn |
| **Tải trọng (payload)** | ~3 kg | ~3 kg (danh định, ~2 kg ở tầm với tối đa) | Cả hai đều thừa sức gắp bulong (vài gram) |
| **Độ lặp lại** | ±0.1 mm | ±0.1 mm (công bố) | Panda có danh tiếng chính xác đã kiểm chứng lâu năm |
| **Khối lượng cánh tay** | ~18 kg | ~4.3 kg | Z1 nhẹ, dễ gắn di động; Panda nặng, cứng vững hơn |
| **Cảm biến mô-men khớp** | Có ở cả 7 khớp | Có (điều khiển lực/mô-men) | Cả hai hỗ trợ compliance; Panda nhiều khớp cảm biến hơn |
| **Giá thành** | Cao (~20.000–30.000 USD) | Thấp hơn nhiều (~4.500–6.000 USD) | Z1 là lợi thế lớn về chi phí |
| **Hỗ trợ MoveIt 2 / ROS 2** | Rất mạnh, là robot chuẩn của MoveIt | Có SDK riêng, hỗ trợ ROS nhưng ít hoàn thiện hơn cho MoveIt | **Lý do quyết định chọn Panda cho dự án dùng MoveIt** |
| **Tài nguyên mô phỏng (URDF/mesh/config)** | Đầy đủ, sẵn sàng | Ít hơn, phải tự dựng nhiều | Rút ngắn thời gian phát triển |

### 2.3. Kết luận lựa chọn

Với bài toán **nghiên cứu/mô phỏng gắp vật bằng thị giác trên MoveIt 2**, Panda là lựa chọn tối ưu nhờ **7-DOF dư bậc tự do + hệ sinh thái MoveIt hoàn chỉnh**, giúp tập trung vào thuật toán nhận diện và lập quỹ đạo thay vì mất thời gian tích hợp phần cứng/mô hình. Unitree Z1 phù hợp hơn khi ưu tiên **chi phí thấp và tính di động** cho triển khai thực tế, nhưng đánh đổi bằng việc thiếu bậc tự do dư và hệ công cụ MoveIt kém hoàn thiện hơn.

---

## 3. NGUYÊN LÝ VÀ CÁCH HOẠT ĐỘNG CỦA CODE

Hệ thống gồm 3 phân hệ chạy song song, giao tiếp qua ROS 2 topic: **thị giác (YOLOv8-OBB)**, **giao diện chọn mục tiêu (UI PyQt5)**, và **điều khiển tay máy (MoveItPy)**.

### 3.1. Phân hệ thị giác — YOLOv8-OBB

File `yolov8_obb_publisher.py`:

```python
self.model = YOLO(os.environ['HOME'] + '/moveit2_obb/src/yolov8_obb/scripts/best.pt')
...
def camera_callback(self, data):
    img = bridge.imgmsg_to_cv2(data, "bgr8")
    results = self.model(img, conf = 0.90)   # chỉ nhận kết quả tự tin > 90%
    for r in results:
        if(r.obb is not None):
            boxes = r.obb
            for box in boxes:
                b = box.xyxyxyxy[0]...        # 4 đỉnh của hộp bao định hướng
                self.inference_result.coordinates = copy.copy(a[0].tolist())
```

Nguyên lý:
- Node subscribe topic `/image_raw` (ảnh từ camera Gazebo).
- Chạy model YOLOv8-OBB đã huấn luyện (`best.pt`) với ngưỡng tin cậy 0.90.
- Điểm mấu chốt: dùng **OBB (Oriented Bounding Box)** thay vì bounding box thẳng thông thường. OBB trả về **4 đỉnh** của hộp bao **có góc xoay**, nhờ đó xác định được không chỉ vị trí mà cả **hướng nằm của bulong** — thông tin sống còn để xoay gripper cho khớp.
- Kết quả (tên lớp + 8 tọa độ 4 đỉnh) được publish lên `/Yolov8_Inference`.

### 3.2. Phân hệ giao diện — UI chọn bulong

File `UI/bolt_selector.py`. Đây là node trung gian giữa thị giác và điều khiển:
- Hiển thị ảnh camera realtime, vẽ đè các OBB nhận được từ `/Yolov8_Inference`.
- Người dùng **rê chuột** tới một bulong (khoảng cách tới tâm hộp < 15 px thì hộp chuyển sang màu đỏ), rồi **click** để chọn.
- Khi click, UI **tính tọa độ thực (x, y) và góc xoay** của bulong (chi tiết ở [mục 6](#6-thuật-toán-điều-khiển-gắp)), đóng gói thành `Float64MultiArray` gồm `[x, y, góc]` và publish lên `/target_point`.

### 3.3. Phân hệ điều khiển — MoveItPy commander

File `arm_control_from_UI.py`. Đây là node `commander`:

```python
self.panda = MoveItPy(node_name="moveit_py")
self.panda_arm = self.panda.get_planning_component("panda_arm")
self.panda_hand = self.panda.get_planning_component("hand")
self.subscription = self.create_subscription(Float64MultiArray, '/target_point', self.listener_callback, 10)
```

Nguyên lý hoạt động:
- Khởi tạo `MoveItPy` (API Python của MoveIt 2) với 2 planning component: `panda_arm` (cánh tay) và `hand` (gripper).
- Subscribe `/target_point`. Mỗi khi UI gửi mục tiêu, callback `listener_callback` chạy **toàn bộ chuỗi gắp** (pick-and-place, xem [mục 6.3](#63-chuỗi-hành-động-gắp-pick-and-place)).
- Dùng `MultiThreadedExecutor` chạy trên luồng riêng để node không bị block trong lúc thực thi quỹ đạo.

**Hàm lõi `plan_and_execute`** — mọi chuyển động đều đi qua đây:
1. `set_start_state_to_current_state()` — lấy trạng thái khớp hiện tại làm điểm xuất phát.
2. **Kẹp giá trị khớp về trong giới hạn (clamp).** Đoạn code duyệt qua từng khớp, nếu vị trí hiện tại vượt biên (do sai số mô phỏng) thì kéo về `[min+ε, max-ε]`. Đây là cách xử lý lỗi `start_state_max_bounds_error` thường gặp khi robot ở sát giới hạn khớp.
3. Gọi `planning_component.plan()` → planner (OMPL/RRTConnect) sinh quỹ đạo.
4. Nếu thành công: `robot.execute(trajectory)` gửi quỹ đạo xuống controller.

### 3.4. Cách khởi động toàn hệ thống (launch)

`moveit_gazebo_obb.py` dựng theo thứ tự:
1. Đặt `GZ_SIM_RESOURCE_PATH`, khởi động **Gazebo** với world `arm_on_the_table` và physics engine **DART**.
2. Xử lý xacro → sinh URDF, **spawn robot** vào Gazebo tại vị trí `(0.05, 0, 1.02)`.
3. Dựng `MoveItConfigsBuilder` — nạp URDF, SRDF, cấu hình controller và `controller_setting.yaml`.
4. Chạy node `arm_control_from_UI.py` với `use_sim_time=True`.
5. Chạy `robot_state_publisher`, `rviz2`, `static_transform_publisher` (world → panda_link0).
6. Chạy `ros_gz_bridge` cầu nối `/image_raw` giữa Gazebo và ROS 2.
7. Spawn 3 controller: `panda_arm_controller`, `panda_hand_controller`, `joint_state_broadcaster`.

---

## 4. THUẬT TOÁN LẬP QUỸ ĐẠO OMPL (RRTConnect)

### 4.1. OMPL và RRTConnect là gì

**OMPL (Open Motion Planning Library)** là thư viện lập quỹ đạo dựa trên **lấy mẫu (sampling-based)** — pipeline planner mặc định và phổ biến nhất của MoveIt. OMPL không tự tính đường đi bằng công thức hình học mà **lấy mẫu ngẫu nhiên các cấu hình khớp** trong không gian cấu hình (C-space), kiểm tra va chạm, rồi nối các mẫu hợp lệ thành một đường đi từ trạng thái đầu tới trạng thái đích.

Dự án dùng thuật toán cụ thể **RRTConnect** (`RRTConnectkConfigDefault`):
- **RRT (Rapidly-exploring Random Tree)** mở rộng một cây ngẫu nhiên phủ dần không gian cấu hình.
- **RRTConnect** là biến thể **hai cây**: một cây mọc từ trạng thái đầu, một cây mọc từ trạng thái đích, rồi cố gắng nối hai cây lại. Cách này thường **nhanh hơn nhiều** RRT một cây và là lựa chọn mặc định kinh điển của MoveIt cho tay máy.

Cấu hình planner trong dự án (`ompl_planning.yaml`):

```yaml
planning_plugins:
  - ompl_interface/OMPLPlanner
planner_configs:
  RRTConnectkConfigDefault:
    type: geometric::RRTConnect
    range: 0.0        # bước mở rộng cây tối đa; 0.0 = tự đặt khi setup()
```

Tham số plan request (`controller_setting.yaml`):

```yaml
plan_request_params:
  planning_attempts: 1
  planning_pipeline: ompl
  planner_id: RRTConnectkConfigDefault
  max_velocity_scaling_factor: 0.5
  max_acceleration_scaling_factor: 0.5
  planning_time: 1.0            # ngân sách thời gian tìm đường (giây)
```

### 4.2. Nguyên lý hoạt động của RRTConnect (từng bước)

1. **Khởi tạo hai cây:** cây `T_start` bắt đầu tại cấu hình khớp hiện tại, cây `T_goal` bắt đầu tại cấu hình đích (từ IK của pose mục tiêu).
2. **Lấy mẫu ngẫu nhiên:** sinh một cấu hình khớp ngẫu nhiên `q_rand` trong không gian cấu hình.
3. **Mở rộng (extend):** tìm nút gần `q_rand` nhất trong `T_start`, kéo một bước (`range`) về phía `q_rand`, kiểm tra va chạm — nếu hợp lệ thì thêm nút mới vào cây.
4. **Nối (connect):** cây còn lại (`T_goal`) cố gắng mọc **liên tục** về phía nút vừa thêm cho tới khi chạm hoặc gặp vật cản.
5. **Đổi vai trò hai cây** rồi lặp lại. Khi hai cây gặp nhau → tìm được một đường đi hợp lệ nối đầu ↔ cuối.
6. **Trích đường đi** và chuyển sang bước hậu xử lý (response adapters) để làm mượt và gán lại thời gian.

Đặc điểm cốt lõi: RRTConnect chỉ cần **tìm ra một đường đi khả thi (feasibility)**, không tối ưu độ mượt. Đường đi thô thường **gấp khúc**, nên MoveIt chạy thêm các response adapter (làm mượt + Time-Optimal Parameterization) để tạo quỹ đạo thực thi được.

### 4.3. Tại sao dự án dùng OMPL / RRTConnect

Trong `controller_setting.yaml`, dự án khai báo 3 pipeline và đặt **ompl (RRTConnect) làm mặc định**:

```yaml
planning_pipelines:
  pipeline_names: ["ompl", "stomp", "pilz_industrial_motion_planner"]
plan_request_params:
  planning_pipeline: ompl
  planner_id: RRTConnectkConfigDefault
```

Lý do phù hợp với bài toán gắp bulong:
- **Xác suất hoàn chỉnh (probabilistically complete).** Nếu tồn tại đường đi thì RRTConnect gần như chắc chắn tìm ra khi cho đủ thời gian — độ tin cậy cao khi tay máy phải với tới nhiều vị trí/góc khác nhau trên bàn.
- **Nhanh và nhẹ.** Kiểu hai cây nối nhau hội tụ rất nhanh; với `planning_time: 1.0 s` là đủ cho các pose gắp/thả trong dự án.
- **Chuẩn mực và ổn định của MoveIt.** RRTConnect là planner mặc định được kiểm chứng lâu năm cho Panda — cấu hình sẵn, ít lỗi tích hợp.
- **Tận dụng dư bậc tự do của Panda 7-DOF.** Không gian cấu hình rộng giúp cây dễ tìm được nhánh né vật cản (bàn, hộp đích).

> **Lưu ý:** dự án vẫn giữ **STOMP** (tối ưu quỹ đạo ngẫu nhiên) và **PILZ** (sinh quỹ đạo công nghiệp) trong danh sách pipeline để dự phòng / so sánh. Có thể chuyển planner mặc định chỉ bằng cách đổi `planning_pipeline` và `planner_id` trong `controller_setting.yaml` mà không sửa code.

### 4.4. Bảng so sánh RRTConnect với các thuật toán khác

| Thuật toán | Nhóm | Nguyên lý | Ưu điểm | Nhược điểm | Có trong dự án |
|---|---|---|---|---|---|
| **RRTConnect** (OMPL) | Dựa trên lấy mẫu (sampling) | Hai cây ngẫu nhiên mọc từ đầu/cuối rồi nối nhau | Xác suất hoàn chỉnh, rất nhanh, ổn định, không cần gradient | Quỹ đạo gấp khúc (cần hậu xử lý làm mượt), kết quả không lặp lại | **Có (mặc định)** |
| **STOMP** | Tối ưu ngẫu nhiên (stochastic optimization) | Nhiễu quỹ đạo + cập nhật theo xác suất | Quỹ đạo mượt sẵn, không cần gradient, xử lý chi phí không khả vi | Có thể kẹt cực tiểu cục bộ, không đảm bảo tìm ra nghiệm | Có (dự phòng) |
| **CHOMP** | Tối ưu dựa trên gradient | Giảm gradient hàm chi phí trơn + va chạm | Quỹ đạo rất mượt, hội tụ nhanh khi khởi tạo tốt | **Cần gradient**, dễ kẹt cực tiểu cục bộ, nhạy với khởi tạo | Có config (`chomp_planning.yaml`) nhưng không dùng |
| **PILZ (PTP/LIN/CIRC)** | Sinh quỹ đạo công nghiệp | Nội suy hình học xác định (điểm–điểm, đường thẳng, cung tròn) | Quỹ đạo xác định, lặp lại được, phù hợp công nghiệp | Không tránh va chạm chủ động, kém linh hoạt với chướng ngại | Có (dự phòng) |
| **TrajOpt** | Tối ưu lồi tuần tự | Sequential convex optimization | Hiệu quả với ràng buộc phức tạp | Cần gradient, cấu hình phức tạp | Có config (`trajopt_planning.yaml`) |
| **LERP** | Nội suy tuyến tính | Nội suy thẳng giữa các điểm | Đơn giản, nhanh | Không tránh va chạm, chỉ dùng thử nghiệm | Có config (`lerp_planning.yaml`) |

**Tóm tắt lựa chọn:** RRTConnect được ưu tiên vì **tìm đường nhanh, tin cậy (xác suất hoàn chỉnh) và là chuẩn mực ổn định của MoveIt cho Panda**. Đổi lại, quỹ đạo thô gấp khúc nên MoveIt tự làm mượt và gán lại thời gian bằng các response adapter (mục 5.3). Dự án vẫn giữ STOMP và PILZ trong pipeline để dự phòng và đối chiếu khi cần quỹ đạo mượt hơn hoặc quỹ đạo hình học xác định.

---

## 5. CÁC PLUGIN VÀ CONTROLLER SỬ DỤNG

### 5.1. Plugin lập kế hoạch (planning plugins)

| Plugin | Khai báo tại | Vai trò |
|---|---|---|
| `ompl_interface/OMPLPlanner` (RRTConnect) | `ompl_planning.yaml` | Planner chính — lấy mẫu ngẫu nhiên, tìm đường nhanh |
| `stomp_moveit/StompPlanner` | `stomp_planning.yaml` | Planner dự phòng — tối ưu quỹ đạo ngẫu nhiên |
| `pilz_industrial_motion_planner` | pipeline `pilz` | Sinh quỹ đạo hình học công nghiệp |

### 5.2. Plugin động học ngược (IK / kinematics)

`config/kinematics.yaml`:
```yaml
panda_arm:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.05
```
- Dùng **KDL (Kinematics and Dynamics Library)** — bộ giải IK số học tổng quát, ổn định cho chuỗi động học 7-DOF.

### 5.3. Request/Response adapters (bộ tiền/hậu xử lý của OMPL)

Từ `ompl_planning.yaml`:

**Request adapters** (chạy *trước* khi lập kế hoạch):
- `ResolveConstraintFrames` — chuẩn hóa hệ quy chiếu của ràng buộc.
- `ValidateWorkspaceBounds` — kiểm tra không gian làm việc.
- `CheckStartStateBounds` — kiểm tra trạng thái đầu có trong giới hạn khớp (liên quan trực tiếp tới đoạn clamp khớp trong `plan_and_execute`).
- `CheckStartStateCollision` — kiểm tra trạng thái đầu không va chạm.

**Response adapters** (chạy *sau* khi có quỹ đạo):
- `AddTimeOptimalParameterization` — gán lại thời gian tối ưu theo giới hạn vận tốc/gia tốc (Time-Optimal Trajectory Generation).
- `ValidateSolution` — kiểm chứng quỹ đạo cuối.
- `DisplayMotionPath` — hiển thị quỹ đạo lên RViz.

### 5.4. Controller (ros2_control)

`config/ros2_controllers.yaml` — chạy ở tần số `update_rate: 100 Hz`:

| Controller | Kiểu (type) | Điều khiển | Ghi chú |
|---|---|---|---|
| `panda_arm_controller` | `joint_trajectory_controller/JointTrajectoryController` | 7 khớp cánh tay (`panda_joint1..7`) | Nhận quỹ đạo, giao diện lệnh `position`, phản hồi `position` + `velocity` |
| `panda_hand_controller` | `parallel_gripper_action_controller/GripperActionController` | Khớp gripper `panda_finger_joint1` | `max_effort: 40.0`, `max_velocity: 0.2`, cho phép "stall" khi kẹp chặt vật |
| `joint_state_broadcaster` | `joint_state_broadcaster/JointStateBroadcaster` | (chỉ đọc) | Phát trạng thái khớp lên `/joint_states` |

### 5.5. Ánh xạ MoveIt → controller

`config/moveit_controllers.yaml` và `gripper_moveit_controllers.yaml`:
- `moveit_simple_controller_manager/MoveItSimpleControllerManager` là trình quản lý.
- `panda_arm_controller` → action `follow_joint_trajectory` (kiểu `FollowJointTrajectory`).
- `panda_hand_controller` → action `gripper_cmd` (kiểu `ParallelGripperCommand`).

### 5.6. Plugin Gazebo (mô phỏng)

Từ `panda.urdf.xacro`:

| Plugin | Vai trò |
|---|---|
| `gz_ros2_control-system` | Cầu nối ros2_control ↔ Gazebo (thực thi lệnh khớp trong mô phỏng) |
| `gz-sim-joint-state-publisher-system` | Phát trạng thái khớp từ mô phỏng |
| `gz-sim-pose-publisher-system` | Phát pose các link |
| `ignition-gazebo-sensors-system` (ogre2) | Render camera |
| Camera sensor | `horizontal_fov=1.8`, ảnh 640×480, publish `/image_raw` |

---

## 6. THUẬT TOÁN ĐIỀU KHIỂN GẮP

Đây là phần lõi biến kết quả nhận diện 2D thành lệnh gắp 3D. Toàn bộ logic tính toán nằm trong `UI/bolt_selector.py`, phần thực thi nằm trong `arm_control_from_UI.py`.

### 6.1. Tính tọa độ gắp của bulong (2D ảnh → 3D thế giới)

**Cấu hình camera** (đồng bộ giữa `panda.urdf.xacro` và code UI):
- Camera gắn cố định trên `panda_link0` tại `xyz = (0.2, 0.6, 0.7)`, xoay `rpy = (0, π/2, 0)` → **nhìn thẳng xuống bàn** (top-down).
- Thông số nội tại (intrinsics) trong UI:
  ```python
  self.fx = 253.936;  self.fy = 253.936   # tiêu cự (pixel)
  self.cx = 320;      self.cy = 240        # tâm ảnh (640×480)
  self.z = 0.7                             # khoảng cách camera→mặt bàn
  self.init_x = 0.2;  self.init_y = 0.6    # đúng bằng vị trí camera trên link0
  ```
  > Kiểm chứng: `fx = (width/2) / tan(fov/2) = 320 / tan(0.9) ≈ 253.9` — khớp với `horizontal_fov = 1.8` trong camera Gazebo.

**Bước 1 — tìm tâm bulong trên ảnh.** Lấy trung bình 4 đỉnh OBB:
```python
points = np.array(r.coordinates).astype(np.int32).reshape([4, 2])
middle_point = np.sum(points, 0) / 4     # (u, v) tâm hộp trên ảnh
```

**Bước 2 — chiếu ngược pixel → tọa độ robot (mô hình pinhole).** Vì camera nhìn thẳng xuống với độ sâu cố định `z = 0.7 m`:
```python
self.target_point[0] = -self.z * (middle_point[1] - self.cy) / self.fy + self.init_x   # X robot
self.target_point[1] = -self.z * (middle_point[0] - self.cx) / self.fx + self.init_y   # Y robot
```
Giải thích:
- `(u - cx)`, `(v - cy)` là độ lệch pixel so với tâm ảnh.
- Nhân `z/f` để đổi từ pixel sang mét trên mặt phẳng bàn (mô hình lỗ kim).
- Dấu trừ và hoán đổi trục (dùng `v` cho X, `u` cho Y) là do **hướng đặt camera** (xoay π/2 quanh trục Y): trục ảnh không trùng trục robot.
- Cộng `init_x`, `init_y` để dịch từ hệ camera về hệ gốc robot `panda_link0`.

Kết quả `target_point[0], target_point[1]` là tọa độ (X, Y) mà đầu gripper cần đến, tính trong `panda_link0`.

### 6.2. Thuật toán xoay gripper (tính góc xoay)

Bulong nằm nghiêng tùy ý trên bàn, gripper phải xoay để kẹp đúng theo **cạnh dài** của bulong. Đây là lý do dùng OBB thay vì box thường.

**Bước 1 — xác định cạnh dài của OBB.** So sánh hai cạnh kề của hộp:
```python
dist1 = khoảng_cách(points[0], points[1])   # cạnh 0-1
dist2 = khoảng_cách(points[1], points[2])   # cạnh 1-2
```

**Bước 2 — tính góc nghiêng của cạnh dài** bằng `atan2` (có xử lý chia cho 0 → góc = π/2):
```python
if dist1 > dist2:                                    # cạnh 0-1 dài hơn
    angle = atan2(points[0][1]-points[1][1], points[0][0]-points[1][0])
else:                                                # cạnh 1-2 dài hơn
    angle = atan2(points[1][1]-points[2][1], points[1][0]-points[2][0])
```
→ `angle` là hướng của **trục dài** bulong trong mặt phẳng ảnh.

**Bước 3 — đổi sang góc lệnh gripper:**
```python
self.target_point[2] = math.pi/2 - angle
```
Lấy `π/2 − angle` vì gripper cần **kẹp vuông góc với trục dài** của bulong (ngón kẹp bám hai cạnh dài, khép lại theo chiều ngắn).

**Bước 4 — áp dụng góc vào hướng end-effector.** Trong `arm_control_from_UI.py`, góc này cộng vào orientation của pose mục tiêu:
```python
self.init_angle = -0.3825   # offset hiệu chỉnh góc gốc của gripper
self.move_to(data.data[0], data.data[1], self.height,
             1.0, self.init_angle + data.data[2], 0.0, 0.0)
#            x,   y,        z,     qx,  qy,                      qz,  qw
```
Góc xoay được đưa vào thành phần `orientation.y` của quaternion pose (cùng offset `init_angle` để hiệu chỉnh tư thế mặc định của Franka Hand).

### 6.3. Chuỗi hành động gắp (pick-and-place)

Khi nhận `/target_point`, `listener_callback` chạy tuần tự (các độ cao lấy từ hằng số trong node):

```python
self.height = 0.18            # độ cao tiếp cận (phía trên bulong)
self.pick_height = 0.113      # độ cao hạ xuống để gắp
self.carrying_height = 0.3    # độ cao nâng lên khi mang vật
self.box_x = 0.3; self.box_y = -0.3   # vị trí hộp đích để thả
```

| Bước | Hành động | Mô tả |
|---|---|---|
| 1 | `move_to(x, y, 0.18, ...)` | Di chuyển tới **phía trên** bulong (approach), đã xoay đúng góc |
| 2 | `gripper_action("open")` | Mở gripper (`panda_finger_joint1 = 0.03`) |
| 3 | `move_to(x, y, 0.113, ...)` | **Hạ xuống** đúng độ cao gắp |
| 4 | `gripper_action("close")` | **Đóng gripper** (`= 0.001`) kẹp bulong |
| 5 | `move_to(x, y, 0.3, ...)` | **Nâng vật** lên độ cao vận chuyển |
| 6 | `move_to(0.3, -0.3, 0.3, ...)` | Di chuyển tới **hộp đích** |
| 7 | `gripper_action("open")` | **Thả** bulong vào hộp |

Mỗi lệnh `move_to` dựng một `PoseStamped` trong frame `panda_link0`, gọi `set_goal_state(pose_link="panda_link8")`, rồi `plan_and_execute` — tức mỗi bước đều được OMPL/RRTConnect lập quỹ đạo và controller thực thi độc lập.

**Điều khiển gripper** (`gripper_action`) dùng cơ chế khác cánh tay: thay vì đặt pose, nó dựng một `RobotState` với giá trị khớp ngón mong muốn, tạo `joint_constraint` bằng `construct_joint_constraint`, rồi để planner của nhóm `hand` đưa ngón kẹp tới giá trị đó.

---

## PHỤ LỤC: TÓM TẮT KIẾN TRÚC MỘT DÒNG

> Camera Gazebo → YOLOv8-OBB phát hiện bulong (4 đỉnh có hướng) → UI PyQt5 để người dùng click chọn → tính (X, Y) bằng mô hình pinhole và góc xoay từ cạnh dài OBB → publish `/target_point` → node `commander` (MoveItPy) chạy chuỗi pick-and-place, mỗi bước lập quỹ đạo bằng **OMPL (RRTConnect)** và thực thi qua **ros2_control** trên **Panda 7-DOF** trong Gazebo.
