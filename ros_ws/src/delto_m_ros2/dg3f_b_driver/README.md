# dg3f_b_driver ROS 2 Package 🚀

## 📌 Overview

The `dg3f_b_driver` ROS 2 package provides a hardware interface leveraging [ros2_control](https://control.ros.org/) for the DG-3F-B grippers (12 DOF, 3 fingers × 4 joints), enabling direct robotic control operations.

## 📦 Dependency Installation

### Navigate to Workspace
```bash
cd ~/your_ws
```

### Update rosdep
```bash
apt update
rosdep update
```

### Install Specific Dependencies
```bash
rosdep install --from-paths src/DELTO_M_ROS2/dg3f_b_driver --ignore-src -r -y
```

### Verify Installation by Building
```bash
colcon build --packages-select dg3f_b_driver delto_hardware
```

---

## 🚀 Launch Files

| Launch File | Description | Controller Type |
|-------------|-------------|-----------------|
| `dg3f_b_driver.launch.py` | DG3F-B - JointTrajectoryController | Position Control |

---

## 🎛️ Controlling Delto Gripper-3F-B

### 1. Loading DG3F-B controller

Launch the Delto Gripper-3F-B controller with:
```bash
ros2 launch dg3f_b_driver dg3f_b_driver.launch.py delto_ip:=169.254.186.72 delto_port:=502
```

### 2. Test scripts:

| Script | Controller Type | Description |
|--------|-----------------|-------------|
| `dg3f_b_jtc_test.py` | JTC | JointTrajectory based test |
| `dg3f_b_operator_test.py` | Operator | Operator mode test |

**Python Example:**
```bash
ros2 run dg3f_b_driver dg3f_b_jtc_test.py
```

**C++ Example:**
```bash
ros2 run dg3f_b_driver dg3f_b_test_cpp
```

---

## 🔧 Controller Types

### JointTrajectoryController (Default)
- **Purpose**: Smooth trajectory interpolation for position control
- **Joints**: 12 joints (j_dg_1_1 - 1_4, j_dg_2_1 - 2_4, j_dg_3_1 - 3_4)
- **Topic**: `/dg3f_b/delto_controller/joint_trajectory`

---

## 🌐 Namespace

All DG3F-B drivers use the `/dg3f_b/` namespace to avoid topic conflicts with other grippers.

---

## 🤝 Contributing
Contributions are encouraged:

1. Fork repository
2. Create branch (`git checkout -b feature/my-feature`)
3. Commit changes (`git commit -am 'Add my feature'`)
4. Push (`git push origin feature/my-feature`)
5. Open pull request

## 📄 License
BSD-3-Clause

## 📧 Contact
[TESOLLO SUPPORT](mailto:support@tesollo.com)

