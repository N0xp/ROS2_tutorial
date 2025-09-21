# ROS 2 Tutorial

A beginner-friendly guide to creating packages in ROS 2 using command-line tools and understanding essential ROS 2 concepts.

---

## Table of Contents

- [Introduction](#introduction)
- [Prerequisites](#prerequisites)
- [Creating Packages in ROS 2](#creating-packages-in-ros-2)
- [Command Explanation](#command-explanation)
- [Dependencies](#dependencies)
- [ROS 2: `rclcpp::spin()` vs `rclcpp::spin_some()` — What's the Difference?](#ROS 2: `rclcpp::spin()` vs `rclcpp::spin_some()` — What's the Difference?)
- [Resources](#resources)
- [License](#license)

---

## Introduction

This tutorial demonstrates how to create a basic ROS 2 package using the `ros2` CLI. It walks you through the process of setting up a package with dependencies in C++.

---

## Prerequisites

Before beginning, ensure you have the following:

- ROS 2 installed (e.g., Humble, Iron, or later)
- Working knowledge of terminal and basic C++ or Python
- A ROS 2 workspace (`colcon`-based)

---

## Creating Packages in ROS 2

Use the following command to create a new C++ package named `ws_tutorial`:

```bash
ros2 pkg create --build-type ament_cmake ws_tutorial --dependencies rclcpp std_msgs
```

This creates a ROS 2 package with the specified dependencies using the `ament_cmake` build system.

---

## Command Explanation

| Command Part                      | Description                                                                 |
|----------------------------------|-----------------------------------------------------------------------------|
| `ros2`                           | Invokes the ROS 2 command-line interface                                   |
| `pkg create`                     | Command to create a new package                                            |
| `--build-type ament_cmake`       | Specifies the build system type (use `ament_cmake` for C++ packages)       |
| `ws_tutorial`                    | Name of the new package                                                    |
| `--dependencies rclcpp std_msgs` | Lists the required dependencies for the package                            |

---

## Dependencies

- **rclcpp**: C++ client library for ROS 2
- **std_msgs**: Standard message definitions, such as `std_msgs/String`

Make sure these are available and sourced in your ROS 2 environment.

---

## 🔄 ROS 2: `rclcpp::spin()` vs `rclcpp::spin_some()` — What's the Difference?

In ROS 2 (C++), both `rclcpp::spin()` and `rclcpp::spin_some()` are used to process callbacks such as subscriptions, timers, services, and actions. However, they behave differently and are suited to different use cases.

### 🧠 Summary Table

| Feature                      | `rclcpp::spin()`                          | `rclcpp::spin_some()`                        |
|-----------------------------|-------------------------------------------|---------------------------------------------|
| **Blocking behavior**       | ✅ Blocks the thread                      | ❌ Non-blocking, returns immediately        |
| **Callback handling**       | Continuously processes callbacks          | Processes only available callbacks once     |
| **Loop needed?**            | ❌ No user loop required                  | ✅ Must be placed inside a `while` loop     |
| **Custom logic control**    | ❌ Harder to integrate with manual logic  | ✅ Easy to combine with publishers, sensors |
| **Performance**             | ✅ More efficient in pure ROS callback nodes | ⚠️ Slight overhead but more control      |
| **Thread usage**            | Holds thread until shutdown               | Returns control, usable in mixed systems    |
| **Best use case**           | Subscribers, services, action servers     | Publishers, sensor loops, hybrid systems    |

---

### 🔁 `rclcpp::spin(node)`

```cpp
rclcpp::spin(node);  // Blocks until rclcpp::shutdown() is called
```
## Resources

- [ROS 2 Documentation](https://docs.ros.org/en/rolling/index.html)
- [Creating Your First ROS 2 Package (Official Guide)](https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)

---
