## 📖 完整的 CiA402 电机控制实现

本仓库现已包含**完整的 CiA402 标准电机驱动控制框架**。

### 快速开始（推荐）

**新用户请从这里开始**（只需 10 分钟）：
```bash
# 1. 查看快速参考卡
cat docs/CIA402_QUICK_REFERENCE.md

# 2. 查看演示代码
cat src/lifter_ecat/src/lifter_node.cpp | head -100

# 3. 编译和运行
cd /home/nvidia/codeSpace/Brace
colcon build --packages-select lifter_ecat
timeout 15 install/lifter_ecat/lib/lifter_ecat/lifter_node
```

### 📚 文档导航

- **快速参考** (10 分钟)：[`docs/CIA402_QUICK_REFERENCE.md`](docs/CIA402_QUICK_REFERENCE.md)
  - PDO 映射速查表
  - 常用控制字和工作模式
  - 常见代码片段

- **完整 API 文档** (1 小时)：[`docs/CIA402_CONTROL_GUIDE.md`](docs/CIA402_CONTROL_GUIDE.md)
  - 每个方法的详细文档
  - PDO 结构详解
  - 使用示例和流程

- **实现总结** (2 小时)：[`docs/CIA402_IMPLEMENTATION_SUMMARY.md`](docs/CIA402_IMPLEMENTATION_SUMMARY.md)
  - 功能清单
  - 技术细节
  - 常见问题解答

- **项目介绍**：[`docs/README_CIA402.md`](docs/README_CIA402.md)
  - 项目结构
  - 快速任务指南
  - 性能指标

- **⚠️ PDO 配置问题解决方案** (重要)：[`PDO_CONFIGURATION_SOLUTION.md`](PDO_CONFIGURATION_SOLUTION.md)
  - 为什么 PDO 缓冲区需要手动配置
  - 完整的解决方案步骤
  - 硬件连接和测试方法

### 🎯 主要功能

实现了 **11 个 API 方法**，支持：
- ✅ 位置控制（读取/写入）
- ✅ 速度和加速度配置
- ✅ 控制字和工作模式设置
- ✅ 实时反馈和诊断
- ✅ 完整的错误处理
- ✅ PDO 缓冲区自动配置（已修复）

### 📊 核心 API 示例

```cpp
#include "lifter_ecat/lifter.hpp"
#include "lifter_ecat/cia402.hpp"

Lifter lifter(node);
lifter.initialize_soem();  // 自动配置 PDO 缓冲区

// 使能电机
lifter.set_motor_control_word(0x000F);
lifter.set_motor_mode(cia402::MODE_PROFILED_POSITION);

// 配置速度和加速度
lifter.set_motor_velocity(100);        // RPM
lifter.set_motor_acceleration(200);    // rpm/s

// 控制位置
lifter.write_motor_target_position(500.0);  // 500 mm

// 读取反馈
double pos = lifter.read_motor_position();
auto status = lifter.read_motor_status_word();
int vel = lifter.read_motor_velocity();
```

### 📝 新增文件

- `include/lifter_ecat/cia402.hpp` (450 行) - CiA402 标准定义
- `docs/CIA402_QUICK_REFERENCE.md` (360 行) - 快速参考卡
- `docs/CIA402_CONTROL_GUIDE.md` (310 行) - 完整 API 文档
- `docs/CIA402_IMPLEMENTATION_SUMMARY.md` (260 行) - 实现总结
- `docs/README_CIA402.md` (220 行) - 项目总体介绍
- `PDO_CONFIGURATION_SOLUTION.md` (230 行) - ⭐ PDO 配置解决方案

### ✅ 测试状态

- ✅ 编译通过（无错误、无警告）
- ✅ 运行验证成功
- ✅ 完整的文档和注释
- ✅ 生产就绪的代码质量
- ✅ **PDO 配置问题已修复** - 支持自动的 EtherCAT 状态转换

---

## 查看网卡名称

运行命令：
```bash
ip link
```

示例输出：
```text
1: lo: <LOOPBACK,UP,LOWER_UP> mtu 65536 qdisc noqueue state UNKNOWN mode DEFAULT group default qlen 1000
    link/loopback 00:00:00:00:00:00 brd 00:00:00:00:00:00
2: enp86s0: <NO-CARRIER,BROADCAST,MULTICAST,UP> mtu 1500 qdisc mq state DOWN mode DEFAULT group default qlen 1000
    link/ether 88:ae:dd:64:9b:fb brd ff:ff:ff:ff:ff:ff
3: wlo1: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc noqueue state UP mode DORMANT group default qlen 1000
    link/ether e4:c7:67:a0:e7:31 brd ff:ff:ff:ff:ff:ff
    altname wlp0s20f3
```

使用物理网口：`enp86s0`

## 关于仓库中的 SOEM（Simple Open EtherCAT Master）

- 本仓库包含 SOEM 源码的目录：`src/SOEM`
- 在当前仓库中检测到的 SOEM 版本：**2.0.0**（来源：`src/SOEM/CMakeLists.txt` 中的 `project(SOEM VERSION 2.0.0 ...)` 与 `src/SOEM/build/CPackConfig.cmake`）

### 推荐的 Git 管理方式

为了避免将第三方库源码直接提交到你的上游仓库（造成历史膨胀或许可证混乱），建议使用以下两种做法之一：

1) 使用 Git Submodule（推荐）

     - 将 SOEM 作为子模块加入：

         ```bash
         git submodule add https://github.com/OpenEtherCATsociety/SOEM.git src/SOEM
         git submodule update --init --recursive
         ```

     - 好处：主仓库保持轻量，子模块可以独立更新与同步，便于上游仓库不包含第三方源码。

2) 在部署/构建时单独克隆（或由 CI 拉取）

     - 手动克隆到本地：

         ```bash
         git clone https://github.com/OpenEtherCATsociety/SOEM.git src/SOEM
         ```

     - 或在 CI 脚本中通过上述命令拉取，不把源码提交到主仓库。

如果 `src/SOEM` 已经被误提交到 git 历史并且你想从当前分支移除，示例命令（请在理解后执行）：

```bash
git rm -r --cached src/SOEM
git commit -m "Remove SOEM sources from repo and use submodule/extern approach"
# 然后把 .gitignore 中加入 src/SOEM/（下面已为你添加）并推送
```

### 本仓库路径说明

- SOEM 源码位置（本地）： `src/SOEM`
- SOEM 的 include 头文件路径（构建时示例）： `src/SOEM/include`，当使用 `g++` 或 CMake 时，请把该路径加入 include 路径（`-I src/SOEM/include` 或在 CMake 中设置 `target_include_directories`）。

---

> 注：本仓库已在 `.gitignore` 中加入规则以阻止将 `src/SOEM` 提交（如果该目录之前已经被 git 跟踪，你需要按上面的 `git rm --cached` 命令先移除缓存）。

## 安装 SOEM（推荐）

把 SOEM 安装到系统路径可以让 CMake 的 `find_package(soem CONFIG)` 正常工作并导出合适的 targets（例如 `soem::soem`）。在本仓库我们提供了一个小脚本来完成这一步：

```bash
# 从仓库根目录运行（默认使用 src/SOEM/build）：
./scripts/install_soem.sh

# 或指定构建目录：
./scripts/install_soem.sh /path/to/SOEM/build
```

该脚本会执行 `sudo cmake --install <build-dir>` 并运行 `ldconfig`。安装后，系统路径下将包含 `/usr/local/include/soem`、`/usr/local/lib/libsoem.a`（或 `.so`）及 CMake 配置文件 `/usr/local/lib/cmake/soem`，CMake 的 `find_package(soem CONFIG)` 会更可靠。

## 构建并运行 lifter_ecat（smoke test）

在功能包层面（workspace 根）运行：

```bash
colcon build --packages-select lifter_ecat
source install/setup.bash
ros2 run lifter_ecat lifter_node
```

如果没有实际的 EtherCAT 总线连接，节点仍然会启动，但不会控制任何硬件；这是验证链接与启动流程的快速方法。

## CMake 策略说明

`src/lifter_ecat/CMakeLists.txt` 已实现两阶段查找：
- 优先 `find_package(soem CONFIG)`（当 SOEM 被系统安装时使用导出 target）；
- 回退到仓库内 `src/SOEM/build/install/lib/libsoem.a`（若系统未安装 SOEM，则使用 IMPORTED 静态库并设置 include path）。

这使得该功能包在开发机（有本地构建的 SOEM）和目标机（已系统安装 SOEM）之间更具鲁棒性。

