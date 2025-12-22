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

