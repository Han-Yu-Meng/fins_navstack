# 把「只有 livox 消息类型的 deb」变成可 apt 安装的软件源

> 场景：CI 镜像（`ros:humble-ros-base`）里没有 `livox_ros_driver2`，但 `fins_navstack` 的
> `livox_driver2` / `FAST_LIO` / `pointcloud_converter` / `ros_bridge` 都要 `find_package(livox_ros_driver2)`。
> 解决办法：自己打一个 **msg-only** 的 deb，再让它「能被 apt 装」。
> 最近更新：2026-10-28

---

## 0. 先确认：官方源里没有现成的

- `livox_ros_driver2` **没有做 bloom release**，不在 `ros/rosdistro` 的 humble release 列表里，
  因此 `packages.ros.org` 上**没有** `ros-humble-livox-ros-driver2`。
  网上文档里出现的 `sudo apt install ros-humble-livox-ros-driver2` 缺官方源支撑，装之前先
  `apt-cache policy ros-humble-livox-ros-driver2` 验证一下（大概率 `Unable to locate`）。
- 上游仓库里只有 `package_ROS2.xml`（不是规范的 `package.xml`），这也正是 bloom 发不出去的原因。
- 所以「别人提供好的包」这条路上，现实可用的只有：自己打包、第三方 apt 托管（Cloudsmith / PackageCloud）、
  或预先编好 `livox_ros_driver2` 的自建 Docker 镜像。

> **重要**：本仓库自己实现了驱动（`livox_ros::Lddc` / `DriverNode` / `Lds`），所以**只装 msg 的 deb 最安全**。
> 如果装的是「带驱动本体」的 deb，会引入 `liblivox_ros_driver2.so`，和本仓库同名类发生符号抢占
> （见 `docs/fins-notes.md` 第 4 节）。

---

## 1. 三条路线对比

| 路线 | 适用 | 工作量 | 缺点 |
|---|---|---|---|
| A. 本地 flat 源（`file://` 或内网 http） | 单机 / 内网多机 | 3 条命令 | 没有 GPG 签名，要 `trusted=yes` |
| B. 标准 `dists/` 源 + GPG（reprepro / aptly） | 团队长期维护 | 一次性 10 分钟 | 稍重，需要维护密钥 |
| C. 公网托管（GitHub Pages / Cloudsmith） | 开源项目、CI | 中 | 公开、要联网 |
| D. **CI 里直接装 deb（不建源）** | 只为了 CI 通过 | 最小 | 不算「软件源」，但最实用 |

---

## 2. 路线 A：本地 flat 源（最快）

```bash
# 1) 生成仓库索引（Packages / Packages.gz）
sudo apt install -y dpkg-dev
mkdir -p ~/aptrepo/pool
cp ros-humble-livox-ros-driver2_*.deb ~/aptrepo/pool/
cd ~/aptrepo
dpkg-scanpackages pool /dev/null > Packages
gzip -kf Packages
```

```bash
# 2) 客户端加源
echo "deb [trusted=yes] file:///home/steven/aptrepo ./" | sudo tee /etc/apt/sources.list.d/fins.list
sudo apt update
sudo apt install ros-humble-livox-ros-driver2     # 或用 deb 里真实的包名
```

内网多机就用 http，把上面第 2 步换成：

```bash
cd ~/aptrepo && python3 -m http.server 8080     # 或 nginx 指向该目录
# 客户端：
echo "deb [trusted=yes] http://192.168.1.5:8080/ ./" | sudo tee /etc/apt/sources.list.d/fins.list
```

要点：
- `./` 表示 **flat 仓库**（`Packages` 就在根目录，没有 `dists/` 结构），最适合手工维护。
- 没有 `Release`/签名时 apt 会报 "does not have a Release file"，`[trusted=yes]` 就是为了跳过这个检查。
- 加 `.deb` 之后执行 `apt update` 才会进缓存；`apt-cache policy <包名>` 能看到来源。

---

## 3. 路线 B：带签名的标准源（reprepro）

```bash
sudo apt install -y reprepro

# 1) 准备密钥（已有就跳过）
gpg --full-generate-key                  # 记下 key id
gpg --armor --export <KEYID> > ~/fins.gpg

# 2) 建仓库
mkdir -p ~/repo/conf
cat > ~/repo/conf/distributions <<'EOF'
Origin: fins
Label: fins local repo
Codename: jammy
Architectures: amd64 arm64
Components: main
Description: FINS local packages
SignWith: <KEYID>
EOF

# 3) 收录 deb（自动生成 dists/jammy/main/binary-amd64/Packages(.gz) 与 Release/InRelease）
reprepro -b ~/repo includedeb jammy ros-humble-livox-ros-driver2_1.0.0-0jammy_amd64.deb
```

发布（http / GitHub Pages 都是把 `~/repo` 整个目录当静态站点）+ 客户端：

```bash
curl -fsSL http://192.168.1.5/fins.gpg | sudo gpg --dearmor -o /usr/share/keyrings/fins.gpg
echo "deb [signed-by=/usr/share/keyrings/fins.gpg] http://192.168.1.5/ jammy main" \
  | sudo tee /etc/apt/sources.list.d/fins.list
sudo apt update && sudo apt install ros-humble-livox-ros-driver2
```

- 多架构 / 多发行版（humble=jammy、jazzy=noble）在 `conf/distributions` 里加多个 stanza 即可。
- `aptly` 是另一个选择（`aptly repo add` / `aptly publish repo`），适合需要快照/多版本并存的场合。
- Cloudsmith / PackageCloud 有免费额度，上传 deb 后自动给你一条带签名的 apt 源和安装脚本，等价于托管版路线 B。

---

## 4. 路线 D：CI 里直接装（推荐给 fins 的 GitHub Actions）

不需要建源，`.github/workflows/fins-build.yml` 里加一步（放在 `fins dep solve` 之前）：

```yaml
      - name: Install livox_ros_driver2 messages
        run: |
          DEB=ros-humble-livox-ros-driver2_1.0.0-0jammy_amd64.deb
          curl -fsSL -o /tmp/$DEB \
            "https://github.com/<org>/<repo>/releases/download/<tag>/$DEB"
          sudo apt-get install -y /tmp/$DEB
```

- 把 deb 当 **Release asset** 挂在仓库上，比维护 apt 源省事得多。
- 想省流量可以配 `actions/cache` 缓存 deb（key 用文件名）。
- 装完可以用一句自检，避免构建到一半才报错：
  `ros2 interface show livox_ros_driver2/msg/CustomMsg`

---

## 5. 自己打 deb 的正确姿势

### 5.1 用 bloom（推荐，产物规范）

```bash
# 前提：仓库有规范 package.xml（不是 package_ROS2.xml），并且代码已提交
bloom-generate rosdebian --os-name ubuntu --os-version jammy --ros-distro humble
fakeroot debian/rules binary
# 产物在上层目录：ros-humble-<包名>_<version>-0jammy_amd64.deb
```

### 5.2 手工打包（msg-only 更快）

先单独建一个只有 `msg/` 的 ament_cmake 包（`rosidl_generate_interfaces(...)`），
`colcon build` 后用 `fakeroot dpkg-deb -b` 或 `checkinstall` 打包，
**关键是要装到 `/opt/ros/humble` 下**，且保留 ament 的注册文件。

### 5.3 检查清单（打完之后务必核对）

```bash
# 1) ament / rosidl 注册项在不在（缺了 find_package 就找不到）
dpkg -c xxx.deb | grep -E 'ament_index/resource_index/(rosidl_interfaces|packages)/livox_ros_driver2'

# 2) CMake config 在不在
dpkg -c xxx.deb | grep -E 'cmake/livox_ros_driver2(Config|Config-version)\.cmake'

# 3) 环境钩子（source /opt/ros/humble/setup.bash 依赖）
dpkg -c xxx.deb | grep -E 'local_setup\.(bash|dsv)|package\.dsv'

# 4) 依赖声明是否齐全
dpkg -I xxx.deb | grep Depends
#   至少要能拉到：ros-humble-rosidl-default-runtime、ros-humble-std-msgs

# 5) 装完验证
ros2 interface show livox_ros_driver2/msg/CustomMsg
ros2 pkg prefix livox_ros_driver2
```

### 5.4 目标名（打包时注意保持一致）

`rosidl_generate_interfaces()` 的第一个 library 名决定 **CMake 导入目标的前缀**，
而 `.so` 文件名用的是 `PROJECT_NAME`：

| 打包方式 | CMake 目标 | .so 名 |
|---|---|---|
| 上游 `/opt/ros` 里的构建（library 名 `livox_interfaces2`） | `livox_ros_driver2::livox_interfaces2__rosidl_*` | `liblivox_ros_driver2__*` |
| 标准 `rosidl_generate_interfaces(${PROJECT_NAME} ...)` | `livox_ros_driver2::livox_ros_driver2__rosidl_*` | `liblivox_ros_driver2__*` |

本仓库四个包（`livox_driver2`、`FAST_LIO`、`pointcloud_converter`、`ros_bridge`）写死的是
`livox_ros_driver2::livox_interfaces2__rosidl_generator_cpp` / `...__rosidl_typesupport_cpp`，
**自己打的 deb 已确认沿用这套命名**，所以可以直接替换 `/opt/ros` 里那份。

如果以后换了别的来源的 deb，装完后先确认目标前缀再构建：

```bash
grep -o 'add_library(livox_ros_driver2::[a-z_0-9]*__rosidl_generator_cpp' \
  /opt/ros/humble/share/livox_ros_driver2/cmake/*Export.cmake
```

输出若不是 `livox_interfaces2__rosidl_generator_cpp`，就把四个 CMakeLists 里的目标名一起换掉
（或在打包时让 `rosidl_generate_interfaces()` 的 library 名保持 `livox_interfaces2`）。

---

## 6. 一句话选型

- 只是让 CI 过 → 路线 D（Release asset + `apt-get install -y /tmp/x.deb`）。
- 团队多机、要 `apt upgrade` 统一版本 → 路线 A（内网 http flat 源）起步，稳定后升到路线 B（reprepro 签名源）。
- 开源公开分发 → 路线 C（GitHub Pages 或 Cloudsmith）。
