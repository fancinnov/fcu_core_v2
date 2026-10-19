# 单机自动任务 MVP

在现有 ROS 包 `fcu_core` 内新增 `fcu_auto_mission_node.py`，仅连接 `fcu_bridge_001`。
自动任务相关的新文件全部集中在 `src/fcu_auto_mission/`：

```text
src/fcu_auto_mission/
├── __init__.py
├── drone.py                      # 动作 API 与规划器转发
├── fcu_auto_mission_node.py      # 节点入口
├── example_mission.py           # 用户任务示例
├── test_auto_mission.py         # 无 ROS 逻辑测试
└── README.md
```

节点由 CMake 安装，不需要根目录的 `setup.py`；源码与安装后均可使用现有 launch 启动。
用户编辑 Python 文件中的 `mission(drone)`；节点加载一次，修改文件后需要重启节点。
任务文件是本机可信 Python 程序，顶层只放定义，不在导入时执行飞行动作。

## 编译与启动

在 Ubuntu 20.04 / ROS Noetic 工作空间根目录执行：

```bash
catkin_make
source devel/setup.bash
roslaunch fcu_core fcu_core.launch enable_auto_mission:=true
```

`enable_auto_mission` 默认 `false`，关闭时沿用原控制话题，不启动自动任务节点。
设为 `true` 时复用主 launch 中的 `bridge_001`，将其命令和目标输入切换到自动任务独立话题，
原 `fcu_mission`、键盘和服务器的旧控制话题不再控制 001；其他 bridge 保持原配置。
通信方式沿用主 launch 的设置，自行启动 VINS 等定位节点。
也可以直接修改 `fcu_core.launch` 中该参数的默认值为 `true`。
节点启动只加载任务，不自动解锁。待里程计和话题连接就绪后执行；不依赖解锁、落地或心跳状态反馈：

```bash
rostopic echo /fcu_auto_mission/status
# 在另一个终端开始任务：
rosservice call /fcu_auto_mission/start "{}"
# 如需取消：
rosservice call /fcu_auto_mission/cancel "{}"
```

服务返回“已启动”只代表接受请求，最终结果查看 `status`。
同一时间只能执行一个任务。执行期间不能再次启动，结束后可以手动重新启动。

指定任务文件或通信方式：

```bash
roslaunch fcu_core fcu_core.launch enable_auto_mission:=true mission_file:=/absolute/path/my_mission.py
# 根据实际降落所需时间调整命令后的等待：
roslaunch fcu_core fcu_core.launch enable_auto_mission:=true arm_wait:=2 takeoff_wait:=5 land_wait:=20 disarm_wait:=2
```

自动任务节点直接定义在 `fcu_core.launch` 中，不再引用独立任务 launch。
bridge 的串口、IP、channel、offboard 等通信参数沿用该主 launch 中的配置。

## 编写积木任务

```python
def mission(drone):
    drone.arm(wait=2.0)
    drone.takeoff(wait=5.0)
    for x, y in [(1.0, 0.0), (1.0, 1.0)]:
        drone.goto(x=x, y=y, z=1.0, yaw=0.0, timeout=30.0)
        drone.hover(seconds=3.0)
    drone.land(wait=20.0)
    drone.disarm(wait=2.0)
```

| 接口 | 行为 |
| --- | --- |
| `arm(wait=None)` | 发送命令 1，等待指定秒数；不检测解锁状态 |
| `takeoff(wait=None)` | 发送命令 3，等待指定秒数；不是高度到达确认 |
| `goto(x, y, z, yaw=0, timeout=None)` | 保持目标；仅 XY 平面距离进入容差并连续稳定后完成 |
| `hover(seconds)` | 保持最近目标并计时；没有目标时固定调用时的位置和 yaw |
| `land(wait=None)` | 清除位置目标，发送命令 4，等待指定秒数；不检测落地 |
| `disarm(wait=None)` | 清除位置目标，发送命令 2，等待指定秒数；不检测上锁状态 |

坐标采用 `/odom_global_001` 坐标约定，单位米，yaw 为弧度、正方向向左。
接口自动转换为 bridge 的 FRU 位置和 FRD yaw；速度、加速度、yaw_rate 全部填零。
`goto` 的 z 为该坐标系中的绝对 Z，不是相对起飞高度。
仅改变 z/yaw 而 XY 已到点时，`goto` 仍会完成；可用 `hover()` 显式等待响应。
悬停是保持目标并计时，不额外检测高度、航向或稳定状态。
起飞使用飞控原有配置，不提供尚未确认协议支持的高度参数。
`wait=None` 使用 launch/节点参数，显式传入 `wait` 则覆盖默认等待时间。
原先 `arm/land/disarm` 的 `timeout` 参数已改为 `wait`；`goto` 仍使用 `timeout` 表示到点超时。

## 接入规划器

默认沿用直接飞点。启用规划模式前，在 `launch/fcu_core.launch` 中同步修改两个节点的私有参数：

| 节点 | 参数 | 直接飞点 | 使用外部规划器 |
| --- | --- | --- | --- |
| `fcu_bridge_001` | `set_goal` | false | true |
| `fcu_auto_mission`（入口 `fcu_auto_mission_node.py`） | `use_planner` | false | true |

这两个参数必须保持一致，分别在各自节点内配置；没有全局 `set_goal` launch arg。
修改后重新启动节点，再开始任务：

```bash
roslaunch fcu_core fcu_core.launch enable_auto_mission:=true
rosservice call /fcu_auto_mission/start "{}"
```

不要只修改其中一个节点的私有参数。启动文件不会启动规划器，需要提前运行规划器及其定位、地图依赖。
规划器必须与自动任务节点共享 ROS master，并使用同一个 `map` 坐标系；
自动任务节点需要接收规划器的 `/planning/pos_cmd`；如果它们使用不同 ROS master，需要桥接该话题。

规划模式链路：

1. `goto(x,y,z,yaw)` 转换坐标，向 `/fcu_auto_mission/mission_001` 发布一次最终目标数组。
2. bridge 的 `missionHandler()` 在 `set_goal=true` 时，将最终目标封装为 `MAV_FRAME_MISSION` 发给飞控。
3. 飞控接收后回传 `MAV_FRAME_MISSION` 目标，bridge 的 `parse_data()` 将其转换为
   `geometry_msgs/PoseStamped`，发布到 `/move_base_simple/goal`。
4. 规划器接收回传目标，向 `/planning/pos_cmd` 发布 `quadrotor_msgs/PositionCommand`。
5. 自动任务节点转换每个规划输出的位置/yaw，发布到 `/fcu_auto_mission/control_001`。
6. bridge 的专用 `control_001` 输入强制按位置指令发送，使用项目已有 `MAV_FRAME_GLOBAL` 标记，
   不受 `set_goal=true` 影响，避免把轨迹采样点再次送去规划。

自动任务节点不发布 `PoseStamped`，不再使用 `planner_goal` 话题或 `planner_goal_topic` 参数。
该链路要求飞控支持目标回传；现有 bridge 回传分支只填位置，没有将 yaw 写入目标四元数，
因此规划器收到的最终航向不由此链路保证。发往飞控的最终目标数组仍包含用户指定的 yaw。
bridge 启动后的前 5 秒会忽略目标，调用 `~start` 前应等待桥接初始化完成。

用户 API 不变，仍只提供 XYZ/yaw；规划输出中的速度、加速度和 yaw_dot 暂不转发，发送时填零。
这是一条规划位置采样点转发链路，不是带速度/加速度前馈的完整轨迹跟踪。
旧 `fcu_mission` 可以继续运行，但其轨迹输出不会进入自动任务控制的 bridge。

等待规划输出时不通过 `control_001` 直接发送最终目标，也不重复发送规划目标或重放旧轨迹采样。
默认首个输出等待 10 秒，包含桥接发送、飞控回传及规划器启动时间；后续输出接收间隔上限 2 秒。
没有输出或输出中断时任务失败，
定位有效时改为发送当前位置保持指令。非法数值或明确的轨迹中止/错误标记也会中断任务。
空闲、起飞、降落、上锁、取消时不转发规划输出；XY 到点后退出轨迹转发并保持最终目标，
`hover()` 期间也不转发轨迹。高度和 yaw 不参与到点检测。

可配置规划话题和等待时间：

```bash
roslaunch fcu_core fcu_core.launch enable_auto_mission:=true \
  planner_command_topic:=/planning/pos_cmd \
  planner_start_timeout:=10 planner_timeout:=2
```

## 话题与状态

| 接口 | 类型/含义 |
| --- | --- |
| `/fcu_auto_mission/command` | `std_msgs/Int16`，bridge 的 `~command` 输入/原有状态事件输出 |
| `/fcu_auto_mission/mission_001` | `std_msgs/Float32MultiArray`，直接模式持续发布位置目标；规划模式每次 `goto` 只发布一次最终目标 |
| `/fcu_auto_mission/control_001` | `std_msgs/Float32MultiArray`，专用位置输入，不受 bridge 的 `set_goal` 影响 |
| `/move_base_simple/goal` | `geometry_msgs/PoseStamped`，bridge 发布的飞控回传目标，供规划器接收 |
| `/planning/pos_cmd` | `quadrotor_msgs/PositionCommand`，规划位置采样输入 |
| `/odom_global_001` | `nav_msgs/Odometry`，位置反馈 |
| `/fcu_auto_mission/status` | `std_msgs/String`，动作、完成或失败原因，锁存最新状态 |

不新增解锁、落地状态话题；bridge 的 `parse_data()` 保留原有逻辑，自动任务不消费其中的状态事件。
解锁、起飞、降落、上锁均采用“发送一次命令 + 定时等待”。等待结束仅代表程序可以进入下一步，
不代表飞控已接受命令或飞机已完成动作；`SUCCEEDED` 只表示任务程序执行结束。
`land_wait` 需要按实际高度和下降速度留足时间，等待结束后程序就会执行后续上锁命令，不做落地拦截。
降落等待和上锁期间不发送位置目标，取消降落等待不会恢复原航点。

## 参数与异常行为

节点私有参数默认值：

| 参数 | 默认值 |
| --- | --- |
| `xy_tolerance` | 0.15 米 |
| `settle_time` | 0.5 秒 |
| `arm_wait` | 2 秒 |
| `takeoff_wait` | 5 秒 |
| `land_wait` | 15 秒 |
| `disarm_wait` | 2 秒 |
| `action_timeout` | 30 秒，默认到点超时 |
| `odom_timeout` | 2 秒，里程计接收有效期 |
| `target_publish_rate` | 10 Hz |
| `use_planner` | false，需手动与 `fcu_bridge_001` 的 `set_goal` 同步设置 |
| `planner_start_timeout` | 10 秒，等待首个规划输出 |
| `planner_timeout` | 2 秒，规划输出接收间隔上限 |

主 `fcu_core.launch` 当前的 `xy_tolerance` 为 0.10 米，优先覆盖节点默认值。
四个命令等待参数均已在主 launch 中暴露。
时间等待及里程计接收有效期使用单调时钟，不依赖 ROS 仿真时钟。
不自动重发起飞、降落、解锁或上锁命令；到点超时则中断任务。
取消/失败时，如果原来在发送位置目标且定位有效，会改为保持当前位置；
起飞或降落期间不发送新的位置目标，不自动降落或上锁。
里程计失效后停止发布目标，飞点/悬停报错中断；命令定时等待不要求里程计持续有效。
开始任务仍要求有效里程计和 bridge 订阅连接；这些检查不表示飞控已确认命令。
任务成功后若仍有位置目标，会继续保持，直至降落、取消或开始下一次任务。
节点退出后无法继续发送目标。

第一版不提供暂停/继续、内置规划器、速度轨迹、Action 接口或多机控制；避障效果取决于外部规划器。
用户循环应调用这些接口，不用长时间 `time.sleep()` 或吞掉 `MissionError`，以便及时处理中断。

## 本地逻辑验证

下面的测试通过替身消息、单调时钟和发布器检查执行逻辑，不连接飞机，也不依赖 ROS 安装：

```bash
python3 -m unittest discover -s src/fcu_auto_mission -p test_auto_mission.py -v
```

实际部署仍需在 ROS Noetic 编译并与飞控联调，确认命令行为和各动作等待时间。
