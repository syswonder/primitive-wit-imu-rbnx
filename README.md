# WIT IMU 原语

独立启动 WIT 标准 11 字节串口协议 IMU 的 Robonix 原语。包内携带原有驱动，
不依赖 ranger 的 WitImu_ws、已安装的 ROS 工作空间或特定本体申报。
设备须使用该标准输出协议及驱动支持的量程；其他 WIT 型号/协议不能仅凭品牌认定兼容。

## 环境与构建

Linux、ROS 2、rbnx/robonix-api；Python 可导入 rclpy、serial、yaml、grpc、grpc_tools。
ROS 环境提供 sensor_msgs、geometry_msgs、std_msgs。运行用户有串口读写权限。
不需要编译自定义 ROS 消息或 colcon 工作空间。

```bash
export PATH="$HOME/.cargo/bin:$PATH"
cd /path/to/primitive-wit-imu-rbnx
rbnx validate .
bash scripts/build.sh
```

start.sh 使用 ROS_SETUP 指定的 setup.bash，或 /opt/ros/$ROS_DISTRO/setup.bash
（未指定发行版时尝试 humble），也支持已 source 的其他 ROS 安装。
RMW_IMPLEMENTATION、ROS_DOMAIN_ID、Zenoh 配置由部署环境传入，不强制绑定 ranger。
WIT_PYTHON 可指定可导入 ROS 2 及 Robonix 依赖的解释器，默认 /usr/bin/python3。

## 启动硬件

默认 manage_driver=true。将实例加入部署的 primitive 列表，按实际位置配置包路径：

```yaml
primitive:
  - name: wit_imu
    path: ./primitives/primitive-wit-imu-rbnx
    config:
      serial:
        port: /dev/serial/by-id/YOUR_WIT_DEVICE
        baud: 9600
      frame_id: wit_10axis_link
```

也可在已加载全局约定和本包 capabilities/ 的 Atlas 上使用
`rbnx start -p /path/to/package -c config.yaml`。config.yaml 对应上面的 config 块。
驱动可独立启动，不以 Soma/URDF 存在为前提。更换本体只需配置设备、坐标系和 ROS 环境，
由整机部署补充实际挂载 TF。实例身份来自 RBNX_INSTANCE_NAME，默认 wit_imu。

serial.port 默认 auto：只尝试 /dev/wit_imu、/dev/imu_usb，或名称含 WIT/WT901/IMU
的 /dev/serial/by-id 路径。多个别名指向同一设备时去重；零个或多个设备时明确报错。
不默认将通用 CP2102 当作 WIT。需要按 USB ID 识别时可显式设置
serial.usb_vid 和 usb_pid（如 0x10c4、0xea60），两项必须一起提供且只匹配一个设备。
USB 转串口型号不代表传感器身份，优先使用已确认设备的稳定路径。
默认波特率 9600，必须与硬件当前设置一致。本包不会发送设备配置、标定或量程修改命令。

## 数据及能力约定

| 能力 | 默认话题 | ROS 类型 |
| --- | --- | --- |
| robonix/primitive/imu/imu（复用标准） | /wit_10axis/data | sensor_msgs/Imu |
| robonix/primitive/imu/magnetic_raw（新增包内） | /wit_10axis/magnetic_raw | geometry_msgs/Vector3Stamped |
| robonix/primitive/imu/pressure（新增包内） | /wit_10axis/pressure | sensor_msgs/FluidPressure |
| robonix/primitive/imu/barometric_altitude（新增包内） | /wit_10axis/barometric_altitude | std_msgs/Float64 |
| robonix/primitive/imu/status（新增包内） | /wit_10axis/status | std_msgs/String，JSON |

生命周期由共享 robonix/lifecycle/driver 自动提供，不新增 Driver TOML。
包内四条扩展约定不修改 Robonix 全局目录。详细语义见 CONTRACTS.md。
加速度和角速度使用 m/s²、rad/s；原始磁场为 LSB，不是已标定的特斯拉；
气压为 Pa，气压高度为 m，不等同于 GNSS 高程或地图 z。
姿态来自设备内部解算，未对齐地理航向；时间戳来自主机串口接收时刻。
协方差未知，不能据此宣称融合定位精度。

配置 topics 可修改对应端点，原语通过 ROS remapping 同步驱动与 Atlas 声明：

```yaml
topics:
  imu: /sensors/wit/imu
  magnetic_raw: /sensors/wit/magnetic_raw
  pressure: /sensors/wit/pressure
  barometric_altitude: /sensors/wit/barometric_altitude
  status: /sensors/wit/status
```

不同实例需使用不同话题和设备；话题必须是互不重复的绝对 ROS 名称。
变更已声明的端点需重启提供方，避免留下过时的 Atlas 绑定。

## 生命周期及已有驱动模式

INIT 校验配置。ACTIVATE 检查没有已有同名话题发布者，再启动包内驱动，
等待有限值的 Imu 数据和 serial_connected=true、valid=true 的状态消息。
探测超时默认 10 秒，可通过 sentinel_timeout_s 配置 (0,120] 秒。
不要求磁场、气压或气压高度必须输出，因此无气压输出的兼容型号仍可提供主 IMU 数据。
扩展话题是否有实际消息取决于硬件型号及其现有输出设置，消费者应检查超时和状态。

已有外部驱动时设置 manage_driver=false，见 deploy.external.fragment.yaml。
此模式只探测和声明已有话题，不控制设备；传入串口/坐标系覆盖项会报错。
停用、关闭或激活失败时，只清理自身驱动和临时参数文件；不停止外部驱动。
串口通过 exclusive=True 独占打开，重新激活会重新获取设备。
原语不发布 TF、odom、全局航向，也不提供地图融合、导航或 MCP 工具。
持续数据质量由消费者检查；ACTIVE 只说明激活时数据流已接通。

## 测试

```bash
python3 -m unittest discover -s tests -p 'test_*.py' -v
```

tests/integration.py 使用独立 Atlas 验证生命周期与发现端点；默认只订阅已有 WIT 数据。
加 --simulate 后，在仅本机 FastDDS domain 218 中使用伪串口，验证包内硬件驱动启动、
五种数据的单位、话题重映射、再次激活、无数据时失败及资源清理。
运行集成测试前 source ROS 并将 robonix-api、本包加入 PYTHONPATH。
测试文件不是构建或启动依赖；发布时可按需要省略，同时调整文档中的测试说明。
