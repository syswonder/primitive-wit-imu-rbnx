# WIT IMU 能力约定 v1

复用标准 robonix/primitive/imu/imu（sensor_msgs/msg/Imu）。本包新增 magnetic_raw、
pressure、barometric_altitude 和 status 四条局部 topic_out 约定，均通过 ROS 2 传输。
IDL 引用框架 common_interfaces，不引入新消息包或 RPC。默认发布端 QoS 为
reliable / volatile / keep_last(10)，启动探测使用兼容的 best-effort 订阅。

## 主 IMU

Imu.header.frame_id 为配置的传感器坐标系；header.stamp 是主机接收/发布时刻。
设备 0x51、0x52、0x53 分别解析加速度、角速度和欧拉角。只有加速度与角速度
距当前时刻均小于 0.25 秒时，才随欧拉角帧发布 Imu。
协议按 ±16 g、±2000 °/s、±180° 换算，分别输出 m/s²、rad/s、四元数。
其他量程或非标准协议必须先确认兼容，不能重复缩放物理量。
设备内部姿态未对齐 ENU/NED 或地理北向，不能直接作为地图 yaw。
协方差为零矩阵，按 sensor_msgs/Imu 表达未知，不是零误差。

## 磁场和气压

- magnetic_raw：Vector3Stamped.vector 三轴为协议 0x54 的有符号原始 LSB。
  无型号专用换算或标定，不能套用 sensor_msgs/MagneticField 的特斯拉语义。
- pressure：FluidPressure.fluid_pressure 为 0x56 的绝对气压（Pa），仅在值大于零时发布，
  header 沿用配置坐标系，variance=0 表示未知。
- barometric_altitude：Float64.data 为 0x56 的高度厘米值除以 100（m）。
  高程参考面未指定，无 header/时间戳，不能当 GNSS 椭球高或地图 z。
  与气压同帧输出；消费者以接收时刻检查新鲜度。

## 状态 JSON

status 的 String.data 为 JSON 对象，约定下列字段；允许将来增加字段：

| 字段 | 类型 | 语义 |
| --- | --- | --- |
| serial_connected | boolean | 驱动当前是否持有已打开串口 |
| port | string | 当前实际设备路径，或尚未打开时的配置值 |
| baud | integer | 当前串口波特率 |
| frame_id | string | 配置的传感器坐标系 |
| timestamp_source | string | HOST_SERIAL_RECEIVE；不是传感器采样时间 |
| imu_age_s | number/null | 距最近主 IMU 输出的秒数 |
| magnetic_age_s | number/null | 距最近磁场输出的秒数 |
| pressure_age_s | number/null | 距最近气压输出的秒数 |
| valid | boolean | 主 IMU 最近有输出且距当前时刻小于 0.5 秒 |
| counts | object | imu、mag、pressure 三类累计输出次数 |
| checksum_errors | integer | 校验失败计数；重新连接后解析器计数重置 |
| orientation_reference | string | SENSOR_INTERNAL_UNALIGNED |
| geographic_heading_valid | boolean | 当前驱动恒为 false，不声称地理航向有效 |
| covariance_known | boolean | 当前驱动恒为 false |

消费者分别检查话题接收超时、相关数据龄期和有效性；串口打开不等于有有效测量，
valid 也不保证磁场、气压或航向有效。上游标准化需独立协商，这些扩展暂不改全局约定。

## 本地扩展：独立角速度

新增 `robonix/primitive/imu/angular_velocity`，Vector3Stamped，默认 `/wit_10axis/angular_velocity`。
在每个有效 0x52 gyro 帧发布 rad/s，header 为该帧主机接收时刻、frame 为配置的传感器坐标系。
原 Imu 消息的姿态/加速度/角速度联合发布规则不变；姿态帧丢失时，此独立 gyro 流仍可供定位/停车反馈使用。
status 新增 `gyro_age_s`，可单独判断 gyro 新鲜度。该流不是地理航向，不包含安装旋转或地图变换。
