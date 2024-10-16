#pragma once

#include "AP_DDS_config.h" //定义DDS相关的宏，比如`AP_DDS_ENABLED`来控制是否启用DDS模块。

#if AP_DDS_ENABLED

#include "uxr/client/client.h" //这两个头文件来自于Micro XRCE-DDS库，用于在资源受限设备上实现DDS客户端。
#include "ucdr/microcdr.h"

#include "ardupilot_msgs/msg/GlobalPosition.h"
#include "builtin_interfaces/msg/Time.h"

#include "sensor_msgs/msg/NavSatFix.h"
#include "tf2_msgs/msg/TFMessage.h"
#include "sensor_msgs/msg/BatteryState.h"
#if AP_DDS_IMU_PUB_ENABLED
#include "sensor_msgs/msg/Imu.h"
#endif // AP_DDS_IMU_PUB_ENABLED
#include "sensor_msgs/msg/Joy.h"
#include "geometry_msgs/msg/PoseStamped.h"
#include "geometry_msgs/msg/TwistStamped.h"
#include "geographic_msgs/msg/GeoPointStamped.h" //这些头文件通过idl生成
#include "geographic_msgs/msg/GeoPoseStamped.h"
#include "rosgraph_msgs/msg/Clock.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Scheduler.h>
#include <AP_HAL/Semaphores.h>

#include "fcntl.h"

#include <AP_Param/AP_Param.h>

#define DDS_MTU             512  //定义了DDS的最大传输单元（MTU），这里为512字节。
#define DDS_STREAM_HISTORY  8      //流的历史长度，这里为8。
#define DDS_BUFFER_SIZE     DDS_MTU * DDS_STREAM_HISTORY //定义了缓冲区大小

#if AP_DDS_UDP_ENABLED
#include <AP_HAL/utility/Socket.h>
#include <AP_Networking/AP_Networking_address.h>
#endif

extern const AP_HAL::HAL& hal;

class AP_DDS_Client
{

private:

    AP_Int8 enabled;

    // Serial Allocation
    uxrSession session; //Session
    bool is_using_serial; // true when using serial transport

    // input and output stream
    uint8_t *input_reliable_stream;
    uint8_t *output_reliable_stream;
    uxrStreamId reliable_in;
    uxrStreamId reliable_out;

    // Outgoing Sensor and AHRS data  数据类型定义，这些类是通过上面引入不同的消息类型头文件得到的
    builtin_interfaces_msg_Time time_topic; 
    geographic_msgs_msg_GeoPointStamped gps_global_origin_topic;
    geographic_msgs_msg_GeoPoseStamped geo_pose_topic;
    geometry_msgs_msg_PoseStamped local_pose_topic;
    geometry_msgs_msg_TwistStamped tx_local_velocity_topic;
    sensor_msgs_msg_BatteryState battery_state_topic;
    sensor_msgs_msg_NavSatFix nav_sat_fix_topic;
#if AP_DDS_IMU_PUB_ENABLED
    sensor_msgs_msg_Imu imu_topic;
#endif // AP_DDS_IMU_PUB_ENABLED
    rosgraph_msgs_msg_Clock clock_topic;
    // incoming joystick data
    static sensor_msgs_msg_Joy rx_joy_topic;  
    // incoming REP147 velocity control
    static geometry_msgs_msg_TwistStamped rx_velocity_control_topic;
    // incoming REP147 goal interface global position
    static ardupilot_msgs_msg_GlobalPosition rx_global_position_control_topic;  ///静态消息，不会占用过多内存，所有对象共享使用
    // outgoing transforms
    tf2_msgs_msg_TFMessage tx_static_transforms_topic;
    // incoming transforms
    static tf2_msgs_msg_TFMessage rx_dynamic_transforms_topic;

    HAL_Semaphore csem;  //**`HAL_Semaphore`** 是 ArduPilot 中用于线程同步和互斥操作的信号量。**`csem`** 可以确保在**多线程或并发环境**下，访问共享资源时避免冲突。

    // connection parametrics  连接状态相关，用于跟踪 AP_DDS_Client 类的通信连接状态。
    bool status_ok{false};
    bool connected{false};

////声明update“更新话题”函数，这些函数**接收传感器等数据并更新 ROS 2 消息**  ////函数重载允许在同一个作用域内定义多个同名函数，只要这些函数的参数列表不同。编译器会根据传入的参数类型和数量来决定调用哪个函数。
    static void update_topic(builtin_interfaces_msg_Time& msg);  //把builtin_interfaces_msg_Time数据类型的值引用传递给 msg变量
    bool update_topic(sensor_msgs_msg_NavSatFix& msg, const uint8_t instance) WARN_IF_UNUSED;
    static void populate_static_transforms(tf2_msgs_msg_TFMessage& msg);
    static void update_topic(sensor_msgs_msg_BatteryState& msg, const uint8_t instance);
    static void update_topic(geometry_msgs_msg_PoseStamped& msg);
    static void update_topic(geometry_msgs_msg_TwistStamped& msg);
    static void update_topic(geographic_msgs_msg_GeoPoseStamped& msg);
#if AP_DDS_IMU_PUB_ENABLED
    static void update_topic(sensor_msgs_msg_Imu& msg);
#endif // AP_DDS_IMU_PUB_ENABLED
    static void update_topic(rosgraph_msgs_msg_Clock& msg);
    static void update_topic(geographic_msgs_msg_GeoPointStamped& msg);

    // subscription callback function //订阅回调函数
    static void on_topic_trampoline(uxrSession* session, uxrObjectId object_id, uint16_t request_id, uxrStreamId stream_id, struct ucdrBuffer* ub, uint16_t length, void* args);
    void on_topic(uxrSession* session, uxrObjectId object_id, uint16_t request_id, uxrStreamId stream_id, struct ucdrBuffer* ub, uint16_t length);

    // service replier callback function //服务响应者的回调函数
    static void on_request_trampoline(uxrSession* session, uxrObjectId object_id, uint16_t request_id, SampleIdentity* sample_id, ucdrBuffer* ub, uint16_t length, void* args);
    void on_request(uxrSession* session, uxrObjectId object_id, uint16_t request_id, SampleIdentity* sample_id, ucdrBuffer* ub, uint16_t length);

    // delivery control parameters
    uxrDeliveryControl delivery_control {
        .max_samples = UXR_MAX_SAMPLES_UNLIMITED,  //最大样本数量，这里表示无限制
        .max_elapsed_time = 0,   //指定 时间窗口（毫秒）**`0`** 表示**没有时间限制**
        .max_bytes_per_second = 0,  //**每秒最大字节数****`0`** 表示没有限制，可以使用**全速传输**
        .min_pace_period = 0  //**最小的节奏周期（毫秒）****`0`** 表示不使用节奏控制，即消息可以尽快传输
    };

    // The last ms timestamp AP_DDS wrote a Time message 
    //时间戳成员变量
    uint64_t last_time_time_ms;
    // The last ms timestamp AP_DDS wrote a NavSatFix message
    uint64_t last_nav_sat_fix_time_ms;
    // The last ms timestamp AP_DDS wrote a BatteryState message
    uint64_t last_battery_state_time_ms;
#if AP_DDS_IMU_PUB_ENABLED
    // The last ms timestamp AP_DDS wrote an IMU message
    uint64_t last_imu_time_ms;
#endif // AP_DDS_IMU_PUB_ENABLED
    // The last ms timestamp AP_DDS wrote a Local Pose message
    uint64_t last_local_pose_time_ms;
    // The last ms timestamp AP_DDS wrote a Local Velocity message
    uint64_t last_local_velocity_time_ms;
    // The last ms timestamp AP_DDS wrote a GeoPose message
    uint64_t last_geo_pose_time_ms;
    // The last ms timestamp AP_DDS wrote a Clock message
    uint64_t last_clock_time_ms;
    // The last ms timestamp AP_DDS wrote a gps global origin message
    uint64_t last_gps_global_origin_time_ms;

    // functions for serial transport 串行通信传输（有线）
    bool ddsSerialInit();
    static bool serial_transport_open(uxrCustomTransport* args);
    static bool serial_transport_close(uxrCustomTransport* transport);
    static size_t serial_transport_write(uxrCustomTransport* transport, const uint8_t* buf, size_t len, uint8_t* error);
    static size_t serial_transport_read(uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* error);
    struct {
        AP_HAL::UARTDriver *port;
        uxrCustomTransport transport;
    } serial;

#if AP_DDS_UDP_ENABLED
    // functions for udp transport UDP传输（无线）
    bool ddsUdpInit();
    static bool udp_transport_open(uxrCustomTransport* args);
    static bool udp_transport_close(uxrCustomTransport* transport);
    static size_t udp_transport_write(uxrCustomTransport* transport, const uint8_t* buf, size_t len, uint8_t* error);
    static size_t udp_transport_read(uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* error);

    struct {
        AP_Int32 port;
        // UDP endpoint
        AP_Networking_IPV4 ip{AP_DDS_DEFAULT_UDP_IP_ADDR};
        // UDP Allocation
        uxrCustomTransport transport;
        SocketAPM *socket;
    } udp;
#endif
    // pointer to transport's communication structure
    uxrCommunication *comm{nullptr};

    // client key we present 客户端密钥，在多客户端时需要设置不同的密钥
    static constexpr uint32_t key = 0xAAAABBBB;

public:
    ~AP_DDS_Client();  //析构函数，释放内存

    bool start(void); //start函数为负责启动客户端的工作流程
    void main_loop(void); //核心循环函数

    //! @brief Initialize the client's transport
    //! @return True on successful initialization, false on failure
    bool init_transport() WARN_IF_UNUSED;  //初始化传输层

    //! @brief Initialize the client's uxr session and IO stream(s)
    //! @return True on successful initialization, false on failure
    bool init_session() WARN_IF_UNUSED;  //初始化一个 **uxr session**（Micro XRCE-DDS 的会话）

    //! @brief Set up the client's participants, data read/writes,
    //         publishers, subscribers
    //! @return True on successful creation, false on failure
    bool create() WARN_IF_UNUSED;  //函数用于设置参与者（participants）、数据流的读写（read/write）、发布者（publishers）和订阅者（subscribers）。

/////////////////////////////////////序列化（**serialize**）并发布数据到 IO 流（**IO stream(s)**）的多个成员函数。//////////////////
    //! @brief Serialize the current time state and publish to the IO stream(s)
    void write_time_topic();
    //! @brief Serialize the current nav_sat_fix state and publish to the IO stream(s)
    void write_nav_sat_fix_topic();
    //! @brief Serialize the static transforms and publish to the IO stream(s)
    void write_static_transforms();
    //! @brief Serialize the current nav_sat_fix state and publish it to the IO stream(s)
    void write_battery_state_topic();
    //! @brief Serialize the current local_pose and publish to the IO stream(s)
    void write_local_pose_topic();
    //! @brief Serialize the current local velocity and publish to the IO stream(s)
    void write_tx_local_velocity_topic();
    //! @brief Serialize the current geo_pose and publish to the IO stream(s)
    void write_geo_pose_topic();
#if AP_DDS_IMU_PUB_ENABLED
    //! @brief Serialize the current IMU data and publish to the IO stream(s)
    void write_imu_topic();
#endif // AP_DDS_IMU_PUB_ENABLED
    //! @brief Serialize the current clock and publish to the IO stream(s)
    void write_clock_topic();
    //! @brief Serialize the current gps global origin and publish to the IO stream(s)
    void write_gps_global_origin_topic();
    //! @brief Update the internally stored DDS messages with latest data
    void update();   //更新函数

    //! @brief GCS message prefix
    static constexpr const char* msg_prefix = "DDS:";

    //! @brief Parameter storage
    static const struct AP_Param::GroupInfo var_info[];

    //! @brief ROS_DOMAIN_ID
    AP_Int32 domain_id;   //用于存储 ROS2 的 **Domain ID**

    //! @brief Enum used to mark a topic as a data reader or writer
    enum class Topic_rw : uint8_t {
        DataReader = 0,
        DataWriter = 1,
    };

    //! @brief Convenience grouping for a single "channel" of data 给AP_DDS_Topic_Table.h文件中声明了一个topic table结构体
    struct Topic_table {
        const uint8_t topic_id;
        const uint8_t pub_id;
        const uint8_t sub_id;    // added sub_id fields to avoid confusion
        const uxrObjectId dw_id;
        const uxrObjectId dr_id; // added dr_id fields to avoid confusion
        const Topic_rw topic_rw;
        const char* topic_name;
        const char* type_name;
        const uxrQoS_t qos;
    };
    static const struct Topic_table topics[]; //Topic_table的实例，静态常量

    //! @brief Enum used to mark a service as a requester or replier
    enum class Service_rr : uint8_t {
        Requester = 0,
        Replier = 1,
    };

    //! @brief Convenience grouping for a single "channel" of services
    struct Service_table {
        //! @brief Request ID for the service
        const uint8_t req_id;

        //! @brief Reply ID for the service
        const uint8_t rep_id;

        //! @brief Service is requester or replier
        const Service_rr service_rr;

        //! @brief Service name as it appears in ROS
        const char* service_name;

        //! @brief Service requester message type
        const char* request_type;

        //! @brief Service replier message type
        const char* reply_type;

        //! @brief Service requester topic name
        const char* request_topic_name;

        //! @brief Service replier topic name
        const char* reply_topic_name;

        //! @brief QoS for the service
        const uxrQoS_t qos;
    };
    static const struct Service_table services[];
};

#endif // AP_DDS_ENABLED


