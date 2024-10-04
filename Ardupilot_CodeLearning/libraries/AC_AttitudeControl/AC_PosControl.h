#pragma once  //这是一个预处理指令，确保头文件只会被编译一次，防止重复包含同一个文件引起的编译错误。

#include <AP_Common/AP_Common.h> //这是一个包含ArduPilot通用功能的头文件，AP_Common中通常定义了一些基础的全局函数、宏、类型以及变量，用于整个ArduPilot代码库的使用。
#include <AP_Param/AP_Param.h>  //该头文件包含了参数管理系统，用于管理、存储和访问与无人机配置相关的参数（如PID参数、飞行模式参数等）。
#include <AP_Math/AP_Math.h>   //这个文件定义了通用的数学运算工具，例如向量、矩阵运算、角度变换等，通常在控制算法中需要使用这些工具。
#include <AC_PID/AC_P.h>            // P library
#include <AC_PID/AC_PID.h>          // PID library
#include <AC_PID/AC_P_1D.h>         // P library (1-axis)
#include <AC_PID/AC_P_2D.h>         // P library (2-axis)
#include <AC_PID/AC_PI_2D.h>        // PI library (2-axis)
#include <AC_PID/AC_PID_Basic.h>    // PID library (1-axis)
#include <AC_PID/AC_PID_2D.h>       // PID library (2-axis)
#include <AP_InertialNav/AP_InertialNav.h>  // Inertial Navigation library
#include "AC_AttitudeControl.h"     // Attitude control library

#include <AP_Logger/LogStructure.h> //这是日志记录相关的库，AP_Logger用于管理数据记录的结构，通常会记录飞行期间的关键数据（如传感器读数、控制输出），便于分析飞行情况或者调试算法。

// position controller default definitions #define用于给无人机位置控制系统中的一些默认值定义常量
//尽管C++有更强大的语言特性（如const、constexpr等）用于定义常量，#define仍然常用于～～定义全局的常量值～～

#define POSCONTROL_ACCEL_XY                     100.0f  // default horizontal acceleration in cm/s/s.  This is overwritten by waypoint and loiter controllers 默认水平加速度，单位：cm/s²。这一值会被航点控制器和悬停控制器覆盖
#define POSCONTROL_JERK_XY                      5.0f    // default horizontal jerk m/s/s/s 默认水平冲击率，单位：m/s³ 无人机和其他控制系统中，冲击率（加加速度）控制通常用来防止加速度发生过快的变化，造成系统不稳定或产生过大的机械冲击。

#define POSCONTROL_STOPPING_DIST_UP_MAX         300.0f  // max stopping distance (in cm) vertically while climbing 最大爬升停止距离（单位：cm）避免超调
#define POSCONTROL_STOPPING_DIST_DOWN_MAX       200.0f  // max stopping distance (in cm) vertically while descending  最大下降停止距离（单位：cm）避免超调

#define POSCONTROL_SPEED                        500.0f  // default horizontal speed in cm/s  默认水平速度，单位：cm/s 用于自动飞行
#define POSCONTROL_SPEED_DOWN                  -150.0f  // default descent rate in cm/s  默认下降速率，单位：cm/s 用于自动飞行
#define POSCONTROL_SPEED_UP                     250.0f  // default climb rate in cm/s  默认爬升速率，单位：cm/s 用于自动飞行

#define POSCONTROL_ACCEL_Z                      250.0f  // default vertical acceleration in cm/s/s. 默认垂直加速度，单位：cm/s² 用于自动飞行
#define POSCONTROL_JERK_Z                       5.0f    // default vertical jerk m/s/s/s  默认垂直冲击率，单位：m/s³ 用于自动飞行

#define POSCONTROL_THROTTLE_CUTOFF_FREQ_HZ      2.0f    // low-pass filter on acceleration error (unit: Hz) 加速度误差的低通滤波频率，单位：Hz 位置控制频率比较低，所以2hz的平滑滤波是可以接受的

#define POSCONTROL_OVERSPEED_GAIN_Z             2.0f    // gain controlling rate at which z-axis speed is brought back within SPEED_UP and SPEED_DOWN range 控制z轴速度恢复到SPEED_UP和SPEED_DOWN范围的增益

#define POSCONTROL_RELAX_TC                     0.16f   // This is used to decay the I term to 5% in half a second. 定义了一个时间常数，用于在位置控制算法中的I项（积分项）的衰减，在半秒内将I项衰减到5%

class AC_PosControl //这是在C++中定义一个类，名为 AC_PosControl。这个类负责无人机或机器人的位置控制功能。
{
public:      //public 关键词表示该类的公共成员和方法可以被外部代码访问。

    /// Constructor 构造函数：这是类的构造函数，它在创建 AC_PosControl 对象时被调用，用于初始化类的成员变量或执行其他初始化操作。
    AC_PosControl(AP_AHRS_View& ahrs, const AP_InertialNav& inav,
                  const class AP_Motors& motors, AC_AttitudeControl& attitude_control);



    /// set_dt / get_dt - dt is the time since the last time the position controllers were updated
    ///   _dt should be set based on the time of the last IMU read used by these controllers
    ///   the position controller should run updates for active controllers on each loop to ensure normal operation
    /// set_dt 可以用来验证输入值，防止非法数据，而 get_dt 可以确保外部只能读取数据，不能修改。
    /// getter 的主要作用是提供访问类的私有成员变量的接口，允许外部代码读取这些数据，而不直接暴露内部实现细节。它的核心功能是读取和返回私有成员变量的值，并保持类的封装性和安全性。
    void set_dt(float dt) { _dt = dt; }
    float get_dt() const { return _dt; } ////这里的 `const` 关键字是在 get_dt() 函数的末尾，这意味着 get_dt() 是一个常量成员函数

    /// get_shaping_jerk_xy_cmsss - gets the jerk limit of the xy kinematic path generation in cm/s/s/s
    float get_shaping_jerk_xy_cmsss() const { return _shaping_jerk_xy*100.0; } //这个方法函数是为了**获取相应的值，并转化为厘米每秒单位**所准备的。其获取X-Y平面的**冲击率**（jerk）的限制值，冲击率表示加速度随时间的变化率。


    ///
    /// 3D position shaper 3D位置整形器, 用于平滑的路径规划
    ///

    /// input_pos_xyz - calculate a jerk limited path from the current position, velocity and acceleration to an input position.
    ///     The function takes the current position, velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
    ///     The kinematic path is constrained by the maximum acceleration and jerk set using the function set_max_speed_accel_xy.
    void input_pos_xyz(const Vector3p& pos, float pos_offset_z, float pos_offset_z_buffer);

    /// pos_offset_z_scaler - calculates a multiplier used to reduce the horizontal velocity to allow the z position controller to stay within the provided buffer range
    float pos_offset_z_scaler(float pos_offset_z, float pos_offset_z_buffer) const;

    ///
    /// Lateral position controller 横向位置控制器
    ///

    /// set_max_speed_accel_xy - set the maximum horizontal speed in cm/s and acceleration in cm/s/s
    ///     This function only needs to be called if using the kinematic shaping.
    ///     This can be done at any time as changes in these parameters are handled smoothly
    ///     by the kinematic shaping.
    void set_max_speed_accel_xy(float speed_cms, float accel_cmss); //设置最大水平速度和加速度

    /// set_max_speed_accel_xy - set the position controller correction velocity and acceleration limit
    ///     This should be done only during initialisation to avoid discontinuities
    void set_correction_speed_accel_xy(float speed_cms, float accel_cmss); //设置修正速度和加速度限制

    /// get_max_speed_xy_cms - get the maximum horizontal speed in cm/s
    float get_max_speed_xy_cms() const { return _vel_max_xy_cms; } //获取最大水平速度

    /// get_max_accel_xy_cmss - get the maximum horizontal acceleration in cm/s/s
    float get_max_accel_xy_cmss() const { return _accel_max_xy_cmss; }  //获取最大水平加速度

    // set the maximum horizontal position error that will be allowed in the horizontal plane
    void set_pos_error_max_xy_cm(float error_max) { _p_pos_xy.set_error_max(error_max); } //设置修正速度和加速度限制
    float get_pos_error_max_xy_cm() { return _p_pos_xy.get_error_max(); }  //获取最大水平速度和加速度

    /// init_xy_controller_stopping_point - initialise the position controller to the stopping point with zero velocity and acceleration.
    ///     This function should be used when the expected kinematic path assumes a stationary initial condition but does not specify a specific starting position.
    ///     The starting position can be retrieved by getting the position target using get_pos_target_cm() after calling this function.
    void init_xy_controller_stopping_point(); //初始化停止点

    // relax_velocity_controller_xy - initialise the position controller to the current position and velocity with decaying acceleration.
    ///     This function decays the output acceleration by 95% every half second to achieve a smooth transition to zero requested acceleration.
    void relax_velocity_controller_xy(); //平滑地放松速度控制器

    /// reduce response for landing
    void soften_for_landing_xy(); //着陆时减缓响应

    // init_xy_controller - initialise the position controller to the current position, velocity, acceleration and attitude.
    ///     This function is the default initialisation for any position control that provides position, velocity and acceleration.
    ///     This function is private and contains all the shared xy axis initialisation functions
    void init_xy_controller(); //初始化XY位置控制器

    /// input_accel_xy - calculate a jerk limited path from the current position, velocity and acceleration to an input acceleration.
    ///     The function takes the current position, velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
    ///     The kinematic path is constrained by the maximum acceleration and jerk set using the function set_max_speed_accel_xy.
    ///     The jerk limit defines the acceleration error decay in the kinematic path as the system approaches constant acceleration.
    ///     The jerk limit also defines the time taken to achieve the maximum acceleration.
    void input_accel_xy(const Vector3f& accel); //输入加速度并生成路径

    /// input_vel_accel_xy - calculate a jerk limited path from the current position, velocity and acceleration to an input velocity and acceleration.
    ///     The function takes the current position, velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
    ///     The kinematic path is constrained by the maximum acceleration and jerk set using the function set_max_speed_accel_xy.
    ///     The function alters the vel to be the kinematic path based on accel
    ///     The parameter limit_output specifies if the velocity and acceleration limits are applied to the sum of commanded and correction values or just correction.
    void input_vel_accel_xy(Vector2f& vel, const Vector2f& accel, bool limit_output = true); // 输入速度和加速度并生成路径

    /// input_pos_vel_accel_xy - calculate a jerk limited path from the current position, velocity and acceleration to an input position velocity and acceleration.
    ///     The function takes the current position, velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
    ///     The kinematic path is constrained by the maximum acceleration and jerk set using the function set_max_speed_accel_xy.
    ///     The function alters the pos and vel to be the kinematic path based on accel
    ///     The parameter limit_output specifies if the velocity and acceleration limits are applied to the sum of commanded and correction values or just correction.
    void input_pos_vel_accel_xy(Vector2p& pos, Vector2f& vel, const Vector2f& accel, bool limit_output = true); //输入位置、速度和加速度并生成路径

    // is_active_xy - returns true if the xy position controller has been run in the previous 5 loop times
    bool is_active_xy() const; //判断XY位置控制器是否活跃

    /// stop_pos_xy_stabilisation - sets the target to the current position to remove any position corrections from the system
    void stop_pos_xy_stabilisation(); //停止位置稳定

    /// stop_vel_xy_stabilisation - sets the target to the current position and velocity to the current velocity to remove any position and velocity corrections from the system
    void stop_vel_xy_stabilisation(); //停止速度稳定

    /// update_xy_controller - runs the horizontal position controller correcting position, velocity and acceleration errors.
    ///     Position and velocity errors are converted to velocity and acceleration targets using PID objects
    ///     Desired velocity and accelerations are added to these corrections as they are calculated
    ///     Kinematically consistent target position and desired velocity and accelerations should be provided before calling this function
    void update_xy_controller(); //更新水平位置控制器

    ///
    /// Vertical position controller
    ///

    /// set_max_speed_accel_z - set the maximum vertical speed in cm/s and acceleration in cm/s/s
    ///     speed_down can be positive or negative but will always be interpreted as a descent speed
    ///     This can be done at any time as changes in these parameters are handled smoothly
    ///     by the kinematic shaping.
    void set_max_speed_accel_z(float speed_down, float speed_up, float accel_cmss); //设置最大垂直速度和加速度

    /// set_correction_speed_accel_z - set the position controller correction velocity and acceleration limit
    ///     speed_down can be positive or negative but will always be interpreted as a descent speed
    ///     This should be done only during initialisation to avoid discontinuities
    void set_correction_speed_accel_z(float speed_down, float speed_up, float accel_cmss); //设置垂直误差修正的速度和加速度限制

    /// get_max_accel_z_cmss - get the maximum vertical acceleration in cm/s/s
    float get_max_accel_z_cmss() const { return _accel_max_z_cmss; } //获取最大垂直加速度

    // get_pos_error_z_up_cm - get the maximum vertical position error up that will be allowed
    float get_pos_error_z_up_cm() { return _p_pos_z.get_error_max(); } //获取最大垂直位置误差

    // get_pos_error_z_down_cm - get the maximum vertical position error down that will be allowed
    float get_pos_error_z_down_cm() { return _p_pos_z.get_error_min(); } 

    /// get_max_speed_up_cms - accessors for current maximum up speed in cm/s
    float get_max_speed_up_cms() const { return _vel_max_up_cms; }//获取最大上升和下降速度

    /// get_max_speed_down_cms - accessors for current maximum down speed in cm/s.  Will be a negative number
    float get_max_speed_down_cms() const { return _vel_max_down_cms; }

    /// init_z_controller_no_descent - initialise the position controller to the current position, velocity, acceleration and attitude.
    ///     This function is the default initialisation for any position control that provides position, velocity and acceleration.
    ///     This function does not allow any negative velocity or acceleration
    void init_z_controller_no_descent(); //初始化垂直位置控制器（禁止下降）

    /// init_z_controller_stopping_point - initialise the position controller to the stopping point with zero velocity and acceleration.
    ///     This function should be used when the expected kinematic path assumes a stationary initial condition but does not specify a specific starting position.
    ///     The starting position can be retrieved by getting the position target using get_pos_target_cm() after calling this function.
    void init_z_controller_stopping_point(); //初始化为静止状态

    // relax_z_controller - initialise the position controller to the current position and velocity with decaying acceleration.
    ///     This function decays the output acceleration by 95% every half second to achieve a smooth transition to zero requested acceleration.
    void relax_z_controller(float throttle_setting); //平滑衰减Z轴加速度

    // init_z_controller - initialise the position controller to the current position, velocity, acceleration and attitude.
    ///     This function is the default initialisation for any position control that provides position, velocity and acceleration.
    ///     This function is private and contains all the shared z axis initialisation functions
    void init_z_controller(); //初始化Z轴控制器

    /// input_accel_z - calculate a jerk limited path from the current position, velocity and acceleration to an input acceleration.
    ///     The function takes the current position, velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
    ///     The kinematic path is constrained by the maximum acceleration and jerk set using the function set_max_speed_accel_z.
    virtual void input_accel_z(float accel); //输入加速度并生成路径

    /// input_vel_accel_z - calculate a jerk limited path from the current position, velocity and acceleration to an input velocity and acceleration.
    ///     The function takes the current position, velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
    ///     The kinematic path is constrained by the maximum acceleration and jerk set using the function set_max_speed_accel_z.
    ///     The function alters the vel to be the kinematic path based on accel
    ///     The parameter limit_output specifies if the velocity and acceleration limits are applied to the sum of commanded and correction values or just correction.
    virtual void input_vel_accel_z(float &vel, float accel, bool limit_output = true); //输入速度和加速度并生成路径

    /// set_pos_target_z_from_climb_rate_cm - adjusts target up or down using a commanded climb rate in cm/s
    ///     using the default position control kinematic path.
    ///     The zero target altitude is varied to follow pos_offset_z
    void set_pos_target_z_from_climb_rate_cm(float vel); //通过爬升率调整目标位置

    /// land_at_climb_rate_cm - adjusts target up or down using a commanded climb rate in cm/s
    ///     using the default position control kinematic path.
    ///     ignore_descent_limit turns off output saturation handling to aid in landing detection. ignore_descent_limit should be true unless landing.
    void land_at_climb_rate_cm(float vel, bool ignore_descent_limit); //以指定爬升率着陆

    /// input_pos_vel_accel_z - calculate a jerk limited path from the current position, velocity and acceleration to an input position velocity and acceleration.
    ///     The function takes the current position, velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
    ///     The function alters the pos and vel to be the kinematic path based on accel
    ///     The parameter limit_output specifies if the velocity and acceleration limits are applied to the sum of commanded and correction values or just correction.
    void input_pos_vel_accel_z(float &pos, float &vel, float accel, bool limit_output = true); //输入位置、速度和加速度并生成路径

    /// set_alt_target_with_slew - adjusts target up or down using a commanded altitude in cm
    ///     using the default position control kinematic path.
    void set_alt_target_with_slew(float pos); //以指定高度目标调整位置

    /// update_pos_offset_z - updates the vertical offsets used by terrain following
    void update_pos_offset_z(float pos_offset); // 更新垂直偏移量（用于地形跟踪）

    // is_active_z - returns true if the z position controller has been run in the previous 5 loop times
    bool is_active_z() const; //判断Z轴控制器是否活跃

    /// update_z_controller - runs the vertical position controller correcting position, velocity and acceleration errors.
    ///     Position and velocity errors are converted to velocity and acceleration targets using PID objects
    ///     Desired velocity and accelerations are added to these corrections as they are calculated
    ///     Kinematically consistent target position and desired velocity and accelerations should be provided before calling this function
    void update_z_controller(); //更新垂直位置控制器



    ///
    /// Accessors //访问器
    ///

    /// set commanded position (cm), velocity (cm/s) and acceleration (cm/s/s) inputs when the path is created externally. //设置3D位置、速度和加速度，通常不会直接作用于控制器
    void set_pos_vel_accel(const Vector3p& pos, const Vector3f& vel, const Vector3f& accel);
    void set_pos_vel_accel_xy(const Vector2p& pos, const Vector2f& vel, const Vector2f& accel);


    /// Position //位置管理

    /// set_pos_target_xy_cm - sets the position target, frame NEU in cm relative to the EKF origin  //设置水平面（XY平面）的目标位置
    void set_pos_target_xy_cm(float pos_x, float pos_y) { _pos_target.x = pos_x; _pos_target.y = pos_y; }

    /// get_pos_target_cm - returns the position target, frame NEU in cm relative to the EKF origin //获取当前的目标位置
    const Vector3p& get_pos_target_cm() const { return _pos_target; }

    /// set_pos_target_z_cm - set altitude target in cm above the EKF origin //设置垂直方向（Z轴）的目标高度
    void set_pos_target_z_cm(float pos_target) { _pos_target.z = pos_target; }

    /// get_pos_target_z_cm - get target altitude (in cm above the EKF origin) //获取当前目标高度
    float get_pos_target_z_cm() const { return _pos_target.z; }

    /// get_stopping_point_xy_cm - calculates stopping point in NEU cm based on current position, velocity, vehicle acceleration //计算水平面（XY平面）的停止点
    void get_stopping_point_xy_cm(Vector2p &stopping_point) const;

    /// get_stopping_point_z_cm - calculates stopping point in NEU cm based on current position, velocity, vehicle acceleration //计算垂直方向（Z轴）的停止点
    void get_stopping_point_z_cm(postype_t &stopping_point) const;

    /// get_pos_error_cm - get position error vector between the current and target position //获取当前位置与目标位置的误差向量
    const Vector3f get_pos_error_cm() const { return (_pos_target - _inav.get_position_neu_cm().topostype()).tofloat(); }

    /// get_pos_error_xy_cm - get the length of the position error vector in the xy plane //获取水平面（XY平面）的误差距离
    float get_pos_error_xy_cm() const { return get_horizontal_distance_cm(_inav.get_position_xy_cm().topostype(), _pos_target.xy()); }

    /// get_pos_error_z_cm - returns altitude error in cm //获取垂直方向的高度误差
    float get_pos_error_z_cm() const { return (_pos_target.z - _inav.get_position_z_up_cm()); }


    /// Velocity //速度管理

    /// set_vel_desired_cms - sets desired velocity in NEU cm/s //设置期望的3D速度
    void set_vel_desired_cms(const Vector3f &des_vel) { _vel_desired = des_vel; }

    /// set_vel_desired_xy_cms - sets horizontal desired velocity in NEU cm/s //设置水平面的期望速度
    void set_vel_desired_xy_cms(const Vector2f &vel) {_vel_desired.xy() = vel; }

    /// get_vel_desired_cms - returns desired velocity (i.e. feed forward) in cm/s in NEU //获取期望速度
    const Vector3f& get_vel_desired_cms() { return _vel_desired; }

    // get_vel_target_cms - returns the target velocity in NEU cm/s //获取目标速度
    const Vector3f& get_vel_target_cms() const { return _vel_target; }

    /// set_vel_desired_z_cms - sets desired velocity in cm/s in z axis //设置垂直方向的期望速度
    void set_vel_desired_z_cms(float vel_z_cms) {_vel_desired.z = vel_z_cms;}

    /// get_vel_target_z_cms - returns target vertical speed in cm/s //获取垂直方向的目标速度
    float get_vel_target_z_cms() const { return _vel_target.z; }


    /// Acceleration //加速度管理

    // set_accel_desired_xy_cmss set desired acceleration in cm/s in xy axis //设置期望的XY轴加速度
    void set_accel_desired_xy_cmss(const Vector2f &accel_cms) { _accel_desired.xy() = accel_cms; }

    // get_accel_target_cmss - returns the target acceleration in NEU cm/s/s //获取目标加速度
    const Vector3f& get_accel_target_cmss() const { return _accel_target; }


    /// Offset //偏移量管理

    /// set_pos_offset_target_z_cm - set altitude offset target in cm above the EKF origin //设置目标高度的偏移量
    void set_pos_offset_target_z_cm(float pos_offset_target_z) { _pos_offset_target_z = pos_offset_target_z; }

    /// set_pos_offset_z_cm - set altitude offset in cm above the EKF origin //设置当前高度的偏移量
    void set_pos_offset_z_cm(float pos_offset_z) { _pos_offset_z = pos_offset_z; }

    /// get_pos_offset_z_cm - returns altitude offset in cm above the EKF origin //获取当前高度的偏移量
    float get_pos_offset_z_cm() const { return _pos_offset_z; }

    /// get_vel_offset_z_cm - returns current vertical offset speed in cm/s //获取当前 Z 轴方向的速度偏移量
    float get_vel_offset_z_cms() const { return _vel_offset_z; }

    /// get_accel_offset_z_cm - returns current vertical offset acceleration in cm/s/s //获取当前 Z 轴方向的加速度偏移量
    float get_accel_offset_z_cmss() const { return _accel_offset_z; }


    /// Outputs //输出控制信号

    /// get desired roll and pitch to be passed to the attitude controller //获取期望的横滚角和俯仰角
    float get_roll_cd() const { return _roll_target; }
    float get_pitch_cd() const { return _pitch_target; }

    /// get desired yaw to be passed to the attitude controller //获取期望的偏航角和偏航速率
    float get_yaw_cd() const { return _yaw_target; }

    /// get desired yaw rate to be passed to the attitude controller
    float get_yaw_rate_cds() const { return _yaw_rate_target; }

    /// get desired roll and pitch to be passed to the attitude controller //获取推力矢量
    Vector3f get_thrust_vector() const;

    /// get_bearing_to_target_cd - get bearing to target position in centi-degrees //获取目标位置的方位角
    int32_t get_bearing_to_target_cd() const;

    /// get_lean_angle_max_cd - returns the maximum lean angle the autopilot may request //获取和设置最大倾斜角
    float get_lean_angle_max_cd() const;

    /*
      set_lean_angle_max_cd - set the maximum lean angle. A value of zero means to use the ANGLE_MAX parameter.
      This is reset to zero on init_xy_controller()
    */
    void set_lean_angle_max_cd(float angle_max_cd) { _angle_max_override_cd = angle_max_cd; }
    

    /// Other

    /// get pid controllers
    AC_P_2D& get_pos_xy_p() { return _p_pos_xy; } //获取 X-Y 轴的位置控制器
    AC_P_1D& get_pos_z_p() { return _p_pos_z; }  ////获取 Z 轴的位置控制器
    AC_PID_2D& get_vel_xy_pid() { return _pid_vel_xy; } //获取 X-Y 轴的速度控制器
    AC_PID_Basic& get_vel_z_pid() { return _pid_vel_z; }  //获取 Z 轴的速度控制器
    AC_PID& get_accel_z_pid() { return _pid_accel_z; } //获取 Z 轴的加速度控制器

    /// set_limit_accel_xy - mark that accel has been limited
    ///     this prevents integrator buildup //限制加速度积累
    void set_externally_limited_xy() { _limit_vector.x = _accel_target.x; _limit_vector.y = _accel_target.y; }

    // lean_angles_to_accel - convert roll, pitch lean angles to lat/lon frame accelerations in cm/s/s //姿态角到加速度的转换
    Vector3f lean_angles_to_accel(const Vector3f& att_target_euler) const;

    // write PSC and/or PSCZ logs //写入日志
    void write_log();

    // provide feedback on whether arming would be a good idea right now: 预启动检查
    bool pre_arm_checks(const char *param_prefix,
                        char *failure_msg,
                        const uint8_t failure_msg_len);

    // enable or disable high vibration compensation //高振动补偿
    void set_vibe_comp(bool on_off) { _vibe_comp_enabled = on_off; }

    /// get_vel_z_error_ratio - returns the proportion of error relative to the maximum request //获取 Z 轴速度误差比例
    float get_vel_z_control_ratio() const { return constrain_float(_vel_z_control_ratio, 0.0f, 1.0f); }

    /// crosstrack_error - returns horizontal error to the closest point to the current track //水平轨道偏离误差
    float crosstrack_error() const; 

    /// standby_xyz_reset - resets I terms and removes position error
    ///     This function will let Loiter and Alt Hold continue to operate
    ///     in the event that the flight controller is in control of the
    ///     aircraft when in standby.
    void standby_xyz_reset(); //积分器 I 项重置，避免积分器过度累积。

    // get earth-frame Z-axis acceleration with gravity removed in cm/s/s with +ve being up //获取地球坐标系下的 Z 轴加速度（不包括重力）。
    float get_z_accel_cmss() const { return -(_ahrs.get_accel_ef().z + GRAVITY_MSS) * 100.0f; }

    /// returns true when the forward pitch demand is limited by the maximum allowed tilt //最大前倾俯仰限制
    bool get_fwd_pitch_is_limited() const { return _fwd_pitch_is_limited; }
    
    // set disturbance north //设置位置扰动
    void set_disturb_pos_cm(Vector2f disturb_pos) {_disturb_pos = disturb_pos;}

    // set disturbance north //设置速度扰动
    void set_disturb_vel_cms(Vector2f disturb_vel) {_disturb_vel = disturb_vel;}

    static const struct AP_Param::GroupInfo var_info[];//声明一个静态常量成员，类型为 AP_Param::GroupInfo 的结构体，用于存储参数表。//这里struct是为了可读性强调，实际上AP_Param::GroupInfo 已经是结构体类型
    //日志记录函数
    static void Write_PSCN(float pos_target, float pos, float vel_desired, float vel_target, float vel, float accel_desired, float accel_target, float accel);
    static void Write_PSCE(float pos_target, float pos, float vel_desired, float vel_target, float vel, float accel_desired, float accel_target, float accel);
    static void Write_PSCD(float pos_target, float pos, float vel_desired, float vel_target, float vel, float accel_desired, float accel_target, float accel);

protected:

    // get throttle using vibration-resistant calculation (uses feed forward with manually calculated gain) //油门振动补偿
    float get_throttle_with_vibration_override();

    // lean_angles_to_accel - convert roll, pitch lean angles to lat/lon frame accelerations in cm/s/s //姿态角到加速度的转换
    void accel_to_lean_angles(float accel_x_cmss, float accel_y_cmss, float& roll_target, float& pitch_target) const;

    // lean_angles_to_accel - convert roll, pitch lean angles to lat/lon frame accelerations in cm/s/s
    void lean_angles_to_accel_xy(float& accel_x_cmss, float& accel_y_cmss) const;

    // calculate_yaw_and_rate_yaw - calculate the vehicle yaw and rate of yaw. //偏航角与偏航速率计算
    void calculate_yaw_and_rate_yaw();

    // calculate_overspeed_gain - calculated increased maximum acceleration and jerk if over speed condition is detected //超速增益计算
    float calculate_overspeed_gain();

    /// initialise and check for ekf position resets //EKF复位处理
    void init_ekf_xy_reset();
    void handle_ekf_xy_reset();
    void init_ekf_z_reset();
    void handle_ekf_z_reset();

    // references to inertial nav and ahrs libraries //内部参数与控制器引用（取别名）
    AP_AHRS_View&           _ahrs;
    const AP_InertialNav&   _inav;
    const class AP_Motors&  _motors;
    AC_AttitudeControl&     _attitude_control;

    // parameters //控制器变量 （声明类创建的对象）
    AP_Float        _lean_angle_max;    // Maximum autopilot commanded angle (in degrees). Set to zero for Angle Max
    AP_Float        _shaping_jerk_xy;   // Jerk limit of the xy kinematic path generation in m/s^3 used to determine how quickly the aircraft varies the acceleration target
    AP_Float        _shaping_jerk_z;    // Jerk limit of the z kinematic path generation in m/s^3 used to determine how quickly the aircraft varies the acceleration target
    AC_P_2D         _p_pos_xy;          // XY axis position controller to convert distance error to desired velocity
    AC_P_1D         _p_pos_z;           // Z axis position controller to convert altitude error to desired climb rate
    AC_PID_2D       _pid_vel_xy;        // XY axis velocity controller to convert velocity error to desired acceleration
    AC_PID_Basic    _pid_vel_z;         // Z axis velocity controller to convert climb rate error to desired acceleration
    AC_PID          _pid_accel_z;       // Z axis acceleration controller to convert desired acceleration to throttle output

    // internal variables //内部变量
    float       _dt;                    // time difference (in seconds) since the last loop time
    uint32_t    _last_update_xy_ticks;  // ticks of last last update_xy_controller call
    uint32_t    _last_update_z_ticks;   // ticks of last update_z_controller call
    float       _vel_max_xy_cms;        // max horizontal speed in cm/s used for kinematic shaping
    float       _vel_max_up_cms;        // max climb rate in cm/s used for kinematic shaping
    float       _vel_max_down_cms;      // max descent rate in cm/s used for kinematic shaping
    float       _accel_max_xy_cmss;     // max horizontal acceleration in cm/s/s used for kinematic shaping
    float       _accel_max_z_cmss;      // max vertical acceleration in cm/s/s used for kinematic shaping
    float       _jerk_max_xy_cmsss;       // Jerk limit of the xy kinematic path generation in cm/s^3 used to determine how quickly the aircraft varies the acceleration target
    float       _jerk_max_z_cmsss;        // Jerk limit of the z kinematic path generation in cm/s^3 used to determine how quickly the aircraft varies the acceleration target
    float       _vel_z_control_ratio = 2.0f;    // confidence that we have control in the vertical axis
    Vector2f    _disturb_pos;           // position disturbance generated by system ID mode
    Vector2f    _disturb_vel;           // velocity disturbance generated by system ID mode

    // output from controller //控制器输出
    float       _roll_target;           // desired roll angle in centi-degrees calculated by position controller
    float       _pitch_target;          // desired roll pitch in centi-degrees calculated by position controller
    float       _yaw_target;            // desired yaw in centi-degrees calculated by position controller
    float       _yaw_rate_target;       // desired yaw rate in centi-degrees per second calculated by position controller

    // position controller internal variables //控制器与内部变量
    Vector3p    _pos_target;            // target location, frame NEU in cm relative to the EKF origin
    Vector3f    _vel_desired;           // desired velocity in NEU cm/s
    Vector3f    _vel_target;            // velocity target in NEU cm/s calculated by pos_to_rate step
    Vector3f    _accel_desired;         // desired acceleration in NEU cm/s/s (feed forward)
    Vector3f    _accel_target;          // acceleration target in NEU cm/s/s
    Vector3f    _limit_vector;          // the direction that the position controller is limited, zero when not limited

    bool        _fwd_pitch_is_limited;     // true when the forward pitch demand is being limited to meet acceleration limits

    float       _pos_offset_target_z;   // vertical position offset target, frame NEU in cm relative to the EKF origin
    float       _pos_offset_z;          // vertical position offset, frame NEU in cm relative to the EKF origin
    float       _vel_offset_z;          // vertical velocity offset in NEU cm/s calculated by pos_to_rate step
    float       _accel_offset_z;        // vertical acceleration offset in NEU cm/s/s

    // ekf reset handling
    uint32_t    _ekf_xy_reset_ms;       // system time of last recorded ekf xy position reset
    uint32_t    _ekf_z_reset_ms;        // system time of last recorded ekf altitude reset

    // high vibration handling //高振动补偿和最大倾角
    bool        _vibe_comp_enabled;     // true when high vibration compensation is on

    // angle max override, if zero then use ANGLE_MAX parameter
    float       _angle_max_override_cd;

    // return true if on a real vehicle or SITL with lock-step scheduling //高层次函数
    bool has_good_timing(void) const;

private:
    // convenience method for writing out the identical PSCE, PSCN, PSCD - and
    // to save bytes
    static void Write_PSCx(LogMessages ID, float pos_target, float pos, float vel_desired, float vel_target, float vel, float accel_desired, float accel_target, float accel);

};
