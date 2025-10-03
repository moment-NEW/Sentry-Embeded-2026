# alg_pid
## 概述
`alg_pid` 是一个用于实现 PID 控制算法的库。PID 控制器是一种广泛应用于工业控制系统中的反馈回路控制器，能够根据误差值（设定值与实际值之间的差异）来调整系统输出，以达到期望的目标。

## 环境
- 硬件平台: stm32
- 相关库: HAL, CMSIS, FreeRTOS

## 使用说明
1. 添加 `alg_pid` 库到项目
2. 在需要使用 PID 控制的模块中包含 `alg_pid.h` 头文件
3. 配置一个 `pid_config` ，注意不需要的参数可以不设置
4. 调用 `pid_regiter` 函数注册 PID 控制器
5. 在主循环或定时器中调用 `Pid_Calculate` 函数来计算目标值

## 实例说明
```c
typedef struct{
    float kp;                        //比例系数
    float ki;                        //计数系数
    float kd;                        //微分系数
    float kf;                        //前馈系数
    float i_max;                     //积分限幅
    float out_max;                   //输出限幅
    float dead_zone;                 //死区
    float angle_max;                 //角度限幅
    float i_variable_min;            //变速积分下限,,设0时值与i_variable_min相同
    float i_variable_max;            //变速积分上限,设0时值与i_variable_min相同
    uint8_t d_first;                 //微分先行开关
    float target[2];                 //两次设定值
    float now[2];                    //两次实际值
    float err[2];                    //两次误差
    float p_out;                     //比例输出
    float i_out;                     //积分输出
    float d_out;                     //微分输出
    float f_out;                     //前馈输出
    float output;                    //输出值
}PidInstance_s;           

typedef struct{           
    float kp;                        //比例系数
    float ki;                        //计数系数
    float kd;                        //微分系数
    float kf;                        //前馈系数
    float angle_max;                 //角度最大值(限幅用，为0则不限幅)
    float i_max;                     //积分限幅(大于输出限幅时，默认大小与输出限幅相等)
    float out_max;                   //输出限幅
    float dead_zone;                 //死区
    float i_variable_min;            //变速积分下限,积分分离的值,为0时没有积分分离
    float i_variable_max;            //变速积分上限，为0时只有积分分离,小于min时值与i_variable_min相同
    uint8_t d_first;                 //微分先行开关(1开0关)
}PidInitConfig_s;
```

## 外部接口
```c
/**
 * @file alg_pid.h
 * @brief PID控制器注册函数
 * @details 该函数用于注册一个PID控制器实例，初始化相关参数。
 * @param config Pid初始化配置结构体指针
 * @return PidInstance_s指针，注册成功返回指针，失败返回NULL
 * @date 2025-07-01
 */
PidInstance_s *Pid_Register(PidInitConfig_s *config);

/**
 * @file alg_pid.h
 * @brief PID控制器计算函数
 * @details 该函数用于计算PID控制器的输出值。
 * @param PID PidInstance_s指针，PID控制器实例
 * @param target 目标值
 * @param now 当前值
 * @return 成功返回计算后的输出值，失败返回0.0f
 * @date 2025-07-01
 */
float Pid_Calculate(PidInstance_s *pid, float target, float now);

/**
 * @file alg_pid.h
 * @brief PID控制器清理函数
 * @details 该函数用于清理把Pid实例中的所有参数清零
 * @param pid PidInstance_s指针，PID控制器实例
 * @return 成功返回true，失败返回false
 * @note 该函数通常在电机失联后调用，以清除PID控制器的状态
 * @note 该函数在调用后不可恢复，只能通过重启程序解决
 * @date 2025-07-01
 */
bool Pid_Clean(PidInstance_s *pid);

/**
 * @file alg_pid.h
 * @brief PID控制器重置函数
 * @param pid PidInstance_s指针，PID控制器实例
 * @return 成功返回true，失败返回false
 * @date 2025-07-01
 */
bool Pid_Reset(PidInstance_s *pid);
```

## 内部函数
```c
static void Pid_Protect(PidInstance_s *pid)
{
    float half_angle = pid->angle_max / 2;
    if (pid->target[0] - pid->now[0] > half_angle)
    {
        pid->now[0] += pid->angle_max;
    }
    else if (pid->target[0] - pid->now[0] < -half_angle)
    {
        pid->now[0] -= pid->angle_max;
    }
}
/**
 * @brief 限制一个整数变量 value 在指定的最小值 min 和最大值 max 之间
 * @param value 输入值
 * @param min 最小值
 * @param max 最大值
 * @return 计算结果
 */
static float Pid_Limit(float value, float min, float max)
{
    if (value < min)
        return min;
    else if (value > max)
        return max;
    else
        return value;
}
```
这里的 `Pid_Limit` 封装到一个新文件里面

## 注意事项
- 在配置 `config` 时，确保所有参数都符合实际应用需求，且注意要配置outmax和i_max的大小,否则输出恒为0
- 在使用 PID 控制器时，确保定期调用 `Pid_Calculate` 函数，以确保控制器能够及时响应目标值的变化
- 如果是角度环，注意配置过零保护，即把`angle_max` 设置为角度范围的绝对值


