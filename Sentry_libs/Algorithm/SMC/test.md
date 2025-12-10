# C版本滑模控制库使用说明

## 概述
本库实现滑模控制（Sliding Mode Control, SMC）的C版本，支持多种控制模式（EXPONENT、POWER、TFSMC、VELSMC、EISMC）。适用于位置环和速度环控制，如机器人或无人机系统。库基于结构体 `Sliding` 和相关函数，提供初始化、参数设置、误差更新和输出计算功能。

## 文件结构
- `alg_Sliding.h`：头文件，定义结构体、枚举和函数声明。
- alg_Sliding.c：实现文件，包含所有函数逻辑。

## 包含和依赖
在项目中包含头文件：
```c
#include "alg_Sliding.h"
```
依赖：`math.h`（用于 `powf` 和 `fabsf`）。

## API概述
- **初始化**：`smc_init(Sliding *smc)` - 初始化结构体。
- **参数设置**：
  - `smc_set_param_exp` - 设置EXPONENT/POWER/VELSMC参数。
  - `smc_set_param_tfsmc` - 设置TFSMC参数。
  - `smc_set_param_eismc` - 设置EISMC参数。
- **误差更新**：
  - `smc_error_update_pos` - 位置环误差更新。
  - `smc_error_update_vel` - 速度环误差更新。
- **计算**：`smc_calculate(Sliding *smc)` - 计算控制输出。
- **其他**：`smc_clear`、`smc_integral_clear`、`smc_get_out`、`smc_set_out`。

## 使用步骤
1. **声明和初始化**：
   ```c
   Sliding smc;
   smc_init(&smc);
   ```

2. **设置参数**（根据模式选择）：
   - EXPONENT/POWER/VELSMC：
     ```c
     smc_set_param_exp(&smc, J, K, c, epsilon, limit, u_max, flag, pos_eps);
     ```
   - TFSMC：
     ```c
     smc_set_param_tfsmc(&smc, J, K, p, q, beta, epsilon, limit, u_max, flag, pos_eps);
     ```
   - EISMC：
     ```c
     smc_set_param_eismc(&smc, J, K, c1, c2, epsilon, limit, u_max, flag, pos_eps);
     ```

3. **运行循环**（位置环示例）：
   ```c
   while (1) {
       // 获取当前位置和速度
       float target = get_target();
       float pos_now = get_position();
       float vol_now = get_velocity();
       
       // 更新误差
       smc_error_update_pos(&smc, target, pos_now, vol_now);
       
       // 计算输出
       float output = smc_calculate(&smc);
       
       // 应用输出
       apply_output(output);
   }
   ```

4. **获取输出**：
   ```c
   float out = smc_get_out(&smc);
   ```

## 调参指南
调参需根据系统特性（如惯性、响应速度）通过实验调整。建议从小值开始，逐步增大，观察稳定性、振荡和跟踪精度。使用仿真工具验证。

### 通用参数
- **J**：系统惯性系数。增大提高稳定性，减小提高响应速度。典型值：0.1-10。
- **K**：滑模增益。增大减少滑模面误差，易引起振荡。典型值：1-100。
- **epsilon**：边界层厚度（饱和函数参数）。增大减少抖动，减小提高精度。典型值：0.01-1。
- **limit**：饱和限幅。控制饱和函数范围，防止过大输出。典型值：0.1-10。
- **u_max**：输出限幅。防止控制量过大。典型值：根据系统最大力矩/电压。
- **pos_eps**：位置误差死区。忽略小误差，避免抖动。典型值：0.001-0.1。

### 模式特定参数
- **EXPONENT**（指数趋近率，位置控制）：
  - **c**：滑模面系数。增大提高速度跟踪。典型值：1-10。
  - 调参：先调K至无振荡，再调epsilon减少抖动，最后调c优化跟踪。

- **POWER**（幂次趋近率，位置控制）：
  - **c**：同EXPONENT。
  - 调参：类似EXPONENT，但epsilon影响幂次项，较小值更平滑。

- **TFSMC**（终端滑模控制，位置控制）：
  - **p, q**：幂次参数（p > q > 0）。控制收敛速度。典型：p=9, q=7。
  - **beta**：滑模面系数。增大提高稳定性。典型值：1-10。
  - 调参：先调p/q确定收敛特性，再调beta和K。

- **VELSMC**（速度控制）：
  - **c**：积分系数。增大提高积分效果。典型值：0.1-1。
  - 调参：K控制滑模，c控制积分，epsilon减少抖动。

- **EISMC**（增强积分滑模，位置控制）：
  - **c1, c2**：比例和积分系数。c1类似c，c2控制积分。典型：c1=1-10, c2=0.1-1。
  - 调参：类似EXPONENT，但c2需小心避免积分饱和。

### 调参步骤
1. 固定其他参数，调整K观察稳定性。
2. 调整epsilon减少高频抖动。
3. 微调J和c优化响应。
4. 使用积分清除（`smc_integral_clear`）重置测试。
5. 记录积分限幅（宏定义V_ERROR_INTEGRAL_MAX/P_ERROR_INTEGRAL_MAX），若积分过大，可启用限幅。

## 示例代码
```c
#include "alg_Sliding.h"

int main() {
    Sliding smc;
    smc_init(&smc);
    smc_set_param_exp(&smc, 1.0f, 10.0f, 5.0f, 0.1f, 1.0f, 100.0f, EXPONENT, 0.01f);
    
    // 模拟循环
    for (int i = 0; i < 100; i++) {
        smc_error_update_pos(&smc, 10.0f, 5.0f, 0.5f);  // 示例值
        float u = smc_calculate(&smc);
        printf("Output: %f\n", u);
    }
    return 0;
}
```

## 注意事项
- 采样周期 `SAMPLE_PERIOD` 固定为0.002s，需匹配实际周期。
- 积分项未默认限幅，可根据宏定义添加检查。
- 空指针检查已内置，但确保传入有效结构体。
- 调试时监控 `smc->s`（滑模面）和 `smc->u`（输出）。
- 若系统不稳定，检查参数单位一致性。