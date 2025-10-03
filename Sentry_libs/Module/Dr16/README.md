# dev_dr16
## 概述
`dev_dr16`是一个用于处理DR16遥控器数据的驱动模块，支持通过DMA接收数据并解析遥控器的输入。
## 注意事项
1. 请将DMA配置成适合DR16的循环模式
2. 目前只做了C板适配，如果需要使用其他开发板，请自行修改为正确的串口
3. 如不可避免的还是发生数据错位，请进行reset操作


## 使用例程
```c
Dr16Instance_s *dr16_instance = NULL;
void App_DebugTask(void const * argument)
{
    dr16_instance = Dr16_Register(&huart3); // 注册遥控器实例
    while (1) {
        osDelay(1);
    }
}
```

## 维护事项
1. 目前使用单缓冲区，其实已经足够
2. 没有强制进行帧校验，只是假定传输只有错位没有丢数据，增加了偏移量用于在数据发生偏移时也正常解析数据