int8 mode
float32 q
float32 dq
float32 ddq
float32 tau_est
# 电机温度信息：类型：int8_t ，可按照实际数值显示（范围：-100 - 150）。
int8 temperature
# 电机丢包信息：可按照实际数值显示（范围：0-9999999999）。
uint32 lost
# 当前电机通信频率+电机错误标志位：（数组：0-电机错误标志位（范围：0-255，可按照实际数值显示），1-当前电机通信频率（范围：0-800，可按照实际数值显示））
uint32[2] reserve