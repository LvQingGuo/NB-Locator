# [NB-IOT定位器] 复盘笔记
## 一、项目概述
### 1.1基础信息

- **项目周期**：2025.8.1 - 2025.8.25
- **核心目标**：开发一款集成 NB-IoT 通信、GPS 定位、电池管理功能的物联网终端，实现定位数据与电池状态向 OneNET 云平台的远程上报
- **硬件架构**：STM32L031F6Px（主控）+ BC20（NB-IoT/GNSS 模块）+ 锂电池（供电）
- **软件环境**：STM32CubeMX + HAL 库 + KEIL MDK
- **附件目录**：`D:\项目复盘合集\1-物联网NB-IoT定位器\attachments`

### 1.2核心功能清单

| 模块     | 核心功能                               | 技术实现                         |
| -------- | -------------------------------------- | -------------------------------- |
| 通信模块 | NB-IoT 网络接入、LwM2M 协议对接 OneNET | BC20 模块 AT 指令、MIPL 协议指令 |
| 定位模块 | GPS 位置采集与解析                     | NMEA 格式 RMC 数据解析           |
| 电源管理 | 电池电压监测、电量估算、低电量保护     | ADC 采样、电压 - 电量映射算法    |
| 调试功能 | 双串口数据转发、状态日志输出           | UART 中断接收、printf 重定向     |

## 二、项目代码解析

### 2.1main.c 文件

#### 2.1.1全局变量定义

```markdown
1. 串口通信模块（外部变量，在stm32l0xx_it.c中定义）
2. 电池状态管理模块
3. NB 模块 - OneNET 通信模块
4. GNSS 定位数据模块
5. 定时中断控制
6. 低电量关机控制
```

#### 	**2.1.2**硬件初始化部分

```c
HAL_GPIO_WritePin(PWR_LED_GPIO_Port, PWR_LED_Pin, GPIO_PIN_RESET); // 点亮电源指示灯
HAL_GPIO_WritePin(GPIOA, PWR_EN_Pin, GPIO_PIN_SET);				   // 开启电源
while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == 0); 				   // 等待硬件就绪
// 初始化NB模块（BC20）：上电、复位
HAL_GPIO_WritePin(GPIOA, NB_PWR_Pin, GPIO_PIN_SET);
HAL_Delay(600);
HAL_GPIO_WritePin(GPIOA, NB_PWR_Pin, GPIO_PIN_RESET);
HAL_GPIO_WritePin(GPIOA, NB_RST_Pin, GPIO_PIN_SET);
HAL_Delay(60);
HAL_GPIO_WritePin(GPIOA, NB_RST_Pin, GPIO_PIN_RESET);
HAL_Delay(10000); // 等待模块启动
```

#### 	**2.1.3**NB-IoT 联网与定位GNSS开启与 LwM2M 协议对接

```markdown
1. AT命令测试模块正常
2. 关闭回显(ATE0)
3. 检测信号强度(AT+CSQ)
4. 等待网络注册(AT+CEREG?)
5. 等待PS附着(AT+CGATT?)
```

​	**联网流程**：

1. 模块自检（AT 指令测试）→ 关闭回显（ATE0）
2. 信号强度检测（AT+CSQ，筛选有效信号值：1~31）
3. 网络注册（AT+CEREG? 等待返回 0,1）→ PS 附着（AT+CGATT? 等待返回 1）

```c
// 检查NB模块是否就绪（发送AT指令，等待OK响应）
while (Send_Cmd((uint8_t *)"AT\r\n", 4, "OK") != 0)
{
    HAL_Delay(1000);
}
printf("BC20模块就绪\r\n");

// 关闭NB模块回显（避免接收冗余数据）
while (Send_Cmd((uint8_t *)"ATE0\r\n", 6, "OK") != 0)
{
    HAL_Delay(1000);
}
printf("已关闭回显\r\n");

// 获取NB模块信号强度（解析+CSQ指令响应）
NB_Signal_Value = 0;
while ((NB_Signal_Value == 0) || (NB_Signal_Value == 99)) // 0和99为无效信号
{
    if (Send_Cmd((uint8_t *)"AT+CSQ\r\n", 8, "OK") == 0)
    {
        if (Res1_Buf[2] == '+')
        {
            if (Res1_Buf[9] == ',') // 信号值为个位数
                NB_Signal_Value = Res1_Buf[8] - 0x30;
            else if (Res1_Buf[10] == ',') // 信号值为两位数
                NB_Signal_Value = (Res1_Buf[8] - 0x30) * 10 + (Res1_Buf[9] - 0x30);
        }
    }
    HAL_Delay(1000);
}
printf("NB_Signal_Value=%d\r\n", NB_Signal_Value);

// 等待NB模块注册到EPS网络（+CEREG: 0,1表示注册成功）
while (Send_Cmd((uint8_t *)"AT+CEREG?\r\n", 11, "+CEREG: 0,1") != 0)
{
    HAL_Delay(1000);
}
printf("EPS网络注册成功\r\n");

// 等待PS附着成功（+CGATT: 1表示附着成功）
while (Send_Cmd((uint8_t *)"AT+CGATT?\r\n", 11, "+CGATT: 1") != 0)
{
    HAL_Delay(1000);
}
printf("PS已附着\r\n");
```

```markdown
6. 打开GNSS定位功能
```

```c
// 开启GNSS模块（卫星定位）
while (Send_Cmd((uint8_t *)"AT+QGNSSC=1\r\n", 13, "OK") != 0)
{
    HAL_Delay(1000);
}
printf("GNSS功能已开启\r\n");
HAL_Delay(2000);

// 确认GNSS模块已开启
while (Send_Cmd((uint8_t *)"AT+QGNSSC?\r\n", 12, "+QGNSSC: 1") != 0)
{
    HAL_Delay(1000);
}
printf("GNSS模块已开启\r\n");
```

```markdown
7. 创建OneNET通信实例
8. 添加对象资源(3320, 3336)
9. 向OneNET注册
```

**LwM2M 协议对接（OneNET）**：

```c
// 创建LwM2M实例（MIPL协议）
Send_Cmd((uint8_t *)"AT+MIPLCREATE\r\n", 15, "OK");
HAL_Delay(100);
// 添加对象（3320：电池，3336：定位）
Send_Cmd((uint8_t *)"AT+MIPLADDOBJ=0,3320,1,\"1\",1,0\r\n", 32, "OK");
Send_Cmd((uint8_t *)"AT+MIPLADDOBJ=0,3336,1,\"1\",2,0\r\n", 32, "OK");
HAL_Delay(100);
// 连接平台（超时86400秒）
Send_Cmd((uint8_t *)"AT+MIPLOPEN=0,86400\r\n", 21, "OK");
```

**LwM2M 协议核心**：通过对象（Object）- 资源（Resource）模型实现数据交互，项目中 3320 对象的 5700 资源用于上报电池电量，3336 对象的 5513（纬度）、5514（经度）资源用于上报定位信息。

#### 2.1.4中断优先级设置

```c
// 所有外设初始化完成后设置中断优先级
HAL_NVIC_SetPriority(TIM2_IRQn, 2, 0);	  // 最低优先级（1s定时中断）
HAL_NVIC_SetPriority(LPUART1_IRQn, 0, 0); // 最高优先级（NB模块通信）
HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);  // 中等优先级（调试串口）

HAL_TIM_Base_Start_IT(&htim2); // 启动TIM2中断(1ms定时中断)
```

#### 	2.1.5 **while(1)主循环结构**

<!--第一个if：接收串口调试助手数据-->

```c
// 处理Res2缓冲区数据（转发到NB模块）
if (Res2_Sign == 1)
{
    // 等待接收完成（超时10ms）
    do
    {
        Res2++;
        HAL_Delay(1);
    } while (Res2 < 10);
    Res2_Sign = 0;
    // 将Res2接收的数据转发到NB模块（LPUART1）
    HAL_UART_Transmit(&hlpuart1, Res2_Buf, Res2_Count, 1000);
    Res2_Count = 0;
}
```

<!--第二个if：接收NB返回数据-->

```c
// 处理Res1缓冲区数据（NB模块接收，转发到调试串口并解析）
if (Res1_Sign == 1)
{
    // 等待接收完成（超时10ms）
    do
    {
        Res1++;
        HAL_Delay(1);
    } while (Res1 < 10);
    Res1_Sign = 0;
    // 将NB模块接收的数据转发到调试串口（USART2）
    HAL_UART_Transmit(&huart2, Res1_Buf, Res1_Count, 1000);
    Res1_Count = 0;
    NB_Rec_Handle(); // 解析NB模块数据（如定位信息、平台指令）
}
```

<!--第三个if：检查按键关机信号-->

```c
// 检查关机信号（低电平触发）
if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == 0)
{
HAL_GPIO_WritePin(PWR_LED_GPIO_Port, PWR_LED_Pin, GPIO_PIN_SET); // 熄灭电源灯
while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == 0)
;												  // 等待信号释放
HAL_GPIO_WritePin(GPIOA, PWR_EN_Pin, GPIO_PIN_RESET); // 关闭电源
}
```

<!--第四个if：1s中断发送信息-->

```c
// 定时中断标志（1秒触发一次）
if (interuptFlag)
{
    interuptFlag = 0; // 清除标志

    // 读取电池电压（10次采样平均滤波）
    Read_Filter_Battery_Voltage();

    // 计算电池百分比并处理低电量关机
    BAT_Q = Calculate_Battery_Percentage(BAT_Value);
    if (BAT_Q == 0)
    {
        shutdown++;
        if (shutdown > 2) // 连续3次检测到电量为0则关机
        {
            HAL_GPIO_WritePin(GPIOA, PWR_EN_Pin, GPIO_PIN_RESET);
        }
    }

    // 若已成功连接OneNet平台，发送电池数据和GNSS查询指令
    if (onenet_ok == 2)
    {
        // 填充电池百分比到NB_NOTIFY5700_CMD指令
        NB_NOTIFY5700_CMD[33 + NB_OB3320_count] = BAT_Q / 100 + 0x30;
        NB_NOTIFY5700_CMD[34 + NB_OB3320_count] = BAT_Q % 100 / 10 + 0x30;
        NB_NOTIFY5700_CMD[35 + NB_OB3320_count] = BAT_Q % 10 + 0x30;

        // 电量变化时才发送更新
        if (BAT_Q != BAT_Q_Last)
        {
            Send_Cmd(NB_NOTIFY5700_CMD, 42 + NB_OB3320_count, "OK");
            HAL_UART_Transmit(&huart2, NB_NOTIFY5700_CMD, 42 + NB_OB3320_count, 1000);
            BAT_Q_Last = BAT_Q; // 更新上次电量值
        }

        // 发送GNSS数据查询指令（获取NMEA/RMC格式定位信息）
        HAL_UART_Transmit(&hlpuart1, (uint8_t *)"AT+QGNSSRD=\"NMEA/RMC\"\r\n", 23, 1000);
    }
}
```

#### 2.1.6 **外设函数修改**

```c
// ADC修改：先校准，后读取芯片内部参考电压->为了获取精准的电池电压。最后启动ADC转换
static void MX_ADC_Init(void)
{
    /* USER CODE BEGIN ADC_Init 2 */
	ADC1->CR |= ADC_CR_ADCAL; // 启动ADC校准
	i = 0;
	// 等待ADC校准完成，最长等待10ms
	while (((ADC1->ISR & ADC_ISR_EOCAL) != ADC_ISR_EOCAL) && (i < 10))
	{
		i++;
		HAL_Delay(1);
	}
	if (i == 10) // 校准超时
	{
		printf("ADC initialization calibration failed!\r\n");
	}
	ADC1->ISR |= ADC_ISR_EOCAL;			   // 清除校准完成标志位
	VREFINT_CAL = *(uint16_t *)0x1FF80078; // 读取芯片内部参考电压校准值（存储在系统存储区）
	ADC1->CR |= ADC_CR_ADSTART;			   // 启动ADC转换
	/* USER CODE END ADC_Init 2 */
}
```

```c
// LPUART1修改：开启LPUART1
static void MX_LPUART1_UART_Init(void)
{
/* USER CODE BEGIN LPUART1_Init 2 */
	__HAL_UART_ENABLE_IT(&hlpuart1, UART_IT_RXNE); // 使能LPUART1接收中断（连接NB模块）
/* USER CODE END LPUART1_Init 2 */
}
```

```c
// USART2修改：开启USART2
static void MX_USART2_UART_Init(void)
{
    /* USER CODE BEGIN USART2_Init 2 */
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE); // 使能USART2接收中断（调试串口）
    /* USER CODE END USART2_Init 2 */
}
```

### 2.2stm32l0xx_it.c 文件

#### 2.2.1全局变量定义

```c
// 串口通信部分
uint8_t Res2_Buf[256];  // UART2接收数据缓冲区，最大存储256字节
uint8_t Res2_Sign = 0;  // UART2接收标志位：1-有新数据接收，0-无新数据
uint8_t Res2_Count = 0; // UART2接收数据计数，记录当前缓冲区已接收字节数
uint8_t Res2 = 0;       // UART2接收状态辅助变量

uint8_t Res1_Buf[256];  // LPUART1接收数据缓冲区，最大存储256字节
uint8_t Res1_Sign = 0;  // LPUART1接收标志位：1-有新数据接收，0-无新数据
uint8_t Res1_Count = 0; // LPUART1接收数据计数，记录当前缓冲区已接收字节数
uint8_t Res1 = 0;       // LPUART1接收状态辅助变量
```

#### 	**2.2.2中断服务函数**

```markdown
USART2_IRQHandler
	处理USART2接收中断
	从调试串口接收数据存入Res2_Buf
	设置接收标志Res2_Sign
```

```c
/* USER CODE BEGIN USART2_IRQn 0 */
// 检查UART2接收非空标志（RXNE），判断是否有新数据接收
if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE))
{
    // 缓冲区边界检查：确保不超过最大容量（255字节，预留1字节防止溢出）
    if (Res2_Count < 255)
    {
        // 从UART2数据寄存器(RDR)读取接收字节，存入缓冲区并递增计数
        Res2_Buf[Res2_Count++] = huart2.Instance->RDR;
        Res2_Sign = 1; // 设置接收标志位，通知应用层有新数据
        Res2 = 0;      // 重置接收状态辅助变量
    }
}
/* USER CODE END USART2_IRQn 0 */
```

```markdown
LPUART1_IRQHandler
	处理LPUART1接收中断
	从NB模块接收数据存入Res1_Buf
	设置接收标志Res1_Sign
```

```c
/* USER CODE BEGIN LPUART1_IRQn 0 */
// 检查LPUART1接收非空标志（RXNE），判断是否有新数据接收
if (__HAL_UART_GET_FLAG(&hlpuart1, UART_FLAG_RXNE))
{
    // 缓冲区边界检查：确保不超过最大容量（255字节，预留1字节防止溢出）
    if (Res1_Count < 255)
    {
        // 从LPUART1数据寄存器(RDR)读取接收字节，存入缓冲区并递增计数
        Res1_Buf[Res1_Count++] = hlpuart1.Instance->RDR;
        Res1_Sign = 1; // 设置接收标志位，通知应用层有新数据
        Res1 = 0;      // 重置接收状态辅助变量
    }
}
/* USER CODE END LPUART1_IRQn 0 */
```

### **2.3系统整体工作流程**

1. **启动阶段**：初始化硬件 → 启动NB模块 → 网络注册 → 云平台连接
2. **数据采集**：定期采集电池电压 → 获取GPS定位数据
3. **数据处理**：解析NB模块响应 → 处理云平台指令 → 格式化上报数据
4. **通信传输**：通过NB-IoT网络将电池电量和位置信息发送到OneNET云平台
5. **电源管理**：监测电池电量，低电量时自动关机

## 三、硬件设计与实现

### 3.1核心器件选型

| 器件型号      | 功能角色         | 选型依据                                                     |
| ------------- | ---------------- | ------------------------------------------------------------ |
| STM32L031F6Px | 主控 MCU         | 支持 LPUART（低功耗串口，匹配 BC20 的 1.8V 电平）、双串口满足调试 + 通信需求 |
| BC20          | NB-IoT/GNSS 模块 | 集成 NB 通信与 GPS 定位，支持 AT 指令控制，适合低功耗物联网场景 |
| TP4065        | 锂电池充电器     | 支持 4.2V 锂电池充电，具备充满自停、低漏电流特性（<1μA）     |
| DTC143ECA     | 数字晶体管       | 内置偏置电阻，简化 BC20 模块的电平转换与信号驱动电路设计     |

#### 3.1.1**STM32L031F6Px** 

```markdown
# BC20模块需要1.8V驱动，对于串口通信的电平要求为低功率；
	STM32L031F6Px芯片具备低功率串口，LPUART是STM32低功耗系列的典型外设，专为低功耗场景设计，支持在低功耗模式下保持串口通信，通过减少时钟频率或优化唤醒机制降低功耗，适合电池供电设备。
# 通信串口数量需要2个及以上
	STM32L031F6Px芯片刚好具备两个串口，其中一个为低功率串口，另一个为常规串口，满足MCU与NB-IOT模块通信以及与串口调试助手调试的需求
```

#### 3.1.2BC20模组

```markdown
	本项目硬件核心是 STM32 主控 + NB-IoT 模块的搭配，重点是接口兼容性、供电稳定性、天线设计，NB-IOT模块，拥有低功耗串口，可与MCU通信，拥有GNSS/GPS天线，可连接NB基站和定位，可连接 NB-Iot SIM卡,连接云平台,若嵌入式设备需电池供电、远距离部署、小数据传输,NB-IoT 是最优选择。
# 关键相关组件说明
	NB-IoT 模块：嵌入式设备的通信核心，内置 NB-IoT 射频芯片、TCP/IP 栈，通过 UART 与 STM32 通信（AT 指令控制）。
	NB-IoT 基站：接收模块信号，转发数据到核心网，支持广覆盖和大连接。
	SIM 卡：NB-IoT 专用 Micro-SIM/Nano-SIM，存储设备身份信息（IMSI）、流量套餐，需开通运营商 NB 业务。
```

### 3.2关键电路设计

#### 3.2.1电源管理电路

- **供电选择电路**：支持 USB（5V）与锂电池（4.2V）双输入，通过二极管实现防反接与自动切换

- **一键开关机电路**：按键按下实现开机和关机

![](D:\项目复盘合集\1-物联网NB-IoT定位器\attachments\hardware\供电选择+一键开关机电路.jpg)

- **降压电路**：5V/4.2V → 3.3V（给 MCU 供电），采用 LDO 稳压芯片保证供电稳定性

![](D:\项目复盘合集\1-物联网NB-IoT定位器\attachments\hardware\降压电路.jpg)

- **滤波电路**：在电源输入端与芯片供电引脚处增加 10μF+0.1μF 电容组合，抑制电源噪声

![](D:\项目复盘合集\1-物联网NB-IoT定位器\attachments\hardware\电源滤波电路.jpg)

- **锂电池充电电路**：给 4.2V 锂电池充电，具备充电灯亮，充满自停特性

![](D:\项目复盘合集\1-物联网NB-IoT定位器\attachments\hardware\锂电池充电电路.jpg)

#### 3.2.2通信接口电路

- **电平转换电路**：实现 STM32（3.3V）与 BC20（1.8V）的串口信号匹配，通过 DTC143ECA 晶体管实现双向电平转换

  ![](D:\项目复盘合集\1-物联网NB-IoT定位器\attachments\hardware\电平转换+信号驱动电路.jpg)

- **USB转串口电路**：USB信号转为TTL电平信号

  ![](D:\项目复盘合集\1-物联网NB-IoT定位器\attachments\hardware\USB转串口电路.jpg)

- **NB-IoT 天线接口**：预留 IPEX 座子，匹配 BC20 模块的射频特性（需远离高速信号线减少干扰）

  ![](D:\项目复盘合集\1-物联网NB-IoT定位器\attachments\hardware\BC20周围电路.jpg)

- **BC20与SIM卡连接电路**：通过电阻限流保护信号传输，抑制浪涌电压，滤除高频噪声

![](D:\项目复盘合集\1-物联网NB-IoT定位器\attachments\hardware\SIM卡连接电路.jpg)

#### 3.2.3电压检测电路

- **电池电压采样**：通过电阻分压网络将电池电压（3.0V—4.2V）降至 ADC 量程内（0—3.3V），接入 STM32 的 ADC 通道 4

![](D:\项目复盘合集\1-物联网NB-IoT定位器\attachments\hardware\端口电压检测电路.jpg)

### 3.3 硬件调试问题与解决方案

| 问题现象                     | 根因分析                                | 解决方案                                                     |
| ---------------------------- | --------------------------------------- | ------------------------------------------------------------ |
| 程序下载后电源指示灯不亮     | MCU 的 PC15 引脚焊接不良                | 重新焊接 MCU，焊后用万用表检测引脚连通性                     |
| 上电后无 3.3V 输出           | PWR_EN 引脚未初始化置高，MCU 未正常工作 | 更换 MCU，加强焊接质量管控（焊后测短路 / 连通）              |
| NB注册网络失败（+CEREG:0,3） | 手机SIM 卡未开通 NB 业务                | 更换为 NB-IOT专用 SIM 卡，测试 AT+CEREG? 注册命令，正确返回 +CEREG: 0,1 |

```markdown
# 问题1细节
问题标注：电源指示灯在4.7KΩ限流电阻的方向的引脚电平测得为0
分析：已排除灯本身问题，3.3V电压正常，怀疑MCU的PC15引脚出现问题，可能由焊接不良导致
# 问题2细节
问题标注：PWR_EN引脚在上电后未被初始化置高电平，导致后续的VCC/3.3V皆为0V
分析：怀疑MCU出问题，同上，可能由焊接不良导致(也可能买到有问题的MCU)
## 综上：初步判断由于焊接/买到假货导致MCU出问题，无法为后续电路供电，需重新换个MCU
## 经验积累：焊接完电路板需要检查电气特性(万用表)
# 问题3细节
不断发送 AT+CEREG? 指令,一直返回的是+CEREG: 0,3（注册被拒绝），导致循环无法退出
可能原因：SIM 卡问题：SIM 卡未开通 NB-IoT 业务（需确认 SIM 卡支持 NB 网络）。
解决方法：检查 SIM 卡：确认 SIM 卡有效、已开通 NB 业务、未欠费，且物理接触良好。
```

## 四、软件设计与实现

### 4.1CubeMX配置

#### 4.1.1GPIO

```markdown
# Input
- 内部上/下拉配置：
	这取决于正常工作时的电平，若需要该引脚检测外部低电平，则配置Pull-up(空闲时引脚电平由内部拉高)，反之亦然；
# Output
- 输出电平配置：
	同样取决于正常工作时电平，若正常工作时需要输出高电平，则初始配置为Low，反之亦然，也有情况是需要初始配置与工作时一致的电平；
- 输出模式配置：
	推挽or开漏，需要强驱动、同电压域、独立输出时选推挽；需要电平匹配、线与总线、双向通信时选开漏；
```

| 选择因素 | 推挽输出的适用场景               | 开漏输出的适用场景              |
| :------: | :------------------------------- | :------------------------------ |
| 驱动能力 | 需强驱动负载（如 LED、小型负载） | 弱驱动、依赖上拉电阻的负载      |
| 电平匹配 | 同电压域通信                     | 跨电压域通信（如 3.3V→5V）      |
| 通信协议 | 无需线与的协议（如 UART）        | 需线与的协议（如 I2C、OneWire） |
| 双向通信 | 一般不用                         | 需双向通信的引脚（如 I2C 引脚） |

```markdown
- 内部上/下拉配置：
1. 推挽输出：内部上下拉对输出结果几乎无影响
	由于主动驱动电平的特性，内部上下拉电阻的作用会被 MOS 管的驱动完全覆盖，因此通常配置为 “No pull-up and no pull-down”（避免不必要的功耗或电平冲突）；
2. 开漏输出：内部上下拉对输出结果有直接影响
	输出低电平时：下拉 MOS 管导通，将引脚拉到VSS；输出 “高电平” 时：下拉 MOS 管关闭，引脚电平完全由外部 / 内部上下拉决定。
	此时内部上下拉的作用是：
	配置 “内部上拉”：下拉 MOS 管关闭时，引脚会被内部上拉拉到VDD（实现 “开漏输出高电平”）；
	配置 “内部下拉”：下拉 MOS 管关闭时，引脚会被内部下拉到VSS；
	配置 “无上下拉”：下拉 MOS 管关闭时，引脚是高阻态（电平不确定，需依赖外部电路）。
```

#### 4.1.2USART/LPUART

| 模式名称                                          | 核心特点                                                     | 引脚需求               | 典型场景                                    |
| ------------------------------------------------- | ------------------------------------------------------------ | ---------------------- | ------------------------------------------- |
| Asynchronous（异步模式）                          | 无需时钟线，波特率同步，支持全双工；通信灵活                 | TX + RX（2 根）        | 串口调试、MCU 与上位机数据传输              |
| Synchronous（同步模式）                           | 需 SCLK 时钟线，以时钟节拍同步数据，速率更高（无波特率误差） | TX + RX + SCLK（3 根） | 对速率 / 同步精度要求高的设备间通信         |
| Single Wire (Half-Duplex)（单线半双工模式）       | 单引脚复用 TX/RX，同一时间仅能收 / 发，节省 GPIO 资源        | 1 根复用线             | 引脚资源紧张的低功耗设备通信                |
| Multiprocessor Communication（多处理器通信模式）  | 区分 “地址帧 / 数据帧”，支持多设备共享同一串口总线，避免冲突 | TX + RX（2 根）        | 多个 MCU / 外设通过串口总线组网             |
| IrDA（红外通信模式）                              | 适配 IrDA 协议，需搭配红外收发器，数据调制为 38kHz 载波格式  | TX + RX（红外适配）    | 短距离红外数据传输（如早期手机红外通信）    |
| LIN（本地互联网络模式）                           | 适配车载 LIN 协议，符合 LIN 帧格式 / 校验规则，适配低成本车载设备 | TX + RX（2 根）        | 汽车电子低速网络通信（车窗、座椅控制器）    |
| SmartCard（智能卡模式）                           | 适配 ISO 7816 协议，支持智能卡的反向电平、低通信速率特性     | TX + RX（智能卡适配）  | MCU 与 SIM 卡、金融 IC 卡的通信             |
| SmartCard with Card Clock（带卡时钟的智能卡模式） | 在 SmartCard 模式基础上，额外输出智能卡所需的外部时钟，无需额外硬件 | TX + RX + 时钟线       | 需外部时钟驱动的智能卡通信                  |
| Modbus Communication（Modbus 模式）               | 适配 Modbus RTU/ASCII 协议，自动处理 Modbus 帧格式、CRC 校验规则 | TX + RX（2 根）        | 工业设备组网（PLC、传感器间的 Modbus 通信） |

```markdown
#
# NVIC
开启串口中断允许标志位
# 参数设置
若选择异步通信，则配置串口通信参数(波特率、位数等)
```

#### 4.1.3ADC

```markdown
# Mode
	选择ADC转换通道：IN4（电池电压），和通道VREFINT（内部参考电压）
# Parameter Settings
```

| 参数项                               | 截图配置值                              | 含义                                                         |
| ------------------------------------ | --------------------------------------- | ------------------------------------------------------------ |
| `Clock Prescaler`                    | Synchronous clock mode divided by 2     | ADC 时钟分频：同步时钟模式下除以 2（控制 ADC 工作时钟，避免超过芯片最大 ADC 时钟限制）。 |
| `Resolution`                         | ADC 12-bit resolution                   | ADC 分辨率：12 位（量化范围 0~4095，分辨率越高、电压精度越高）。 |
| `Data Alignment`                     | Right alignment                         | 数据对齐方式：右对齐（ADC 转换结果存到寄存器的**低 12 位**，方便后续数值计算）。 |
| `Scan Direction`                     | Forward                                 | 多通道扫描方向：正向（按通道编号从小到大扫描，如 IN4→IN5→IN6）。 |
| `Continuous Conversion Mode`         | Enabled                                 | 连续转换模式：使能（ADC 完成一次转换后，自动启动下一次转换，持续采集）。 |
| `Discontinuous Conversion Mode`      | Disabled                                | 间断转换模式：禁用（多通道扫描时，不会拆分通道组、单次触发只转换部分通道）。 |
| `DMA Continuous Requests`            | Disabled                                | DMA 连续请求：禁用（若开启，ADC 转换完成后会持续向 DMA 发请求，批量传输数据到内存）。 |
| `End Of Conversion Selection`        | End of single conversion                | 转换结束标志：单次转换结束（多通道时可选择 “序列转换结束”，即所有通道转换完才触发标志）。 |
| `Low Power Auto Wait`                | Enabled                                 | 低功耗自动等待：使能（ADC 在转换间隙进入等待状态，降低功耗，适合低功耗场景）。 |
| `Auto Off`                           | Enabled                                 | 自动关闭：使能（ADC 转换完成后自动断电，下次触发时再唤醒，进一步降低功耗）。 |
| `Sampling Time`                      | 160.5 Cycles                            | 采样时间：160.5 个 ADC 时钟周期（采样时间越长，模拟信号采集越稳定，但转换速度越慢）。 |
| `External Trigger Conversion Source` | Regular Conversion launched by software | 转换触发源：软件触发（通过`HAL_ADC_Start()`函数启动转换，也可选择外部引脚 / 定时器触发）。 |
| `External Trigger Conversion Edge`   | None                                    | 外部触发边沿：无（因为是软件触发，无需边沿触发条件）。       |

### 4.2设计函数赏析

<!--串口发送重定向函数：在printf被调用之前,fputc必须已经被定义,需要在main函数执行前被编译和定义-->

```c
// 重定向fputc函数，使printf通过USART2输出（调试用）
int fputc(int ch, FILE *f)
{
	while ((USART2->ISR & 0X40) == 0)
		;					   // 等待发送缓冲区为空
	USART2->TDR = (uint8_t)ch; // 发送字符
	return ch;
}
```

<!--重定义TIM2回调函数-->

```c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == htim2.Instance) // 确认是TIM2中断
	{
		static uint16_t time = 0; // 累计中断次数（1ms/次）
		time++;
		if (time >= 1000) // 累计1000次即1秒
		{
			interuptFlag = 1; // 置1秒定时标志
			time = 0;		  // 重置计数
		}
	}
}
```

<!--获取两个ADC通道数据并计算-->

```c
void Get_BAT_Value(void)
{
	uint8_t i;

	// 等待ADC通道4（电池电压）转换完成（超时10ms）
	i = 0;
	while (((ADC1->ISR & ADC_ISR_EOC) != ADC_ISR_EOC) && (i < 10))
	{
		i++;
		HAL_Delay(1);
	}
	if (i == 10)
	{
		printf("通道4转换失败\r\n");
	}
	BAT_DATA = ADC1->DR; // 读取电池ADC原始值

	// 等待ADC通道VREFINT（内部参考电压）转换完成（超时10ms）
	i = 0;
	while (((ADC1->ISR & ADC_ISR_EOC) != ADC_ISR_EOC) && (i < 10))
	{
		i++;
		HAL_Delay(1);
	}
	if (i == 10)
	{
		printf("通道17转换失败\r\n");
	}
	VREFINT_DATA = ADC1->DR; // 读取内部参考电压ADC原始值

	// 计算实际参考电压（VDDA）：3.0V * 校准值 / 实际采样值
	VDDA_Value = 3.0 * VREFINT_CAL / VREFINT_DATA;
	// 计算电池电压：参考电压 * 电池ADC值 / 2048（12位ADC，量程0-VDDA，分压后需根据硬件调整）
	BAT_Value = VDDA_Value * BAT_DATA / 2048;
}
```

<!--电压10次均值滤波-->

```c
// 读取并进行10次采样平均滤波处理电池电压
void Read_Filter_Battery_Voltage(void)
{
    uint8_t i;
    float temp = 0;
    
    // 10次采样平均滤波
    for (i = 0; i < 10; i++)
    {
        Get_BAT_Value();
        temp += BAT_Value;
    }
    BAT_Value = temp / 10;
}
```

<!--计算电池电压百分比-->

```c
uint8_t Calculate_Battery_Percentage(float voltage)
{
	if (voltage > 4.1)
		return 100; // 4.1V以上为满电
	if (voltage < 3.0)
		return 0; // 3.0V以下为没电

	// 3.0V~4.1V之间线性计算百分比
	return (uint8_t)(100 - (4.1 - voltage) * 100 / 1.1);
}
```

<!--AT命令发送函数-->

```c
uint8_t Send_Cmd(uint8_t *cmd, uint8_t len, char *recdata)
{
    uint8_t ret;               // 函数返回值，用于标识处理结果
    uint16_t count = 0;        // 超时计数变量，用于控制等待响应的时长

    // 清空接收缓冲区（Res1_Buf为全局接收缓存，此处假设其内容以null结尾）
    memset(Res1_Buf, 0, strlen((const char *)Res1_Buf));
    // 通过LPUART1向NB模块发送命令，超时时间1000ms
    HAL_UART_Transmit(&hlpuart1, cmd, len, 1000);		

    // 等待NB模块响应，最长等待300ms（循环300次，每次延迟1ms）
    // Res1_Sign为全局标志位，由接收中断置位表示有数据到达
    while ((Res1_Sign == 0) && (count < 300))
    {
        count++;
        HAL_Delay(1);  // 1ms延迟，降低CPU占用
    }

    if (count == 300)  // 超时未收到响应（Res1_Sign始终为0）
    {
        ret = 1;
    }
    else  // 收到响应（Res1_Sign已置位）
    {
        // 等待接收完成（额外等待10ms，确保模块完整返回数据）
        // Res1为局部计数变量，用于控制等待时长
        do
        {
            Res1++;
            HAL_Delay(1);
        } while (Res1 < 10);

        // 将接收到的响应数据通过UART2（调试串口）转发，便于调试查看
        HAL_UART_Transmit(&huart2, Res1_Buf, Res1_Count, 1000);
        // 重置接收相关全局变量，为下一次接收做准备
        Res1_Count = 0;  // Res1_Count：接收数据长度计数
        Res1_Sign = 0;   // 重置响应标志位
        ret = 2;         // 先默认标记为"收到响应但不匹配"

        // 检查接收缓冲区中是否包含预期字符串recdata
        if (strstr((const char *)Res1_Buf, recdata))
        {
            ret = 0;  // 包含预期字符串，更新为成功状态
        }
    }

    return ret;
}
```

<!--处理NB模块接收的数据：伪代码展示-->

```c
// NB模块接收数据处理函数
NB_Rec_Handle():
    定义临时循环/计算变量i、j、temp_int、temp_float

    // 1. 处理OneNET平台观察者请求（+MIPLOBSERVE:）
    若接收缓冲区包含"+MIPLOBSERVE:"：
        校验指令格式（指定位置为逗号）：
            提取指令中参数长度（最多10位，防溢出）
            若参数长度非法，打印错误信息
            否则：
                若指令包含"3320"（电池对象）：
                    拷贝参数到观察者响应指令和电池上报指令
                    拼接电池上报指令（对象3320、资源5700等信息）
                    记录3320对象参数长度
                若指令包含"3336"（定位对象）：
                    拷贝参数到观察者响应指令、纬度/经度上报指令
                    拼接纬度/经度上报指令（对象3336、资源5513/5514等信息）
                    记录3336对象参数长度
                拼接观察者响应指令并发送

    // 2. 处理OneNET平台发现请求（+MIPLDISCOVER:）
    否则若接收缓冲区包含"+MIPLDISCOVER:"：
        校验指令格式（指定位置为逗号）：
            提取指令中参数长度（最多10位，防溢出）
            若参数长度非法，打印错误信息
            否则：
                拷贝参数到发现响应指令
                若指令包含"3336"（定位对象）：
                    拼接响应指令（包含资源5513/5514）并发送
                    更新OneNET连接状态（定位对象就绪）
                若指令包含"3320"（电池对象）：
                    拼接响应指令（包含资源5700）并发送
                    更新OneNET连接状态（电池对象就绪）

    // 3. 处理GNSS定位数据（+QGNSSRD:）
    若接收缓冲区包含"+QGNSSRD:"且定位有效（NMEA协议标记'A'）：
        // 解析纬度（ddmm.mmmm转dd.dddddd°，放大1e6）
        提取纬度度分数据 → 转换为十进制度 → 放大为整数
        填充纬度上报指令字符串
        // 解析经度（dddmm.mmmm转ddd.dddddd°，放大1e6）
        提取经度度分数据 → 转换为十进制度 → 放大为整数
        填充经度上报指令字符串
        // 上报经纬度到OneNET，同时调试打印指令
        发送纬度上报指令 + 调试打印
        发送经度上报指令 + 调试打印
```

### 4.3代码优化

#### 4.3.1TIM定时器**1s**中断上报数据

```markdown
	在STM32L031中采用TIM2定时中断实现1s定时，通过配置APB1总线8MHz时钟源、PSC=7和ARR=999可实现精准1ms定时，其无需占用GPIO引脚，采用“中断置标志位+主函数轮询”的成熟架构，与现有代码兼容且新增配置量小、无侵入性，能高效完成定时任务且不阻塞主程序。
```

​	**优化前代码：近似的逻辑模拟**

```c
/**
 * @brief  代码中 “main_count 每 1ms 加 1” 是一种近似的逻辑假设，实际循环体的执行时间受动态操作（如UART传输、模块交互）影响，不可能稳定消耗 72000 个时钟周期，因此 “每次循环 1ms” 的计时精度较低
 */
int main(void)
{
    uint16_t main_count = 0; // 主循环计数（用于定时任务）
    while(1)
    {
		main_count++;	// 定时任务（约1秒一次，main_count每1ms加1，1000即1秒）
        HAL_Delay(0); 	// 让出CPU时间
        if (main_count > 1000)
		{
            main_count = 0;
            // 下面为处理1s定时待完成的任务
        }
    }
}
```

​	**优化后代码： 定时器精确定时**

```c
（1）定时器初始化配置
/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{
  // ...
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8-1; // 预分频器，时钟分频后为1MHz（系统时钟8MHz）
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1; // 计数周期，1000次计数为1ms
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  // ...
}

（2）定时器中断启动与优先级配置
/* USER CODE BEGIN 2 */
// 所有外设初始化完成后设置中断优先级
HAL_NVIC_SetPriority(TIM2_IRQn, 2, 0);      // 最低优先级
HAL_NVIC_SetPriority(LPUART1_IRQn, 0, 0);   // 最高优先级（NB模块通信）
HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);    // 中等优先级（调试串口）

HAL_TIM_Base_Start_IT(&htim2);	// 启动TIM2中断(1ms定时中断)
/* USER CODE END 2 */

（3）定时器中断回调函数（实现1s计数）
// 中断回调函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == htim2.Instance)	// 定时器TIM2发生了中断
  {
    static uint16_t time = 0;
    time++;
    if(time >= 1000)  // 累计1000次1ms中断，即1s
    {
      // 1s中断，置标志位
      interuptFlag = 1;
      time = 0;
    }
  }
}

（4）main循环中处理定时任务
while (1)
{
  // ...（省略其他逻辑）
  if(interuptFlag)  // 检测1s定时标志	interuptFlag为全局变量
  {
    interuptFlag = 0;
    // ...（省略其他逻辑）
  }
}
```

## 五、调试日志

### 5.1 硬件电路板调试

#### 5.1.1 基础电气特性测试

​	在硬件焊接完成后，首先进行基础电气测试：

- **短路测试**：测量VUSB/VBAT/VCC/3.3V与GND之间电阻，确认无短路现象
- **通路测试**：使用万用表验证MCU和BC20模组所有引脚的连通性
- **电压测试**：
  - VUSB：正常测得接近5V
  - VBAT：正常测得接近4.2V
  - VCC/3.3V：按键按下后输出正常
- **接地测试**：确认所有GND正常共地

#### 5.1.2 功能指示灯验证

- **电源指示灯**：程序下载后，按下按键电源指示灯正常点亮；成功连接云平台后再次按下按键，指示灯熄灭
- **充电指示灯**：
  - 未充电状态：指示灯持续闪烁
  - 充电状态：指示灯保持常亮
  - 充电完成：指示灯熄灭

### 5.2 串口通信调试

#### 5.2.1 双串口通信验证

​	建立MCU与外部设备的串口通信链路：

- **USART2调试串口**：连接PC端串口调试助手，实现调试信息输出和指令接收
- **LPUART1通信串口**：连接BC20 NB-IoT模块，实现AT指令发送和响应接收

#### 5.2.2 通信流程验证

​	通过串口调试助手向MCU发送AT指令，验证完整通信链路：

```markdown
PC(串口调试助手) → USART2 → MCU → LPUART1 → BC20模块
BC20模块 → LPUART1 → MCU → USART2 → PC(串口调试助手)
```

### 5.3 ADC电压采集调试

#### 5.3.1 多通道ADC数据采集

​	实现精确的电池电压测量：

- **ADC通道4**：采集电池电压分压信号(BAT_DATA)
- **ADC通道VREFINT**：采集内部参考电压(VREFINT_DATA)

#### 5.3.2 电压计算算法

```c
// 计算实际VDDA参考电压
VDDA_Value = 3.0 * VREFINT_CAL / VREFINT_DATA;

// 计算电池实际电压
BAT_Value = VDDA_Value * BAT_DATA / 2048;
```

#### 5.3.3 电量估算与保护机制

- **采样优化**：10次采样取平均值，提高测量精度
- **电量计算**：根据3.0V~4.2V电压范围线性估算电量百分比
- **低功耗设计：**只有电池电量变化时才上报电量
- **低电保护**：设置三次确认机制，防止误触发关机

### 5.4 NB-IoT模块基础功能调试

#### 5.4.1 AT指令手动测试

​	通过串口调试助手直接向BC20模块发送基础AT指令：

- `AT` - 模块通信测试
- `AT+CSQ` - 信号强度查询
- `AT+QGNSSRD` - GPS定位数据获取

#### 5.4.2 网络注册验证

​	检查SIM卡和网络状态：

- **SIM卡验证**：确认使用NB-IoT专用物联网卡，已开通NB业务
- **网络注册**：通过`AT+CEREG?`指令确认返回`+CEREG: 0,1`
- **PS附着**：通过`AT+CGATT?`指令确认返回`+CGATT: 1`

### 5.5 云平台接入调试

#### 5.5.1 OneNET平台设备配置

- **设备创建**：在OneNET平台创建LwM2M设备(设备名称：Qinn_Locator)
- **协议配置**：选择LwM2M协议，记录设备ID和密钥

![](D:\项目复盘合集\1-物联网NB-IoT定位器\attachments\debug_logs\产品开发界面.jpg)

![](D:\项目复盘合集\1-物联网NB-IoT定位器\attachments\debug_logs\设备管理界面.jpg)

#### 5.5.2 LwM2M协议对接

```markdown
	LwM2M协议,专为资源受限设备（如STM32单片机）和低带宽、高延迟网络（如 NB-IoT、LoRa、GPRS）设计,核心目标是实现物联网设备的远程管理、数据上报、指令下发等功能,是嵌入式物联网接入的主流标准之一.
	若设备为 STM32 等嵌入式单片机，且使用 NB-IoT/LoRa 网络，LwM2M 是最优选择.
```

​	通过AT指令完成平台注册：

```c
AT+MIPLCREATE        // 创建LwM2M实例
AT+MIPLADDOBJ=0,3320,1,"1",1,0  // 添加电池对象
AT+MIPLADDOBJ=0,3336,1,"1",2,0  // 添加定位对象  
AT+MIPLOPEN=0,86400  // 连接平台
```

#### 5.5.3 数据上报验证

- **设备激活**：设备首次成功上报数据后，平台状态变为"在线"

  ![](D:\项目复盘合集\1-物联网NB-IoT定位器\attachments\debug_logs\设备详情界面.jpg)

- **数据源配置**：正确设置accessKey和userId参数[^1]

- **数据流验证**：确认电池电量和定位数据正常上报

  ![](D:\项目复盘合集\1-物联网NB-IoT定位器\attachments\debug_logs\设备资源界面.jpg)

### 5.6 软件功能模块调试

#### 5.6.1 AT命令发送函数设计

​	封装AT指令交互逻辑：

- **缓冲区管理**：发送前清空接收缓冲区，避免数据混杂
- **超时机制**：设置300ms等待超时，防止程序卡死
- **响应验证**：检查返回数据是否包含预期关键字
- **状态返回**：定义三种返回状态(成功、超时、响应不匹配)

#### 5.6.2 NB模组初始化程序设计

​	按照BC20手册流程实现模块初始化：

1. 模块自检(AT)
2. 关闭回显(ATE0)
3. 信号强度检测(AT+CSQ)
4. 网络注册等待(AT+CEREG?)
5. PS附着等待(AT+CGATT?)
6. GNSS功能开启(AT+QGNSSC=1)
7. 云平台实例创建和注册

#### 5.6.3 NB模块返回信息解析函数设计

- **数据转发**：将NB模块响应数据转发至调试串口
- **指令解析**：识别和处理平台下发的各种指令
- **定位数据提取**：从NMEA格式数据中解析经纬度信息

### 5.7 系统集成测试

#### 5.7.1 端到端功能验证

- **定位功能**：连接GNSS天线，验证定位数据获取和解析
- **通信功能**：确认数据通过NB-IoT网络正常上报至云平台
- **电源管理**：验证电池电量监测和低电保护功能
- **用户交互**：测试开关机按键和状态指示灯

#### 5.7.2 性能优化验证

- **功耗测试**：测量各工作模式下的电流消耗
- **稳定性测试**：长时间运行验证系统稳定性
- **网络适应性**：在不同信号强度环境下测试通信可靠性

#### 5.7.3数据可视化应用开发

​	新建一个项目

![](D:\项目复盘合集\1-物联网NB-IoT定位器\attachments\debug_logs\应用APP项目.jpg)

![](D:\项目复盘合集\1-物联网NB-IoT定位器\attachments\debug_logs\项目概括.jpg)

​	并把该项目关联前面创建的设备

![](D:\项目复盘合集\1-物联网NB-IoT定位器\attachments\debug_logs\设备管理.jpg)

​	新建可视化项目

![](D:\项目复盘合集\1-物联网NB-IoT定位器\attachments\debug_logs\可视化项目.jpg)

​	GUI设计（移动端）

**采取WGS84 转 GCJ-02**

- **WGS84**：是全球定位系统（GPS）使用的标准坐标系（地球坐标系），是国际通用的原始定位坐标。
- **GCJ-02**：也叫 “火星坐标系”，是中国对 WGS84 进行非线性加密后的坐标系，是国内地图（如高德、腾讯地图）必须使用的合法坐标系。
- 采用百度坐标则需要授权

![](D:\项目复盘合集\1-物联网NB-IoT定位器\attachments\debug_logs\编辑界面.jpg)

​	创建数据源模板（重点）

![](D:\项目复盘合集\1-物联网NB-IoT定位器\attachments\debug_logs\建立数据源模板.jpg)

​	[1] 正确设置accessKey和userId参数（userld为限时专业版的，7天后再设置为基础版）

![](D:\项目复盘合集\1-物联网NB-IoT定位器\attachments\debug_logs\参数设置.jpg)

![](D:\项目复盘合集\1-物联网NB-IoT定位器\attachments\debug_logs\参数设置Per.jpg)

​	创建完数据可视化界面后，需要发布，后会生成链接，点击链接即可进入GUI界面

![](D:\项目复盘合集\1-物联网NB-IoT定位器\attachments\debug_logs\GUI项目发布.jpg)

#### 5.7开机状态下续航能力

```markdown
# 线性估算结果-20:39分准时开始，剩余电量100%，22:28分准时关闭，剩余电量86%
计算逻辑（基于均匀放电假设）：
	已消耗电量占比：100% - 86% = 14%
	单位电量放电时长：总耗时 ÷ 已消耗电量 = 109 分钟 ÷ 14% ≈ 7.79 分钟 / 每 1% 电量
	总放电时间（100%→0%）：100% × 7.79 分钟 /% ≈ 779 分钟（约 12 小时 59 分钟）
核心前提（实际场景中需验证）：
	电池放电速率完全线性（锂电池实际为非线性，但线性值是后续修正的基准）；
	设备工作模式（定位频率、NB-IoT 通信时长、休眠占比）无变化；
	环境温度、供电电压等影响功耗的因素稳定。
```

## 六、总结与经验

**硬件开发：**
	焊接后必须逐点检测（短路、连通性、电压），避免因虚焊 / 错焊导致的隐性问题

​	NB 模块供电需稳定（纹波 < 100mV），射频部分远离数字电路减少干扰

**软件开发：**
	串口中断接收需注意缓冲区溢出（建议增加计数上限判断）

​	AT 指令交互必须加超时处理，避免模块无响应时程序卡死

**协议与通信：**
	LwM2M 协议的对象 / 资源模型需严格匹配平台要求（参考 OneNET 设备手册）

​	NB 网络注册失败时，优先排查 SIM 卡（是否开通业务）、频段（匹配运营商）、信号强度

​	SIM卡要开通NB业务，此时的SIM卡也即为物联网卡，一般于公司批量订购（也可以淘宝购买）

**云平台应用设计：**

​	数据源选择时的参数设置，重点在于accessKey和userId填写正确（解决参考[CSDN博客](https://blog.csdn.net/freestrong_4G/article/details/145856618?fromshare=blogdetail&sharetype=blogdetail&sharerId=145856618&sharerefer=PC&sharesource=weixin_71956626&sharefrom=from_link)）

## 七、附件索引

硬件原理图：[NBIoT定位器-基于BC20模组原理图](D:\项目复盘合集\1-物联网NB-IoT定位器\attachments\hardware\NBIoT定位器-基于BC20模组原理图.pdf)

CubeMX配置：[DWQ.ioc](D:\项目复盘合集\1-物联网NB-IoT定位器\attachments\firmware\DWQ.ioc)

完整的main文件代码：[main.c](D:\项目复盘合集\1-物联网NB-IoT定位器\attachments\code_snippets\main.c)

完整的中断处理文件代码：[stm32l0xx_it.c](D:\项目复盘合集\1-物联网NB-IoT定位器\attachments\code_snippets\stm32l0xx_it.c)
