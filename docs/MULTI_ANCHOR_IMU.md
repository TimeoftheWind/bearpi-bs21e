# 五锚点测距 + 单锚点 IMU：实验开发版

本分支 `feat/multi-anchor-imu-telemetry` 基于原稳定分支
`feat/realtime-pdr-telemetry` 的 `72ffd35`，原分支与原工作目录不修改。
新工作目录：`/Users/aukira/bearpi/bearpi-multi-anchor-imu`。

## 固定配置

| 设备 | SLE 地址 | 测距目标 | IMU |
| --- | --- | --- | --- |
| Client（带 IMU） | 02 02 02 02 02 02 | 与每个 Server 各 1 Hz | 20 Hz 采样，4 样本/批，目标 5 批/秒 |
| server5 | 01 01 01 01 01 05 | 1 Hz，计算并输出本端 DIST | 唯一 IMU 接收端，输出全部收到的样本 |
| server6 | 01 01 01 01 01 06 | 1 Hz，计算并输出本端 DIST | 不接收 |
| server7 | 01 01 01 01 01 07 | 1 Hz，计算并输出本端 DIST | 不接收 |
| server8 | 01 01 01 01 01 08 | 1 Hz，计算并输出本端 DIST | 不接收 |
| server9 | 01 01 01 01 01 09 | 1 Hz，计算并输出本端 DIST | 不接收 |

“server5”指地址最后一个字节为 05，不是连接编号 5。协议栈分配的 conn_id 可以不连续。
当前 Client 白名单固定 05–09；周围地址 01 的旧固件不会被连接。
集中配置文件为 `application/samples/products/sle_measure_dis/multi_anchor_config.h`。
`con_anchor_num=5`、`cs_interval=2` 对应已有示例的 1 Hz 档位，不能把数值 2 当成 2 Hz。

## 实现与边界

- 每条链路分别记录服务句柄、MTU、IQ 组装缓存和连接代次。断线前的 IQ 不会发到复用的连接编号。
- 连接初始化及所有 Client SSAP 发送在应用任务执行。IQ 回调只做有限拷贝和非阻塞入队，
  不在 `bt_service` 里等待、分配发送缓冲或直接调用发送。这降低原高负载栈错误的风险，
  但不能替代真实硬件的栈水位验证。
- IQ 优先，队列 15 条；900 ms 内未发成功或连接失效则丢弃并输出 `IQ_DROP`/统计。
  发送成功之间至少留 10 ms。IMU 不广播，仅定向 server5。
- IMU 每批 4 样本，server5 输出完该批 CSV 后回 ACK；Client 保留未确认批次，
  500 ms 未确认则重发。server5 对重复批次不重复输出，只重发 ACK。
- ACK 代表 Server 已调用日志输出，**不代表电脑串口文件已持久化**。请独立保存串口日志。
- IMU 环形缓冲 128 样本，约 6.4 秒；未确认批次最长保留 6 秒。
  长时间断线、输出阻塞或持续发送失败不能保证无损；溢出/超时明确报告 `IMU_DROP`。
  正常链路下完整样本流是目标，验收以样本序号连续性为准。
- 5 批/秒是健康链路下的发送目标，不是已测得的板上性能；重试会增加发送次数，
  ACK 延迟可能降低有效批次频率。测距无效值仍可能为负数。
- 五个 Server 分别保留自己的 DIST 日志，没有把五路结果汇总到 server5。
  Client 与各 Server 的微秒时钟独立；不能直接把跨设备时间戳当成已经同步。

## 在另一台电脑编译烧录

本 Git 工作目录缺少完整 SDK 的 `build/config`、`build/script` 等构建依赖，
不能仅凭本机主机检查产出固件。请使用此前能成功编译的完整 HiSpark BS21E SDK，
为实验复制出一个独立工程，并保持工具链、库版本和 FEATURE_GLE_HADM 等配置与稳定版一致。
不要覆盖原稳定工程。

将本分支整个 `application/samples/products/sle_measure_dis` 目录放到实验 SDK 的相同路径；
同时同步 `drivers/chips/bs2x/main_init/app_os_init.c` 中应用启动延迟 7000 ms 的改动。
本版不使用 vendor 下的 `sle_poll_measure_dis`：编译日志应指向
`application/samples/products/sle_measure_dis`。

1. Client：选择 `Enable SLE MEASURE_DIS Client sample`，重新完整编译，烧录带 IMU 的 Client。
2. Server：选择 `Enable SLE MEASURE_DIS Server sample`。
   在同一配置菜单的 `Server address suffix (5 receives IMU; others range only)`
   中分别填 5、6、7、8、9，每改一次重新编译并分别保存/烧录相应镜像。
3. 若工具没有显示新地址配置或该整数没有成为编译宏，临时在
   `multi_anchor_config.h` 的 `#ifndef MEASURE_SERVER_SUFFIX` 后加入
   `#define MEASURE_SERVER_SUFFIX 6U`（其他板对应 5U–9U），按板重新编译。
   不要五块板都烧默认 05 地址的同一镜像。
4. 五块 Server 都建议烧本分支的新 Server 程序，尤其 server5 必须使用含 ACK 的新版。
   server6–9 没有 IMU 硬件也正常，不启动 IMU 采样任务。
5. 保留各锚点原有校准值；先检查新工程中的 `sle_measure_dis_server_alg.c`
   与各板原校准配置，避免覆盖校准后误判距离精度。

Client 启动应出现 `IMU_RANGE_BUILD multi-anchor-v1-1Hz-imu05-20Hz`，
以及五条对应不同地址的 `LINK_READY`。Server 启动有 `START SERVER INIT anchor:5` 等。

## 验收

先仅上电 Client 和 server5，确认 server5 每秒约 20 条 IMU、约 1 条 DIST，
再依次增加 server6–9。串口运行参数沿用此前成功设置（通常 115200、8N1），
不要混淆烧录下载波特率与应用串口波特率。

记录不少于 30 分钟，检查：

- 五个 Server 的有效 DIST 频率、无效比例、时间间隔和距离稳定性；各自约 1 Hz。
- server5 的 IMU `sample_sequence` 连续、无重复，样本时间差约 50000 us；
  每批 4 条，目标约 5 个不同批次/秒。按时间窗口统计，不用无线发送次数代替有效采样率。
- Client 每 10 秒输出累计 `TX_STATS`。用相邻统计差值除以时间测频率，
  `imu_ack` 表示确认批次数，`imu_tx` 包含重传；理想 `iq` 增长约 5/s、`imu_ack` 约 5/s。
- 不出现 `APP|exception`、`SP error`、自动 boot、`IMU_DROP`、持续 `TX_FAIL` 或 IQ 丢弃。
  即使重启后自动恢复，也不算通过稳定性验收。
- 短暂断开 server7，其他链路和 server5 的 IMU 应继续；恢复后该板应重新连接。
  短暂断开 server5，测距其余链路应继续；恢复后检查缓冲重传与明确丢样提示。

CSV 沿用 `STATUS.md` 格式：server5 同时输出 `IMU`、`DIST`，其余只输出 `DIST`。
日志解析必须忽略启动文字和诊断行，不把 `IMU_DROP` 当成 `IMU` 样本。

## 当前验证范围

执行 `python3 tools/check_multi_anchor.py`：真实 SDK 头文件语法检查、Client 路由与代次、
IQ 独立缓存/边界/队列满、IMU 环形缓冲、ACK 匹配、Server 输出后确认、重试去重测试。
测试使用主机 OSAL/SLE 模拟，并不覆盖控制器调度、目标二进制布局、无线环境或任务栈峰值。
旧 IMU GPIO 初始化保留四条枚举转换警告，尚未改动该硬件驱动。
正式固件交叉编译、RAM/Flash 链接容量和板上长期稳定性仍待完整 SDK/硬件验证。
