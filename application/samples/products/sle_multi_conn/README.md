
### SLE 多连接示例（sle_multi_conn）分步解读

本指南将 `sle_multi_conn` 示例分解为清晰的步骤，帮助您理解一个 SLE 客户端如何同时连接并管理多个服务端。

---

### 第一步：环境准备与核心配置

在烧录和运行代码之前，必须首先配置好编译环境，明确每个设备的角色和网络身份。

1.  **选择设备角色 (Role Selection)**
    * 在 `Kconfig` 菜单中，确定当前设备是作为 **服务端** 还是 **客户端**。
    * **服务端**：使能 `CONFIG_SAMPLE_SUPPORT_SLE_MULTI_CONN_SERVER` (默认选项)。
    * **客户端**：使能 `CONFIG_SAMPLE_SUPPORT_SLE_MULTI_CONN_CLIENT`。

2.  **配置关键参数 (Key Configuration)**
    * **`CONFIG_SLE_MULTI_CONN_NUM`**
        * **作用**：定义客户端计划连接的服务端总数。
        * **范围**：1 ~ 8，默认为 4。

    * **服务端专属配置**
        * **`CONFIG_CURRENT_SERVER_INDEX`**：为当前编译的服务端实例指定一个唯一的索引（0-7）。此索引将用于自动派生其 MAC 地址和广播名称，确保每个服务端都独一无二。
        * **`CONFIG_CURRENT_SERVER_ADDR`**：根据上述索引自动生成，无需手动修改。
        * **`CONFIG_SLE_MULTI_CONN_SERVER_NAME`**：根据上述索引自动生成，无需手动修改。

    * **客户端专属配置**
        * **`CONFIG_SLE_MULTI_CONN_CLIENT_ADDR`**：设置客户端自身的 MAC 地址。
        * **`CONFIG_SLE_MULTI_CONN_SERVERx_ADDR`** (x=0..7)：配置目标服务端地址库。**此处的 MAC 地址必须与您实际运行的每个服务端的 `CONFIG_CURRENT_SERVER_ADDR` 完全一致。**

---

### 第二步：服务端工作流程解析

服务端的工作流程相对简单：初始化、广播、等待连接，并在连接后主动推送数据。

1.  **启动与初始化**
    * 程序入口 `sle_multi_conn_entry()` 创建一个名为 `SleMultiConnServerTask` 的任务。
    * 该任务执行 `sle_multi_conn_server_init()` 进行核心初始化。

2.  **配置服务与广播**
    * **创建 SSAP 服务**：通过 `ssaps_register_server()` 注册一个服务（UUID: `0x2222`）和一个属性（UUID: `0x2323`），该属性用于后续的数据通知。
    * **启动广播**：配置广播参数（基于 `CONFIG_CURRENT_SERVER_INDEX` 派生的名称和地址），并启动广播，等待客户端的连接请求。广播句柄也与索引相关，确保了唯一性。

3.  **处理连接与数据发送**
    * **连接建立**：当客户端连接成功，`sle_connect_state_changed_cbk` 回调被触发，记录下此连接的 ID (`conn_id`)。
    * **参数更新**：连接参数更新完成后，`sle_connect_param_update_cb` 回调被触发。
    * **主动推送数据**：初始化流程的最后，或在特定逻辑触发时，服务端调用 `sle_multi_conn_server_send_notify_indicate()` 函数，将自身的设备名称 (`CONFIG_SLE_MULTI_CONN_SERVER_NAME`) 作为通知（Notify）或指示（Indicate）发送给已连接的客户端。
    * **断线重连**：如果连接断开，服务端会自动重新启动广播，等待新的连接。

---

### 第三步：客户端核心工作流程解析

客户端的流程更为复杂，它通过一个串行循环的方式，逐个建立与多个服务端的并行连接。

1.  **启动与地址库加载**
    * 程序入口 `sle_multi_conn_entry()` 创建 `SleMultiConnClientTask` 任务。
    * **初始化**：`sle_multi_conn_client_init()` 函数执行以下关键操作：
        * 从 Kconfig 加载所有目标服务端的 MAC 地址，构建一个待连接的地址库。
        * 设置客户端本地 MAC 地址。
        * 注册所有必需的回调函数（扫描、连接、配对、SSAP 客户端事件等）。
        * 最后，调用 `sle_multi_conn_start_scan()` **启动第一次扫描**。

2.  **扫描 → 连接 → 再扫描”的循环**
    * **发现目标**：扫描回调 `seek_result_cb` 持续接收广播。一旦发现一个存在于地址库中且当前**尚未连接**的服务端，立即停止扫描。
    * **发起连接**：在扫描停止的回调 `seek_disable_cb` 中，对刚刚发现的目标设备发起 `sle_connect_remote_device()` 连接请求。
    * **连接成功**：连接成功后，`connect_state_changed_cb` 回调被触发：
        * 记录并保存该连接的 `conn_id` 到与该服务端索引对应的数组位置（例如 `g_sle_multi_conn_param.conn_id[i]`）。
        * 更新已连接设备数量 `g_connected_num`。
        * **关键**：如果已连接数 `g_connected_num` 小于目标总数 `CONFIG_SLE_MULTI_CONN_NUM`，**则再次调用 `sle_multi_conn_start_scan()` 继续扫描下一个未连接的服务端**。

3.  **配对、发现服务与参数更新（单个连接的深化）**
    * **发起配对**：连接成功后，立即发起 `sle_pair_remote_device()` 配对请求。
    * **服务发现**：配对完成回调 `pair_complete_cb` 被触发后，客户端会立即：
        * 交换 MTU 大小。
        * 发起服务发现 (`ssapc_find_structure()`) 与属性发现 (`ssapc_find_property...`)。
        * 在相应的回调中，保存获取到的服务端属性句柄 `conn_hdl[i]` 和服务 `uuid[i]`。
    * **参数更新**：服务发现完成后，发起连接参数更新请求。
    * **准备就绪**：当参数更新完成回调 `connect_param_update_cb` 被触发，程序会将该连接的就绪标志位置位：`param_update_ready[i] = 1`。**至此，该路连接才算完全建立，可以进行业务数据交互**。

---

### 第四步：客户端业务逻辑与数据交互

当一路或多路连接的 `param_update_ready` 标志位为 1 后，客户端的主任务循环开始与它们进行数据交互。

1.  **业务任务轮询**
    * `sle_multi_conn_client_task` 任务以固定的周期（`MULTI_CONN_TASK_DURATION_MS`）运行。
    * 它会遍历所有潜在的连接（`for i in 0..NUM-1`）。
    * 如果检测到 `param_update_ready[i] == 1`，则证明第 `i` 个服务端已完全就绪。

2.  **向指定服务端发送数据（读/写）**
    * **区分目标**：通过数组索引 `i`，客户端可以精确地获取到目标服务端的 `conn_id[i]` 和 `conn_hdl[i]`。
    * **写请求 (Write)**：调用 `ssapc_write_req()`，传入 `conn_id[i]` 和包含 `conn_hdl[i]` 的参数。示例中写入了固定的 "abcd" 字符串。
    * **读请求 (Read)**：调用 `ssapc_read_req()`，传入 `conn_id[i]` 和 `conn_hdl[i]`。

3.  **接收来自服务端的推送数据**
    * **接收点**：客户端预先注册了两个关键回调函数来处理服务端推送的数据：
        * **通知 (Notify) 接收**：`sle_multi_conn_notification_cb()`
        * **指示 (Indicate) 接收**：`sle_multi_conn_indication_cb()`
    * **区分来源**：这两个回调函数的参数中都包含 `conn_id`。通过调用 `get_sle_multi_conn_server_index(conn_id)`，客户端可以将 `conn_id` 反向解析出它对应的服务端索引 `i`，从而明确数据是来自哪个服务端，并进行相应的处理。


