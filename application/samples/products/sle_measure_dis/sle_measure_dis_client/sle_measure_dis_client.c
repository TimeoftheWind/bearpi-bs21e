/**
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2023-2023. All rights reserved.
 *
 * Description: SLE uart sample of client. \n
 *
 * History: \n
 * 2023-04-03, Create file. \n
 */
#include "sle_measure_dis_client.h"
#include "sle_errcode.h"
#include "sle_common.h"
#include "sle_device_manager.h"
#include "sle_connection_manager.h"
#include "sle_device_manager.h"
#include "sle_device_discovery.h"
#include "sle_ssap_client.h"
#include "sle_hadm_manager.h"
#include "cmsis_os2.h"
#include "sle_measure_dis_client_slem.h"
#include "tcxo.h"

static uint8_t g_sle_tx_buf[sizeof(measure_ids_msg_t) + sizeof(sle_channel_sounding_iq_trans_t)]
    __attribute__((aligned(8)));

#define BLE_SLE_TAG_TASK_DURATION_MS 10
#define ADV_TO_SERVER 2
/* UUID只支持2字节和16字节 */
#define SLEM_UUID_LEN SLE_UUID_LEN
#define KEY_CONNECT_DELAY_TIME 100
#define CLIENT_SETUP_START_DELAY_MS 100
#define SLEM_IQ_DATALEN 512

static uint8_t g_stack_enable_status = 0;
enum { LINK_FREE, LINK_DISCOVER, LINK_WAIT_SERVICE, LINK_SET_PARAM, LINK_ENABLE, LINK_READY };
typedef struct {
    uint16_t conn_id;
    uint16_t property_handle;
    uint16_t mtu;
    uint8_t state;
    bool service_complete;
    uint32_t generation;
    uint64_t next_setup_us;
} anchor_link_t;
static anchor_link_t g_links[MAX_SERVERS];
/* Short IRQ-protected snapshots; never hold this protection across SLE calls. */
static volatile bool g_scan_active;
static volatile bool g_scan_starting;
static volatile bool g_connect_pending;
static volatile bool g_connect_submitted;
static sle_addr_t g_connect_addr;
static uint64_t g_connect_deadline_us;
static uint32_t g_connect_generation;
static uint64_t g_next_scan_us;
uint8_t g_measure_dis_client_addr[SLE_ADDR_LEN] = { 2, 2, 2, 2, 2, 2 };

measure_dis_server_data_t g_measure_dis_server_data = {
    .server_uuid = {0x11, 0x22, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    .service_uuid = {0x11, 0x33, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0},
    .property_uuid = {0x11, 0x44, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0},
};

errcode_t measure_dis_slem_set_param(uint16_t conn_id)
{
    sle_set_channel_sounding_param_ex_t param = {
        .acb_interval = 0,
        .con_anchor_num = MAX_SERVERS,
        .cs_interval = SLE_MEASURE_1HZ,
        .freq_space = 0,
        .is_cs_param_chg = 0,
        .refresh_rate = 1,
    };
    errcode_t ret = sle_set_channel_sounding_param_ex(conn_id, &param);
    if ((ret != ERRCODE_SUCC) || (conn_id == SLEM_CONNET_INVAILD)) {
        osal_printk("slem set param failed, ret = %d.\n", ret);
    }
    osal_printk("slem set param finish. refresh_rate:%d \n", param.refresh_rate);
    return ret;
}

errcode_t measure_dis_slem_set_enable(uint16_t conn_id)
{
    osal_printk("SLEM SET ENABLE conn_id:%x.\r\n", conn_id);
    errcode_t ret = sle_set_channel_sounding_enable(conn_id);
    /* 解决测距使能之后，马上断链导致没有底层回调 */
    if ((ret != ERRCODE_SUCC) || (conn_id == SLEM_CONNET_INVAILD)) {
        osal_printk("slem set enable failed, ret = %d.\n", ret);
    }
    return ret;
}

static int measure_dis_client_discover_key(uint16_t conn_id, uint8_t type)
{
    uint32_t ret = ERRCODE_SLE_FAIL;

    ssapc_find_structure_param_t param = {0};
    param.type = type,
    param.start_hdl = 1,
    param.end_hdl = 0xFFFF,
    param.uuid.len = SLEM_UUID_LEN;
    if (type == SSAP_FIND_TYPE_PRIMARY_SERVICE) {
        if (memcpy_s(param.uuid.uuid, SLEM_UUID_LEN,
            g_measure_dis_server_data.service_uuid, SLEM_UUID_LEN) != EOK) {
            return ERRCODE_MEMCPY;
        }
    } else if (type == SSAP_FIND_TYPE_PROPERTY) {
        if (memcpy_s(param.uuid.uuid, SLEM_UUID_LEN,
            g_measure_dis_server_data.property_uuid, SLEM_UUID_LEN) != EOK) {
            return ERRCODE_MEMCPY;
        }
    }
    ret = ssapc_find_structure(0, conn_id, &param);
    osal_printk("SLE START CLIENT DISCOVER SERVER. conn_id:%d, ret:%d \r\n", conn_id, ret);

    return ret;
}

static int anchor_slot(const sle_addr_t *addr)
{
    static const uint8_t prefix[5] = {1, 1, 1, 1, 1};
    if (addr == NULL || memcmp(addr->addr, prefix, sizeof(prefix)) != 0 ||
        addr->addr[5] < 5U || addr->addr[5] >= 5U + MAX_SERVERS) {
        return -1;
    }
    return addr->addr[5] - 5U;
}

static int conn_slot(uint16_t conn_id)
{
    for (unsigned int i = 0; i < MAX_SERVERS; i++) {
        if (g_links[i].state != LINK_FREE && g_links[i].conn_id == conn_id) {
            return (int)i;
        }
    }
    return -1;
}

bool measure_dis_client_link_token(uint16_t conn_id, measure_dis_link_token_t *token)
{
    unsigned int irq = osal_irq_lock();
    int slot = conn_slot(conn_id);
    if (slot >= 0 && token != NULL) {
        *token = (measure_dis_link_token_t){conn_id, (uint16_t)slot, g_links[slot].generation};
    }
    osal_irq_restore(irq);
    return slot >= 0 && token != NULL;
}

bool measure_dis_client_token_ready(const measure_dis_link_token_t *token)
{
    if (token == NULL || token->slot >= MAX_SERVERS) {
        return false;
    }
    unsigned int irq = osal_irq_lock();
    anchor_link_t link = g_links[token->slot];
    osal_irq_restore(irq);
    return link.state == LINK_READY && link.conn_id == token->conn_id &&
        link.generation == token->generation;
}

bool measure_dis_client_imu_token(measure_dis_link_token_t *token)
{
    unsigned int irq = osal_irq_lock();
    unsigned int slot = IMU_SERVER_SUFFIX - 5U;
    anchor_link_t link = g_links[slot];
    if (token != NULL) {
        *token = (measure_dis_link_token_t){link.conn_id, (uint16_t)slot, link.generation};
    }
    osal_irq_restore(irq);
    return token != NULL && link.state == LINK_READY;
}

static void measure_dis_cm_conn_state_cbk(uint16_t conn_id, const sle_addr_t *addr,
    sle_acb_state_t conn_state, sle_pair_state_t pair_state, sle_disc_reason_t disc_reason)
{
    unused(pair_state);
    unsigned int irq = osal_irq_lock();
    int slot = anchor_slot(addr);
    if (conn_state == SLE_ACB_STATE_CONNECTED && slot >= 0 && g_links[slot].state == LINK_FREE) {
        uint32_t generation = g_links[slot].generation + 1U;
        g_links[slot] = (anchor_link_t){ .conn_id = conn_id, .generation = generation,
            .state = LINK_DISCOVER };
        if (g_connect_pending && memcmp(addr->addr, g_connect_addr.addr, SLE_ADDR_LEN) == 0) {
            g_connect_pending = false;
            g_connect_submitted = false;
        }
        g_scan_active = false;
    } else if (conn_state == SLE_ACB_STATE_DISCONNECTED) {
        slot = conn_slot(conn_id);
        if (slot >= 0) {
            g_links[slot].state = LINK_FREE;
            g_links[slot].generation++;
        }
        /* An established anchor dropping must not cancel another anchor's connect. */
        if (addr != NULL && g_connect_pending &&
            memcmp(addr->addr, g_connect_addr.addr, SLE_ADDR_LEN) == 0) {
            g_connect_pending = false;
            g_connect_submitted = false;
        }
    } else {
        slot = -1;
    }
    osal_irq_restore(irq);
    if (slot >= 0) {
        osal_printk("LINK,anchor:%u,conn:%u,state:%u,reason:0x%x\r\n",
            (unsigned int)slot + 5U, (unsigned int)conn_id,
            (unsigned int)conn_state, (unsigned int)disc_reason);
    }
}

/* All connection setup and scanning commands run in the application worker. */
void measure_dis_client_process(void)
{
    uint64_t now = uapi_tcxo_get_us();
    unsigned int connect_irq = osal_irq_lock();
    bool submit = g_connect_pending && !g_connect_submitted;
    bool timed_out = g_connect_pending && now > g_connect_deadline_us;
    sle_addr_t connect_addr = g_connect_addr;
    uint32_t connect_generation = g_connect_generation;
    if (submit) {
        g_connect_submitted = true;
    }
    osal_irq_restore(connect_irq);
    if (submit && !timed_out) {
        (void)sle_stop_seek();
        errcode_t ret = sle_connect_remote_device(&connect_addr);
        if (ret != ERRCODE_SUCC) {
            connect_irq = osal_irq_lock();
            if (g_connect_generation == connect_generation) {
                g_connect_pending = false;
                g_connect_submitted = false;
            }
            osal_irq_restore(connect_irq);
            g_next_scan_us = now + 500000ULL;
            osal_printk("CONNECT_FAIL,ret:0x%x\r\n", (unsigned int)ret);
        }
    }
    if (timed_out) {
        (void)sle_disconnect_remote_device(&connect_addr);
        connect_irq = osal_irq_lock();
        if (g_connect_generation == connect_generation) {
            g_connect_pending = false;
            g_connect_submitted = false;
        }
        osal_irq_restore(connect_irq);
    }
    bool missing = false;
    for (unsigned int i = 0; i < MAX_SERVERS; i++) {
        unsigned int irq = osal_irq_lock();
        anchor_link_t link = g_links[i];
        osal_irq_restore(irq);
        if (link.state == LINK_FREE) {
            missing = true;
            continue;
        }
        if (link.state == LINK_READY || now < link.next_setup_us) {
            continue;
        }
        uint8_t next = link.state;
        errcode_t ret = ERRCODE_SUCC;
        if (link.state == LINK_DISCOVER) {
            ret = measure_dis_client_discover_key(link.conn_id, SSAP_FIND_TYPE_PROPERTY);
            ssap_exchange_info_t info = { .mtu_size = SLEM_IQ_DATALEN, .version = 1 };
            if (ret == ERRCODE_SUCC) {
                ret = ssapc_exchange_info_req(0, link.conn_id, &info);
            }
            next = LINK_WAIT_SERVICE;
        } else if (link.state == LINK_WAIT_SERVICE) {
            if (link.service_complete && link.property_handle != 0 && link.mtu >= sizeof(g_sle_tx_buf)) {
                next = LINK_SET_PARAM;
            } else if (now >= link.next_setup_us) {
                next = LINK_DISCOVER;
            }
        } else if (link.state == LINK_SET_PARAM) {
            ret = measure_dis_slem_set_param(link.conn_id);
            next = LINK_ENABLE;
        } else if (link.state == LINK_ENABLE) {
            ret = measure_dis_slem_set_enable(link.conn_id);
            next = LINK_READY;
        }
        irq = osal_irq_lock();
        if (g_links[i].generation == link.generation && g_links[i].state != LINK_FREE) {
            g_links[i].state = ret == ERRCODE_SUCC ? next : LINK_DISCOVER;
            g_links[i].next_setup_us = now + (next == LINK_WAIT_SERVICE ? 1000000ULL : 100000ULL);
        }
        osal_irq_restore(irq);
        if (next == LINK_READY && ret == ERRCODE_SUCC) {
            osal_printk("LINK_READY,anchor:%u,conn:%u,rate:1Hz,imu:%u\r\n", i + 5U,
                (unsigned int)link.conn_id, i + 5U == IMU_SERVER_SUFFIX);
        }
    }
    if (missing && !g_connect_pending && !g_scan_active && !g_scan_starting && now >= g_next_scan_us) {
        g_next_scan_us = now + 1000000ULL;
        (void)measure_dis_start_scan();
    }
}

errcode_t measure_dis_cm_register_cbks(void)
{
    sle_connection_callbacks_t cm_cbks = {0};
    cm_cbks.connect_state_changed_cb = measure_dis_cm_conn_state_cbk;
    cm_cbks.connect_param_update_req_cb = NULL;
    cm_cbks.connect_param_update_cb = NULL;
    cm_cbks.auth_complete_cb = NULL;
    cm_cbks.pair_complete_cb = NULL;
    cm_cbks.read_rssi_cb = NULL;

    return sle_connection_register_callbacks(&cm_cbks);
}

static void measure_dis_dd_enable_cbk(uint8_t status)
{
    osal_printk("sle enable result statue:0x%x .\r\n", status);
    g_stack_enable_status = 1;
}

static void measure_dis_dd_disenable_cbk(uint8_t status)
{
    osal_printk("sle disable result statue:0x%x .\r\n", status);
    g_stack_enable_status = 0;
}

static void measure_dis_dd_seek_result_cbk(sle_seek_result_info_t *seek_result_data)
{
    if (seek_result_data == NULL) {
        return;
    }

    unsigned int irq = osal_irq_lock();
    int slot = anchor_slot(&seek_result_data->addr);
    if (slot >= 0 && g_links[slot].state == LINK_FREE && !g_connect_pending) {
        g_connect_addr = seek_result_data->addr;
        g_connect_generation++;
        g_connect_deadline_us = uapi_tcxo_get_us() + 5000000ULL;
        g_connect_pending = true;
        g_connect_submitted = false;
    }
    osal_irq_restore(irq);
}

static void client_seek_enabled(errcode_t status)
{
    g_scan_starting = false;
    g_scan_active = status == ERRCODE_SUCC;
}

static void client_seek_disabled(errcode_t status)
{
    unused(status);
    g_scan_active = false;
    g_scan_starting = false;
}

errcode_t measure_dis_dm_register_cbks(void)
{
    sle_dev_manager_callbacks_t dm_cbks = {
        .sle_power_on_cb = NULL,
        .sle_enable_cb = measure_dis_dd_enable_cbk,
        .sle_disable_cb = measure_dis_dd_disenable_cbk,
    };
    return sle_dev_manager_register_callbacks(&dm_cbks);
}

errcode_t measure_dis_dd_register_cbks(void)
{
    sle_announce_seek_callbacks_t dd_cbks  = {
        .announce_enable_cb = NULL,
        .announce_disable_cb = NULL,
        .announce_terminal_cb = NULL,
        .seek_enable_cb = client_seek_enabled,
        .seek_disable_cb = client_seek_disabled,
        .seek_result_cb = measure_dis_dd_seek_result_cbk,
    };
    return sle_announce_seek_register_callbacks(&dd_cbks);
}

STATIC void measure_dis_ssapc_find_structure_cbk(uint8_t client_id, uint16_t conn_id,
    ssapc_find_service_result_t *service, errcode_t status)
{
    if (service == NULL) {
        return;
    }
    osal_printk("[ssap client] find structure cbk client: %d conn_id:%d status: %d \n",
        client_id, conn_id, status);
    osal_printk("[ssap client] find structure cbk start_hdl: %d end_hdl:%d \n",
        service->start_hdl, service->end_hdl);

}

STATIC void measure_dis_ssapc_find_property_cbk(uint8_t client_id, uint16_t conn_id,
    ssapc_find_property_result_t *property, errcode_t status)
{
    if (property == NULL) {
        return;
    }
    osal_printk("[ssap client] find property cbk client: %d conn_id:%d status: %d \n",
        client_id, conn_id, status);
    osal_printk("[ssap client] find property cbk handle: %d \n", property->handle);

    unsigned int irq = osal_irq_lock();
    int slot = conn_slot(conn_id);
    if (slot >= 0 && status == ERRCODE_SUCC) {
        g_links[slot].property_handle = property->handle;
    }
    osal_irq_restore(irq);
}

STATIC void measure_dis_ssapc_find_structure_complete_cbk(uint8_t client_id, uint16_t conn_id,
    ssapc_find_structure_result_t *structure_result, errcode_t status)
{
    osal_printk("[ssap client] find structure complete cbk client: %d conn_id:%d status: %d \n",
                client_id, conn_id, status);
    if (structure_result == NULL) {
        return;
    }

    osal_printk("SERVICES DISCOVER COMPLETE. \r\n");
    unsigned int irq = osal_irq_lock();
    int slot = conn_slot(conn_id);
    if (slot >= 0 && status == ERRCODE_SUCC) {
        g_links[slot].service_complete = true;
    }
    osal_irq_restore(irq);
}

void measure_dis_client_msg_proc(uint8_t *data, uint16_t data_len)
{
    unused(data_len);
    uint32_t ret = ERRCODE_SLE_FAIL;
    measure_ids_msg_t *slem_profile_msg = (measure_ids_msg_t *)(data);

    switch (slem_profile_msg->type) {
        case SLEM_PROFILE_MSG_IQ:
            break;
        default:
            for (int i = 0; i < data_len; i++) {
                osal_printk("[%d]:%d\r\n", i, data[i]);
            }
            break;
    }

    if (unlikely(ret != ERRCODE_SLE_SUCCESS)) {
        osal_printk("client proc msg failed MSG_TYPE:%x ret:0x%x \r\n", slem_profile_msg->type, ret);
    }
}

/**
 * @brief  数据通知处理函数
 */
STATIC void measure_dis_ssapc_notification_cbk(uint8_t client_id, uint16_t conn_id, ssapc_handle_value_t *data,
    errcode_t status)
{
    unused(client_id);
    if ((data == NULL) || (data->data == NULL)) {
        return;
    }
    measure_dis_link_token_t token;
    if (status == ERRCODE_SUCC && measure_dis_client_link_token(conn_id, &token) &&
        token.slot + 5U == IMU_SERVER_SUFFIX && data->data_len == sizeof(measure_ids_msg_t) + sizeof(sle_imu_ack_t)) {
        measure_ids_msg_t *msg = (measure_ids_msg_t *)data->data;
        if (msg->type == SLEM_PROFILE_MSG_IMU_ACK && msg->len == sizeof(sle_imu_ack_t)) {
            sle_imu_ack_t ack;
            (void)memcpy_s(&ack, sizeof(ack), msg->data, sizeof(ack));
            sle_imu_tx_ack(&ack);
        }
    }
}

/**
 * @brief  数据指示处理函数
 */
STATIC void measure_dis_ssapc_indication_cb(uint8_t client_id, uint16_t conn_id, ssapc_handle_value_t *data,
    errcode_t status)
{
    unused(client_id);
    osal_printk("[ssap client] indication info cbk client %d,mtu_size:%d, version:%x, status:%x\n",
        conn_id, data->handle, data->type, status);
}

/**
 * @brief  MTU信息交换响应处理函数
 */
STATIC void measure_dis_ssapc_exchange_info_cbk(uint8_t client_id, uint16_t conn_id, ssap_exchange_info_t *param,
    errcode_t status)
{
    unused(client_id);
    if (param == NULL) {
        return;
    }
    osal_printk("[ssap client] exchange info cbk client %d,mtu_size:%d, version:%x, status:%x\n",
        conn_id, param->mtu_size, param->version, status);
    unsigned int irq = osal_irq_lock();
    int slot = conn_slot(conn_id);
    if (slot >= 0 && status == ERRCODE_SUCC) {
        g_links[slot].mtu = param->mtu_size;
    }
    osal_irq_restore(irq);
}

errcode_t measure_dis_ssapc_register_cbks(void)
{
    ssapc_callbacks_t ssapc_cbks = {0};

    osal_printk("[ssap client] client register cbk\r\n");
    /* 服务发现模块回调函数 */
    ssapc_cbks.find_structure_cb = measure_dis_ssapc_find_structure_cbk;
    ssapc_cbks.ssapc_find_property_cbk = measure_dis_ssapc_find_property_cbk;
    ssapc_cbks.find_structure_cmp_cb = measure_dis_ssapc_find_structure_complete_cbk;
    ssapc_cbks.read_cfm_cb = NULL;
    ssapc_cbks.read_by_uuid_cmp_cb = NULL;
    ssapc_cbks.write_cfm_cb = NULL;
    ssapc_cbks.exchange_info_cb = measure_dis_ssapc_exchange_info_cbk;
    ssapc_cbks.notification_cb = measure_dis_ssapc_notification_cbk;
    ssapc_cbks.indication_cb = measure_dis_ssapc_indication_cb;
    return ssapc_register_callbacks(&ssapc_cbks);
}

errcode_t measure_dis_set_local_addr(uint8_t *addr)
{
    sle_addr_t sle_addr = {0};
    if (memcpy_s(sle_addr.addr, SLE_ADDR_LEN, addr, SLE_ADDR_LEN) != EOK) {
        return ERRCODE_MEMCPY;
    };
    return sle_set_local_addr(&sle_addr);
}

errcode_t measure_dis_set_local_name(uint8_t *name, uint8_t len)
{
    return sle_set_local_name(name, len);
}

void measure_dis_protocol_stack_init(uint8_t *addr, uint8_t *name)
{
    uint32_t ret = ERRCODE_SLE_SUCCESS;

    ret |= measure_dis_dm_register_cbks();
    ret |= measure_dis_cm_register_cbks();
    ret |= measure_dis_dd_register_cbks();
    ret = enable_sle();
    while (g_stack_enable_status != 1) {
        osal_msleep(BLE_SLE_TAG_TASK_DURATION_MS);
    }
    measure_dis_set_local_addr(addr);
    measure_dis_set_local_name(name, strlen((char *)name));
}

static int measure_dis_client_write_proc(const measure_dis_link_token_t *token,
    uint8_t type, uint16_t len, uint8_t *data)
{
    errcode_t ret = ERRCODE_FAIL;
    ssapc_write_param_t param = {0};
    if (!measure_dis_client_token_ready(token)) {
        return ERRCODE_FAIL;
    }
    unsigned int irq = osal_irq_lock();
    anchor_link_t link = g_links[token->slot];
    osal_irq_restore(irq);
    if (len > link.mtu) {
        return ERRCODE_INVALID_PARAM;
    }
    param.handle = link.property_handle;
    param.type = type;
    param.data_len = len;
    param.data = data;

    ret = ssapc_write_cmd(0, token->conn_id, &param);
    if (ret != ERRCODE_SUCC) {
        osal_printk("TX_FAIL,anchor:%u,conn:%u,ret:0x%x\r\n",
            token->slot + 5U, (unsigned int)token->conn_id, (unsigned int)ret);
    }

    return ret;
}

int measure_dis_client_send_now(const measure_dis_link_token_t *token,
    uint32_t type, const uint8_t *data, uint32_t data_len)
{
    uint32_t ret = ERRCODE_SUCC;
    if (data == NULL || data_len > sizeof(g_sle_tx_buf) - sizeof(measure_ids_msg_t)) {
        return ERRCODE_INVALID_PARAM;
    }
    uint16_t len = (uint16_t)(sizeof(measure_ids_msg_t) + data_len);
    measure_ids_msg_t *slem_msg = (measure_ids_msg_t *)g_sle_tx_buf;
    slem_msg->type = type;
    slem_msg->len = data_len;
    if (memcpy_s(slem_msg->data, data_len, data, data_len) != EOK) {
        return ERRCODE_MEMCPY;
    }
    ret = measure_dis_client_write_proc(token, 0, len, (uint8_t *)(slem_msg));
    if (ret != ERRCODE_SUCC) {
        osal_printk("client send to server failed.\n");
    }
    return ret;
}

errcode_t measure_dis_set_scan(void)
{
    errcode_t ret;
    sle_seek_param_t param = {0};
    param.own_addr_type = 0;
    param.filter_duplicates = 1;
    param.seek_filter_policy = SLE_SEEK_FILTER_ALLOW_ALL;
    param.seek_phys = 1;
    param.seek_type[0] = 1;
    param.seek_interval[0] = 0x12c0;
    param.seek_window[0] = 0x500;
    ret = sle_set_seek_param(&param);

    return ret;
}

int measure_dis_start_scan(void)
{
    errcode_t ret;
    ret = measure_dis_set_scan();
    if (ret != ERRCODE_SLE_SUCCESS) {
        return ret;
    }
    g_scan_starting = true;
    ret = sle_start_seek();
    if (ret != ERRCODE_SLE_SUCCESS) {
        g_scan_starting = false;
        return ret;
    }
    return ERRCODE_SLE_SUCCESS;
}

int measure_dis_client_init(void)
{
    uint32_t ret = ERRCODE_SLE_SUCCESS;
    uint8_t measure_dis_name[SLE_NAME_MAX_LEN] = {'s', 'l', 'e', 'm', '-', 'c', '\0'};
    osal_printk("START CLIENT INIT .\r\n");
    measure_dis_protocol_stack_init(g_measure_dis_client_addr, measure_dis_name);
    measure_dis_reg_callbacks();
    ret = measure_dis_ssapc_register_cbks();
    osal_printk("MULTI_ANCHOR_BUILD v2,anchors:05-09,range:1Hz,imu:05,batch:%u,window:%u\r\n",
        (unsigned int)SLE_IMU_BATCH_SAMPLES, (unsigned int)IMU_TX_WINDOW_SIZE);
    return ret;
}
/* 钥匙 */

// btc回调函数
// lm_gle_rssi_info_cb(uint16_t conn_id, int8_t rssi)
void carkey_sle_rssi_report_cbk(uint16_t conn_id, int8_t rssi)
{
    osal_printk("SLE RSSI REPORT conn_id:%d, rssi:%d .\r\n", conn_id, rssi);
}
