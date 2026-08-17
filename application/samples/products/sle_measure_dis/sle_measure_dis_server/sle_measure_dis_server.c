/**
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2023-2023. All rights reserved.
 *
 * Description: SLE UART Server Source. \n
 *
 * History: \n
 * 2023-07-17, Create file. \n
 */
#include "sle_measure_dis_server.h"
#include "uart_porting.h"
#include "sle_errcode.h"
#include "sle_common.h"
#include "sle_device_manager.h"
#include "sle_connection_manager.h"
#include "sle_device_manager.h"
#include "sle_device_discovery.h"
#include "sle_ssap_server.h"
#include "sle_hadm_manager.h"
#include "sle_measure_dis_server_adv.h"
#include "sle_measure_dis_server_alg.h"

#define DATA_LEN 2
#define ADV_TO_CLIENT 1
#define BLE_SLE_TAG_TASK_DURATION_MS 10
#define SLEM_IQ_DATALEN 512

static uint8_t g_stack_enable_status = 0;

/* sle server handle */
static uint8_t g_server_id = 0;
/* sle service handle */
static uint16_t g_service_handle = 0;
/* sle ntf property handle */
static uint16_t g_property_handle = 0;

/* UUID只支持2字节和16字节 */
#define SLEM_UUID_LEN SLE_UUID_LEN
#define SLEM_CONNET_INVAILD 0xFF
uint8_t g_measure_dis_server_addr[SLE_ADDR_LEN] = { 1, 1, 1, 1, 1, 1 };
uint8_t g_measure_dis_client_addr[SLE_ADDR_LEN] = { 2, 2, 2, 2, 2, 2 };
uint8_t g_measure_dis_conn_id = SLEM_CONNET_INVAILD;

osal_event measure_dis_evt;
unsigned long g_measure_dis_queue;
measure_dis_msg_node_t g_msg_data;
osal_event g_imu_output_evt;
static unsigned long g_imu_output_queue;

typedef struct {
    sle_imu_batch_t *batch;
} imu_output_node_t;

typedef struct __attribute__((packed)) {
    uint16_t sync;
    uint8_t type;
    uint8_t version;
    uint16_t payload_len;
    uint32_t sequence;
} imu_port_frame_header_t;

#define IMU_PORT_FRAME_SYNC 0xA55AU
#define IMU_PORT_FRAME_TYPE_BATCH 1U
#define IMU_PORT_FRAME_VERSION 1U

measure_dis_server_data_t g_measure_dis_server_data = {
    .server_uuid = {0x11, 0x22, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    .service_uuid = {0x11, 0x33, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0},
    .property_uuid = {0x11, 0x44, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0},
};

static void measure_dis_cm_conn_state_cbk(uint16_t conn_id, const sle_addr_t *addr,
                                          sle_acb_state_t conn_state,
                                          sle_pair_state_t pair_state, sle_disc_reason_t disc_reason)
{
    unused(pair_state);
    unused(addr);
    unused(disc_reason);
    if (conn_state == SLE_ACB_STATE_CONNECTED) {
        osal_printk("Connected. conn_id:%d\r\n", conn_id);
        g_measure_dis_conn_id = conn_id;
    } else if (conn_state == SLE_ACB_STATE_DISCONNECTED) {
        osal_printk("Disconnected disc_reason:0x%x .\r\n", disc_reason);
        g_measure_dis_conn_id = SLEM_CONNET_INVAILD;
        measure_dis_start_adv(ADV_TO_CLIENT);
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

static void measure_dis_dd_announce_enable_cbk(uint32_t discovery_id, errcode_t status)
{
    osal_printk("discovery_enable_callback :discover_id:0x%x, status:0x%x \r\n",
                discovery_id, status);
}

static void measure_dis_dd_seek_result_cbk(sle_seek_result_info_t *seek_result_data)
{
    if (seek_result_data == NULL) {
        return;
    }

    if ((memcmp(g_measure_dis_server_addr, seek_result_data->addr.addr, SLE_ADDR_LEN) == 0)) {
        sle_stop_seek();
        sle_connect_remote_device(&(seek_result_data->addr));
    }
}

errcode_t measure_dis_dm_register_cbks(void)
{
    sle_dev_manager_callbacks_t dm_cbks  = {
        .sle_enable_cb = measure_dis_dd_enable_cbk,
        .sle_disable_cb = measure_dis_dd_disenable_cbk,
    };
    return sle_dev_manager_register_callbacks(&dm_cbks);
}

errcode_t measure_dis_dd_register_cbks(void)
{
    sle_announce_seek_callbacks_t dd_cbks  = {
        .announce_enable_cb = measure_dis_dd_announce_enable_cbk,
        .announce_disable_cb = NULL,
        .announce_terminal_cb = NULL,
        .seek_enable_cb = NULL,
        .seek_disable_cb = NULL,
        .seek_result_cb = measure_dis_dd_seek_result_cbk,
    };
    return sle_announce_seek_register_callbacks(&dd_cbks);
}

static void measure_dis_ssaps_read_request_cbk(uint8_t server_id, uint16_t conn_id, ssaps_req_read_cb_t *read_cb_para,
    errcode_t status)
{
    unused(server_id);
    unused(conn_id);
    unused(read_cb_para);
    unused(status);
}

void measure_dis_server_msg_proc(uint8_t *data, uint16_t data_len)
{
    uint32_t ret = ERRCODE_SLE_FAIL;
    if ((data == NULL) || (data_len < sizeof(measure_ids_msg_t))) {
        return;
    }
    measure_ids_msg_t *slem_profile_msg = (measure_ids_msg_t *)(data);
    if (slem_profile_msg->len > (uint32_t)(data_len - sizeof(measure_ids_msg_t))) {
        osal_printk("invalid SLE message length\r\n");
        return;
    }

    switch (slem_profile_msg->type) {
        case SLEM_PROFILE_MSG_IQ:
            ret = measure_dis_remote_iq(slem_profile_msg->len, slem_profile_msg->data);
            break;
        case SLEM_PROFILE_MSG_IMU:
            ret = ERRCODE_INVALID_PARAM;
            break;
        case SLEM_PROFILE_MSG_IMU_BATCH: {
            if (slem_profile_msg->len != sizeof(sle_imu_batch_t)) {
                osal_printk("invalid IMU batch length:%u\r\n", slem_profile_msg->len);
                break;
            }
            const sle_imu_batch_t *batch = (const sle_imu_batch_t *)slem_profile_msg->data;
            if ((batch->sample_count == 0U) || (batch->sample_count > SLE_IMU_BATCH_SAMPLES)) {
                osal_printk("invalid IMU batch count:%u\r\n", batch->sample_count);
                break;
            }
            sle_imu_batch_t *batch_copy = (sle_imu_batch_t *)osal_kmalloc(sizeof(*batch_copy), 0);
            if (batch_copy == NULL) {
                ret = ERRCODE_MALLOC;
                break;
            }
            if (memcpy_s(batch_copy, sizeof(*batch_copy), batch, sizeof(*batch_copy)) != EOK) {
                osal_kfree(batch_copy);
                ret = ERRCODE_MEMCPY;
                break;
            }
            ret = sle_measure_dis_imu_output_add(batch_copy);
            if (ret != ERRCODE_SUCC) {
                osal_kfree(batch_copy);
            }
            break;
        }
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

static void measure_dis_ssaps_write_request_cbk(uint8_t server_id, uint16_t conn_id,
    ssaps_req_write_cb_t *write_cb_para, errcode_t status)
{
    unused(server_id);
    unused(conn_id);
    unused(status);
    if ((write_cb_para == NULL) || (write_cb_para->value == NULL)) {
        return;
    }
    measure_dis_server_msg_proc(write_cb_para->value, write_cb_para->length);
}

static void measure_dis_ssaps_mtu_changed_cbk(uint8_t server_id, uint16_t conn_id,  ssap_exchange_info_t *mtu_size,
    errcode_t status)
{
    osal_printk("[scd server] ssaps myu change cbk server_id:%x, conn_id:%x, mtu_size:%x, status:%x\r\n",
                server_id, conn_id, mtu_size->mtu_size, status);
}

static void measure_dis_ssaps_start_service_cbk(uint8_t server_id, uint16_t handle, errcode_t status)
{
    osal_printk("[scd server] start service cbk server_id:%x, handle:%x, status:%x\r\n",
                server_id, handle, status);
}

static void measure_dis_ssaps_register_cbks(void)
{
    ssaps_callbacks_t ssaps_cbk = {0};
    ssaps_cbk.start_service_cb = measure_dis_ssaps_start_service_cbk;
    ssaps_cbk.mtu_changed_cb = measure_dis_ssaps_mtu_changed_cbk;
    ssaps_cbk.read_request_cb = measure_dis_ssaps_read_request_cbk;
    ssaps_cbk.write_request_cb = measure_dis_ssaps_write_request_cbk;
    ssaps_register_callbacks(&ssaps_cbk);
}

static errcode_t measure_dis_server_service_add(void)
{
    errcode_t ret;
    sle_uuid_t service_uuid = {0};
    service_uuid.len = SLEM_UUID_LEN;
    if (memcpy_s(service_uuid.uuid, SLE_UUID_LEN, g_measure_dis_server_data.service_uuid, SLEM_UUID_LEN) != EOK) {
        return ERRCODE_MEMCPY;
    }
    ret = ssaps_add_service_sync(g_server_id, &service_uuid, 1, &g_service_handle);
    if (ret != ERRCODE_SLE_SUCCESS) {
        return ERRCODE_SLE_FAIL;
    }
    return ERRCODE_SLE_SUCCESS;
}

static errcode_t measure_dis_server_property_add(void)
{
    errcode_t ret;
    uint8_t property_data[DATA_LEN] = {11, 11};
    uint8_t descriptor_data[DATA_LEN] = {0x01, 0x00};
    ssaps_property_info_t property = {0};
    ssaps_desc_info_t descriptor = {0};

    property.uuid.len = SLEM_UUID_LEN;
    if (memcpy_s(property.uuid.uuid, SLE_UUID_LEN, g_measure_dis_server_data.property_uuid, SLEM_UUID_LEN) != EOK) {
        return ERRCODE_MEMCPY;
    }
    property.permissions = SSAP_PERMISSION_READ | SSAP_PERMISSION_WRITE;
    property.value = property_data;
    property.value_len = DATA_LEN;

    ret = ssaps_add_property_sync(g_server_id, g_service_handle, &property,  &g_property_handle);
    if (ret != ERRCODE_SLE_SUCCESS) {
        return ERRCODE_SLE_FAIL;
    }
    descriptor.permissions = SSAP_PERMISSION_READ | SSAP_PERMISSION_WRITE;
    descriptor.value = descriptor_data;
    descriptor.value_len = DATA_LEN;
    descriptor.type = SSAP_DESCRIPTOR_CLIENT_CONFIGURATION;

    ret = ssaps_add_descriptor_sync(g_server_id, g_service_handle, g_property_handle, &descriptor);
    if (ret != ERRCODE_SLE_SUCCESS) {
        return ERRCODE_SLE_FAIL;
    }

    return ERRCODE_SLE_SUCCESS;
}

int measure_dis_server_set_mtu(uint16_t mtu_size, uint16_t version)
{
    ssap_exchange_info_t info = {
        .mtu_size = mtu_size,
        .version = version
    };

    return ssaps_set_info(g_server_id, &info);
}

int measure_dis_server_add(void)
{
    errcode_t ret;
    sle_uuid_t app_uuid = {0};

    measure_dis_server_set_mtu(SLEM_IQ_DATALEN, 1);  /* 需要在连接建立之前 */
    measure_dis_ssaps_register_cbks();
    osal_printk("[server] sle uuid add service in\r\n");
    app_uuid.len = SLEM_UUID_LEN;
    if (memcpy_s(app_uuid.uuid, SLE_UUID_LEN, g_measure_dis_server_data.server_uuid, SLEM_UUID_LEN) != EOK) {
        return ERRCODE_SLE_FAIL;
    }
    ssaps_register_server(&app_uuid, &g_server_id);

    if (measure_dis_server_service_add() != ERRCODE_SLE_SUCCESS) {
        ssaps_unregister_server(g_server_id);
        return ERRCODE_SLE_FAIL;
    }

    if (measure_dis_server_property_add() != ERRCODE_SLE_SUCCESS) {
        ssaps_unregister_server(g_server_id);
        return ERRCODE_SLE_FAIL;
    }
    osal_printk("[server] server_id:%x, service_handle:%x, property_handle:%x\r\n",
        g_server_id, g_service_handle, g_property_handle);
    ret = ssaps_start_service(g_server_id, g_service_handle);
    if (ret != ERRCODE_SLE_SUCCESS) {
        return ERRCODE_SLE_FAIL;
    }
    osal_printk("[server] sle uuid add service out\r\n");
    return ERRCODE_SLE_SUCCESS;
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

int measure_dis_sle_server_ntf_by_addr(uint16_t len, uint8_t *data)
{
    /* type must be 0 */
    errcode_t ret = ERRCODE_FAIL;
    ssaps_ntf_ind_by_uuid_t param = {
        .uuid.len = SLEM_UUID_LEN,
        .start_handle = g_service_handle,
        .end_handle = g_property_handle,
        .type = 0,
        .value_len = len,
        .value = data,
    };
    if (memcpy_s(param.uuid.uuid, SLE_UUID_LEN, g_measure_dis_server_data.property_uuid, SLEM_UUID_LEN) != EOK) {
        return ERRCODE_MEMCPY;
    }
    uint8_t conn_id = g_measure_dis_conn_id;
    ret = ssaps_notify_indicate_by_uuid(g_server_id, conn_id, &param);

    return ret;
}

int measure_dis_server_write_client(uint32_t type, uint8_t *data, uint32_t data_len)
{
    uint16_t len = sizeof(measure_ids_msg_t) + data_len;
    measure_ids_msg_t *slem_msg = (measure_ids_msg_t *)osal_kmalloc(len, 0);
    if (slem_msg == NULL) {
        return ERRCODE_MALLOC;
    }
    slem_msg->type = type;
    slem_msg->len = data_len;
    if (memcpy_s(slem_msg->data, data_len, data, data_len) != EOK) {
        osal_kfree(slem_msg);
        return ERRCODE_MEMCPY;
    }
    uint32_t ret = measure_dis_sle_server_ntf_by_addr(len, (uint8_t *)(slem_msg));
    if (ret != ERRCODE_SUCC) {
        osal_printk("server send to client failed.\n");
    }
    osal_kfree(slem_msg);

    return ret;
}

void sle_measure_dis_msg_proc(void)
{
    while (osal_msg_queue_get_msg_num(g_measure_dis_queue) > 0U) {
        uint32_t msg_data_size = sizeof(measure_dis_msg_node_t);
        errcode_t ret = osal_msg_queue_read_copy(g_measure_dis_queue, &g_msg_data, &msg_data_size, 0);
        if (ret != ERRCODE_SUCC) {
            return;
        }
        ret = measure_dis_match_msg(&g_msg_data);
        if (ret != ERRCODE_SUCC) {
            osal_printk("MSG PROC ERROR type:%d \r\n", g_msg_data.type);
        }
    }
}

errcode_t sle_measure_dis_imu_output_add(sle_imu_batch_t *batch)
{
    imu_output_node_t node = { .batch = batch };
    if ((batch == NULL) || (g_imu_output_queue == 0U) ||
        (osal_msg_queue_write_copy(g_imu_output_queue, &node, sizeof(node), 0) != OSAL_SUCCESS) ||
        (osal_event_write(&g_imu_output_evt, IMU_OUTPUT_EVENT) != OSAL_SUCCESS)) {
        return ERRCODE_FAIL;
    }
    return ERRCODE_SUCC;
}

void sle_measure_dis_imu_output_proc(void)
{
    while (osal_msg_queue_get_msg_num(g_imu_output_queue) > 0U) {
        imu_output_node_t node = {0};
        uint32_t node_size = sizeof(node);
        if (osal_msg_queue_read_copy(g_imu_output_queue, &node, &node_size, 0) != ERRCODE_SUCC) {
            return;
        }
        if (node.batch != NULL) {
#ifdef SW_UART_DEBUG
            uint8_t frame[sizeof(imu_port_frame_header_t) + sizeof(sle_imu_batch_t)];
            imu_port_frame_header_t header = {
                .sync = IMU_PORT_FRAME_SYNC,
                .type = IMU_PORT_FRAME_TYPE_BATCH,
                .version = IMU_PORT_FRAME_VERSION,
                .payload_len = sizeof(sle_imu_batch_t),
                .sequence = node.batch->sequence,
            };
            (void)memcpy_s(frame, sizeof(frame), &header, sizeof(header));
            (void)memcpy_s(frame + sizeof(header), sizeof(frame) - sizeof(header),
                           node.batch, sizeof(*node.batch));
            print_buffer(frame, sizeof(frame));
#endif
            osal_kfree(node.batch);
        }
    }
}

errcode_t sle_measure_dis_msg_add(measure_dis_msg_node_t *msg)
{
    errcode_t ret = ERRCODE_FAIL;

    do {
        if ((msg == NULL) || (g_measure_dis_queue == 0)) {
            break;
        }
        if (osal_msg_queue_write_copy(g_measure_dis_queue,
                                      (void *)msg, sizeof(measure_dis_msg_node_t), 0) != OSAL_SUCCESS) {
            ret = ERRCODE_FAIL;
            break;
        }
        if (osal_event_write(&measure_dis_evt, MEASURE_DIS_MSG_EVENT) != OSAL_SUCCESS) {
            ret = ERRCODE_FAIL;
            break;
        }
        ret = ERRCODE_SUCC;
    } while (0);

    if (ret != ERRCODE_SUCC) {
        uint8_t msg_num = osal_msg_queue_get_msg_num(g_measure_dis_queue);
        osal_printk("MSG ADD FAIL. msg_num:%d ret:%d. \r\n", msg_num, ret);
    }

    return ret;
}


void sle_measure_dis_init(void)
{
    if (osal_event_init(&measure_dis_evt) != ERRCODE_SUCC) {
        osal_printk("touch osal_event_init fail! \r\n");
        return;
    }

    if (osal_msg_queue_create("measure_dis_queue", MEASURE_DIS_MSG_QUEUE_SIZE,
                              &g_measure_dis_queue, 0, sizeof(measure_dis_msg_node_t)) != ERRCODE_SUCC) {
        osal_printk("osal_queue_init init failed \r\n");
        return;
    }
    if ((osal_event_init(&g_imu_output_evt) != ERRCODE_SUCC) ||
        (osal_msg_queue_create("imu_output_queue", IMU_OUTPUT_QUEUE_SIZE, &g_imu_output_queue,
                               0, sizeof(imu_output_node_t)) != ERRCODE_SUCC)) {
        osal_printk("imu output queue init failed\r\n");
    }
}

int measure_dis_server_init(void)
{
    uint32_t ret = ERRCODE_SLE_SUCCESS;
    uint8_t measure_dis_name[SLE_NAME_MAX_LEN] = {'s', 'l', 'e', 'm', '-', 's', '\0'};
    sle_measure_dis_init();
    osal_printk("START SERVER INIT .\r\n");
    measure_dis_protocol_stack_init(g_measure_dis_server_addr, measure_dis_name);
    measure_dis_reg_callbacks();
    ret = measure_dis_server_add();
    check_rc_return_rc(ret, "server add");
    measure_dis_sle_set_adv(ADV_TO_CLIENT, g_measure_dis_server_addr);
    measure_dis_start_adv(ADV_TO_CLIENT);
    return ret;
}
/* 锚点 */

// btc回调函数
// lm_gle_rssi_info_cb(uint16_t conn_id, int8_t rssi)
void carkey_sle_rssi_report_cbk(uint16_t conn_id, int8_t rssi)
{
    osal_printk("SLE RSSI REPORT conn_id:%d, rssi:%d .\r\n", conn_id, rssi);
}
