/**
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2024-2024. All rights reserved.
 *
 * Description: APP STATUS SOURCE \n
 *
 * History: \n
 * 2024-5-21, Create file. \n
 */
#include "securec.h"
#include "common_def.h"
#include "osal_debug.h"
#include "app_common.h"
#include "app_msg_queue.h"
#include "app_status.h"

static app_globle_status_t g_app_cur_status;

app_globle_status_t get_app_globle_status(void)
{
    return g_app_cur_status;
}

void set_app_sys_status(app_sys_status_t status)
{
    g_app_cur_status.app_sys_status = status;
}

void set_app_sle_adv_status(uint16_t conn_id, sle_adv_type_t status)
{
    if (conn_id >= CONFIG_SLE_MULTICON_NUM) {
        osal_printk("connect id exceeded the maximum value\r\n");
        return;
    }
    g_app_cur_status.app_sle_adv_status[conn_id] = status;
}

void set_app_ble_adv_status(sle_adv_type_t status)
{
    g_app_cur_status.app_ble_adv_status = status;
}

void set_app_sle_conn_status(uint16_t conn_id, app_connect_status_t status)
{
    if (conn_id >= CONFIG_SLE_MULTICON_NUM) {
        osal_printk("connect id exceeded the maximum value\r\n");
        return;
    }
    g_app_cur_status.app_sle_conn_status[conn_id] = status;
}

void set_app_ble_conn_status(app_connect_status_t status)
{
    g_app_cur_status.app_ble_conn_status = status;
}

void set_rcu_mode(uint8_t mode)
{
    g_app_cur_status.rcu_mode |= mode;
    app_print("[SYS]set: 0x%x, now: 0x%x\r\n", mode, g_app_cur_status.rcu_mode);
}

void clear_rcu_mode(uint8_t mode)
{
    g_app_cur_status.rcu_mode &= ~mode;
    app_print("[SYS]clear: 0x%x, now: 0x%x\r\n", mode, g_app_cur_status.rcu_mode);

    if (g_app_cur_status.rcu_mode == RCU_MODE_IDLE) {
        app_msg_data_t msg;
        msg.type = SLEPP_CHECK_MSG;
        msg.length = 0;
        app_write_msgqueue(msg);
    }
}

uint8_t get_rcu_mode(void)
{
    return g_app_cur_status.rcu_mode;
}

void set_work_on_sle(bool flg)
{
    g_app_cur_status.is_work_on_sle = flg;
}

void set_sle_work_conn_id(uint16_t conn_id)
{
    g_app_cur_status.sle_work_conn_id = conn_id;
}