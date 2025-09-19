/**
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2024-2024. All rights reserved.
 *
 * Description: APP STATUS HEADER FILE. \n
 *
 * History: \n
 * 2024-05-22, Create file. \n
 */
#ifndef APP_STATUS_H
#define APP_STATUS_H

#include "stdbool.h"
#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cplusplus */
#endif /* __cplusplus */

typedef enum {
    APP_SYS_WORK,
    APP_SYS_STANDBY,
    APP_SYS_SLEEP,
} app_sys_status_t;

typedef enum {
    APP_CONNECT_STATUS_IDLE = 0,
    APP_CONNECT_STATUS_ADVING,
    APP_CONNECT_STATUS_CONNECTED,
    APP_CONNECT_STATUS_DISCONNECT,
    APP_CONNECT_STATUS_PAIRED,
} app_connect_status_t;

typedef enum {
    SLE_ADV_TYPE_IDLE = 0,
    SLE_ADV_TYPE_DEFAULT,
    SLE_ADV_TYPE_DIRECTED,
    SLE_ADV_TYPE_WAKEUP,
} sle_adv_type_t;

typedef enum {
    SLE_STOP_ADV_TYPE_IDLE = 0,
    SLE_STOP_ADV_TYPE_DEFAULT,
    SLE_STOP_ADV_TYPE_DIRECTED,
    SLE_STOP_ADV_TYPE_WAKEUP,
} sle_stop_adv_type_t;

enum {
    RCU_MODE_IDLE = 0,
    RCU_MODE_VOICE_WORK = 0x01,
    RCU_MODE_KEY_OPERATION = 0x02,
    RCU_MODE_IR_SEND = 0x04,
    RCU_MODE_IR_STUDY = 0x08,
    RCU_MODE_ADV_SEND = 0x10,
    RCU_MODE_LED_OPEN = 0x20,
    RCU_MODE_OTA_UPGRADE = 0x40,

    RCU_MODE_TEST_NO_SLEPP = 0x80,
};

typedef struct app_globle_status {
    app_sys_status_t app_sys_status;
    app_connect_status_t app_sle_conn_status[CONFIG_SLE_MULTICON_NUM];
    app_connect_status_t app_ble_conn_status;
    sle_adv_type_t app_sle_adv_status[CONFIG_SLE_MULTICON_NUM];
    sle_adv_type_t app_ble_adv_status;
    uint8_t rcu_mode;
    bool is_work_on_sle;
    uint16_t sle_work_conn_id;
} app_globle_status_t;

app_globle_status_t get_app_globle_status(void);
void set_app_sys_status(app_sys_status_t status);
void set_app_sle_adv_status(uint16_t conn_id, sle_adv_type_t status);
void set_app_ble_adv_status(sle_adv_type_t status);
void set_app_sle_conn_status(uint16_t conn_id, app_connect_status_t status);
void set_app_ble_conn_status(app_connect_status_t status);
void set_rcu_mode(uint8_t mode);
void clear_rcu_mode(uint8_t mode);
uint8_t get_rcu_mode(void);
void set_work_on_sle(bool flg);
void set_sle_work_conn_id(uint16_t conn_id);

#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cplusplus */
#endif /* __cplusplus */

#endif