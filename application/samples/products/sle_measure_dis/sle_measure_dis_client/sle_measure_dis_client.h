/**
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2023-2023. All rights reserved.
 *
 * Description: SLE MEASURE_DIS sample of client. \n
 *
 * History: \n
 * 2023-04-03, Create file. \n
 */
#ifndef SLE_MEASURE_DIS_CLIENT_H
#define SLE_MEASURE_DIS_CLIENT_H

#include <stdint.h>
#include <stdbool.h>
#include <securec.h>
#include "errcode.h"
#include "std_def.h"
#include "soc_osal.h"
#include "common_def.h"
#include "sle_ssap_client.h"
#include "../app_imu_protocol.h"

#define MEASURE_DIS_NUM_CARRY_10_7     10000000
#define MEASURE_DIS_NUM_CARRY_1000     1000
#define MEASURE_DIS_NUM_CARRY_100      100
#define MEASURE_DIS_NUM_CARRY_10       10
#define MEASURE_DIS_NUM_CARRY_2        2

#define SLEM_CONNET_INVAILD 0xFF

typedef enum {
    SLEM_PROFILE_MSG_IQ = 0xFFFFFFEA,
} slem_profile_msg_type_t;

typedef struct {
    uint32_t        type;                             /*!< 消息类型 */
    uint32_t        len;                              /*!< 消息类型 */
    uint8_t         data[0];
} measure_ids_msg_t;

typedef struct server_data {
    uint8_t property_uuid[SLE_UUID_LEN];
    uint8_t server_uuid[SLE_UUID_LEN];
    uint8_t service_uuid[SLE_UUID_LEN];
    uint16_t begin_hdl;
    uint16_t end_hdl;
    uint16_t property_handle;
} measure_dis_server_data_t;

#define check_rc_return_rc(rc, err)                                                             \
    do {                                                                                         \
        if ((rc) != ERRCODE_SUCC) {                                                              \
            osal_printk("CARKEY ERROR: %s fail!: call %s return 0x%x!\n", err, __FUNCTION__, rc);   \
        }                                                                                        \
    } while (0)

typedef struct {
    uint16_t conn_id;
    uint16_t slot;
    uint32_t generation;
} measure_dis_link_token_t;

int measure_dis_client_send_now(const measure_dis_link_token_t *token,
    uint32_t type, const uint8_t *data, uint32_t data_len);
errcode_t sle_imu_tx_enqueue_iq(uint16_t conn_id, const uint8_t *data, uint32_t data_len);
bool measure_dis_client_link_token(uint16_t conn_id, measure_dis_link_token_t *token);
bool measure_dis_client_imu_token(measure_dis_link_token_t *token);
bool measure_dis_client_token_ready(const measure_dis_link_token_t *token);
void sle_imu_tx_ack(const sle_imu_ack_t *ack);
int measure_dis_client_init(void);
int measure_dis_start_scan(void);
void measure_dis_client_process(void);
void sle_imu_client_start(void);

#endif
