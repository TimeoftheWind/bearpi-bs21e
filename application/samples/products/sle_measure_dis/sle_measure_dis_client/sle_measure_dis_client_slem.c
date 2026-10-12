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
#include "sle_ssap_client.h"
#include "sle_hadm_manager.h"
#include "cmsis_os2.h"
#include "sle_measure_dis_client_slem.h"

typedef struct {
    sle_channel_sounding_iq_trans_t data;
    measure_dis_link_token_t token;
    uint8_t next_idx;
    bool initialized;
} iq_assembly_t;
static iq_assembly_t g_iq_assembly[MAX_SERVERS];

uint8_t g_iq_debug_flag = 0;
void measure_dis_print_iq_data(uint8_t num, uint8_t role, sle_channel_sounding_iq_trans_t *report)
{
    if (!g_iq_debug_flag) {
        return;
    }
    osal_printk("[iq_data]%d,%d,%d,%d,0x%x,",
                num, role, report->timestamp_sn, report->samp_cnt, report->rssi);
    sle_channel_sounding_qte_trans_t *data = report->data;
    uint16_t i = 0;
    while (i < POSALG_DATA_NUM) {
        osal_printk("[%d]:0x%x,0x%x", i, data[i].i_data, data[i].q_data);
        i++;
        if (i == POSALG_DATA_NUM) {
            osal_printk(".\r\n");
        } else {
            osal_printk(";");
        }
    }
}

static errcode_t measure_dis_store_local_iq(iq_assembly_t *assembly,
    const measure_dis_link_token_t *token, sle_channel_sounding_iq_report_t *report)
{
    sle_channel_sounding_iq_trans_t *data = &assembly->data;
    if (report->report_idx == 0) {
        *data = (sle_channel_sounding_iq_trans_t){0};
        data->rssi = report->rssi[0];
        data->es_sn = report->es_sn;
        data->timestamp_sn = report->timestamp_sn;
        assembly->token = *token;
        assembly->initialized = true;
#if (defined(GLE_CS_MODE3_SUPPORT))
        data->tof_result = report->tof_result;
#endif
        assembly->next_idx = 0;
    }
    if (!assembly->initialized || assembly->token.generation != token->generation ||
        report->report_idx >= MEASURE_DIS_IQ_REPORT_CNT_MAX || report->samp_cnt > SLE_CS_IQ_REPORT_COUNT ||
        report->timestamp_sn != data->timestamp_sn || assembly->next_idx != report->report_idx ||
        (unsigned int)data->samp_cnt + report->samp_cnt > IQ_DATA_MAX) {
        return ERRCODE_INVALID_PARAM;
    }
    data->samp_cnt += report->samp_cnt;
    unsigned int offset = report->report_idx * SLE_CS_IQ_REPORT_COUNT;
    for (uint8_t i = 0; i < report->samp_cnt; i++) {
        data->data[offset + i].i_data = report->i_data[i];
        data->data[offset + i].q_data = report->q_data[i];
    }
    assembly->next_idx = report->report_idx + 1;

#if (defined(GLE_CS_MODE3_SUPPORT))
    osal_printk("local tof_result = %d.%03d \r\n", report->tof_result / MEASURE_DIS_NUM_CARRY_1000,
                report->tof_result % MEASURE_DIS_NUM_CARRY_1000);
#endif

    return ERRCODE_SUCC;
}

errcode_t measure_dis_recv_local_iq(uint16_t conn_id, sle_channel_sounding_iq_report_t *report)
{
    measure_dis_link_token_t token;
    if (report == NULL || !measure_dis_client_link_token(conn_id, &token)) {
        return ERRCODE_INVALID_PARAM;
    }
    iq_assembly_t *assembly = &g_iq_assembly[token.slot];
    errcode_t ret = ERRCODE_SUCC;

    ret = measure_dis_store_local_iq(assembly, &token, report);
    if (ret != ERRCODE_SUCC) {
        return ERRCODE_INVALID_PARAM;
    }

    if (report->report_idx + 1 == MEASURE_DIS_IQ_REPORT_CNT_MAX) {
        /* 如果是钥匙则发送给从锚点 */
        ret = sle_imu_tx_enqueue_iq(conn_id, (const uint8_t *)&assembly->data,
                                    sizeof(assembly->data));
        assembly->initialized = false;
    }
    return ret;
}

void measure_dis_read_local_cs_caps_cb(sle_channel_sounding_caps_t *caps, errcode_t status)
{
    UNUSED(caps);
    osal_printk("SLEM READ LOCAL CAPS. status:%d. \r\n", status);
}

void measure_dis_read_remote_cs_caps_cb(uint16_t conn_id, sle_channel_sounding_caps_t *caps, errcode_t status)
{
    UNUSED(caps);
    UNUSED(conn_id);
    osal_printk("SLEM READ REMOTE CAPS. status:%d. \r\n", status);
}

void measure_dis_set_cs_param_cb(uint16_t conn_id, errcode_t status)
{
    UNUSED(conn_id);
    osal_printk("SLEM SET PARAM. status:%d. \r\n", status);
}

void measure_dis_cs_state_changed_cb(uint8_t slem_status, errcode_t status)
{
    unused(slem_status);
    osal_printk("SLEM STATE CHANGED. status:%d. \r\n", status);
}

void measure_dis_cs_iq_report_cb(uint16_t conn_id, sle_channel_sounding_iq_report_t *report)
{
    (void)measure_dis_recv_local_iq(conn_id, report);
}

errcode_t measure_dis_reg_callbacks(void)
{
    sle_hadm_callbacks_t scd_cbks = {0};

    scd_cbks.read_local_cs_caps_cb = measure_dis_read_local_cs_caps_cb;
    scd_cbks.read_remote_cs_caps_cb = measure_dis_read_remote_cs_caps_cb;
    scd_cbks.cs_state_changed_cb = measure_dis_cs_state_changed_cb;
    scd_cbks.cs_iq_report_cb = measure_dis_cs_iq_report_cb;

    errcode_t ret = sle_hadm_register_callbacks(&scd_cbks);
    osal_printk("SCD SET CALLBACK DONE. ret = %x.\r\n", ret);
    return ret;
}
