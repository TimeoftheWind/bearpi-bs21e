/**
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2023-2023. All rights reserved.
 *
 * Description: sle adv config for sle uart server. \n
 *
 * History: \n
 * 2023-07-17, Create file. \n
 */
#include "sle_measure_dis_server.h"
#include "sle_common.h"
#include "slem_smooth.h"
#include "slem_alg_smooth_dis.h"
#include "tcxo.h"
#include "sle_measure_dis_server_alg.h"

#define TOF_DEFAULT 2070
#define DIS_ALG_MODE 4
#define DIS_ALG_RSSI_LIMIT (-120)
#define DIS_ALG_THRESHOULD_COND2 20
#define DIS_ALG_R_START 2

uint8_t g_iq_debug_flag = 0;
uint32_t counter = 0;
static uint32_t g_last_distance_calc_ms;

#define MEASURE_DIS_CALC_MIN_INTERVAL_MS 400U
#define IQ_SLOT_COUNT 2U

static sle_channel_sounding_iq_report_t g_local_iq_slots[IQ_SLOT_COUNT];
static sle_channel_sounding_iq_trans_t g_remote_iq_slots[IQ_SLOT_COUNT];
static uint8_t g_local_iq_slot_busy[IQ_SLOT_COUNT];
static uint8_t g_remote_iq_slot_busy[IQ_SLOT_COUNT];
static uint32_t g_local_iq_dropped;
static uint32_t g_remote_iq_dropped;

static int32_t iq_slot_claim(uint8_t *busy)
{
    uint32_t irq_status = osal_irq_lock();
    for (uint32_t i = 0; i < IQ_SLOT_COUNT; i++) {
        if (busy[i] == 0U) {
            busy[i] = 1U;
            osal_irq_restore(irq_status);
            return (int32_t)i;
        }
    }
    osal_irq_restore(irq_status);
    return -1;
}

static void iq_slot_release(uint8_t *busy, uint32_t slot)
{
    uint32_t irq_status = osal_irq_lock();
    busy[slot] = 0U;
    osal_irq_restore(irq_status);
}

measure_dis_stored_iq_data_t g_md_local_iq_data;
uint8_t g_md_store_local_iq_complete;
uint8_t g_next_idx;

void slem_posalg_set_base_para(slem_alg_para_dis *alg_para, uint8_t key_id)
{
    unused(key_id);
    alg_para->calib_val = 0;
    alg_para->tof_calib = TOF_DEFAULT;
    alg_para->ranging_method = DIS_ALG_MODE;
}

void measure_dis_print_cal_dis(float dist_first, float dist_ori, float prob, uint32_t time)
{
    unused(dist_ori);
    unused(prob);
    int32_t distance_mm = (int32_t)(dist_first * MEASURE_DIS_NUM_CARRY_1000 +
        ((dist_first >= 0.0f) ? 0.5f : -0.5f));
    uint32_t magnitude_mm = (distance_mm < 0) ? (uint32_t)(-distance_mm) : (uint32_t)distance_mm;
    osal_printk(
        "DIST,%llu,%s%u.%03u,%u\r\n",
        (unsigned long long)uapi_tcxo_get_us(), (distance_mm < 0) ? "-" : "",
        magnitude_mm / MEASURE_DIS_NUM_CARRY_1000, magnitude_mm % MEASURE_DIS_NUM_CARRY_1000, time);
}

static void measure_dis_posalg_set_distance(slem_smoothed_dis_result *dis,
    measure_dis_profile_msg_dis_t *measure_dis_temp)
{
    measure_dis_temp->dist_first = dis->dis_smoothed;
    measure_dis_temp->dist_second = dis->dis_ori;
    measure_dis_temp->dist_double = dis->dis_slight_smoothed;
    measure_dis_temp->high = dis->height;
    measure_dis_temp->prob = dis->prob;
    measure_dis_temp->rssi = dis->rssi;
    measure_dis_temp->smooth_num = dis->smooth_num;
}

void measure_dis_posalg_set_base_para(slem_alg_para_dis *alg_para)
{
    alg_para->calib_val = 0;
    alg_para->tof_calib = TOF_DEFAULT;
    alg_para->ranging_method = DIS_ALG_MODE;
}

void measure_dis_posalg_get_distance(measure_dis_profile_msg_dis_t *measure_dis_temp,
                                     sle_channel_sounding_iq_trans_t *remote_iq_data)
{
    slem_alg_para_dis alg_para = {0};

    slem_posalg_set_base_para(&alg_para, 0);
    alg_para.rssi_rtd = g_md_local_iq_data.rssi;
    alg_para.rssi_dut = remote_iq_data[0].rssi;
#if (defined(GLE_CS_MODE3_SUPPORT))
    alg_para.tof_rtd = g_md_local_iq_data.tof_result;
    alg_para.tof_dut = remote_iq_data[0].tof_result;
#endif
    alg_para.iq_rtd = (slem_alg_iq *)&(g_md_local_iq_data.data[0]);
    alg_para.iq_dut = (slem_alg_iq *)&(remote_iq_data[0].data[0]);
    g_md_store_local_iq_complete = false;

    slem_para_pair para_limit = {.rssi_limit = DIS_ALG_RSSI_LIMIT,
                                 .threshold_cond2 = DIS_ALG_THRESHOULD_COND2,
                                 .r_start = DIS_ALG_R_START};
    alg_para.para_limit = para_limit;
    alg_para.key_id = 0;
    /* remote iq 是 dut， local iq 是 rtd */
    uint32_t time_start = (uint32_t)uapi_tcxo_get_ms();

    alg_para.cur_count = counter;
    alg_para.cur_time = time_start;
    slem_smoothed_dis_result result_dist;
    slem_alg_calc_smoothed_dis(&result_dist, &alg_para);
    counter++;
    uint32_t time_end = (uint32_t)uapi_tcxo_get_ms();
    measure_dis_print_cal_dis(result_dist.dis_smoothed, result_dist.dis_ori,
                              result_dist.prob, time_end - time_start);

    measure_dis_posalg_set_distance(&result_dist, measure_dis_temp);
}

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

errcode_t measure_dis_store_local_iq(sle_channel_sounding_iq_report_t *report)
{
    if (report->report_idx == 0) {
        g_md_local_iq_data.samp_cnt = 0;
        g_md_local_iq_data.rssi = report->rssi[0];
        g_md_local_iq_data.es_sn = report->es_sn;
        g_md_local_iq_data.timestamp_sn = report->timestamp_sn;
#if (defined(GLE_CS_MODE3_SUPPORT))
        g_md_local_iq_data.tof_result = report->tof_result;
#endif
        g_next_idx = 0;
    }
    if ((report->report_idx >= MEASURE_DIS_IQ_REPORT_CNT_MAX ||
         report->timestamp_sn != g_md_local_iq_data.timestamp_sn) ||
        (g_next_idx != report->report_idx)) {
        osal_printk("MEASURE_DIS STORE LOCAL IQ FAIL. report_idx:%d, timestamp:%d, g_next_idx:%d. \r\n",
                    report->report_idx, report->timestamp_sn, g_next_idx);
        return ERRCODE_INVALID_PARAM;
    }
    g_md_local_iq_data.samp_cnt += report->samp_cnt;
    uint8_t offset = report->report_idx * SLE_CS_IQ_REPORT_COUNT;
    for (uint8_t i = 0; i < report->samp_cnt; i++) {
        g_md_local_iq_data.data[offset + i].i_data = report->i_data[i];
        g_md_local_iq_data.data[offset + i].q_data = report->q_data[i];
    }
    g_next_idx = report->report_idx + 1;

#if (defined(GLE_CS_MODE3_SUPPORT))
    osal_printk("local tof_result = %d.%03d \r\n", report->tof_result / MEASURE_DIS_NUM_CARRY_1000,
                report->tof_result % MEASURE_DIS_NUM_CARRY_1000);
#endif

    return ERRCODE_SUCC;
}

errcode_t measure_dis_proc_local_iq(uint16_t conn_id, sle_channel_sounding_iq_report_t *report)
{
    unused(conn_id);
    errcode_t ret = ERRCODE_SUCC;

    ret = measure_dis_store_local_iq(report);
    if (ret != ERRCODE_SUCC) {
        return ERRCODE_INVALID_PARAM;
    }

    if (report->report_idx + 1 == MEASURE_DIS_IQ_REPORT_CNT_MAX) {
        g_md_store_local_iq_complete = true;
        /* 如果是钥匙则发送给从锚点 */
        measure_dis_print_iq_data(0, 0, (sle_channel_sounding_iq_trans_t *)(&g_md_local_iq_data));
    }
    return ret;
}

errcode_t measure_dis_proc_remote_iq(uint16_t conn_id, sle_channel_sounding_iq_trans_t *report)
{
    unused(conn_id);
    uint8_t num = 0; /* SLAVE只有一路MEASURE_DIS业务 */
    unused(num);
    measure_dis_print_iq_data(0, 1, report);
    if (abs((int32_t)(g_md_local_iq_data.timestamp_sn - report->timestamp_sn)) >= MEASURE_DIS_TIMESTAMP_DIFF_MAX) {
        osal_printk("local:%d, remote:%d. \r\n",
                    g_md_local_iq_data.timestamp_sn, report->timestamp_sn);
        return ERRCODE_INVALID_PARAM;
    }

    if (g_md_store_local_iq_complete) {
        uint32_t now_ms = (uint32_t)uapi_tcxo_get_ms();
        if ((g_last_distance_calc_ms == 0U) ||
            ((uint32_t)(now_ms - g_last_distance_calc_ms) >= MEASURE_DIS_CALC_MIN_INTERVAL_MS)) {
            g_md_store_local_iq_complete = false;
            g_last_distance_calc_ms = now_ms;
            measure_dis_profile_msg_dis_t measure_dis_dis_temp = { report->timestamp_sn, 0, 0, 0, 0, 0, 0, 0};
            measure_dis_posalg_get_distance(&measure_dis_dis_temp, (sle_channel_sounding_iq_trans_t *)report);
        }
    }

    return ERRCODE_SUCC;
}

void measure_dis_read_local_cs_caps_cb(sle_channel_sounding_caps_t *caps, errcode_t status)
{
    unused(caps);
    osal_printk("SLEM READ LOCAL CAPS. status:%d. \r\n", status);
}

void measure_dis_read_remote_cs_caps_cb(uint16_t conn_id, sle_channel_sounding_caps_t *caps, errcode_t status)
{
    unused(caps);
    unused(conn_id);
    osal_printk("SLEM READ REMOTE CAPS. status:%d. \r\n", status);
}

void measure_dis_set_cs_param_cb(uint16_t conn_id, errcode_t status)
{
    unused(conn_id);
    osal_printk("SLEM SET PARAM. status:%d. \r\n", status);
}

void measure_dis_cs_state_changed_cb(uint8_t slem_status, errcode_t status)
{
    unused(slem_status);
    osal_printk("SLEM STATE CHANGED. status:%d. \r\n", status);
}

void measure_dis_cs_iq_report_cb(uint16_t conn_id, sle_channel_sounding_iq_report_t *report)
{
    if (report == NULL) {
        return;
    }
    int32_t slot = iq_slot_claim(g_local_iq_slot_busy);
    if (slot < 0) {
        g_local_iq_dropped++;
        osal_printk("IQ_DROP,local,%u\r\n", g_local_iq_dropped);
        return;
    }
    if (memcpy_s(&g_local_iq_slots[slot], sizeof(g_local_iq_slots[slot]),
                 report, sizeof(*report)) != EOK) {
        iq_slot_release(g_local_iq_slot_busy, (uint32_t)slot);
        return;
    }
    measure_dis_msg_node_t msg_node = {conn_id, SLEM_MSG_LOCAL_IQ, (uint16_t)slot,
        &g_local_iq_slots[slot]};
    if (sle_measure_dis_msg_add(&msg_node) != ERRCODE_SUCC) {
        iq_slot_release(g_local_iq_slot_busy, (uint32_t)slot);
    }
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

errcode_t measure_dis_recv_local_iq(measure_dis_msg_node_t *msg_node)
{
    /* key send to anchor */
    errcode_t ret = measure_dis_proc_local_iq(msg_node->conn_id, (sle_channel_sounding_iq_report_t *)(msg_node->data));
    iq_slot_release(g_local_iq_slot_busy, msg_node->result);
    return ret;
}

errcode_t measure_dis_recv_remote_iq(measure_dis_msg_node_t *msg_node)
{
    errcode_t ret = measure_dis_proc_remote_iq(msg_node->conn_id, (sle_channel_sounding_iq_trans_t *)(msg_node->data));
    iq_slot_release(g_remote_iq_slot_busy, msg_node->result);
    return ret;
}

errcode_t measure_dis_match_msg(measure_dis_msg_node_t *msg_node)
{
    errcode_t ret = ERRCODE_FAIL;
    switch (msg_node->type) {
        case SLEM_MSG_LOCAL_IQ:
            ret = measure_dis_recv_local_iq(msg_node);
            break;
        case SLEM_MSG_REMOTE_IQ:
            ret = measure_dis_recv_remote_iq(msg_node);
            break;
        default:
            break;
    }
    return ret;
}

errcode_t measure_dis_remote_iq(uint16_t len, uint8_t *value)
{
    if ((value == NULL) || (len != sizeof(sle_channel_sounding_iq_trans_t))) {
        return ERRCODE_INVALID_PARAM;
    }
    int32_t slot = iq_slot_claim(g_remote_iq_slot_busy);
    if (slot < 0) {
        g_remote_iq_dropped++;
        osal_printk("IQ_DROP,remote,%u\r\n", g_remote_iq_dropped);
        return ERRCODE_FAIL;
    }
    if (memcpy_s(&g_remote_iq_slots[slot], sizeof(g_remote_iq_slots[slot]), value, len) != EOK) {
        iq_slot_release(g_remote_iq_slot_busy, (uint32_t)slot);
        return ERRCODE_MEMCPY;
    }
    measure_dis_msg_node_t msg_node = {g_measure_dis_conn_id, SLEM_MSG_REMOTE_IQ, (uint16_t)slot,
        &g_remote_iq_slots[slot]};
    errcode_t ret = sle_measure_dis_msg_add(&msg_node);
    if (ret != ERRCODE_SUCC) {
        iq_slot_release(g_remote_iq_slot_busy, (uint32_t)slot);
    }

    return ret;
}
