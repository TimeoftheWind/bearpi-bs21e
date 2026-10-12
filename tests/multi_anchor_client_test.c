/* Exercise actual production functions with SDK headers and host OSAL mocks. */
#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "../application/samples/products/sle_measure_dis/sle_measure_dis_client/sle_measure_dis_client.c"
#include "../application/samples/products/sle_measure_dis/sle_measure_dis_client/sle_measure_dis_client_slem.c"
#include "../application/samples/products/sle_measure_dis/imu/imu_client.c"

static uint64_t fake_us;
static iq_tx_node_t captured_iq;
static unsigned int queue_writes;
static bool queue_full;
static uint16_t sent_conn, sent_handle;
errcode_t ssapc_write_cmd(uint8_t client, uint16_t conn, ssapc_write_param_t *param)
{
    (void)client; sent_conn = conn; sent_handle = param->handle;
    assert(param->data_len <= 512); return ERRCODE_SUCC;
}
unsigned int osal_irq_lock(void) { return 0; }
void osal_irq_restore(unsigned int state) { (void)state; }
int osal_mutex_lock(osal_mutex *m) { (void)m; return 0; }
void osal_mutex_unlock(osal_mutex *m) { (void)m; }
int osal_event_write(osal_event *e, unsigned int mask) { (void)e; (void)mask; return 0; }
uint64_t uapi_tcxo_get_us(void) { return fake_us; }
void osal_printk(const char *fmt, ...) { (void)fmt; }
errno_t memcpy_s(void *dst, size_t cap, const void *src, size_t len)
{
    if (len > cap) { return ERANGE; }
    memcpy(dst, src, len);
    return EOK;
}
int osal_msg_queue_write_copy(unsigned long id, void *buf, unsigned int size, unsigned int timeout)
{
    (void)id; (void)timeout;
    if (queue_full) { return -1; }
    assert(size == sizeof(captured_iq));
    memcpy(&captured_iq, buf, size);
    queue_writes++;
    return 0;
}

int main(void)
{
    sle_addr_t a = {.addr = {1, 1, 1, 1, 1, 5}};
    measure_dis_link_token_t token, old;
    assert(anchor_slot(&a) == 0);
    a.addr[5] = 9;
    assert(anchor_slot(&a) == 4);
    a.addr[5] = 1;
    assert(anchor_slot(&a) == -1);
    a.addr[5] = 5;
    measure_dis_cm_conn_state_cbk(8, &a, SLE_ACB_STATE_CONNECTED, 0, 0);
    assert(measure_dis_client_link_token(8, &token));
    assert(token.slot == 0);
    g_links[0].state = LINK_READY;
    g_links[0].property_handle = 17;
    assert(measure_dis_client_imu_token(&old) && old.conn_id == 8);
    g_connect_addr.addr[5] = 9; g_connect_pending = true;
    measure_dis_cm_conn_state_cbk(8, &a, SLE_ACB_STATE_DISCONNECTED, 0, 0);
    assert(g_connect_pending); /* Dropping anchor5 does not cancel anchor9 setup. */
    g_connect_pending = false;
    measure_dis_cm_conn_state_cbk(8, &a, SLE_ACB_STATE_CONNECTED, 0, 0);
    g_links[0].state = LINK_READY;
    assert(!measure_dis_client_token_ready(&old)); /* Same conn_id, new generation. */

    a.addr[5] = 7;
    measure_dis_cm_conn_state_cbk(2, &a, SLE_ACB_STATE_CONNECTED, 0, 0);
    g_links[0].mtu = 512; g_links[0].property_handle = 17;
    g_links[2].state = LINK_READY; g_links[2].mtu = 512; g_links[2].property_handle = 19;
    uint8_t payload[16] = {0};
    assert(measure_dis_client_link_token(2, &token));
    assert(measure_dis_client_send_now(&token, SLEM_PROFILE_MSG_IQ, payload, sizeof(payload)) == 0);
    assert(sent_conn == 2 && sent_handle == 19);
    assert(measure_dis_client_imu_token(&token) && token.conn_id == 8);
    assert(measure_dis_client_send_now(&token, SLEM_PROFILE_MSG_IMU_BATCH, payload, sizeof(payload)) == 0);
    assert(sent_conn == 8 && sent_handle == 17);
    g_started = true;
    sle_channel_sounding_iq_report_t report = {0};
    report.samp_cnt = 1;
    report.timestamp_sn = 100;
    report.i_data[0] = 111;
    report.q_data[0] = 222;
    assert(measure_dis_recv_local_iq(8, &report) == ERRCODE_SUCC);
    assert(captured_iq.token.slot == 0 && captured_iq.token.conn_id == 8);
    assert(captured_iq.len == sizeof(sle_channel_sounding_iq_trans_t));
    const sle_channel_sounding_iq_trans_t *wire = (const void *)captured_iq.data;
    assert(wire->data[0].i_data == 111 && wire->tof_result == 0);
    report.timestamp_sn = 200;
    report.i_data[0] = 333;
    assert(measure_dis_recv_local_iq(2, &report) == ERRCODE_SUCC);
    assert(captured_iq.token.slot == 2 && captured_iq.token.conn_id == 2);
    assert(g_iq_assembly[0].data.timestamp_sn == 100);
    assert(g_iq_assembly[2].data.timestamp_sn == 200);
    report.samp_cnt = SLE_CS_IQ_REPORT_COUNT + 1;
    unsigned int before = queue_writes;
    assert(measure_dis_recv_local_iq(2, &report) != ERRCODE_SUCC);
    assert(queue_writes == before);
    report.samp_cnt = 1;
    queue_full = true;
    assert(measure_dis_recv_local_iq(2, &report) != ERRCODE_SUCC);
    assert(g_iq_queue_drops == 1);

    sle_imu_packet_t sample = {0};
    for (unsigned int i = 0; i < SLE_IMU_RING_CAPACITY + 4; i++) {
        sample.sequence = i;
        sample.timestamp_us = i * 50000ULL;
        sample_push(&sample);
    }
    sle_imu_batch_t batch;
    assert(sample_batch(&batch));
    assert(batch.samples[0].sequence == 4 && batch.samples[3].sequence == 7);
    assert(batch.dropped_samples == 4 && g_sample_count == SLE_IMU_RING_CAPACITY - 4);
    sle_imu_ack_t ack = {.sequence = batch.sequence,
        .last_timestamp_us = batch.samples[3].timestamp_us + 1};
    sle_imu_tx_ack(&ack);
    assert(!batch_acked(&batch));
    ack.last_timestamp_us--;
    sle_imu_tx_ack(&ack);
    assert(batch_acked(&batch));
    assert(!batch_acked(&batch));
    assert(sizeof(sle_imu_batch_t) + sizeof(measure_ids_msg_t) <= sizeof(g_sle_tx_buf));
    printf("PASS: address routing, reconnect generation, per-link IQ, bounds, queue overflow, sample ring, ACK identity\n");
    return 0;
}
