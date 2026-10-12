/* Execute the production server queue/dedup/output paths, without hardware. */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../application/samples/products/sle_measure_dis/sle_measure_dis_server/sle_measure_dis_server.c"

static imu_output_node_t captured;
static bool queued, full;
static unsigned int writes, csv_lines, notifications;
static sle_imu_ack_t notified;
errcode_t measure_dis_remote_iq(uint16_t len, uint8_t *value)
{ (void)len; (void)value; assert(false); return ERRCODE_FAIL; }
unsigned int osal_irq_lock(void) { return 0; }
void osal_irq_restore(unsigned int state) { (void)state; }
int osal_event_write(osal_event *event, unsigned int mask)
{ (void)event; (void)mask; return 0; }
void osal_printk(const char *fmt, ...)
{ if (strcmp(fmt, "%s") == 0) { csv_lines++; } }
void *osal_kmalloc(unsigned long size, unsigned int flag)
{ (void)flag; return malloc(size); }
void osal_kfree(void *ptr) { free(ptr); }
errno_t memcpy_s(void *dst, size_t cap, const void *src, size_t len)
{ if (len > cap) { return ERANGE; } memcpy(dst, src, len); return EOK; }
int osal_msg_queue_write_copy(unsigned long id, void *buf, unsigned int size, unsigned int timeout)
{
    (void)timeout;
    assert(id == 1 && size == sizeof(captured));
    if (full || queued) { return -1; }
    memcpy(&captured, buf, size); queued = true; writes++; return 0;
}
int osal_msg_queue_read_copy(unsigned long id, void *buf, unsigned int *size, unsigned int timeout)
{
    (void)timeout;
    if (id != 1 || !queued) { return -1; }
    assert(*size >= sizeof(captured));
    memcpy(buf, &captured, sizeof(captured)); *size = sizeof(captured); queued = false; return 0;
}
unsigned int osal_msg_queue_get_msg_num(unsigned long id) { return id == 1 && queued; }
errcode_t ssaps_notify_indicate_by_uuid(uint8_t server, uint16_t conn, ssaps_ntf_ind_by_uuid_t *param)
{
    (void)server; assert(conn == 0);
    const measure_ids_msg_t *msg = (const void *)param->value;
    assert(msg->type == SLEM_PROFILE_MSG_IMU_ACK && msg->len == sizeof(notified));
    memcpy(&notified, msg->data, sizeof(notified)); notifications++; return 0;
}
int main(void)
{
    g_imu_output_queue = 1; g_distance_output_queue = 2; g_measure_dis_conn_id = 0;
    sle_imu_batch_t batch = {.sequence = 10, .sample_count = SLE_IMU_BATCH_SAMPLES};
    for (unsigned int i = 0; i < SLE_IMU_BATCH_SAMPLES; i++) {
        batch.samples[i].sequence = i; batch.samples[i].timestamp_us = 50000ULL * i;
    }
    union { uint64_t alignment; uint8_t bytes[sizeof(measure_ids_msg_t) + sizeof(batch)]; } wire;
    measure_ids_msg_t *msg = (void *)wire.bytes;
    msg->type = SLEM_PROFILE_MSG_IMU_BATCH; msg->len = sizeof(batch);
    memcpy(msg->data, &batch, sizeof(batch));
    measure_dis_server_msg_proc(wire.bytes, sizeof(wire.bytes) - 1);
    assert(writes == 0); /* Reject truncated envelope. */
    measure_dis_server_msg_proc(wire.bytes, sizeof(wire.bytes));
    assert(writes == (MEASURE_SERVER_SUFFIX == IMU_SERVER_SUFFIX ? 1U : 0U));
    if (queued) { sle_measure_dis_telemetry_output_proc(); }
    queued = false; writes = csv_lines = notifications = 0;
    g_last_imu_valid = g_last_imu_written = g_imu_ack_pending = false;
    assert(sle_measure_dis_imu_output_add(&batch) == 0 && writes == 1);
    assert(!g_last_imu_written && !g_imu_ack_pending);
    assert(sle_measure_dis_imu_output_add(&batch) == 0 && writes == 1);
    sle_imu_batch_t next = batch; next.sequence++;
    assert(sle_measure_dis_imu_output_add(&next) != 0); /* Busy, no silent eviction. */
    sle_measure_dis_telemetry_output_proc();
    assert(csv_lines == 4 && notifications == 1 && g_last_imu_written);
    assert(notified.sequence == 10 && notified.last_timestamp_us == 150000);
    assert(sle_measure_dis_imu_output_add(&batch) == 0 && writes == 1);
    sle_measure_dis_telemetry_output_proc(); /* Lost ACK: resend ACK, not CSV. */
    assert(csv_lines == 4 && notifications == 2);
    full = true;
    assert(sle_measure_dis_imu_output_add(&next) != 0 && !g_last_imu_valid);
    full = false;
    assert(sle_measure_dis_imu_output_add(&next) == 0 && writes == 2);
    sle_measure_dis_telemetry_output_proc();
    assert(csv_lines == 8 && notifications == 3);
    batch.sample_count = 0;
    assert(sle_measure_dis_imu_output_add(&batch) != 0);
    batch.sample_count = SLE_IMU_BATCH_SAMPLES + 1;
    assert(sle_measure_dis_imu_output_add(&batch) != 0);
    char text[24];
    assert(imu_float_text(text, sizeof(text), -1.25f, 3) && strcmp(text, "-1.250") == 0);
    assert(!imu_float_text(text, sizeof(text), __builtin_nanf(""), 3));
    printf("PASS: server%u role filter, malformed frame, queue backpressure, post-output ACK, dedup/retry\n",
        (unsigned int)MEASURE_SERVER_SUFFIX);
    return 0;
}
