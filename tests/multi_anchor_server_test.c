/* Execute the production server queue/dedup/output paths, without hardware. */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include "../application/samples/products/sle_measure_dis/sle_measure_dis_server/sle_measure_dis_server.c"

static imu_output_node_t output[IMU_OUTPUT_QUEUE_SIZE];
static imu_ack_node_t ack_queue[IMU_ACK_QUEUE_SIZE];
static unsigned int output_head, output_count, ack_head, ack_count;
static bool full;
static unsigned int writes, csv_lines, notifications;
static sle_imu_ack_t notified;
int snprintf_s(char *out, size_t size, size_t count, const char *fmt, ...)
{
    assert(count < size);
    va_list args; va_start(args, fmt);
    int ret = vsnprintf(out, size, fmt, args); va_end(args);
    return ret < 0 || (size_t)ret > count ? -1 : ret;
}
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
    if (id == 3) {
        assert(size == sizeof(imu_ack_node_t));
        if (ack_count == IMU_ACK_QUEUE_SIZE) { return -1; }
        memcpy(&ack_queue[(ack_head + ack_count++) % IMU_ACK_QUEUE_SIZE], buf, size); return 0;
    }
    assert(id == 1 && size == sizeof(imu_output_node_t));
    if (full || output_count == IMU_OUTPUT_QUEUE_SIZE) { return -1; }
    memcpy(&output[(output_head + output_count++) % IMU_OUTPUT_QUEUE_SIZE], buf, size); writes++; return 0;
}
int osal_msg_queue_read_copy(unsigned long id, void *buf, unsigned int *size, unsigned int timeout)
{
    (void)timeout;
    if (id == 3) {
        if (!ack_count) { return -1; }
        assert(*size >= sizeof(imu_ack_node_t));
        memcpy(buf, &ack_queue[ack_head++ % IMU_ACK_QUEUE_SIZE], sizeof(imu_ack_node_t));
        *size = sizeof(imu_ack_node_t); ack_count--; return 0;
    }
    if (id != 1 || !output_count) { return -1; }
    assert(*size >= sizeof(imu_output_node_t));
    memcpy(buf, &output[output_head++ % IMU_OUTPUT_QUEUE_SIZE], sizeof(imu_output_node_t));
    *size = sizeof(imu_output_node_t); output_count--; return 0;
}
unsigned int osal_msg_queue_get_msg_num(unsigned long id) { return id == 1 ? output_count : 0; }
errcode_t ssaps_notify_indicate_by_uuid(uint8_t server, uint16_t conn, ssaps_ntf_ind_by_uuid_t *param)
{
    (void)server; assert(conn == 0);
    const measure_ids_msg_t *msg = (const void *)param->value;
    assert(msg->type == SLEM_PROFILE_MSG_IMU_ACK && msg->len == sizeof(notified));
    memcpy(&notified, msg->data, sizeof(notified)); notifications++; return 0;
}
int main(void)
{
    g_imu_output_queue = 1; g_distance_output_queue = 2; g_imu_ack_queue = 3; g_measure_dis_conn_id = 0;
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
    sle_measure_dis_telemetry_output_proc(); sle_measure_dis_imu_ack_proc();
    output_count = ack_count = 0; writes = csv_lines = notifications = 0;
    g_imu_history_count = g_imu_history_next = 0;
    assert(sle_measure_dis_imu_output_add(&batch) == 0 && writes == 1);
    sle_measure_dis_imu_ack_proc();
    assert(notifications == 1 && csv_lines == 0); /* ACK before independent CSV output. */
    assert(sle_measure_dis_imu_output_add(&batch) == 0 && writes == 1);
    sle_imu_batch_t next = batch; next.sequence++;
    assert(sle_measure_dis_imu_output_add(&next) == 0 && writes == 2); /* Pipeline admission. */
    sle_measure_dis_imu_ack_proc();
    assert(notifications == 3 && csv_lines == 0);
    sle_measure_dis_telemetry_output_proc();
    assert(csv_lines == 2 * SLE_IMU_BATCH_SAMPLES);
    assert(notified.sequence == 11 && notified.last_timestamp_us == 250000);
    assert(sle_measure_dis_imu_output_add(&batch) == 0 && writes == 2);
    sle_measure_dis_imu_ack_proc(); /* Older duplicate: resend ACK, not CSV. */
    assert(csv_lines == 2 * SLE_IMU_BATCH_SAMPLES && notifications == 4);
    next.sequence++;
    full = true;
    unsigned int old_history = g_imu_history_count;
    assert(sle_measure_dis_imu_output_add(&next) != 0 && g_imu_history_count == old_history);
    sle_measure_dis_imu_ack_proc(); assert(notifications == 4); /* No ACK on full. */
    full = false;
    assert(sle_measure_dis_imu_output_add(&next) == 0 && writes == 3);
    g_server_link_generation++; /* Queued ACK must not leak to a reused connection. */
    sle_measure_dis_imu_ack_proc(); assert(notifications == 4);
    assert(sle_measure_dis_imu_output_add(&next) == 0 && writes == 3);
    sle_measure_dis_imu_ack_proc(); assert(notifications == 5);
    sle_measure_dis_telemetry_output_proc();
    assert(csv_lines == 3 * SLE_IMU_BATCH_SAMPLES);
    batch.sample_count = 0;
    assert(sle_measure_dis_imu_output_add(&batch) != 0);
    batch.sample_count = SLE_IMU_BATCH_SAMPLES + 1;
    assert(sle_measure_dis_imu_output_add(&batch) != 0);
    char text[24];
    assert(imu_float_text(text, sizeof(text), -1.25f, 3) && strcmp(text, "-1.250") == 0);
    assert(!imu_float_text(text, sizeof(text), __builtin_nanf(""), 3));
    /* Lost/full ACK queue must not duplicate CSV or forget accepted batches. */
    unsigned int before_writes = writes;
    for (unsigned int i = 0; i < IMU_ACK_QUEUE_SIZE + 3; i++) {
        assert(sle_measure_dis_imu_output_add(&next) == 0);
    }
    assert(writes == before_writes && ack_count == IMU_ACK_QUEUE_SIZE);
    sle_measure_dis_imu_ack_proc();
    assert(sle_measure_dis_imu_output_add(&next) == 0 && ack_count == 1 && writes == before_writes);
    sle_measure_dis_imu_ack_proc();
    /* Real output queue capacity: full batch is not accepted or cached/ACKed. */
    for (unsigned int i = 0; i < IMU_OUTPUT_QUEUE_SIZE; i++) {
        next.sequence = 20 + i;
        assert(sle_measure_dis_imu_output_add(&next) == 0);
    }
    next.sequence++;
    unsigned int before_acks = ack_count;
    old_history = g_imu_history_count;
    assert(sle_measure_dis_imu_output_add(&next) != 0);
    assert(ack_count == before_acks && g_imu_history_count == old_history);
    sle_measure_dis_telemetry_output_proc();
    assert(sle_measure_dis_imu_output_add(&next) == 0);
    sle_measure_dis_imu_ack_proc(); sle_measure_dis_telemetry_output_proc();
    printf("PASS: server%u frame/role filter, pipeline admission ACK, backpressure, historical dedup/reconnect\n",
        (unsigned int)MEASURE_SERVER_SUFFIX);
    return 0;
}
