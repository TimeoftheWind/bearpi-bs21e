#include "soc_osal.h"
#include "tcxo.h"
#include "sle_measure_dis_client.h"
#include "sle_measure_dis_client_slem.h"
#include "imu_sensor.h"

#define IMU_TASK_STACK 0x2000
#define IMU_TX_TASK_STACK 0x1800
#define IMU_TASK_PRIORITY 26
#define IMU_TX_TASK_PRIORITY 28
#define IMU_SAMPLE_PERIOD_US 50000ULL
#define IMU_TX_EVENT 1U

typedef struct {
    measure_dis_link_token_t token;
    uint64_t enqueued_us;
    uint32_t len;
    uint8_t data[sizeof(sle_channel_sounding_iq_trans_t)];
} iq_tx_node_t;

static sle_imu_packet_t g_samples[SLE_IMU_RING_CAPACITY];
static uint16_t g_sample_head;
static uint16_t g_sample_count;
static uint16_t g_dropped;
static osal_mutex g_sample_lock;
static osal_event g_tx_event;
static unsigned long g_iq_queue;
static bool g_started;
static uint32_t g_batch_sequence;
static uint32_t g_sample_sequence;
static uint32_t g_iq_queue_drops;
static uint32_t g_iq_expired;
static uint32_t g_imu_acked;
static uint32_t g_imu_expired;
static sle_imu_ack_t g_ack;
static bool g_ack_pending;

static void sample_push(const sle_imu_packet_t *sample)
{
    (void)osal_mutex_lock(&g_sample_lock);
    if (g_sample_count == SLE_IMU_RING_CAPACITY) {
        g_sample_head = (uint16_t)((g_sample_head + 1U) % SLE_IMU_RING_CAPACITY);
        g_sample_count--;
        if (g_dropped != UINT16_MAX) {
            g_dropped++;
        }
    }
    unsigned int tail = (g_sample_head + g_sample_count) % SLE_IMU_RING_CAPACITY;
    g_samples[tail] = *sample;
    g_sample_count++;
    osal_mutex_unlock(&g_sample_lock);
    (void)osal_event_write(&g_tx_event, IMU_TX_EVENT);
}

static bool sample_batch(sle_imu_batch_t *batch)
{
    bool ready = false;
    (void)osal_mutex_lock(&g_sample_lock);
    if (g_sample_count >= SLE_IMU_BATCH_SAMPLES) {
        batch->sequence = g_batch_sequence++;
        batch->sample_count = SLE_IMU_BATCH_SAMPLES;
        batch->dropped_samples = g_dropped;
        g_dropped = 0;
        for (unsigned int i = 0; i < SLE_IMU_BATCH_SAMPLES; i++) {
            batch->samples[i] = g_samples[g_sample_head];
            g_sample_head = (uint16_t)((g_sample_head + 1U) % SLE_IMU_RING_CAPACITY);
        }
        g_sample_count -= SLE_IMU_BATCH_SAMPLES;
        ready = true;
    }
    osal_mutex_unlock(&g_sample_lock);
    return ready;
}

/* bt_service callback: bounded queue copy, no allocation or blocking send. */
errcode_t sle_imu_tx_enqueue_iq(uint16_t conn_id, const uint8_t *data, uint32_t len)
{
    iq_tx_node_t node;
    if (!g_started || data == NULL || len > sizeof(node.data) ||
        !measure_dis_client_link_token(conn_id, &node.token)) {
        return ERRCODE_INVALID_PARAM;
    }
    node.enqueued_us = uapi_tcxo_get_us();
    node.len = len;
    if (memcpy_s(node.data, sizeof(node.data), data, len) != EOK) {
        return ERRCODE_MEMCPY;
    }
    if (osal_msg_queue_write_copy(g_iq_queue, &node, sizeof(node), 0) != OSAL_SUCCESS) {
        g_iq_queue_drops++;
        return ERRCODE_FAIL;
    }
    (void)osal_event_write(&g_tx_event, IMU_TX_EVENT);
    return ERRCODE_SUCC;
}

void sle_imu_tx_ack(const sle_imu_ack_t *ack)
{
    unsigned int irq = osal_irq_lock();
    g_ack = *ack;
    g_ack_pending = true;
    osal_irq_restore(irq);
}

static bool batch_acked(const sle_imu_batch_t *batch)
{
    unsigned int irq = osal_irq_lock();
    bool ok = g_ack_pending && g_ack.sequence == batch->sequence &&
        g_ack.last_timestamp_us == batch->samples[batch->sample_count - 1U].timestamp_us;
    if (ok) {
        g_ack_pending = false;
    }
    osal_irq_restore(irq);
    return ok;
}

static void *imu_tx_task(const char *arg)
{
    iq_tx_node_t iq;
    sle_imu_batch_t batch = {0};
    bool iq_pending = false;
    bool batch_pending = false;
    bool waiting_ack = false;
    uint64_t iq_retry_us = 0;
    uint64_t next_send_us = 0;
    uint64_t batch_retry_us = 0;
    uint64_t batch_begin_us = 0;
    uint64_t stats_us = 0;
    unsigned int iq_sent = 0;
    unsigned int imu_sent = 0;
    unused(arg);
    while (1) {
        (void)osal_event_read(&g_tx_event, IMU_TX_EVENT, 10,
            OSAL_WAITMODE_OR | OSAL_WAITMODE_CLR);
        uint64_t now = uapi_tcxo_get_us();
        if (now >= stats_us) {
            osal_printk("TX_STATS,iq:%u,iq_drop:%u,iq_expired:%u,imu_tx:%u,imu_ack:%u,imu_expired:%u\r\n",
                iq_sent, g_iq_queue_drops, g_iq_expired, imu_sent, g_imu_acked, g_imu_expired);
            stats_us = now + 10000000ULL;
        }
        if (batch_pending && batch_acked(&batch)) {
            batch_pending = false;
            waiting_ack = false;
            g_imu_acked++;
        }
        if (batch_pending && now - batch_begin_us > IMU_BATCH_MAX_AGE_US) {
            osal_printk("IMU_DROP,%u,ack_timeout\r\n", batch.sequence);
            batch_pending = false;
            waiting_ack = false;
            g_imu_expired++;
        }
        if (now < next_send_us) {
            continue;
        }
        if (!iq_pending) {
            uint32_t size = sizeof(iq);
            iq_pending = osal_msg_queue_read_copy(g_iq_queue, &iq, &size, 0) == OSAL_SUCCESS;
        }
        if (iq_pending) {
            if (!measure_dis_client_token_ready(&iq.token) ||
                now - iq.enqueued_us > TELEMETRY_IQ_MAX_AGE_US) {
                g_iq_expired++;
                osal_printk("IQ_DROP,anchor:%u,conn:%u,stale_or_not_ready\r\n",
                    iq.token.slot + 5U, (unsigned int)iq.token.conn_id);
                iq_pending = false;
                continue;
            }
            if (now >= iq_retry_us) {
                if (measure_dis_client_send_now(&iq.token, SLEM_PROFILE_MSG_IQ, iq.data, iq.len) == ERRCODE_SUCC) {
                    iq_sent++;
                    iq_pending = false;
                }
                iq_retry_us = now + 20000ULL;
                next_send_us = now + TELEMETRY_SEND_GUARD_US;
                continue;
            }
        }
        measure_dis_link_token_t target;
        if (!measure_dis_client_imu_token(&target)) {
            continue;
        }
        if (!batch_pending) {
            batch_pending = sample_batch(&batch);
            if (batch_pending) {
                batch_begin_us = now;
                batch_retry_us = 0;
                waiting_ack = false;
                if (batch.dropped_samples != 0) {
                    osal_printk("IMU_DROP,%u,ring_overflow:%u\r\n",
                        batch.sequence, (unsigned int)batch.dropped_samples);
                }
            }
        }
        if (!batch_pending || (waiting_ack && now < batch_retry_us)) {
            continue;
        }
        if (measure_dis_client_send_now(&target, SLEM_PROFILE_MSG_IMU_BATCH,
                (const uint8_t *)&batch, sizeof(batch)) == ERRCODE_SUCC) {
            imu_sent++;
            waiting_ack = true;
            batch_retry_us = now + IMU_ACK_TIMEOUT_US;
        } else {
            waiting_ack = true;
            batch_retry_us = now + 20000ULL;
        }
        next_send_us = now + TELEMETRY_SEND_GUARD_US;
    }
    return NULL;
}

static void *imu_client_task(const char *arg)
{
    sle_imu_packet_t packet;
    unused(arg);
    if (sle_imu_sensor_init() != ERRCODE_SUCC) {
        osal_printk("IMU init failed\r\n");
        return NULL;
    }
    osal_printk("IMU sampling:20Hz,batch:%u,target:05\r\n", (unsigned int)SLE_IMU_BATCH_SAMPLES);
    uint64_t deadline = uapi_tcxo_get_us();
    while (1) {
        uint64_t now = uapi_tcxo_get_us();
        if (now >= deadline) {
            deadline += IMU_SAMPLE_PERIOD_US;
            if (now + IMU_SAMPLE_PERIOD_US / 2U >= deadline) {
                osal_printk("IMU_SAMPLE_LATE\r\n");
                deadline = now + IMU_SAMPLE_PERIOD_US;
            }
            if (sle_imu_sensor_read(&packet) == ERRCODE_SUCC) {
                packet.sequence = g_sample_sequence++;
                sample_push(&packet);
            }
        }
        osal_msleep(1);
    }
    return NULL;
}

void sle_imu_client_start(void)
{
    if (g_started) {
        return;
    }
    if (osal_mutex_init(&g_sample_lock) != OSAL_SUCCESS ||
        osal_event_init(&g_tx_event) != OSAL_SUCCESS ||
        osal_msg_queue_create("iq_tx", IQ_TX_QUEUE_SIZE, &g_iq_queue, 0, sizeof(iq_tx_node_t)) != OSAL_SUCCESS) {
        osal_printk("IMU scheduler init failed\r\n");
        return;
    }
    osal_task *tx = osal_kthread_create((osal_kthread_handler)imu_tx_task, NULL, "SleImuTx", IMU_TX_TASK_STACK);
    if (tx == NULL) {
        osal_printk("IMU TX task create failed\r\n");
        return;
    }
    osal_kthread_set_priority(tx, IMU_TX_TASK_PRIORITY);
    osal_kfree(tx);
    g_started = true;
    osal_task *sample = osal_kthread_create((osal_kthread_handler)imu_client_task, NULL, "SleImuSample", IMU_TASK_STACK);
    if (sample != NULL) {
        osal_kthread_set_priority(sample, IMU_TASK_PRIORITY);
        osal_kfree(sample);
    } else {
        osal_printk("IMU sample task create failed\r\n");
    }
}
