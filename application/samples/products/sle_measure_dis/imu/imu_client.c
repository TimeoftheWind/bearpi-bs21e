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
/* Fail at compile time rather than silently losing oversized IMU packets. */
typedef char imu_batch_must_fit_iq_payload[
    sizeof(sle_imu_batch_t) <= sizeof(sle_channel_sounding_iq_trans_t) ? 1 : -1];

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
typedef struct {
    sle_imu_batch_t batch;
    uint64_t begin_us;
    uint64_t first_send_us;
    uint64_t retry_us;
    uint64_t ack_us;
    bool active;
    bool sent;
    bool acked;
} imu_window_entry_t;
static imu_window_entry_t g_window[IMU_TX_WINDOW_SIZE];
static uint64_t g_ack_max_us;
static uint64_t g_iq_wait_max_us;

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
    if (ack == NULL) {
        return;
    }
    unsigned int irq = osal_irq_lock();
    for (unsigned int i = 0; i < IMU_TX_WINDOW_SIZE; i++) {
        imu_window_entry_t *entry = &g_window[i];
        if (entry->active && entry->sent && !entry->acked &&
            ack->sequence == entry->batch.sequence &&
            ack->last_timestamp_us == entry->batch.samples[entry->batch.sample_count - 1U].timestamp_us) {
            entry->ack_us = uapi_tcxo_get_us();
            entry->acked = true;
            break;
        }
    }
    osal_irq_restore(irq);
    (void)osal_event_write(&g_tx_event, IMU_TX_EVENT);
}

static void window_maintain(uint64_t now)
{
    for (unsigned int i = 0; i < IMU_TX_WINDOW_SIZE; i++) {
        unsigned int irq = osal_irq_lock();
        imu_window_entry_t *entry = &g_window[i];
        bool expired = false;
        uint32_t sequence = entry->batch.sequence;
        if (entry->active && entry->acked) {
            uint64_t delay = entry->ack_us - entry->first_send_us;
            if (delay > g_ack_max_us) {
                g_ack_max_us = delay;
            }
            entry->active = false;
            g_imu_acked++;
        } else if (entry->active && now - entry->begin_us > IMU_BATCH_MAX_AGE_US) {
            entry->active = false;
            g_imu_expired++;
            expired = true;
        }
        osal_irq_restore(irq);
        if (expired) {
            osal_printk("IMU_DROP,%u,ack_timeout\r\n", sequence);
        }
    }
}

static void window_fill(uint64_t now)
{
    for (unsigned int i = 0; i < IMU_TX_WINDOW_SIZE; i++) {
        if (g_window[i].active) {
            continue;
        }
        sle_imu_batch_t batch;
        if (!sample_batch(&batch)) {
            break;
        }
        unsigned int irq = osal_irq_lock();
        g_window[i] = (imu_window_entry_t){ .batch = batch, .begin_us = now, .active = true };
        osal_irq_restore(irq);
        if (batch.dropped_samples != 0) {
            osal_printk("IMU_DROP,%u,ring_overflow:%u\r\n", batch.sequence,
                (unsigned int)batch.dropped_samples);
        }
    }
}

/* Oldest eligible batch first; one outstanding batch never blocks the other slots. */
static int window_next(uint64_t now)
{
    int next = -1;
    unsigned int irq = osal_irq_lock();
    for (unsigned int i = 0; i < IMU_TX_WINDOW_SIZE; i++) {
        imu_window_entry_t *entry = &g_window[i];
        if (entry->active && !entry->acked && now >= entry->retry_us &&
            (next < 0 || (int32_t)(entry->batch.sequence - g_window[next].batch.sequence) < 0)) {
            next = (int)i;
        }
    }
    osal_irq_restore(irq);
    return next;
}

static void *imu_tx_task(const char *arg)
{
    iq_tx_node_t iq;
    bool iq_pending = false;
    uint64_t iq_retry_us = 0;
    uint64_t next_send_us = 0;
    uint64_t stats_us = 0;
    unsigned int iq_sent = 0;
    unsigned int imu_sent = 0;
    unused(arg);
    while (1) {
        (void)osal_event_read(&g_tx_event, IMU_TX_EVENT, 10,
            OSAL_WAITMODE_OR | OSAL_WAITMODE_CLR);
        uint64_t now = uapi_tcxo_get_us();
        window_maintain(now);
        if (now >= stats_us) {
            unsigned int inflight = 0;
            for (unsigned int i = 0; i < IMU_TX_WINDOW_SIZE; i++) {
                inflight += g_window[i].active;
            }
            osal_printk("TX_STATS,iq:%u,iq_drop:%u,iq_expired:%u,imu_tx:%u,imu_ack:%u,imu_expired:%u\r\n",
                iq_sent, g_iq_queue_drops, g_iq_expired, imu_sent, g_imu_acked, g_imu_expired);
            (void)osal_mutex_lock(&g_sample_lock);
            unsigned int backlog = g_sample_count;
            osal_mutex_unlock(&g_sample_lock);
            osal_printk("TX_FLOW,inflight:%u,backlog:%u,ack_max_us:%llu,iq_wait_max_us:%llu\r\n",
                inflight, backlog, (unsigned long long)g_ack_max_us, (unsigned long long)g_iq_wait_max_us);
            g_ack_max_us = g_iq_wait_max_us = 0;
            stats_us = now + 10000000ULL;
        }
        if (now < next_send_us) {
            continue;
        }
        if (!iq_pending) {
            uint32_t size = sizeof(iq);
            iq_pending = osal_msg_queue_read_copy(g_iq_queue, &iq, &size, 0) == OSAL_SUCCESS;
        }
        if (iq_pending) {
            uint64_t wait_us = now - iq.enqueued_us;
            if (wait_us > g_iq_wait_max_us) {
                g_iq_wait_max_us = wait_us;
            }
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
        window_fill(now);
        int index = window_next(now);
        if (index < 0) {
            continue;
        }
        imu_window_entry_t *entry = &g_window[index];
        unsigned int irq = osal_irq_lock();
        bool previously_sent = entry->sent;
        if (!entry->sent) {
            entry->first_send_us = now;
            entry->sent = true; /* Publish before the API: a synchronous ACK is valid. */
        }
        osal_irq_restore(irq);
        if (measure_dis_client_send_now(&target, SLEM_PROFILE_MSG_IMU_BATCH,
                (const uint8_t *)&entry->batch, sizeof(entry->batch)) == ERRCODE_SUCC) {
            imu_sent++;
            entry->retry_us = now + IMU_ACK_TIMEOUT_US;
        } else {
            irq = osal_irq_lock();
            entry->sent = previously_sent;
            osal_irq_restore(irq);
            entry->retry_us = now + 20000ULL;
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
