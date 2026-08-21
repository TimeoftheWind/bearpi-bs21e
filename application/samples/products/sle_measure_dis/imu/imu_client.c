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
#define IMU_SAMPLE_REPHASE_THRESHOLD_US (IMU_SAMPLE_PERIOD_US / 2U)
#define IMU_TX_EVENT 1U
#define IMU_TX_IDLE_GUARD_US 80000ULL
#define IMU_TX_RETRY_GUARD_US 10000ULL
#define IMU_TX_WAKE_PERIOD_MS 10U

typedef struct {
    sle_imu_packet_t samples[SLE_IMU_RING_CAPACITY];
    uint16_t read_index;
    uint16_t write_index;
    uint16_t count;
    uint16_t dropped;
} imu_ring_t;

static imu_ring_t g_imu_ring;
static osal_mutex g_imu_lock;
static osal_event g_imu_tx_event;
static uint8_t g_imu_sync_ready;
static volatile uint8_t g_iq_pending;
static uint8_t g_iq_data[sizeof(sle_channel_sounding_iq_trans_t)];
static uint32_t g_iq_data_len;
static uint32_t g_batch_sequence;
static uint32_t g_sample_sequence;
static uint64_t g_imu_tx_not_before_us;

static void imu_ring_push(const sle_imu_packet_t *sample)
{
    (void)osal_mutex_lock(&g_imu_lock);
    if (g_imu_ring.count == SLE_IMU_RING_CAPACITY) {
        g_imu_ring.read_index = (uint16_t)((g_imu_ring.read_index + 1U) % SLE_IMU_RING_CAPACITY);
        g_imu_ring.count--;
        g_imu_ring.dropped++;
    }
    g_imu_ring.samples[g_imu_ring.write_index] = *sample;
    g_imu_ring.write_index = (uint16_t)((g_imu_ring.write_index + 1U) % SLE_IMU_RING_CAPACITY);
    g_imu_ring.count++;
    osal_mutex_unlock(&g_imu_lock);
    if (g_imu_sync_ready != 0U) {
        (void)osal_event_write(&g_imu_tx_event, IMU_TX_EVENT);
    }
}

static bool imu_ring_take_batch(sle_imu_batch_t *batch)
{
    bool ready = false;
    (void)osal_mutex_lock(&g_imu_lock);
    if (g_imu_ring.count >= SLE_IMU_BATCH_SAMPLES) {
        batch->sequence = g_batch_sequence++;
        batch->sample_count = SLE_IMU_BATCH_SAMPLES;
        batch->dropped_samples = g_imu_ring.dropped;
        g_imu_ring.dropped = 0;
        for (uint32_t i = 0; i < SLE_IMU_BATCH_SAMPLES; i++) {
            batch->samples[i] = g_imu_ring.samples[g_imu_ring.read_index];
            g_imu_ring.read_index = (uint16_t)((g_imu_ring.read_index + 1U) % SLE_IMU_RING_CAPACITY);
        }
        g_imu_ring.count -= SLE_IMU_BATCH_SAMPLES;
        ready = true;
    }
    osal_mutex_unlock(&g_imu_lock);
    return ready;
}

errcode_t sle_imu_tx_enqueue_iq(const uint8_t *data, uint32_t data_len)
{
    if (!g_imu_sync_ready || data == NULL || data_len > sizeof(g_iq_data)) {
        return ERRCODE_INVALID_PARAM;
    }
    (void)osal_mutex_lock(&g_imu_lock);
    if (memcpy_s(g_iq_data, sizeof(g_iq_data), data, data_len) != EOK) {
        osal_mutex_unlock(&g_imu_lock);
        return ERRCODE_MEMCPY;
    }
    g_iq_data_len = data_len;
    g_iq_pending = 1U;
    osal_mutex_unlock(&g_imu_lock);
    (void)osal_event_write(&g_imu_tx_event, IMU_TX_EVENT);
    return ERRCODE_SUCC;
}

static bool imu_tx_take_iq(uint8_t *data, uint32_t *data_len)
{
    bool ready = false;
    (void)osal_mutex_lock(&g_imu_lock);
    if (g_iq_pending != 0U) {
        *data_len = g_iq_data_len;
        if (memcpy_s(data, sizeof(g_iq_data), g_iq_data, g_iq_data_len) == EOK) {
            g_iq_pending = 0U;
            ready = true;
        }
    }
    osal_mutex_unlock(&g_imu_lock);
    return ready;
}

static bool imu_tx_iq_pending(void)
{
    bool pending;
    (void)osal_mutex_lock(&g_imu_lock);
    pending = (g_iq_pending != 0U);
    osal_mutex_unlock(&g_imu_lock);
    return pending;
}

static void *imu_tx_task(const char *arg)
{
    uint8_t iq_data[sizeof(sle_channel_sounding_iq_trans_t)];
    uint32_t iq_len;
    sle_imu_batch_t batch;
    bool batch_pending = false;
    unused(arg);

    while (1) {
        (void)osal_event_read(&g_imu_tx_event, IMU_TX_EVENT, IMU_TX_WAKE_PERIOD_MS,
                              OSAL_WAITMODE_OR | OSAL_WAITMODE_CLR);
        if (!measure_dis_client_is_ready()) {
            continue;
        }

        if (imu_tx_take_iq(iq_data, &iq_len)) {
            if (measure_dis_client_send_now(SLEM_PROFILE_MSG_IQ, iq_data, iq_len) == ERRCODE_SUCC) {
                g_imu_tx_not_before_us = uapi_tcxo_get_us() + IMU_TX_IDLE_GUARD_US;
            }
            continue;
        }

        if (imu_tx_iq_pending() || (uapi_tcxo_get_us() < g_imu_tx_not_before_us)) {
            continue;
        }
        if (!batch_pending) {
            batch_pending = imu_ring_take_batch(&batch);
        }
        if (!batch_pending) {
            continue;
        }
        if (measure_dis_client_send_now(SLEM_PROFILE_MSG_IMU_BATCH,
                                        (const uint8_t *)&batch, sizeof(batch)) != ERRCODE_SUCC) {
            osal_printk("IMU batch send failed, sequence:%u\r\n", batch.sequence);
            g_imu_tx_not_before_us = uapi_tcxo_get_us() + IMU_TX_RETRY_GUARD_US;
        } else {
            batch_pending = false;
            g_imu_tx_not_before_us = uapi_tcxo_get_us() + IMU_TX_IDLE_GUARD_US;
        }
    }
    return NULL;
}

static void *imu_client_task(const char *arg)
{
    sle_imu_packet_t packet;
    unused(arg);
    osal_printk("IMU init begin\r\n");
    if (sle_imu_sensor_init() != ERRCODE_SUCC) {
        osal_printk("IMU init failed\r\n");
        return NULL;
    }
    osal_printk("IMU init ok, sampling 9-axis data at 20 Hz, batching 4 samples\r\n");
    uint64_t next_sample_us = uapi_tcxo_get_us();
    while (1) {
        uint64_t now_us = uapi_tcxo_get_us();
        if ((int64_t)(now_us - next_sample_us) >= 0) {
            uint64_t scheduled_sample_us = next_sample_us;
            next_sample_us += IMU_SAMPLE_PERIOD_US;
            if ((int64_t)(now_us - scheduled_sample_us) >=
                (int64_t)IMU_SAMPLE_REPHASE_THRESHOLD_US) {
                next_sample_us = now_us + IMU_SAMPLE_PERIOD_US;
            }
            if (sle_imu_sensor_read(&packet) == ERRCODE_SUCC) {
                packet.sequence = g_sample_sequence++;
                imu_ring_push(&packet);
            }
        }
        osal_msleep(1);
    }
    return NULL;
}

void sle_imu_client_start(void)
{
    if ((osal_mutex_init(&g_imu_lock) != OSAL_SUCCESS) ||
        (osal_event_init(&g_imu_tx_event) != OSAL_SUCCESS)) {
        osal_printk("IMU scheduler init failed\r\n");
        return;
    }
    g_imu_sync_ready = 1U;

    osal_task *tx_task = osal_kthread_create((osal_kthread_handler)imu_tx_task, NULL,
        "SleImuTx", IMU_TX_TASK_STACK);
    if (tx_task != NULL) {
        osal_kthread_set_priority(tx_task, IMU_TX_TASK_PRIORITY);
        osal_kfree(tx_task);
    }

    osal_task *sample_task = osal_kthread_create((osal_kthread_handler)imu_client_task, NULL,
        "SleImuSample", IMU_TASK_STACK);
    if (sample_task != NULL) {
        osal_kthread_set_priority(sample_task, IMU_TASK_PRIORITY);
        osal_kfree(sample_task);
    }
}
