// SPDX-License-Identifier: BSD-3-Clause
// Copyright 2026 Lime Microsystems

#include "transmitter.h"

#include "dmac.h"
#include "dfe.h"
#include "tone_generator.h"
#include "l1-trace.h"

#include "dma_common.h"
#include "iqstream_signals.h"

#include "axiq-la9310.h"
#include "vcpu.h"

#include "opstatus.h"
#include "vspa_iqstream.h"
#include "phytimer.h"

#define PHY_TMR_DMA_CHAN 0
#define PHY_TMR_DMA_CHAN_MASK (1 << PHY_TMR_DMA_CHAN)

#define TX_DAC_FIFO_BEAT_COUNT 16
#define dac_axi_fifo_addr (0x4400B000) // + 0x1000 - TX_DAC_FIFO_BEAT_COUNT * 16) // axi_DAC_FIFO_addr

#define XFER_SAMPLES 512

#define DAC_XFER_SAMPLE_COUNT XFER_SAMPLES
#define DAC_XFER_SIZE_BYTES (DAC_XFER_SAMPLE_COUNT * 4)

#define DDR_XFER_SAMPLE_COUNT XFER_SAMPLES
#define DDR_XFER_SIZE_BYTES (DDR_XFER_SAMPLE_COUNT * 4)

static const uint32_t reschedule_time = DAC_XFER_SAMPLE_COUNT / 2;

extern uint32_t stream_origin_phytime;

cfixed16_t dac_buffer[TX_MAX_LANE_COUNT][MAX_DMA_ENQ * DAC_XFER_SAMPLE_COUNT] _VSPA_VECTOR_ALIGN
    __attribute__((section(".ippu_dmem")));
cfixed16_t ddr_read_buffer[TX_MAX_LANE_COUNT][MAX_DMA_ENQ * DDR_XFER_SAMPLE_COUNT] _VSPA_VECTOR_ALIGN
    __attribute__((section(".ippu_dmem")));

tx_pipeline_t txpipe[TX_MAX_LANE_COUNT] = { { false } };
tone_state_t tx_tone_state[TX_MAX_LANE_COUNT];
dac_pipeline_t dac[TX_MAX_LANE_COUNT];

tx_ddr_pipeline_t txddr[TX_MAX_LANE_COUNT];
struct PipeStats tx_stats;
tx_tdd_config_t tx_tdd;

// when DAC clock divider is disabled, 1 PHYtimer tick can represent only >=2 samples period
static const uint16_t dac_clock_divider_disabled = 0; // TODO: read from config registers

#define TX_MAX_UPSAMPLE_TAPS 64
cfixed16_t int_history[TX_MAX_UPSAMPLE_TAPS] __attribute__((aligned(64), section(".vcpu_dmem"))) = { 0 };
uint16_t int_ratio_pow2[TX_MAX_LANE_COUNT] = { 0 };

// Interpolation function prototypes
extern void X2_interp_tap32_filter(__fx16 *output, __fx16 *input, unsigned int num_samples, __fx16 *history, float *filter_taps);
extern void X4_interp_tap64_filter(__fx16 *output, __fx16 *input, unsigned int num_samples, __fx16 *history, float *filter_taps);

const float tx_filter_taps_upsampling_x2[32] _VSPA_VECTOR_ALIGN = {
#include "fir_interpolation_x2.txt"
};
// same coefficients as x2, just interleaved multiple instances.
const float tx_filter_taps_upsampling_x4[128] _VSPA_VECTOR_ALIGN = {
#include "fir_interpolation_x4.txt"
};

static inline void schedule_dac_start(const tx_tdd_config_t *tdd, uint32_t phytime) {
    const uint16_t ids[] = {
        PHY_TIMER_COMP_CH5_TX_ALLOWED, PHY_TIMER_COMP_RFCTL_0 // RF switch
        ,
        PHY_TIMER_COMP_RFCTL_5 // PA_EN
    };
    const uint16_t triggers[] = { ePhyTimerComparatorOut1, tdd->rf_sw_active_trigger, ePhyTimerComparatorOut1 };
    uint32_t phytimes[3];
    phytimes[0] = phytime + tdd->dac_allowed;
    phytimes[1] = phytimes[0] + tdd->rf_sw_on;
    phytimes[2] = phytimes[0] + tdd->pa_on;
    timers_trig_schedule(ids, triggers, phytimes, 3);
}

static inline void schedule_dac_end(const tx_tdd_config_t *tdd, uint32_t phytime) {
    const uint16_t ids[] = {
        PHY_TIMER_COMP_CH5_TX_ALLOWED, PHY_TIMER_COMP_RFCTL_0 // RF switch
        ,
        PHY_TIMER_COMP_RFCTL_5 // PA_EN
    };
    const uint16_t triggers[] = { ePhyTimerComparatorOut0,
                                  (tdd->rf_sw_active_trigger == ePhyTimerComparatorOut1 ? ePhyTimerComparatorOut0
                                                                                        : ePhyTimerComparatorOut1),
                                  ePhyTimerComparatorOut0 };
    uint32_t phytimes[3];
    phytimes[0] = phytime + tdd->dac_allowed;
    phytimes[1] = phytimes[0] + tdd->rf_sw_off;
    phytimes[2] = phytimes[0] + tdd->pa_off;
    timers_trig_schedule(ids, triggers, phytimes, 3);
}

// @brief Attempt to enque all available DDR buffers
static inline void tx_initial_ddr_enque(uint16_t lane) {
    tx_ddr_pipeline_t *const ddr = &txddr[lane];
    if (ddr->count_dmac_enque - ddr->buffers_consumed < MAX_DMA_ENQ)
        tx_lane_try_ddr_enqueue(&txddr[lane], true);
    if (ddr->count_dmac_enque - ddr->buffers_consumed < MAX_DMA_ENQ)
        tx_lane_try_ddr_enqueue(&txddr[lane], true);
}

static inline void TxAXIQ(bool enabled) {
    if (enabled) {
        axiq_tx_enable();
        // log_info("TxAXIQ on" LOG_EOL);
    } else {
        axiq_tx_disable();
        // log_info("TxAXIQ off" LOG_EOL);
    }
}

static inline bool tx_axiq_is_enabled() {
    return gpord(7, (1 << 0)); // Tx AXIQ enable
}

static inline void stream_write_ptr_rst_trig(uint16_t lane) {
    const uint32_t ctrl = DMAC_FIFO_RESET | DMAC_WRC | dac[lane].dma_channel;
    // Need to write enough data to trigger AXIQ FIFO threshold
    dmac_enable(ctrl, TX_DAC_FIFO_BEAT_COUNT * 16, dac[lane].axi_fifo_addr, VCPU_ADDR_FOR_DMA(dac_buffer[lane]));
}

static void tx_dac_reset(dac_pipeline_t *dac, cfixed16_t *buffer) {
    dac->base_buffer = buffer;
    dac->buffers_provided = 0;
    dac->count_dmac_enque = 0;
    dac->count_dmac_complete = 0;
    dac->reschedule_start = false;

    axiq_fifo_tx_cr(AXIQ_BANK_0, (enum axiq_fifo_e)dac->axi_fifo_index, AXIQ_CR_CLRERR, AXIQ_CR_CLRERR);
    axiq_fifo_tx_cr(AXIQ_BANK_0, (enum axiq_fifo_e)dac->axi_fifo_index, AXIQ_CR_CLRERR, 0);

    // reset dma
    const uint32_t dma_mask = (1 << dac->dma_channel);
    dmac_clear_complete(dma_mask);
    dmac_clear_event(dma_mask);
    dmac_clear_errxfr(dma_mask);
    dmac_clear_errcfg(dma_mask);
}

static void tx_host_dma_reset(tx_ddr_pipeline_t *ddr) {
    ddr->dma_hif.tcd_table.done = 0;
    tcd_fifo_reset(&ddr->dma_hif.tcd_table);
}

static void tx_ddr_reset(tx_ddr_pipeline_t *ddr, cfixed16_t *buffer) {
    timer_trig_immediate(PHY_TIMER_COMP_VSPA_GO_1, ePhyTimerComparatorOut0);
    ddr->base_buffer = buffer;
    ddr->ready_buffer = buffer;
    ddr->count_dmac_enque = 0;
    ddr->count_dmac_complete = 0;
    ddr->buffers_consumed = 0;
    ddr->ready_buffer_offset = 0;
    ddr->deffered = false;
    memclr(ddr->meta, sizeof(ddr->meta));

    const uint32_t dma_mask = (1 << ddr->dma_channel);
    dmac_abort(dma_mask);
    WAIT_FOR(!dmac_is_running(dma_mask), VSPA_DEFAULT_TIMEOUT);

    dmac_clear_complete(dma_mask);
    dmac_clear_event(dma_mask);
    dmac_clear_errxfr(dma_mask);
    dmac_clear_errcfg(dma_mask);
}

// Configure pipline's persistent parameters
void tx_lane_setup(uint16_t lane, uint16_t channel) {
    channel = 0; // TX has only 1 channel
    memclr(&tx_stats, sizeof(tx_stats));
    dac[lane].axi_fifo_addr = dac_axi_fifo_addr + (channel * 0x1000);
    dac[lane].axi_fifo_index = (enum axiq_fifo_e)(AXIQ_FIFO_TX0) + channel;
    dac[lane].dma_channel = TX_DAC_WR_DMA_CHANNEL + channel;
    tx_dac_reset(&dac[lane], dac_buffer[lane]);

    tx_tone_state[lane].amplitude = 0.9;
    tx_tone_state[lane].phase = 0;
    tx_tone_state[lane].freq_bin = 8192;

    txddr[lane].dma_channel = DDR_RD_DMA_CHANNEL_1; // + lane;
    tx_ddr_reset(&txddr[lane], ddr_read_buffer[lane]);
    tx_host_dma_reset(&txddr[lane]);

    memclr(int_history, sizeof(int_history));
}

void tx_lane_try_ddr_enqueue(tx_ddr_pipeline_t *ddr, bool vcpu_trig) {
    TRACE_START_DURATION(t1);

    // TRACE_COUNTER(CNT_TX_TCD, tcd_fifo_size(&ddr->dma_hif.tcd_table));
    if (tcd_fifo_isempty(&ddr->dma_hif.tcd_table)) {
        if (gpird(1, 1 << 16)) // tx_dma_allowed
            ++tx_stats.host_udr;
        return;
    }

    dma_tcd_t *tcd;
    tx_meta_t *meta;
    do {
        tcd = tcd_fifo_front(&ddr->dma_hif.tcd_table);
        if (tcd->flags & PKT_START) {
            if (dac[0].buffers_provided - dac[0].count_dmac_complete > 0) {
                // deffer next Tx burst reading after the current one finishes
                // DAC currently active, cannot schedule next start while it's not finished
                TRACE_EVENT(T_TX_BURST_DEFFER, 3, tcd->timestamp_lsb);
                TRACE_DURATION(T_DDR_RD_ENQ, DEFAULT_THREAD_ID, t1);
                return;
            }
            TRACE_EVENT(T_TX_BURST_START, 3, tcd->timestamp_lsb);
        }

        meta = &ddr->meta[ddr->count_dmac_enque & 0x1];
        // Convert the sample timestamp into phytimer value amount.
        // phytimer stream start offset will be applied right before scheduling.
        meta->phytime = (tcd->timestamp_lsb << int_ratio_pow2[0]) >> dac_clock_divider_disabled;

        const uint32_t phytime_now = gpird(4); // Phytimer
        const uint32_t ddr_advance = (meta->phytime + stream_origin_phytime) - phytime_now;
        // TRACE_COUNTER(CNT_PHYTIME, ddr_advance);
        if (ddr_advance < reschedule_time) {
            // too late to enque, just drop it.
            ++tx_stats.dfe_drop;
            ++tx_stats.dfe_udr;
            tcd_fifo_pop(&ddr->dma_hif.tcd_table);
            ++ddr->dma_hif.tcd_table.done;
            vspa_to_host_signal(ddr->dma_hif.vth_tcd_done_flag_mask);
            if (tcd_fifo_isempty(&ddr->dma_hif.tcd_table))
                return;
            continue;
        }
        break;
    } while (1);

    meta->flags = tcd->flags;
    // TODO: use modulo buffer registers for circular addressing?
    cfixed16_t *const dest = ddr->base_buffer + (ddr->count_dmac_enque & 0x1) * DDR_XFER_SAMPLE_COUNT;
    uint32_t xfer_size = DDR_XFER_SIZE_BYTES;
    if (tcd->size < DDR_XFER_SIZE_BYTES) {
        xfer_size = tcd->size;
        // host can send any amount of data, but internally it's processed in fixed size chunks.
        // clear residual data if host is not going to overwrite it all.
        memclr(dest, DDR_XFER_SAMPLE_COUNT * 2);
    }

    iowr(DMA_DMEM_PRAM_ADDR, VCPU_ADDR_FOR_DMA(dest));
    iowr(DMA_AXI_ADDRESS, tcd->la9310_mem_address);
    iowr(DMA_AXI_BYTE_CNT, xfer_size);

    tcd->la9310_mem_address += xfer_size;
    tcd->size -= xfer_size;
    tcd->flags &= ~(PKT_START); // clear start tag once used
    tcd->timestamp_lsb += DDR_XFER_SAMPLE_COUNT;

    if (tcd->size == 0) {
        meta->flags |= PKT_DMA_TCD_END;
        tcd_fifo_pop(&ddr->dma_hif.tcd_table);
    } else {
        meta->flags &= ~(PKT_IRQ | PKT_END); // not yet the end of TCD
    }

    uint32_t dma_ctrl = DMAC_MBRE | DMAC_RDC | ddr->dma_channel;
    if (vcpu_trig)
        dma_ctrl |= DMAC_TRIG_VCPU;
    iowr(DMA_XFR_CTRL, dma_ctrl); // ddr enque
    TRACE_DMA_BEGIN(ddr->dma_channel, ddr->count_dmac_enque & 1);

    ++ddr->count_dmac_enque;
    TRACE_COUNTER(CNT_TX_DDR_ENQ, ddr->count_dmac_enque - ddr->count_dmac_complete);
    ++tx_stats.dfe_enq;

    TRACE_DURATION(T_DDR_RD_ENQ, DEFAULT_THREAD_ID, t1);
}

static inline bool consume_ddr(tx_ddr_pipeline_t *ddr, uint16_t samplesCount) {
    ddr->ready_buffer_offset += samplesCount;
    if (ddr->ready_buffer_offset < DDR_XFER_SAMPLE_COUNT)
        return false;

    ++ddr->buffers_consumed;
    ddr->ready_buffer_offset = 0;
    return true;
}

inline static void dac_enque(dac_pipeline_t *dac, cfixed16_t *buffer, uint32_t extra_dmac_flags) {
    const uint32_t dma_ctrl = (DMAC_WRC | DMAC_FIFO | DMAC_TRIG_VCPU) | dac->dma_channel | extra_dmac_flags;
    dmac_enable(dma_ctrl, // flags
                DAC_XFER_SIZE_BYTES, // size
                dac->axi_fifo_addr, // axi addr
                VCPU_ADDR_FOR_DMA(buffer) // dmem addr
    );
    TRACE_DMA_BEGIN(dac->dma_channel, dac->count_dmac_enque & 1);

    ++dac->count_dmac_enque;
    tx_stats.afe_enq = dac->count_dmac_enque;
}

void dac_prime(dac_pipeline_t *dac) {
    dac_enque(dac, dac->base_buffer, 0x0);
    dac_enque(dac, dac->base_buffer + DAC_XFER_SAMPLE_COUNT, 0x0);
    // DMA will be ready, and start on PHYTimer trigger
}

static inline void interpol(cfixed16_t *dest, cfixed16_t *src, cfixed16_t *history, uint16_t src_count) {
    uint16_t lane = 0;
    switch (int_ratio_pow2[lane]) {
    case 1:
        X2_interp_tap32_filter((__fx16 *)dest, (__fx16 *)src, src_count, (__fx16 *)int_history,
                               (float *)tx_filter_taps_upsampling_x2);
        break;
    case 2:
        X4_interp_tap64_filter((__fx16 *)dest, (__fx16 *)src, src_count, (__fx16 *)int_history,
                               (float *)tx_filter_taps_upsampling_x4);
        break;
    default:
        break;
    }
}

static void ddr_to_dac_work(uint16_t lane) {
    TRACE_START_DURATION(t1);
    tx_ddr_pipeline_t *const ddr = &txddr[lane];

    // TODO: use modulo buffer registers for circular addressing?
    cfixed16_t *const src = ddr->base_buffer + (ddr->buffers_consumed & 0x1) * DDR_XFER_SAMPLE_COUNT + ddr->ready_buffer_offset;
    cfixed16_t *const dest = dac->base_buffer + (dac->buffers_provided & 0x1) * DAC_XFER_SAMPLE_COUNT;
    tx_meta_t *const ddr_meta = &ddr->meta[ddr->buffers_consumed & 0x1];
    tx_meta_t *const dac_meta = &dac->meta[dac->buffers_provided & 0x1];
    dac_meta->flags = 0;
    dac_meta->phytime = ddr_meta->phytime;
    TRACE_EVENT(T_PHYTIME, 5, dac_meta->phytime);

    if (int_ratio_pow2[lane]) {
        if (ddr_meta->flags & PKT_START) {
            ddr_meta->flags ^= PKT_START; // clear as ddr buffer is consumed over multiple interpolations
            memclr(int_history, sizeof(int_history)); // reset interpolator for new start
        }
        const uint16_t work_samples_count = DAC_XFER_SAMPLE_COUNT >> int_ratio_pow2[lane];
        interpol(dest, src, int_history, work_samples_count);
        // qec inplace
        tx_qec_correction(dest, dest, DAC_XFER_SAMPLE_COUNT);

        ddr->ready_buffer_offset += work_samples_count;
        ddr_meta->phytime += DAC_XFER_SAMPLE_COUNT;
        TRACE_DURATION(T_TX_WORK, DEFAULT_THREAD_ID, t1);
        if (ddr->ready_buffer_offset >= DDR_XFER_SAMPLE_COUNT) {
            ++ddr->buffers_consumed;
            ddr->ready_buffer_offset = 0;
            dac_meta->flags = ddr_meta->flags & PKT_END;
            tx_lane_try_ddr_enqueue(ddr, false);
        }

    } else {
        tx_qec_correction(dest, src, DAC_XFER_SAMPLE_COUNT);
        TRACE_DURATION(T_TX_WORK, DEFAULT_THREAD_ID, t1);
        ++ddr->buffers_consumed;
        dac_meta->flags = ddr_meta->flags & PKT_END;
        tx_lane_try_ddr_enqueue(ddr, false);
    }
    ++dac->buffers_provided;
}

void dac_dma_complete(uint16_t lane) {
    TRACE_START_DURATION(t1);
    // const uint16_t buf_index = dac[lane].count_dmac_complete & 1;
    TRACE_DMA_END(dac[lane].dma_channel, dac[lane].count_dmac_complete & 1);

    // const uint32_t cyccnt = ccnt_read();
    // tx_stats.afe_xfer_pace[buf_index] = cyccnt - tx_stats.afe_enq_cycle[buf_index];

    const uint32_t dma_mask = (1 << dac[lane].dma_channel);
    dmac_clear_complete(dma_mask); // redundant, only using event bits
    dmac_clear_event(dma_mask);

    // TODO: use modulo buffer registers for circular addressing?
    cfixed16_t *const completed_buffer = dac->base_buffer + (dac->count_dmac_complete & 0x1) * DAC_XFER_SAMPLE_COUNT;
    ++dac[lane].count_dmac_complete;
    tx_stats.afe_compl = dac[lane].count_dmac_complete;

    // tx_check_axiq_status();

    if (txddr[lane].count_dmac_complete - txddr[lane].buffers_consumed == 0) {
        TRACE_DURATION(T_DAC_COMPLETE, DEFAULT_THREAD_ID, t1);
        return;
    }

    // ddr buffer is available, attempt to fill dac with valid data
    ddr_to_dac_work(lane);

    if (dac[lane].buffers_provided - dac[lane].count_dmac_enque == 0) {
        TRACE_DURATION(T_DAC_COMPLETE, DEFAULT_THREAD_ID, t1);
        return;
    }

    // was dac buffer prepared
    TRACE_START_DURATION(t2);
    uint32_t dmac_flags = 0;
    tx_meta_t *const dac_meta = &dac[lane].meta[dac[lane].count_dmac_enque & 1];
    if (dac_meta->flags & PKT_END) {
        dmac_flags = DMAC_FIFO_RESET;
        const uint32_t tx_dma_off_phytime = dac_meta->phytime + DAC_XFER_SAMPLE_COUNT;
        schedule_dac_end(&tx_tdd, tx_dma_off_phytime);

        TRACE_EVENT(T_TX_BURST_DEFFER, 4, tx_dma_off_phytime);
        deffer_next_tx_burst(tx_dma_off_phytime + 2 * DAC_XFER_SAMPLE_COUNT);
    }
    TRACE_EVENT(T_PHYTIME, 5, dac_meta->phytime);
    dac_enque(dac, completed_buffer, dmac_flags);

    TRACE_DURATION(T_DAC_ENQ, DEFAULT_THREAD_ID, t2);
    TRACE_DURATION(T_DAC_COMPLETE, DEFAULT_THREAD_ID, t1);
}

// @brief Aborts and drops all current DDR buffers
static void tx_drop_ddr(uint16_t lane) {
    tx_ddr_pipeline_t *const ddr = &txddr[lane];
    tx_stats.dfe_drop += ddr->count_dmac_enque - ddr->buffers_consumed;

    if (ddr->count_dmac_enque - ddr->count_dmac_complete > 0) {
        dmac_abort(1 << ddr->dma_channel);
        if ((ddr->meta[0].flags | ddr->meta[1].flags) & PKT_DMA_TCD_END) {
            ++ddr->dma_hif.tcd_table.done;
            vspa_to_host_signal(ddr->dma_hif.vth_tcd_done_flag_mask);
        }
    }

    // tx_stats.dfe_compl = 0;
    // tx_stats.dfe_enq = 0;
    ddr->count_dmac_enque = 0;
    ddr->count_dmac_complete = 0;
    ddr->buffers_consumed = 0;
    ddr->ready_buffer_offset = 0;
    ddr->deffered = false;
    memclr(ddr->meta, sizeof(ddr->meta));
}

void tx_ddr_complete(uint16_t lane) {
    TRACE_START_DURATION(t1);
    tx_ddr_pipeline_t *const ddr = &txddr[lane];
    TRACE_DMA_END(ddr->dma_channel, ddr->count_dmac_complete & 1);
    dmac_clear_complete(1 << ddr->dma_channel);
    dmac_clear_event(1 << ddr->dma_channel);

    tx_meta_t *const meta = &ddr->meta[ddr->count_dmac_complete & 0x1];
    ++ddr->count_dmac_complete;
    ++tx_stats.dfe_compl;
    // TRACE_COUNTER(CNT_TX_DDR_ENQ, ddr->count_dmac_enque - ddr->count_dmac_complete);

    if (meta->flags & PKT_DMA_TCD_END) {
        ++ddr->dma_hif.tcd_table.done;
        vspa_to_host_signal(ddr->dma_hif.vth_tcd_done_flag_mask);
    }

    meta->phytime += stream_origin_phytime;
    if (meta->flags & PKT_START || dac[lane].reschedule_start) {
        const uint32_t tx_dma_allowed = gpird(1, 1 << 16); // Phytimer trigger value
        if (tx_dma_allowed) {
            // DAC currently still active, cannot schedule next start while it's not finished
            TRACE_EVENT(T_TX_BURST_DEFFER, 4, 11);
            TRACE_DURATION(T_DDR_RD_COMPLETE, DEFAULT_THREAD_ID, t1);
            return;
        }
        const uint32_t phytime_now = gpird(4); // Phytimer
        const uint32_t metatime_diff = (meta->phytime + stream_origin_phytime) - phytime_now;
        if (metatime_diff < reschedule_time || metatime_diff > 0x7FFFFFFFu) {
            // data arrived too late
            tx_drop_ddr(lane);
            tx_initial_ddr_enque(lane);
            dac[lane].reschedule_start = true;
            return;
        }
        dac[lane].reschedule_start = false;
        schedule_dac_start(&tx_tdd, meta->phytime);
    }
    if (dac[lane].count_dmac_enque - dac[lane].count_dmac_complete < MAX_DMA_ENQ) {
        ddr_to_dac_work(lane);
        TRACE_START_DURATION(t2);
        dac_enque(&dac[lane], dac->base_buffer + (dac->count_dmac_enque & 0x1) * DAC_XFER_SAMPLE_COUNT, 0x0);
        TRACE_DURATION(T_DAC_ENQ, DEFAULT_THREAD_ID, t2);
    }

    TRACE_DURATION(T_DDR_RD_COMPLETE, DEFAULT_THREAD_ID, t1);
}

void transmitter_init(void) {
    // tx_tdd.dac_allowed = -33; // over analog loopback there is 33 samples delay RxT0 = TxT33
    TxAXIQ(false);
    for (int lane = 0; lane < TX_MAX_LANE_COUNT; ++lane) {
        vspa_dma_hif_t *dma_hif = &txddr[lane].dma_hif;
        dma_hif->tcd_table.done = 0;
        dma_hif->htv_tcd_pending_flag_mask = (HTV_SIGNAL_TXLANE0_TCD_PENDING << lane);
        dma_hif->vth_tcd_done_flag_mask = (VTH_SIGNAL_TXLANE0_TCD_DONE << lane);
        tcd_fifo_reset(&txddr[lane].tcd_table);
        clear_htv_signal(dma_hif->htv_tcd_pending_flag_mask);
    }

    tx_lane_setup(0, 0);
}

int tx_set_oversampling(uint16_t lane, uint16_t oversample_pow2) {
    if (lane > TX_MAX_LANE_COUNT)
        return lime_Result_InvalidValue;

    int_ratio_pow2[lane] = oversample_pow2;
    return lime_Result_Success;
}

// @brief Aborts DAC DMA and drops all pending buffers
static void inline tx_axiq_fifo_reset(uint16_t lane) {
    TRACE_START_DURATION(t1);
    timer_trig_immediate(PHY_TIMER_COMP_CH5_TX_ALLOWED, ePhyTimerComparatorOut1);

    TxAXIQ(true); // enable just in case it wasn't. We'll need falling edge.
    uint32_t dma_mask = 1 << dac[lane].dma_channel;
    TxAXIQ(false); // falling edge, enters DMA flush mode
    // aborted DMA transactions won't trigger their complete/go/ptr_rst
    dmac_abort(dma_mask);

    // ensure abort has ended before issuing new dma commands
    WAIT_FOR(!dmac_is_running(dma_mask), VSPA_DEFAULT_TIMEOUT);

    stream_write_ptr_rst_trig(lane); // exit flush mode, tx_dma_allowed trigger must be still enabled at this point
    // wait for ptr reset
    WAIT_FOR(dmac_is_complete(dma_mask), VSPA_DEFAULT_TIMEOUT);

    dma_mask |= timer_trig_immediate_async(PHY_TIMER_COMP_CH5_TX_ALLOWED, ePhyTimerComparatorOut0);
    // treat as all transfers completed
    tx_stats.afe_drop += dac[lane].buffers_provided - dac[lane].count_dmac_complete;
    dac[lane].buffers_provided = 0;
    dac[lane].count_dmac_complete = 0;
    dac[lane].count_dmac_enque = 0;
    // tx_stats.afe_compl = 0;
    // tx_stats.afe_enq = 0;
    WAIT_FOR(dmac_is_complete(dma_mask) == dma_mask, VSPA_DEFAULT_TIMEOUT);
    dmac_clear_complete(dma_mask);
    dmac_clear_event(dma_mask);
    TRACE_DURATION(T_DAC_AXIQ_RST, DEFAULT_THREAD_ID, t1);
}

static inline void tx_handle_dac_underrun(void) {
    TRACE_START_DURATION(t1);
    uint16_t lane = 0;

    // clear everything from DAC
    tx_axiq_fifo_reset(lane);

    // Drop current DDR buffers as they are most likely too close in time for rescheduling new start
    tx_drop_ddr(lane); // aborts ddr dma

    TxAXIQ(true);
    axiq_fifo_tx_cr(AXIQ_BANK_0, AXIQ_FIFO_TX0, AXIQ_CR_CLRERR, AXIQ_CR_CLRERR);
    axiq_fifo_tx_cr(AXIQ_BANK_0, AXIQ_FIFO_TX0, AXIQ_CR_CLRERR, 0);

    // force phytimer sheduling upon next TCD regardless if it has PKT_START tag
    dac[lane].reschedule_start = true;

    tx_ddr_pipeline_t *const ddr = &txddr[lane];
    if (tcd_fifo_isempty(&ddr->dma_hif.tcd_table))
        return;

    // reenque two initial buffers
    tx_initial_ddr_enque(lane);

    TRACE_DURATION(T_DAC_AXIQ_RST, DEFAULT_THREAD_ID, t1);
}

// Resets pipeline and initiates start for new transmission
void tx_lane_prime(uint16_t lane) {
    tx_ddr_pipeline_t *const ddr = &txddr[lane];
    memclr(&tx_stats, sizeof(tx_stats));

    // reset ddr state
    tx_ddr_reset(ddr, ddr_read_buffer[lane]);
    tx_host_dma_reset(ddr);

    // Prime dac AXIQ and DMA engine, the actual start is triggered by phytimer
    tx_dac_reset(&dac[lane], dac_buffer[lane]);
    tx_axiq_fifo_reset(lane);

    TxAXIQ(true);
    axiq_fifo_tx_cr(AXIQ_BANK_0, (enum axiq_fifo_e)dac->axi_fifo_index, AXIQ_CR_CLRERR, AXIQ_CR_CLRERR);
    axiq_fifo_tx_cr(AXIQ_BANK_0, (enum axiq_fifo_e)dac->axi_fifo_index, AXIQ_CR_CLRERR, 0);

    // dac_prime(&dac[lane]);
}

void tx_lane_ddr_enable(uint16_t lane, bool enable) {
    if (enable)
        tx_lane_prime(lane);
}

// Aborts any pending transfers and resets the pipeline's AXIQ FIFO
void tx_lane_abort(uint16_t lane) {
    tx_drop_ddr(lane);
    tx_axiq_fifo_reset(lane);
}

int tx_tone_enable(uint16_t lane, bool enable) {
    if (txpipe[lane].generate_tone == enable)
        return 0;
    txpipe[lane].generate_tone = enable;

    if (enable) {
        // Enable trigger for proper AXIQ reset
        timer_trig_immediate(PHY_TIMER_COMP_CH5_TX_ALLOWED, ePhyTimerComparatorOut1);

        tx_lane_prime(lane);

        timer_trig_immediate(PHY_TIMER_COMP_CH5_TX_ALLOWED, ePhyTimerComparatorOut0);
        // initial work twice to produce two initial buffers
        // tx_pipeline_work(lane);
        // tx_pipeline_work(lane);
        timer_trig_immediate(PHY_TIMER_COMP_CH5_TX_ALLOWED, ePhyTimerComparatorOut1);
    } else {
        tx_lane_abort(lane);
        timer_trig_immediate(PHY_TIMER_COMP_CH5_TX_ALLOWED, ePhyTimerComparatorOut0);
    }
    return 0;
}

void host_submitted_tx_tcd(uint16_t lane) {
    if (txddr[lane].count_dmac_enque - txddr[lane].buffers_consumed >= MAX_DMA_ENQ)
        return; // skip, all ddr buffers are in use

    // enque two initial buffers
    if (txddr[0].count_dmac_enque - txddr[0].buffers_consumed < MAX_DMA_ENQ)
        tx_lane_try_ddr_enqueue(&txddr[lane], true);
    if (txddr[0].count_dmac_enque - txddr[0].buffers_consumed < MAX_DMA_ENQ)
        tx_lane_try_ddr_enqueue(&txddr[lane], true);
}

void tx_check_axiq_status(void) {
    // Check AXIQ tx fifo is not full or overrun
    uint32_t status = axiq_fifo_tx_sr(AXIQ_BANK_0, AXIQ_FIFO_TX0, AXIQ_SR_FIELD_ERROVER | AXIQ_SR_FIELD_ERRUNDER);
    if (status == 0)
        return;

    const uint8_t field_shift = axiq_sr_shift(AXIQ_FIFO_TX0);
    status >>= field_shift;
    if (status & AXIQ_SR_FIELD_ERROVER) {
        ++tx_stats.afe_ovr;
        TRACE_COUNTER(CNT_TX_AFE_OVR, tx_stats.afe_ovr);
    }
    if (status & AXIQ_SR_FIELD_ERRUNDER) {
        ++tx_stats.afe_udr;
        TRACE_COUNTER(CNT_TX_AFE_UDR, tx_stats.afe_udr);
        tx_handle_dac_underrun();
        return;
    }
    axiq_fifo_tx_cr(AXIQ_BANK_0, AXIQ_FIFO_TX0, AXIQ_CR_CLRERR, AXIQ_CR_CLRERR);
    axiq_fifo_tx_cr(AXIQ_BANK_0, AXIQ_FIFO_TX0, AXIQ_CR_CLRERR, 0);
}

void deffer_next_tx_burst(uint32_t phytime) {
    txddr[0].deffered = true;
    timer_trig_schedule(PHY_TIMER_COMP_VSPA_GO_1, ePhyTimerComparatorOut1, phytime);
}

void tx_deffered_start(void) {
    const uint16_t lane = 0;
    tx_ddr_pipeline_t *const ddr = &txddr[lane];
    if (!ddr->deffered)
        return;
    TRACE_EVENT(T_DAC_AXIQ_RST, 4, 1);

    ddr->deffered = false;
    if (ddr->count_dmac_enque - ddr->buffers_consumed >= MAX_DMA_ENQ)
        return;

    timer_trig_immediate_async(PHY_TIMER_COMP_VSPA_GO_1, ePhyTimerComparatorOut0);
    tx_initial_ddr_enque(lane);
}