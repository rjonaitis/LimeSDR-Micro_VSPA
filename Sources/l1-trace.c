// SPDX-License-Identifier: BSD-3-Clause
// Copyright 2026 Lime Microsystems

#include "l1-trace.h"

#include "ccnt.h"
#include "dma_common.h"
#include "dmac.h"
#include "vspa_iqstream.h"

void l1_trace_init(void) {
    ccnt_disable();
    ccnt_reset();
    ccnt_enable();
    l1_trace_clear();
}

#if TRACE_ENABLED

#define L1_TRACE_CAPACITY (128)

#define L1_TRACE_DMA_CHANNEL DDR_WR_DMA_CHANNEL_4

l1_trace_hif_t trace_hif = { 0, 0, 0, 0, 0 };

// double buffer, fill one while another is DMA tranferred
l1_trace_data_t events_buffer[2][L1_TRACE_CAPACITY] _VSPA_VECTOR_ALIGN __attribute__((section(".vcpu_dmem")));
uint16_t events_active_buffer = 0;
uint16_t trace_last_batch_size = 0;
l1_trace_data_t *next_event = NULL;

void l1_trace_clear(void) {
    dmac_abort((1 << L1_TRACE_DMA_CHANNEL));

    next_event = events_buffer[0];
    events_active_buffer = 0;

    trace_hif.bytes_produced = 0;
    trace_hif.event_count = 0;
    trace_hif.event_drops = 0;
    WAIT_FOR(!dmac_is_running(1 << L1_TRACE_DMA_CHANNEL), 5000);

    dmac_clear_complete((1 << L1_TRACE_DMA_CHANNEL));
    dmac_clear_event((1 << L1_TRACE_DMA_CHANNEL));
    dmac_clear_errcfg((1 << L1_TRACE_DMA_CHANNEL));
    dmac_clear_errxfr((1 << L1_TRACE_DMA_CHANNEL));
}

void push_traces(void) {
#if TRACE_ENABLED
    TRACE_START_DURATION(t1);

    check_l1_trace_complete();
    if (trace_hif.la9310_mem_address == 0)
        return;

    const uint32_t xfer_size = ((uint32_t)next_event - (uint32_t)events_buffer[events_active_buffer]) << 1;
    if (xfer_size < (L1_TRACE_CAPACITY / 4) * 16)
        return;

    // only 1 transfer is queued up, wait for complete stop of the dma
    if (dmac_is_enabled((1 << L1_TRACE_DMA_CHANNEL))) {
        ++trace_hif.event_drops;
        return;
    }

    trace_last_batch_size = xfer_size;
    dmac_enable(DMAC_WR | L1_TRACE_DMA_CHANNEL, xfer_size,
                trace_hif.la9310_mem_address + (trace_hif.bytes_produced % trace_hif.buffer_size),
                VSPA_HALF_WORDS(events_buffer[events_active_buffer]));

    // swap active buffer
    events_active_buffer ^= 0x1;
    next_event = events_buffer[events_active_buffer];

    TRACE_DURATION(T_TRACE_PUSH, 1, t1);
#endif
}

void check_l1_trace_complete(void) {
#if TRACE_ENABLED
    if (!dmac_is_complete(1 << L1_TRACE_DMA_CHANNEL))
        return;

    dmac_clear_complete(1 << L1_TRACE_DMA_CHANNEL);
    // dmac_clear_event((1 << L1_TRACE_DMA_CHANNEL)); // not used

    trace_hif.bytes_produced += trace_last_batch_size;
#endif
}

#endif
