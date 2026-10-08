// SPDX-License-Identifier: BSD-3-Clause
// Copyright 2026 Lime Microsystems

#ifndef TRANSMITTER_H
#define TRANSMITTER_H

#include "iqplayer_commands.h"
#include "vcpu.h"
#include "vspa_dma_hif.h"
#include "tone_generator.h"

#include <stdint.h>

typedef struct TxBufferMetaData {
    uint32_t phytime;
    uint32_t flags;
} tx_meta_t;

typedef struct TxDDR_lane {
    vspa_dma_hif_t dma_hif;
    cfixed16_t *base_buffer;
    cfixed16_t *ready_buffer;
    dma_tcd_fifo_t tcd_table;
    tx_meta_t meta[2];
    uint32_t count_dmac_enque;
    uint32_t count_dmac_complete;
    uint16_t dma_channel;
    uint32_t buffers_consumed;
    uint16_t ready_buffer_offset;
    bool deffered;
} tx_ddr_pipeline_t;

typedef struct DAC_lane {
    cfixed16_t *base_buffer;
    uint32_t start_phytime;
    uint32_t buffers_provided;
    uint32_t count_dmac_enque;
    uint32_t count_dmac_complete;
    tx_meta_t meta[2];

    uint32_t axi_fifo_addr;
    uint16_t axi_fifo_index;
    uint16_t dma_channel;
    bool reschedule_start;
} dac_pipeline_t;

typedef struct TxPipeline {
    bool generate_tone;
} tx_pipeline_t;

typedef struct TxTDD_Config {
    uint16_t rf_sw_active_trigger;
    int16_t dac_allowed;
    int16_t pa_on;
    int16_t pa_off;
    int16_t rf_sw_on;
    int16_t rf_sw_off;
} tx_tdd_config_t;

#define TX_MAX_LANE_COUNT 1

extern tx_ddr_pipeline_t txddr[TX_MAX_LANE_COUNT];
extern tone_state_t tx_tone_state[TX_MAX_LANE_COUNT];

void transmitter_init(void);
void tx_lane_setup(uint16_t lane, uint16_t channel);
int tx_set_oversampling(uint16_t lane, uint16_t oversample_pow2);

void tx_lane_ddr_enable(uint16_t lane, bool enable);

void tx_lane_prime(uint16_t lane);
void tx_lane_abort(uint16_t lane);

void dac_dma_complete(uint16_t lane);
void tx_ddr_complete(uint16_t lane);
void tx_lane_try_ddr_enqueue(tx_ddr_pipeline_t *ddr, bool vcpu_trig);

int tx_tone_enable(uint16_t lane, bool enable);
void host_submitted_tx_tcd(uint16_t lane);
void tx_check_axiq_status(void);

void deffer_next_tx_burst(uint32_t phytime);
void tx_deffered_start(void);

#endif /* IQMOS_RX_H_ */
