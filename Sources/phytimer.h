#ifndef LIME_TIMER_CONTROL_H
#define LIME_TIMER_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#define PHY_TIMER_COMP_VSPA_GO_0 0
#define PHY_TIMER_COMP_CH1_RX_ALLOWED 1
#define PHY_TIMER_COMP_CH2_RX_ALLOWED 2
#define PHY_TIMER_COMP_CH3_RX_ALLOWED 3
#define PHY_TIMER_COMP_CH4_RX_ALLOWED 4
#define PHY_TIMER_COMP_CH5_RX_ALLOWED 5
#define PHY_TIMER_COMP_CH6_RX_ALLOWED 6
#define PHY_TIMER_COMP_CH5_TX_ALLOWED 11
#define PHY_TIMER_COMP_VSPA_GO_1 12
#define PHY_TIMER_COMP_PPS_IN 13
#define PHY_TIMER_COMP_PPS_OUT 14
#define PHY_TIMER_COMP_RFCTL_0 15
#define PHY_TIMER_COMP_RFCTL_1 16
#define PHY_TIMER_COMP_RFCTL_2 17
#define PHY_TIMER_COMP_RFCTL_3 18
#define PHY_TIMER_COMP_RFCTL_4 19
#define PHY_TIMER_COMP_RFCTL_5 20

enum ePhyTimerComparatorTrigger {
    ePhyTimerComparatorNoChange = 0x0,
    /** Comparator output signal set to '0' */
    ePhyTimerComparatorOut0,
    /** Comparator output signal set to '1' */
    ePhyTimerComparatorOut1,
    /** Comparator output signal is toggled */
    ePhyTimerComparatorOutToggle,
};

void phy_tmr_enable(void);
void phy_tmr_configure(void);

uint16_t timer_trig_immediate(uint32_t id, enum ePhyTimerComparatorTrigger trigger);
uint32_t timer_trig_immediate_async(uint32_t id, enum ePhyTimerComparatorTrigger trigger);
uint16_t timer_trig_schedule(uint32_t id, enum ePhyTimerComparatorTrigger trigger, uint32_t timestamp);
uint16_t timer_trig_schedule_async(uint32_t id, enum ePhyTimerComparatorTrigger trigger, uint32_t timestamp);

void check_timer_dma(void);
void stream_trig_schedule_async(enum ePhyTimerComparatorTrigger trigger, uint32_t timestamp);

#endif // LIME_TIMER_CONTROL_H