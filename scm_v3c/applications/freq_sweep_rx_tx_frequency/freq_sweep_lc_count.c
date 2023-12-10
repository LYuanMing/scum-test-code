/**
\brief This program conducts a freqency sweeping test.

After loading this program, SCuM will turn on radio to listen on each
configuration of coarse[5 bits], middle[5 bits] and fine[5 bits] for 15ms.
Meanwhile, an OpenMote will send NUMPKT_PER_CFG packet on one channel every
4ms. SCuM will print out the pkt it received at each config settings.

This program supposes to be run 16 times on for each channel setting on OpenMote
side.
*/

#include <string.h>

#include "memory_map.h"
#include "optical.h"
#include "radio.h"
#include "rftimer.h"
#include "scm3c_hw_interface.h"

//=========================== defines =========================================

#define CRC_VALUE (*((unsigned int*)0x0000FFFC))
#define CODE_LENGTH (*((unsigned int*)0x0000FFF8))

#define TIMER_PERIOD 50000  ///< 25000 = 50ms@500kHz
#define STEPS_PER_CONFIG 32

#define NUM_SAMPLES 10

// measure the LC_count at RX state by default
// uncomment the following macro to change to measure at TX state
// #define FREQ_SWEEP_TX

//=========================== variables =======================================

typedef struct {
    volatile uint8_t sample_index;
    uint32_t samples[NUM_SAMPLES];

    uint8_t cfg_coarse;
    uint8_t cfg_mid;
    uint8_t cfg_fine;
	
	uint8_t timer_expire;
} app_vars_t;

app_vars_t app_vars;

//=========================== prototypes ======================================

void cb_timer(void);

uint32_t average_sample(void);
void update_configuration(void);

//=========================== main ============================================

int main(void) {
    uint32_t calc_crc;

    uint8_t i;
    uint8_t j;
    uint8_t offset;
	uint32_t count_2M;
    uint32_t count_LC;
    uint32_t count_adc;
	uint32_t restart_flag;
	
    memset(&app_vars, 0, sizeof(app_vars_t));

    printf("Initializing...");

    // Set up mote configuration
    // This function handles all the analog scan chain setup
    initialize_mote();

    rftimer_set_callback(cb_timer);

    // Disable interrupts for the radio and rftimer
    rftimer_disable_interrupts();

    // Check CRC to ensure there were no errors during optical programming
    printf("\r\n-------------------\r\n");
    printf("Validating program integrity...");

    calc_crc = crc32c(0x0000, CODE_LENGTH);

    if (calc_crc == CRC_VALUE) {
        printf("CRC OK\r\n");
    } else {
        printf(
            "\r\nProgramming Error - CRC DOES NOT MATCH - Halting "
            "Execution\r\n");
        while (1)
            ;
    }

    // Debug output
    // printf("\r\nCode length is %u bytes",code_length);
    // printf("\r\nCRC calculated by SCM is: 0x%X",calc_crc);

    // printf("done\r\n");

    // After bootloading the next thing that happens is frequency calibration
    // using optical
    printf("Calibrating frequencies...\r\n");

    // Initial frequency calibration will tune the frequencies for HCLK, the
    // RX/TX chip clocks, and the LO
// #define FREQ_SWEEP_TX
#ifdef FREQ_SWEEP_TX
    radio_txEnable();
#else
    // For the LO, calibration for RX channel 11, so turn on AUX, IF, and LO
    // LDOs by calling radio rxEnable
    radio_rxEnable();
#endif

#define CAL_RC_OSC
#ifndef CAL_RC_OSC
	restart_flag = 1;
    while (1) {
		for (app_vars.cfg_coarse = 23; app_vars.cfg_coarse < 25; app_vars.cfg_coarse++) {
            for (app_vars.cfg_mid = 0; app_vars.cfg_mid <= 32; app_vars.cfg_mid++) {
                for (app_vars.cfg_fine = 0; app_vars.cfg_fine <= 32; app_vars.cfg_fine++) {
						if(restart_flag == 1) {
							app_vars.cfg_coarse = 23;
							app_vars.cfg_mid = 29;
							app_vars.cfg_fine = 9;
							restart_flag = 0;
						}
						 printf("setting: %d.%d.%d \r\n", app_vars.cfg_coarse, app_vars.cfg_mid, app_vars.cfg_fine);
						 LC_FREQCHANGE(app_vars.cfg_coarse, app_vars.cfg_mid, app_vars.cfg_fine);
						 // repeat 100 times
						 for (i = 0; i < 100; i++) {
							 app_vars.timer_expire = 0;
							 // only for resetting the counters
							 read_counters_3B(&count_2M, &count_LC, &count_adc);
							 rftimer_setCompareIn(rftimer_readCounter() + TIMER_PERIOD);
							 while (app_vars.timer_expire == 0);
						 }
                } 
            }
        }
    }
#else
	restart_flag = 1;
    while (1) {
		for (app_vars.cfg_coarse = 27; app_vars.cfg_coarse < 32; app_vars.cfg_coarse++) {
            for (app_vars.cfg_mid = 0; app_vars.cfg_mid <= 32; app_vars.cfg_mid++) {
                for (app_vars.cfg_fine = 0; app_vars.cfg_fine <= 32; app_vars.cfg_fine++) {
						if(restart_flag == 1) {
							app_vars.cfg_coarse = 28;
							app_vars.cfg_mid = 25;
							app_vars.cfg_fine = 13;
							restart_flag = 0;
						}
						printf("setting: %d.%d.%d \r\n", app_vars.cfg_coarse, app_vars.cfg_mid, app_vars.cfg_fine);
						
						set_2M_RC_frequency(31, 31, app_vars.cfg_coarse, app_vars.cfg_mid, app_vars.cfg_fine);

						scm3c_hw_interface_set_RC2M_coarse(app_vars.cfg_coarse);
						scm3c_hw_interface_set_RC2M_fine(app_vars.cfg_mid);
						scm3c_hw_interface_set_RC2M_superfine(app_vars.cfg_fine);
							
						analog_scan_chain_write();
						analog_scan_chain_load();
						
						 // repeat 100 times
						 for (i = 0; i < 50; i++) {
							 app_vars.timer_expire = 0;
							 // only for resetting the counters
							 read_counters_3B(&count_2M, &count_LC, &count_adc);
							 rftimer_setCompareIn(rftimer_readCounter() + TIMER_PERIOD);
							 while (app_vars.timer_expire == 0);
						 }
                } 
            }
        }
    }
#endif
}

//=========================== public ==========================================

//=========================== private =========================================


void cb_timer(void) {

    uint32_t count_2M;
    uint32_t count_LC;
    uint32_t count_adc;

    read_counters_3B(&count_2M, &count_LC, &count_adc);
	printf("count_2M: %d, count_LC: %d \r\n", count_2M, count_LC);
	app_vars.timer_expire = 1;
}
