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
#define MEASUREMENT_INTERVAL 25000 // 25000   = 50ms@500kHz
#define NUM_SAMPLES 10

#define HIGH_BOUDRY_FINE 30
#define LOW_BOUDRY_FINE 2
#define CANDIDATE_SETTING_OFFSET_RX(nextOrPrev) (nextOrPrev * (32 - 6))
#define CANDIDATE_SETTING_OFFSET_TX(nextOrPrev) (nextOrPrev * (32 - 6))
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
	uint8_t inter_calibration_timer_done;
	uint16_t tx_setting;
	uint16_t rx_setting;
} app_vars_t;

app_vars_t app_vars;

//=========================== prototypes ======================================

void cb_timer(void);

uint32_t average_sample(void);
void update_configuration(void);
void inter_calibrate_RX_setting(void);
void inter_calibrate_TX_setting(void);
void lc_setting_edge_detection(uint16_t* setting, uint8_t txOrRx);

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
	app_vars.rx_setting = ((23 & 0x001F) << 10) + ((1 & 0x001F) << 5) + (1 & 0x001F);
	app_vars.tx_setting = ((23 & 0x001F) << 10) + ((1 & 0x001F) << 5) + (1 & 0x001F);
    printf("Calibrating frequencies...\r\n");
	while(1) {
		inter_calibrate_RX_setting();
		printf("-----------------------------\r\n");
		app_vars.inter_calibration_timer_done = 0;
		rftimer_setCompareIn(rftimer_readCounter() + MEASUREMENT_INTERVAL * 2);
		while(app_vars.inter_calibration_timer_done == 0){};
		//inter_calibrate_TX_setting();
	}
}

//=========================== public ==========================================

//=========================== private =========================================
void inter_calibrate_RX_setting(void)
{
	uint32_t count_2M;
    uint32_t count_LC;
    uint32_t count_adc;
	uint32_t count_LC_RX_measured;
	uint32_t count_2M_RC_measured;
	int32_t adjustment_LC_RX;
	int32_t tmp;
	uint32_t RC2M_coarse;
    uint32_t RC2M_fine;
    uint32_t RC2M_superfine;
	int32_t A;
	int32_t B;
	int32_t C;
	
	int32_t freq_distance;
	
	// start a timer
	app_vars.inter_calibration_timer_done = 0;
	rftimer_setCompareIn(rftimer_readCounter() + MEASUREMENT_INTERVAL);
	
	// only for resetting the counters
    read_counters_3B(&count_2M, &count_LC, &count_adc);
	radio_rxEnable();
	// waiting for the timer to end
	while(app_vars.inter_calibration_timer_done == 0){};
	
	// read the counters again
    read_counters_3B(&count_2M, &count_LC, &count_adc);
	printf("RX LC: %d, 2M: %d\r\n", count_LC, count_2M);
	RC2M_coarse = scm3c_hw_interface_get_RC2M_coarse();
    RC2M_fine = scm3c_hw_interface_get_RC2M_fine();
    RC2M_superfine = scm3c_hw_interface_get_RC2M_superfine();
	// simplified
	count_LC_RX_measured = count_LC;
	count_2M_RC_measured = count_2M;
	
	freq_distance = (count_2M_RC_measured - 100000) * 20;
	freq_distance /= 1930;
	printf("adjust_2M: %d\r\n", freq_distance);
	//freq_distance = (count_LC_RX_measured - 125000) * 20 / 8;
		
		
	// beware of overflow
	// adjustment_2M_RC_mid_simplified = (A - B) / C
	A = count_LC_RX_measured * 2 * 960;
	B = 2405 * count_2M_RC_measured; // MHz * Hz
		
	// C = count_LC_RX_measured * 1930 * 960 / 1000 / 1000 ===> C = count_LC_RX_measured * 193 * 96 / 10000
    // ===> C = count_LC_RX_measured * 193 * 96 / 10000  ===> 2316 / 1250 ===> 1158 / 625
	C = 8 * count_2M_RC_measured / 100; // MHz * Hz
	
	// * 1486 * 96 / 100 / 1000 ===> 4458 / 3125 ===> 743 * 6 / 125 / 25
	// 15 * 96 / 1000 ===> 36 / 25
	// C = count_LC_RX_measured * 36 / 5 ; // MHz * Hz
	
	
	// printf("A: %d, B: %d, C: %d\r\n", A, B, C);
	adjustment_LC_RX = (A - B) / C;
	printf("adjust_LC_RX: %d\r\n", adjustment_LC_RX);
	
	
	RC2M_fine += (freq_distance);
	if (RC2M_fine >= 32) {
		RC2M_coarse += RC2M_fine / 32;
		RC2M_fine %= RC2M_fine;
	}
	printf("2M setting: %d.%d.%d\r\n", RC2M_coarse, RC2M_fine, RC2M_superfine);
	set_2M_RC_frequency(31, 31, RC2M_coarse, RC2M_fine, RC2M_superfine);

    scm3c_hw_interface_set_RC2M_coarse(RC2M_coarse);
    scm3c_hw_interface_set_RC2M_fine(RC2M_fine);
    scm3c_hw_interface_set_RC2M_superfine(RC2M_superfine);
		
    analog_scan_chain_write();
    analog_scan_chain_load();
	
	app_vars.rx_setting -= adjustment_LC_RX;
	lc_setting_edge_detection(&app_vars.rx_setting, 0);
	printf("RX setting:%d.%d.%d\r\n", (app_vars.rx_setting >> 10) & 0x001f,
										(app_vars.rx_setting >> 5) & 0x001f,
										(app_vars.rx_setting) & 0x001f);
}

void inter_calibrate_TX_setting(void)
{
	uint32_t count_2M;
    uint32_t count_LC;
    uint32_t count_adc;
	uint32_t count_LC_TX_measured;
	uint32_t count_2M_RC_measured;
	int32_t adjustment_LC_TX;
	int32_t tmp;
	uint32_t RC2M_coarse;
    uint32_t RC2M_fine;
    uint32_t RC2M_superfine;
	int32_t A;
	int32_t B;
	int32_t C;
	
	int32_t freq_distance;
	
	// start a timer
	app_vars.inter_calibration_timer_done = 0;
	rftimer_setCompareIn(rftimer_readCounter() + MEASUREMENT_INTERVAL);
	
	// only for resetting the counters
    read_counters_3B(&count_2M, &count_LC, &count_adc);
	radio_txEnable();
	// waiting for the timer to end
	while(app_vars.inter_calibration_timer_done == 0){};
	
	// read the counters again
    read_counters_3B(&count_2M, &count_LC, &count_adc);
	printf("TX LC: %d, 2M: %d\r\n", count_LC, count_2M);
	RC2M_coarse = scm3c_hw_interface_get_RC2M_coarse();
    RC2M_fine = scm3c_hw_interface_get_RC2M_fine();
    RC2M_superfine = scm3c_hw_interface_get_RC2M_superfine();
	// simplified
	count_LC_TX_measured = count_LC;
	count_2M_RC_measured = count_2M;
	
	freq_distance = (count_2M_RC_measured - 100000) * 20;
	freq_distance /= 1930;
		
		
	// beware of overflow
	// adjustment_2M_RC_mid_simplified = (A - B) / C
	A = count_LC_TX_measured * 2 * 960;
	B = 2405 * count_2M_RC_measured; // MHz * Hz
		
	// C = count_LC_RX_measured * 1930 * 960 / 1000 / 1000 ===> C = count_LC_RX_measured * 193 * 96 / 10000
    // ===> C = count_LC_RX_measured * 193 * 96 / 10000  ===> 2316 / 1250 ===> 1158 / 625
	C = 8 * count_2M_RC_measured / 100; // MHz * Hz
	
	// * 1486 * 96 / 100 / 1000 ===> 4458 / 3125 ===> 743 * 6 / 125 / 25
	// 15 * 96 / 1000 ===> 36 / 25
	// C = count_LC_RX_measured * 36 / 5 ; // MHz * Hz
	
	
	// printf("A: %d, B: %d, C: %d\r\n", A, B, C);
	adjustment_LC_TX = (A - B) / C;
	printf("adjust_LC_TX: %d\r\n", adjustment_LC_TX);
	
	
	RC2M_fine += (freq_distance);
	
	set_2M_RC_frequency(31, 31, RC2M_coarse, RC2M_fine, RC2M_superfine);

    scm3c_hw_interface_set_RC2M_coarse(RC2M_coarse);
    scm3c_hw_interface_set_RC2M_fine(RC2M_fine);
    scm3c_hw_interface_set_RC2M_superfine(RC2M_superfine);
		
    analog_scan_chain_write();
    analog_scan_chain_load();
	
	app_vars.tx_setting -= adjustment_LC_TX;
	lc_setting_edge_detection(&app_vars.tx_setting, 1);
	printf("TX setting:%d.%d.%d\r\n", (app_vars.tx_setting >> 10) & 0x001f,
										(app_vars.tx_setting >> 5) & 0x001f,
										(app_vars.tx_setting) & 0x001f);
}

void lc_setting_edge_detection(uint16_t* setting, uint8_t txOrRx) {
    uint8_t i;
    int8_t move_mid_nextORprev;
    int8_t offset;

    // 0: do nothing 1: mid next -1: move previous
    move_mid_nextORprev = 0;
    if ((setting[i] & 0x001f) >= HIGH_BOUDRY_FINE) {
        move_mid_nextORprev = 1;
    } else {
        if ((setting[i] & 0x001f) <= LOW_BOUDRY_FINE) {
            move_mid_nextORprev = -1;
        }
    }

    if (move_mid_nextORprev != 0) {
        // txOrRx 0: RX  1: TX

        if (txOrRx == 0) {
            offset = CANDIDATE_SETTING_OFFSET_RX(move_mid_nextORprev);
        } else {
            offset = CANDIDATE_SETTING_OFFSET_TX(move_mid_nextORprev);
        }

        if (i == 0) {
            setting[0] = setting[0] + offset;
        } else {
            setting[1] = setting[0] + offset;
        }

        printf("setting[0] %d %d %d | setting[1] %d %d %d | offset %d\r\n",
               (setting[0] >> 10) & 0x001f, (setting[0] >> 5) & 0x001f,
               (setting[0]) & 0x001f, (setting[1] >> 10) & 0x001f,
               (setting[1] >> 5) & 0x001f, (setting[1]) & 0x001f, offset);
    }
}


void cb_timer(void) {
	app_vars.inter_calibration_timer_done = 1;
}
