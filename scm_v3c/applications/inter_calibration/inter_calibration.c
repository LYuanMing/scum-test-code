/**
\brief This program conducts continuously calibration over temperature changes.

After loading this program, SCuM will do the following:
- sweep frequency settings to get the RX setting
- sweep frequency settings to get the TX setting
- using the TX and RX setting to transmit frame to OpenMote and listen for ACK

Temperature will change during the last phase.
This program allows SCuM to continuously calibrate its frequency according to
the frequency offset for TX in ACK and the IF counts for RX.

This calibration only applies on on signel channel, e.g. channel 11.

*/

#include <string.h>

#include "freq_setting_selection.h"
#include "gpio.h"
#include "memory_map.h"
#include "optical.h"
#include "radio.h"
#include "rftimer.h"
#include "scm3c_hw_interface.h"

//=========================== defines =========================================

#define CRC_VALUE (*((unsigned int*)0x0000FFFC))
#define CODE_LENGTH (*((unsigned int*)0x0000FFF8))

#define RX_TIMEOUT 500       // 500 = 1ms@500kHz
#define RX_ACK_TIMEOUT 1500  // 500 = 1ms@500kHz

#define SWEEP_START ((23 << 10) | (0 << 5) | (0))
#define SWEEP_END ((23 << 10) | (31 << 5) | (31))

#define SETTING_SIZE 100
#define BEACON_PERIOD 20        // seconds
#define SECOND_IN_TICKS 500000  // 500000 = 1s@500kHz
#define SENDING_INTERVAL 50000  // 50000  = 100ms@500kHz
#define MEASUREMENT_INTERVAL 25000 // 25000   = 50ms@500kHz
#define TICKS_OF_PERIODICAL_BEACON 150000 // 50000 = 100ms@500kHz
#define MS_IN_TICKS 500 // 500 = 1ms@500kHz

#define MAX_PKT_SIZE 125 + LENGTH_CRC
#define TARGET_PKT_SIZE 5 + LENGTH_CRC

#define HISTORY_SAMPLE_SIZE 10

#define ACK_IN_MS 30
#define TICK_IN_MS 500

// when nextOrPrev ==  1: move to next     mid setting
// when nextOrPrev == -1: move to previous mid setting
#define CANDIDATE_SETTING_OFFSET_RX(nextOrPrev) (nextOrPrev * (32 - 6))
#define CANDIDATE_SETTING_OFFSET_TX(nextOrPrev) (nextOrPrev * (32 - 6))
#define SWITCH_COUNTER_MASK 0x01
#define NUM_CANDIDATE_SETTING 2
#define DEFAULT_SETTING 0
#define ALTERNATIVE_SETTING 1

#define HIGH_BOUDRY_FINE 30
#define LOW_BOUDRY_FINE 2

#define TARGET_2M_COUNT 60000  // 60000  = 30ms@2MHz

#define MHz_2_kHz(MHz) ((MHz) * 1000)
#define kHz_2_Hz(kHz) ((kHz) * 1000)
#define MHz_2_Hz(MHz) ((MHz) * 1000000)

// frequency difference in 2M RC coarse code is about 10.860621093749996  KHz
// frequency difference in 2M RC fine code is about 1.597589717741936  KHz
#define Hz_2M_RC_per_fine_code 1597
#define Hz_2M_RC_per_coarse_code 10860

// frequency difference in TX coarse code is about 14663.148989610949  KHz
// frequency difference in TX middle code is about 794.8616541470408  KHz
// frequency difference in TX find code is about 130.90244749698303  KHz
#define Hz_TX_LC_per_fine_code 130902
#define Hz_TX_LC_per_middle_code 794861
#define Hz_TX_LC_per_coarse_code 14663148

//=========================== variables =======================================

typedef enum {
    SWEEP_RX = 0,
    SWEEP_RX_DONE = 1,
    SWEEP_TX = 2,
    SWEEP_TX_DONE = 3,
	SYNC_TIMER = 4,
    CONTINUOUSLY_CAL = 5,
} state_t;

typedef struct {
    // setting for tx
    uint16_t tx_settings_list[SETTING_SIZE];
    int8_t tx_settings_freq_offset_list[SETTING_SIZE];
    uint32_t count_2m_list[SETTING_SIZE];
    uint16_t tx_list_index;

    // setting for rx
    uint16_t rx_settings_list[SETTING_SIZE];
    uint16_t rx_settings_if_count_list[SETTING_SIZE];
    uint16_t rx_list_index;

    // setting for 2m clock
    uint32_t count_2M;
    uint32_t count_LC;
    uint32_t count_adc;

    uint16_t current_setting;

    // stats
    state_t state;
    uint8_t rx_done;
    uint8_t tx_done;
    uint8_t schedule_rx_ack_or_rx_timeout;
	uint8_t inter_calibration_flag;
	uint8_t inter_calibration_timer_done;
	uint8_t periodical_timer_expired;
	uint8_t retry_count;

    // for sync
    uint8_t beacon_stops_in;  // in seconds

    // for continuously calibration
    uint32_t if_history[HISTORY_SAMPLE_SIZE];
    int8_t fo_history[HISTORY_SAMPLE_SIZE];
    uint32_t count_2m_history[HISTORY_SAMPLE_SIZE];
    uint8_t history_index;
    uint32_t target_count_2m;

    // last temperature
    uint16_t last_temperature;

	// last count of receving
	uint32_t receving_count;

    // candidate settings
    uint16_t tx_setting_candidate[NUM_CANDIDATE_SETTING];
    uint16_t rx_setting_candidate[NUM_CANDIDATE_SETTING];
    uint8_t switch_counter;
    uint8_t setting_index;

} app_vars_t;

app_vars_t app_vars;
uint16_t retry_index = 0;
volatile uint32_t freq;
uint32_t packet_idx = 0;
uint8_t DYNAMIC_SAMPLE_SIZE = 1;
//=========================== prototypes ======================================

void cb_timer(void);
void cb_startFrame_tx(uint32_t timestamp);
void cb_endFrame_tx(uint32_t timestamp);
void cb_startFrame_rx(uint32_t timestamp);
void cb_endFrame_rx(uint32_t timestamp);

void getFrequencyTx(uint16_t setting_start, uint16_t setting_end);
void getFrequencyRx(uint16_t setting_start, uint16_t setting_end);
void contiuously_calibration_start(void);
void update_target_settings(void);
void switch_lc_setting(void);
void lc_setting_edge_detection(uint16_t* settings, uint8_t txOrRx);

void delay_turnover(void);
void delay_tx(void);
void delay_lc_setup(void);

void sync_timer(void);
void inter_calibrate_2M_setting(void);
void inter_calibrate_Tx_setting(void);
void send_ack(void);
void update_rc_setting(int32_t frequency_difference_RC_2M, uint32_t RC2M_coarse, uint32_t RC2M_fine, uint32_t RC2M_superfine);
void update_tx_setting(int32_t frequency_difference_TX_LC);
//=========================== main ============================================

int main(void) {
    uint32_t calc_crc;

    uint8_t i;
    uint8_t j;
    uint8_t offset;

    memset(&app_vars, 0, sizeof(app_vars_t));

    app_vars.beacon_stops_in = BEACON_PERIOD;

    printf("Initializing...");

    // Set up mote configuration
    // This function handles all the analog scan chain setup
    initialize_mote();

    rftimer_set_callback(cb_timer);
    radio_setStartFrameTxCb(cb_startFrame_tx);
    radio_setEndFrameTxCb(cb_endFrame_tx);
    radio_setStartFrameRxCb(cb_startFrame_rx);
    radio_setEndFrameRxCb(cb_endFrame_rx);

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

    // For the LO, calibration for RX channel 11, so turn on AUX, IF, and LO
    // LDOs by calling radio rxEnable
    radio_rxEnable();

    // Enable optical SFD interrupt for optical calibration
    optical_enable();

    // Wait for optical cal to finish
    while (optical_getCalibrationFinshed() == 0)
        ;

    printf("Cal complete\r\n");

    app_vars.target_count_2m = TARGET_2M_COUNT;

    app_vars.state = SWEEP_RX;

    while (1) {
        // obtain frequency setting for RX
        getFrequencyRx(SWEEP_START, SWEEP_END);

        printf("RX target setting = %d %d %d\r\n",
               (app_vars.rx_setting_candidate[DEFAULT_SETTING] >> 10) & 0x001f,
               (app_vars.rx_setting_candidate[DEFAULT_SETTING] >> 5) & 0x001f,
               (app_vars.rx_setting_candidate[DEFAULT_SETTING]) & 0x001f);

        if ((app_vars.rx_setting_candidate[DEFAULT_SETTING] & 0x001f) >= 16) {
            app_vars.rx_setting_candidate[ALTERNATIVE_SETTING] =
                app_vars.rx_setting_candidate[DEFAULT_SETTING] +
                CANDIDATE_SETTING_OFFSET_RX(1);
        } else {
            app_vars.rx_setting_candidate[ALTERNATIVE_SETTING] =
                app_vars.rx_setting_candidate[DEFAULT_SETTING] +
                CANDIDATE_SETTING_OFFSET_RX(-1);
        }

        printf(
            "RX alternative setting = %d %d %d\r\n",
            (app_vars.rx_setting_candidate[ALTERNATIVE_SETTING] >> 10) & 0x001f,
            (app_vars.rx_setting_candidate[ALTERNATIVE_SETTING] >> 5) & 0x001f,
            (app_vars.rx_setting_candidate[ALTERNATIVE_SETTING]) & 0x001f);

        // obtain frequency setting for TX
        getFrequencyTx(SWEEP_START, SWEEP_END);

        printf("TX target setting = %d %d %d\r\n",
               (app_vars.tx_setting_candidate[DEFAULT_SETTING] >> 10) & 0x001f,
               (app_vars.tx_setting_candidate[DEFAULT_SETTING] >> 5) & 0x001f,
               (app_vars.tx_setting_candidate[DEFAULT_SETTING]) & 0x001f);

        if ((app_vars.tx_setting_candidate[DEFAULT_SETTING] & 0x001f) >= 16) {
            app_vars.tx_setting_candidate[ALTERNATIVE_SETTING] =
                app_vars.tx_setting_candidate[DEFAULT_SETTING] +
                CANDIDATE_SETTING_OFFSET_RX(1);
        } else {
            app_vars.tx_setting_candidate[ALTERNATIVE_SETTING] =
                app_vars.tx_setting_candidate[DEFAULT_SETTING] +
                CANDIDATE_SETTING_OFFSET_TX(-1);
        }

        printf(
            "TX alternative setting = %d %d %d\r\n",
            (app_vars.tx_setting_candidate[ALTERNATIVE_SETTING] >> 10) & 0x001f,
            (app_vars.tx_setting_candidate[ALTERNATIVE_SETTING] >> 5) & 0x001f,
            (app_vars.tx_setting_candidate[ALTERNATIVE_SETTING]) & 0x001f);

        // start contiuously calibration
        contiuously_calibration_start();
    }
}

//=========================== public ==========================================

//=========================== callback ========================================

void cb_timer(void) {
    switch (app_vars.state) {
        case SWEEP_RX:
            app_vars.rx_done = 1;
            break;
        case SWEEP_RX_DONE:
            app_vars.state = SWEEP_TX;
            break;
        case SWEEP_TX:

            if (app_vars.schedule_rx_ack_or_rx_timeout) {
                radio_rxEnable();
                radio_rxNow();

                gpio_3_toggle();

                app_vars.schedule_rx_ack_or_rx_timeout = 0;
                rftimer_setCompareIn(rftimer_readCounter() + RX_ACK_TIMEOUT);
            } else {
                app_vars.rx_done = 1;  // rx for ack
            }
            break;
		case SYNC_TIMER:
			radio_rfOff();
			LC_FREQCHANGE((app_vars.rx_setting_candidate[DEFAULT_SETTING] >> 10) & 0x001f,
				   (app_vars.rx_setting_candidate[DEFAULT_SETTING] >> 5) & 0x001f,
				   (app_vars.rx_setting_candidate[DEFAULT_SETTING]) & 0x001f);
			
			radio_rxEnable();
			radio_rxNow();
			retry_index++;
			rftimer_setCompareIn(rftimer_readCounter() + MS_IN_TICKS);
		break;
        case CONTINUOUSLY_CAL:
			/*
            if (app_vars.schedule_rx_ack_or_rx_timeout) {
                radio_rxEnable();
                radio_rxNow();

                gpio_3_toggle();

                app_vars.schedule_rx_ack_or_rx_timeout = 0;
                rftimer_setCompareIn(rftimer_readCounter() + SENDING_INTERVAL);
            } else
			*/		
			if (app_vars.retry_count <= 6) {
				radio_rfOff();
				LC_FREQCHANGE((app_vars.rx_setting_candidate[DEFAULT_SETTING] >> 10) & 0x001f,
					   (app_vars.rx_setting_candidate[DEFAULT_SETTING] >> 5) & 0x001f,
					   (app_vars.rx_setting_candidate[DEFAULT_SETTING]) & 0x001f);
				radio_rxEnable();
                radio_rxNow();
				if (app_vars.retry_count != 6)
					rftimer_setCompareIn(rftimer_readCounter() + 5 * MS_IN_TICKS);
				else {
					rftimer_setCompareIn(app_vars.receving_count + TICKS_OF_PERIODICAL_BEACON);
					radio_rfOff();
				}
				app_vars.retry_count++;
			} else if (app_vars.inter_calibration_flag) {
				app_vars.inter_calibration_timer_done = 1;
			} else {
				//printf("timer expired\r\n");
				app_vars.retry_count = 0;
                app_vars.rx_done = 1;
				app_vars.periodical_timer_expired = 1;
				app_vars.receving_count = rftimer_readCounter() - 19;
				//printf("timer expired, %d\r\n", app_vars.receving_count);
            }
            break;
        default:
            printf("error! app_vars state = %d (code location %d)\r\n",
                   app_vars.state, 0);
            break;
    }
}

void cb_startFrame_rx(uint32_t timestamp) {
    // read the  count_2M counters
    read_counters_3B(&app_vars.count_2M, &app_vars.count_LC,
                     &app_vars.count_adc);
	app_vars.receving_count = rftimer_readCounter();
	// printf("timer count:%d\r\n", app_vars.receving_count);
}

void cb_endFrame_rx(uint32_t timestamp) {
    uint8_t pkt[MAX_PKT_SIZE];
    uint8_t pkt_len;
    int8_t rssi;
    uint8_t lqi;

    uint16_t temperature;
	int32_t adjustment;
	int32_t if_avg;
	int16_t i;

    // disable timeout interrupt
    rftimer_disable_interrupts();

    radio_getReceivedFrame(&(pkt[0]), &pkt_len, sizeof(pkt), &rssi, &lqi);
	// printf("%d\r\n", pkt_len);
    if (radio_getCrcOk() && pkt_len == TARGET_PKT_SIZE) {

        switch (app_vars.state) {
            case SWEEP_RX:

                app_vars.rx_settings_list[app_vars.rx_list_index] =
                    app_vars.current_setting;
                app_vars.rx_settings_if_count_list[app_vars.rx_list_index] =
                    radio_getIFestimate();
                app_vars.beacon_stops_in = BEACON_PERIOD - pkt[2];
                app_vars.rx_list_index++;
                break;
            case SWEEP_TX:

                app_vars.tx_settings_list[app_vars.tx_list_index] =
                    app_vars.current_setting;
                app_vars.tx_settings_freq_offset_list[app_vars.tx_list_index] =
                    (int8_t)(pkt[2]);
                app_vars.count_2m_list[app_vars.tx_list_index] =
                    app_vars.count_2M;
                app_vars.tx_list_index++;
                break;
			case SYNC_TIMER:
				break;
            case CONTINUOUSLY_CAL:
				// adjust RX according to IF
				app_vars.if_history[app_vars.history_index++] = radio_getIFestimate();
				printf("IF: %d\r\n", app_vars.if_history[app_vars.history_index - 1]);
				if (app_vars.if_history[app_vars.history_index - 1] == 0) {
					app_vars.history_index--;
				}
				if (app_vars.history_index == DYNAMIC_SAMPLE_SIZE) {
					app_vars.history_index = 0;
					for (i = 0; i < DYNAMIC_SAMPLE_SIZE; i++) {
						if_avg += app_vars.if_history[i];
					}
					if_avg /= DYNAMIC_SAMPLE_SIZE;
					adjustment = ((if_avg - 500)) / 16;
					// printf("adjustment: %d\r\n", adjustment);
					app_vars.rx_setting_candidate[DEFAULT_SETTING] += adjustment;
					lc_setting_edge_detection(app_vars.rx_setting_candidate, 0);
					LC_FREQCHANGE((app_vars.rx_setting_candidate[DEFAULT_SETTING] >> 10) & 0x001f,
								   (app_vars.rx_setting_candidate[DEFAULT_SETTING] >> 5) & 0x001f,
								   (app_vars.rx_setting_candidate[DEFAULT_SETTING]) & 0x001f);
					if (DYNAMIC_SAMPLE_SIZE < HISTORY_SAMPLE_SIZE) {
						DYNAMIC_SAMPLE_SIZE++;
					}
				}
				printf("RX setting:%d.%d.%d\r\n", (app_vars.rx_setting_candidate[DEFAULT_SETTING] >> 10) & 0x001f,
										          (app_vars.rx_setting_candidate[DEFAULT_SETTING] >> 5) & 0x001f,
										          (app_vars.rx_setting_candidate[DEFAULT_SETTING]) & 0x001f);
				
                break;
            default:
                printf("error! app_vars state = %d (code location %d)\r\n",
                       app_vars.state, 1);
                break;
        }
    }
	app_vars.rx_done = 1;
}

void cb_startFrame_tx(uint32_t timestamp) {}

void cb_endFrame_tx(uint32_t timestamp) {
    // uint32_t count_2M;
    // uint32_t count_LC;
    // uint32_t count_adc;

    // only for resetting the counters
    // read_counters_3B(&count_2M, &count_LC, &count_adc);

    // schedule when to start to listen ACK

    // app_vars.schedule_rx_ack_or_rx_timeout = 1;  // 1 = schedule_rx_ack

    // start to listen for ack 1 ms early
    // rftimer_setCompareIn(timestamp + ACK_IN_MS * TICK_IN_MS -
    //                     RX_ACK_TIMEOUT / 2);

    app_vars.tx_done = 1;
}

//=========================== delays ==========================================

// 0x2bff roughly corresponds to 17.2ms
#define TUNROVER_DELAY 0x2bff

void delay_turnover(void) {
    uint16_t i;
    for (i = 0; i < TUNROVER_DELAY; i++)
        ;
}

// 0x07ff roughly corresponds to 2.8ms
#define TX_DELAY 0x07ff

void delay_tx(void) {
    uint16_t i;
    for (i = 0; i < TX_DELAY; i++)
        ;
}

// 0x02ff roughly corresponds to 1.2ms
#define LC_SETUP_DELAY 0x02ff

void delay_lc_setup(void) {
    uint16_t i;
    for (i = 0; i < LC_SETUP_DELAY; i++)
        ;
}

//=========================== prototype========================================

//=== getFrequency

void getFrequencyRx(

    uint16_t setting_start, uint16_t setting_end) {
    uint16_t i;

    // make sure we are at SWEEP_RX state

    while (app_vars.state != SWEEP_RX)
        ;

    // sweep settings to find the ones for RX
	
    for (app_vars.current_setting = setting_start;
         app_vars.current_setting < setting_end; app_vars.current_setting++) {
        radio_rfOff();

        app_vars.rx_done = 0;

        LC_FREQCHANGE((app_vars.current_setting >> 10) & 0x001F,
                      (app_vars.current_setting >> 5) & 0x001F,
                      (app_vars.current_setting) & 0x001F);

        delay_turnover();

        delay_lc_setup();

        radio_rxEnable();
        radio_rxNow();
        rftimer_setCompareIn(rftimer_readCounter() + RX_TIMEOUT);
        while (app_vars.rx_done == 0)
            ;
    }
	
    // update state and schedule next state

    app_vars.state = SWEEP_RX_DONE;
		    rftimer_setCompareIn(rftimer_readCounter() +
                         app_vars.beacon_stops_in * SECOND_IN_TICKS);

    printf("schedule sweep Tx in %d seconds\r\n", app_vars.beacon_stops_in);

    // choose the median setting in the rx_settings_list as
    //      target rx frequency setting

    //    app_vars.rx_setting_target_main = \
//        freq_setting_selection_if(
    //            app_vars.rx_settings_list,
    //            app_vars.rx_settings_if_count_list
    //        );
	// app_vars.rx_setting_candidate[DEFAULT_SETTING] = ((23 & 0x001F) << 10) + ((13 & 0x001F) << 5) + (8 & 0x001F);
	    app_vars.rx_setting_candidate[DEFAULT_SETTING] =
        freq_setting_selection_median(app_vars.rx_settings_list);
}
		
void getFrequencyTx(uint16_t setting_start, uint16_t setting_end) {
    uint16_t i;
    int8_t diff;
    uint8_t pkt[TARGET_PKT_SIZE];

    while (app_vars.state != SWEEP_TX)
        ;

    printf("SWEEP_TX started\r\n");

    // sweep settings to find the ones for TX
	/*
    for (app_vars.current_setting = setting_start;
         app_vars.current_setting < setting_end; app_vars.current_setting++) {
        // transmit probe frame

        radio_rfOff();

        app_vars.tx_done = 0;

        pkt[0] = 'C';
        pkt[1] = 'F';
        pkt[2] = ACK_IN_MS;
        radio_loadPacket(pkt, TARGET_PKT_SIZE);
        LC_FREQCHANGE((app_vars.current_setting >> 10) & 0x001F,
                      (app_vars.current_setting >> 5) & 0x001F,
                      (app_vars.current_setting) & 0x001F);
        radio_txEnable();

        delay_tx();

        radio_txNow();
        while (app_vars.tx_done == 0)
            ;

        // listen for ack

        radio_rfOff();

        app_vars.rx_done = 0;

        LC_FREQCHANGE(
            (app_vars.rx_setting_candidate[DEFAULT_SETTING] >> 10) & 0x001F,
            (app_vars.rx_setting_candidate[DEFAULT_SETTING] >> 5) & 0x001F,
            (app_vars.rx_setting_candidate[DEFAULT_SETTING]) & 0x001F);

        while (app_vars.rx_done == 0)
            ;
    }
	
    // choose the first setting in the tx_settings_list within the threshold
    //      as the target tx frequency setting
	
    app_vars.tx_setting_candidate[DEFAULT_SETTING] =
        freq_setting_selection_fo_alternative(
            app_vars.tx_settings_list, app_vars.tx_settings_freq_offset_list);
	*/
	app_vars.tx_setting_candidate[DEFAULT_SETTING] = ((23 & 0x001F) << 10) + ((1 & 0x001F) << 5) + (1 & 0x001F);
    // calculate target count 2m
    //    i = 0;
    //    app_vars.target_count_2m = 0;
    //    while (app_vars.count_2m_list[i]!=0) {
    //        app_vars.target_count_2m += app_vars.count_2m_list[i];
    //        i++;
    //    }
    //    app_vars.target_count_2m /= i;
    //
    //    printf("target count 2M = %d\r\n",app_vars.target_count_2m);

    app_vars.state = SWEEP_TX_DONE;
}

//=== contiuously cal

void contiuously_calibration_start(void) {
    uint8_t i;
    int8_t diff;
    uint8_t pkt[TARGET_PKT_SIZE];

    uint16_t tx_setting_target;
    uint16_t rx_setting_target;

    while (app_vars.state != SWEEP_TX_DONE)
        ;
	app_vars.state = SYNC_TIMER;
	// sync the timer
	sync_timer();
	
    app_vars.state = CONTINUOUSLY_CAL;

    printf("CONTINUOUSLY_CAL started\r\n");

    // rftimer_setCompareIn(rftimer_readCounter() + SENDING_INTERVAL);

	
	
    while (1) {
		if (app_vars.periodical_timer_expired || app_vars.rx_done) {
			// listen for packet
			radio_rfOff();
			app_vars.rx_done = 0;

			LC_FREQCHANGE((app_vars.rx_setting_candidate[DEFAULT_SETTING] >> 10) & 0x001f,
				   (app_vars.rx_setting_candidate[DEFAULT_SETTING] >> 5) & 0x001f,
				   (app_vars.rx_setting_candidate[DEFAULT_SETTING]) & 0x001f);

			radio_rxEnable();
			radio_rxNow();
			rftimer_setCompareIn(rftimer_readCounter() + 5 * MS_IN_TICKS);
			// sleep until rx_done becomes 1
			while (app_vars.rx_done == 0)
				;
			if (app_vars.periodical_timer_expired == 0) {
				// printf("try: %d times\r\n", app_vars.retry_count);
				// timer is not expired, so it receive a packet
				app_vars.retry_count = 99;
				app_vars.rx_done = 0;
				// start to calibrate oscillators
				app_vars.inter_calibration_flag = 1;
				inter_calibrate_2M_setting();
				inter_calibrate_Tx_setting();
				
				rftimer_setCompareIn(app_vars.receving_count + (TICKS_OF_PERIODICAL_BEACON - 18 * MS_IN_TICKS));
				app_vars.inter_calibration_flag = 0;
				send_ack();
				printf("-----------------------\r\n");
			} else {
				// timer is expired
				/*
				set_2M_RC_frequency(31, 31, 20, 12, 20);

				scm3c_hw_interface_set_RC2M_coarse(20);
				scm3c_hw_interface_set_RC2M_fine(12);
				scm3c_hw_interface_set_RC2M_superfine(20);
					
				analog_scan_chain_write();
				analog_scan_chain_load();
				*/
			}
			app_vars.periodical_timer_expired = 0;
			/*
			// update RX settings according to IF count
			printf("ready to inter-calibrate\r\n");
			
			app_vars.inter_calibration_flag = 1;
			inter_calibrate_2M_setting();
			inter_calibrate_Tx_setting();
			app_vars.inter_calibration_flag = 0;
			*/
		}
    }
}

void sync_timer(void)
{
	radio_rfOff();
	app_vars.rx_done = 0;
	LC_FREQCHANGE((app_vars.rx_setting_candidate[DEFAULT_SETTING] >> 10) & 0x001f,
               (app_vars.rx_setting_candidate[DEFAULT_SETTING] >> 5) & 0x001f,
               (app_vars.rx_setting_candidate[DEFAULT_SETTING]) & 0x001f);

	radio_rxEnable();
    radio_rxNow();
	
	// to check weather RX timeout occu
	rftimer_setCompareIn(rftimer_readCounter() + MS_IN_TICKS);
	while (app_vars.rx_done == 0)
        ;
	rftimer_setCompareIn(app_vars.receving_count + (TICKS_OF_PERIODICAL_BEACON - 18 * MS_IN_TICKS));
	// set a timer to open the radio periodically, but we need to open radio in advance
	app_vars.periodical_timer_expired = 0;
	app_vars.retry_count = 99;
	app_vars.rx_done = 0;
}

void inter_calibrate_2M_setting(void)
{
	uint32_t count_2M;
    uint32_t count_LC;
    uint32_t count_adc;
	uint32_t count_LC_RX_measured;
	uint32_t count_2M_RC_measured;
	int32_t adjustment_2M_RC_mid_simplified;
	int32_t frequency_difference_RC_2M;
	int32_t tmp;
	uint32_t RC2M_coarse;
    uint32_t RC2M_fine;
    uint32_t RC2M_superfine;
	volatile int32_t A;
	volatile int32_t B;
	volatile int32_t C;
	
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
	
	// freq_distance = (count_2M_RC_measured - 100000) * 20;
	// freq_distance /= 1930;
	
	//freq_distance = (count_LC_RX_measured - 125000) * 20 / 8;
		
	//freq = count_LC_RX_measured * 960 / MEASUREMENT_INTERVAL * 500 * 1000 / 1000000;
	// freq = count_LC_RX_measured * 4800 / MEASUREMENT_INTERVAL;
		
	// beware of overflow
	// adjustment_2M_RC_mid_simplified = (A - B) / C
	printf("freq: %d\r\n", freq);
	freq = 24024;
	A = count_2M_RC_measured / 10 * freq; // MHz * Hz
	B = 2 * count_LC_RX_measured * 960; // MHz * Hz
		
	// C = count_LC_RX_measured * 1930 * 960 / 1000 / 1000 ===> C = count_LC_RX_measured * 193 * 96 / 10000
    // ===> C = count_LC_RX_measured * 193 * 96 / 10000  ===> 2316 / 1250 ===> 1158 / 625
	// C = count_LC_RX_measured * 1158 / 125 ; // MHz * Hz
	
	// * 1486 * 96 / 100 / 1000 ===> 4458 / 3125 ===> 743 * 6 / 125 / 25
	// 15 * 96 / 1000 ===> 36 / 25
	// C = count_LC_RX_measured * 36 / 5 ; // MHz * Hz
	//printf("A: %d, B: %d, C: %d\r\n", A, B, C);
	// adjustment_2M_RC_mid_simplified = (A - B) * 5 / C;
	// printf("adjust_2M: %d, freq_distance: %d\r\n", adjustment_2M_RC_mid_simplified, freq_distance);
	// RC2M_fine += (adjustment_2M_RC_mid_simplified);
	
	C = count_LC_RX_measured * 960 / 1000;
	
	frequency_difference_RC_2M = A - B;
	// freq = 24015, freq / 10 = 2401.5   
	// * 200 / C ===> * 20 / (freq / 10)
	frequency_difference_RC_2M = frequency_difference_RC_2M * 1000 / C;
	
	update_rc_setting(frequency_difference_RC_2M, RC2M_coarse, RC2M_fine, RC2M_superfine);
	
	printf("2M setting: %d.%d.%d\r\n", RC2M_coarse, RC2M_fine, RC2M_superfine);
}

void inter_calibrate_Tx_setting(void)
{
	uint32_t count_2M;
    uint32_t count_LC;
    uint32_t count_adc;
	uint32_t count_LC_TX_measured;
	uint32_t count_2M_RC_measured;
	int32_t adjustment_LC_TX_fine_simplified;
	int32_t frequency_difference_LC_TX;
	volatile int32_t A;
	volatile int32_t B;
	volatile int32_t C;
	LC_FREQCHANGE((app_vars.tx_setting_candidate[DEFAULT_SETTING] >> 10) & 0x001f,
               (app_vars.tx_setting_candidate[DEFAULT_SETTING] >> 5) & 0x001f,
               (app_vars.tx_setting_candidate[DEFAULT_SETTING]) & 0x001f);
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
	// simplified
	count_LC_TX_measured = count_LC;
	count_2M_RC_measured = count_2M;
	
	A = count_LC_TX_measured * 2 * 960; // MHz * Hz
	B = (count_2M_RC_measured / 10) * (24050); // MHz * Hz
	// printf("B: %d\r\n", B); // don't delete this line

	
	// 130.9KHz / 1000 ===> * 1309 / 10000
	C = count_2M_RC_measured * (Hz_TX_LC_per_fine_code / 100) / 10000; // MHz * Hz0
	adjustment_LC_TX_fine_simplified = (A - B) / C;
	//printf("adjust_LC: %d\r\n", adjustment_LC_TX_fine_simplified);
	app_vars.tx_setting_candidate[DEFAULT_SETTING] -= adjustment_LC_TX_fine_simplified;
	lc_setting_edge_detection(app_vars.tx_setting_candidate, 1);
	
		
	/*
	C = 2;
	// printf("A: %d, B: %d, C: %d\r\n", A, B, C);
	frequency_difference_LC_TX = A - B;	
	frequency_difference_LC_TX = frequency_difference_LC_TX * 20 / C;
	// adjustment_LC_TX_fine_simplified = frequency_difference_LC_TX / Hz_TX_LC_per_fine_code;
	// app_vars.tx_setting_candidate[DEFAULT_SETTING] = app_vars.tx_setting_candidate[DEFAULT_SETTING] - adjustment_LC_TX_fine_simplified;
	// lc_setting_edge_detection(app_vars.tx_setting_candidate, 1);
	update_tx_setting(frequency_difference_LC_TX);
	*/
	
	printf("TX setting:%d.%d.%d\r\n", (app_vars.tx_setting_candidate[DEFAULT_SETTING] >> 10) & 0x001f,
										(app_vars.tx_setting_candidate[DEFAULT_SETTING] >> 5) & 0x001f,
										(app_vars.tx_setting_candidate[DEFAULT_SETTING]) & 0x001f);
	
}

void send_ack(void)
{
    uint8_t pkt[TARGET_PKT_SIZE];
	radio_rfOff();

	app_vars.tx_done = 0;
	printf("packet_idx: %d\r\n", packet_idx);
    pkt[0] = 'P';
	
	pkt[1] = (packet_idx & 0xff000000) >> 24;
	pkt[2] = (packet_idx & 0x00ff0000) >> 16;
	pkt[3] = (packet_idx & 0x0000ff00) >> 8;
	pkt[4] = (packet_idx & 0x000000ff);
	
	packet_idx++;
    radio_loadPacket(pkt, TARGET_PKT_SIZE);
	LC_FREQCHANGE((app_vars.tx_setting_candidate[DEFAULT_SETTING] >> 10) & 0x001f,
               (app_vars.tx_setting_candidate[DEFAULT_SETTING] >> 5) & 0x001f,
               (app_vars.tx_setting_candidate[DEFAULT_SETTING]) & 0x001f);
    radio_txEnable();

    delay_tx();

    radio_txNow();
    while (app_vars.tx_done == 0){};
}

void update_rc_setting(int32_t frequency_difference_RC_2M, uint32_t RC2M_coarse, uint32_t RC2M_fine, uint32_t RC2M_superfine)
{
	// frequency difference in coarse code is about 10.860621093749996  KHz
	// frequency difference in fine code is about 1.597589717741936  KHz
	int32_t adjustment_coarse_code = 0;
	int32_t adjustment_fine_code;
	int32_t adjusted_fine_code;
	int32_t intermediate_result = RC2M_coarse;
	// printf("frequency_difference_RC_2M: %d\r\n", frequency_difference_RC_2M);
	
	adjustment_fine_code = frequency_difference_RC_2M / Hz_2M_RC_per_fine_code;
	adjusted_fine_code = RC2M_fine + adjustment_fine_code;
	if (adjusted_fine_code < 0 || adjusted_fine_code >= 32) {
		// it beyond the capacity of fine code, so we need to adjust coarse code too.
		adjustment_coarse_code = frequency_difference_RC_2M / Hz_2M_RC_per_coarse_code;
		frequency_difference_RC_2M = frequency_difference_RC_2M % Hz_2M_RC_per_coarse_code;
		
		adjustment_fine_code = frequency_difference_RC_2M / Hz_2M_RC_per_fine_code;
		adjusted_fine_code = RC2M_fine;
		adjusted_fine_code += adjustment_fine_code;
	}
	
	if (adjusted_fine_code < 0) {
		adjustment_coarse_code -= 1;
		adjusted_fine_code += (Hz_2M_RC_per_coarse_code/Hz_2M_RC_per_fine_code); // 6 or 7 fine codes is equal to 1 corse code
	} else if (adjusted_fine_code >= 32) {
		adjustment_coarse_code += 1;
		adjusted_fine_code -= (Hz_2M_RC_per_coarse_code/Hz_2M_RC_per_fine_code);
	}
	intermediate_result += adjustment_coarse_code;
	if ((intermediate_result >= 0) && (intermediate_result) < 32)
		RC2M_coarse = intermediate_result;
	RC2M_fine = adjusted_fine_code;
	
	set_2M_RC_frequency(31, 31, RC2M_coarse, RC2M_fine, RC2M_superfine);

    scm3c_hw_interface_set_RC2M_coarse(RC2M_coarse);
    scm3c_hw_interface_set_RC2M_fine(RC2M_fine);
    scm3c_hw_interface_set_RC2M_superfine(RC2M_superfine);
		
    analog_scan_chain_write();
    analog_scan_chain_load();
}

void update_tx_setting(int32_t frequency_difference_TX_LC)
{
	// frequency difference in middle code is about 794.8616541470408  KHz
	// frequency difference in find code is about 130.90244749698303  KHz
	int32_t adjustment_middle_code = 0;
	int32_t adjustment_fine_code = 0;
	int32_t adjusted_fine_code = (app_vars.tx_setting_candidate[DEFAULT_SETTING]) & 0x001f;
	
	// printf("frequency_difference_TX_LC: %d\r\n", frequency_difference_TX_LC);
	adjustment_fine_code = frequency_difference_TX_LC / Hz_TX_LC_per_fine_code;
	adjusted_fine_code -= adjustment_fine_code;
	if (adjusted_fine_code < 0 || adjusted_fine_code >= 32) {
		// it beyond the capacity of fine code, so we need to adjust middle code too.
		adjustment_middle_code = frequency_difference_TX_LC / Hz_TX_LC_per_middle_code;
		frequency_difference_TX_LC = frequency_difference_TX_LC % Hz_TX_LC_per_middle_code;
		
		adjustment_fine_code = frequency_difference_TX_LC / Hz_TX_LC_per_fine_code;
		adjusted_fine_code = (app_vars.tx_setting_candidate[DEFAULT_SETTING]) & 0x001f - adjustment_fine_code;
	}
	
	if (adjusted_fine_code < 0) {
		adjustment_middle_code += 1;
		adjusted_fine_code += (Hz_TX_LC_per_middle_code / Hz_TX_LC_per_fine_code); // 6 or 7 fine codes is equal to 1 corse code
	} else if (adjusted_fine_code >= 32) {
		adjustment_middle_code -= 1;
		adjusted_fine_code -= (Hz_TX_LC_per_middle_code / Hz_TX_LC_per_fine_code);
	}
	
	app_vars.tx_setting_candidate[DEFAULT_SETTING] -= (adjustment_middle_code << 5);
	app_vars.tx_setting_candidate[DEFAULT_SETTING] = (app_vars.tx_setting_candidate[DEFAULT_SETTING] & (~0x001f)) + (adjusted_fine_code & 0x001f);
	
}

void update_target_settings(void) {
    uint8_t i;
    uint32_t avg_if;
    int32_t adjustment;
    int32_t tmp;
    int16_t avg_fo;
    uint32_t avg_count_2M;

    uint32_t RC2M_coarse;
    uint32_t RC2M_fine;
    uint32_t RC2M_superfine;

    // update target setting for RX
    avg_if = 0;
    for (i = 0; i < HISTORY_SAMPLE_SIZE; i++) {
        avg_if += app_vars.if_history[i];
    }
    avg_if /= HISTORY_SAMPLE_SIZE;

    adjustment = ((int32_t)(avg_if - 500)) / 17;
    app_vars.rx_setting_candidate[app_vars.setting_index] += adjustment;

    // update target setting for TX
    avg_fo = 0;
    for (i = 0; i < HISTORY_SAMPLE_SIZE; i++) {
        avg_fo += app_vars.fo_history[i];
    }
    avg_fo /= HISTORY_SAMPLE_SIZE;

    adjustment = (int16_t)(avg_fo + 8) / 9;
    app_vars.tx_setting_candidate[app_vars.setting_index] -= adjustment;

    // update target setting for 2M RC OSC
    avg_count_2M = 0;
    for (i = 0; i < HISTORY_SAMPLE_SIZE; i++) {
        avg_count_2M += app_vars.count_2m_history[i];
    }
    avg_count_2M /= HISTORY_SAMPLE_SIZE;

    RC2M_coarse = scm3c_hw_interface_get_RC2M_coarse();
    RC2M_fine = scm3c_hw_interface_get_RC2M_fine();
    RC2M_superfine = scm3c_hw_interface_get_RC2M_superfine();

    adjustment = (int32_t)(avg_count_2M - app_vars.target_count_2m);
    if (adjustment >= 331 || adjustment <= -331) {
        tmp = adjustment / 331;
        RC2M_coarse += tmp;
        adjustment = adjustment - tmp * 331;
    }

    if (adjustment >= 58 || adjustment <= -58) {
        tmp = adjustment / 58;
        RC2M_fine += tmp;
        adjustment = adjustment - tmp * 58;
    }

    //    if (adjustment >= 7 || adjustment <= -7) {
    //        tmp               = adjustment/7;
    //        RC2M_superfine   += tmp;
    //    }
		
    set_2M_RC_frequency(31, 31, RC2M_coarse, RC2M_fine, RC2M_superfine);

    scm3c_hw_interface_set_RC2M_coarse(RC2M_coarse);
    scm3c_hw_interface_set_RC2M_fine(RC2M_fine);
    scm3c_hw_interface_set_RC2M_superfine(RC2M_superfine);
		
    analog_scan_chain_write();
    analog_scan_chain_load();

    // output for plot

    printf(
        "TX setting %d %d %d (avg_fo=%d) | RX setting %d %d %d (avg_if=%d) | "
        "2M setting %d %d %d (avg_count_2M=%d) | temp=%d\r\n",
        (app_vars.tx_setting_candidate[app_vars.setting_index] >> 10) & 0x001f,
        (app_vars.tx_setting_candidate[app_vars.setting_index] >> 5) & 0x001f,
        (app_vars.tx_setting_candidate[app_vars.setting_index]) & 0x001f,
        avg_fo,
        (app_vars.rx_setting_candidate[app_vars.setting_index] >> 10) & 0x001f,
        (app_vars.rx_setting_candidate[app_vars.setting_index] >> 5) & 0x001f,
        (app_vars.rx_setting_candidate[app_vars.setting_index]) & 0x001f,
        avg_if, RC2M_coarse, RC2M_fine, RC2M_superfine, avg_count_2M,
        app_vars.last_temperature);
}

//=== switch lc setting between main and candidate settings

void switch_lc_setting(void) {
    if (app_vars.setting_index == 0) {
        app_vars.setting_index = 1;
    } else {
        app_vars.setting_index = 0;
    }
}

//=== edge detection

void lc_setting_edge_detection(uint16_t* setting, uint8_t txOrRx) {
    uint8_t i;
    int8_t move_mid_nextORprev;
    int8_t offset;

    // 0: do nothing 1: mid next -1: move previous
    move_mid_nextORprev = 0;
    for (i = 0; i < NUM_CANDIDATE_SETTING; i++) {
        if ((setting[i] & 0x001f) >= HIGH_BOUDRY_FINE) {
            move_mid_nextORprev = 1;
            break;
        } else {
            if ((setting[i] & 0x001f) <= LOW_BOUDRY_FINE) {
                move_mid_nextORprev = -1;
                break;
            }
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
