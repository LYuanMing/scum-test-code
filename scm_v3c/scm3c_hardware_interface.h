unsigned reverse(unsigned x);
unsigned int crc32c(unsigned char *message, unsigned int length);
unsigned char flipChar(unsigned char b);
void init_ldo_control(void);
unsigned int sram_test(unsigned int * baseAddress, unsigned int num_dwords);
void radio_init_rx_MF(void);
void radio_init_rx_ZCC(void);
void radio_init_tx(void);
void radio_init_divider(unsigned int div_value);
void radio_disable_all(void);
void GPO_control(char row1, char row2, char row3, char row4);
void GPI_control(char row1, char row2, char row3, char row4);
unsigned int read_IF_estimate(void);
unsigned int read_LQI(void);
unsigned int read_RSSI(void);
void set_IF_clock_frequency(int coarse, int fine, int high_range);
void GPO_enables(unsigned int mask);
void GPI_enables(unsigned int mask);
void set_IF_LDO_voltage(int code);
void set_VDDD_LDO_voltage(int code);
void set_AUX_LDO_voltage(int code);
void set_ALWAYSON_LDO_voltage(int code);
void radio_enable_PA(void);
void radio_enable_LO(void);
void radio_enable_RX(void);
void read_counters_3B(unsigned int* count_2M, unsigned int* count_LC, unsigned int* count_adc);
void do_fake_cal(void);
void packet_test_loop(unsigned int num_packets);
void set_IF_stg3gm_ASC(unsigned int Igm, unsigned int Qgm);
void set_IF_gain_ASC(unsigned int Igain, unsigned int Qgain);
void set_zcc_demod_threshold(unsigned int thresh);
void set_IF_comparator_trim_I(unsigned int ptrim, unsigned int ntrim);
void set_IF_comparator_trim_Q(unsigned int ptrim, unsigned int ntrim);
void set_IF_ZCC_clkdiv(unsigned int div_value);
void set_IF_ZCC_early(unsigned int early_value);
void initialize_mote(void);
void set_sys_clk_secondary_freq(unsigned int coarse, unsigned int fine);
unsigned int estimate_temperature_2M_32k(void);
