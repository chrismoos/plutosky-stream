/*
 * Thin C layer between the Rust daemon and the AD9361 no-OS driver. It owns
 * the AD9361_InitParam so Rust never has to mirror that struct. The SPI,
 * GPIO and delay hooks from platform.h are implemented in Rust.
 */
#include <stdint.h>
#include <string.h>

#include "ad9361.h"
#include "ad9361_api.h"
#include "platform.h"

/* GPIO numbers understood by the Rust gpio_set_value hook. */
#define PSKS_GPIO_RESETB 0
#define PSKS_GPIO_NONE   (-1)

#define PSKS_INTERFACE_DELAY_MAX 15

/* interface_flags bits, in the order the daemon's --ad9361-flags help lists. */
#define PSKS_IFACE_INVERT_DATA_CLK  (1u << 0)
#define PSKS_IFACE_FRAME_LEVEL_MODE (1u << 1)
#define PSKS_IFACE_INVERT_RX_FRAME  (1u << 2)
#define PSKS_IFACE_SWAP_PORTS       (1u << 3)
#define PSKS_IFACE_NO_PORT_SWAP     (1u << 4)
#define PSKS_IFACE_INVERT_DATA_BUS  (1u << 5)

/* BBPLL, ADC or DAC, the halfband stages and the sample clock, in Hz, for
 * ADI's 30.72 MS/s reference profile. */

static const uint32_t default_rx_path_clks[6] = {
	983040000, 245760000, 122880000, 61440000, 30720000, 30720000
};
static const uint32_t default_tx_path_clks[6] = {
	983040000, 122880000, 122880000, 61440000, 30720000, 30720000
};

/* ADI's reference defaults for a 2R2T LVDS FDD part, narrowed to the fields
 * the daemon decides at runtime. */
static void psks_default_init_param(AD9361_InitParam *p,
					  uint32_t ref_clk_hz,
					  uint64_t tx_lo_hz,
					  uint32_t tx_bw_hz,
					  int32_t tx_atten_mdb,
					  uint64_t rx_lo_hz,
					  uint32_t rx_bw_hz,
					  uint8_t gc_mode)
{
	memset(p, 0, sizeof(*p));
	p->dev_sel = ID_AD9363A;
	p->id_no = 0;
	p->reference_clk_rate = ref_clk_hz;
	p->two_rx_two_tx_mode_enable = 1;
	p->one_rx_one_tx_mode_use_rx_num = 1;
	p->one_rx_one_tx_mode_use_tx_num = 1;
	p->frequency_division_duplex_mode_enable = 1;
	p->frequency_division_duplex_independent_mode_enable = 1;
	p->tdd_use_dual_synth_mode_enable = 0;
	p->tdd_skip_vco_cal_enable = 0;
	p->tx_fastlock_delay_ns = 0;
	p->rx_fastlock_delay_ns = 0;
	p->rx_fastlock_pincontrol_enable = 0;
	p->tx_fastlock_pincontrol_enable = 0;
	p->external_rx_lo_enable = 0;
	p->external_tx_lo_enable = 0;
	p->dc_offset_tracking_update_event_mask = 5;
	p->dc_offset_attenuation_high_range = 6;
	p->dc_offset_attenuation_low_range = 5;
	p->dc_offset_count_high_range = 0x28;
	p->dc_offset_count_low_range = 0x32;
	p->split_gain_table_mode_enable = 0;
	p->trx_synthesizer_target_fref_overwrite_hz = MAX_SYNTH_FREF;
	p->qec_tracking_slow_mode_enable = 0;
	p->ensm_enable_pin_pulse_mode_enable = 0;
	p->ensm_enable_txnrx_control_enable = 0;
	p->rx_synthesizer_frequency_hz = rx_lo_hz;
	p->tx_synthesizer_frequency_hz = tx_lo_hz;
	p->tx_lo_powerdown_managed_enable = 1;
	memcpy(p->rx_path_clock_frequencies, default_rx_path_clks,
	       sizeof(default_rx_path_clks));
	memcpy(p->tx_path_clock_frequencies, default_tx_path_clks,
	       sizeof(default_tx_path_clks));
	p->rf_rx_bandwidth_hz = rx_bw_hz;
	p->rf_tx_bandwidth_hz = tx_bw_hz;
	p->rx_rf_port_input_select = 0;
	p->tx_rf_port_input_select = 0;
	p->tx_attenuation_mdB = tx_atten_mdb;
	p->update_tx_gain_in_alert_enable = 0;
	p->xo_disable_use_ext_refclk_enable = 0;
	p->dcxo_coarse_and_fine_tune[0] = 8;
	p->dcxo_coarse_and_fine_tune[1] = 5920;
	p->clk_output_mode_select = 0;
	p->gc_rx1_mode = gc_mode;
	p->gc_rx2_mode = gc_mode;
	p->gc_adc_large_overload_thresh = 58;
	p->gc_adc_ovr_sample_size = 4;
	p->gc_adc_small_overload_thresh = 47;
	p->gc_dec_pow_measurement_duration = 8192;
	p->gc_dig_gain_enable = 0;
	p->gc_lmt_overload_high_thresh = 800;
	p->gc_lmt_overload_low_thresh = 704;
	p->gc_low_power_thresh = 24;
	p->gc_max_dig_gain = 15;
	p->mgc_dec_gain_step = 2;
	p->mgc_inc_gain_step = 2;
	p->mgc_rx1_ctrl_inp_enable = 0;
	p->mgc_rx2_ctrl_inp_enable = 0;
	p->mgc_split_table_ctrl_inp_gain_mode = 0;
	p->agc_adc_large_overload_exceed_counter = 10;
	p->agc_adc_large_overload_inc_steps = 2;
	p->agc_adc_lmt_small_overload_prevent_gain_inc_enable = 0;
	p->agc_adc_small_overload_exceed_counter = 10;
	p->agc_dig_gain_step_size = 4;
	p->agc_dig_saturation_exceed_counter = 3;
	p->agc_gain_update_interval_us = 1000;
	p->agc_immed_gain_change_if_large_adc_overload_enable = 0;
	p->agc_immed_gain_change_if_large_lmt_overload_enable = 0;
	p->agc_inner_thresh_high = 10;
	p->agc_inner_thresh_high_dec_steps = 1;
	p->agc_inner_thresh_low = 12;
	p->agc_inner_thresh_low_inc_steps = 1;
	p->agc_lmt_overload_large_exceed_counter = 10;
	p->agc_lmt_overload_large_inc_steps = 2;
	p->agc_lmt_overload_small_exceed_counter = 10;
	p->agc_outer_thresh_high = 5;
	p->agc_outer_thresh_high_dec_steps = 2;
	p->agc_outer_thresh_low = 18;
	p->agc_outer_thresh_low_inc_steps = 2;
	p->agc_attack_delay_extra_margin_us = 1;
	p->agc_sync_for_gain_counter_enable = 0;
	p->fagc_dec_pow_measuremnt_duration = 64;
	p->fagc_state_wait_time_ns = 260;
	p->fagc_allow_agc_gain_increase = 0;
	p->fagc_lp_thresh_increment_time = 5;
	p->fagc_lp_thresh_increment_steps = 1;
	p->fagc_lock_level_lmt_gain_increase_en = 1;
	p->fagc_lock_level_gain_increase_upper_limit = 5;
	p->fagc_lpf_final_settling_steps = 1;
	p->fagc_lmt_final_settling_steps = 1;
	p->fagc_final_overrange_count = 3;
	p->fagc_gain_increase_after_gain_lock_en = 0;
	p->fagc_gain_index_type_after_exit_rx_mode = 0;
	p->fagc_use_last_lock_level_for_set_gain_en = 0;
	p->fagc_rst_gla_stronger_sig_thresh_exceeded_en = 1;
	p->fagc_optimized_gain_offset = 5;
	p->fagc_rst_gla_stronger_sig_thresh_above_ll = 10;
	p->fagc_rst_gla_engergy_lost_sig_thresh_exceeded_en = 1;
	p->fagc_rst_gla_engergy_lost_goto_optim_gain_en = 1;
	p->fagc_rst_gla_engergy_lost_sig_thresh_below_ll = 10;
	p->fagc_energy_lost_stronger_sig_gain_lock_exit_cnt = 8;
	p->fagc_rst_gla_large_adc_overload_en = 1;
	p->fagc_rst_gla_large_lmt_overload_en = 1;
	p->fagc_rst_gla_en_agc_pulled_high_en = 0;
	p->fagc_rst_gla_if_en_agc_pulled_high_mode = 0;
	p->fagc_power_measurement_duration_in_state5 = 64;
	p->rssi_delay = 1;
	p->rssi_duration = 1000;
	p->rssi_restart_mode = 3;
	p->rssi_unit_is_rx_samples_enable = 0;
	p->rssi_wait = 1;
	p->aux_adc_decimation = 256;
	p->aux_adc_rate = 40000000;
	p->aux_dac_manual_mode_enable = 1;
	p->temp_sense_decimation = 256;
	p->temp_sense_measurement_interval_ms = 1000;
	p->temp_sense_offset_signed = 0xCE;
	p->temp_sense_periodic_measurement_enable = 1;
	p->ctrl_outs_enable_mask = 0xFF;
	p->ctrl_outs_index = 0;
	p->digital_interface_tune_skip_mode = 0;
	p->digital_interface_tune_fir_disable = 0;
	p->pp_tx_swap_enable = 1;
	p->pp_rx_swap_enable = 1;
	p->tx_channel_swap_enable = 0;
	p->rx_channel_swap_enable = 0;
	p->rx_frame_pulse_mode_enable = 1;
	p->two_t_two_r_timing_enable = 1;
	p->invert_data_bus_enable = 0;
	p->invert_data_clk_enable = 0;
	p->fdd_alt_word_order_enable = 0;
	p->invert_rx_frame_enable = 0;
	p->fdd_rx_rate_2tx_enable = 0;
	p->swap_ports_enable = 0;
	p->single_data_rate_enable = 0;
	p->lvds_mode_enable = 1;
	p->half_duplex_mode_enable = 0;
	p->single_port_mode_enable = 0;
	p->full_port_enable = 0;
	p->full_duplex_swap_bits_enable = 0;
	p->delay_rx_data = 0;
	p->rx_data_clock_delay = 0;
	p->rx_data_delay = 4;
	p->tx_fb_clock_delay = 7;
	p->tx_data_delay = 0;
	p->lvds_bias_mV = 150;
	p->lvds_rx_onchip_termination_enable = 1;
	p->rx1rx2_phase_inversion_en = 0;
	p->lvds_invert1_control = 0xFF;
	p->lvds_invert2_control = 0x0F;
	p->low_high_gain_threshold_mdB = 37000;
	p->low_gain_dB = 0;
	p->high_gain_dB = 24;
	p->tx_mon_track_en = 0;
	p->one_shot_mode_en = 0;
	p->tx_mon_delay = 511;
	p->tx_mon_duration = 8192;
	p->tx1_mon_front_end_gain = 2;
	p->tx2_mon_front_end_gain = 2;
	p->tx1_mon_lo_cm = 48;
	p->tx2_mon_lo_cm = 48;
	p->gpio_resetb = PSKS_GPIO_RESETB;
	p->gpio_sync = PSKS_GPIO_NONE;
	p->gpio_cal_sw1 = PSKS_GPIO_NONE;
	p->gpio_cal_sw2 = PSKS_GPIO_NONE;
	p->ad9361_rfpll_ext_recalc_rate = NULL;
	p->ad9361_rfpll_ext_round_rate = NULL;
	p->ad9361_rfpll_ext_set_rate = NULL;
}

int32_t psks_ad9361_init(void **phy_out, uint32_t ref_clk_hz,
			 uint64_t tx_lo_hz, uint64_t rx_lo_hz,
			 uint32_t sample_rate_hz, uint32_t bw_hz,
			 int32_t tx_atten_mdb, int32_t rx_gain_db,
			 uint8_t gc_mode, uint8_t two_rx_two_tx,
			 uint32_t interface_flags)
{
	AD9361_InitParam param;
	struct ad9361_rf_phy *phy = NULL;
	int32_t ret;
	uint8_t ch;

	psks_default_init_param(&param, ref_clk_hz, tx_lo_hz, bw_hz,
				tx_atten_mdb, rx_lo_hz, bw_hz, gc_mode);
	param.two_rx_two_tx_mode_enable = two_rx_two_tx;
	/* The port's 2T2R timing must follow the channel count: with it set in
	 * 1R1T the chip still clocks four cycles per sample and repeats each
	 * sample, and the fabric PHY's single-channel assembly reads that as two
	 * identical samples. */
	param.two_t_two_r_timing_enable = two_rx_two_tx;
	/* Interface experiments, bit-mapped so the daemon can sweep them. */
	if (interface_flags & PSKS_IFACE_INVERT_DATA_CLK)
		param.invert_data_clk_enable = 1;
	if (interface_flags & PSKS_IFACE_FRAME_LEVEL_MODE)
		param.rx_frame_pulse_mode_enable = 0;
	if (interface_flags & PSKS_IFACE_INVERT_RX_FRAME)
		param.invert_rx_frame_enable = 1;
	if (interface_flags & PSKS_IFACE_SWAP_PORTS)
		param.swap_ports_enable = 1;
	if (interface_flags & PSKS_IFACE_NO_PORT_SWAP)
		param.pp_rx_swap_enable = 0;
	if (interface_flags & PSKS_IFACE_INVERT_DATA_BUS)
		param.invert_data_bus_enable = 1;
	ret = ad9361_init(&phy, &param, 1);
	if (ret < 0)
		return ret;
	ret = ad9361_set_tx_sampling_freq(phy, sample_rate_hz);
	if (ret < 0)
		return ret;
	ret = ad9361_set_rx_sampling_freq(phy, sample_rate_hz);
	if (ret < 0)
		return ret;
	ret = ad9361_set_tx_rf_bandwidth(phy, bw_hz);
	if (ret < 0)
		return ret;
	ret = ad9361_set_rx_rf_bandwidth(phy, bw_hz);
	if (ret < 0)
		return ret;
	for (ch = 0; ch < 2; ch++) {
		ret = ad9361_set_tx_attenuation(phy, ch, tx_atten_mdb);
		if (ret < 0)
			return ret;
		if (gc_mode == RF_GAIN_MGC) {
			ret = ad9361_set_rx_rf_gain(phy, ch, rx_gain_db);
			if (ret < 0)
				return ret;
		}
	}
	*phy_out = phy;
	return 0;
}

/* The driver retries last_tx_quad_cal_phase before sweeping all 32 NCO phase
 * offsets, and the sweep keeps the calibration tone on the air for over a
 * second. The phase that converges depends on the digital filter
 * configuration, which follows the sample rate, so it is remembered per rate. */
#define PSKS_TX_QUAD_PHASES 32
#define PSKS_TX_QUAD_PHASE_SLOTS 8

struct psks_tx_quad_phase {
	uint32_t rate_hz;
	uint32_t phase;
};

static struct psks_tx_quad_phase psks_tx_quad_phase_cache[PSKS_TX_QUAD_PHASE_SLOTS];
static unsigned psks_tx_quad_phase_next;

static struct psks_tx_quad_phase *psks_tx_quad_phase_slot(uint32_t rate_hz)
{
	for (unsigned i = 0; i < PSKS_TX_QUAD_PHASE_SLOTS; i++)
		if (psks_tx_quad_phase_cache[i].rate_hz == rate_hz)
			return &psks_tx_quad_phase_cache[i];
	return NULL;
}

int32_t psks_ad9361_set_tx_lo(void *phy, uint64_t hz)
{
	struct ad9361_rf_phy *p = (struct ad9361_rf_phy *)phy;
	uint32_t rate_hz = 0;
	struct psks_tx_quad_phase *slot;
	int32_t ret;

	if (ad9361_get_tx_sampling_freq(p, &rate_hz) < 0)
		rate_hz = 0;
	slot = rate_hz ? psks_tx_quad_phase_slot(rate_hz) : NULL;
	if (slot)
		p->last_tx_quad_cal_phase = slot->phase;

	ret = ad9361_set_tx_lo_freq(p, hz);

	if (rate_hz && p->last_tx_quad_cal_phase < PSKS_TX_QUAD_PHASES) {
		if (!slot) {
			slot = &psks_tx_quad_phase_cache[psks_tx_quad_phase_next];
			psks_tx_quad_phase_next =
				(psks_tx_quad_phase_next + 1) % PSKS_TX_QUAD_PHASE_SLOTS;
			slot->rate_hz = rate_hz;
		}
		slot->phase = p->last_tx_quad_cal_phase;
	}
	return ret;
}

int32_t psks_ad9361_get_tx_lo(void *phy, uint64_t *hz)
{
	return ad9361_get_tx_lo_freq((struct ad9361_rf_phy *)phy, hz);
}

int32_t psks_ad9361_set_rx_lo(void *phy, uint64_t hz)
{
	return ad9361_set_rx_lo_freq((struct ad9361_rf_phy *)phy, hz);
}

int32_t psks_ad9361_get_rx_lo(void *phy, uint64_t *hz)
{
	return ad9361_get_rx_lo_freq((struct ad9361_rf_phy *)phy, hz);
}

int32_t psks_ad9361_set_tx_rate(void *phy, uint32_t hz)
{
	return ad9361_set_tx_sampling_freq((struct ad9361_rf_phy *)phy, hz);
}

int32_t psks_ad9361_get_tx_rate(void *phy, uint32_t *hz)
{
	return ad9361_get_tx_sampling_freq((struct ad9361_rf_phy *)phy, hz);
}

int32_t psks_ad9361_set_rx_rate(void *phy, uint32_t hz)
{
	return ad9361_set_rx_sampling_freq((struct ad9361_rf_phy *)phy, hz);
}

int32_t psks_ad9361_get_rx_rate(void *phy, uint32_t *hz)
{
	return ad9361_get_rx_sampling_freq((struct ad9361_rf_phy *)phy, hz);
}

int32_t psks_ad9361_set_tx_bw(void *phy, uint32_t hz)
{
	return ad9361_set_tx_rf_bandwidth((struct ad9361_rf_phy *)phy, hz);
}

int32_t psks_ad9361_get_tx_bw(void *phy, uint32_t *hz)
{
	return ad9361_get_tx_rf_bandwidth((struct ad9361_rf_phy *)phy, hz);
}

int32_t psks_ad9361_set_rx_bw(void *phy, uint32_t hz)
{
	return ad9361_set_rx_rf_bandwidth((struct ad9361_rf_phy *)phy, hz);
}

int32_t psks_ad9361_get_rx_bw(void *phy, uint32_t *hz)
{
	return ad9361_get_rx_rf_bandwidth((struct ad9361_rf_phy *)phy, hz);
}

int32_t psks_ad9361_set_tx_atten(void *phy, uint8_t ch, uint32_t mdb)
{
	return ad9361_set_tx_attenuation((struct ad9361_rf_phy *)phy, ch, mdb);
}

int32_t psks_ad9361_get_tx_atten(void *phy, uint8_t ch, uint32_t *mdb)
{
	return ad9361_get_tx_attenuation((struct ad9361_rf_phy *)phy, ch, mdb);
}

/* Manual RX gain in whole dB. Ignored by the part unless the channel's
 * gain-control mode is MGC. */
int32_t psks_ad9361_set_rx_gain(void *phy, uint8_t ch, int32_t gain_db)
{
	return ad9361_set_rx_rf_gain((struct ad9361_rf_phy *)phy, ch, gain_db);
}

int32_t psks_ad9361_get_rx_gain(void *phy, uint8_t ch, int32_t *gain_db)
{
	return ad9361_get_rx_rf_gain((struct ad9361_rf_phy *)phy, ch, gain_db);
}

/* mode is the no-OS rf_gain_ctrl_mode code (0 MGC, 1 fast AGC, 2 slow AGC). */
int32_t psks_ad9361_set_gain_control_mode(void *phy, uint8_t ch, uint8_t mode)
{
	return ad9361_set_rx_gain_control_mode((struct ad9361_rf_phy *)phy, ch,
					       mode);
}

int32_t psks_ad9361_get_gain_control_mode(void *phy, uint8_t ch, uint8_t *mode)
{
	return ad9361_get_rx_gain_control_mode((struct ad9361_rf_phy *)phy, ch,
					       mode);
}

int32_t psks_ad9361_set_rx_port(void *phy, uint32_t mode)
{
	return ad9361_set_rx_rf_port_input((struct ad9361_rf_phy *)phy, mode);
}

int32_t psks_ad9361_set_tx_port(void *phy, uint32_t mode)
{
	return ad9361_set_tx_rf_port_output((struct ad9361_rf_phy *)phy, mode);
}

int32_t psks_ad9361_get_temperature(void *phy, int32_t *millidegrees)
{
	return ad9361_get_temperature((struct ad9361_rf_phy *)phy,
				      millidegrees);
}

/* Runtime RSSI of one receive channel. The part reports it as a magnitude
 * below full scale scaled by `multiplier`, so the caller divides. */
int32_t psks_ad9361_get_rx_rssi(void *phy, uint8_t ch, int32_t *symbol,
				int32_t *multiplier)
{
	struct rf_rssi rssi;
	int32_t ret;

	ret = ad9361_get_rx_rssi((struct ad9361_rf_phy *)phy, ch, &rssi);
	if (ret < 0)
		return ret;
	*symbol = (int32_t)rssi.symbol;
	*multiplier = rssi.multiplier;
	return 0;
}

int32_t psks_ad9361_tx_analog_off(void *phy, uint16_t *saved)
{
	struct ad9361_rf_phy *p = phy;
	const uint8_t channels = 3;
	int32_t tx = ad9361_spi_read(p->spi, REG_TX_ANALOG_POWER_DOWN_OVERRIDE_1);
	int32_t analog = ad9361_spi_read(p->spi, REG_ANALOG_POWER_DOWN_OVERRIDE);
	int32_t ret;
	if (tx < 0)
		return tx;
	if (analog < 0)
		return analog;
	*saved = ((uint16_t)tx << 8) | analog;
	ret = ad9361_spi_write(p->spi, REG_ANALOG_POWER_DOWN_OVERRIDE,
		analog | TX_UPCONVERTER_POWER_DOWN(channels));
	if (ret < 0)
		return ret;
	return ad9361_spi_write(p->spi, REG_TX_ANALOG_POWER_DOWN_OVERRIDE_1,
		tx | TX_SECONDARY_FILTER_POWER_DOWN(channels) |
		TX_BBF_POWER_DOWN(channels) | TX_DAC_POWER_DOWN(channels) |
		TX_DAC_BIAS_POWER_DOWN(channels));
}

int32_t psks_ad9361_restore_tx_analog(void *phy, uint16_t saved)
{
	struct ad9361_rf_phy *p = phy;
	int32_t ret = ad9361_spi_write(p->spi, REG_TX_ANALOG_POWER_DOWN_OVERRIDE_1,
				      saved >> 8);
	if (ret < 0)
		return ret;
	return ad9361_spi_write(p->spi, REG_ANALOG_POWER_DOWN_OVERRIDE, saved & 0xff);
}

/* Releases what ad9361_init allocated. The driver has no teardown of its own. */
void psks_ad9361_free(void *phy_)
{
	struct ad9361_rf_phy *phy = phy_;
	int i;

	if (!phy)
		return;
	for (i = 0; i < NUM_AD9361_CLKS; i++) {
		free(phy->clks[i]);
		free(phy->ref_clk_scale[i]);
	}
	free(phy->clk_data.clks);
	free(phy->spi);
#ifndef AXI_ADC_NOT_PRESENT
	free(phy->adc_conv);
	free(phy->adc_state);
#endif
	free(phy->clk_refin);
	free(phy->pdata);
	free(phy);
}

int32_t psks_ad9361_set_fdd(void *phy)
{
	return ad9361_set_en_state_machine_mode((struct ad9361_rf_phy *)phy,
						ENSM_MODE_PINCTRL_FDD_INDEP);
}

uint8_t psks_ad9361_ensm_state(void *phy)
{
	return ad9361_ensm_get_state((struct ad9361_rf_phy *)phy);
}

int32_t psks_ad9361_tx_lo_locked(void *phy)
{
	int32_t v = ad9361_spi_read(((struct ad9361_rf_phy *)phy)->spi,
				    REG_TX_CP_OVERRANGE_VCO_LOCK);
	if (v < 0)
		return v;
	return (v & VCO_LOCK) ? 1 : 0;
}

int32_t psks_ad9361_rx_lo_locked(void *phy)
{
	int32_t v = ad9361_spi_read(((struct ad9361_rf_phy *)phy)->spi,
				    REG_RX_CP_OVERRANGE_VCO_LOCK);
	if (v < 0)
		return v;
	return (v & VCO_LOCK) ? 1 : 0;
}

/* Inject (or stop) an RX PRBS from the AD9361 so the fabric PRBS checker can
 * confirm the RX LVDS interface delay is aligned. */
int32_t psks_ad9361_bist_prbs_rx(void *phy, uint8_t on)
{
	return ad9361_bist_prbs((struct ad9361_rf_phy *)phy,
				on ? BIST_INJ_RX : BIST_DISABLE);
}

/* AD9361 internal TX->RX digital loopback (mode 1) or off (mode 0). Unlike the
 * fabric loopback, this drives the real RX data pins from the TX side, so it
 * tells apart dead RX LVDS pins from an unconfigured RX port. */
int32_t psks_ad9361_bist_loopback(void *phy, int32_t mode)
{
	return ad9361_bist_loopback((struct ad9361_rf_phy *)phy, mode);
}

int32_t psks_ad9361_get_interface_delay(void *phy, uint8_t tx)
{
	struct ad9361_rf_phy *p = (struct ad9361_rf_phy *)phy;
	return ad9361_spi_read(p->spi,
		tx ? REG_TX_CLOCK_DATA_DELAY : REG_RX_CLOCK_DATA_DELAY);
}

int32_t psks_ad9361_set_interface_delay(void *phy, uint8_t tx,
					uint8_t clock_delay, uint8_t data_delay)
{
	struct ad9361_rf_phy *p = (struct ad9361_rf_phy *)phy;
	uint32_t reg = tx ? REG_TX_CLOCK_DATA_DELAY : REG_RX_CLOCK_DATA_DELAY;
	uint8_t value = DATA_CLK_DELAY(clock_delay) | RX_DATA_DELAY(data_delay);
	int32_t previous = ad9361_spi_read(p->spi, reg);
	int32_t result;
	bool clock_changed;

	if (previous < 0)
		return previous;
	clock_changed = ((previous ^ value) & DATA_CLK_DELAY(PSKS_INTERFACE_DELAY_MAX)) != 0;
	if (clock_changed)
		ad9361_ensm_force_state(p, ENSM_STATE_ALERT);
	result = ad9361_spi_write(p->spi, reg, value);
	if (clock_changed)
		ad9361_ensm_restore_prev_state(p);
	if (result >= 0) {
		if (tx)
			p->pdata->port_ctrl.tx_clk_data_delay = value;
		else
			p->pdata->port_ctrl.rx_clk_data_delay = value;
	}
	return result;
}

#define PSKS_FIR_MAX_TAPS 128
#define PSKS_FIR_BOTH_CHANNELS 3
#define PSKS_RX_FIR_GAIN_DB_PER_OCTAVE (-6)
#define PSKS_TX_FIR_GAIN_DB 0

/*
 * Selects the programmable FIR for both directions. factor 1 bypasses it.
 * factor 2 or 4 loads coef into both receive channels as a decimator and both
 * transmit channels as an interpolator. coef sums to factor in Q15. The TX
 * interpolator divides by its ratio and the RX decimator does not, so the RX
 * gain stage removes the factor. The selection takes effect on the next
 * sample-rate call, which rebuilds the clock chain from the driver's bypass
 * flags and writes the FIR enable fields.
 */
int32_t psks_ad9361_select_fir(void *phy, uint32_t factor, const int16_t *coef,
			       uint8_t ntaps)
{
	struct ad9361_rf_phy *p = (struct ad9361_rf_phy *)phy;
	AD9361_RXFIRConfig rx;
	AD9361_TXFIRConfig tx;
	int32_t ret;

	if (factor == 1) {
		p->bypass_rx_fir = true;
		p->bypass_tx_fir = true;
		return 0;
	}
	if ((factor != 2 && factor != 4) || ntaps > PSKS_FIR_MAX_TAPS)
		return -EINVAL;

	memset(&rx, 0, sizeof(rx));
	rx.rx = PSKS_FIR_BOTH_CHANNELS;
	rx.rx_gain = PSKS_RX_FIR_GAIN_DB_PER_OCTAVE * (factor == 4 ? 2 : 1);
	rx.rx_dec = factor;
	rx.rx_coef_size = ntaps;
	memcpy(rx.rx_coef, coef, ntaps * sizeof(int16_t));
	ret = ad9361_set_rx_fir_config(p, rx);
	if (ret < 0)
		return ret;

	memset(&tx, 0, sizeof(tx));
	tx.tx = PSKS_FIR_BOTH_CHANNELS;
	tx.tx_gain = PSKS_TX_FIR_GAIN_DB;
	tx.tx_int = factor;
	tx.tx_coef_size = ntaps;
	memcpy(tx.tx_coef, coef, ntaps * sizeof(int16_t));
	ret = ad9361_set_tx_fir_config(p, tx);
	if (ret < 0)
		return ret;

	p->bypass_rx_fir = false;
	p->bypass_tx_fir = false;
	return 0;
}

/* The enable fields read 0 for bypass and 1, 2 or 3 for x1, x2 or x4. */
static int32_t psks_fir_field_factor(int32_t field)
{
	return field ? 1 << (field - 1) : 1;
}

int32_t psks_ad9361_fir_factors(void *phy, uint32_t *rx_factor,
				uint32_t *tx_factor)
{
	struct ad9361_rf_phy *p = (struct ad9361_rf_phy *)phy;
	int32_t rx = ad9361_spi_read(p->spi, REG_RX_ENABLE_FILTER_CTRL);
	int32_t tx = ad9361_spi_read(p->spi, REG_TX_ENABLE_FILTER_CTRL);

	if (rx < 0)
		return rx;
	if (tx < 0)
		return tx;
	*rx_factor = psks_fir_field_factor(rx & RX_FIR_ENABLE_DECIMATION(~0));
	*tx_factor = psks_fir_field_factor(tx & TX_FIR_ENABLE_INTERPOLATION(~0));
	return 0;
}
