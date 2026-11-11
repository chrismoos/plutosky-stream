//! Bindings to `csrc/ad9361_shim.c`.

use std::ffi::c_void;

unsafe extern "C" {
    pub fn psks_ad9361_init(
        phy_out: *mut *mut c_void,
        ref_clk_hz: u32,
        tx_lo_hz: u64,
        rx_lo_hz: u64,
        sample_rate_hz: u32,
        bw_hz: u32,
        tx_atten_mdb: i32,
        rx_gain_db: i32,
        gc_mode: u8,
        two_rx_two_tx: u8,
        interface_flags: u32,
    ) -> i32;
    pub fn psks_ad9361_set_tx_lo(phy: *mut c_void, hz: u64) -> i32;
    pub fn psks_ad9361_get_tx_lo(phy: *mut c_void, hz: *mut u64) -> i32;
    pub fn psks_ad9361_set_rx_lo(phy: *mut c_void, hz: u64) -> i32;
    pub fn psks_ad9361_get_rx_lo(phy: *mut c_void, hz: *mut u64) -> i32;
    pub fn psks_ad9361_set_tx_rate(phy: *mut c_void, hz: u32) -> i32;
    pub fn psks_ad9361_get_tx_rate(phy: *mut c_void, hz: *mut u32) -> i32;
    pub fn psks_ad9361_set_rx_rate(phy: *mut c_void, hz: u32) -> i32;
    pub fn psks_ad9361_get_rx_rate(phy: *mut c_void, hz: *mut u32) -> i32;
    pub fn psks_ad9361_select_fir(
        phy: *mut c_void,
        factor: u32,
        coef: *const i16,
        ntaps: u8,
    ) -> i32;
    pub fn psks_ad9361_fir_factors(
        phy: *mut c_void,
        rx_factor: *mut u32,
        tx_factor: *mut u32,
    ) -> i32;
    pub fn psks_ad9361_set_tx_bw(phy: *mut c_void, hz: u32) -> i32;
    pub fn psks_ad9361_get_tx_bw(phy: *mut c_void, hz: *mut u32) -> i32;
    pub fn psks_ad9361_set_rx_bw(phy: *mut c_void, hz: u32) -> i32;
    pub fn psks_ad9361_get_rx_bw(phy: *mut c_void, hz: *mut u32) -> i32;
    pub fn psks_ad9361_set_tx_atten(phy: *mut c_void, ch: u8, mdb: u32) -> i32;
    pub fn psks_ad9361_get_tx_atten(phy: *mut c_void, ch: u8, mdb: *mut u32) -> i32;
    pub fn psks_ad9361_set_rx_gain(phy: *mut c_void, ch: u8, gain_db: i32) -> i32;
    pub fn psks_ad9361_get_rx_gain(phy: *mut c_void, ch: u8, gain_db: *mut i32) -> i32;
    pub fn psks_ad9361_set_gain_control_mode(phy: *mut c_void, ch: u8, mode: u8) -> i32;
    pub fn psks_ad9361_get_gain_control_mode(phy: *mut c_void, ch: u8, mode: *mut u8) -> i32;
    pub fn psks_ad9361_set_rx_port(phy: *mut c_void, mode: u32) -> i32;
    pub fn psks_ad9361_set_tx_port(phy: *mut c_void, mode: u32) -> i32;
    pub fn psks_ad9361_get_temperature(phy: *mut c_void, millidegrees: *mut i32) -> i32;
    pub fn psks_ad9361_get_rx_rssi(
        phy: *mut c_void,
        ch: u8,
        symbol: *mut i32,
        multiplier: *mut i32,
    ) -> i32;
    pub fn psks_ad9361_free(phy: *mut c_void);
    pub fn psks_ad9361_set_fdd(phy: *mut c_void) -> i32;
    pub fn psks_ad9361_tx_analog_off(phy: *mut c_void, saved: *mut u16) -> i32;
    pub fn psks_ad9361_restore_tx_analog(phy: *mut c_void, saved: u16) -> i32;
    pub fn psks_ad9361_ensm_state(phy: *mut c_void) -> u8;
    pub fn psks_ad9361_tx_lo_locked(phy: *mut c_void) -> i32;
    pub fn psks_ad9361_rx_lo_locked(phy: *mut c_void) -> i32;
    pub fn psks_ad9361_bist_prbs_rx(phy: *mut c_void, on: u8) -> i32;
    pub fn psks_ad9361_bist_loopback(phy: *mut c_void, mode: i32) -> i32;
    pub fn psks_ad9361_get_interface_delay(phy: *mut c_void, tx: u8) -> i32;
    pub fn psks_ad9361_set_interface_delay(
        phy: *mut c_void,
        tx: u8,
        clock_delay: u8,
        data_delay: u8,
    ) -> i32;
}
