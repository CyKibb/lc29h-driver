#ifndef QC_LC29_DRIVER_INTERNAL_H_INCLUDED
#define QC_LC29_DRIVER_INTERNAL_H_INCLUDED

#include "qc_lc29_driver.h"
#include <stdbool.h>

struct qc_lc29_driver_s {
  qc_lc29x_fix_rate_t fix_rate;
  uint16_t min_snr;
  qc_lc29x_nmea_output_rate_s nmea_output_rate;
  // qc_lc29x_constellation_t active_const;
  qc_lc29x_gnss_search_mode_s gnss_search_mode;
  uint16_t static_search_mode;
  int16_t sat_elevation_mask;
  bool ait_enabled;
  qc_lc29x_nav_mode_t nav_mode;
  qc_lc29x_dec_accuracy_t decimal_accuracy;
  qc_nmea_output_mode nmea_output_mode;
  bool dual_band_enable;
  uint8_t static_spd_thrshld;
  bool gnss_jamming_detect_enable;
  qc_lc29x_dgps_mode_t dgps_mode;
  bool sbas_enable;
  bool easy_enable;
  qc_lc29x_easy_mode_status_t easy_status;
  uint32_t low_pwr_rtc_clk;
  qc_lc29x_periodic_sleep_mode_t sleep_mode; // Disable is default setting
  qc_lc29x_pps_setting_t pps_pin_setting;
  uint16_t baud_rate;
  qc_lc29x_pqtm_output_rate_settings_t dr_rtk_output_rate;
  qc_lc29x_pqtm_custom_message_settings_t dr_rtk_custom_message_settings;
  qc_lc29x_driver_response_t (*lc29_driver_hw_init)(void);
  qc_lc29x_driver_response_t (*lc29_driver_write)(char *data, int length);
  qc_lc29x_driver_response_t (*lc29_driver_read)(char *data, int length);
  qc_lc29x_driver_response_t (*lc29_driver_config)(char config);
};

qc_lc29_driver_s *lc29_driver_init(
    qc_lc29_driver_s *driver,
    qc_lc29x_driver_response_t (*lc29_driver_hw_init)(void),
    qc_lc29x_driver_response_t (*lc29_driver_write)(char *data, int length),
    qc_lc29x_driver_response_t (*lc29_driver_read)(char *data, int length),
    qc_lc29x_driver_response_t (*lc29_driver_config)(char config));

int lc29_driver_parse_string_by_comma(int cmd_id_len, char *string, int *values,
                                      int max_values);
char *Lc29_driver_crop_sentence(char *sentence, size_t length);

#endif