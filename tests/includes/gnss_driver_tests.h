#ifndef GNSS_DRIVER_TESTS_H_INCLUDED
#define GNSS_DRIVER_TESTS_H_INCLUDED

#include "qc_lc29_driver.h"

qc_lc29x_driver_response_t driverA_init(void);
qc_lc29x_driver_response_t driverA_write(char *data, int length);
qc_lc29x_driver_response_t driverA_read_fix_rate(char *data, int length);
qc_lc29x_driver_response_t driverA_read_min_snr(char *data, int length);
qc_lc29x_driver_response_t driverA_config(char config);
qc_lc29x_driver_response_t driverA_read_nmea_output_rate(char *data,
                                                         int length);
qc_lc29x_driver_response_t driverA_read_search_mode_settings(char *data,
                                                             int length);
qc_lc29x_driver_response_t driverA_read_static_threshold(char *data,
                                                         int length);
qc_lc29x_driver_response_t driverA_query_fix_rate_response(char *data,
                                                           int length);
qc_lc29x_driver_response_t driverA_query_min_snr_response(char *data,
                                                          int length);

qc_lc29x_driver_response_t driverA_query_pqtm_settings(char *data, int length);

qc_lc29x_driver_response_t driverA_read_dr_rtk_message_output(char *data,
                                                              int length);

qc_lc29x_driver_response_t driverA_query_dr_rtk_message_output(char *data,
                                                               int length);
#endif