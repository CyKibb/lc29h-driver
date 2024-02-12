
#include "qc_lc29_driver.h" // Make sure to include your header file
#include "qc_lc29_driver_internal.h"

#include "gnss_driver_tests.h"

#include <check.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*
 *
 *   LC29 Driver Private Method Tests
 *
 */
START_TEST(test_lc29driver_construct) {
  qc_lc29_driver_s *driver = Lc29_driver_ctor(
      driverA_init, driverA_write, driverA_read_min_snr, driverA_config);

  ck_assert_int_eq(driver->fix_rate, ONE_HZ);
  ck_assert_int_eq(driver->min_snr, 9);
  ck_assert_int_eq(driver->sat_elevation_mask, 5);
  ck_assert_int_eq(driver->nav_mode, NORMAL_MODE);
  ck_assert_int_eq(driver->nmea_output_rate.gga.output_rate, 1);
  ck_assert_int_eq(driver->lc29_driver_hw_init(), DRIVER_SUCCESS);
}
END_TEST

START_TEST(test_lc29_driver_parse_response) {
  char response_string[] = "$PAIR001,050,0*3E\n\r";
  uint16_t command_id = 50;
  qc_lc29x_ack_reponse_t result =
      lc29_driver_parse_response(response_string, command_id);
  ck_assert_int_eq(result, 0);
}
END_TEST

START_TEST(test_lc29_driver_validate_string) {
  char sentence[] = "$PAIR001,050,0*3E\n\r";
  size_t length = strlen(sentence);
  int check_checksum = 1;
  qc_lc29x_response_error_t result =
      lc29_driver_validate_string(PAIR_ACK, sentence, length, check_checksum);
  ck_assert_int_eq(result, VALID_RESPONSE);
}
END_TEST

START_TEST(test_lc29_driver_pair_cmd_builder) {
  char fix_rate_packet[50];
  char *cmd_id = PAIR_COMMON_SET_FIX_RATE;
  uint8_t num_args = 1;
  char *args_test_1[] = {"100"};
  ck_assert_int_eq(lc29_driver_build_pair_cmd(num_args, cmd_id, args_test_1,
                                              fix_rate_packet),
                   VALID_RESPONSE);

  ck_assert_int_eq(lc29_driver_build_pair_cmd(num_args, cmd_id, args_test_1,
                                              fix_rate_packet),
                   VALID_RESPONSE);
}
END_TEST

/*
 *
 *   LC29 Driver Setter Methods Tests
 *
 */
START_TEST(test_lc29_fix_rate_methods) {
  qc_lc29_driver_s *driver = Lc29_driver_ctor(
      driverA_init, driverA_write, driverA_read_fix_rate, driverA_config);
  char fix_rate[] = "100";
  ck_assert_int_eq(driver->fix_rate, ONE_HZ);
  ck_assert_int_eq(lc29_driver_set_fix_rate(driver, fix_rate),
                   CMD_SEND_SUCCESS);
  ck_assert_int_eq(driver->fix_rate, TEN_HZ);
}
END_TEST

START_TEST(test_lc29_min_snr_methods) {
  qc_lc29_driver_s *driver = Lc29_driver_ctor(
      driverA_init, driverA_write, driverA_read_min_snr, driverA_config);
  char min_snr[] = "15";
  ck_assert_int_eq(driver->min_snr, 9);
  ck_assert_int_eq(lc29_driver_set_min_snr(driver, min_snr), CMD_SEND_SUCCESS);
  ck_assert_int_eq(driver->min_snr, 15);
}
END_TEST

START_TEST(test_lc29_nmea_set_output_rate_methods) {
  qc_lc29_driver_s *driver =
      Lc29_driver_ctor(driverA_init, driverA_write,
                       driverA_read_nmea_output_rate, driverA_config);
  char nmea_output_rate[] = "20";
  char nmea_output_rate_id[] = "0";
  ck_assert_int_eq(driver->nmea_output_rate.gga.output_rate, 1);
  ck_assert_int_eq(lc29_driver_set_nmea_output_rate(driver, nmea_output_rate_id,
                                                    nmea_output_rate),
                   CMD_SEND_SUCCESS);
  ck_assert_int_eq(driver->nmea_output_rate.gga.output_rate, 20);
}
END_TEST

START_TEST(test_lc29_nmea_search_mode_methods) {
  qc_lc29_driver_s *driver =
      Lc29_driver_ctor(driverA_init, driverA_write,
                       driverA_read_search_mode_settings, driverA_config);
  qc_lc29x_gnss_search_mode_s search_setting = {
      .gps_enabled = true,
      .galileo_enabled = false,
      .glonass_enabled = true,
      .beidou_enabled = false,
      .qzss_enabled = false,
      .reserved = false,
  };
  ck_assert_int_eq(driver->nmea_output_rate.gga.output_rate, 1);
  ck_assert_int_eq(lc29_driver_set_gnss_search_mode(driver, search_setting),
                   CMD_SEND_SUCCESS);
  ck_assert_int_eq(driver->gnss_search_mode.gps_enabled, true);
  ck_assert_int_eq(driver->gnss_search_mode.galileo_enabled, false);
  ck_assert_int_eq(driver->gnss_search_mode.glonass_enabled, true);
  ck_assert_int_eq(driver->gnss_search_mode.beidou_enabled, false);
  ck_assert_int_eq(driver->gnss_search_mode.qzss_enabled, false);
  ck_assert_int_eq(driver->gnss_search_mode.reserved, false);
}
END_TEST

START_TEST(test_lc29_set_static_speed_threshold) {
  qc_lc29_driver_s *driver =
      Lc29_driver_ctor(driverA_init, driverA_write,
                       driverA_read_static_threshold, driverA_config);
  char new_static_spd_threshold[] = "20";
  ck_assert_int_eq(driver->static_spd_thrshld, 0);
  ck_assert_int_eq(
      lc29_driver_set_static_threshold(driver, new_static_spd_threshold),
      CMD_SEND_SUCCESS);
  ck_assert_int_eq(driver->static_spd_thrshld, 20);
}
END_TEST

START_TEST(test_lc29_set_nav_mode) {
  // TODO
}
END_TEST

START_TEST(test_lc29_set_decimal_precision) {
  // TODO
}
END_TEST

START_TEST(test_lc29_set_nmea_output_mode) {
  // TODO
}
END_TEST

START_TEST(test_lc29_set_dual_band_mode) {
  // TODO
}
END_TEST

START_TEST(test_lc29_set_sbas_mode) {
  // TODO
}
END_TEST

START_TEST(test_lc29_set_easy_status) {
  // TODO
}
END_TEST

START_TEST(test_lc29_set_nvm_save_setting) {
  // TODO
}
END_TEST

START_TEST(test_lc29_set_low_power_mode) {
  // TODO
}
END_TEST

START_TEST(test_lc29_set_io_baudrate) {
  // TODO
}
END_TEST

/*
 *
 *   LC29 Driver Query (Getter) Methods Test
 *
 */
START_TEST(test_lc29_query_parser) {

  char cmd_response_string[] = "$PAIR001,063,0*3E\n\r";
  char query_response_string[] = "$PAIR063,0,3*3C\n\r";
  int query_response_vals[10];
  uint16_t command_id = 63;
  char pair_cmd_id[] = "$PAIR063";
  size_t length = strlen(query_response_string);

  // parse the cmd response to see if it is a valid cmd
  qc_lc29x_ack_reponse_t result =
      lc29_driver_parse_response(cmd_response_string, command_id);
  ck_assert_int_eq(result, 0);

  // parse the query response
  ck_assert_int_eq(lc29_driver_parse_query_response(query_response_string,
                                                    pair_cmd_id, length, 2,
                                                    query_response_vals),
                   CMD_SEND_SUCCESS);
  ck_assert_int_eq(query_response_vals[0], 0);
  ck_assert_int_eq(query_response_vals[1], 3);
}
END_TEST

START_TEST(test_lc29_query_fix_rate) {
  qc_lc29_driver_s *driver =
      Lc29_driver_ctor(driverA_init, driverA_write,
                       driverA_query_fix_rate_response, driverA_config);

  ck_assert_int_eq(lc29_driver_get_fix_rate(driver), CMD_SEND_SUCCESS);

  ck_assert_int_eq(driver->fix_rate, 1000);
}
END_TEST

START_TEST(test_lc29_query_min_snr) {
  qc_lc29_driver_s *driver =
      Lc29_driver_ctor(driverA_init, driverA_write,
                       driverA_query_min_snr_response, driverA_config);

  ck_assert_int_eq(lc29_driver_get_min_snr(driver), CMD_SEND_SUCCESS);

  ck_assert_int_eq(driver->min_snr, 15);
}
END_TEST

/*
 *
 *   LC29 Driver PQTM DR RTK Methods Test
 *
 */

START_TEST(test_lc29_dr_parser) {
  char *lc29_dr_response_ok = "$PQTMCFGEINSMSGOK*16\n\r";
  char *lc29_dr_response_error = "$PQTMCFGEINSMSGERROR*4A\n\r";

  ck_assert_int_eq(lc29_driver_parse_dr_cmd_response(
                       lc29_dr_response_ok, strlen(lc29_dr_response_ok),
                       LC29_DR_RESPONSE_OK, strlen(LC29_DR_RESPONSE_OK), 1),
                   CMD_SEND_SUCCESS);

  ck_assert_int_eq(lc29_driver_parse_dr_cmd_response(
                       lc29_dr_response_error, strlen(lc29_dr_response_error),
                       LC29_DR_RESPONSE_ERROR, strlen(LC29_DR_RESPONSE_ERROR),
                       1),
                   CMD_SEND_SUCCESS);
}
END_TEST

START_TEST(test_lc29_pqtm_set_method) {
  qc_lc29_driver_s *driver = Lc29_driver_ctor(
      driverA_init, driverA_write, driverA_query_pqtm_settings, driverA_config);

  ck_assert_int_eq(lc29_driver_set_get_pqtm_message_settings(driver, true, true,
                                                             true, true, "100"),
                   CMD_SEND_SUCCESS);
  ck_assert_int_eq(driver->dr_rtk_output_rate.imu.fix_rate, 100);
  ck_assert_int_eq(driver->dr_rtk_output_rate.gps.fix_rate, 10);
}
END_TEST

START_TEST(test_lc29_pqtm_get_method) {
  qc_lc29_driver_s *driver = Lc29_driver_ctor(
      driverA_init, driverA_write, driverA_query_pqtm_settings, driverA_config);

  ck_assert_int_eq(lc29_driver_set_get_pqtm_message_settings(
                       driver, false, true, true, true, "10"),
                   CMD_SEND_SUCCESS);
}
END_TEST

START_TEST(test_lc29_pqtm_dr_rtk_message_output_set_method) {
  qc_lc29_driver_s *driver =
      Lc29_driver_ctor(driverA_init, driverA_write,
                       driverA_query_dr_rtk_message_output, driverA_config);

  ck_assert_int_eq(lc29_driver_get_dr_rtk_message_output(driver, "1"),
                   CMD_SEND_SUCCESS);
  ck_assert_int_eq(driver->dr_rtk_custom_message_settings.sensor_output.enabled,
                   false);
}
END_TEST

Suite *str_suite(void) {
  Suite *s;
  TCase *tc_core;

  s = suite_create("LC29 Driver");

  /* Core test case */
  tc_core = tcase_create("Core");
  tcase_add_test(tc_core, test_lc29driver_construct);
  tcase_add_test(tc_core, test_lc29_driver_parse_response);
  tcase_add_test(tc_core, test_lc29_driver_validate_string);
  tcase_add_test(tc_core, test_lc29_driver_pair_cmd_builder);
  tcase_add_test(tc_core, test_lc29_fix_rate_methods);
  tcase_add_test(tc_core, test_lc29_min_snr_methods);
  tcase_add_test(tc_core, test_lc29_nmea_set_output_rate_methods);
  tcase_add_test(tc_core, test_lc29_nmea_search_mode_methods);
  tcase_add_test(tc_core, test_lc29_set_static_speed_threshold);
  tcase_add_test(tc_core, test_lc29_query_parser);
  tcase_add_test(tc_core, test_lc29_query_fix_rate);
  tcase_add_test(tc_core, test_lc29_query_min_snr);
  tcase_add_test(tc_core, test_lc29_dr_parser);
  tcase_add_test(tc_core, test_lc29_pqtm_set_method);
  tcase_add_test(tc_core, test_lc29_pqtm_dr_rtk_message_output_set_method);
  suite_add_tcase(s, tc_core);

  return s;
}

int main(void) {
  int number_failed;
  Suite *s;
  SRunner *sr;

  s = str_suite();
  sr = srunner_create(s);

  srunner_run_all(sr, CK_NORMAL);
  number_failed = srunner_ntests_failed(sr);
  srunner_free(sr);
  return (number_failed == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}
