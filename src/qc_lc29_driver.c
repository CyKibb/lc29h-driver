/*
  Quectel GNSS DR Module LC29X Driver
  Written by: @Cykibb

  TODO List:
  - Clean-up getter and setter methods to support transmit_cmd() method
  - Add support for dead-reckoning commands

---

Quectel LC29H and LC79H GNSS modules support GPS, Galileo, GLONASS, BeiDou, QZSS
constellations. Concurrent tracking of GPS L1 C/A, GLONASS L1, BeiDou B1,
Galileo E1, GPS L5, BeiDou B2a, and Galileo E5a frequency bands provides fast
and accurate acquisition and makes these modules ideal solutions for positioning
and navigation in various vertical markets. This document describes the software
commands that are needed to control and modify the module configuration. The
software commands are NMEA proprietary commands defined by Quectel (PQTM
messages) and the chipset supplier (PAIR messages). To report GNSS information,
the modules support output messages in NMEA 0183 standard protocol format.

---

This document describes the dead reckoning (DR) and real-time kinematic (RTK)
features, including DR and RTK configurations and DR related messages for
Quectel LC29H (BA), LC29H (CA), LC29H (DA) and LC29H (EA) modules. The features
supported by each module are as follows:
-  LC29H (BA) supports DR and RTK. (update rate: , nav rate: )
-  LC29H (CA) only supports DR. (update rate: , nav rate: )
-  LC29H (DA) only supports RTK (update rate: 1 Hz).
-  LC29H (EA) only supports RTK (update rate: 1–10 Hz, 10 Hz by default).

LC29H (BA) and LC29H (CA) support two DR modes: ADR (Automotive Dead Reckoning)
and UDR (Untethered Dead Reckoning). In ADR mode, the module relies on speed
data from the vehicle and the onboard 6-axis sensor for enhanced accuracy in
environments with nonexistent GNSS coverage. The UDR mode does not require speed
data. The firmware automatically switches to UDR mode if no speed data is
injected upon module power-up.

---
*/

#include "qc_lc29_driver_internal.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Quectel LC29 Driver Class Constructor
qc_lc29_driver_s *Lc29_driver_ctor(
    qc_lc29x_driver_response_t (*lc29_driver_hw_init)(void),
    qc_lc29x_driver_response_t (*lc29_driver_write)(char *data, int length),
    qc_lc29x_driver_response_t (*lc29_driver_read)(char *data, int length),
    qc_lc29x_driver_response_t (*lc29_driver_config)(char config)) {

  qc_lc29_driver_s *lc29_driver = malloc(sizeof(qc_lc29_driver_s));
  if (NULL == lc29_driver) {
    return NULL;
  }

  return lc29_driver_init(lc29_driver, lc29_driver_hw_init, lc29_driver_write,
                          lc29_driver_read, lc29_driver_config);
}

/*
 lc29_driver_init() reflects the default parameters that are laid out within the
 Quectel_LC29H&LC79H_GNSS_Protocol_Specification_v1.pdf to view default settings
 for LC29H modules...
*/
qc_lc29_driver_s *lc29_driver_init(
    qc_lc29_driver_s *driver,
    qc_lc29x_driver_response_t (*lc29_driver_hw_init)(void),
    qc_lc29x_driver_response_t (*lc29_driver_write)(char *data, int length),
    qc_lc29x_driver_response_t (*lc29_driver_read)(char *data, int length),
    qc_lc29x_driver_response_t (*lc29_driver_config)(char config)) {
  // Set Default Parameters for LC29 Module
  driver->fix_rate = ONE_HZ;
  driver->min_snr = 9;
  driver->nmea_output_rate = (qc_lc29x_nmea_output_rate_s){
      .gga = {NMEA_SEN_GGA, 1}, // Initialization for `gga`
      .gll = {NMEA_SEN_GLL, 1}, // Initialization for `gll`
      .gsa = {NMEA_SEN_GSA, 1}, // Initialization for `gsa`
      .gsv = {NMEA_SEN_GSV, 1}, // Initialization for `gsv`
      .rmc = {NMEA_SEN_RMC, 1}, // Initialization for `rmc`
      .vtg = {NMEA_SEN_VTG, 1}  // Initialization for `vtg`
  };
  driver->gnss_search_mode = (qc_lc29x_gnss_search_mode_s){
      .gps_enabled = true,
      .galileo_enabled = true,
      .glonass_enabled = true,
      .beidou_enabled = true,
      .qzss_enabled = true,
  };
  driver->static_search_mode = 0;
  driver->sat_elevation_mask = 5;
  driver->ait_enabled = true;
  driver->nav_mode = NORMAL_MODE;
  driver->decimal_accuracy = LAT_LON_6_ALT_3;
  driver->nmea_output_mode = ENABLE_ASCII_NMEA_4_10;
  driver->dual_band_enable = true;
  driver->gnss_jamming_detect_enable = true;
  driver->static_spd_thrshld = 0;
  driver->dgps_mode = SBAS_ENABLED;
  driver->sbas_enable = true;
  driver->easy_enable = true;
  driver->easy_status = NOT_INITIALIZED;
  driver->low_pwr_rtc_clk = 0;
  driver->sleep_mode = DISABLE_PERIODIC_MODE; // Disable is default setting
  driver->pps_pin_setting = FIX_ONLY_3D;
  driver->baud_rate = (uint16_t)LC29_BAUD_RATE_115200;
  driver->dr_rtk_output_rate = (qc_lc29x_pqtm_output_rate_settings_t){
      .ins = {PQTM_INS, false, 1},
      .imu = {PQTM_IMU, false, 1},
      .gps = {PQTM_GPS, false, 1},
  };
  driver->dr_rtk_custom_message_settings =
      (qc_lc29x_pqtm_custom_message_settings_t){
          .vehicle_info = {PQTMVEHMSG, false},
          .sensor_output = {PQTMSENMSG, false},
          .dr_calibration = {PQTMDRCAL, false},
          .imu_type = {PQTMIMUTYPE, true},
          .dr_vehicle_motion = {PQTMVEHMOT, false},
      };
  driver->lc29_driver_hw_init = lc29_driver_hw_init;
  driver->lc29_driver_read = lc29_driver_read;
  driver->lc29_driver_write = lc29_driver_write;
  driver->lc29_driver_config = lc29_driver_config;

  return driver;
}

/*
Parsing 2.4.1. Packet Type: 001 PAIR_ACK

This chapter explains PAIR messages (proprietary NMEA messages defined by the
chipset supplier). “P” means proprietary message, “AIR” means the command
defined by the chipset supplier. 2.4.1.

Packet Type: 001 PAIR_ACK Acknowledges a
PAIR command. An acknowledgement packet PAIR_ACK is returned to inform the
sender that the receiver has received the packet.

Type: Output.

Synopsis:
$PAIR001,<Command_ID>,<Result>*<Checksum><CR><LF>

Description <Result>:
The type of command/packet to be acknowledged.
0 = The command has been successfully sent
1 = The command is being processed. Please wait for the result
2 = Command sending failed
3 = The command ID is not supported
4 = Command parameter error. Out of range/some parameters were lost/checksum
error
5 = The MNL service is busy
*/
// TODO: (@Kibby) Adjust method name to parse_cmd_response
qc_lc29x_ack_reponse_t lc29_driver_parse_response(char *response_string,
                                                  int command_id) {

  char *cmd_token;
  char *cmd_response_token;
  int response_cmd_id, response_cmd_status;
  // validate incoming string
  qc_lc29x_response_error_t validate_response = lc29_driver_validate_string(
      PAIR_ACK, response_string, strlen(response_string), 1);

  if (validate_response != VALID_RESPONSE) {
    return CMD_SEND_FAIL;
  }

  // Check for if the received command id and response command id are the same
  // Extract the command ID
  cmd_token = strtok(response_string + 9, ",");
  if (cmd_token == NULL) {
    return CMD_SEND_FAIL; // Invalid input string format
  }

  response_cmd_id = atoi(cmd_token);
  if (response_cmd_id != command_id) {
    return CMD_SEND_FAIL;
  }

  cmd_response_token = strtok(response_string + 14, ",");
  if (cmd_response_token == NULL) {
    return CMD_SEND_FAIL; // Invalid input string format
  }
  // TODO (@Kibby) - Make a more robust check and condition handling for other
  // cases of command processing etc...
  response_cmd_status = atoi(cmd_response_token);
  if (response_cmd_status > 5) {
    return CMD_SEND_FAIL;
  }

  return response_cmd_status;
}

/*
  The LC29H gnss module will respond to a query request command with a result of
  the query. The query can contain 1 or more attributes. An example of a

  For example:
  1. Packet Type: 051 PAIR_COMMON_GET_FIX_RATE
  $PAIR051*3E       <-- QUERY REQUEST
  $PAIR001,051,0*3F <-- CMD RESPONSE
  $PAIR051,1000*13  <-- QUERY RESPONSE

  2. Packet Type: 067 PAIR_COMMON_GET_GNSS_SEARCH_MODE
  $PAIR067*3B              <-- QUERY REQUEST
  $PAIR001,067,0*3A        <-- CMD RESPONSE
  $PAIR067,1,0,0,0,0,0*3A  <-- QUERY RESPONSE
*/
qc_lc29x_ack_reponse_t lc29_driver_parse_query_response(char *response_string,
                                                        char *command_id,
                                                        int response_string_len,
                                                        int response_num_args,
                                                        int *parsed_query) {

  // int query_response_args = 0;

  // Step 1: validate incoming string
  qc_lc29x_response_error_t validate_response = lc29_driver_validate_string(
      command_id, response_string, response_string_len, 1);

  if (validate_response != VALID_RESPONSE) {
    return CMD_SEND_FAIL;
  }

  // Step 2: Parse the response values
  if (lc29_driver_parse_string_by_comma(strlen(command_id) + 1, response_string,
                                        parsed_query, response_num_args) !=
      response_num_args) {
    return CMD_SEND_FAIL;
  }

  return CMD_SEND_SUCCESS;
}

/*
Acknowledges a PAIR command. An acknowledgement packet PAIR_ACK is returned to
inform the sender that the receiver has received the packet.
*/
qc_lc29x_response_error_t lc29_driver_validate_string(char *pair_id,
                                                      const char *sentence,
                                                      size_t length,
                                                      int check_checksum) {
  /* should have atleast 9 characters */
  if (9 > length) {
    return LC_RESPONSE_INVALID_LENGTH;
  }

  /* should start with $ */
  if ('$' != *sentence) {
    return LC_RESPONSE_INVALID_START_CHAR;
  }
  /* should end with \r\n, or other... */
  if (strcmp(&sentence[length - 4], "\r\n") == 0) {
    return LC_RESPONSE_INVALID_R_N;
  }

  // Verify that the input string starts with '$PAIR001,' or general PAIR CMD ID
  if (strncmp(sentence, pair_id, 8) != 0) {
    return LC_RESPONSE_INVALID_IDENTIFIER; // Invalid input string format
  }

  // /* should have a comma after the type word */
  // if (',' != sentence[8]) {
  //   return LC_RESPONSE_INVALID_IDENTIFIER;
  // }
  /* check for checksum */
  if (0 != lc29_driver_response_has_checksum(sentence, length)) {
    return LC_RESPONSE_NO_CHECKSUM;
  }

  if (1 == check_checksum) {
    uint8_t actual_chk;
    uint8_t expected_chk;
    char checksum[3];

    checksum[0] = sentence[length - 4];
    checksum[1] = sentence[length - 3];
    checksum[2] = '\0';
    actual_chk = lc29_driver_get_checksum(sentence);
    expected_chk = (uint8_t)strtol(checksum, NULL, 16);

    if (expected_chk != actual_chk) {
      return LC_RESPONSE_INVALID_CHECKSUM;
    }
  }
  return VALID_RESPONSE;
}

char *Lc29_driver_crop_sentence(char *sentence, size_t length) {
  /* Skip type word, 7 characters (including $ and ,) */
  sentence += 8;
  /* Find the where the * is located and crop accordingly */
  char *ret = strchr(sentence, (int)'*');
  if (ret == NULL) {
    *sentence = '\0';
    return sentence;
  } else {
    *ret = '\0';
  }
  return sentence;
}

int lc29_driver_parse_string_by_comma(int cmd_id_len, char *string, int *values,
                                      int max_values) {
  // Skip beginning command_id chars $PAIRXXX,
  string += cmd_id_len;
  int i = 0;
  char *string_val = string;

  while (i < max_values && NULL != (string = strchr(string, ','))) {
    *string = '\0';
    values[i] = atoi(string_val);
    string_val = ++string;
    i++;
  }
  // Capture the ending chars before checksum...
  if (NULL != (string = strchr(string_val, '*'))) {
    *string = '\0';
    values[i] = atoi(string_val);
    i++;
  }

  return i;
}

uint8_t lc29_driver_response_has_checksum(const char *sentence, size_t length) {
  if ('*' == sentence[length - 5]) {
    return 0;
  }
  return LC_RESPONSE_NO_CHECKSUM;
}

uint8_t lc29_driver_get_checksum(const char *sentence) {
  const char *n = sentence + 1;
  uint8_t chk = 0;

  /* While current char isn't '*' or sentence ending (newline) */
  while ('*' != *n && LC29_RESPONSE_END_CHAR != *n && '\0' != *n) {
    chk ^= (uint8_t)*n;
    n++;
  }

  return chk;
}

/*
  Construct a method that takes a variable number of inputs in and then
  constructs the PAIR CMD...
  Note: cmd sentence is built based on the order the
  args are passed into the array and assumes the values are NOT comma
  seperated...
*/
qc_lc29x_response_error_t lc29_driver_build_pair_cmd(uint8_t num_args,
                                                     char *cmd_id, char *args[],
                                                     char *cmd_result) {

  char checksum[3];
  char *csv_seperator = ",";

  // Append the sentence header
  strcat(cmd_result, cmd_id);

  // Append the arguments
  for (u_int8_t i = 0; i < num_args; i++) {

    strcat(cmd_result, csv_seperator);
    strcat(cmd_result, args[i]);
  }

  // Append sentence ending char
  strcat(cmd_result, PAIR_CMD_ENDING_CHAR);
  // Get checksum
  uint8_t checksum_i = lc29_driver_get_checksum(cmd_result);
  sprintf(checksum, "%02X", checksum_i);
  strcat(cmd_result, checksum);
  strcat(cmd_result, LC29_CMD_END_CHARS);

  // Perform PAIR_CMD integrity check
  return lc29_driver_validate_string(cmd_id, cmd_result, strlen(cmd_result),
                                     checksum_i);
}

/*
  Method to clean up the code to handle the transmission and verification of
  $PAIR commands.
*/
// qc_lc29x_response_error_t lc29_driver_transmit_cmd(char *cmd_payload,
//                                                    char *cmd_response,
//                                                    int cmd_id) {

// // Step 2: Send query request
// if (driver->lc29_driver_write(cmd_payload, strlen(cmd_payload)) !=
//     DRIVER_SUCCESS) {
//   return CMD_SEND_FAIL;
// }

// // Step 3: Validate Command Response
// if (driver->lc29_driver_read(driver_cmd_response,
//                              strlen(driver_cmd_response)) != DRIVER_SUCCESS)
//                              {
//   return CMD_SEND_FAIL;
// }

// // Step 4: Check for command acceptance
// cmd_response = lc29_driver_parse_response(driver_cmd_response, cmd_id);
// if (cmd_response != CMD_SEND_SUCCESS) {
//   return cmd_response;
// }

//   return CMD_SEND_SUCCESS;
// }

// Driver Set Methods

/*
Sets position fix interval. The ULP (Ultra Low Power) mode only supports 1 Hz.
Type: Set. Synopsis: $PAIR050,<Time>*<Checksum><CR><LF>

Parameter: <Time>
Numeric Millisecond
Position fix interval. Range: 100–1000. Default value: 1000.

Result: Returns a PAIR_ACK message.

Example:
$PAIR050,1000*12
$PAIR001,050,0*3E
*/
qc_lc29x_ack_reponse_t lc29_driver_set_fix_rate(qc_lc29_driver_s *driver,
                                                char *fix_rate) {
  qc_lc29x_ack_reponse_t cmd_response;
  char driver_cmd_response[22] = {0};
  char fix_rate_packet[50] = {0};
  char *args[] = {fix_rate};

  // Build PAIR Command
  if (lc29_driver_build_pair_cmd(1, PAIR_COMMON_SET_FIX_RATE, args,
                                 fix_rate_packet) != VALID_RESPONSE) {
    return CMD_SEND_FAIL;
  }

  // Send request
  if (driver->lc29_driver_write(fix_rate_packet, strlen(fix_rate_packet)) !=
      DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }

  // Get LC29 response
  if (driver->lc29_driver_read(driver_cmd_response,
                               sizeof(driver_cmd_response)) != DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }
  // Validate response
  cmd_response = lc29_driver_parse_response(driver_cmd_response, 50);
  if (cmd_response != CMD_SEND_SUCCESS) {
    return cmd_response;
  }

  // update driver config
  driver->fix_rate = atoi(fix_rate);

  return CMD_SEND_SUCCESS;
}

/*
Sets the minimum SNR of satellites in use. If the minimum SNR threshold value is
set, the module will not use the satellites with SNR below the threshold.

Type:
Set.
Synopsis:
$PAIR058,<MIN_SNR>*<Checksum><CR><LF>
Parameter:
<MIN_SNR> Numeric dB-Hz
Minimum SNR threshold of satellites in use. Range: 9–37. Default value: 9

Result:
Returns a PAIR_ACK message.

Example:
$PAIR058,15*1F
$PAIR001,058,0*36
*/
qc_lc29x_ack_reponse_t lc29_driver_set_min_snr(qc_lc29_driver_s *driver,
                                               char *min_snr) {
  qc_lc29x_ack_reponse_t cmd_response;
  char driver_cmd_response[22] = {0};
  char min_snr_packet[50] = {0};
  char *args[] = {min_snr};

  uint16_t min_snr_i = atoi(min_snr);
  // Check to ensure SNR remains within bounds...
  if (min_snr_i < 9 || min_snr_i > 49) {
    return CMD_INVALID;
  }

  // Build PAIR Command
  if (lc29_driver_build_pair_cmd(1, PAIR_COMMON_SET_MIN_SNR, args,
                                 min_snr_packet) != VALID_RESPONSE) {
    return CMD_SEND_FAIL;
  }

  // Send request
  if (driver->lc29_driver_write(min_snr_packet, strlen(min_snr_packet)) !=
      DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }

  // Get LC29 response
  if (driver->lc29_driver_read(driver_cmd_response,
                               sizeof(driver_cmd_response)) != DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }
  // Validate response
  cmd_response = lc29_driver_parse_response(driver_cmd_response, 58);
  if (cmd_response != CMD_SEND_SUCCESS) {
    return cmd_response;
  }

  // update driver config
  driver->min_snr = min_snr_i;

  return CMD_SEND_SUCCESS;
}

/*
Sets the output interval of standard NMEA sentences of each type.

Set.
Synopsis:
$PAIR062,<Type>,<Output_Rate>*<Checksum><CR><LF>

Parameter:
<Type>
NMEA type:
-1 = Reset the output rates of all types of sentences to the default value
0 = NMEA_SEN_GGA
1 = NMEA_SEN_GLL
2 = NMEA_SEN_GSA
3 = NMEA_SEN_GSV
4 = NMEA_SEN_RMC
5 = NMEA_SEN_VTG

<Output_Rate>
RMC sentence output frequency:
0 = Disable sentence output
n = Output once every n position fixes Range of n: 0–20. Default value: 1

Result:
Returns a PAIR_ACK message.
Example:
 $PAIR062,0,3*3D
 $PAIR001,062,0*3F
*/
qc_lc29x_ack_reponse_t
lc29_driver_set_nmea_output_rate(qc_lc29_driver_s *driver, char *output_rate_id,
                                 char *output_rate) {
  qc_lc29x_ack_reponse_t cmd_response;
  char driver_cmd_response[22] = {0};
  char nmea_output_rate_packet[50] = {0};

  // Check to ensure output ID is valid remains within bounds...
  qc_lc29x_nmea_output_rate_id_t rate_id =
      (qc_lc29x_nmea_output_rate_id_t)atoi(output_rate_id);
  if (rate_id > 5 || rate_id < 0) {
    return CMD_INVALID;
  }

  // Check to ensure output rate remains within bounds...
  uint16_t output_rate_i = atoi(output_rate);
  if (output_rate_i > 20) {
    return CMD_INVALID;
  }

  char *args[] = {output_rate_id, output_rate};
  // Build PAIR Command
  if (lc29_driver_build_pair_cmd(2, PAIR_COMMON_SET_NMEA_OUTPUT_RATE, args,
                                 nmea_output_rate_packet) != VALID_RESPONSE) {
    return CMD_SEND_FAIL;
  }

  // Send request
  if (driver->lc29_driver_write(nmea_output_rate_packet,
                                strlen(nmea_output_rate_packet)) !=
      DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }

  // Get LC29 response
  if (driver->lc29_driver_read(driver_cmd_response,
                               sizeof(driver_cmd_response)) != DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }
  // Validate response
  cmd_response = lc29_driver_parse_response(driver_cmd_response, 62);
  if (cmd_response != CMD_SEND_SUCCESS) {
    return cmd_response;
  }

  // update driver config
  switch (rate_id) {
  case NMEA_SEN_GGA:
    driver->nmea_output_rate.gga.output_rate = output_rate_i;
    break;
  case NMEA_SEN_GLL:
    driver->nmea_output_rate.gll.output_rate = output_rate_i;
    break;
  case NMEA_SEN_GSA:
    driver->nmea_output_rate.gsa.output_rate = output_rate_i;
    break;
  case NMEA_SEN_GSV:
    driver->nmea_output_rate.gsv.output_rate = output_rate_i;
    break;
  case NMEA_SEN_RMC:
    driver->nmea_output_rate.rmc.output_rate = output_rate_i;
    break;
  case NMEA_SEN_VTG:
    driver->nmea_output_rate.vtg.output_rate = output_rate_i;
    break;
  default:
    break;
  }

  return CMD_SEND_SUCCESS;
}

/*
Sets the types of GNSS satellites the module searches for. The setting is valid
when the NVM data are valid. The module is restarted when it receives this
command. S

Synopsis:
$PAIR066,<GPS_Enabled>,<GLONASS_Enabled>,<Galileo_Enabled>,<BeiDou_Enabled>,<QZSS_En
abled>,0*<Checksum><CR><LF>

Parameters:
<GPS_Enabled>
0 = Disable (Do NOT Search for GPS satellites)
1 = Search for GPS Satellites
<GLONASS_Enabled>
0 = Disable (Do NOT Search for GLONNASS satellites)
1 = Search for GLONASS Satellites
<Galileo_Enabled>
0 = Disable (Do NOT Search for Galileo satellites)
1 = Search for Galileo Satellites
<BeiDou_Enabled>
0 = Disable (Do NOT Search for BeiDou satellites)
1 = Search for BeiDou Satellites
<QZSS_Enabled>
0 = Disable (Do NOT Search for QZSS satellites)
1 = Search for QZSS Satellites
<Reserved>
Keep it as 0

Result:
Returns a PAIR_ACK message.

Example:
//Search for GPS + GLONASS + Galileo + BeiDou satellites:
$PAIR066,1,1,1,1,0,0*3A
$PAIR001,066,0*3B

Note: LC29H and LC79H modules are capable of accessing GPS, Galileo, GLONASS,
BeiDou, and QZSS systems. When dual bands (L1 + L5) are enabled (default), only
the following GNSS search mode is supported:

-GPS + Galileo + GLONASS + BeiDou + QZSS When only the single band L1 is enabled
with PAIR104,0, the following GNSS search modes are supported:
*/
qc_lc29x_ack_reponse_t lc29_driver_set_gnss_search_mode(
    qc_lc29_driver_s *driver,
    qc_lc29x_gnss_search_mode_s search_mode_settings) {
  qc_lc29x_ack_reponse_t cmd_response;
  char driver_cmd_response[22] = {0};
  char gnss_search_mode_packet[50] = {0};
  // char default_param
  char *args[] = {search_mode_settings.gps_enabled ? "1" : "0",
                  search_mode_settings.glonass_enabled ? "1" : "0",
                  search_mode_settings.galileo_enabled ? "1" : "0",
                  search_mode_settings.beidou_enabled ? "1" : "0",
                  search_mode_settings.qzss_enabled ? "1" : "0",
                  "0"};
  // Build PAIR Command
  if (lc29_driver_build_pair_cmd(6, PAIR_COMMON_SET_GNSS_SEARCH_MODE, args,
                                 gnss_search_mode_packet) != VALID_RESPONSE) {
    return CMD_SEND_FAIL;
  }

  // Send request
  if (driver->lc29_driver_write(gnss_search_mode_packet,
                                strlen(gnss_search_mode_packet)) !=
      DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }

  // Get LC29 response
  if (driver->lc29_driver_read(driver_cmd_response,
                               sizeof(driver_cmd_response)) != DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }
  // Validate response
  cmd_response = lc29_driver_parse_response(driver_cmd_response, 66);
  if (cmd_response != CMD_SEND_SUCCESS) {
    return cmd_response;
  }

  // update driver config
  driver->gnss_search_mode.gps_enabled = search_mode_settings.gps_enabled;
  driver->gnss_search_mode.glonass_enabled =
      search_mode_settings.glonass_enabled;
  driver->gnss_search_mode.galileo_enabled =
      search_mode_settings.galileo_enabled;
  driver->gnss_search_mode.beidou_enabled = search_mode_settings.beidou_enabled;
  driver->gnss_search_mode.qzss_enabled = search_mode_settings.qzss_enabled;
  driver->gnss_search_mode.reserved = false;

  return CMD_SEND_SUCCESS;
}

/*
Sets the speed threshold for static navigation. If the actual speed is below the
threshold, the output position remains unchanged and the output speed is zero.
If the threshold value is set to 0, this function is disabled.

Synopsis:
$PAIR070,<Speed_threshold>*<Checksum><CR><LF>

Parameter:
<Speed_threshold>
Numeric dm/s Speed threshold. Range: 0–20. Default value: 0

Result:
Returns a PAIR_ACK message.

Example:
$PAIR070,4*25
$PAIR001,070,0*3C
*/
qc_lc29x_ack_reponse_t
lc29_driver_set_static_threshold(qc_lc29_driver_s *driver,
                                 char *speed_threshold) {
  qc_lc29x_ack_reponse_t cmd_response;
  char driver_cmd_response[22] = {0};
  char static_spd_thshld_packet[50] = {0};
  char *args[] = {speed_threshold};

  uint16_t spd_threshold_i = atoi(speed_threshold);
  // Check to ensure SNR remains within bounds...
  if (spd_threshold_i > 20) {
    return CMD_INVALID;
  }

  // Build PAIR Command
  if (lc29_driver_build_pair_cmd(1, PAIR_COMMON_SET_STATIC_THRESHOLD, args,
                                 static_spd_thshld_packet) != VALID_RESPONSE) {
    return CMD_SEND_FAIL;
  }

  // Send request
  if (driver->lc29_driver_write(static_spd_thshld_packet,
                                strlen(static_spd_thshld_packet)) !=
      DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }

  // Get LC29 response
  if (driver->lc29_driver_read(driver_cmd_response,
                               sizeof(driver_cmd_response)) != DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }
  // Validate response
  cmd_response = lc29_driver_parse_response(driver_cmd_response, 70);
  if (cmd_response != CMD_SEND_SUCCESS) {
    return cmd_response;
  }

  // update driver config
  driver->static_spd_thrshld = spd_threshold_i;

  return CMD_SEND_SUCCESS;
}

/*
Synopsis:
$PAIR080,<CmdType>*<Checksum><CR><LF>

<CmdType> Numeric -
0 = Normal Mode. For general purposes. (default)
1 = Fitness Mode: For running and walking purposes so that the
low-speed (< 5 m/s) movement will have more effect on position calculation.
2 = Reserved
3 = Reserved
4 = Stationary Mode: For stationary applications where zero dynamics is assumed.
5 = Reserved
6 = Reserved
7 = Swimming Mode: For swimming purpose so that it smooths the trajectory and
improves the accuracy of distance calculation.

Result:
Returns a PAIR_ACK message.

Example:
$PAIR080,1*2F
$PAIR001,080,0*33
*/
qc_lc29x_ack_reponse_t
lc29_driver_set_navigation_mode(qc_lc29_driver_s *driver,
                                qc_lc29x_nav_mode_t nav_mode) {
  qc_lc29x_ack_reponse_t cmd_response;
  char driver_cmd_response[22] = {0};
  char payload[50] = {0};
  char *arg;
  // Determine arg based on nav_mode_t
  switch (nav_mode) {
  case NORMAL_MODE:
    arg = "0";
    break;
  case FITNESS_MODE:
    arg = "1";
    break;
  case STATIONARY_MODE:
    arg = "4";
    break;
  case SWIMMING_MODE:
    arg = "7";
    break;
  default:
    return CMD_INVALID;
  }

  char *args[] = {arg};
  // Build PAIR Command
  if (lc29_driver_build_pair_cmd(1, PAIR_COMMON_SET_NAVIGATION_MODE, args,
                                 payload) != VALID_RESPONSE) {
    return CMD_SEND_FAIL;
  }

  // Send request
  if (driver->lc29_driver_write(payload, strlen(payload)) != DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }

  // Get LC29 response
  if (driver->lc29_driver_read(driver_cmd_response,
                               sizeof(driver_cmd_response)) != DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }
  // Validate response
  cmd_response = lc29_driver_parse_response(driver_cmd_response, 80);
  if (cmd_response != CMD_SEND_SUCCESS) {
    return cmd_response;
  }

  // update driver config
  driver->nav_mode = nav_mode;

  return CMD_SEND_SUCCESS;
}

/*
Sets the coordinates precision, i.e., the decimal places in the output
coordinates.

Synopsis:
$PAIR100,<NMEA_Mode>,<PROPRIETARY_Mode>*<Checksum><CR><LF>

Numeric -
0 = Latitude, Longitude: 4; Altitude: 1.
1 = Latitude, Longitude: 5; Altitude: 2.
2 = Latitude, Longitude: 6; Altitude: 3 (default)
3 = Latitude, Longitude: 7; Altitude: 3.

Example:
$PAIR098,0*27
$PAIR001,098,0*3A

*/
qc_lc29x_ack_reponse_t
lc29_driver_set_nmea_decimal_precision(qc_lc29_driver_s *driver,
                                       qc_lc29x_dec_accuracy_t accuracy) {

  qc_lc29x_ack_reponse_t cmd_response;
  char driver_cmd_response[22] = {0};
  char payload[50] = {0};
  char *arg;
  // Determine arg based on nav_mode_t
  switch (accuracy) {
  case LAT_LON_4_ALT_1:
    arg = "0";
    break;
  case LAT_LON_5_ALT_2:
    arg = "1";
    break;
  case LAT_LON_6_ALT_3:
    arg = "2";
    break;
  case LAT_LON_7_ALT_3:
    arg = "3";
    break;
  default:
    return CMD_INVALID;
  }

  char *args[] = {arg};
  // Build PAIR Command
  if (lc29_driver_build_pair_cmd(1, PAIR_COMMON_SET_NMEA_POS_DECIMAL_PRECISION,
                                 args, payload) != VALID_RESPONSE) {
    return CMD_SEND_FAIL;
  }

  // Send request
  if (driver->lc29_driver_write(payload, strlen(payload)) != DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }

  // Get LC29 response
  if (driver->lc29_driver_read(payload, sizeof(payload)) != DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }
  // Validate response
  cmd_response = lc29_driver_parse_response(driver_cmd_response, 98);
  if (cmd_response != CMD_SEND_SUCCESS) {
    return cmd_response;
  }

  // update driver config
  driver->decimal_accuracy = accuracy;

  return CMD_SEND_SUCCESS;
}

/*
Sets NMEA output mode.
Field
<NMEA_Mode>
<PROPRIETARY_Mode>


Unit Description
0 = Disable NMEA output
- 1 = Enable ASCII NMEA 4.10 output (default)
2 = Enable ASCII NMEA 3.01 output
0 = Disable proprietary sentence output - (default)
1 = Enable proprietary sentence output

Result:
Returns a PAIR_ACK message.

Example:
$PAIR100,1,0*3A $PAIR001,100,0*3A
*/
qc_lc29x_ack_reponse_t
lc29_driver_set_nmea_output_mode(qc_lc29_driver_s *driver,
                                 qc_nmea_output_mode nmea_output_mode,
                                 bool proprietary_mode) {

  qc_lc29x_ack_reponse_t cmd_response;
  char driver_cmd_response[22] = {0};
  char payload[50] = {0};
  char *arg;
  // Determine arg based on nav_mode_t
  switch (nmea_output_mode) {
  case DISABLE_NMEA_OUTPUT:
    arg = "0";
    break;
  case ENABLE_ASCII_NMEA_4_10:
    arg = "1";
    break;
  case ENABLE_ASCII_NMEA_3_01:
    arg = "2";
    break;
  default:
    return CMD_INVALID;
  }

  char *args[] = {arg, proprietary_mode ? "1" : "0"};
  // Build PAIR Command
  if (lc29_driver_build_pair_cmd(2, PAIR_COMMON_SET_NMEA_OUTPUT_MODE, args,
                                 payload) != VALID_RESPONSE) {
    return CMD_SEND_FAIL;
  }

  // Send request
  if (driver->lc29_driver_write(payload, strlen(payload)) != DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }

  // Get LC29 response
  if (driver->lc29_driver_read(driver_cmd_response,
                               sizeof(driver_cmd_response)) != DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }
  // Validate response
  cmd_response = lc29_driver_parse_response(driver_cmd_response, 100);
  if (cmd_response != CMD_SEND_SUCCESS) {
    return cmd_response;
  }

  // update driver config
  driver->nmea_output_mode = nmea_output_mode;

  return CMD_SEND_SUCCESS;
}

/*

Sets Dual Band state when the GNSS system is powered off.

Synopsis:
$PAIR104,<DUAL_BAND_Enable>*<Checksum><CR><LF>

Parameter:
<DUAL_BAND_Enable> Numeric -
Result:
Returns a PAIR_ACK message.
Example:
0 = Disable
1 = Enable (default)

Example:
$PAIR104,0*23
$PAIR001,104,0*3E

*/
qc_lc29x_ack_reponse_t lc29_driver_set_dual_band_mode(qc_lc29_driver_s *driver,
                                                      bool enable) {
  qc_lc29x_ack_reponse_t cmd_response;
  char driver_cmd_response[22] = {0};
  char payload[50] = {0};

  char *args[] = {enable ? "1" : "0"};
  // Build PAIR Command
  if (lc29_driver_build_pair_cmd(1, PAIR_COMMON_SET_DUAL_BAND, args, payload) !=
      VALID_RESPONSE) {
    return CMD_SEND_FAIL;
  }

  // Send request
  if (driver->lc29_driver_write(payload, strlen(payload)) != DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }

  // Get LC29 response
  if (driver->lc29_driver_read(driver_cmd_response,
                               sizeof(driver_cmd_response)) != DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }
  // Validate response
  cmd_response = lc29_driver_parse_response(driver_cmd_response, 104);
  if (cmd_response != CMD_SEND_SUCCESS) {
    return cmd_response;
  }

  // update driver config
  driver->dual_band_enable = enable;

  return CMD_SEND_SUCCESS;
}

/*
Enables or disables the searching of SBAS satellites. SBAS supports wide-area or
regional augmentation through geostationary satellite broadcast messages. The
geostationary satellites broadcast GNSS integrity and correction data with the
assistance of multiple ground stations that are located at accurately-surveyed
points.

Synopsis:
$PAIR410,<Enabled>*<Checksum><CR><LF>

<Enabled> Numeric -
Result:
Returns a PAIR_ACK message.
Example:
0 = Disable
1 = Enable (default)

$PAIR490,1*2A,
$PAIR001,490,0*36
*/
qc_lc29x_ack_reponse_t lc29_driver_set_sbas_mode(qc_lc29_driver_s *driver,
                                                 bool enable) {
  qc_lc29x_ack_reponse_t cmd_response;
  char driver_cmd_response[22] = {0};
  char payload[50] = {0};

  char *args[] = {enable ? "1" : "0"};
  // Build PAIR Command
  if (lc29_driver_build_pair_cmd(1, PAIR_SBAS_ENABLE, args, payload) !=
      VALID_RESPONSE) {
    return CMD_SEND_FAIL;
  }

  // Send request
  if (driver->lc29_driver_write(payload, strlen(payload)) != DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }

  // Get LC29 response
  if (driver->lc29_driver_read(driver_cmd_response,
                               sizeof(driver_cmd_response)) != DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }
  // Validate response
  cmd_response = lc29_driver_parse_response(driver_cmd_response, 410);
  if (cmd_response != CMD_SEND_SUCCESS) {
    return cmd_response;
  }

  // update driver config
  driver->sbas_enable = enable;

  return CMD_SEND_SUCCESS;
}

/*
Queries the status of EASY^TM.
Type:
Get.
Synopsis:
$PAIR491,<Enable>,<Status>*<Checksum><CR><LF>
Result:
Returns a PAIR_ACK message and the query result.


Query result message format:
Numeric -
Numeric -
Enabled or disabled:
<Enable>
0 = Disabled
1 = Enabled (default)
<Status>
0 = Not finished
1 = Finished 1-day extension 2 = Finished 2-day extension 3 = Finished 3-day
extension

Example:
$PAIR490,1*2A
$PAIR001,490,0*36
*/
qc_lc29x_ack_reponse_t lc29_driver_set_easy_status(qc_lc29_driver_s *driver,
                                                   bool enable) {
  qc_lc29x_ack_reponse_t cmd_response;
  char driver_cmd_response[22] = {0};
  char payload[50] = {0};

  char *args[] = {enable ? "1" : "0"};
  // Build PAIR Command
  if (lc29_driver_build_pair_cmd(1, PAIR_EASY_ENABLE, args, payload) !=
      VALID_RESPONSE) {
    return CMD_SEND_FAIL;
  }

  // Send request
  if (driver->lc29_driver_write(payload, strlen(payload)) != DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }

  // Get LC29 response
  if (driver->lc29_driver_read(driver_cmd_response,
                               sizeof(driver_cmd_response)) != DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }
  // Validate response
  cmd_response = lc29_driver_parse_response(driver_cmd_response, 490);
  if (cmd_response != CMD_SEND_SUCCESS) {
    return cmd_response;
  }

  // update driver config
  driver->easy_enable = enable;

  return CMD_SEND_SUCCESS;
}

/*
Saves the current configurations to the quecterl hardware file system.
Type:
Command.
Synopsis:
$PAIR513*<Checksum><CR><LF>

Result:
Returns a PAIR_ACK message.
Example:
$PAIR513*3D
$PAIR001,513,0*3C
*/
qc_lc29x_ack_reponse_t lc29_driver_nvm_save_setting(qc_lc29_driver_s *driver,
                                                    bool enable) {
  qc_lc29x_ack_reponse_t cmd_response;
  char driver_cmd_response[22] = {0};
  char payload[50] = {0};

  // char *args[] = {};
  // Build PAIR Command
  if (lc29_driver_build_pair_cmd(0, PAIR_NVM_SAVE_SETTING, NULL, payload) !=
      VALID_RESPONSE) {
    return CMD_SEND_FAIL;
  }

  // Send request
  if (driver->lc29_driver_write(payload, strlen(payload)) != DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }

  // Get LC29 response
  if (driver->lc29_driver_read(driver_cmd_response,
                               sizeof(driver_cmd_response)) != DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }
  // Validate response
  cmd_response = lc29_driver_parse_response(driver_cmd_response, 513);
  if (cmd_response != CMD_SEND_SUCCESS) {
    return cmd_response;
  }

  return CMD_SEND_SUCCESS;
}

/*
Shuts down all systems, including GNSS and CM4. When this command is sent, CM4
will be set to the RTC-Mode, in which it cannot receive any commands. CM4 can be
awoken by the timer or the RTC_EINT pin. All system resources will re-initialize
after wake up.

Type:
Set.
Synopsis:
$PAIR650,<Second>*<Checksum><CR><LF>

Parameter:
<Second> Numeric Second

Result:
Time to leave RTC-Mode. Range: 0 and 10–62208000.
Field Format
Unit Description
-If there is no error, the RTC will be set into RTC-Mode and cannot receive
any commands.
-In case of any command parameter error, the PAIR_ACK message
will be returned.

Example:
$PAIR650,2*27
$PAIR001,650,4*3C
*/
qc_lc29x_ack_reponse_t lc29_driver_set_low_power_mode(qc_lc29_driver_s *driver,
                                                      char *wakeup_time) {
  qc_lc29x_ack_reponse_t cmd_response;
  char driver_cmd_response[22] = {0};
  char payload[50] = {0};
  char *args[] = {wakeup_time};

  // Check to ensure SNR remains within bounds...
  uint32_t wakeup_time_i = (uint32_t)atoi(wakeup_time);

  if (wakeup_time_i != 0) {
    if (wakeup_time_i < 10 || wakeup_time_i > 62208000) {
      return CMD_INVALID;
    }
  }

  // Build PAIR Command
  if (lc29_driver_build_pair_cmd(1, PAIR_LOW_POWER_ENTER_RTC_MODE, args,
                                 payload) != VALID_RESPONSE) {
    return CMD_SEND_FAIL;
  }

  // Send request
  if (driver->lc29_driver_write(payload, strlen(payload)) != DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }

  // Get LC29 response
  if (driver->lc29_driver_read(driver_cmd_response,
                               sizeof(driver_cmd_response)) != DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }
  // Validate response
  cmd_response = lc29_driver_parse_response(driver_cmd_response, 650);
  if (cmd_response != CMD_SEND_SUCCESS) {
    return cmd_response;
  }

  // update driver config
  driver->low_pwr_rtc_clk = wakeup_time_i;

  return CMD_SEND_SUCCESS;
}

/*
Sets the NMEA port baud rate.

Synopsis:
$PAIR864,<Port_Type>,<Port_Index>,<Baudrate>*<Checksum><CR><LF>

Parameter:
<Port_Type>
HW Port Type 0 = UART
<Port_Index>
HW Port Index 0 = UART0
<Baudrate>
9600
19200
38400
57600
115200 (default)

Example:
$PAIR864,0,0,115200*1B
$PAIR001,864,0*31
*/
qc_lc29x_ack_reponse_t lc29_driver_set_io_baudrate(qc_lc29_driver_s *driver,
                                                   char *port_type,
                                                   char *port_index,
                                                   char *baud_rate) {

  qc_lc29x_ack_reponse_t cmd_response;
  char driver_cmd_response[22] = {0};
  char payload[50] = {0};
  char *args[] = {port_type, port_index, baud_rate};

  // Build PAIR Command
  if (lc29_driver_build_pair_cmd(3, PAIR_IO_SET_BAUDRATE, args, payload) !=
      VALID_RESPONSE) {
    return CMD_SEND_FAIL;
  }

  // Send request
  if (driver->lc29_driver_write(payload, strlen(payload)) != DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }

  // Get LC29 response
  if (driver->lc29_driver_read(driver_cmd_response,
                               sizeof(driver_cmd_response)) != DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }
  // Validate response
  cmd_response = lc29_driver_parse_response(driver_cmd_response, 864);
  if (cmd_response != CMD_SEND_SUCCESS) {
    return cmd_response;
  }

  // update driver config
  driver->baud_rate = atoi(baud_rate);

  return CMD_SEND_SUCCESS;
}

// Driver Get Methods

/*
Querys the position fix interval.
Type:
Get.
Synopsis:
$PAIR051*<Checksum><CR><LF>

Result:
Returns a PAIR_ACK message and the query result.
Query result message format:
$PAIR051,<Time>*<Checksum><CR><LF>
Parameter included in the result:

Parameter included in the result:
Field Format Unit Description
GNSS Module Series
<Time> Numeric Millisecond
Position fix interval. Range: 100–1000. Default value: 1000.

Example:
$PAIR051*3E
$PAIR001,051,0*3F
$PAIR051,1000*13

*/
qc_lc29x_ack_reponse_t lc29_driver_get_fix_rate(qc_lc29_driver_s *driver) {

  char driver_cmd_response[22] = {0};
  char cmd_payload[50] = {0};
  char query_response[50] = {0};
  int query_response_vals[10]; // To be passed into the parse query response
  qc_lc29x_ack_reponse_t cmd_response;
  char *args[] = {0};

  // // Step 1: Build PAIR Command
  if (lc29_driver_build_pair_cmd(0, PAIR_COMMON_GET_FIX_RATE, args,
                                 cmd_payload) != VALID_RESPONSE) {
    return CMD_SEND_FAIL;
  }

  // Step 2: Send query request
  if (driver->lc29_driver_write(cmd_payload, strlen(cmd_payload)) !=
      DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }

  // Step 3: Validate Command Response
  if (driver->lc29_driver_read(driver_cmd_response,
                               strlen(driver_cmd_response)) != DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }

  cmd_response = lc29_driver_parse_response(driver_cmd_response, 51);
  if (cmd_response != CMD_SEND_SUCCESS) {
    return cmd_response;
  }

  // Step 4: Validate and parse query response
  if (driver->lc29_driver_read(query_response, sizeof(query_response)) !=
      DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }

  if (lc29_driver_parse_query_response(
          query_response, PAIR_COMMON_GET_FIX_RATE, strlen(query_response),
          PAIR_QUERY_FIX_RATE_NUM_ARGS,
          query_response_vals) != CMD_SEND_SUCCESS) {

    return CMD_SEND_FAIL;
  }

  driver->fix_rate = query_response_vals[0];

  return CMD_SEND_SUCCESS;
}

/*
Queries the minimum SNR of satellites in use.
Type:
Get.
Synopsis:
$PAIR059*<Checksum><CR><LF>

Result:
Returns a PAIR_ACK message and the query result.
Query result message format:
$PAIR059,<MIN_SNR>*<Checksum><CR><LF>

Parameter included in the result:
<MIN_SNR>
db-Hz
Minimum SNR threshold of satellites in use. Range: 9–37. Default value: 9

Example:
$PAIR059*36
$PAIR001,059,0*37
$PAIR059,15*1E

*/
qc_lc29x_ack_reponse_t lc29_driver_get_min_snr(qc_lc29_driver_s *driver) {

  char driver_cmd_response[22] = {0};
  char cmd_payload[50] = {0};
  char query_response[50] = {0};
  int query_response_vals[10]; // To be passed into the parse query response
  qc_lc29x_ack_reponse_t cmd_response;
  char *args[] = {0};

  // // Step 1: Build PAIR Command
  if (lc29_driver_build_pair_cmd(0, PAIR_COMMON_GET_MIN_SNR, args,
                                 cmd_payload) != VALID_RESPONSE) {
    return CMD_SEND_FAIL;
  }

  // Step 2: Send query request
  if (driver->lc29_driver_write(cmd_payload, strlen(cmd_payload)) !=
      DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }

  // Step 3: Validate Command Response
  if (driver->lc29_driver_read(driver_cmd_response,
                               strlen(driver_cmd_response)) != DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }

  cmd_response = lc29_driver_parse_response(driver_cmd_response, 59);
  if (cmd_response != CMD_SEND_SUCCESS) {
    return cmd_response;
  }

  // Step 4: Validate and parse query response
  if (driver->lc29_driver_read(query_response, sizeof(query_response)) !=
      DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }

  if (lc29_driver_parse_query_response(
          query_response, PAIR_COMMON_GET_MIN_SNR, strlen(query_response),
          PAIR_QUERY_MIN_SNR_NUM_ARGS,
          query_response_vals) != CMD_SEND_SUCCESS) {
    return CMD_SEND_FAIL;
  }

  driver->min_snr = query_response_vals[0];

  return CMD_SEND_SUCCESS;
}

/*
2.4.43. Packet Type: 865 PAIR_IO_GET_BAUDRATE

Queries the baud rate configuration of the current NMEA UART port.

Type:
Get.

Synopsis:
$PAIR865,<Port_Type>,<Port_Index>*<Checksum><CR><LF>

Result:

Returns a PAIR_ACK and the query result.
Query result message format:
$PAIR865,<Baudrate>*<Checksum><CR><LF>
Parameter included in the result:s

Parameters:
<Port_Type>
Numeric
<Port_Index>
 - Numeric -
HW Port
0 = UART
HW Port Index 0 = UART0
Field Format
Unit Description

<Baudrate>
Baud rate:
4800
9600
19200
38400
57600
115200 (default)
921600

Example:
$PAIR865,0,0*31
$PAIR001,865,0*30
$PAIR865,115200*1A

*/
qc_lc29x_ack_reponse_t lc29_driver_get_baudrate(qc_lc29_driver_s *driver) {
  const int cmd_id = 865;

  char driver_cmd_response[22] = {0};
  char cmd_payload[50] = {0};
  char query_response[50] = {0};
  int query_response_vals[10]; // To be passed into the parse query response
  qc_lc29x_ack_reponse_t cmd_response;
  char *args[] = {"0", "0"};

  // // Step 1: Build PAIR Command
  if (lc29_driver_build_pair_cmd(0, PAIR_IO_GET_BAUDRATE, args, cmd_payload) !=
      VALID_RESPONSE) {
    return CMD_SEND_FAIL;
  }

  // Step 2: Send query request
  if (driver->lc29_driver_write(cmd_payload, strlen(cmd_payload)) !=
      DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }

  // Step 3: Validate Command Response
  if (driver->lc29_driver_read(driver_cmd_response,
                               strlen(driver_cmd_response)) != DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }

  cmd_response = lc29_driver_parse_response(driver_cmd_response, cmd_id);
  if (cmd_response != CMD_SEND_SUCCESS) {
    return cmd_response;
  }

  // Step 4: Validate and parse query response
  if (driver->lc29_driver_read(query_response, strlen(query_response)) !=
      DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }

  if (lc29_driver_parse_query_response(
          query_response, PAIR_IO_GET_BAUDRATE, strlen(query_response),
          PAIR_QUERY_BAUD_RATE_NUM_ARGS,
          query_response_vals) != CMD_SEND_SUCCESS) {

    return CMD_SEND_FAIL;
  }

  driver->baud_rate = query_response_vals[0];

  return CMD_SEND_SUCCESS;
}

/*
2.4.14. Packet Type: 063 PAIR_COMMON_GET_NMEA_OUTPUT_RATE

Queries the output interval of standard NMEA sentences of each type.
Type: Get.

Synopsis:
$PAIR063,<Type>*<Checksum><CR><LF>

<Type> Numeric -
NMEA type:
-1 = Return the output rate of every type of standard NMEA sentence.
0 = NMEA_SEN_GGA
1 = NMEA_SEN_GLL
2 = NMEA_SEN_GSA
3 = NMEA_SEN_GSV
4 = NMEA_SEN_RMC
5 = NMEA_SEN_VTG

Result:
Returns a PAIR_ACK message and the query result.

Query result message format:
$PAIR063,<Type>,<Output_Rate>*<Checksum><CR><LF>

Parameters included in the result:
<Type> Numeric -
NMEA sentence type:
0 = NMEA_SEN_GGA
1 = NMEA_SEN_GLL
2 = NMEA_SEN_GSA
3 = NMEA_SEN_GSV
4 = NMEA_SEN_RMC
5 = NMEA_SEN_VTG

<Output_Rate> Numeric -
Output interval setting:
0 = Disabled or not supported
n = Output once every n position fixes. Range of n: 1–20. Default value: 1

Example:
$PAIR063,0*23
$PAIR001,063,0*3E
$PAIR063,0,3*3C
*/

qc_lc29x_ack_reponse_t
lc29_driver_get_nmea_output_rate(qc_lc29_driver_s *driver, char *nmea_rate_id) {
  const uint8_t cmd_id = 63;
  char driver_cmd_response[22] = {0};
  char cmd_payload[50] = {0};
  char query_response[50] = {0};
  int query_response_vals[10]; // To be passed into the parse query response
  qc_lc29x_ack_reponse_t cmd_response;

  // Check to ensure output ID is valid remains within bounds...
  qc_lc29x_nmea_output_rate_id_t rate_id =
      (qc_lc29x_nmea_output_rate_id_t)atoi(nmea_rate_id);
  if (rate_id > 5 || rate_id < 0) {
    return CMD_INVALID;
  }

  char *args[] = {nmea_rate_id};

  // // Step 1: Build PAIR Command
  if (lc29_driver_build_pair_cmd(0, PAIR_COMMON_GET_NMEA_OUTPUT_RATE, args,
                                 cmd_payload) != VALID_RESPONSE) {
    return CMD_SEND_FAIL;
  }

  // Step 2: Send query request
  if (driver->lc29_driver_write(cmd_payload, strlen(cmd_payload)) !=
      DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }

  // Step 3: Validate Command Response
  if (driver->lc29_driver_read(driver_cmd_response,
                               strlen(driver_cmd_response)) != DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }

  cmd_response = lc29_driver_parse_response(driver_cmd_response, cmd_id);
  if (cmd_response != CMD_SEND_SUCCESS) {
    return cmd_response;
  }

  // Step 4: Validate and parse query response
  if (driver->lc29_driver_read(query_response, strlen(query_response)) !=
      DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }

  if (lc29_driver_parse_query_response(
          query_response, PAIR_COMMON_GET_NMEA_OUTPUT_RATE,
          strlen(query_response), PAIR_QUERY_NMEA_RATE_NUM_ARGS,
          query_response_vals) != CMD_SEND_SUCCESS) {

    return CMD_SEND_FAIL;
  }

  rate_id = (qc_lc29x_nmea_output_rate_id_t)query_response_vals[0];

  uint16_t output_rate_i = query_response_vals[1];

  // update driver config
  switch (rate_id) {
  case NMEA_SEN_GGA:
    driver->nmea_output_rate.gga.output_rate = output_rate_i;
    break;
  case NMEA_SEN_GLL:
    driver->nmea_output_rate.gll.output_rate = output_rate_i;
    break;
  case NMEA_SEN_GSA:
    driver->nmea_output_rate.gsa.output_rate = output_rate_i;
    break;
  case NMEA_SEN_GSV:
    driver->nmea_output_rate.gsv.output_rate = output_rate_i;
    break;
  case NMEA_SEN_RMC:
    driver->nmea_output_rate.rmc.output_rate = output_rate_i;
    break;
  case NMEA_SEN_VTG:
    driver->nmea_output_rate.vtg.output_rate = output_rate_i;
    break;
  default:
    break;
  }

  return CMD_SEND_SUCCESS;
}

/*
Queries the GNSS search mode.
Type: Get.
Synopsis: $PAIR067*<Checksum><CR><LF>
Parameter: None


Result: Returns a PAIR_ACK message and the query result.

Query result message format:
$PAIR067<GPS_Enabled>,<GLONASS_Enabled>,<Galileo_Enabled>,<BeiDou_Enabled>,<QZSS_Ena
bled>,0*<Checksum><CR><LF>

Response Parameters:
<GPS_Enabled> <GLONASS_Enabled> <Galileo_Enabled> <BeiDou_Enabled>
<QZSS_Enabled> 0 = Disabled 1 = Search for GPS Satellites

Example:
$PAIR067*3B
$PAIR001,067,0*3A
$PAIR067,1,0,0,0,0,0*3A

*/
qc_lc29x_ack_reponse_t
lc29_driver_get_gnss_search_mode(qc_lc29_driver_s *driver) {
  const uint8_t cmd_id = 67;
  char driver_cmd_response[22] = {0};
  char cmd_payload[50] = {0};
  char query_response[50] = {0};
  int query_response_vals[10]; // To be passed into the parse query response
  qc_lc29x_ack_reponse_t cmd_response;
  char *args[] = {0};

  // // Step 1: Build PAIR Command
  if (lc29_driver_build_pair_cmd(0, PAIR_COMMON_GET_GNSS_SEARCH_MODE, args,
                                 cmd_payload) != VALID_RESPONSE) {
    return CMD_SEND_FAIL;
  }

  // Step 2: Send query request
  if (driver->lc29_driver_write(cmd_payload, strlen(cmd_payload)) !=
      DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }

  // Step 3: Validate Command Response
  if (driver->lc29_driver_read(driver_cmd_response,
                               strlen(driver_cmd_response)) != DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }

  cmd_response = lc29_driver_parse_response(driver_cmd_response, cmd_id);
  if (cmd_response != CMD_SEND_SUCCESS) {
    return cmd_response;
  }

  // Step 4: Validate and parse query response
  if (driver->lc29_driver_read(query_response, sizeof(query_response)) !=
      DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }

  if (lc29_driver_parse_query_response(
          query_response, PAIR_COMMON_GET_GNSS_SEARCH_MODE,
          strlen(query_response), PAIR_QUERY_MIN_SNR_NUM_ARGS,
          query_response_vals) != CMD_SEND_SUCCESS) {
    return CMD_SEND_FAIL;
  }

  // update driver config
  driver->gnss_search_mode.gps_enabled = query_response_vals[0];
  driver->gnss_search_mode.glonass_enabled = query_response_vals[1];
  driver->gnss_search_mode.galileo_enabled = query_response_vals[2];
  driver->gnss_search_mode.beidou_enabled = query_response_vals[3];
  driver->gnss_search_mode.qzss_enabled = query_response_vals[4];
  driver->gnss_search_mode.reserved = false;

  return CMD_SEND_SUCCESS;
}

/*
Queries the static navigation speed threshold.
Type: Get.

Synopsis: $PAIR071*<Checksum><CR><LF>
Parameter: None
Result: Returns a PAIR_ACK message and the query result.

Query result message format:
$PAIR071,<Speed_threshold>*<Checksum><CR><LF>

<Speed_threshold> Numeric dm/s
Speed threshold.
Range: 0–20. Default value: 0

Example:
$PAIR071*3C
$PAIR001,071,0*3D
$PAIR071,0*20
*/
qc_lc29x_ack_reponse_t
lc29_driver_get_static_threshold(qc_lc29_driver_s *driver) {
  const uint8_t cmd_id = 71;
  char driver_cmd_response[22] = {0};
  char cmd_payload[50] = {0};
  char query_response[50] = {0};
  int query_response_vals[10]; // To be passed into the parse query response
  qc_lc29x_ack_reponse_t cmd_response;
  char *args[] = {0};

  // // Step 1: Build PAIR Command
  if (lc29_driver_build_pair_cmd(0, PAIR_COMMON_GET_STATIC_THRESHOLD, args,
                                 cmd_payload) != VALID_RESPONSE) {
    return CMD_SEND_FAIL;
  }

  // Step 2: Send query request
  if (driver->lc29_driver_write(cmd_payload, strlen(cmd_payload)) !=
      DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }

  // Step 3: Validate Command Response
  if (driver->lc29_driver_read(driver_cmd_response,
                               strlen(driver_cmd_response)) != DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }

  cmd_response = lc29_driver_parse_response(driver_cmd_response, cmd_id);
  if (cmd_response != CMD_SEND_SUCCESS) {
    return cmd_response;
  }

  // Step 4: Validate and parse query response
  if (driver->lc29_driver_read(query_response, sizeof(query_response)) !=
      DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }

  if (lc29_driver_parse_query_response(
          query_response, PAIR_COMMON_GET_STATIC_THRESHOLD,
          strlen(query_response), PAIR_QUERY_STATIC_THRESHOLD_ARGS,
          query_response_vals) != CMD_SEND_SUCCESS) {
    return CMD_SEND_FAIL;
  }

  // update driver config
  driver->static_spd_thrshld = query_response_vals[0];

  return CMD_SEND_SUCCESS;
}

/*
2.4.23. Packet Type: 081 PAIR_COMMON_GET_NAVIGATION_MODE
Queries navigation mode.
Type: Get.
Synopsis: $PAIR081*<Checksum><CR><LF>

Parameter: None
Result: Returns a PAIR_ACK message and the query result.

Query result message format: $PAIR081,<CmdType>*<Checksum><CR><LF>

Parameter included in the result:
0 = Normal Mode. For general purposes.
1 = Fitness Mode: For running and walking purposes so that the low-speed (< 5
m/s) movement will have more effect on position calculation.

2 = Reserved
3 = Reserved
4 = Stationary Mode: For stationary applications where zero
dynamics is assumed.
5 = Reserved
6 = Reserved
7 = Swimming Mode: For swimming
purpose so that it smooths the trajectory and improves the accuracy of distance
calculation.

Example:
$PAIR081*33
$PAIR001,081,0*32
$PAIR081,0*2F

*/
qc_lc29x_ack_reponse_t
lc29_driver_get_navigation_mode(qc_lc29_driver_s *driver) {
  const uint8_t cmd_id = 81;
  char driver_cmd_response[22] = {0};
  char cmd_payload[50] = {0};
  char query_response[50] = {0};
  int query_response_vals[10]; // To be passed into the parse query response
  qc_lc29x_ack_reponse_t cmd_response;
  char *args[] = {0};

  // // Step 1: Build PAIR Command
  if (lc29_driver_build_pair_cmd(0, PAIR_COMMON_GET_NAVIGATION_MODE, args,
                                 cmd_payload) != VALID_RESPONSE) {
    return CMD_SEND_FAIL;
  }

  // Step 2: Send query request
  if (driver->lc29_driver_write(cmd_payload, strlen(cmd_payload)) !=
      DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }

  // Step 3: Validate Command Response
  if (driver->lc29_driver_read(driver_cmd_response,
                               strlen(driver_cmd_response)) != DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }

  cmd_response = lc29_driver_parse_response(driver_cmd_response, cmd_id);
  if (cmd_response != CMD_SEND_SUCCESS) {
    return cmd_response;
  }

  // Step 4: Validate and parse query response
  if (driver->lc29_driver_read(query_response, sizeof(query_response)) !=
      DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }

  if (lc29_driver_parse_query_response(
          query_response, PAIR_COMMON_GET_NAVIGATION_MODE,
          strlen(query_response), PAIR_QUERY_STATIC_THRESHOLD_ARGS,
          query_response_vals) != CMD_SEND_SUCCESS) {
    return CMD_SEND_FAIL;
  }

  // update driver config
  driver->nav_mode = query_response_vals[0];

  return CMD_SEND_SUCCESS;
}

/*
2.4.25. Packet Type: 099 PAIR_COMMON_GET_NMEA_POS_DECIMAL_PRECISION
Queries the precision of coordinates.
Type: Get.
Synopsis: $PAIR099*<Checksum><CR><LF>
Parameter: None
Result: Returns a PAIR_ACK message and the query result.
GNSS Module Series


Query result message format:
$PAIR099,<Mode>*<Checksum><CR><LF>

<Mode>
Example:
Numeric -
0 = Latitude, Longitude: 4; Altitude: 1.
1 = Latitude, Longitude: 5; Altitude: 2.
2 = Latitude, Longitude: 6; Altitude: 3 (default)
3 = Latitude, Longitude: 7; Altitude: 3.

Example:
$PAIR099*3A
$PAIR001,099,0*3B
$PAIR099,3*25
*/
qc_lc29x_ack_reponse_t
lc29_driver_get_NMEA_decimal_precision(qc_lc29_driver_s *driver) {
  const uint8_t cmd_id = 99;
  char driver_cmd_response[22] = {0};
  char cmd_payload[50] = {0};
  char query_response[50] = {0};
  int query_response_vals[10]; // To be passed into the parse query response
  qc_lc29x_ack_reponse_t cmd_response;
  char *args[] = {0};

  // // Step 1: Build PAIR Command
  if (lc29_driver_build_pair_cmd(0, PAIR_COMMON_GET_NMEA_POS_DECIMAL_PRECISION,
                                 args, cmd_payload) != VALID_RESPONSE) {
    return CMD_SEND_FAIL;
  }

  // Step 2: Send query request
  if (driver->lc29_driver_write(cmd_payload, strlen(cmd_payload)) !=
      DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }

  // Step 3: Validate Command Response
  if (driver->lc29_driver_read(driver_cmd_response,
                               strlen(driver_cmd_response)) != DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }

  cmd_response = lc29_driver_parse_response(driver_cmd_response, cmd_id);
  if (cmd_response != CMD_SEND_SUCCESS) {
    return cmd_response;
  }

  // Step 4: Validate and parse query response
  if (driver->lc29_driver_read(query_response, sizeof(query_response)) !=
      DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }

  if (lc29_driver_parse_query_response(
          query_response, PAIR_COMMON_GET_NMEA_POS_DECIMAL_PRECISION,
          strlen(query_response), PAIR_QUERY_NMEA_DECIMAL_PRECISION_ARGS,
          query_response_vals) != CMD_SEND_SUCCESS) {
    return CMD_SEND_FAIL;
  }

  // update driver config
  driver->nav_mode = query_response_vals[0];

  return CMD_SEND_SUCCESS;
}

// TODO (@Kibby) - Not a high priority at the moment to support
qc_lc29x_ack_reponse_t
lc29_driver_get_nmea_output_mode(qc_lc29_driver_s *driver) {
  return 0;
}

/*
2.4.29. Packet Type: 105 PAIR_COMMON_GET_DUAL_BAND
Queries whether Dual Band is enabled or disabled.
Type: Get.
Synopsis: $PAIR105*<Checksum><CR><LF>

Result: Returns a PAIR_ACK message and the query result.

Query result message format:
$PAIR105,<Enable>*<Checksum><CR><LF>

<Enable>
0 = Disabled
1 = Enabled (default)
3 = Critical status

Example:
$PAIR105*3E
$PAIR001,105,0*3F
$PAIR105,1*23

*/
qc_lc29x_ack_reponse_t
lc29_driver_get_dual_band_mode(qc_lc29_driver_s *driver) {
  const uint8_t cmd_id = 105;
  char driver_cmd_response[22] = {0};
  char cmd_payload[50] = {0};
  char query_response[50] = {0};
  int query_response_vals[10]; // To be passed into the parse query response
  qc_lc29x_ack_reponse_t cmd_response;
  char *args[] = {0};

  // // Step 1: Build PAIR Command
  if (lc29_driver_build_pair_cmd(0, PAIR_COMMON_GET_DUAL_BAND, args,
                                 cmd_payload) != VALID_RESPONSE) {
    return CMD_SEND_FAIL;
  }

  // Step 2: Send query request
  if (driver->lc29_driver_write(cmd_payload, strlen(cmd_payload)) !=
      DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }

  // Step 3: Validate Command Response
  if (driver->lc29_driver_read(driver_cmd_response,
                               strlen(driver_cmd_response)) != DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }

  cmd_response = lc29_driver_parse_response(driver_cmd_response, cmd_id);
  if (cmd_response != CMD_SEND_SUCCESS) {
    return cmd_response;
  }

  // Step 4: Validate and parse query response
  if (driver->lc29_driver_read(query_response, sizeof(query_response)) !=
      DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }

  if (lc29_driver_parse_query_response(
          query_response, PAIR_COMMON_GET_DUAL_BAND, strlen(query_response),
          PAIR_QUERY_DUAL_BAND_MODE_ARGS,
          query_response_vals) != CMD_SEND_SUCCESS) {
    return CMD_SEND_FAIL;
  }

  // update driver config
  driver->dual_band_enable = query_response_vals[0];

  return CMD_SEND_SUCCESS;
}

/*
2.4.34. Packet Type: 411 PAIR_SBAS_GET_STATUS

Queries if the searching of SBAS satellites is enabled or not.
Type: Get.
Synopsis:  $PAIR411*<Checksum><CR><LF>


Result: Returns a PAIR_ACK message and the query result.
Query result message format:
$PAIR411,<Enabled>*<Checksum><CR><LF>

<Enabled>:
0 = Disabled
1 = Enabled (default)

Example:
$PAIR411*3E
$PAIR001,411,0*3F
$PAIR411,2*21
*/
qc_lc29x_ack_reponse_t lc29_driver_get_sbas_mode(qc_lc29_driver_s *driver) {
  const int cmd_id = 411;
  char driver_cmd_response[22] = {0};
  char cmd_payload[50] = {0};
  char query_response[50] = {0};
  int query_response_vals[10]; // To be passed into the parse query response
  qc_lc29x_ack_reponse_t cmd_response;
  char *args[] = {0};

  // // Step 1: Build PAIR Command
  if (lc29_driver_build_pair_cmd(0, PAIR_SBAS_GET_STATUS, args, cmd_payload) !=
      VALID_RESPONSE) {
    return CMD_SEND_FAIL;
  }

  // Step 2: Send query request
  if (driver->lc29_driver_write(cmd_payload, strlen(cmd_payload)) !=
      DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }

  // Step 3: Validate Command Response
  if (driver->lc29_driver_read(driver_cmd_response,
                               strlen(driver_cmd_response)) != DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }

  cmd_response = lc29_driver_parse_response(driver_cmd_response, cmd_id);
  if (cmd_response != CMD_SEND_SUCCESS) {
    return cmd_response;
  }

  // Step 4: Validate and parse query response
  if (driver->lc29_driver_read(query_response, sizeof(query_response)) !=
      DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }

  if (lc29_driver_parse_query_response(
          query_response, PAIR_SBAS_GET_STATUS, strlen(query_response),
          PAIR_QUERY_SBAS_STATUS_ARGS,
          query_response_vals) != CMD_SEND_SUCCESS) {
    return CMD_SEND_FAIL;
  }

  // update driver config
  driver->sbas_enable = query_response_vals[0];

  return CMD_SEND_SUCCESS;
}

/*
2.4.36. Packet Type: 491 PAIR_EASY_GET_STATUS
Queries the status of EASYTM.

Type: Get.
Synopsis: $PAIR491*<Checksum><CR><LF>

Result: Returns a PAIR_ACK message and the query result.

Query result message format:
$PAIR491,<Enable>,<Status>*<Checksum><CR><LF>

<Enable>
Enabled or disabled:
0 = Disabled
1 = Enabled (default)

<Status>
0 = Not finished
1 = Finished 1-day extension
2 = Finished 2-day extension
3 = Finished 3-day extension

Example:
$PAIR491*36
$PAIR001,491,0*37
$PAIR491,1,0*37

NOTE:
If EASYTM function is not enabled, only the <Enable> value will be returned
after executing this command.

*/
qc_lc29x_ack_reponse_t lc29_driver_get_easy_satus(qc_lc29_driver_s *driver) {
  const int cmd_id = 491;
  char driver_cmd_response[22] = {0};
  char cmd_payload[50] = {0};
  char query_response[50] = {0};
  int query_response_vals[10]; // To be passed into the parse query response
  qc_lc29x_ack_reponse_t cmd_response;
  char *args[] = {0};

  // // Step 1: Build PAIR Command
  if (lc29_driver_build_pair_cmd(0, PAIR_EASY_GET_STATUS, args, cmd_payload) !=
      VALID_RESPONSE) {
    return CMD_SEND_FAIL;
  }

  // Step 2: Send query request
  if (driver->lc29_driver_write(cmd_payload, strlen(cmd_payload)) !=
      DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }

  // Step 3: Validate Command Response
  if (driver->lc29_driver_read(driver_cmd_response,
                               strlen(driver_cmd_response)) != DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }

  cmd_response = lc29_driver_parse_response(driver_cmd_response, cmd_id);
  if (cmd_response != CMD_SEND_SUCCESS) {
    return cmd_response;
  }

  // Step 4: Validate and parse query response
  if (driver->lc29_driver_read(query_response, sizeof(query_response)) !=
      DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }

  if (lc29_driver_parse_query_response(
          query_response, PAIR_EASY_GET_STATUS, strlen(query_response),
          PAIR_QUERY_EASY_STATUS_ARGS,
          query_response_vals) != CMD_SEND_SUCCESS) {
    return CMD_SEND_FAIL;
  }

  // update driver config
  driver->easy_status = query_response_vals[0];

  return CMD_SEND_SUCCESS;
}

/*
2.4.38. Packet Type: 650 PAIR_LOW_POEWR_ENTER_RTC_MODE

Shuts down all systems, including GNSS and CM4. When this command is sent, CM4
will be set to the RTC-Mode, in which it cannot receive any commands. CM4 can be
awoken by the timer or the RTC_EINT pin. All system resources will re-initialize
after wake up.


*** NOTE: CM4 can be awoken by the timer or the RTC_EINT pin.***

TODO (@CyKibb) - RTC_EINT gpio pin support needed with interface method
*/
qc_lc29x_ack_reponse_t
lc29_driver_get_low_power_mode(qc_lc29_driver_s *driver) {
  // TODO (@Cykibb) - Build support for this module
  return 0;
}

/******************* LC29H (BA,CA,DA,EA) DR&RTK Operations *******************/

/*
Notes related to the DR&RTK System:

3.1. PQTM Messages
This chapter outlines the Quectel DR related PQTM (proprietary NMEA) messages
supported by the Quectel LC29H (BA) and LC29H(CA) modules.

Definitions:
INS = Inertial Navigation System
IMU = Inertial Measurement Unit
GPS = Global Positioning System


3.1.1. PQTMDRCAL
Indicates the DR calibration state.
Type: Output
Synopsis: $PQTMDRCAL,<MsgVer>,<CalState>,<NavType>*<Checksum><CR><LF>

<MsgVer>
Message version.
1 = Version 1 (Always 1 for this version.)

<CalState>
DR calibration state.
0 = Not calibrated
1 = DR is lightly calibrated
2 = DR is fully calibrated

<NavType>
Navigation type.
0 = No position
1 = GNSS only
2 = DR only
3 = Combination (GNSS + DR)
*/

// NOTE (@Kibby): This method is rebuilt as the original parse_cmd_response only
// supports $PAIR001 cmds...
qc_lc29x_ack_reponse_t
lc29_driver_parse_dr_cmd_response(char *response_string,
                                  int response_string_len, char *dr_cmd_id,
                                  int dr_cmd_id_len, int check_checksum) {

  int response_cmd_id, response_cmd_status;

  if (9 > response_string_len) {
    return CMD_SEND_FAIL;
  }

  /* should start with $ */
  if ('$' != *response_string) {
    return CMD_SEND_FAIL;
  }
  /* should end with \r\n, or other... */
  if (strcmp(&response_string[response_string_len - 4], "\r\n") == 0) {
    return CMD_SEND_FAIL;
  }

  // Verify that the input string starts with input CMD ID
  if (strncmp(response_string, dr_cmd_id, dr_cmd_id_len) != 0) {
    return CMD_SEND_FAIL; // Invalid input string format
  }

  // received string integrity check...
  if (0 !=
      lc29_driver_response_has_checksum(response_string, response_string_len)) {
    return CMD_SEND_FAIL;
  }

  if (1 == check_checksum) {
    uint8_t actual_chk;
    uint8_t expected_chk;
    char checksum[3];

    checksum[0] = response_string[response_string_len - 4];
    checksum[1] = response_string[response_string_len - 3];
    checksum[2] = '\0';
    actual_chk = lc29_driver_get_checksum(response_string);
    expected_chk = (uint8_t)strtol(checksum, NULL, 16);

    if (expected_chk != actual_chk) {
      return CMD_SEND_FAIL;
    }
  }

  return CMD_SEND_SUCCESS;
}

/*
3.1.9. PQTMCFGEINSMSG
Sets/gets $PQTMINS, $PQTMIMU and $PQTMGPS message configurations.

Type: Set/get

Synopsis:
Set message configurations:
$PQTMCFGEINSMSG,<Type>,<INS_Enabled>,<IMU_Enabled>,<GPS_Enabled>,<Rate>*<Checksum>
<CR><LF>

Get message configurations:
$PQTMCFGEINSMSG,<Type>*<Checksum><CR><LF>

<Type>
Set/get message configurations.
0=Get
1 = Set

<INS_Enabled>
Enable/disable the output of $PQTMINS message.
0 = Disable
1 = Enable

<IMU_Enabled>
Enable/disable the output of $PQTMIMU message.
0 = Disable
1 = Enable

<GPS_Enabled>
Enable/disable the output of $PQTMGPS message.
0 = Disable
1 = Enable

<Rate> (Unit = Hz)
Set the output rate of $PQTMINS, $PQTMIMU or
$PQTMGPS message.
It can be set to 1, 2, 4, 5, 10, 20, 50, or 100.
For $PQTMGPS, the output rate is fixed at 1 Hz.
For $PQTMINS, when <Rate> is set to be greater than 10, the
message will be output at 10 Hz.


Result:
- If successful, the module returns:
- If failed, the module returns: $PQTMCFGEINSMSGERROR*4A

Example:
GNSS Module Series
Set:
$PQTMCFGEINSMSGOK*16
Get:
$PQTMCFGEINSMSG,<Type>,<INS_Enabled>,<IMU_Enabled>,<GPS_Enabled>,<Rate>*<Checksum>
<CR><LF>

If failed, the module returns: $PQTMCFGEINSMSGERROR*4A

Example:
//Set message configurations:
$PQTMCFGEINSMSG,1,1,1,1,10*3F
$PQTMCFGEINSMSGOK*16
//Get message configurations:
$PQTMCFGEINSMSG,0*0E
$PQTMEINSMSG,0,1,1,1,10*3E

NOTE:
1. Send $PQTMSAVEPAR*5A and reset the module for $PQTMCFGEINSMSG to take effect.
2. This command is only supported by LC29H (BA) and LC29H (CA) with software
versions dedicated for two-wheel vehicles. Contact Quectel Technical Support
(support@quectel.com) for details about the software versions.
3. type: "1:true" set or "0:false" get
*/

qc_lc29x_ack_reponse_t
lc29_driver_set_get_pqtm_message_settings(qc_lc29_driver_s *driver, bool type,
                                          bool ins_enabled, bool imu_enabled,
                                          bool gps_enabled, char *rate) {
  // const uint8_t cmd_id = 491;
  char driver_cmd_response[22] = {0};
  char cmd_payload[50] = {0};
  char query_response[50] = {0};
  int query_response_vals[10]; // To be passed into the parse query response
  qc_lc29x_ack_reponse_t cmd_response;
  char *args[] = {type ? "1" : "0", ins_enabled ? "1" : "0",
                  imu_enabled ? "1" : "0", gps_enabled ? "1" : "0", rate};

  // Step 1: Build PAIR Command
  if (lc29_driver_build_pair_cmd(5, LC29_DR_PQTM_MESSAGE_CONFIG_HEADER, args,
                                 cmd_payload) != VALID_RESPONSE) {
    return CMD_SEND_FAIL;
  }

  // Step 2: Send query request
  if (driver->lc29_driver_write(cmd_payload, strlen(cmd_payload)) !=
      DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }

  // Step 3: Validate Command Response
  if (driver->lc29_driver_read(driver_cmd_response,
                               strlen(driver_cmd_response)) != DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }

  // The expected command response here is $PQTMCFGEINSMSGOK*16... This
  if (lc29_driver_parse_dr_cmd_response(
          driver_cmd_response, strlen(driver_cmd_response), LC29_DR_RESPONSE_OK,
          strlen(LC29_DR_RESPONSE_OK), 1) != CMD_SEND_SUCCESS) {
    return CMD_SEND_FAIL;
  }

  if (!type) {
    // Step 4: Validate and parse query response,
    if (driver->lc29_driver_read(query_response, sizeof(query_response)) !=
        DRIVER_SUCCESS) {
      return CMD_SEND_FAIL;
    }
    // TODO: Update these parameters to support the proper expected response
    if (lc29_driver_parse_query_response(
            query_response, LC29_DR_PQTM_MESSAGE_CONFIG_RESPONSE_HEADER,
            strlen(query_response), LC29_DR_QUERY_PQTM_CONFIG_RESPONSE_ARGS,
            query_response_vals) != CMD_SEND_SUCCESS) {
      return CMD_SEND_FAIL;
    }

    // update driver config
    driver->dr_rtk_output_rate.ins.enabled = query_response_vals[1];
    driver->dr_rtk_output_rate.ins.fix_rate = query_response_vals[4];
    driver->dr_rtk_output_rate.imu.enabled = query_response_vals[2];
    driver->dr_rtk_output_rate.imu.fix_rate = query_response_vals[4];
    driver->dr_rtk_output_rate.gps.enabled = query_response_vals[3];
    driver->dr_rtk_output_rate.gps.enabled =
        query_response_vals[4] > 10 ? 10 : query_response_vals[4];
  }

  // update driver config
  driver->dr_rtk_output_rate.ins.enabled = ins_enabled;
  driver->dr_rtk_output_rate.ins.fix_rate = atoi(rate);
  driver->dr_rtk_output_rate.imu.enabled = imu_enabled;
  driver->dr_rtk_output_rate.imu.fix_rate = atoi(rate);
  driver->dr_rtk_output_rate.gps.enabled = gps_enabled;
  driver->dr_rtk_output_rate.gps.fix_rate = atoi(rate) > 10 ? 10 : atoi(rate);

  return CMD_SEND_SUCCESS;
}

/*
3.2.1. Packet Type: 6010 PAIR_CUSTOM_SET_MSG_OUTPUT

Enables/disables the output of $PQTMVEHMSG, $PQTMSENMSG, $PQTMDRCAL,
$PQTMIMUTYPE and $PQTMVEHMOT messages.

Type: Set

Synopsis: $PAIR6010,<Type>,<Output_State>*<Checksum><CR><LF>

<Type>
-1 = Reset output state of all following sentence types to the default value
0 = $PQTMVEHMSG (Default: disabled)
1 = $PQTMSENMSG (Default: disabled)
2 = $PQTMDRCAL (Default: disabled)
3 = $PQTMIMUTYPE (Default: enabled)
4 = $PQTMVEHMOT (Default: disabled)

<Output_State>
Message output state.
0 = Disabled
1 = Enabled

Example:
$PAIR6010,0,1*0C
$PAIR001,6010,0*0C


1. Send $PQTMSAVEPAR*5A and reset the module for $PAIR6010 to take effect.
2. The output rate of $PQTMVEHMSG and $PQTMSENMSG is always 10 Hz. The output
rate of $PQTMDRCAL and $PQTMVEHMOT depends on position fix rate. $PQTMIMUTYPE is
only output once after each boot-up.
*/

qc_lc29x_ack_reponse_t
lc29_driver_set_dr_rtk_message_output(qc_lc29_driver_s *driver, char *msg_type,
                                      bool msg_type_output_state) {
  const int cmd_id = 6010;
  char driver_cmd_response[22] = {0};
  char cmd_payload[50] = {0};
  char query_response[50] = {0};
  int query_response_vals[10]; // To be passed into the parse query response
  qc_lc29x_ack_reponse_t cmd_response;
  char *args[] = {msg_type, msg_type_output_state ? "1" : "0"};

  // // Step 1: Build PAIR Command
  if (lc29_driver_build_pair_cmd(2, PAIR_SET_CUSTOM_MSG_OUTPUT, args,
                                 cmd_payload) != VALID_RESPONSE) {
    return CMD_SEND_FAIL;
  }

  // Step 2: Send query request
  if (driver->lc29_driver_write(cmd_payload, strlen(cmd_payload)) !=
      DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }

  // Step 3: Validate Command Response
  if (driver->lc29_driver_read(driver_cmd_response,
                               strlen(driver_cmd_response)) != DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }

  cmd_response = lc29_driver_parse_response(driver_cmd_response, cmd_id);
  if (cmd_response != CMD_SEND_SUCCESS) {
    return cmd_response;
  }

  qc_lc29x_dr_custom_message_id_t msg_id =
      (qc_lc29x_dr_custom_message_id_t)atoi(msg_type);

  switch (msg_id) {
  case PQTMVEHMSG:
    driver->dr_rtk_custom_message_settings.vehicle_info.enabled =
        msg_type_output_state;
    break;
  case PQTMSENMSG:
    driver->dr_rtk_custom_message_settings.sensor_output.enabled =
        msg_type_output_state;
    break;
  case PQTMDRCAL:
    driver->dr_rtk_custom_message_settings.dr_calibration.enabled =
        msg_type_output_state;
    break;
  case PQTMIMUTYPE:
    driver->dr_rtk_custom_message_settings.imu_type.enabled =
        msg_type_output_state;
    break;
  case PQTMVEHMOT:
    driver->dr_rtk_custom_message_settings.dr_vehicle_motion.enabled =
        msg_type_output_state;
    break;
  default:
    return CMD_SEND_FAIL;
    break;
  }

  return CMD_SEND_SUCCESS;
}

/*
3.2.2. Packet Type: 6011 PAIR_CUSTOM_GET_MSG_OUTPUT
Gets whether the output of $PQTMVEHMSG, $PQTMSENMSG, $PQTMDRCAL, $PQTMIMUTYPE
and $PQTMVEHMOT messages is enabled.

Type: Get
Synopsis: $PAIR6011,<Type>*<Checksum><CR><LF>

<Type>
Message type.
0 = $PQTMVEHMSG
1 = $PQTMSENMSG
2 = $PQTMDRCAL
3 = $PQTMIMUTYPE
4 = $PQTMVEHMOT

Result:
Returns $PAIR001 message and query result. See document [2] protocol
specification for details.

Query result message format:
$PAIR6011,<Type>,<Output_State>*<Checksum><CR><LF>

<Type>
Message type.
0 = $PQTMVEHMSG
1 = $PQTMSENMSG
2 = $PQTMDRCAL
3 = $PQTMIMUTYPE
4 = $PQTMVEHMOT

<Output_State>
Message output state.
0 = Disabled
1 = Enabled

Example:
$PAIR6011,1*11
$PAIR001,6011,0*0D
$PAIR6011,1,0*0D

Note:
This command is only supported by LC29H (BA) and LC29H (CA) with software
versions dedicated for four-wheel vehicles. Contact Quectel Technical Support
(support@quectel.com) for details about the software versions.

*/
qc_lc29x_ack_reponse_t
lc29_driver_get_dr_rtk_message_output(qc_lc29_driver_s *driver,
                                      char *msg_type) {
  const int cmd_id = 6011;
  char driver_cmd_response[22] = {0};
  char cmd_payload[50] = {0};
  char query_response[50] = {0};
  int query_response_vals[10]; // To be passed into the parse query response
  qc_lc29x_ack_reponse_t cmd_response;
  char *args[] = {msg_type};

  // // Step 1: Build PAIR Command
  if (lc29_driver_build_pair_cmd(1, PAIR_GET_CUSTOM_MSG_OUTPUT, args,
                                 cmd_payload) != VALID_RESPONSE) {
    return CMD_SEND_FAIL;
  }

  // Step 2: Send query request
  if (driver->lc29_driver_write(cmd_payload, strlen(cmd_payload)) !=
      DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }

  // Step 3: Validate Command Response
  if (driver->lc29_driver_read(driver_cmd_response,
                               strlen(driver_cmd_response)) != DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }

  cmd_response = lc29_driver_parse_response(driver_cmd_response, cmd_id);
  if (cmd_response != CMD_SEND_SUCCESS) {
    return cmd_response;
  }

  // Step 4: Validate and parse query response
  if (driver->lc29_driver_read(query_response, sizeof(query_response)) !=
      DRIVER_SUCCESS) {
    return CMD_SEND_FAIL;
  }

  if (lc29_driver_parse_query_response(
          query_response, PAIR_GET_CUSTOM_MSG_OUTPUT, strlen(query_response),
          PAIR_QUERY_CUSTOM_MSG_OUTPUT,
          query_response_vals) != CMD_SEND_SUCCESS) {
    return CMD_SEND_FAIL;
  }

  qc_lc29x_dr_custom_message_id_t msg_id =
      (qc_lc29x_dr_custom_message_id_t)query_response_vals[0];

  switch (msg_id) {
  case PQTMVEHMSG:
    driver->dr_rtk_custom_message_settings.vehicle_info.enabled =
        query_response_vals[1];
    break;
  case PQTMSENMSG:
    driver->dr_rtk_custom_message_settings.sensor_output.enabled =
        query_response_vals[1];
    break;
  case PQTMDRCAL:
    driver->dr_rtk_custom_message_settings.dr_calibration.enabled =
        query_response_vals[1];
    break;
  case PQTMIMUTYPE:
    driver->dr_rtk_custom_message_settings.imu_type.enabled =
        query_response_vals[1];
    break;
  case PQTMVEHMOT:
    driver->dr_rtk_custom_message_settings.dr_vehicle_motion.enabled =
        query_response_vals[1];
    break;
  default:
    return CMD_SEND_FAIL;
    break;
  }
  return CMD_SEND_SUCCESS;
}
