#ifndef QC_LC29_DRIVER_H_INCLUDED
#define QC_LC29_DRIVER_H_INCLUDED

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#define LC29_RESPONSE_FIXED_LENGTH 19
#define LC29_RESPONSE_END_CHAR '\r'
#define LC29_CMD_END_CHARS "\r\n"

/* LC29 COMMON GNSS PAIR COMMAND HEADERS */
#define PAIR_CMD_PREFIX "$PAIR"

#define PAIR_ACK "$PAIR001"
#define PAIR_GNSS_SUBSYS_POWER_ON "$PAIR002"
#define PAIR_GNSS_SUBSYS_POWER_OFF "$PAIR003"
#define PAIR_GNSS_SUBSYS_HOT_START "$PAIR004"
#define PAIR_GNSS_SUBSYS_WARM_START "$PAIR005"
#define PAIR_GNSS_SUBSYS_COLD_START "$PAIR006"
#define PAIR_GNSS_SUBSYS_FULL_COLD_START "$PAIR007"
#define PAIR_REQUEST_AIDING "$PAIR010"
#define PAIR_COMMON_SET_FIX_RATE "$PAIR050"
#define PAIR_COMMON_GET_FIX_RATE "$PAIR051"
#define PAIR_COMMON_SET_MIN_SNR "$PAIR058"
#define PAIR_COMMON_GET_MIN_SNR "$PAIR059"
#define PAIR_COMMON_SET_NMEA_OUTPUT_RATE "$PAIR062"
#define PAIR_COMMON_GET_NMEA_OUTPUT_RATE "$PAIR063"
#define PAIR_COMMON_SET_GNSS_SEARCH_MODE "$PAIR066"
#define PAIR_COMMON_GET_GNSS_SEARCH_MODE "$PAIR067"
#define PAIR_COMMON_SET_STATIC_THRESHOLD "$PAIR070"
#define PAIR_COMMON_GET_STATIC_THRESHOLD "$PAIR071"
#define PAIR_COMMON_SET_ELEV_MASK "$PAIR072"
#define PAIR_COMMON_GET_ELEV_MASK "$PAIR073"
#define PAIR_COMMON_SET_AIC_ENABLE "$PAIR074"
#define PAIR_COMMON_SET_NAVIGATION_MODE "$PAIR080"
#define PAIR_COMMON_GET_NAVIGATION_MODE "$PAIR081"
#define PAIR_COMMON_SET_NMEA_POS_DECIMAL_PRECISION "$PAIR098"
#define PAIR_COMMON_GET_NMEA_POS_DECIMAL_PRECISION "$PAIR099"
#define PAIR_COMMON_SET_NMEA_OUTPUT_MODE "$PAIR100"
#define PAIR_COMMON_GET_NMEA_OUTPUT_MODE "$PAIR101"
#define PAIR_COMMON_SET_DUAL_BAND "$PAIR104"
#define PAIR_COMMON_GET_DUAL_BAND "$PAIR105"
#define PAIR_TEST_JAMMING_DETECT "$PAIR391"
#define PAIR_DGPS_SET_MODE "$PAIR400"
#define PAIR_DGPS_GET_MODE "$PAIR401"
#define PAIR_SBAS_ENABLE "$PAIR410"
#define PAIR_SBAS_GET_STATUS "$PAIR411"
#define PAIR_EASY_ENABLE "$PAIR490"
#define PAIR_EASY_GET_STATUS "$PAIR491"
#define PAIR_NVM_SAVE_SETTING "$PAIR513"
#define PAIR_LOW_POWER_ENTER_RTC_MODE "$PAIR650"
#define PAIR_PERIODIC_SET_MODE "$PAIR690"
#define PAIR_PERIODIC_GET_MODE "$PAIR691"
#define PAIR_PPS_SET_CONFIG_CMD "$PAIR752"
#define PAIR_IO_SET_BAUDRATE "$PAIR864"
#define PAIR_IO_GET_BAUDRATE "$PAIR865"

#define PAIR_QUERY_FIX_RATE_NUM_ARGS 1
#define PAIR_QUERY_MIN_SNR_NUM_ARGS 1
#define PAIR_QUERY_BAUD_RATE_NUM_ARGS 1
#define PAIR_QUERY_NMEA_RATE_NUM_ARGS 2
#define PAIR_QUERY_GNSS_SEARCH_MODE_ARGS 6
#define PAIR_QUERY_STATIC_THRESHOLD_ARGS 1
#define PAIR_QUERY_NAVIGATION_MODE_ARGS 1
#define PAIR_QUERY_NMEA_DECIMAL_PRECISION_ARGS 1
#define PAIR_QUERY_DUAL_BAND_MODE_ARGS 1
#define PAIR_QUERY_SBAS_STATUS_ARGS 1
#define PAIR_QUERY_EASY_STATUS_ARGS 1

#define LC29_BAUD_RATE_4800 4800
#define LC29_BAUD_RATE_9600 9600
#define LC29_BAUD_RATE_19200 19200
#define LC29_BAUD_RATE_38400 38400
#define LC29_BAUD_RATE_57600 57600
#define LC29_BAUD_RATE_115200 115200

/* LC29 DR & RTK GNSS COMMAND HEADERS */
#define LC29_DR_MESSAGE_HEADER "$PQTM"
#define LC29_DR_CALIBRATION_HEADER "$PQTMDRCAL"
#define LC29_DR_IMU_TYPE_HEADER "$PQTMIMUTYPE"
#define LC29_DR_VEHICLE_INFO_HEADER "$PQTMVEHMSG"
#define LC29_DR_SAVE_SETTINGS_HEADER "$PQTMSAVEPAR"
#define LC29_DR_RESTORE_SETTINGS_HEADER "$PQTMRESTOREPAR"
#define LC29_DR_NAV_RESULTS_HEADER "$PQTMINS"
#define LC29_DR_IMU_DATA_HEADER "$PQTMIMU"
#define LC29_DR_GNSS_DATA_HEADER "$PQTMGPS"
#define LC29_DR_PQTM_MESSAGE_CONFIG_HEADER "$PQTMCFGEINSMSG"
#define LC29_DR_PQTM_MESSAGE_CONFIG_RESPONSE_HEADER "$PQTMEINSMSG"
#define LC29_DR_MOTION_OUTPUT_AFTER_CALIBRATION_HEADER "$PQTMVEHMOT"
#define LC29_DR_SENSOR_OUTPUT_HEADER "$PQTMSENMSG"
#define LC29_DR_RUNNING_TIME_COUNT_HEADER "$PQTMCFGDRRTD"
#define LC29_DR_IMU_TEMP_COMPENSATION_HEADER "$PQTMCFGIMUTC"
#define LC29_DR_POSITION_VELOCITY_ALTITUDE_HEADER "$PQTMDRPVA"
#define LC29_DR_HOT_START_HEADER "$PQTMCFGDRHOT"
#define LC29_DR_STATE_HEADER "$PQTMCFGDR"

#define LC29_DR_RESPONSE_OK "$PQTMCFGEINSMSGOK"
#define LC29_DR_RESPONSE_ERROR "$PQTMCFGEINSMSGERROR"
#define LC29_DR_CMD_RESPONSE_FIXED_LENGTH 17

#define PAIR_SET_CUSTOM_MSG_OUTPUT "$PAIR6010"
#define PAIR_GET_CUSTOM_MSG_OUTPUT "$PAIR6011"

#define PAIR_QUERY_CUSTOM_MSG_OUTPUT 2

#define PAIR_CMD_ENDING_CHAR "*"

#define LC29_DR_QUERY_PQTM_CONFIG_RESPONSE_ARGS 5

typedef struct qc_lc29_driver_s qc_lc29_driver_s;

typedef enum { DRIVER_SUCCESS, DRIVCER_FAIL } qc_lc29x_driver_response_t;

typedef struct {
  qc_lc29x_driver_response_t (*init)(void);
  qc_lc29x_driver_response_t (*write)(unsigned char *data, int length);
  qc_lc29x_driver_response_t (*read)(unsigned char *data, int length);
  qc_lc29x_driver_response_t (*config)(unsigned char config);
} qc_lc29x_driver_interface_s;

typedef enum {
  CMD_SEND_SUCCESS = 0,
  COMAND_BEING_PROCESSED,
  CMD_INVALID,
  CMD_SEND_FAIL,
  CMD_ID_NOT_SUPPORTED,
  CMD_PARAM_ERROR,
  MNL_SERVICE_BUSY,
} qc_lc29x_ack_reponse_t;

typedef enum { ONE_HZ = 1000, FIVE_HZ = 500, TEN_HZ = 100 } qc_lc29x_fix_rate_t;

typedef enum {
  GPS,
  GLONASS,
  GALILEO,
  BEIDOU,
  QZSS,
  MULTI
} qc_lc29x_constellation_t;

typedef enum {
  NMEA_SEN_GGA,
  NMEA_SEN_GLL,
  NMEA_SEN_GSA,
  NMEA_SEN_GSV,
  NMEA_SEN_RMC,
  NMEA_SEN_VTG
} qc_lc29x_nmea_output_rate_id_t;

typedef struct {
  qc_lc29x_nmea_output_rate_id_t nmea_id;
  uint8_t output_rate;
} qc_lc29x_nmea_output_rate_settings_s;

typedef struct {
  qc_lc29x_nmea_output_rate_settings_s gga;
  qc_lc29x_nmea_output_rate_settings_s gll;
  qc_lc29x_nmea_output_rate_settings_s gsa;
  qc_lc29x_nmea_output_rate_settings_s gsv;
  qc_lc29x_nmea_output_rate_settings_s rmc;
  qc_lc29x_nmea_output_rate_settings_s vtg;
} qc_lc29x_nmea_output_rate_s;

typedef struct {
  bool gps_enabled;
  bool glonass_enabled;
  bool galileo_enabled;
  bool beidou_enabled;
  bool qzss_enabled;
  bool reserved;
} qc_lc29x_gnss_search_mode_s;

typedef enum {
  NORMAL_MODE,
  FITNESS_MODE,
  RESERVED_2,
  RESERVED_3,
  STATIONARY_MODE,
  RESERVED_5,
  RESERVED_6,
  SWIMMING_MODE
} qc_lc29x_nav_mode_t;

typedef enum {
  LAT_LON_4_ALT_1,
  LAT_LON_5_ALT_2,
  LAT_LON_6_ALT_3,
  LAT_LON_7_ALT_3,
} qc_lc29x_dec_accuracy_t;

typedef enum {
  DISABLE_NMEA_OUTPUT,
  ENABLE_ASCII_NMEA_4_10, // default
  ENABLE_ASCII_NMEA_3_01
} qc_nmea_output_mode;

typedef enum {
  NO_DGPS_SOURCE,
  RESERVED,
  SBAS_ENABLED,
} qc_lc29x_dgps_mode_t;

typedef enum {
  NOT_INITIALIZED,
  NOT_FINISHED,
  FINISHED_1_DAY,
  FINISHED_2_DAY,
  FINISHED_3_DAY,
} qc_lc29x_easy_mode_status_t;

typedef enum {
  DISABLE_PERIODIC_MODE,
  SMART_PERIODIC_MODE,
  STRICT_PERIODIC_MODE
} qc_lc29x_periodic_sleep_mode_t;

typedef enum {
  DISABLE,
  AFTER_FIRST_FIX,
  FIX_ONLY_3D,
  FIX_ONLY_2D_OR_3D,
  ALWAYS
} qc_lc29x_pps_setting_t;

typedef enum {
  VALID_RESPONSE,
  LC_RESPONSE_INVALID_LENGTH,
  LC_RESPONSE_INVALID_START_CHAR,
  LC_RESPONSE_INVALID_R_N,
  LC_RESPONSE_INVALID_IDENTIFIER,
  LC_RESPONSE_NO_CHECKSUM,
  LC_RESPONSE_INVALID_CHECKSUM
} qc_lc29x_response_error_t;

typedef enum { PQTM_INS, PQTM_IMU, PQTM_GPS } qc_lc29x_dr_message_id_t;

typedef struct {
  qc_lc29x_dr_message_id_t message_id;
  bool enabled;
  int fix_rate;
} qc_lc29x_pqtm_output_rate_t;

typedef struct {
  qc_lc29x_pqtm_output_rate_t ins;
  qc_lc29x_pqtm_output_rate_t imu;
  qc_lc29x_pqtm_output_rate_t gps;
} qc_lc29x_pqtm_output_rate_settings_t;

typedef enum {
  PQTMVEHMSG,
  PQTMSENMSG,
  PQTMDRCAL,
  PQTMIMUTYPE,
  PQTMVEHMOT
} qc_lc29x_dr_custom_message_id_t;

typedef struct {
  qc_lc29x_dr_custom_message_id_t message_id;
  bool enabled;
} qc_lc29x_pqtm_custom_message_output_t;

typedef struct {
  qc_lc29x_pqtm_custom_message_output_t vehicle_info;
  qc_lc29x_pqtm_custom_message_output_t sensor_output;
  qc_lc29x_pqtm_custom_message_output_t dr_calibration;
  qc_lc29x_pqtm_custom_message_output_t imu_type;
  qc_lc29x_pqtm_custom_message_output_t dr_vehicle_motion;
} qc_lc29x_pqtm_custom_message_settings_t;

/* LC29 Driver Generic Methods */
qc_lc29_driver_s *Lc29_driver_ctor(
    qc_lc29x_driver_response_t (*lc29_driver_hw_init)(void),
    qc_lc29x_driver_response_t (*lc29_driver_write)(char *data, int length),
    qc_lc29x_driver_response_t (*lc29_driver_read)(char *data, int length),
    qc_lc29x_driver_response_t (*lc29_driver_config)(char config));

qc_lc29x_ack_reponse_t lc29_driver_parse_response(char *response_string,
                                                  int command_id);
qc_lc29x_ack_reponse_t lc29_driver_parse_query_response(char *response_string,
                                                        char *command_id,
                                                        int response_string_len,
                                                        int response_num_args,
                                                        int *parsed_query);

qc_lc29x_response_error_t lc29_driver_validate_string(char *pair_id,
                                                      const char *sentence,
                                                      size_t length,
                                                      int check_checksum);
uint8_t lc29_driver_response_has_checksum(const char *sentence, size_t length);
uint8_t lc29_driver_get_checksum(const char *sentence);
// qc_lc29x_response_error_t lc29_driver_transmit_cmd(char *cmd_payload,
//                                                    char *driver_cmd_response,
//                                                    uint16_t cmd_id);
qc_lc29x_ack_reponse_t lc29_driver_nvm_save_setting(qc_lc29_driver_s *driver,
                                                    bool enable);

/* LC29H Driver Setter Methods */
qc_lc29x_ack_reponse_t lc29_driver_set_fix_rate(qc_lc29_driver_s *driver,
                                                char *fix_rate);
qc_lc29x_ack_reponse_t lc29_driver_set_min_snr(qc_lc29_driver_s *driver,
                                               char *min_snr);
qc_lc29x_response_error_t lc29_driver_build_pair_cmd(uint8_t num_args,
                                                     char *cmd_id, char *args[],
                                                     char *cmd_result);
qc_lc29x_ack_reponse_t
lc29_driver_set_nmea_output_rate(qc_lc29_driver_s *driver, char *output_rate_id,
                                 char *output_rate);
qc_lc29x_ack_reponse_t lc29_driver_set_gnss_search_mode(
    qc_lc29_driver_s *driver, qc_lc29x_gnss_search_mode_s search_mode_settings);
qc_lc29x_ack_reponse_t
lc29_driver_set_static_threshold(qc_lc29_driver_s *driver,
                                 char *speed_threshold);
qc_lc29x_ack_reponse_t
lc29_driver_set_navigation_mode(qc_lc29_driver_s *driver,
                                qc_lc29x_nav_mode_t nav_mode);

/* LC29H Driver Getter Methods */
qc_lc29x_ack_reponse_t lc29_driver_get_fix_rate(qc_lc29_driver_s *driver);
qc_lc29x_ack_reponse_t lc29_driver_get_min_snr(qc_lc29_driver_s *driver);
qc_lc29x_ack_reponse_t
lc29_driver_get_nmea_output_rate(qc_lc29_driver_s *driver, char *nmea_rate_id);
qc_lc29x_ack_reponse_t
lc29_driver_get_gnss_search_mode(qc_lc29_driver_s *driver);
qc_lc29x_ack_reponse_t
lc29_driver_get_navigation_mode(qc_lc29_driver_s *driver);
qc_lc29x_ack_reponse_t lc29_driver_get_dual_band_mode(qc_lc29_driver_s *driver);

/* LC29H DR & RTK Message Structure */
qc_lc29x_ack_reponse_t
lc29_driver_parse_dr_cmd_response(char *response_string,
                                  int response_string_len, char *dr_cmd_id,
                                  int dr_cmd_id_len, int check_checksum);
qc_lc29x_ack_reponse_t
lc29_driver_set_dr_rtk_message_output(qc_lc29_driver_s *driver, char *msg_type,
                                      bool msg_type_output_state);

qc_lc29x_ack_reponse_t
lc29_driver_set_get_pqtm_message_settings(qc_lc29_driver_s *driver, bool type,
                                          bool ins_enabled, bool imu_enabled,
                                          bool gps_enabled, char *rate);

qc_lc29x_ack_reponse_t
lc29_driver_get_dr_rtk_message_output(qc_lc29_driver_s *driver, char *msg_type);

#endif