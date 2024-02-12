#include "gnss_driver_tests.h"
#include <stdio.h>
#include <string.h>

qc_lc29x_driver_response_t driverA_init(void) {
  // Initialization code for Microcontroller A (random things being returned)
  return DRIVER_SUCCESS;
}

qc_lc29x_driver_response_t driverA_write(char *data, int length) {
  // Write data to Microcontroller A (random things being returned)
  printf("Driver Write Func %s", data);
  return DRIVER_SUCCESS;
}

qc_lc29x_driver_response_t driverA_read_fix_rate(char *data, int length) {
  // Read data from Microcontroller A (random things being returned)
  char test_response[] = "$PAIR001,050,0*3E\n\r";
  strcpy(data, test_response);
  return DRIVER_SUCCESS;
}

qc_lc29x_driver_response_t driverA_query_fix_rate_response(char *data,
                                                           int length) {
  static int response_state = 0;

  if (response_state == 0) {
    char test_response[] = "$PAIR001,051,0*3F\n\r";
    strcpy(data, test_response);
    response_state = 1;
    return DRIVER_SUCCESS;
  } else {
    char test_response[] = "$PAIR051,1000*13\n\r";
    strcpy(data, test_response);
    response_state = 0;
    return DRIVER_SUCCESS;
  }
}

qc_lc29x_driver_response_t driverA_read_min_snr(char *data, int length) {
  // Read data from Microcontroller A (random things being returned)
  char test_response[] = "$PAIR001,058,0*36\n\r";
  strcpy(data, test_response);
  return DRIVER_SUCCESS;
}

qc_lc29x_driver_response_t driverA_query_min_snr_response(char *data,
                                                          int length) {
  static int response_state = 0;

  if (response_state == 0) {
    char test_response[] = "$PAIR001,059,0*37\n\r";
    strcpy(data, test_response);
    response_state = 1;
    return DRIVER_SUCCESS;
  } else {
    char test_response[] = "$PAIR059,15*1E\n\r";
    strcpy(data, test_response);
    response_state = 0;
    return DRIVER_SUCCESS;
  }
}

qc_lc29x_driver_response_t driverA_read_nmea_output_rate(char *data,
                                                         int length) {
  // Read data from Microcontroller A (random things being returned)
  char test_response[] = "$PAIR001,062,0*3F\n\r";
  strcpy(data, test_response);
  return DRIVER_SUCCESS;
}

qc_lc29x_driver_response_t driverA_read_search_mode_settings(char *data,
                                                             int length) {
  // Read data from Microcontroller A (random things being returned)
  char test_response[] = "$PAIR001,066,0*3B\n\r";
  strcpy(data, test_response);
  return DRIVER_SUCCESS;
}

qc_lc29x_driver_response_t driverA_read_static_threshold(char *data,
                                                         int length) {
  // Read data from Microcontroller A (random things being returned)
  char test_response[] = "$PAIR001,070,0*3C\n\r";
  strcpy(data, test_response);
  return DRIVER_SUCCESS;
}

qc_lc29x_driver_response_t driverA_query_pqtm_settings(char *data, int length) {
  static int response_state = 0;

  if (response_state == 0) {
    char test_response[] = "$PQTMCFGEINSMSGOK*16\n\r";
    strcpy(data, test_response);
    response_state = 1;
    return DRIVER_SUCCESS;
  } else {
    char test_response[] = "$PQTMEINSMSG,0,1,1,1,10*7C\n\r";
    strcpy(data, test_response);
    response_state = 0;
    return DRIVER_SUCCESS;
  }
  return DRIVER_SUCCESS;
}

qc_lc29x_driver_response_t driverA_read_dr_rtk_message_output(char *data,
                                                              int length) {
  // Read data from Microcontroller A (random things being returned)
  char test_response[] = "$PAIR001,6010,0*0C\n\r";
  strcpy(data, test_response);
  return DRIVER_SUCCESS;
}

qc_lc29x_driver_response_t driverA_query_dr_rtk_message_output(char *data,
                                                               int length) {
  static int response_state = 0;

  if (response_state == 0) {
    char test_response[] = "$PAIR001,6011,0*0D\n\r";
    strcpy(data, test_response);
    response_state = 1;
    return DRIVER_SUCCESS;
  } else {
    char test_response[] = "$PAIR6011,1,0*0D\n\r";
    strcpy(data, test_response);
    response_state = 0;
    return DRIVER_SUCCESS;
  }
  return DRIVER_SUCCESS;
}

qc_lc29x_driver_response_t driverA_config(char config) {
  // Configure Microcontroller A (random things being returned)

  return DRIVER_SUCCESS;
}