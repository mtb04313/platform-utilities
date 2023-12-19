/*******************************************************************************
* File Name: cy_sntp_time.c
*
* Description: This file contains functions that get/set time using SNTP
* (Simple Network Time Protocol)
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2020-2021, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#include <stdio.h>
#include <string.h>

#include "feature_config.h"

/* FreeRTOS header files */
#include "FreeRTOS.h"
#include "task.h"

#include "cy_time_misc.h"
#include "cy_sntp_time.h"
#include "cy_memtrack.h"
#include "cy_debug.h"
#include "cy_string.h"

#include "sntp.h"
#include "time_config.h"

#if (FEATURE_FLASH_EEPROM == ENABLE_FEATURE)
#include "flash_eeprom.h"
#endif

/*-- Local Definition -------------------------------------------------*/

#define USE_24_HOUR_CLOCK     1 /* enable=1; disable=0 */

/* Number of seconds between 1970 and Feb 7, 2036 06:28:16 UTC (epoch 1) */
#define DIFF_SEC_1970_2036          ((uint32_t)2085978496L)

#define COMPACT_DATETIME_LEN        12

/*-- Local Data -------------------------------------------------*/

static bool s_sntp_updated = false;


/*-- Local Functions -------------------------------------------------*/

static void initialize_sntp(void)
{
  // Force cy_time.c (in mtb_shared/clib_support) to create the RTC instance first,
  // which involves a RTC reset.
  // Then cy_sntp_set_system_time_callback() can set the RTC
  time_t time_now = time(NULL);
  DEBUG_PRINT(("time_now = %ld\n", (long)time_now));

  sntp_setoperatingmode(SNTP_OPMODE_POLL);
  sntp_setservername(0, "pool.ntp.org");

  sntp_init();
  s_sntp_updated = false;
}


/*-- Public Functions -------------------------------------------------*/

void cy_sntp_time_start_sync(void)
{
  DEBUG_PRINT(("Initializing SNTP\n"));
  initialize_sntp();
}

void cy_sntp_time_stop_sync(void)
{
  DEBUG_PRINT(("Stopping SNTP\n"));
  sntp_stop();
}

void cy_sntp_set_system_time_callback( int32_t sec,
                                       uint32_t frac /* ignored */)
{
  bool do_restore = false;
  time_t ut;
  float timezone_diff = DEFAULT_TIMEZONE_DIFF;

  DEBUG_PRINT(("SNTP sec = %ld\n", sec));

#if (FEATURE_FLASH_EEPROM == ENABLE_FEATURE)
  cy_time_info_t time_info;

  memset(&time_info, 0, sizeof(time_info));
  if (!flash_eeprom_get_time_info(&time_info,
                                  sizeof(time_info))) {
    DEBUG_PRINT(("flash_eeprom_get_time_info failed\n"));
  }

  timezone_diff = time_info.timezone_diff;

  // if caller passes 'zero' as input, we'll restore timestamp from eeprom
  if (sec == 0) {
    do_restore = true;
    sec = time_info.timestamp;
  }

#else

  if (sec == 0) {
    return;
  }
#endif

  // get epoch time (num of seconds since 1970) in GMT timezone
  ut = (uint32_t)((uint32_t)sec + DIFF_SEC_1970_2036);

  // adjust epoch time to local timezone
  ut += (int32_t)(timezone_diff * 3600);

  // convert epoch time to struct tm
  struct tm *tm_p = gmtime(&ut);

  // convert struct tm to RTC time
  cy_stc_rtc_config_t rtcTime ;
  memset(&rtcTime, 0, sizeof(rtcTime)) ;

  rtcTime.sec = tm_p->tm_sec;
  rtcTime.min = tm_p->tm_min;
  rtcTime.hour = tm_p->tm_hour;
  rtcTime.amPm = CY_RTC_AM;
  rtcTime.hrFormat = CY_RTC_24_HOURS;
  rtcTime.dayOfWeek = CY_RTC_DAY_AUTO; //tm_p->tm_wday + 1;
  rtcTime.date = tm_p->tm_mday;
  rtcTime.month = tm_p->tm_mon + 1;
  rtcTime.year = tm_p->tm_year - 121; // tm_year is 121 for year 2021

  if (Cy_RTC_SetDateAndTime(&rtcTime) == CY_RTC_SUCCESS) {
    if (!do_restore) {
      s_sntp_updated = true;
    }
  }
}

bool cy_is_sntp_time_updated(void)
{
  return s_sntp_updated;
}

bool cy_wait_for_sntp_time(int timeout_sec)
{
  bool result = false;
  int i = 0;

  for (i = 0; i < timeout_sec; i++) {
    DEBUG_PRINT(("Waiting for system time to be set... (%d/%d)\n",
        i, timeout_sec));

    result = cy_is_sntp_time_updated();
    if (result)
      break;

    //vTaskDelay(2000 / portTICK_PERIOD_MS);
    msleep(1000);
  }

  return result;
}


bool cy_sntp_handle_timezone_change( float previous_diff,
                                     float new_diff)
{
  cy_stc_rtc_config_t now;
  struct tm tm ;

  DEBUG_PRINT(("%s [%d]: previous_diff = %f, new_diff = %f\n",
      __FUNCTION__, __LINE__, previous_diff, new_diff));

  if (previous_diff == new_diff) {
    // nothing to do
    return true;
  }

  // get current RTC time
  Cy_RTC_GetDateAndTime(&now) ;

  // convert RTC time to struct tm
  memset(&tm, 0, sizeof(tm)) ;
  tm.tm_sec = now.sec ;
  tm.tm_min = now.min ;
  tm.tm_hour = now.hour ;
  tm.tm_mon = now.month - 1;
  tm.tm_mday = now.date ;
  tm.tm_wday = now.dayOfWeek - 1;
  tm.tm_year = now.year + 2021 - 1900;

  // convert struct tm to epoch time (num of seconds since 1970)
  time_t ut = mktime(&tm);

  // adjust epoch time to GMT timezone
  ut -= (int32_t)(previous_diff * 3600);

  // adjust to new local timezone
  ut += (int32_t)(new_diff * 3600); // convert to local time in sec

  // convert epoch time to struct tm
  struct tm *tm_p = gmtime(&ut);

  // convert struct tm to RTC time
  cy_stc_rtc_config_t rtcTime ;
  memset(&rtcTime, 0, sizeof(rtcTime)) ;

  rtcTime.sec = tm_p->tm_sec;
  rtcTime.min = tm_p->tm_min;
  rtcTime.hour = tm_p->tm_hour;
  rtcTime.amPm = CY_RTC_AM;
  rtcTime.hrFormat = CY_RTC_24_HOURS;
  rtcTime.dayOfWeek = CY_RTC_DAY_AUTO; //tm_p->tm_wday + 1;
  rtcTime.date = tm_p->tm_mday;
  rtcTime.month = tm_p->tm_mon + 1;
  rtcTime.year = tm_p->tm_year - 121; // tm_year is 121 for year 2021

  return (Cy_RTC_SetDateAndTime(&rtcTime) == CY_RTC_SUCCESS);
}

// Ref:
// https://www.cplusplus.com/reference/ctime/strftime/
void cy_sntp_get_current_time( char *buf,
                               size_t buf_size,
                               const char* format_specifier)
{
  cy_stc_rtc_config_t now;
  struct tm tm ;

  VoidAssert(buf != NULL);
  VoidAssert(format_specifier != NULL);

  Cy_RTC_GetDateAndTime(&now) ;

  memset(&tm, 0, sizeof(tm)) ;
  tm.tm_sec = now.sec ;
  tm.tm_min = now.min ;
  tm.tm_hour = now.hour ;
  tm.tm_mon = now.month - 1;
  tm.tm_mday = now.date ;
  tm.tm_wday = now.dayOfWeek - 1;
  tm.tm_year = now.year + 2021 - 1900;

  strftime (buf, buf_size, format_specifier, &tm);
}

// e.g. 17:07:27
void cy_sntp_get_current_time_short( char *buf,
                                     size_t buf_size)
{
#if USE_24_HOUR_CLOCK
  cy_sntp_get_current_time(buf, buf_size, "%H:%M:%S");     // 24-hour
#else
  cy_sntp_get_current_time(buf, buf_size, "%I:%M:%S %p");  // 12-hour
#endif
}

// e.g. Thu Jun 17, 2021, 17:07:27
void cy_sntp_get_current_time_long(char *buf,
                                   size_t buf_size)
{
#if USE_24_HOUR_CLOCK
  cy_sntp_get_current_time(buf, buf_size, "%a %b %d, %Y, %H:%M:%S");     // 24-hour
#else
  cy_sntp_get_current_time(buf, buf_size, "%a %b %d, %Y, %I:%M:%S %p");  // 12-hour
#endif
}

// e.g. Thu 17 Jun
void cy_sntp_get_current_date_short( char *buf,
                                     size_t buf_size)
{
  cy_sntp_get_current_time(buf, buf_size, "%a %d %b");
}

// e.g. 210617170727
void cy_sntp_get_current_date_compact(char *buf,
                                      size_t buf_size)
{
  cy_sntp_get_current_time(buf, buf_size, "%y%m%d%H%M%S");
}

void cy_sntp_print_timestamp(const char* text_p, int line)
{
  char compactDateTime[COMPACT_DATETIME_LEN + 1] = "";
  cy_sntp_get_current_date_compact(compactDateTime, sizeof(compactDateTime));
  DEBUG_PRINT(("%s [%d] compactDateTime = %s\n", text_p, line, compactDateTime));
}

/* [] END OF FILE */
