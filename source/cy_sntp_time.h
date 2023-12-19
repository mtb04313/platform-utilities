/*******************************************************************************
* File Name: cy_sntp_time.h
*
* Description: This file is the public interface of cy_sntp_time.c
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

/*******************************************************************************
 *  Include guard
 ******************************************************************************/
#ifndef SOURCE_CY_SNTP_TIME_H_
#define SOURCE_CY_SNTP_TIME_H_

/*******************************************************************************
 * Header file includes
 ******************************************************************************/
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
 * Function Prototypes
 ******************************************************************************/
void cy_sntp_time_start_sync(void);

void cy_sntp_time_stop_sync(void);

void cy_sntp_set_system_time_callback(int32_t sec, uint32_t frac);

bool cy_is_sntp_time_updated(void);

bool cy_wait_for_sntp_time(int timeout_sec);

bool cy_sntp_handle_timezone_change( float previous_diff,
                                     float new_diff);

void cy_sntp_get_current_time( char *buf,
                               size_t buf_size,
                               const char* format_specifier);

void cy_sntp_get_current_time_short( char *buf,
                                     size_t buf_size);

void cy_sntp_get_current_time_long(char *buf,
                                   size_t buf_size);

void cy_sntp_get_current_date_short( char *buf,
                                     size_t buf_size);

void cy_sntp_get_current_date_compact(char *buf,
                                      size_t buf_size);

void cy_sntp_print_timestamp(const char* text_p, int line);

#ifdef __cplusplus
}
#endif

#endif // SOURCE_CY_SNTP_TIME_H_

/* [] END OF FILE */
