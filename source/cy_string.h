/******************************************************************************
* File Name:   cy_string.h
*
* Description: This file is the public interface of cy_string.c
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

#ifndef SOURCE_CY_STRING_H_
#define SOURCE_CY_STRING_H_


#ifdef __cplusplus
extern "C"
{
#endif


/*-- Public Definitions -------------------------------------------------*/

#include <string.h>
#include <stdio.h>

#define F_ZU  "u"
#define F_U32 "lu"

#define SNPRINTF    snprintf
#define STRNICMP    strncasecmp
#define STRTOK_S    strtok_r

#define STRNCPY_APPEND_NULL(dest, source, num)     strncpy((dest), (source), (num)), (dest)[(num)] = '\0'
#define STRDUP      cy_strdup

/*-- Public Functions -------------------------------------------------*/

char* left_trim(char* text_p);

char* right_trim(char* text_p); // Note: function will modify text_p

char *cy_strdup(const char *text_p);

char* remove_left_char(char* input_text_p, char matching_char);

char* remove_right_char(char* input_text_p, char matching_char); // Note: function will modify input_text_p

char* remove_matching_char(char* input_text_p, char matching_char); // Note: function will modify input_text_p

char* remove_quotes(char* quoted_text_p); // Note: function will modify quoted_text_p

char* remove_crlf(char* input_text_p); // Note: function will modify input_text_p

#ifdef __cplusplus
}
#endif

#endif // SOURCE_CY_STRING_H_
