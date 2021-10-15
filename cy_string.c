/******************************************************************************
* File Name:   cy_string.c
*
* Description: This file implements the string functions.
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

#include <stdlib.h>
#include <ctype.h>
#include <string.h>

#include "cy_string.h"
#include "cy_debug.h"
#include "cy_memtrack.h"

/*-- Public Functions -------------------------------------------------*/

char* left_trim(char* text_p)
{
    char *temp = NULL;
    if (text_p != NULL) {
        temp = text_p;

        while ((*temp != '\0') && isspace((int)*temp)) {
            temp++;
        }
    }
    return temp;
}

// remove trailing spaces
// Note: function will modify text_p
char* right_trim(char* text_p)
{
    char *temp = NULL;

    if (text_p != NULL) {
        size_t len = strlen(text_p);
        if (len > 0) {
            temp = &text_p[len - 1];

            while (isspace((int)*temp)) {
                *temp = '\0';
                temp--;
                if (temp == text_p) {
                    break;
                }
            }
        }
        temp = text_p;
    }
    return temp;
}

char *cy_strdup(const char *text_p)
{
    size_t len = strlen(text_p);
    char *dup_p = (char*)CY_MEMTRACK_MALLOC(len + 1);

    if (dup_p != NULL) {
        strcpy(dup_p, text_p);
    }
    return dup_p;
}
