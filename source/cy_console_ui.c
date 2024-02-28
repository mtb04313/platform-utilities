/******************************************************************************
* File Name:   cy_console_ui.c
*
* Description: This file implements the console user interface
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
#include <stdint.h>
#include <ctype.h>

#include "cy_console_ui.h"
#include "cy_conio.h"
#include "cy_debug.h"
#include "cy_time_misc.h"

#define USER_CONFIRMATION_INTERVAL_MSEC     1000


/*-- Public Functions -------------------------------------------------*/

bool get_user_confirmation(void)
{
    bool result = false;
    do {
        uint8_t selection = 0x00;
        PRINT_MSG(("\n# Proceed?\n"));
        PRINT_MSG(("  Y  Yes\n"));
        PRINT_MSG(("  N  No\n"));

        selection = tolower(wait_for_key());
        PRINT_MSG(("\n"));

        if ((selection == 'y') || (selection == 'n')) {
            result = (selection == 'y');
            break;
        }
    } while (true);

    return result;
}

void get_user_acknowledgement(const char* message_p)
{
    if (message_p != NULL) {
        PRINT_MSG(("%s\n", message_p));
    }

    wait_for_key();
    PRINT_MSG(("\n"));
}

bool get_user_confirmation_with_timeout(bool default_answer, int timeout_sec)
{
    bool result = default_answer;
    uint8_t selection = 0x00;
    int stamp = timeout_sec;

    while (stamp > 0) {

        selection = tolower(get_key());
        PRINT_MSG(("%d\n", stamp));

        if ((selection == 'y') || (selection == 'n')) {
            result = (selection == 'y');
            break;
        }

        stamp--;

        msleep(USER_CONFIRMATION_INTERVAL_MSEC);
    }

    return result;
}
