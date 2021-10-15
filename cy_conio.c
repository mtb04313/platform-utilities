/*******************************************************************************
* File Name: cy_conio.c
*
* Description: This file contains functions that read input char or
* string from the UART console.
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
 * Header file includes
 ******************************************************************************/
#include "cy_conio.h"
#include "cy_debug.h"
#include "cy_pdl.h"
#include "cyhal.h"

/* UART HAL object used by Retarget-IO for Debug UART port */
extern cyhal_uart_t cy_retarget_io_uart_obj;


/*******************************************************************************
 * Public Functions
 ******************************************************************************/

uint8_t get_key(void)
{
    uint8_t uart_read_value;

    if (cyhal_uart_getc(&cy_retarget_io_uart_obj, &uart_read_value, 1)
            == CY_RSLT_SUCCESS) {
        return uart_read_value;
    }

    return 0;
}


uint8_t wait_for_key(void)
{
    while (1) {
        uint8_t uart_read_value = get_key();
        if (uart_read_value != 0) {
            return uart_read_value;
        }
    }
}

// Read a string until Enter key is pressed
// Warning: does not handle a backspace correctly
// Requires Echo mode to be enabled in terminal program (minicom, putty)
int get_string(char *out_p, int max_chars)
{
    int i = 0;
    ReturnAssert(out_p != NULL, 0);
    ReturnAssert(max_chars > 0, 0);

    while (i < max_chars) {
        char key = wait_for_key();
        if ((key != '\n') && (key != '\r')) {
            out_p[i] = key;
            i++;
        } else {
            break;
        }
    };

    return i;
}


/* [] END OF FILE */
