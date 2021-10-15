/******************************************************************************
* File Name:   cy_byte_array.c
*
* Description: This file implements byte array manipulation functions.
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
#include <string.h>
#include <stdio.h>

#include "cy_byte_array.h"
#include "cy_debug.h"
#include "cy_string.h"


/*-- Public Functions -------------------------------------------------*/

// Method to return a String with the hex representation of the ByteArray.
// E.g. "053F5A"
bool ByteArrayToString( const uint8_t *array_p,
                        size_t arraySize,
                        char *toString_p,
                        size_t toStringBufSize)
{
#define HAVE_SPACE_DELIMITER    0

    size_t i;
    size_t requiredBufSize = 0;

    ReturnAssert(array_p != NULL, false);
    ReturnAssert(arraySize != 0, false);
    ReturnAssert(toString_p != NULL, false);

#if (HAVE_SPACE_DELIMITER == 1)
    requiredBufSize = (arraySize * 3) - 1;
#else
    requiredBufSize = (arraySize * 2);
#endif
    requiredBufSize++;  // add 1 for null terminator

    ReturnAssert(requiredBufSize <= toStringBufSize, false);
    toString_p[0] = '\0';

    for (i = 0; i < arraySize; i++) {
        char tempBuf[3];
        SNPRINTF(tempBuf, sizeof(tempBuf), "%02X", array_p[i]);
        tempBuf[2] = '\0';  // make sure it's null-terminated
        strcat(toString_p, tempBuf);

#if (HAVE_SPACE_DELIMITER == 1)
        if (i < (arraySize - 1)) {
            strcat(toString_p, " ");
        }
#endif
    }

    return true;

#undef HAVE_SPACE_DELIMITER
}

// Method to return a ByteArray from a String hex representation
// E.g. "053F5A"
bool StringToByteArray( const char *fromString,
                        uint8_t *toArray_p,
                        size_t toArrayBufSize)
{
    size_t fromIndex = 0;
    size_t fromLen;
    size_t toIndex = 0;

    ReturnAssert(fromString != NULL, false);
    ReturnAssert(toArray_p != NULL, false);

    fromLen = strlen(fromString);

    ReturnAssert((fromLen % 2) == 0, false);
    ReturnAssert(toArrayBufSize >= (fromLen / 2), false);

    while (fromIndex < fromLen) {
        unsigned int value;

        /* sscanf returns the num of items successfully filled */
        if (sscanf(&fromString[fromIndex], "%02X", &value) == 1) {
            toArray_p[toIndex] = value;
            toIndex++;
        }
        fromIndex += 2;
    }

    return true;
}
