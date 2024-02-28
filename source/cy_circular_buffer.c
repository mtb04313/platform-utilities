/******************************************************************************
* File Name:   cy_circular_buffer.c
*
* Description: This file implements circular buffer related functions.
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

#include <string.h>   /* for memset */
#include "cy_circular_buffer.h"
#include "cy_debug.h"
#include "cy_misc_types.h"


/*-- Local Functions -------------------------------------------------*/


/*-- Public Functions -------------------------------------------------*/

void CyInitCircularBufferFromArray( CyCircularBuffer_t *p_cyBuf,
                                    uint8_t *p_data_buf,
                                    uint32_t data_buf_len)
{
    VoidAssert(p_cyBuf != NULL);
    VoidAssert(p_data_buf != NULL);

    p_cyBuf->p_data = p_data_buf;
    p_cyBuf->len = data_buf_len;
    memset(p_cyBuf->p_data, 0, p_cyBuf->len);

    p_cyBuf->head_index = 0;
    p_cyBuf->tail_index = 0;
    p_cyBuf->head_loop = 0;
    p_cyBuf->tail_loop = 0;
    p_cyBuf->safety_margin_min_percent = SAFETY_MARGIN_100_PERCENT;
}

void CyCheckAndWrapAround( CyCircularBuffer_t *p_cyBuf,
                           uint32_t *p_temp_index,
                           uint32_t *p_temp_loop)
{
    VoidAssert(p_cyBuf != NULL);
    VoidAssert(p_temp_index != NULL);
    VoidAssert(p_temp_loop != NULL);

    // wrap-over if we have reach the end
    if (*p_temp_index == p_cyBuf->len) {
        *p_temp_index = 0;
        (*p_temp_loop)++;
    }
}

// We are saving index and loop into passed-in params instead of the members of IfxCircularBuffer_t
// This gives the caller the flexibility of committing them at a later time
void CyStoreUint8( CyCircularBuffer_t *p_cyBuf,
                   uint32_t *p_temp_head_index,
                   uint32_t *p_temp_head_loop,
                   uint8_t data)
{
    CyCheckAndWrapAround(p_cyBuf,
                         p_temp_head_index,
                         p_temp_head_loop);

    p_cyBuf->p_data[(*p_temp_head_index)++] = data;
}

// We are saving index and loop into passed-in params instead of the members of IfxCircularBuffer_t
// This gives the caller the flexibility of committing them at a later time
void CyStoreUint16(CyCircularBuffer_t *p_cyBuf,
                   uint32_t *p_temp_head_index,
                   uint32_t *p_temp_head_loop,
                   uint16_t data)
{
    CyStoreUint8(p_cyBuf,
                 p_temp_head_index,
                 p_temp_head_loop,
                 HIBYTE(data));

    CyStoreUint8(p_cyBuf,
                 p_temp_head_index,
                 p_temp_head_loop,
                 LOBYTE(data));
}

// We are saving index and loop into passed-in params instead of the members of IfxCircularBuffer_t
// This gives the caller the flexibility of committing them at a later time
void CyStoreUint8Array(CyCircularBuffer_t *p_cyBuf,
                       uint32_t *p_temp_head_index,
                       uint32_t *p_temp_head_loop,
                       uint8_t* p_array,
                       uint32_t array_len)
{
    CyCheckAndWrapAround(p_cyBuf,
                         p_temp_head_index,
                         p_temp_head_loop);

    if (p_cyBuf->len >= (*p_temp_head_index + array_len)) {
        // there's space to store the array as 1 contiguous piece
        memcpy(&(p_cyBuf->p_data[*p_temp_head_index]), p_array, array_len);
        *p_temp_head_index += array_len;
    }
    else {
        // store the array in 2 pieces
        uint16_t lenPart1 = p_cyBuf->len - *p_temp_head_index;
        uint16_t lenPart2 = array_len - lenPart1;

        // first piece
        memcpy(&(p_cyBuf->p_data[*p_temp_head_index]), p_array, lenPart1);

        // wrap around
        *p_temp_head_index = 0;
        (*p_temp_head_loop)++;

        // second piece
        memcpy(&(p_cyBuf->p_data[*p_temp_head_index]), &p_array[lenPart1], lenPart2);
        *p_temp_head_index += lenPart2;
    }

    CyCheckAndWrapAround(p_cyBuf,
                         p_temp_head_index,
                         p_temp_head_loop);
}

void CyComputeSafetyMargin(CyCircularBuffer_t *p_cyBuf,
                           uint32_t *p_temp_head_index,
                           uint32_t *p_temp_head_loop)
{
    uint32_t margin_percent = SAFETY_MARGIN_100_PERCENT;

    VoidAssert(p_cyBuf != NULL);
    VoidAssert(p_temp_head_index != NULL);
    VoidAssert(p_temp_head_loop != NULL);

    if (*p_temp_head_loop > p_cyBuf->tail_loop) {
        if (*p_temp_head_index >= p_cyBuf->tail_index) {
            // if head_index catches up with tail_index, then a buffer overrun has occurred,
            margin_percent = 0;
        }
        else {
            margin_percent = (p_cyBuf->tail_index - *p_temp_head_index)
                             * 100 / p_cyBuf->len;
        }
    }
    else {
        margin_percent = (p_cyBuf->len - *p_temp_head_index + p_cyBuf->tail_index)
                         * 100 / p_cyBuf->len;
    }

    if (margin_percent < p_cyBuf->safety_margin_min_percent) {
        p_cyBuf->safety_margin_min_percent = margin_percent;
    }
}

// commit the passed-in params into the members of IfxCircularBuffer_t
void CySaveHeadIndexAndLoop(CyCircularBuffer_t *p_cyBuf,
                            uint32_t *p_temp_head_index,
                            uint32_t *p_temp_head_loop)
{
    VoidAssert(p_cyBuf != NULL);
    VoidAssert(p_temp_head_index != NULL);
    VoidAssert(p_temp_head_loop != NULL);

    p_cyBuf->head_index = *p_temp_head_index;
    p_cyBuf->head_loop = *p_temp_head_loop;
}

// commit the passed-in params into the members of IfxCircularBuffer_t
void CySaveTailIndexAndLoop(CyCircularBuffer_t *p_cyBuf,
                            uint32_t *p_temp_tail_index,
                            uint32_t *p_temp_tail_loop)
{
    VoidAssert(p_cyBuf != NULL);
    VoidAssert(p_temp_tail_index != NULL);
    VoidAssert(p_temp_tail_loop != NULL);

    p_cyBuf->tail_index = *p_temp_tail_index;
    p_cyBuf->tail_loop = *p_temp_tail_loop;
}

void CyComputeHeadAndTailDiff( CyCircularBuffer_t *p_cyBuf,
                               uint32_t temp_head_index,
                               uint32_t temp_tail_index,
                               uint32_t *p_diff)
{
    VoidAssert(p_cyBuf != NULL);
    VoidAssert(p_diff != NULL);
    VoidAssert(p_cyBuf->len > temp_head_index);
    VoidAssert(p_cyBuf->len > temp_tail_index);

    if (temp_head_index > temp_tail_index) {
        *p_diff = temp_head_index - temp_tail_index;
    }
    else if (temp_head_index < temp_tail_index) {
        *p_diff = p_cyBuf->len - temp_tail_index + temp_head_index;
    }
    else {
        *p_diff = 0;
    }
}

// We are saving index and loop into passed-in params instead of the members of IfxCircularBuffer_t
// This gives the caller the flexibility of committing them at a later time
void CyFetchUint8( CyCircularBuffer_t *p_cyBuf,
                   uint32_t *p_temp_tail_index,
                   uint32_t *p_temp_tail_loop,
                   uint8_t *p_data)
{
    CyCheckAndWrapAround(p_cyBuf,
                         p_temp_tail_index,
                         p_temp_tail_loop);

    VoidAssert(p_data != NULL);
    *p_data = p_cyBuf->p_data[(*p_temp_tail_index)++];
}

// We are saving index and loop into passed-in params instead of the members of IfxCircularBuffer_t
// This gives the caller the flexibility of committing them at a later time
void CyFetchUint16(CyCircularBuffer_t *p_cyBuf,
                   uint32_t *p_temp_tail_index,
                   uint32_t *p_temp_tail_loop,
                   uint16_t *p_data)
{
    uint8_t hiByte = 0;
    uint8_t loByte = 0;

    CyFetchUint8(p_cyBuf,
                 p_temp_tail_index,
                 p_temp_tail_loop,
                 &hiByte);

    CyFetchUint8(p_cyBuf,
                 p_temp_tail_index,
                 p_temp_tail_loop,
                 &loByte);

    VoidAssert(p_data != NULL);
    *p_data = MAKE_UWORD(hiByte, loByte);
}

// We are saving index and loop into passed-in params instead of the members of IfxCircularBuffer_t
// This gives the caller the flexibility of committing them at a later time
void CyFetchUint32(CyCircularBuffer_t *p_cyBuf,
                   uint32_t *p_temp_tail_index,
                   uint32_t *p_temp_tail_loop,
                   uint32_t *p_data)
{
    uint16_t hiWord = 0;
    uint16_t loWord = 0;

    CyFetchUint16(p_cyBuf,
                  p_temp_tail_index,
                  p_temp_tail_loop,
                  &hiWord);

    CyFetchUint16(p_cyBuf,
                  p_temp_tail_index,
                  p_temp_tail_loop,
                  &loWord);

    VoidAssert(p_data != NULL);
    *p_data = MAKE_ULONG(hiWord, loWord);
}

// We are saving index and loop into passed-in params instead of the members of IfxCircularBuffer_t
// This gives the caller the flexibility of committing them at a later time
void CyFetchUint8Array(CyCircularBuffer_t *p_cyBuf,
                       uint32_t *p_temp_tail_index,
                       uint32_t *p_temp_tail_loop,
                       uint32_t array_len,
                       uint8_t* p_out_buf,
                       uint32_t out_buf_len)
{
    CyCheckAndWrapAround(p_cyBuf,
                         p_temp_tail_index,
                         p_temp_tail_loop);

    VoidAssert(p_out_buf != NULL);
    VoidAssert(out_buf_len >= array_len);

    if (p_cyBuf->len >= (*p_temp_tail_index + array_len)) {
        // fetch the array as 1 contiguous piece
        memcpy(p_out_buf, &(p_cyBuf->p_data[*p_temp_tail_index]), array_len);
        *p_temp_tail_index += array_len;
    }
    else {
        // fetch the array in 2 pieces
        uint16_t lenPart1 = p_cyBuf->len - *p_temp_tail_index;
        uint16_t lenPart2 = array_len - lenPart1;

        // first piece
        memcpy(p_out_buf, &(p_cyBuf->p_data[*p_temp_tail_index]), lenPart1);

        // wrap around
        *p_temp_tail_index = 0;
        (*p_temp_tail_loop)++;

        // second piece
        memcpy(&p_out_buf[lenPart1], &(p_cyBuf->p_data[*p_temp_tail_index]), lenPart2);
        *p_temp_tail_index += lenPart2;
    }

    CyCheckAndWrapAround(p_cyBuf,
                         p_temp_tail_index,
                         p_temp_tail_loop);
}

