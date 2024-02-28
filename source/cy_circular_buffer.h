/******************************************************************************
* File Name:   cy_circular_buffer.h
*
* Description: This file is the public interface of cy_circular_buffer.c
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

#ifndef _CY_CIRCULAR_BUFFER_H_
#define _CY_CIRCULAR_BUFFER_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

/*-- Public Definitions -------------------------------------------------*/

#define SAFETY_MARGIN_100_PERCENT   100

typedef struct {
    uint32_t head_index;
    uint32_t tail_index;
    uint32_t head_loop;
    uint32_t tail_loop;
    uint32_t safety_margin_min_percent;
    uint32_t len;
    uint8_t* p_data;

} CyCircularBuffer_t;


/*-- Public Functions -------------------------------------------------*/

void CyInitCircularBufferFromArray( CyCircularBuffer_t *p_cyBuf,
                                    uint8_t *p_data_buf,
                                    uint32_t data_buf_len);

void CyCheckAndWrapAround( CyCircularBuffer_t *p_cyBuf,
                           uint32_t *p_temp_index,
                           uint32_t *p_temp_loop);

// store operations
void CyStoreUint8( CyCircularBuffer_t *p_cyBuf,
                   uint32_t *p_temp_head_index,
                   uint32_t *p_temp_head_loop,
                   uint8_t data);

void CyStoreUint16(CyCircularBuffer_t *p_cyBuf,
                   uint32_t *p_temp_head_index,
                   uint32_t *p_temp_head_loop,
                   uint16_t data);

void CyStoreUint8Array(CyCircularBuffer_t *p_cyBuf,
                       uint32_t *p_temp_head_index,
                       uint32_t *p_temp_head_loop,
                       uint8_t* p_array,
                       uint32_t array_len);

void CyComputeSafetyMargin(CyCircularBuffer_t *p_cyBuf,
                           uint32_t *p_temp_head_index,
                           uint32_t *p_temp_head_loop);

void CySaveHeadIndexAndLoop(CyCircularBuffer_t *p_cyBuf,
                            uint32_t *p_temp_head_index,
                            uint32_t *p_temp_head_loop);


// fetch operations
void CyComputeHeadAndTailDiff( CyCircularBuffer_t *p_cyBuf,
                               uint32_t temp_head_index,
                               uint32_t temp_tail_index,
                               uint32_t *p_diff);

void CyFetchUint8( CyCircularBuffer_t *p_cyBuf,
                   uint32_t *p_temp_tail_index,
                   uint32_t *p_temp_tail_loop,
                   uint8_t *p_data);

void CyFetchUint16(CyCircularBuffer_t *p_cyBuf,
                   uint32_t *p_temp_tail_index,
                   uint32_t *p_temp_tail_loop,
                   uint16_t *p_data);

void CyFetchUint32(CyCircularBuffer_t *p_cyBuf,
                   uint32_t *p_temp_tail_index,
                   uint32_t *p_temp_tail_loop,
                   uint32_t *p_data);

void CyFetchUint8Array(CyCircularBuffer_t *p_cyBuf,
                       uint32_t *p_temp_tail_index,
                       uint32_t *p_temp_tail_loop,
                       uint32_t array_len,
                       uint8_t* p_out_buf,
                       uint32_t out_buf_len);

void CySaveTailIndexAndLoop(CyCircularBuffer_t *p_cyBuf,
                            uint32_t *p_temp_tail_index,
                            uint32_t *p_temp_tail_loop);

#ifdef __cplusplus
}
#endif

#endif // _CY_CIRCULAR_BUFFER_H_
