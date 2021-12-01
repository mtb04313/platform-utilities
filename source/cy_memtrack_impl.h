/******************************************************************************
* File Name:   cy_memtrack_impl.h
*
* Description: This file is the public interface of cy_memtrack_impl.c
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

#ifndef SOURCE_CY_MEMTRACK_IMPL_H_
#define SOURCE_CY_MEMTRACK_IMPL_H_

#include <memory.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


/*-- Public Functions -------------------------------------------------*/

/* must call once to initialize module at system boot time */
void memtrack_initialize(void);

void* memtrack_malloc(  size_t size,
                        const char* file,
                        size_t line_num);

void memtrack_free( void* mem,
                    const char* file,
                    size_t line_num);

void memtrack_malloc_stats(void);

void* memtrack_realloc( void* mem_ptr,
                        size_t new_size,
                        const char* file,
                        size_t line_num);

void *memtrack_calloc(  size_t number,
                        size_t size,
                        const char* file,
                        size_t line_num);

void memtrack_malloc_stats_ex(  size_t *pMaxUsed,
                                size_t *pUsed);

void memtrack_destroy(void);

#ifdef __cplusplus
};  /* end of extern "C" */
#endif /* __cplusplus */

#endif // SOURCE_CY_MEMTRACK_IMPL_H_
