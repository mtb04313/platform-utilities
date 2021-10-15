/******************************************************************************
* File Name:   cy_memtrack.h
*
* Description: This file is the public interface of the memory tracking
*              feature.
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

#ifndef SOURCE_CY_MEMTRACK_H_
#define SOURCE_CY_MEMTRACK_H_

#include "memtrack_config.h"    // for USE_CY_MEMTRACK

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


/*-- Public Definitions -------------------------------------------------*/

#if (USE_CY_MEMTRACK)
#include "cy_memtrack_impl.h"

#define CY_MEMTRACK_INITIALIZE()               memtrack_initialize()
#define CY_MEMTRACK_DESTROY()                  memtrack_destroy()
#define CY_MEMTRACK_MALLOC(size)               memtrack_malloc((size), __FUNCTION__, __LINE__)
#define CY_MEMTRACK_FREE(mem_ptr)              memtrack_free((mem_ptr), __FUNCTION__, __LINE__)
#define CY_MEMTRACK_REALLOC(mem_ptr, new_size) memtrack_realloc((mem_ptr), (new_size), __FUNCTION__, __LINE__)
#define CY_MEMTRACK_CALLOC(count, size)        memtrack_calloc((count), (size), __FUNCTION__, __LINE__)
#define CY_MEMTRACK_MALLOC_STATS()             memtrack_malloc_stats()
/* Note: use calloc (instead of malloc) if you want allocated memory to be initialized to zero */

#define CY_MEMTRACK_MALLOC_STATS_EX(pMaxUsed, pUsed) \
            memtrack_malloc_stats_ex((pMaxUsed), (pUsed))

#else
#include <stdlib.h>

#define CY_MEMTRACK_INITIALIZE()               ((void)0)
#define CY_MEMTRACK_DESTROY()                  ((void)0)
#define CY_MEMTRACK_MALLOC(size)               malloc(size)
#define CY_MEMTRACK_FREE(mem_ptr)              free(mem_ptr)
#define CY_MEMTRACK_REALLOC(mem_ptr, new_size) realloc((mem_ptr), (new_size))
#define CY_MEMTRACK_CALLOC(count, size)        calloc((count), (size))
#define CY_MEMTRACK_MALLOC_STATS()             ((void)0)
#define CY_MEMTRACK_MALLOC_STATS_EX()          ((void)0)
#endif

/*
    // sample client code
    #include "cy_memtrack.h"
    void main(void)
    {
        unsigned char * p_buf;
        CY_MEMTRACK_INITIALIZE();
        p_buf = CY_MEMTRACK_MALLOC(10);

        if (p_buf)
        {
            CY_MEMTRACK_FREE(p_buf);
        }
        CY_MEMTRACK_DESTROY();
    }
*/


#ifdef __cplusplus
};  /* end of extern "C" */
#endif /* __cplusplus */

#endif // SOURCE_CY_MEMTRACK_H_
