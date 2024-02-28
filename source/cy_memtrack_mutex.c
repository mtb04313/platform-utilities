/******************************************************************************
* File Name:   cy_memtrack_mutex.c
*
* Description: This file implements a mutex without relying on abstraction-rtos,
*              so that cy_memtrack can be used to track abstraction-rtos memory
*              usage.
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

#include "cy_memtrack_mutex.h"
#include "cy_debug.h"

#if defined(CY_RTOS_AWARE)
#include "cyabs_rtos.h"
#endif

#define USE_ABSTRACTION_RTOS        0   // 1:enable, 0:disable


/*-- Public Functions -------------------------------------------------*/

ThreadMutex_t ThreadMutexCreate(void)
{
#if USE_ABSTRACTION_RTOS
    cy_mutex_t *mutex = malloc(sizeof(*mutex));

    if (mutex != NULL) {
        if (cy_rtos_init_mutex2(mutex, false) == CY_RSLT_SUCCESS) {
            return mutex;

        } else {
            free(mutex);
            mutex = NULL;
        }
        return mutex;
    }
    return NULL;

#else
    uint8_t *mutex = malloc(sizeof(*mutex));
    if (mutex != NULL) {
        *mutex = 0;
    }

    return (ThreadMutex_t) mutex;

#endif
}

void ThreadMutexDestroy(ThreadMutex_t mutex)
{
#if USE_ABSTRACTION_RTOS
    if (mutex != NULL) {
        cy_rtos_deinit_mutex((cy_mutex_t *)mutex);
        free(mutex);
    }

#else

    if (mutex != NULL) {
        free(mutex);
    }

#endif
}

bool ThreadMutexAcquire(ThreadMutex_t mutex)
{
    ReturnAssert(mutex != NULL, false);

#if USE_ABSTRACTION_RTOS
    return (cy_rtos_get_mutex((cy_mutex_t *)mutex, CY_RTOS_NEVER_TIMEOUT)
            == CY_RSLT_SUCCESS);

#elif defined COMPONENT_FREERTOS

    // wait for value to become 0
    while (*(uint8_t*)mutex);

    taskENTER_CRITICAL();
    *(uint8_t*)mutex = 1;
    taskEXIT_CRITICAL();

    return *(uint8_t*)mutex;

#elif defined COMPONENT_RTTHREAD

    // wait for value to become 0
    while (*(uint8_t*)mutex);

    rt_enter_critical();
    *(uint8_t*)mutex = 1;
    rt_exit_critical();

    return *(uint8_t*)mutex;

#else
    return false;
#endif
}

void ThreadMutexRelease(ThreadMutex_t mutex)
{
    VoidAssert(mutex != NULL);

#if USE_ABSTRACTION_RTOS

    cy_rtos_set_mutex((cy_mutex_t *)mutex);

#elif defined COMPONENT_FREERTOS

    taskENTER_CRITICAL();
    *(uint8_t*)mutex = 0;
    taskEXIT_CRITICAL();

#elif defined COMPONENT_RTTHREAD

    rt_enter_critical();
    *(uint8_t*)mutex = 0;
    rt_exit_critical();

#endif
}
