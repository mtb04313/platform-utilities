/******************************************************************************
* File Name:   cy_status.c
*
* Description: This is the source code for Status tracking component for the
*              project.
*
* Related Document: See Readme.md
*
*******************************************************************************/
/*******************************************************************************
* (c) 2020, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
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
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*******************************************************************************/

#include "cy_status.h"
#include "cy_memtrack_mutex.h"
#include "cy_string.h"
#include "cy_debug.h"
#include "cy_status_config.h"


/*-- Local Data -------------------------------------------------*/

static ThreadMutex_t s_mutex = NULL;

static char s_status_str[CY_STATUS_STR_MAX_LEN] = "";


/*-- Public Functions -------------------------------------------------*/

bool status_initialize(void)
{
  s_mutex = ThreadMutexCreate();
  return (s_mutex != NULL);
}

void status_destroy(void)
{
  ThreadMutexDestroy(s_mutex);
  s_mutex = NULL;

}

bool set_status_str(char *status_p)
{
  bool result = false;
  ReturnAssert(status_p != NULL, false);

  if (ThreadMutexAcquire(s_mutex)) {
    SNPRINTF(s_status_str, sizeof(s_status_str), "%s", status_p);

    ThreadMutexRelease(s_mutex);
  }

  return result;
}

const char* get_status_str(void)
{
  return s_status_str;
}


/* [] END OF FILE */
