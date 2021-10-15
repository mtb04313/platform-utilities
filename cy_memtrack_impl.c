/******************************************************************************
* File Name:   cy_memtrack_impl.c
*
* Description: This file implements the memory tracking functions.
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
#include <stdio.h>

#include "cy_memtrack_impl.h"
#include "cy_misc_types.h"
#include "cy_debug.h"
#include "cy_string.h"


/*-- Local Definitions -------------------------------------------------*/

#define USE_THREAD_MUTEX  1 /* 1=enable, 0=disable */

#if (USE_THREAD_MUTEX == 1)
#include "cy_memtrack_mutex.h"
#else
#include "cyabs_rtos.h"
#endif

#define LOG_TO_FILE       0 /* 1=enable, 0=disable */
#define CHECK_FOR_LEAKS   1 /* 1=enable, 0=disable */

#define LOG_FILE_NAME     "cy_memtrack.csv"
#define TAG_MALLOC        'M'
#define TAG_CALLOC        'C'
#define TAG_REALLOC       'R'
#define TAG_FREE          'F'
#define IGNORED           -1
#define NOT_FOUND         -1

#define NOT_CHECKED       0
#define CHECKED           1

#if (CHECK_FOR_LEAKS == 1)
#define MEM_INFO_CHUNK_SIZE   10000

typedef uint16_t MYSIZE;

typedef struct
__attribute__((__packed__))
{
    void *mem_p;            /* F=ignored; Others=mem_p */
    void *realloc_prev_mem_p; /* R=prev_mem_p; Others=ignored */
    MYSIZE size;            /* M=size; R=new_size; C=num; F=ignored */
    MYSIZE calloc_size;     /* C=size; Others=ignored */
    char tag;               /* M, R, C or F */
    unsigned char checked;  /* 1=checked 0=not checked */
    const char *file;       /* function or file name */
    MYSIZE line_num;        /* line number */
}
Mem_Info_t;
#endif


/*-- Local Data -------------------------------------------------*/

/* mutex protects the static variables */
#if (USE_THREAD_MUTEX == 1)
static ThreadMutex_t s_mutex = NULL;
#else
static cy_mutex_t s_mutex;
#endif

#if (LOG_TO_FILE == 1)
static FILE *s_file_p = NULL;
static char s_file_buf[256];
#endif

#if (CHECK_FOR_LEAKS == 1)
static Mem_Info_t *s_mem_info_p = NULL;
static size_t s_max_index = 0;
static size_t s_index = 0;
#endif


/*-- Local Functions -------------------------------------------------*/

#if (CHECK_FOR_LEAKS == 1)
static void save_mem_info(void *mem_p,
                          void *realloc_prev_mem_p,
                          size_t size,
                          size_t calloc_size,
                          char tag,
                          const char *file,
                          size_t line_num)
{
    if (s_index == s_max_index) {
        size_t resized_index = s_max_index + MEM_INFO_CHUNK_SIZE;
        Mem_Info_t* temp;

        temp = (Mem_Info_t*)realloc(s_mem_info_p, resized_index * sizeof(Mem_Info_t));
        if (temp == NULL) {
            DEBUG_PRINT(("%s [%d]: resized_index = %" F_ZU "\n",
                         __FUNCTION__, __LINE__, resized_index));
            VoidAssert(temp != NULL); /* if assertion fails, your device has run out of memory! */
        }

        s_mem_info_p = temp;
        s_max_index = resized_index;
    }

    DEBUG_ASSERT(s_index < s_max_index);

    s_mem_info_p[s_index].mem_p = mem_p;
    s_mem_info_p[s_index].realloc_prev_mem_p = realloc_prev_mem_p;
    s_mem_info_p[s_index].size = size;
    s_mem_info_p[s_index].calloc_size = calloc_size;
    s_mem_info_p[s_index].tag = tag;
    s_mem_info_p[s_index].file = file;
    s_mem_info_p[s_index].line_num = line_num;

    s_mem_info_p[s_index].checked = NOT_CHECKED;
    s_index++;
}

/* find its previous most-recent allocated size */
static size_t find_and_check_mem_info(  char tag,
                                        void *mem_p,
                                        size_t end_index)
{
    int i = end_index - 1;
    DEBUG_ASSERT(end_index > 0);
    DEBUG_ASSERT((tag != TAG_MALLOC) && (tag != TAG_CALLOC));

    /* search backwards */
    while (i >= 0) {
        if (s_mem_info_p[i].mem_p == mem_p) {
            if (s_mem_info_p[i].tag == TAG_FREE) {
                DEBUG_PRINT(("%s [%" F_ZU "]\n", s_mem_info_p[i].file, s_mem_info_p[i].line_num));
                DEBUG_ASSERT(s_mem_info_p[i].tag != TAG_FREE);
            }

            s_mem_info_p[i].checked = CHECKED;
            return (s_mem_info_p[i].tag == TAG_CALLOC)?
                   (s_mem_info_p[i].size * s_mem_info_p[i].calloc_size) : s_mem_info_p[i].size;
        }
        i--;
    }
    return (size_t)NOT_FOUND;
}

#if 0
static void show_mem_usage(void)
{
    size_t i;
    for (i = 0; i < s_index; i++) {
        DEBUG_PRINT(("%p, %p, %" F_ZU ", %" F_ZU ", %c, %u, %s, %" F_ZU "\n",
                     s_mem_info_p[i].mem_p,
                     s_mem_info_p[i].realloc_prev_mem_p,
                     s_mem_info_p[i].size,
                     s_mem_info_p[i].calloc_size,
                     s_mem_info_p[i].tag,
                     s_mem_info_p[i].checked,
                     s_mem_info_p[i].file,
                     s_mem_info_p[i].line_num));
    }
}
#endif

static void analyze_mem_usage(size_t *pMaxUsed,
                              size_t *pUsed)
{
    size_t maxUsed = 0;
    size_t used = 0;
    size_t i;

    //show_mem_usage();

    for (i = 0; i < s_index; i++) {
        switch (s_mem_info_p[i].tag) {
        case TAG_MALLOC:
            used += s_mem_info_p[i].size;
            break;

        case TAG_CALLOC:
            used += (s_mem_info_p[i].size * s_mem_info_p[i].calloc_size);
            break;

        case TAG_REALLOC:
            if (s_mem_info_p[i].realloc_prev_mem_p == NULL) {
                used += s_mem_info_p[i].size;
            } else {
                /* find its previous most-recent allocated size */
                size_t prev_size = find_and_check_mem_info(s_mem_info_p[i].tag, s_mem_info_p[i].realloc_prev_mem_p, i);
                DEBUG_ASSERT(prev_size != (size_t)NOT_FOUND);

                DEBUG_ASSERT(used >= prev_size);
                used += (s_mem_info_p[i].size - prev_size);
            }
            break;

        case TAG_FREE: {
            /* find its previous most-recent allocated size */
            size_t prev_size = find_and_check_mem_info(s_mem_info_p[i].tag, s_mem_info_p[i].mem_p, i);
            if (prev_size == (size_t)NOT_FOUND) {
                DEBUG_PRINT(("prev_size not found: %s [%" F_ZU "]\n", s_mem_info_p[i].file, s_mem_info_p[i].line_num));
                DEBUG_ASSERT(prev_size != (size_t)NOT_FOUND);
            }

            DEBUG_ASSERT(used >= prev_size);
            used -= prev_size;

            s_mem_info_p[i].size = prev_size;
            s_mem_info_p[i].checked = CHECKED;
            break;
        }

        default:
            DEBUG_PRINT(("%s [%d]: i=%" F_ZU ", s_index=%" F_ZU ", unexpected_tag=%c\n",
                         __FUNCTION__, __LINE__, i, s_index, s_mem_info_p[i].tag));
            DEBUG_ASSERT(0);  /* unexpected tag */
            break;
        }

        if (used > maxUsed) {
            maxUsed = used;
        }
    }

    if (used != 0) {
        DEBUG_PRINT(("\n--- MEMTRACK: %" F_ZU " bytes ---\n", used));
    }

    /* identify leak sources */
    for (i = 0; i < s_index; i++) {

        if (s_mem_info_p[i].checked == NOT_CHECKED) {
            if (s_mem_info_p[i].tag == TAG_FREE) {
                /* unexpected freeing of mem - could be coding error */
                DEBUG_PRINT(("Unexpected FREE: %p, %s, %" F_ZU "\n",
                             s_mem_info_p[i].mem_p,
                             s_mem_info_p[i].file,
                             s_mem_info_p[i].line_num));
            } else {
                /* mem not freed */
                size_t leak_size = (s_mem_info_p[i].tag == TAG_CALLOC)?
                                   s_mem_info_p[i].size * s_mem_info_p[i].calloc_size : s_mem_info_p[i].size;

                DEBUG_PRINT(("%c (%" F_ZU " bytes): %s, %" F_ZU "\n",
                             s_mem_info_p[i].tag,
                             leak_size,
                             s_mem_info_p[i].file,
                             s_mem_info_p[i].line_num));

                (void)leak_size;
            }
        }
    }

    //show_mem_usage();

    *pMaxUsed = maxUsed;
    *pUsed = used;
}
#endif


/*-- Public Functions -------------------------------------------------*/

void memtrack_initialize(void)
{
#define INITIAL_INDEX_SIZE      (2 * MEM_INFO_CHUNK_SIZE)

#if (USE_THREAD_MUTEX == 1)
    s_mutex = ThreadMutexCreate();
    VoidAssert(s_mutex != NULL);

#else
    if (CY_RSLT_SUCCESS != cy_rtos_init_mutex(&s_mutex)) {
        VoidAssert(0);
    }
#endif

#if (LOG_TO_FILE == 1)
    s_file_p = fopen(LOG_FILE_NAME, "w");
    VoidAssert(s_file_p != NULL);
#endif

#if (CHECK_FOR_LEAKS == 1)
    s_max_index = INITIAL_INDEX_SIZE;
    s_mem_info_p = (Mem_Info_t*)malloc(s_max_index * sizeof(Mem_Info_t));
    //DEBUG_PRINT(("sizeof(Mem_Info_t) = %" F_ZU "\n", sizeof(Mem_Info_t)));
    VoidAssert(s_mem_info_p != NULL);

    s_index = 0;
#endif

#undef INITIAL_INDEX_SIZE
}

void memtrack_destroy(void)
{
#if (USE_THREAD_MUTEX == 1)
    if (s_mutex != NULL) {
      ThreadMutexDestroy(s_mutex);
      s_mutex = NULL;
    }
#else
    cy_rtos_deinit_mutex(&s_mutex);
#endif

#if (LOG_TO_FILE == 1)
    fclose(s_file_p);
    s_file_p = NULL;
#endif

#if (CHECK_FOR_LEAKS == 1)
    free(s_mem_info_p);
    s_mem_info_p = NULL;
    s_max_index = 0;
    s_index = 0;
#endif
}

void* memtrack_malloc(  size_t size,
                        const char* file,
                        size_t line_num)
{
    void* mem_ptr = malloc(size);

    if (mem_ptr != NULL) {

#if (USE_THREAD_MUTEX == 1)
        if (ThreadMutexAcquire(s_mutex)) {
#else
        if (cy_rtos_get_mutex(&s_mutex, CY_RTOS_NEVER_TIMEOUT) == CY_RSLT_SUCCESS) {
#endif

#if (LOG_TO_FILE == 1)
            SNPRINTF(s_file_buf, sizeof(s_file_buf), "%p, %d, %" F_ZU ", %d, %c, %s, %" F_ZU "\n",
                     mem_ptr, IGNORED, size, IGNORED, TAG_MALLOC, file, line_num);
            fwrite(s_file_buf, 1, strlen(s_file_buf), s_file_p);
#endif

#if (CHECK_FOR_LEAKS == 1)
            save_mem_info(mem_ptr,
                          (void*)IGNORED,
                          size,
                          IGNORED,
                          TAG_MALLOC,
                          file,
                          line_num);
#endif

#if (USE_THREAD_MUTEX == 1)
            ThreadMutexRelease(s_mutex);
#else
            cy_rtos_set_mutex(&s_mutex);
#endif

        } else {
            DEBUG_ASSERT(0);
        }
    }
    return mem_ptr;
}

void memtrack_free( void* mem_ptr,
                    const char* file,
                    size_t line_num)
{
    if (mem_ptr != NULL) {
        free(mem_ptr);

#if (USE_THREAD_MUTEX == 1)
        if (ThreadMutexAcquire(s_mutex)) {
#else
        if (cy_rtos_get_mutex(&s_mutex, CY_RTOS_NEVER_TIMEOUT) == CY_RSLT_SUCCESS) {
#endif

#if (LOG_TO_FILE == 1)
            SNPRINTF(s_file_buf, sizeof(s_file_buf), "%p, %d, %d, %d, %c, %s, %" F_ZU "\n",
                     mem_ptr, IGNORED, IGNORED, IGNORED, TAG_FREE, file, line_num);
            fwrite(s_file_buf, 1, strlen(s_file_buf), s_file_p);
#endif

#if (CHECK_FOR_LEAKS == 1)
            save_mem_info(mem_ptr,
                          (void*)IGNORED,
                          IGNORED,
                          IGNORED,
                          TAG_FREE,
                          file,
                          line_num);
#endif

#if (USE_THREAD_MUTEX == 1)
            ThreadMutexRelease(s_mutex);
#else
            cy_rtos_set_mutex(&s_mutex);
#endif
        } else {
            DEBUG_ASSERT(0);
        }
    }
}

void* memtrack_realloc( void* mem_ptr,
                        size_t new_size,
                        const char* file,
                        size_t line_num)
{
    void* new_mem_ptr = realloc(mem_ptr, new_size);

    if (new_mem_ptr != NULL) {

#if (USE_THREAD_MUTEX == 1)
        if (ThreadMutexAcquire(s_mutex)) {
#else
        if (cy_rtos_get_mutex(&s_mutex, CY_RTOS_NEVER_TIMEOUT) == CY_RSLT_SUCCESS) {
#endif

#if (LOG_TO_FILE == 1)
            SNPRINTF(s_file_buf, sizeof(s_file_buf), "%p, %p, %" F_ZU ", %d, %c, %s, %" F_ZU "\n",
                     new_mem_ptr, mem_ptr, new_size, IGNORED, TAG_REALLOC, file, line_num);

            fwrite(s_file_buf, 1, strlen(s_file_buf), s_file_p);
#endif

#if (CHECK_FOR_LEAKS == 1)
            save_mem_info(new_mem_ptr,
                          mem_ptr,
                          new_size,
                          IGNORED,
                          TAG_REALLOC,
                          file,
                          line_num);
#endif

#if (USE_THREAD_MUTEX == 1)
            ThreadMutexRelease(s_mutex);
#else
            cy_rtos_set_mutex(&s_mutex);
#endif
        } else {
            DEBUG_ASSERT(0);
        }
    }

    return new_mem_ptr;
}

void *memtrack_calloc(  size_t number,
                        size_t size,
                        const char* file,
                        size_t line_num)
{
    void* mem_ptr = calloc(number, size);

    if (mem_ptr != NULL) {

#if (USE_THREAD_MUTEX == 1)
        if (ThreadMutexAcquire(s_mutex)) {
#else
        if (cy_rtos_get_mutex(&s_mutex, CY_RTOS_NEVER_TIMEOUT) == CY_RSLT_SUCCESS) {
#endif

#if (LOG_TO_FILE == 1)
            SNPRINTF(s_file_buf, sizeof(s_file_buf), "%p, %d, %" F_ZU ", %" F_ZU ", %c, %s, %" F_ZU "\n",
                     mem_ptr, IGNORED, number, size, TAG_CALLOC, file, line_num);
            fwrite(s_file_buf, 1, strlen(s_file_buf), s_file_p);
#endif

#if (CHECK_FOR_LEAKS == 1)
            save_mem_info(mem_ptr,
                          (void*)IGNORED,
                          number,
                          size,
                          TAG_CALLOC,
                          file,
                          line_num);
#endif

#if (USE_THREAD_MUTEX == 1)
            ThreadMutexRelease(s_mutex);
#else
            cy_rtos_set_mutex(&s_mutex);
#endif
        } else {
            DEBUG_ASSERT(0);
        }
    }
    return mem_ptr;
}

void memtrack_malloc_stats(void)
{
#if (CHECK_FOR_LEAKS == 1)
    size_t maxUsed = 0;
    size_t used = 0;

    memtrack_malloc_stats_ex(  &maxUsed,
                               &used);

    DEBUG_ASSERT(s_max_index != 0);

    DEBUG_PRINT(("\n"));
    DEBUG_PRINT(("ram Used    = %10" F_ZU "\n", maxUsed));

    DEBUG_PRINT(("index Used  = %10" F_ZU " (%" F_ZU "%%)\n", s_index, s_index*100/s_max_index));
    DEBUG_PRINT(("index Avail = %10" F_ZU "\n\n", s_max_index));
#endif
}

void memtrack_malloc_stats_ex(  size_t *pMaxUsed,
                                size_t *pUsed)
{
#if (CHECK_FOR_LEAKS == 1)
    analyze_mem_usage(pMaxUsed, pUsed);
#endif
}
