/******************************************************************************
* File Name:   cy_psoc_util.c
*
* Description: This file implements PSoC utility functions.
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

#include "cy_psoc_util.h"
#include "cy_debug.h"
#include "cyhal.h"

/*-- Public Functions -------------------------------------------------*/

void PsocHandleError(void)
{
     /* Disable all interrupts. */
    __disable_irq();

    DEBUG_ASSERT(0);
    CY_ASSERT(0);
}

const char* PsocGetCyResultType(uint8_t type)
{
    switch(type) {
    case CY_RSLT_TYPE_INFO:
        return "INFO";
    case CY_RSLT_TYPE_WARNING:
        return "WARNING";
    case CY_RSLT_TYPE_ERROR:
        return "ERROR";
    case CY_RSLT_TYPE_FATAL:
        return "FATAL";
    default:
        DEBUG_ASSERT(0);  // unexpected
        break;
    }

    return "TYPE_UNKNOWN";
}

const char* PsocGetCyResultModule(uint16_t module)
{
    switch(module) {
    case CY_RSLT_MODULE_DRIVERS_PDL_BASE:
        return "DRIVERS_PDL_BASE";
    case CY_RSLT_MODULE_DRIVERS_WHD_BASE:
        return "DRIVERS_WHD_BASE";
    case CY_RSLT_MODULE_ABSTRACTION_HAL:
        return "ABSTRACTION_HAL";
    case CY_RSLT_MODULE_ABSTRACTION_BSP:
        return "ABSTRACTION_BSP";
    case CY_RSLT_MODULE_ABSTRACTION_FS:
        return "ABSTRACTION_FS";
    case CY_RSLT_MODULE_ABSTRACTION_RESOURCE:
        return "ABSTRACTION_RESOURCE";
    case CY_RSLT_MODULE_ABSTRACTION_OS:
        return "ABSTRACTION_OS";
    case CY_RSLT_MODULE_ABSTRACTION_ENV:
        return "ABSTRACTION_ENV";
    case CY_RSLT_MODULE_BOARD_LIB_RETARGET_IO:
        return "BOARD_LIB_RETARGET_IO";
    case CY_RSLT_MODULE_BOARD_LIB_RGB_LED:
        return "BOARD_LIB_RGB_LED";
    case CY_RSLT_MODULE_BOARD_LIB_SERIAL_FLASH:
        return "BOARD_LIB_SERIAL_FLASH";
    case CY_RSLT_MODULE_BOARD_LIB_WHD_INTEGRATION:
        return "BOARD_LIB_WHD_INTEGRATION";
    case CY_RSLT_MODULE_BOARD_SHIELD_028_EPD:
        return "BOARD_SHIELD_028_EPD";
    case CY_RSLT_MODULE_BOARD_SHIELD_028_TFT:
        return "BOARD_SHIELD_028_TFT";
    case CY_RSLT_MODULE_BOARD_SHIELD_032:
        return "BOARD_SHIELD_032";
    case CY_RSLT_MODULE_BOARD_HARDWARE_BMI160:
        return "BOARD_HARDWARE_BMI160";
    case CY_RSLT_MODULE_BOARD_HARDWARE_E2271CS021:
        return "BOARD_HARDWARE_E2271CS021";
    case CY_RSLT_MODULE_BOARD_HARDWARE_THERMISTOR:
       return "BOARD_HARDWARE_THERMISTOR";
    case CY_RSLT_MODULE_BOARD_HARDWARE_SSD1306:
        return "BOARD_HARDWARE_SSD1306";
    case CY_RSLT_MODULE_BOARD_HARDWARE_ST7789V:
        return "BOARD_HARDWARE_ST7789V";
    case CY_RSLT_MODULE_BOARD_HARDWARE_LIGHT_SENSOR:
        return "BOARD_HARDWARE_LIGHT_SENSOR";
    case CY_RSLT_MODULE_BOARD_HARDWARE_AK4954A:
        return "BOARD_HARDWARE_AK4954A";
    case CY_RSLT_MODULE_MIDDLEWARE_BASE:
        return "MIDDLEWARE_BASE";
    //case CY_SCB_ID:
    //  return "SCB_ID";

    default:
        //DEBUG_ASSERT(0);  // unexpected
        break;
    }

    return "MODULE_UNKNOWN";
}

const char* PsocGetI2cStatusCode(uint32_t code)
{
    switch(code) {
    case CY_SCB_I2C_SUCCESS:
        return "I2C_SUCCESS";
    case CY_SCB_I2C_BAD_PARAM:
        return "I2C_BAD_PARAM";
    case CY_SCB_I2C_MASTER_NOT_READY:
        return "I2C_MASTER_NOT_READY";
    case CY_SCB_I2C_MASTER_MANUAL_TIMEOUT:
        return "I2C_MASTER_MANUAL_TIMEOUT";
    case CY_SCB_I2C_MASTER_MANUAL_ADDR_NAK:
        return "I2C_MASTER_MANUAL_ADDR_NAK";
    case CY_SCB_I2C_MASTER_MANUAL_NAK:
        return "I2C_MASTER_MANUAL_NAK";
    case CY_SCB_I2C_MASTER_MANUAL_ARB_LOST:
        return "I2C_MASTER_MANUAL_ARB_LOST";
    case CY_SCB_I2C_MASTER_MANUAL_BUS_ERR:
        return "I2C_MASTER_MANUAL_BUS_ERR";
    case CY_SCB_I2C_MASTER_MANUAL_ABORT_START:
        return "I2C_MASTER_MANUAL_ABORT_START";

    default:
        //DEBUG_ASSERT(0);  // unexpected
        break;
    }

    return "I2C_CODE_UNKNOWN";
}

void PsocPrintCyResult(cy_rslt_t result)
{
    uint8_t type = CY_RSLT_GET_TYPE(result);
    uint16_t module = CY_RSLT_GET_MODULE(result);
    uint16_t code = CY_RSLT_GET_CODE(result);
  
    DEBUG_PRINT(("type=0x%02x (%s), module=0x%04x (%s), code=0x%04x\n",
        type, PsocGetCyResultType(type), module, PsocGetCyResultModule(module), code));
}

