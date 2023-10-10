/****************************************************************************
**
** Copyright (C) 2023 MikroElektronika d.o.o.
** Contact: https://www.mikroe.com/contact
**
** This file is part of the mikroSDK package
**
** Commercial License Usage
**
** Licensees holding valid commercial NECTO compilers AI licenses may use this
** file in accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The MikroElektronika Company.
** For licensing terms and conditions see
** https://www.mikroe.com/legal/software-license-agreement.
** For further information use the contact form at
** https://www.mikroe.com/contact.
**
**
** GNU Lesser General Public License Usage
**
** Alternatively, this file may be used for
** non-commercial projects under the terms of the GNU Lesser
** General Public License version 3 as published by the Free Software
** Foundation: https://www.gnu.org/licenses/lgpl-3.0.html.
**
** The above copyright notice and this permission notice shall be
** included in all copies or substantial portions of the Software.
**
** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
** OF MERCHANTABILITY, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
** TO THE WARRANTIES FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
** IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
** DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT
** OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
** OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
**
****************************************************************************/
/*!
 * @file hal_ll_rtc.h
 * @brief Rtc TARGETS layer implementation.
 */
#ifndef _HAL_LL_RTC_H_
#define _HAL_LL_RTC_H_

#ifdef __cplusplus
extern "C"{
#endif
    
#include "stdint.h"
#include "hal_target.h"
    
/*!< @brief Mask used for register clearing */
#define HAL_LL_CLEAR 0x00U

/**
 * @brief Writes specified value to
 *        specified register.
 *
 * @param[in] reg  - register address.
 * @param[in] _val - Value to be written.
 */
#define write_reg(reg,_val) (selected_reg(reg) = (_val))

/**
 * @brief Clears all bits in a register.
 *
 * @param[in] reg  - register address
 */
#define clear_reg(reg) write_reg(reg, HAL_LL_CLEAR)

/**
 * @brief Returns value stored
 *        in a register.
 *
 * @param[in] reg  - register address.
 *
 * @return Register(reg) value.
 */
#define read_reg(reg) (selected_reg(reg))

/**
 * @brief RTC time structure, consisted of the following fields :
 * @details User needs to specify values into the structure.
 * @note Take into consideration that any value can be changed explicitly by the user.
 * @warning The following example includes time registers for the RTC.
 * Take into consideration that some MCU do not have time registers.
 * Default values are unprecise if you do not have time registers.
 *
 * @b Example
 * @code
 *   // Define RTC Time structure.
 *   static rtc_t time;
 *
 *   // Fill structure with default values.
 *   #define time_default_value  time.second = 0; \
 *                               time.minute = 5; \
 *                               time.hour = 0; \
 *                               time.day_week = 1; \
 *                               time.day_month = 1; \
 *                               time.month = 1; \
 *                               time.year = 0;
 *
 *   // Use the structure :
 *   rtc_configure_default ( &time )
 *
 * @endcode
 */
typedef struct
{
    uint16_t year;           /*!< Time for year. */
    uint16_t month;          /*!< Time for month. */
    uint16_t day_month;      /*!< Time for month. */
    uint16_t day_week;       /*!< Time for week. */
    uint16_t hour;           /*!< Time for hour. */
    uint16_t minute;         /*!< Time for minute. */
    uint32_t second;         /*!< Time for second. */
} hal_ll_rtc_t;


typedef enum
{
    RTC_SUCCESS = 0,    /*!< Success. */
    RTC_ERROR = (-1)    /*!< Error. */
} hal_ll_rtc_err_t;

/**
 * @brief initialize the counter.
 * @details initialize the RTC set the date to 01/01/2000 at 00:00:00.
 * @b Example
 * @code
 * // Initialise RTC.
 * rtc_init();
 * @endcode
 * */
void  hal_ll_rtc_init();

/**
 * @brief initialize the default value of counter.
 * @details set the RTC value at time.
 * @b Example
 * @code
 * // Time structure.
 * static rtc_t time;
 * 
 * // Initialise RTC.
 * rtc_init();
 * 
 * // Configure RTC.
 * configure_default( time );
 * @endcode
 * */
err_t  hal_ll_configure_default( hal_ll_rtc_t *time );

/**
 * @brief Enable counter.
 * @details Start RTC module count.
 * @return The function can return one of the values defined by
 * hal_ll_rtc_err_t, which is size dependant on the architecture.
 * @pre Before calling this function, the user is expected
 * to configure RTC by using #rtc_configure_default function.
 * */


err_t  hal_ll_rtc_start();

/**
 * @brief Disable counter.
 * @details Stop RTC module count.
 * @return The function can return one of the values defined by
 * hal_ll_rtc_err_t, which is size dependant on the architecture.
 * @pre Before calling this function, the user is expected
 * to have the RTC started.
 * */
err_t hal_ll_rtc_stop();

/**
 * @brief Configure time of RTC.
 * @details Allow to configure the date and time of the calendar of the rtc.
 * @return The function can return one of the values defined by
 * hal_ll_rtc_err_t, which is size dependant on the architecture.
 * @pre Before calling this function, the user is expected
 * to have the RTC enable and calendar mode activate.
 * */
err_t  hal_ll_rtc_reset();

/**
 * @brief Reset the microcontoller.
 * @details Resets the core and all on-chip peripherals except the Debug
 * interface.
 * */
err_t  hal_ll_rtc_set_time( hal_ll_rtc_t *time );

/**
 * @brief Get RTC counter.
 * @details Get the rtc counter/the time and date. 
 * @return The function can return one of the values defined by
 * hal_ll_rtc_err_t, which is size dependant on the architecture.
 * @pre Before calling this function, the user is expected
 * to have the RTC enable.
 * */
err_t  hal_ll_rtc_get_time( hal_ll_rtc_t *time );

/**
 * @brief Reset the counter.
 * @details Make the rtc counter go back at his satrt value
 * @return The function can return one of the values defined by
 * hal_ll_rtc_err_t structure, which is size dependant on the architecture.
 * @pre Before calling this function, the user is expected
 * to have the RTC enable.
 * */
void  hal_ll_software_reset();

/**
 * @brief Configure the watchdog.
 * @details Conifguration in order to do a software reset.
 * */
void  configureWatchdog();

#ifdef __cplusplus
}
#endif

#endif // _HAL_LL_RTC_H_
// ------------------------------------------------------------------------- END
