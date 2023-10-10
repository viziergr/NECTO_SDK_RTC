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
 * @file hal_ll_rtc.c
 * @brief Rtc HAL LOW LEVEL layer implementation.
 */

#include "hal_ll_rtc.h"

#define SYSCTL_RCGCHIB_R0_BIT   0
#define HIB_IM_WC_BIT           4
#define HIB_CTL_RTCEN_BIT       0
#define HIB_CTL_CLK32EN_BIT     6
#define HIB_CTL_WRC_BIT         31
#define HIB_MIS_WC_BIT          4
#define HIB_CALCTL_CALEN_BIT    0
#define HIB_CALCTL_CAL24_BIT    2

#define WRC                     check_reg_bit( registers.hib_ctl, HIB_CTL_WRC_BIT ) == 0
#define WC                      check_reg_bit( registers.hib_mis, HIB_MIS_WC_BIT ) == 0

#define MASK_SECOND             0x0000003F
#define MASK_MINUTE             0x00003F00
#define MASK_HOUR               0x001f0000
#define MASK_DAY_MONTH          0x0000001F
#define MASK_MONTH              0x00000F00
#define MASK_YEAR               0x007f0000

#define GET_SECOND              time_second % 60
#define GET_MINUTE              time_second / 60 - GET_HOUR * 60
#define GET_HOUR                time_second / 3600

#define SET_TIME                time->hour * 3600 + time->minute * 60 + time->second

#define RESET                   0x00000000
#define LOCK_KEY                0xA3359554
#define SOFTWARE_RESET_KEY      0x05FA0004

/**
 * @brief Writes specified value to
 *        specified register.
 *
 * @param[in] reg  - register address.
 * @param[in] _val - Value to be written.
 */
#define write_reg(reg,_val) (selected_reg(reg) = (_val))

/**
 * @brief Returns value stored
 *        in a register.
 *
 * @param[in] reg  - register address.
 *
 * @return Register(reg) value.
 */
#define read_reg(reg) (selected_reg(reg))

typedef struct
{
    hal_ll_base_addr_t* sysctl_rcgchib;
    hal_ll_base_addr_t* hib_mis;
    hal_ll_base_addr_t* hib_ctl;
    hal_ll_base_addr_t* hib_im;
    hal_ll_base_addr_t* hib_rtcld;
    hal_ll_base_addr_t* nvic_apint;
    hal_ll_base_addr_t* hib_rtcc;
    hal_ll_base_addr_t* hib_lock;
} reg_t;

#ifdef __GNUC__
reg_t registers = { &SYSCTL_RCGCHIB_R, &HIB_MIS_R, &HIB_CTL_R, &HIB_IM_R, &HIB_RTCLD_R, &NVIC_APINT_R, &HIB_RTCC_R, &HIB_LOCK_R };
#else
reg_t registers = { &SYSCTL_RCGCHIB, &HIB_MIS, &HIB_CTL, &HIB_IM, &HIB_RTCLD, &NVIC_APINT, &HIB_RTCC, &HIB_LOCK };
#endif

#ifdef __GNUC__

/**
 * @brief initialize the counter.
 * @details initialize the RTC set the date to 01/01/2000 at 00:00:00.
 * @b Example
 * @code
 * // Initialise RTC
 * hal_ll_rtc_init();
 * @endcode
 * */
void __attribute__((weak)) hal_ll_rtc_init();

/**
 * @brief initialize the default value of counter.
 * @details set the RTC value at time.
 * @b Example
 * @code
 * // Time structure
 * static rtc_t time;
 *
 * // Initialise RTC
 * rtc_init();
 *
 * //Configure RTC
 * if ( hal_ll_rtc_configure_default( time ) == RTC_ERROR )
 *   {
 *       // Error handling strategy
 *   }
 * @endcode
 * */
err_t __attribute__((weak)) hal_ll_configure_default( hal_ll_rtc_t *time );

/**
 * @brief Start the RTC module.
 * @details Continue the RTC module count from the last time it has count.
 * @param none
 * @return The function can return one of the values defined by
 * rtc_err_t structure, which is size dependant on the architecture.
 * @pre Before calling this function, the user is expected
 * to have initialized RTC by using rtc_init() and have the RTC not counting.
 * @note It is recommended to have configured the RTC module using rtc_configure_default(rtc_t *time) before the first start.
 *
 * @b Example
 * @code
 *   // RTC time structure.
 *   static rtc_t time;
 *
 *   // initalize the RTC module.
 *   rtc_init();
 *
 *   // Start the RTC module.
 *   if ( hal_ll_rtc_start() == RTC_ERROR )
 *   {
 *       // Error handling strategy
 *   }
 * @endcode
 */
err_t __attribute__((weak)) hal_ll_rtc_start();

/**
 * @brief Stop the RTC module.
 * @details Stop the RTC module count from the last time it has count.
 * @param none
 * @return The function can return one of the values defined by
 * rtc_err_t structure, which is size dependant on the architecture.
 * @pre Before calling this function, the user is expected
 * to have the RTC started.
 * @b Example
 * @code
 *   // RTC time structure.
 *   static rtc_t time;
 *
 *   // initalize the RTC module.
 *   rtc_init();
 *
 *   // Start the RTC module.
 *   rtc_start();
 *
 *   // Stop the RTC module.
 *   if ( hal_ll_rtc_stop() == RTC_ERROR )
 *   {
 *       // Error handling strategy
 *   }
 * @endcode
 */
err_t __attribute__((weak)) hal_ll_rtc_stop();

/**
 * @brief Reset the RTC module.
 * @details Reset the RTC counter go back to the default values for the time and date registers.
 * @param none
 * @return The function can return one of the values defined by
 * rtc_err_t structure, which is size dependant on the architecture.
 * @pre Before calling this function, the user is expected
 * to have the RTC started.
 * @b Example
 * @code
 *   // RTC time structure.
 *   static rtc_t time;
 *
 *   // initalize the RTC module.
 *   rtc_init();
 *
 *   // Start the RTC module.
 *   rtc_start();
 *
 *   // Reset the RTC module.
 *   if ( hal_ll_rtc_reset() == RTC_ERROR )
 *   {
 *       // Error handling strategy
 *   }
 * @endcode
 */
err_t __attribute__((weak)) hal_ll_rtc_reset();

/**
 * @brief Configure time of RTC.
 * @details Allow to configure the date and time of the calendar of the rtc.
 * @return The function can return one of the values defined by
 * rtc_err_t structure, which is size dependant on the architecture.
 * @pre Before calling this function, the user is expected
 * to have the RTC enable and calendar mode activate.
 * @note It is recommended to check return value for error.
 *
 * @b Example
 * @code
 *  // Time structure
 *  static rtc_t time;
 *
 *  // Initialise RTC
 *  rtc_init();
 *
 *  // Modify time
 *  time.second = 0;
 *  time.minute = 5;
 *  time.hour = 3;
 *  time.day_week = 1;
 *  time.day_month = 2;
 *  time.month = 2;
 *  time.year = 1;
 *
 *  // Set time
 *  if ( RTC_SUCCESS != hal_ll_rtc_set_time( &time ) ) {
 *      // Error handling strategy
 *  }
 * @endcode
 * */
err_t __attribute__((weak)) hal_ll_rtc_set_time( hal_ll_rtc_t *time );

/**
 * @brief Get RTC counter.
 * @details Get the rtc counter/the time and date.
 * @return the date/counter.
 * @pre Before calling this function, the user is expected
 * to have the RTC enable.
 * @note It is recommended to check return value for error.
 *
 * @b Example
 * @code
 *  // Time structure
 *  static rtc_t time;
 *
 *  // Initialise RTC
 *  rtc_init();
 *
 *  // Start RTC;
 *  if ( RTC_SUCCESS != rtc_start() ) {
 *      // Error handling strategy
 *  }
 *
 *  if ( RTC_SUCCESS != hal_ll_rtc_get_time( &time ) ) {
 *      // Error handling strategy
 *  }
 *  @endcode
 * */
err_t __attribute__((weak)) hal_ll_rtc_get_time( hal_ll_rtc_t *time );

/**
 * @brief Reset the microcontoller
 * @details Resets the core and all on-chip peripherals except the Debug
 * interface
 * @note It is recommended to check return value for error.
 *
 * @b Example
 * @code
 *  // Time structure
 *  static rtc_t time;
 *
 *  // Initialise RTC
 *  rtc_init();
 *
 *  // Reset
 *  hal_ll_software_reset();
 *  @endcode
 * */
void __attribute__((weak)) hal_ll_software_reset();

#endif

void hal_ll_rtc_init() {
    set_reg_bit( registers.sysctl_rcgchib, SYSCTL_RCGCHIB_R0_BIT );
    if ( !check_reg_bit( registers.hib_ctl, HIB_CTL_CLK32EN ) ) {
        set_reg_bit( registers.hib_im, HIB_IM_WC_BIT );
        while ( WRC );
        set_reg_bit( registers.hib_ctl, HIB_CTL_CLK32EN_BIT );
        while ( WC );
    }
}

err_t hal_ll_configure_default( hal_ll_rtc_t *time ) {
    if ( !check_reg_bit( registers.hib_ctl, HIB_CTL_CLK32EN_BIT ) ) {
        return RTC_ERROR;
    }
    if ( registers.hib_lock ) {
        write_reg( registers.hib_lock, LOCK_KEY );
    }
    while ( WRC );
    write_reg( registers.hib_rtcld, SET_TIME );
    while ( WRC );
    if ( read_reg( registers.hib_rtcc ) != SET_TIME ) {
        return RTC_ERROR;
    }
    return RTC_SUCCESS;
}

err_t hal_ll_rtc_start() {
    while ( WRC );
    set_reg_bit( registers.hib_ctl, HIB_CTL_RTCEN_BIT );
    while ( WRC );
    if ( check_reg_bit( registers.hib_ctl, HIB_CTL_RTCEN_BIT ) ) {
        return RTC_SUCCESS;
    }
    return RTC_ERROR;
}

err_t hal_ll_rtc_stop() {
    while ( WRC );
    clear_reg_bit( registers.hib_ctl, HIB_CTL_RTCEN_BIT );
    while ( WRC );
    if ( check_reg_bit( registers.hib_ctl, HIB_CTL_RTCEN_BIT ) ) {
        return RTC_ERROR;
    }
    return RTC_SUCCESS;
}

err_t hal_ll_rtc_reset() {
    if ( !check_reg_bit( registers.hib_ctl, HIB_CTL_CLK32EN_BIT ) ) {
        return RTC_ERROR;
    }
    if ( RTC_ERROR == hal_ll_rtc_stop() ) {
        return RTC_ERROR;
    }
    if ( registers.hib_lock ) {
        write_reg( registers.hib_lock, LOCK_KEY );
    }
    while ( WRC );
    write_reg( registers.hib_rtcld, RESET );
    if ( RTC_ERROR == hal_ll_rtc_start() ) {
        return RTC_ERROR;
    }
    return RTC_SUCCESS;
}

err_t hal_ll_rtc_set_time( hal_ll_rtc_t *time ) {
    if ( !check_reg_bit( registers.hib_ctl, HIB_CTL_CLK32EN_BIT ) ) {
        return RTC_ERROR;
    }
    if ( RTC_ERROR == hal_ll_rtc_stop() ) {
        return RTC_ERROR;
    }
    if ( registers.hib_lock ) {
        write_reg( registers.hib_lock, LOCK_KEY );
    }
    while ( WRC );
    write_reg( registers.hib_rtcld, SET_TIME );
    while ( WRC );
    if ( read_reg( registers.hib_rtcc ) != SET_TIME ) {
        return RTC_ERROR;
    }
    if ( RTC_ERROR == hal_ll_rtc_start() ) {
        return RTC_ERROR;
    }
    return RTC_SUCCESS;
}

err_t hal_ll_rtc_get_time( hal_ll_rtc_t *time ) {
    uint16_t time_second = read_reg( registers.hib_rtcc );
    time->second = GET_SECOND;
    time->minute = GET_MINUTE;
    time->hour = GET_HOUR;
    return RTC_SUCCESS;
}

void hal_ll_software_reset() {
    write_reg( registers.nvic_apint, SOFTWARE_RESET_KEY );
}
