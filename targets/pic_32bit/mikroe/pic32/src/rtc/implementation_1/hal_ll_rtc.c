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
 * @file  hal_ll_rtc.c
 * @brief RTC HAL LOW LEVEL layer implementation.
 */

#include "hal_ll_rtc.h"

// ---------------------------------------------------------------- PRIVATE MACROS

/*!< @brief MACROS for the code */
#define DEFAULT_TIME               0x00000000
#define DEFAULT_DATE               0x00010107
#define FIRST_KEY                  0xaa996655
#define SECOND_KEY                 0x556699aa
#define ENABLE_WRITE               0x00000008
#define DISABLE_RTC_INTERRUPTION   0x00008000
#define SET_RTCC_OFF               0x00008000
#define SET_RTCC_ON                0x00008000
#define CLOCK_OFF                  0x00000040
#define CLEAR_RTCC_EVENT           0x00008000
#define CLEAR_PRIORITY             0x1f000000
#define CLEAR_RTCALRM              0x0000CFFF
#define RTC_ON                     read_reg(registers.rtc_con) & CLOCK_OFF
#define RTC_OFF                    !(read_reg(registers.rtc_con) & CLOCK_OFF)
#define UNREADABLE_REGISTER        !check_reg_bit(registers.rtc_con, B2)
#define MODIFIED_REGISTER          (read_reg(registers.rtc_time) == new_time) & (read_reg(registers.rtc_date) == new_date)
#define DEFAULT_REGISTER           (read_reg(registers.rtc_time) == DEFAULT_TIME) & (read_reg(registers.rtc_date) == DEFAULT_DATE)
#define CLEAR                      0
#define BIT0                       0
#define BIT1                       1
#define BIT22                      22
#define TWO_CENTURIES              2000

/*!< @brief RTCTIME REGISTER MASKS */
#define RTCTIME_MASK_HR10          0x30000000
#define RTCTIME_MASK_HR01          0x0f000000
#define RTCTIME_MASK_MIN10         0x00700000
#define RTCTIME_MASK_MIN01         0x000f0000
#define RTCTIME_MASK_SEC10         0x00007000
#define RTCTIME_MASK_SEC01         0x00000f00

/*!< @brief RTCDATE REGISTER MASKS */
#define RTCDATE_MASK_YEAR10        0xf0000000
#define RTCDATE_MASK_YEAR01        0x0f000000
#define RTCDATE_MASK_MONTH10       0x00100000
#define RTCDATE_MASK_MONTH01       0x000f0000
#define RTCDATE_MASK_DAY10         0x00003000
#define RTCDATE_MASK_DAY01         0x00000f00
#define RTCDATE_MASK_WDAY01        0x00000007

/*!< @brief REGISTER CONVERTER MACROS */
#define YEAR10_TO_YEAR             ((read_reg(registers.rtc_date) & RTCDATE_MASK_YEAR10) >> 28) * 10
#define YEAR01_TO_YEAR             (read_reg(registers.rtc_date) & RTCDATE_MASK_YEAR01) >> 24
#define MONTH10_TO_MONTH           ((read_reg(registers.rtc_date) & RTCDATE_MASK_MONTH10) >> 20) * 10
#define MONTH01_TO_MONTH           (read_reg(registers.rtc_date) & RTCDATE_MASK_MONTH01) >> 16
#define DAY10_TO_DAY               ((read_reg(registers.rtc_date) & RTCDATE_MASK_DAY10) >> 12) * 10
#define DAY01_TO_DAY               (read_reg(registers.rtc_date) & RTCDATE_MASK_DAY01) >> 8
#define DAY_WEEK                   read_reg(registers.rtc_date) & RTCDATE_MASK_WDAY01

#define HOUR10_TO_HOUR             ((read_reg(registers.rtc_time) & RTCTIME_MASK_HR10) >> 28) * 10
#define HOUR01_TO_HOUR             (read_reg(registers.rtc_time) & RTCTIME_MASK_HR01) >> 24
#define MINUTE10_TO_MINUTE         ((read_reg(registers.rtc_time) & RTCTIME_MASK_MIN10) >> 20) * 10
#define MINUTE01_TO_MINUTE         (read_reg(registers.rtc_time) & RTCTIME_MASK_MIN01) >> 16
#define SECOND10_TO_SECOND         ((read_reg(registers.rtc_time) & RTCTIME_MASK_SEC10) >> 12) * 10
#define SECOND01_TO_SECOND         (read_reg(registers.rtc_time) & RTCTIME_MASK_SEC01) >> 8

#define SECOND_TO_SECOND01         time->second % 10
#define SECOND_TO_SECOND10         (time->second - seconds_01) / 10
#define MINUTE_TO_MINUTE01         time->minute % 10
#define MINUTE_TO_MINUTE10         (time->minute - minutes_01) / 10
#define HOUR_TO_HOUR01             time->hour % 10
#define HOUR_TO_HOUR10             (time->hour - hours_01) / 10
#define SECOND_TO_REGISTER         ((uint32_t)seconds_01 << 8) | ((uint32_t)seconds_10 << 12)
#define MINUTE_TO_REGISTER         ((uint32_t)minutes_01 << 16) | ((uint32_t)minutes_10 << 20)
#define HOUR_TO_REGISTER           ((uint32_t)hours_01 << 24) | ((uint32_t)hours_10 << 28)

#define YEAR_TO_YEAR01             (time->year) % 10
#define YEAR_TO_YEAR10             (time->year - years_01) / 10
#define MONTH_TO_MONTH01           time->month % 10
#define MONTH_TO_MONTH10           (time->month - month_01) / 10
#define DAY_TO_DAY01               time->day_month % 10
#define DAY_TO_DAY10               (time->day_month - day_month_01) / 10
#define YEAR_TO_REGISTER           ((uint32_t)years_01 << 24) | ((uint32_t)years_10 << 28)
#define MONTH_TO_REGISTER          ((uint32_t)month_01 << 16) | ((uint32_t)month_10 << 20)
#define DAY_TO_REGISTER            (time->day_week) | ((uint32_t)day_month_01 << 8) | ((uint32_t)day_month_10 << 12)

/*!< @brief SOFTWARE RESET MACROS */
#define WAIT_RESET                 asm nop;\
                                   asm nop;\
                                   asm nop;\
                                   asm nop;

// ----------------------------------------------------------------- PRIVATE TYPES

/*!< @brief RTC register structure. */
typedef struct
{
    hal_ll_base_addr_t sys_key;
    hal_ll_base_addr_t osc_con;
    hal_ll_base_addr_t rtc_con;
    hal_ll_base_addr_t rtc_con_set;
    hal_ll_base_addr_t rtc_con_clr;
    hal_ll_base_addr_t rtc_date;
    hal_ll_base_addr_t rtc_time;
    hal_ll_base_addr_t rtc_alrm_clr;
    hal_ll_base_addr_t rswrst_set;
    hal_ll_base_addr_t rswrst;
    hal_ll_base_addr_t iec1_clr;
    hal_ll_base_addr_t ifs1_clr;
    hal_ll_base_addr_t ipc8_clr;
} reg_t;

// -------------------------------------------------------------------- VARIABLES

/*!< @brief RTC registers info  */
reg_t registers = { &SYSKEY, &OSCCON, &RTCCON, &RTCCONSET, &RTCCONCLR, &RTCDATE, &RTCTIME, &RTCALRMCLR, &RSWRSTSET, &RSWRST, &IEC1CLR, &IFS1CLR, &IPC8CLR };

// ------------------------------------------------ PRIVATE FUNCTION DECLARATIONS

/**
 * @brief create an uint32_t variable that can be used to change the time register
 * @param[in] time : RTC time structure
 * @return uint32_t variable that suits the RTC time register requirement
 * @details use the different values of the RTC time structure to create a variable that can be used to change the time register
 *
 * @b Example
 * @code
 *  // Time structure
 *  static rtc_t time;
 *
 *  // create a new variable that can be used in the time register
 *  uint32_t NEW_TIME = set_time_register(time);
 *  @endcode
 * */
static uint32_t set_time_register( hal_ll_rtc_t *time );

/**
 * @brief create an uint32_t variable that can be used to change the date register
 * @param[in] time : RTC time structure
 * @return uint32_t variable that suits the RTC date register requirement
 * @details use the different values of the RTC time structure to create a variable that can be used to change the date register
 *
 * @b Example
 * @code
 *  // Time structure
 *  static rtc_t time;
 *
 *  // create a new variable that can be used in the date register
 *  uint32_t NEW_TIME = set_time_register(time);
 *  @endcode
 * */
static uint32_t set_date_register( hal_ll_rtc_t *time );

// ------------------------------------------------ PUBLIC FUNCTION DEFINITIONS

err_t hal_ll_configure_default( hal_ll_rtc_t *time ) {

    uint32_t new_time;
    uint32_t new_date;

    // Unlock write protection.
    write_reg( registers.sys_key, CLEAR );
    write_reg( registers.sys_key, FIRST_KEY );
    write_reg( registers.sys_key, SECOND_KEY );

    if( RTC_ON )
        return RTC_ERROR;

    new_time = set_time_register( time );
    new_date = set_date_register( time );
    write_reg( registers.rtc_time, new_time );
    write_reg( registers.rtc_date, new_date );

    if ( MODIFIED_REGISTER )
        return RTC_SUCCESS;
    else
        return RTC_ERROR;
}

err_t hal_ll_rtc_get_time( hal_ll_rtc_t *rtc ) {

    while( UNREADABLE_REGISTER );

    rtc->year  = YEAR10_TO_YEAR;
    rtc->year += YEAR01_TO_YEAR;
    rtc->year += TWO_CENTURIES;

    rtc->month  = MONTH10_TO_MONTH;
    rtc->month += MONTH01_TO_MONTH;

    rtc->day_month  = DAY10_TO_DAY;
    rtc->day_month += DAY01_TO_DAY;

    rtc->day_week = DAY_WEEK;

    rtc->hour  = HOUR10_TO_HOUR;
    rtc->hour += HOUR01_TO_HOUR;

    rtc->minute  = MINUTE10_TO_MINUTE;
    rtc->minute += MINUTE01_TO_MINUTE;

    rtc->second  = SECOND10_TO_SECOND;
    rtc->second += SECOND01_TO_SECOND;

    return RTC_SUCCESS;
}

void hal_ll_rtc_init() {

    // Unlock write protection.
    write_reg( registers.sys_key, CLEAR );
    write_reg( registers.sys_key, FIRST_KEY );
    write_reg( registers.sys_key, SECOND_KEY );

    // Turn off the RTCC.
    write_reg( registers.rtc_con_set, ENABLE_WRITE );
    write_reg( registers.iec1_clr, DISABLE_RTC_INTERRUPTION );
    write_reg( registers.rtc_con_clr, SET_RTCC_OFF );

     // Wait for RTCC to be turned off.
    while ( RTC_ON );

    write_reg( registers.ifs1_clr, CLEAR_RTCC_EVENT );
    write_reg( registers.ipc8_clr, CLEAR_PRIORITY );
    write_reg( registers.rtc_alrm_clr, CLEAR_RTCALRM );
    write_reg( registers.sys_key, CLEAR );
}

err_t hal_ll_rtc_stop() {

    // Turn off the RTCC.
    write_reg( registers.rtc_con_clr, SET_RTCC_OFF );

    // Wait for RTCC to be turned off.
    while ( RTC_ON );
    write_reg( registers.sys_key, CLEAR );
    if ( RTC_OFF )
      return RTC_SUCCESS;
    else
        return RTC_ERROR;
}

err_t hal_ll_rtc_start() {

    // Turn on the clock.
    write_reg( registers.sys_key, CLEAR );
    set_reg_bit( registers.rtc_con, BIT1 );
    set_reg_bit( registers.rtc_con, BIT22 );

     // Unlock write protection.
    write_reg( registers.sys_key, FIRST_KEY );
    write_reg( registers.sys_key, SECOND_KEY );

    // Turn on the RTCC.
    write_reg( registers.rtc_con_set, ENABLE_WRITE );
    write_reg( registers.rtc_con_set, SET_RTCC_ON );

    // Wait for RTCC to be turned on.
    while ( RTC_OFF );

    if ( RTC_ON )
      return RTC_SUCCESS;
    else
        return RTC_ERROR;
}

err_t hal_ll_rtc_set_time( hal_ll_rtc_t *time ) {

    uint32_t new_time = set_time_register( time );
    uint32_t new_date = set_date_register( time );

    // Turn off the RTCC.
    write_reg( registers.iec1_clr, DISABLE_RTC_INTERRUPTION );
    write_reg( registers.rtc_con_clr, SET_RTCC_OFF );

    // Wait for clock to be turned off.
    while ( RTC_ON );

    // Update the time and date.
    write_reg( registers.rtc_time, new_time );
    write_reg( registers.rtc_date, new_date );

    // Turn on the RTCC.
    write_reg( registers.rtc_con_set, SET_RTCC_ON );

    // Wait for clock to be turned on.
    while ( RTC_OFF );

    if ( MODIFIED_REGISTER )
        return RTC_SUCCESS;
    else
        return RTC_ERROR;
}

err_t hal_ll_rtc_reset() {

    // Turn off the RTCC.
    write_reg( registers.iec1_clr, DISABLE_RTC_INTERRUPTION );
    write_reg( registers.rtc_con_clr, SET_RTCC_OFF );

    // Wait for clock to be turned off.
    while ( RTC_ON );

    // Update the date to date.
    write_reg( registers.rtc_time, DEFAULT_TIME );
    write_reg( registers.rtc_date, DEFAULT_DATE );

    // Turn on the RTCC.
    write_reg( registers.rtc_con_set, SET_RTCC_ON );

    // Wait for clock to be turned on.
    while ( RTC_OFF );

    if ( DEFAULT_REGISTER )
        return RTC_SUCCESS;
    else
        return RTC_ERROR;
}

void hal_ll_software_reset() {

    uint32_t read;

    // Turn on the clock.
    write_reg( registers.sys_key, CLEAR );
    write_reg( registers.sys_key, FIRST_KEY );
    write_reg( registers.sys_key, SECOND_KEY );

    // Enable reset.
    set_reg_bit(registers.rswrst_set, BIT1);

    // Lauch reset.
    read = read_reg( registers.rswrst );

    WAIT_RESET;
}


// ------------------------------------------------ PRIVATE FUNCTION DEFINITIONS

static uint32_t set_time_register( hal_ll_rtc_t *time ) {

    uint8_t seconds_01 = SECOND_TO_SECOND01;
    uint8_t seconds_10 = SECOND_TO_SECOND10;

    uint8_t minutes_01 = MINUTE_TO_MINUTE01;
    uint8_t minutes_10 = MINUTE_TO_MINUTE10;

    uint8_t hours_01 = HOUR_TO_HOUR01;
    uint8_t hours_10 = HOUR_TO_HOUR10;

    uint32_t new_time = SECOND_TO_REGISTER;

    new_time |= MINUTE_TO_REGISTER;
    new_time |= HOUR_TO_REGISTER;

    return new_time;
}

static uint32_t set_date_register( hal_ll_rtc_t *time ) {

    if( time->year >= TWO_CENTURIES )
        time->year -=  TWO_CENTURIES;

    uint8_t years_01 = YEAR_TO_YEAR01;
    uint8_t years_10 = YEAR_TO_YEAR10;

    uint8_t month_01 = MONTH_TO_MONTH01;
    uint8_t month_10 = MONTH_TO_MONTH10;

    uint8_t day_month_01 = DAY_TO_DAY01;
    uint8_t day_month_10 = DAY_TO_DAY10;

    uint32_t new_date = DAY_TO_REGISTER;
    new_date |= MONTH_TO_REGISTER;
    new_date |= YEAR_TO_REGISTER;

    return new_date;
}
