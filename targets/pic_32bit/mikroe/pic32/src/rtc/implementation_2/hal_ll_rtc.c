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
/**
 * @file main.c
 * @brief Main function for VIZIER_RTC_GD32VF application.
 */

#include "hal_ll_rtc.h"

// ---------------------------------------------------------------- PRIVATE MACROS
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

/**
 * @brief Returns value of specified bit
 *        mask from a register.
 *
 * @param[in] reg  - register address
 * @param[in] bit_mask - bit mask.
 *
 * @return Register(reg) bits value.
 */
#define read_reg_bits(reg,bit_mask) (selected_reg(reg) & (bit_mask))

/*!< @brief MACROS for the code */
#define RCC_BDCR_LSEON_BIT          0
#define RCC_BDCR_LSERDY_BIT         1
#define RCC_BDCR_RTCSEL_BIT_1       8
#define RCC_BDCR_RTCSEL_BIT_2       9
#define RCC_BDCR_RTCEN_BIT          15
#define RCC_APB1ENR_PWREN_BIT       28
#define RCC_BDCR_BDRST_BIT          16

#define PWR_CR_DBP_BIT              8

#define RTC_ISR_INIT_BIT            7
#define RTC_ISR_INITF_BIT           6
#define INIT_CHECK                  check_reg_bit(registers.rtc_isr, RTC_ISR_INITF_BIT)

#define RTC_TR_SU_MASK              0x000f
#define RTC_TR_ST_MASK              0x0070
#define RTC_TR_MNU_MASK             0x0f00
#define RTC_TR_MNT_MASK             0x7000
#define RTC_TR_HU_MASK              0x000f0000
#define RTC_TR_HT_MASK              0x00300000

#define RTC_DR_DU_MASK              0x0000000f
#define RTC_DR_DT_MASK              0x00000030
#define RTC_DR_MU_MASK              0x00000f00
#define RTC_DR_MT_MASK              0x00001000
#define RTC_DR_WDU_MASK             0x0000e000
#define RTC_DR_YU_MASK              0x000f0000
#define RTC_DR_YT_MASK              0x00f00000

#define FIRST_KEY                   0xCA
#define SECOND_KEY                  0x53
#define ENABLE_INIT                 0x00000080
#define DEFAULT_PRESC               0x007F00FF
#define CLEAR                       0x000000ff
#define MODIFIED_REGISTER           ( RTC_TR == NEW_TIME ) & ( RTC_DR == NEW_DATE )
#define EXIT                        0

#define DEFAULT_TIME                0x00400000
#define DEFAULT_DATE                0x00002101
#define DEFAULT_REGISTER            ( read_reg( registers.rtc_tr ) == DEFAULT_TIME ) & ( read_reg( registers.rtc_dr ) == DEFAULT_DATE )


/*!< @brief REGISTER CONVERTER MACROS */
#define YEAR10_TO_YEAR              (( read_reg(registers.rtc_dr) & RTC_DR_YT_MASK ) >> 20 ) *10
#define YEAR01_TO_YEAR              ( read_reg(registers.rtc_dr) & RTC_DR_YU_MASK ) >> 16
#define MONTH10_TO_MONTH            (( read_reg(registers.rtc_dr) & RTC_DR_MT_MASK ) >> 12 ) *10
#define MONTH01_TO_MONTH            ( read_reg(registers.rtc_dr) & RTC_DR_MU_MASK ) >> 8
#define DAY10_TO_DAY                (( read_reg(registers.rtc_dr) & RTC_DR_DT_MASK ) >> 4 ) *10
#define DAY01_TO_DAY                ( read_reg(registers.rtc_dr) & RTC_DR_DU_MASK )
#define DAY_WEEK                    (read_reg(registers.rtc_dr) & RTC_DR_WDU_MASK) >> 13

#define HOUR10_TO_HOUR              (( read_reg(registers.rtc_tr) & RTC_TR_HT_MASK ) >> 20 ) *10
#define HOUR01_TO_HOUR              (read_reg(registers.rtc_tr) & RTC_TR_HU_MASK ) >> 16
#define MINUTE10_TO_MINUTE          (( read_reg(registers.rtc_tr) & RTC_TR_MNT_MASK ) >> 12 ) *10
#define MINUTE01_TO_MINUTE          ( read_reg(registers.rtc_tr) & RTC_TR_MNU_MASK ) >> 8
#define SECOND10_TO_SECOND          (( read_reg(registers.rtc_tr) & RTC_TR_ST_MASK ) >> 4 ) *10
#define SECOND01_TO_SECOND          ( read_reg(registers.rtc_tr) & RTC_TR_SU_MASK )

#define TWO_CENTURIES  2000

#define SECOND_TO_SECOND01          time->second % 10
#define SECOND_TO_SECOND10          ( time->second - seconds_01 ) / 10
#define MINUTE_TO_MINUTE01          time->minute % 10
#define MINUTE_TO_MINUTE10          ( time->minute - minutes_01 ) / 10
#define HOUR_TO_HOUR01              time->hour % 10
#define HOUR_TO_HOUR10              ( time->hour - hours_01 ) / 10
#define SECOND_TO_REGISTER          ( (uint32_t)seconds_01 ) | ( (uint32_t)seconds_10 << 4 )
#define MINUTE_TO_REGISTER          ( (uint32_t)minutes_01 << 8 ) | ( (uint32_t)minutes_10 << 12 )
#define HOUR_TO_REGISTER            ( (uint32_t)hours_01 << 16 ) | ( (uint32_t)hours_10 << 20 )

#define YEAR_TO_YEAR01              ( time->year ) % 10
#define YEAR_TO_YEAR10              ( time->year - years_01 ) / 10
#define MONTH_TO_MONTH01            time->month % 10
#define MONTH_TO_MONTH10            ( time->month - month_01 ) / 10
#define DAY_TO_DAY01                time->day_month % 10
#define DAY_TO_DAY10                ( time->day_month - day_month_01 ) / 10
#define YEAR_TO_REGISTER            ( (uint32_t)years_01 << 16) | ( (uint32_t)years_10 << 20 )
#define MONTH_TO_REGISTER           ( (uint32_t)month_01 << 8 ) | ( (uint32_t)month_10 << 12 )
#define DAY_TO_REGISTER             ( (uint32_t)time->day_week << 13) | ( (uint32_t)day_month_01 ) | ( (uint32_t)day_month_10 << 4 )


/*!< @brief SOFTWARE RESET MACROS */
#define WATCHDOG_START              0x0000CCCC
#define WATCHDOG_ENABLE_ACCESS      0x00005555
#define WATCHDOG_PR_256             111
#define WATCHDOG_RLR_1              1
#define WATCHDOG_KEY_VALUE          0x0000AAAA

// ----------------------------------------------------------------- PRIVATE TYPES

/*!< @brief RTC register structure. */
typedef struct
{
    hal_ll_base_addr_t* rtc_wpr;
    hal_ll_base_addr_t* rcc_bdcr;
    hal_ll_base_addr_t* rcc_apb1enr;
    hal_ll_base_addr_t* pwr_cr;
    hal_ll_base_addr_t* rtc_tr;
    hal_ll_base_addr_t* rtc_dr;
    hal_ll_base_addr_t* rtc_isr;
    hal_ll_base_addr_t* rtc_bkp0r;
    hal_ll_base_addr_t* rtc_bkp1r;
    hal_ll_base_addr_t* iwdg_kr;
    hal_ll_base_addr_t* iwdg_pr;
    hal_ll_base_addr_t* iwdg_rlr;
    hal_ll_base_addr_t* iwdg_sr;
} reg_t;

// -------------------------------------------------------------------- VARIABLES
 #ifdef __GNUC__

/*!< @brief RTC registers info  */
reg_t registers = { &WPR, &BDCR, &APB1ENR, &CR, &TR, &DR, &ISR, &BKP0R, &BKP1R, &KR, &PR, &RLR, &SR };

 #else

/*!< @brief RTC registers info  */
reg_t registers = { &RTC_WPR, &RCC_BDCR, &RCC_APB1ENR, &PWR_CR, &RTC_TR, &RTC_DR, &RTC_ISR, &RTC_BKP0R, &RTC_BKP1R, &IWDG_KR, &IWDG_PR, &IWDG_RLR, &IWDG_SR };


#endif
#ifdef __GNUC__


void __attribute__ ((weak)) hal_ll_rtc_init();

err_t __attribute__ ((weak)) hal_ll_configure_default( hal_ll_rtc_t *time );

err_t __attribute__ ((weak)) hal_ll_rtc_start();

err_t __attribute__ ((weak)) hal_ll_rtc_stop();

err_t __attribute__ ((weak)) hal_ll_rtc_reset();

err_t __attribute__ ((weak)) hal_ll_rtc_set_time( hal_ll_rtc_t *time );

err_t __attribute__ ((weak)) hal_ll_rtc_get_time( hal_ll_rtc_t *time );

void __attribute__ ((weak)) hal_ll_software_reset();

#else

//------------------------------------------------ PRIVATE FUNCTION DEFINITIONS

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
static uint32_t set_time_register( hal_ll_rtc_t *time ) {

    uint8_t seconds_01 = SECOND_TO_SECOND01;
    uint8_t seconds_10 = SECOND_TO_SECOND10;

    uint8_t minutes_01 = MINUTE_TO_MINUTE01;
    uint8_t minutes_10 = MINUTE_TO_MINUTE10;

    uint8_t hours_01 = HOUR_TO_HOUR01;
    uint8_t hours_10 = HOUR_TO_HOUR10;

    uint32_t NEW_TIME = SECOND_TO_REGISTER;
    NEW_TIME |= MINUTE_TO_REGISTER;
    NEW_TIME |= HOUR_TO_REGISTER;

    return NEW_TIME;

}

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
static uint32_t set_date_register( hal_ll_rtc_t *time ) {

    if(time -> year >= TWO_CENTURIES){
       time -> year -=  TWO_CENTURIES ;
    }
    uint8_t years_01 = YEAR_TO_YEAR01 ;
    uint8_t years_10 = YEAR_TO_YEAR10 ;

    uint8_t month_01 = MONTH_TO_MONTH01;
    uint8_t month_10 = MONTH_TO_MONTH10;

    uint8_t day_month_01 = DAY_TO_DAY01;
    uint8_t day_month_10 = DAY_TO_DAY10;

    uint32_t NEW_DATE = DAY_TO_REGISTER;
    NEW_DATE |= MONTH_TO_REGISTER;
    NEW_DATE |= YEAR_TO_REGISTER;

    return NEW_DATE;
}

//------------------------------------------------ PUBLIC FUNCTION DEFINITIONS

#ifdef __GNUC__

void __attribute__ ((weak)) hal_ll_rtc_init();

err_t __attribute__ ((weak)) hal_ll_configure_default( hal_ll_rtc_t *time );

err_t __attribute__ ((weak)) hal_ll_rtc_start();

err_t __attribute__ ((weak)) hal_ll_rtc_stop();

err_t __attribute__ ((weak)) hal_ll_rtc_reset();

err_t __attribute__ ((weak)) hal_ll_rtc_set_time(hal_ll_rtc_t *time);

err_t __attribute__((weak)) hal_ll_rtc_get_time(hal_ll_rtc_t *rtc);

void __attribute__ ((weak)) hal_ll_software_reset();

#endif

// ------------------------------------------------ PUBLIC FUNCTION DEFINITIONS
void hal_ll_rtc_init() {
    set_reg_bit( registers.rcc_apb1enr, RCC_APB1ENR_PWREN_BIT );
    set_reg_bit( registers.pwr_cr, PWR_CR_DBP_BIT );
    write_reg( registers.rtc_wpr, FIRST_KEY );
    write_reg( registers.rtc_wpr, SECOND_KEY );
    set_reg_bit( registers.rcc_bdcr, RCC_BDCR_RTCSEL_BIT_1 );
    clear_reg_bit( registers.rcc_bdcr, RCC_BDCR_RTCSEL_BIT_2 );
    set_reg_bit( registers.rcc_bdcr, RCC_BDCR_LSEON_BIT );
}

err_t hal_ll_rtc_reset() {
    write_reg( registers.rtc_wpr, CLEAR );              // Unlock write protection
    write_reg( registers.rtc_wpr, FIRST_KEY );          // Unlock write protection
    write_reg( registers.rtc_wpr, SECOND_KEY );         // Unlock write protection
    write_reg( registers.rtc_isr, ENABLE_INIT );        // Enter initialization mode

    while(!INIT_CHECK);

    write_reg( registers.rtc_tr, DEFAULT_TIME );        // Setting time to 12.35.00
    write_reg( registers.rtc_dr, DEFAULT_DATE );        // Set date to  2012-07-18
    write_reg( registers.rtc_isr, EXIT );

    if ( DEFAULT_REGISTER ) {
        return RTC_SUCCESS;
    }
    else return RTC_ERROR;
}

err_t hal_ll_rtc_start() {
    set_reg_bit( registers.rcc_bdcr, RCC_BDCR_RTCEN_BIT );
    return RTC_SUCCESS;
}

err_t hal_ll_rtc_stop() {
    clear_reg_bit( registers.rcc_bdcr, RCC_BDCR_RTCEN_BIT );
    return RTC_SUCCESS;
}

err_t hal_ll_rtc_set_time( hal_ll_rtc_t *time ) {
    write_reg( registers.rtc_wpr, CLEAR );              // Unlock write protection
    write_reg( registers.rtc_wpr, FIRST_KEY );          // Unlock write protection
    write_reg( registers.rtc_wpr, SECOND_KEY );         // Unlock write protection
    write_reg( registers.rtc_isr, ENABLE_INIT );        // Enter initialization mode

    uint32_t NEW_TIME =  set_time_register( time );
    uint32_t NEW_DATE =  set_date_register( time );

    write_reg( registers.rtc_tr, NEW_TIME );            // Setting time to 12.35.00
    write_reg( registers.rtc_dr, NEW_DATE );            // Set date to  2012-07-18
    write_reg( registers.rtc_isr, EXIT );

        if ( MODIFIED_REGISTER ) {
        return RTC_SUCCESS;
    }

    else return RTC_ERROR;
}

err_t hal_ll_rtc_get_time( hal_ll_rtc_t *time ) {

    time -> year          =  YEAR10_TO_YEAR;
    time -> year         +=  YEAR01_TO_YEAR;
    time -> year         +=  TWO_CENTURIES;

    time -> month         =  MONTH10_TO_MONTH;
    time -> month        +=  MONTH01_TO_MONTH;

    time -> day_month     =  DAY10_TO_DAY;
    time -> day_month    +=  DAY01_TO_DAY;

    time -> day_week      =  DAY_WEEK;

    time -> hour          =  HOUR10_TO_HOUR;
    time -> hour         +=  HOUR01_TO_HOUR;

    time -> minute        =  MINUTE10_TO_MINUTE;
    time -> minute       +=  MINUTE01_TO_MINUTE;

    time -> second        =  SECOND10_TO_SECOND;
    time -> second       +=  SECOND01_TO_SECOND;

    return RTC_SUCCESS;
}

err_t hal_ll_configure_default( hal_ll_rtc_t *time ) {
    hal_ll_rtc_set_time( time );
    return RTC_SUCCESS;
}

void hal_ll_software_reset() {
    write_reg( registers.rtc_wpr, CLEAR );              // Unlock write protection
    write_reg( registers.rtc_wpr, FIRST_KEY );          // Unlock write protection
    write_reg( registers.rtc_wpr, SECOND_KEY );         // Unlock write protection
    write_reg( registers.iwdg_kr, WATCHDOG_START );
    write_reg( registers.iwdg_kr, WATCHDOG_ENABLE_ACCESS );
    write_reg( registers.iwdg_pr, WATCHDOG_PR_256 );
    write_reg( registers.iwdg_rlr, WATCHDOG_RLR_1 );
    write_reg( registers.iwdg_kr, WATCHDOG_KEY_VALUE );
}
#endif
