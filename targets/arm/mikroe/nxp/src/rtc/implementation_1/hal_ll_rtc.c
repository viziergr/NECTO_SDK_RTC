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
 * @brief Rtc TARGETS layer implementation.
 */

#include "hal_ll_rtc.h"
#include "hal_ll_bit_control.h"

/**
 * @defgroup WDOG_macros
 * @brief WDOG macros for the configuration of the software reset
 * @{
 */
#define WDOG_UNLOCK_1                          0xC520
#define WDOG_UNLOCK_2                          0xD928
#define WDOG_VALUE_HIGH                        0x0000
#define WDOG_VALUE_LOW                         0x0004
#define WDOG_INCORRECT_SEQUENCE                0x0000
/** @}*/

/**
 * @brief Macro used to retrieve the value of the rtc_tsr regsiter.
 */
#define CONVERT_RTC_TSR                        *(uint32_t *)registers.rtc_tsr

/**
 * @brief Bit used to enable access write for RTC.
 */
#define SIM_SCGC_B29                           29

/**
 * @defgroup RTC_bits
 * @brief RTC bit numbers used for the configuration.
 * @{
 */
#define RTC_CR_B8                              8
#define RTC_SR_B4                              4
#define RTC_IER_B2                             2
/** @}*/

/**
 * @defgroup WDOG_bits
 * @brief WDOG bit numbers used for the configuration.
 * @{
 */
#define WDOG_STCTRLH_B0                        0
#define WDOG_STCTRLH_B2                        2
/** @}*/

/**
 * @defgroup Time_getters
 * @brief Getters used to retrieve the date and time.
 * @{
 */
#define GET_YEAR                               seconds / 3153600 + 2000
#define GET_MONTH                              seconds / 262800 + 1
#define GET_DAY                                seconds / 86400 + 1
#define GET_HOUR                               seconds / 3600
#define GET_MINUTE                             seconds / 60
#define GET_SECONDS                            seconds % 60
#define GET_YEAR_TO_SECONDS                    seconds % 3153600
#define GET_MONTH_TO_SECONDS                   seconds % 262800
#define GET_DAY_TO_SECONDS                     seconds % 86400
#define GET_HOUR_TO_SECONDS                    seconds % 3600
/** @}*/

/**
 * @brief Macro used to initialize seconds.
 */
#define SECOND_INIT                            0

/**
 * @defgroup Time_converters
 * @brief Getters used to retrieve the date and time.
 * @{
 */
#define YEAR_IN_SECONDS                        ( time->year ) * 3153600
#define MONTH_IN_SECONDS                       ( time->month - 1 ) * 262800
#define DAY_IN_SECONDS                         ( time->day_month - 1 ) * 86400
#define HOUR_IN_SECONDS                        ( time->hour ) * 3600
#define MINUTE_IN_SECONDS                      ( time->minute - 1 ) * 60
#define SECONDS_IN_SECONDS                     ( seconds - 1 )
/** @}*/

/**
 * @brief Macro used to reset RTC register.
 */
#define RTC_TSR_NULL                           0

/**
 * @brief Address of SIM register for GCC. 
 */
#define GCC_SIM_SCGC6                          0x4004803C

/**
 * @defgroup GCC_RTC_bits
 * @brief Adresses of the RTC registers for GCC.
 * @{
 */
#define GCC_RTC_TSR                            0x4003D000
#define GCC_RTC_CR                             0x4003D010
#define GCC_RTC_SR                             0x4003D014
#define GCC_RTC_IER                            0x4003D01C
#define GCC_RTC_TAR                            0x4003D008
/** @}*/

/**
 * @defgroup GCC_WDOG_bits
 * @brief Adresses of the WDOG registers for GCC.
 * @{
 */
#define GCC_WDOG_REFRESH                       0x4005200C
#define GCC_WDOG_STCTRLH                       0x40052000
#define GCC_WDOG_UNLOCK                        0x4005200E
#define GCC_WDOG_TOVALH                        0x40052004
#define GCC_WDOG_TOVALL                        0x40052006
/** @}*/

/**
 * @brief Macro used to adjust seconds correctly. 
 */
#define MINUS_SECOND_1                         -1

typedef struct 
{
    hal_ll_base_addr_t* sim_scgc6;
    hal_ll_base_addr_t* rtc_tsr;
    hal_ll_base_addr_t* rtc_cr;
    hal_ll_base_addr_t* rtc_sr;
    hal_ll_base_addr_t* rtc_ier;
    hal_ll_base_addr_t* rtc_tar;
    uint16_t* wdog_refresh;
    uint16_t* wdog_stctrlh;
    uint16_t* wdog_unlock;
    uint16_t* wdog_tovalh;
    uint16_t* wdog_tovall;
} reg2_t;

typedef struct
{
    hal_ll_base_addr_t* seconds2;
}regsec_t;

#ifdef  __GNUC__
reg2_t registers = { GCC_SIM_SCGC6, GCC_RTC_TSR, GCC_RTC_CR, GCC_RTC_SR, GCC_RTC_IER, GCC_RTC_TAR, GCC_WDOG_REFRESH, GCC_WDOG_STCTRLH, GCC_WDOG_UNLOCK, GCC_WDOG_TOVALH, GCC_WDOG_TOVALL };
#else
reg2_t registers = { &SIM_SCGC6, &RTC_TSR, &RTC_CR, &RTC_SR, &RTC_IER, &RTC_TAR, &WDOG_REFRESH, &WDOG_STCTRLH, &WDOG_UNLOCK, &WDOG_TOVALH, &WDOG_TOVALL };
#endif


unsigned long seconds;

#ifdef  __GNUC__
/**
 * @brief initialize the counter .
 * @details initialize the RTC set the date to 01/01/2000 at 00:00:00.
 * */
void __attribute__((weak)) hal_ll_rtc_init();

/**
 * @brief initialize the default value of counter .
 * @details set the RTC value at time.
 * */
err_t __attribute__((weak)) hal_ll_configure_default ( hal_ll_rtc_t *time );

/**
 * @brief Enable counter.
 * @details Start RTC module count.
 * @return The function can return one of the values defined by
 * hal_ll_rtc_err_t, which is size dependant on the architecture.
 * @pre Before calling this function, the user is expected
 * to configure RTC by using #rtc_configure_default function.
 * */


err_t __attribute__((weak)) hal_ll_rtc_start();

/**
 * @brief Disable counter.
 * @details Stop RTC module count.
 * @return The function can return one of the values defined by
 * hal_ll_rtc_err_t, which is size dependant on the architecture.
 * @pre Before calling this function, the user is expected
 * to have the RTC started.
 * */
err_t __attribute__((weak)) hal_ll_rtc_stop();

/**
 * @brief Configure time of RTC.
 * @details Allow to configure the date and time of the calendar of the rtc.
 * @return The function can return one of the values defined by
 * hal_ll_rtc_err_t, which is size dependant on the architecture.
 * @pre Before calling this function, the user is expected
 * to have the RTC enable and calendar mode activate.
 * */
err_t __attribute__((weak)) hal_ll_rtc_reset();

/**
 * @brief Reset the microcontoller
 * @details Resets the core and all on-chip peripherals except the Debug
 * interface
 * */
err_t __attribute__((weak)) hal_ll_rtc_set_time(hal_ll_rtc_t *time);

/**
 * @brief Get RTC counter.
 * @details Get the rtc counter/the time and date. 
 * @return The function can return one of the values defined by
 * hal_ll_rtc_err_t, which is size dependant on the architecture.
 * @pre Before calling this function, the user is expected
 * to have the RTC enable.
 * */
err_t __attribute__((weak)) hal_ll_rtc_get_time(hal_ll_rtc_t *time);

/**
 * @brief Reset the counter.
 * @details Make the rtc counter go back at his satrt value
 * @return The function can return one of the values defined by
 * hal_ll_rtc_err_t structure, which is size dependant on the architecture.
 * @pre Before calling this function, the user is expected
 * to have the RTC enable.
 * */
void __attribute__((weak)) hal_ll_software_reset();

/**
 * @brief Configure the watchdog
 * @details Conifguration in order to do a software reset
 * */
void __attribute__((weak)) configureWatchdog();

#endif
void hal_ll_rtc_init( ) {
    set_reg_bit( registers.sim_scgc6, SIM_SCGC_B29 );
    if ( read_reg( registers.rtc_tsr ) == RTC_TSR_NULL ) {
        
        hal_ll_rtc_reset();
    }
    set_reg_bit( registers.rtc_cr, RTC_CR_B8 );
    Delay_100ms();
    clear_reg_bit( registers.rtc_sr, RTC_SR_B4 );
    clear_reg_bit( registers.rtc_ier, RTC_IER_B2 );
}

err_t hal_ll_configure_default( hal_ll_rtc_t *time ) {
    hal_ll_rtc_init();
    hal_ll_rtc_set_time( time );
    return RTC_SUCCESS;
}
    
err_t hal_ll_rtc_set_time( hal_ll_rtc_t *time ) {
    seconds = SECOND_INIT;
    seconds += YEAR_IN_SECONDS;
    seconds += MONTH_IN_SECONDS;
    seconds += DAY_IN_SECONDS;
    seconds += HOUR_IN_SECONDS;
    seconds += MINUTE_IN_SECONDS;
    seconds += SECONDS_IN_SECONDS ;
    write_reg( registers.rtc_tsr, &seconds );
    return RTC_SUCCESS;
}

err_t hal_ll_rtc_start() {
    set_reg_bit( registers.rtc_sr, RTC_SR_B4 );
    return RTC_SUCCESS;
}

err_t hal_ll_rtc_stop() {
    clear_reg_bit( registers.rtc_sr, RTC_SR_B4 );
    return RTC_SUCCESS;
}

err_t hal_ll_rtc_get_time( hal_ll_rtc_t *time ) {
    seconds = CONVERT_RTC_TSR;
    time -> year = GET_YEAR;
    seconds = GET_YEAR_TO_SECONDS;
    time -> month = GET_MONTH;
    seconds = GET_MONTH_TO_SECONDS;
    time -> day_month = GET_DAY;
    seconds = GET_DAY_TO_SECONDS;
    time -> hour = GET_HOUR;
    seconds = GET_HOUR_TO_SECONDS; 
    time -> minute = GET_MINUTE;
    time -> second = GET_SECONDS;
    return RTC_SUCCESS;
}

err_t hal_ll_set_alarm( long time ) {
    registers.rtc_tar = time - MINUS_SECOND_1;
    set_reg_bit( registers.rtc_ier, RTC_IER_B2 );
    return RTC_SUCCESS;
}
    
err_t hal_ll_rtc_reset() {
    hal_ll_rtc_stop();
    clear_reg( registers.rtc_tsr );
    return RTC_SUCCESS;
}

void hal_ll_software_reset() {
    configureWatchdog();
    write_reg( registers.wdog_refresh, WDOG_INCORRECT_SEQUENCE );
}

void configureWatchdog() {
    
    clear_reg_bit( registers.wdog_stctrlh, WDOG_STCTRLH_B0 );
    write_reg( registers.wdog_unlock, WDOG_UNLOCK_1 );
    write_reg( registers.wdog_unlock, WDOG_UNLOCK_2 );
    write_reg( registers.wdog_tovalh, WDOG_VALUE_HIGH );
    write_reg( registers.wdog_tovall, WDOG_VALUE_LOW );
    set_reg_bit( registers.wdog_stctrlh, WDOG_STCTRLH_B2 );
    set_reg_bit( registers.wdog_stctrlh, WDOG_STCTRLH_B0 );
    
}
