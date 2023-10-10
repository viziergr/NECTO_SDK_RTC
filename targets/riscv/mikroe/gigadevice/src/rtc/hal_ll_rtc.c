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

#define PMU_CTL_REG (hal_ll_base_addr_t*)       0x40007000
#define RTC_CTL_REG (hal_ll_base_addr_t*)       0x40002804
#define RTC_CNTL_REG (hal_ll_base_addr_t*)      0x4000281C
#define RTC_CNTH_REG (hal_ll_base_addr_t*)      0x40002818
#define RTC_INTEN_REG (hal_ll_base_addr_t*)     0x40002800
#define RCU_APB1EN_REG (hal_ll_base_addr_t*)    0x4002101C
#define RCU_BDCTL_REG (hal_ll_base_addr_t*)     0x40021020
#define FWDGT_CTL_REG (hal_ll_base_addr_t*)     0x40003000

#define PMU_CTL_BKPWEN_BIT                      8
#define RCU_BDCTL_LXTALEN_BIT                   0
#define RCU_BDCTL_LXTALSTB_BIT                  1
#define RCU_BDCTL_RTCSRC_BIT                    8
#define RCU_BDCTL_RTCEN_BIT                     15
#define RCU_BDCTL_BKPRST_BIT                    16
#define RCU_APB1EN_WWDGTEN_BIT                  11
#define RCU_APB1EN_BKPIEN_BIT                   27
#define RCU_APB1EN_PMUEN_BIT                    28
#define RTC_INTEN_SCIE_BIT                      0
#define RTC_CTL_CMF_BIT                         4
#define RTC_CTL_LWOFF_BIT                       6
#define FWDGT_CTL_BIT_2                         2
#define FWDGT_CTL_BIT_3                         3
#define FWDGT_CTL_BIT_6                         6
#define FWDGT_CTL_BIT_7                         7
#define FWDGT_CTL_BIT_10                        10
#define FWDGT_CTL_BIT_11                        11
#define FWDGT_CTL_BIT_14                        14
#define FWDGT_CTL_BIT_15                        15
#define FWDGT_PRESCALER_BIT_0                   0

#define SEC_IN_MIN                              60
#define SEC_IN_HOUR                             3600
#define SEC_IN_DAY                              86400
#define SEC_IN_MONTH                            2592000
#define SEC_IN_YEAR                             31536000

#define START_FWDGT                             0x5555
#define UNLOCK_FWDGT_WRITE_PROTECTION           0xCCCC
#define LOCK_FWDGT_WRITE_PROTECTION             0x0000

typedef struct
{
    hal_ll_base_addr_t* pmu_ctl;
    hal_ll_base_addr_t* rtc_ctl;
    hal_ll_base_addr_t* rtc_cntl;
    hal_ll_base_addr_t* rtc_cnth;
    hal_ll_base_addr_t* rcu_apb1en;
    hal_ll_base_addr_t* rcu_bdctl;
    hal_ll_base_addr_t* fwdgt_ctl;
    hal_ll_base_addr_t* rtc_inten;
    hal_ll_base_addr_t* fwdgt_psc;
    
} reg_t;

reg_t registers = { PMU_CTL_REG, RTC_CTL_REG, RTC_CNTL_REG, RTC_CNTH_REG, RCU_APB1EN_REG, RCU_BDCTL_REG, FWDGT_CTL_REG, RTC_INTEN_REG, FWDGT_PSC };

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
void __attribute__ ((weak)) hal_ll_rtc_init();

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
err_t __attribute__ ((weak)) hal_ll_configure_default( hal_ll_rtc_t *time );

/**
 * @brief Start the RTC module.
 * @details Continue the RTC module count from the last time it has count.
 * @param none
 * @return The function can return one of the values defined by
 * rtc_err_t structure, which is size dependant on the architecture.
 * @pre Before calling this function, the user is expected
 * to have initialized RTC by using rtc_init() and the RTC * not counting.
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
err_t __attribute__ ((weak)) hal_ll_rtc_start();

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
err_t __attribute__ ((weak)) hal_ll_rtc_stop();

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
err_t __attribute__ ((weak)) hal_ll_rtc_reset();

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
 *  if( RTC_SUCCESS != hal_ll_rtc_set_time( &time ) ) {
 *      // Error handling strategy
 *  }
 * @endcode
 * */
err_t __attribute__ ((weak)) hal_ll_rtc_set_time( hal_ll_rtc_t *time );

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
 *  if( RTC_SUCCESS != hal_ll_rtc_start() ) {
 *      // Error handling strategy
 *  }
 *
 *  if( RTC_SUCCESS != hal_ll_rtc_get_time( &time ) ){
 *      // Error handling strategy
 *  }
 *  @endcode
 * */
err_t __attribute__((weak)) hal_ll_rtc_get_time( hal_ll_rtc_t *rtc );

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
void __attribute__ ((weak)) hal_ll_software_reset();

#endif

void hal_ll_rtc_init() {
    if( !check_reg_bit( registers.pmu_ctl, PMU_CTL_BKPWEN_BIT ) ) {
        set_reg_bit( registers.rcu_apb1en, RCU_APB1EN_BKPIEN_BIT );
        set_reg_bit( registers.rcu_apb1en, RCU_APB1EN_PMUEN_BIT );
        set_reg_bit( registers.pmu_ctl, PMU_CTL_BKPWEN_BIT );
        set_reg_bit( registers.rcu_apb1en, RCU_APB1EN_WWDGTEN_BIT );
        set_reg_bit( registers.rcu_bdctl, RCU_BDCTL_LXTALEN_BIT );
        set_reg_bit( registers.rcu_bdctl, RCU_BDCTL_LXTALSTB_BIT );
        set_reg_bit( registers.rcu_bdctl, RCU_BDCTL_RTCSRC_BIT );
    }
}

err_t hal_ll_rtc_reset() {
    set_reg_bit( registers.rcu_bdctl, RCU_BDCTL_BKPRST_BIT );
    return RTC_SUCCESS;
}

err_t hal_ll_rtc_start() {
    set_reg_bit( registers.rcu_bdctl, RCU_BDCTL_RTCEN_BIT );
    set_reg_bit( registers.rcu_bdctl, RCU_BDCTL_LXTALEN_BIT );
    return RTC_SUCCESS;
}

err_t hal_ll_rtc_stop() {
    clear_reg_bit( registers.rcu_bdctl, RCU_BDCTL_RTCEN_BIT );
    return RTC_SUCCESS;
}

err_t hal_ll_rtc_set_time( hal_ll_rtc_t *time ) {
    /* Empty because registers are not writable */
    return RTC_SUCCESS;
}

err_t hal_ll_rtc_get_time( hal_ll_rtc_t *time ) {
    uint32_t sec = *registers.rtc_cntl;
    time->year = sec / SEC_IN_YEAR;
    sec %= SEC_IN_YEAR;

    time->month = sec / SEC_IN_MONTH;
    sec %= SEC_IN_MONTH;

    time->day_month = sec / SEC_IN_DAY;
    sec %= SEC_IN_DAY;

    time->hour = sec / SEC_IN_HOUR;
    sec %= SEC_IN_HOUR;

    time->minute = sec / SEC_IN_MIN;
    sec %= SEC_IN_MIN;

    time->second = sec;

    return RTC_SUCCESS;
}

err_t hal_ll_configure_default( hal_ll_rtc_t *time ) {
    /* Empty because there is no hal_ll_set_time */
    return RTC_SUCCESS;
}

void hal_ll_software_reset() {
    write_reg( registers.fwdgt_ctl, UNLOCK_FWDGT_WRITE_PROTECTION );
    write_reg( registers.fwdgt_psc, FWDGT_PRESCALER_BIT_0 );
    write_reg( registers.fwdgt_ctl, LOCK_FWDGT_WRITE_PROTECTION );
    write_reg( registers.fwdgt_ctl, START_FWDGT );
}
