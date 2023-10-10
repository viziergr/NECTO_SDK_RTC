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
#define GET_SECOND              time_second % 60
#define GET_MINUTE              time_second / 60 - GET_HOUR * 60
#define GET_HOUR                time_second / 3600
#define REGISTER_CHANGED        (read_reg(registers.rtc_cnth)!=(TIME_TO_RTCH))|(read_reg(registers.rtc_cntl)!=(TIME_TO_RTCL))
#define REGISTER_RESET          (read_reg(registers.rtc_cnth)!=0)|(read_reg(registers.rtc_cntl)!=0)
#define RTC_OFF                 check_reg_bit(registers.rcc_bdcr , BIT15)>>15!=1
#define RTC_ON                  check_reg_bit(registers.rcc_bdcr , BIT15)>>15!=0
#define SET_TIME                time->hour * 3600 + time->minute * 60 + time->second
#define WRITE_ACCESS            check_reg_bit(registers.rtc_crl,BIT5)

#define BIT4     4
#define BIT5     5
#define BIT8     8
#define BIT15   15
#define BIT28   28


/*!< @brief REGISTER CONVERTER MACROS */
#define RTCH_TO_TIME            read_reg(registers.rtc_cnth)<<16
#define RTCL_TO_TIME            read_reg(registers.rtc_cntl)
#define TIME_TO_RTCH            time2&0xFFFF0000
#define TIME_TO_RTCL            time2&0x0000FFFF

#define FIRST_KEY_WDG           0x000000ca
#define SECOND_KEY_WDG          0x00000053
#define RCC_DEFAULT             0x00008101
#define DEFAULT_PRESC           0x00007FFF
#define DEFAULT_TIME            0x00000000


/*!< @brief SOFTWARE RESET MACROS */
#define WDG_ACCESS              0x00005555
#define WDG_RLR_SETUP           0x00000001
#define WDG_LAUCH               0x0000CCCC


// ----------------------------------------------------------------- PRIVATE TYPES

/*!< @brief RTC register structure. */
typedef struct
{

    hal_ll_base_addr_t* rcc_apb1enr;
    hal_ll_base_addr_t* pwr_cr;
    hal_ll_base_addr_t* rcc_bdcr;
    hal_ll_base_addr_t* rtc_crl;
    hal_ll_base_addr_t* rtc_prll;
    hal_ll_base_addr_t* rtc_cnth;
    hal_ll_base_addr_t* rtc_cntl;
    hal_ll_base_addr_t* iwdg_kr;
    hal_ll_base_addr_t* iwdg_rlr;

} reg_t;

// -------------------------------------------------------------------- VARIABLES
#ifdef __GNUC__

/*!< @brief RTC registers info  */
reg_t registers = { &APB1ENR, &CR, &BDCR, &CRL ,&PRLL, &CNTH, &CNTL, &KR ,&RLR };

#else

/*!< @brief RTC registers info  */
reg_t registers = { &RCC_APB1ENR, &PWR_CR, &RCC_BDCR, &RTC_CRL ,&RTC_PRLL, &RTC_CNTH, &RTC_CNTL, &IWDG_KR ,&IWDG_RLR };

#endif

#ifdef __GNUC__

// ------------------------------------------------ PUBLIC FUNCTION DEFINITIONS

void __attribute__((weak)) hal_ll_rtc_init();

err_t __attribute__((weak)) hal_ll_configure_default( hal_ll_rtc_t *time );

err_t __attribute__((weak)) hal_ll_rtc_start();

err_t __attribute__((weak)) hal_ll_rtc_stop();

err_t __attribute__((weak)) hal_ll_rtc_reset();

err_t __attribute__((weak)) hal_ll_rtc_set_time( hal_ll_rtc_t *time );

err_t __attribute__((weak)) hal_ll_rtc_get_time( hal_ll_rtc_t *time );

void __attribute__((weak)) hal_ll_software_reset();

#else

// ------------------------------------------------ PUBLIC FUNCTION DEFINITIONS

void hal_ll_rtc_init() {

    set_reg_bit( registers.rcc_apb1enr, BIT28 );
    set_reg_bit( registers.pwr_cr, BIT8 );    
    write_reg( registers.rcc_bdcr, RCC_DEFAULT );
    
    while( !WRITE_ACCESS );
    
    set_reg_bit( registers.rtc_crl, BIT4 );
    write_reg( registers.rtc_prll, DEFAULT_PRESC );
    clear_reg_bit( registers.rtc_crl, BIT4 );
    
    while( !WRITE_ACCESS );
    
    clear_reg_bit( registers.rcc_bdcr, BIT15 );   
}

err_t hal_ll_configure_default( hal_ll_rtc_t *time ) {
    
    uint32_t time2 = SET_TIME;  
    
    while( !WRITE_ACCESS );

    set_reg_bit( registers.rcc_bdcr, BIT15 );
    set_reg_bit (registers.rtc_crl, BIT4 );
    
    write_reg( registers.rtc_cnth, TIME_TO_RTCH );
    write_reg( registers.rtc_cntl, TIME_TO_RTCL );

    clear_reg_bit( registers.rtc_crl, BIT4 );

    while( !WRITE_ACCESS );
    
    clear_reg_bit( registers.rcc_bdcr, BIT15 );
    
    if( REGISTER_CHANGED ){
        return RTC_ERROR;
    }
    return RTC_SUCCESS;
}

err_t hal_ll_rtc_start() {
    set_reg_bit( registers.rcc_bdcr, BIT15 );
    if( RTC_OFF ){
        return RTC_ERROR;
    }
    return RTC_SUCCESS;
}

err_t hal_ll_rtc_stop() {
    clear_reg_bit( registers.rcc_bdcr, BIT15 );
    if( RTC_ON ){
        return RTC_ERROR;
    }
    return RTC_SUCCESS;
}

err_t hal_ll_rtc_reset() {

    while( !WRITE_ACCESS );
    set_reg_bit( registers.rtc_crl, BIT4 );
       
    write_reg( registers.rtc_cnth, DEFAULT_TIME );
    write_reg( registers.rtc_cntl, DEFAULT_TIME );
    
    clear_reg_bit( registers.rtc_crl, BIT4 );
    while( !WRITE_ACCESS );
    
    if( REGISTER_RESET ){
        return RTC_ERROR;
    }
    
    return RTC_SUCCESS;
}

err_t hal_ll_rtc_set_time( hal_ll_rtc_t *time ) {
    
    uint32_t time2 = SET_TIME;  
    
    while( !WRITE_ACCESS );

    set_reg_bit( registers.rtc_crl, BIT4 );
    
    write_reg( registers.rtc_cnth, TIME_TO_RTCH );
    write_reg( registers.rtc_cntl, TIME_TO_RTCL );

    clear_reg_bit( registers.rtc_crl, BIT4 );

    while( !WRITE_ACCESS );
    
    if( REGISTER_CHANGED ){
        return RTC_ERROR;
    }
    return RTC_SUCCESS;
}

err_t hal_ll_rtc_get_time( hal_ll_rtc_t *time ) {
    
    uint32_t time_second = RTCH_TO_TIME;
    
    time_second += RTCL_TO_TIME;
    time->second = GET_SECOND;
    time->minute = GET_MINUTE;
    time->hour = GET_HOUR;
    
    return RTC_SUCCESS;
}

void hal_ll_software_reset() {
    write_reg( registers.iwdg_kr, WDG_ACCESS );
    write_reg( registers.iwdg_rlr, WDG_RLR_SETUP );
    write_reg( registers.iwdg_kr, WDG_LAUCH );
}

#endif