/*****************************************************************************/
/**
* @file RV3028_Defs.h
*
* Micro Crystal RV3028 I2C extreme low power RTC driver.
*
* GNU GENERAL PUBLIC LICENSE:
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
* Errors and commissions should be reported to DanielKampert@kampis-elektroecke.de.
*
* <pre>
* MODIFICATION HISTORY:
*
* Ver   Who  Date        Changes
* ----- ---  --------    -----------------------------------------------
* 1.00  dk   03/11/2020  First release
*
* </pre>
******************************************************************************/

#ifndef RV3028_DEFS_H_
#define RV3028_DEFS_H_

 #include <time.h>
 #include <stdint.h>
 #include <stddef.h>
 #include <stdbool.h>

 /**@brief RV3028 I2C device slave address.
  */
 #define RV3028_ADDRESS                      0x52

 /**@brief Error codes for the RV328 driver.
  */
 typedef enum
 {
    RV3028_NO_ERROR		= 0x00,			    /**< No error. */
    RV3028_INVALID_PARAM	= 0x01,			    /**< Invalid parameter passed to function call. */
    RV3028_TIMEOUT		= 0x02,			    /**< Communication timeout. */
    RV3028_NOT_INITIALIZED	= 0x03,			    /**< Device is not initialized. Please call the
								 RV3028_Init function. */
    RV3028_NOT_READY		= 0x04,			    /**< Device function is not ready. Please initialize them first. */
    RV3028_WP_ACTIVE		= 0x05,			    /**< Device is write protected. Please unprotect the device first. */
    RV3028_COMM_ERROR		= 0x06,			    /**< Communication error. */
 } rv3028_error_t;

 /**@brief Hour modes supported by the RV3028.
  */
 typedef enum
 {
    RV3028_HOURMODE_24		= 0x00,			    /**< 24 hour mode. */
    RV3028_HOURMODE_12		= 0x01,			    /**< 12 hour mode. */
 } rv3028_hourmode_t;

 /**@brief Output clock frequencies supported by the RV3028.
  */
 typedef enum
 {
    RV3028_CLKOUT_32KHZ		= 0x00,			    /**< 32.768 kHz output frequency. */
    RV3028_CLKOUT_8KHZ		= 0x01,			    /**< 8192 Hz output frequency. */
    RV3028_CLKOUT_1KHZ		= 0x02,			    /**< 1024 Hz output frequency. */
    RV3028_CLKOUT_64HZ		= 0x03,			    /**< 64 Hz output frequency. */
    RV3028_CLKOUT_32HZ		= 0x04,			    /**< 32 Hz output frequency. */
    RV3028_CLKOUT_1HZ		= 0x05,			    /**< 1 Hz output frequency. */
    RV3028_CLKOUT_PRE		= 0x06,			    /**< Predefined period countdown timer interrupt. */
 } rv3028_clkout_t;

 /**@brief Trickle charger series resistance options supported by the RV3028.
  */
 typedef enum
 {
    RV3028_TCT_3K		= 0x00,			    /**< 3 kOhms series resistance. */
    RV3028_TCT_5K		= 0x01,			    /**< 3 kOhms series resistance. */
    RV3028_TCT_9K		= 0x02,			    /**< 3 kOhms series resistance. */
    RV3028_TCT_15K		= 0x03,			    /**< 3 kOhms series resistance. */
 } rv3028_tcr_t;

 /**@brief Battery switchover modes for the RV3028.
  */
 typedef enum
 {
    RV3028_BAT_DISABLED		= 0x00,			    /**< Battery switchover disabled. */
    RV3028_BAT_DSM		= 0x01,			    /**< Battery direct switching mode (DSM). */
    RV3028_BAT_LSM		= 0x03,			    /**< Battery level switching mode (LSM). */
 } rv3028_bat_t;

 /**@brief Status flags that can be modified by the user.
  */
 typedef enum
 {
    RV3028_FLAG_POR		= (0x01 << 0x00),	    /**< Power On Reset flag. */
    RV3028_FLAG_EVENT		= (0x01 << 0x01),	    /**< Event flag. */
    RV3028_FLAG_ALARM		= (0x01 << 0x02),	    /**< Alarm flag. */
    RV3028_FLAG_COUNTDOWN	= (0x01 << 0x03),	    /**< Periodic Time Countdown flag. */
    RV3028_FLAG_UPDATE		= (0x01 << 0x04),	    /**< Periodic Time Update flag. */
    RV3028_FLAG_BATTERY		= (0x01 << 0x05),	    /**< Battery Switch flag. */
 } rv3028_flags_t;

 /**@brief Time stamp source for the RV3028.
  */
 typedef enum
 {
    RV3028_TS_EVENT		= 0x00,			    /**< Use an external event as time stamp source. */
    RV3028_TS_BAT		= 0x01,			    /**< Use the automatic backup switchover as time stamp source. */
 } rv3028_ts_src_t;

 /**@brief Event filtering options for the RV3028.
  */
 typedef enum
 {
    RV3028_FILTER_NO		= 0x00,			    /**< No filtering. Edge detection enabled. */
    RV3028_FILTER_256HZ		= 0x01,			    /**< Sampling period 3.9 ms. Level detection. */
    RV3028_FILTER_64HZ		= 0x02,			    /**< Sampling period 15.6 ms. Level detection. */
    RV3028_FILTER_8HZ		= 0x03,			    /**< Sampling period 125 ms. Level detection. */
 } rv3028_evt_filter_t;

 /**@brief Period time update interrupt sources.
  */
 typedef enum
 {
    RV3028_UPDATE_SECOND	= 0x00,			    /**< Periodic time update every second. */
    RV3028_UPDATE_MINUTE	= 0x01,			    /**< Periodic time update every minute. */
 } rv3028_ud_src_t;

 /**@brief Period countdown clock frequencies.
  */
 typedef enum
 {
    RV3028_COUNTDOWN_4096HZ	= 0x00,			    /**< Timer Clock Frequency: 4096 Hz. */
    RV3028_COUNTDOWN_64HZ	= 0x01,			    /**< Timer Clock Frequency: 64 Hz. */
    RV3028_COUNTDOWN_1HZ	= 0x02,			    /**< Timer Clock Frequency: 1 Hz. */
    RV3028_COUNTDOWN_1HZ60	= 0x03,			    /**< Timer Clock Frequency: 1/60 Hz. */
 } rv3028_cd_freq_t;

 /**@brief		Bus communication function pointer which should be mapped to the platform specific read functions of the user.
  * @param Device_Addr	I2C device address.
  * @param Reg_Addr	Register address.
  * @param Reg_Data	Data from the specified address.
  * @param Length	Length of the reg_data array.
  * @return		Communication error code.
  */
 typedef uint8_t (*rv3028_read_fptr_t)(uint8_t Device_Addr, uint8_t Reg_Addr, uint8_t* p_Reg_Data, uint32_t Length);

 /**@brief		Bus communication function pointer which should be mapped to the platform specific write functions of the user.
  * @param Device_Addr	I2C device address.
  * @param Reg_Addr	Register address.
  * @param Reg_Data	Data to the specified address.
  * @param Length	Length of the reg_data array.
  * @return		Communication error code.
  */
 typedef uint8_t (*rv3028_write_fptr_t)(uint8_t Device_Addr, uint8_t Reg_Addr, const uint8_t* p_Reg_Data, uint32_t Length);

 /**@brief RV3028 device initialization object structure.
  */
 typedef struct
 {
    bool		DisableSync;			    /**< Boolean flag to disable the sync for the CLKOUT pin. */
    bool		EnableEventInt;			    /**< Set to #true to enable the event interrupt function on the INT pin. */
    bool		EventHighLevel;			    /**< Set to #true to enable the rising edge or high level event detection. */
    bool		EnableTS;			    /**< Set to #true to enable the time stamp mode. */
    bool		EnableTSOverwrite;		    /**< Set to #true to enable the overwrite mode in time stamp mode. */
    bool		EnableClkOut;			    /**< Boolean flag to enable the CLKOUT function. */
    bool		EnablePOR;			    /**< Boolean flag to enable the POR function on the INT pin. */
    bool		EnableBSIE;			    /**< Boolean flag to enable the BSIE function on the INT pin. */
    bool		EnableCharge;			    /**< Boolean flag to enable the trickle charger function.
								 NOTE: Only needed when \ref rv3028_t.BatteryMode is enabled. */

    rv3028_ts_src_t	TSMode;				    /**< Time stamp mode used by the RV3028.
								 NOTE: Only needed when \ref rv3028_t.EnableTS is set to #true. */
    rv3028_evt_filter_t Filter;				    /**< Event filter options. */
    rv3028_bat_t	BatteryMode;			    /**< Battery mode used by the RV3028. */
    rv3028_clkout_t	Frequency;			    /**< Output frequency for the CLKOUT pin.
								 NOTE: Only needed when \ref rv3028_t.EnableClkOut is set to #true. */
    rv3028_tcr_t	Resistance;			    /**< Trickle charger series resistance selection.
								 NOTE: Only needed when \ref rv3028_t.BatteryMode is enabled. */
    rv3028_hourmode_t	HourMode;			    /**< Hour mode for the RTC. */

    struct tm*		p_CurrentTime;			    /**< Pointer to initial time for the RTC. */

    uint32_t		CurrentUnixTime;		    /**< Initial Unix time for the RTC. */
    uint32_t		Password;			    /**< Initial password for the RTC. Set to a value > 0 to enable the password function. */
 } rv3028_init_t;

 /**@brief RV3028 device object structure.
  */
 typedef struct
 {
    uint8_t		HID;				    /**< Hardware ID from the RTC. */
    uint8_t		VID;				    /**< Version ID from the RTC. */
    uint8_t		DeviceAddr;			    /**< RTC device address. */

    bool		IsInitialized;			    /**< Boolean flag to indicate a successful initialization. */
    bool		IsPOREnabled;			    /**< Current state of the POR function of the INT pin. */
    bool		IsBSIEEnabled;			    /**< Current state of the BSIE function of the INT pin. */
    bool		IsEventIntEnabled;		    /**< Boolean flag to indicate the state of the EIE bit. */
    bool		IsEventHighLevel;		    /**< Boolean flag to indicate the use of high level events on EVI. */
    bool		IsPasswordEnabled;		    /**< Current state of the password function. */
    bool		IsAlarmEnabled;			    /**< Current state of the alarm function. */
    bool		IsTSEnabled;			    /**< Current state of the time stamp function. */
    bool		IsTSOverwriteEnabled;		    /**< Current state of the time stamp overwrite function. */
    bool		IsClkOutEnabled;		    /**< Current state of the CLKOUT function. */
    bool		IsChargeEnabled;		    /**< Current state of the charging function. */

    rv3028_evt_filter_t Filter;				    /**< Selected event filter options. */
    rv3028_ts_src_t	TSMode;				    /**< Time stamp mode used by the RV3028. */
    rv3028_bat_t	BatteryMode;			    /**< Battery mode used by the RV3028. */
    rv3028_hourmode_t	HourMode;			    /**< Current hour mode of the RTC. */
    rv3028_clkout_t	Frequency;			    /**< Output frequency for the CLKOUT pin. */
    rv3028_tcr_t	Resistance;			    /**< Trickle charger series resistance selection. */
    rv3028_read_fptr_t	p_Read;				    /**< Pointer to RV3028 I2C read function. */
    rv3028_write_fptr_t p_Write;			    /**< Pointer to RV3028 I2C write function. */
 } rv3028_t;

 /**@brief RV3028 alarm configuration object structure.
  */
 typedef struct
 {
    bool		EnableInterrupts;		    /**< Set to #true to enable enable the alarm interrupt on INT. */
    bool		EnableMinutesAlarm;		    /**< Set to #true to enable the minutes alarm. */
    bool		EnableHoursAlarm;		    /**< Set to #true to enable the hours alarm. */
    bool		EnableDayAlarm;			    /**< Set to #true to enable the weekday / date alarm. */
    bool		PM;				    /**< PM hours used by the hours alarm. 
								 NOTE: Only important when \ref rv3028_alarm_t.EnableHoursAlarm is set to #true.
								 NOTE: Only important when \ref rv3028_init_t.HourMode is set to #RV3028_HOURMODE_12. 
								 NOTE: The alarm value must be re-initialized when the hour mode is changed! */
    bool		UseDateAlarm;			    /** Set to #true to use the date alarm instead of the weekday alarm. */
    uint8_t		Minutes;			    /**< Value for the minutes alarm.
								 NOTE: Only important when \ref rv3028_alarm_t.EnableMinutesAlarm is set to #true. */
    uint8_t		Hours;				    /**< Value for the hours alarm.
								 NOTE: Only important when \ref rv3028_alarm_t.EnableHoursAlarm is set to #true. */
    uint8_t		Day;				    /**< Value for the weekday / date alarm.
								 NOTE: Only important when \ref rv3028_alarm_t.EnableDayAlarm is set to #true. */
 } rv3028_alarm_t;

 /**@brief RV3028 periodic countdown configuration object structure.
  */
 typedef struct
 {
    bool		EnableRepeat;                       /**< Set to #true to enable the repeat mode. */
    bool		UseInt;                             /**< Set to #true to enable the INT pin. */
    bool		UseClockOut;                        /**< Set to #true to enable the CLKOUT function of the timer. */
    rv3028_cd_freq_t    Frequency;                          /**< Countdown timer frequency. */
    uint16_t		Value;				    /**< Value for the countdown timer. */
 } rv3028_cd_config_t;

#endif /* RV3028_DEFS_H_ */
