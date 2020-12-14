/*****************************************************************************/
/**
* @file RV3028.h
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

#ifndef RV3028_H_
#define RV3028_H_

 #include "RV3028_Defs.h"

 /**@brief		Initialize the RTC.
  * @param p_Init	Pointer to device initialization structure.
  * @param p_Device	Pointer to RV3028 device structure.
  * @return		Communication error code.
  */
 rv3028_error_t RV3028_Init(rv3028_init_t* p_Init, rv3028_t* p_Device);

 /**@brief		Configure the periodic update interrupt.
  * @param p_Device	Pointer to RV3028 device structure.
  * @param Source	Update source.
  * @param UseInt	Set to #true to enable the INT pin.
  * @return		Communication error code.
  */
 rv3028_error_t RV3028_InitUpdate(rv3028_t* p_Device, rv3028_ud_src_t Source, bool UseInt);

 /**@brief		Configure the periodic countdown timer.
  * @param p_Device	Pointer to RV3028 device structure.
  * @param p_Config	Pointer to periodic countdown configuration structure.
  * @return		Communication error code.
  */
 rv3028_error_t RV3028_InitCountdown(rv3028_t* p_Device, rv3028_cd_config_t* p_Config);

 /**@brief		Disable the periodic countdown timer.
  * @param p_Device	Pointer to RV3028 device structure.
  * @return		Communication error code.
  */
 rv3028_error_t RV3028_DisableCountdown(rv3028_t* p_Device);

 /**@brief		Disable the write protection of the RTC.
  * @param p_Device	Pointer to RV3028 device structure.
  * @param Password	Device password.
  * @return		Communication error code.
  */
 rv3028_error_t RV3028_DisableWP(rv3028_t* p_Device, uint32_t Password);

 /**@brief		Unlock the device.
  * @param p_Device	Pointer to RV3028 device structure.
  * @param Password	Password for the RTC.
  * @return		Communication error code.
  */
 rv3028_error_t RV3028_UnlockWP(rv3028_t* p_Device, uint32_t Password);

 /**@brief		Reset the clock prescaler of the device.
  * @param p_Device	Pointer to RV3028 device structure.
  * @return		Communication error code.
  */
 rv3028_error_t RV3028_Reset(rv3028_t* p_Device);

 /**@brief		Adjust the oscillator offset for frequency compensation purposes.
  * @param p_Device	Pointer to RV3028 device structure.
  * @param Offset	Oscillator offset.
  * @return		Communication error code.
  */
 rv3028_error_t RV3028_Compensate(rv3028_t* p_Device, uint16_t Offset);

 /**@brief		Read the status register to get the pending interrupt flags.
  * @param p_Device	Pointer to RV3028 device structure.
  * @param p_Flags	Pointer to interrupt status flags.
  * @return		Communication error code.
  */
 rv3028_error_t RV3028_GetFlags(rv3028_t* p_Device, uint8_t* p_Flags);

 /**@brief		Clear one or more interrupt status flags.
  * @param p_Device	Pointer to RV3028 device structure.
  * @param Mask		Interrupt status flags mask.
  * @return		Communication error code.
  */
 rv3028_error_t RV3028_ClearFlags(rv3028_t* p_Device, rv3028_flags_t Mask);

 /**@brief		Enable / Disable the POR function of the INT pin.
  * @param p_Device	Pointer to RV3028 device structure.
  * @param Enable	Enable / Disable status for the CLKOUT pin.
  * @return		Communication error code.
  */
 rv3028_error_t RV3028_EnablePOR(rv3028_t* p_Device, bool Enable);

 /**@brief		Check if a battery switchover has occured.
  * @param p_Device	Pointer to RV3028 device structure.
  * @param p_Active	#true when the battery is active.
  * @return		Communication error code.
  */
 rv3028_error_t RV3028_CheckBatterySwitch(rv3028_t* p_Device, bool* p_Active);

 /**@brief		Enable / Disable the CLKOUT pin.
  * @param p_Device	Pointer to RV3028 device structure.
  * @param Enable	Enable / Disable status for the CLKOUT pin.
  * @param DisableSync	Set to #true to disable the synchronization of the CLKOUT pin.
  *			NOTE: Only important when setting \p Enable to #true.
  * @return		Communication error code.
  */
 rv3028_error_t RV3028_EnableClkOut(rv3028_t* p_Device, bool Enable, bool DisableSync);

 /**@brief		Set the output frequency for the CLKOUT pin.
  * @param p_Device	Pointer to RV3028 device structure.
  * @param Frequency	Output frequency for the CLKOUT pin.
  * @return		Communication error code.
  */
 rv3028_error_t RV3028_SetClkOut(rv3028_t* p_Device, rv3028_clkout_t Frequency);

 /**@brief		Set the series resistance of the trickle charger.
  * @param p_Device	Pointer to RV3028 device structure.
  * @param Resistance	Series resistance selection.
  * @return		Communication error code.
  */
 rv3028_error_t RV3028_SetResistance(rv3028_t* p_Device, rv3028_tcr_t Resistance);

 /**@brief		Set the hour mode for the RTC.
  * @param p_Device	Pointer to RV3028 device structure.
  * @param Mode		Hour mode option.
  * @return		Communication error code.
  */
 rv3028_error_t RV3028_SetHourMode(rv3028_t* p_Device, rv3028_hourmode_t Mode);

 /**@brief		Write a single byte into the user EEPROM.
  * @param p_Device	Pointer to RV3028 device structure.
  * @param Address	EEPROM address.
  * @param Data		Data byte.
  * @return		Communication error code.
  */
 rv3028_error_t RV3028_SetEEPROM(rv3028_t* p_Device, uint8_t Address, uint8_t Data);

 /**@brief		Read a single byte from the user EEPROM.
  * @param p_Device	Pointer to RV3028 device structure.
  * @param Address	EEPROM address.
  * @param p_Data	Pointer to data byte.
  * @return		Communication error code.
  */
 rv3028_error_t RV3028_GetEEPROM(rv3028_t* p_Device, uint8_t Address, uint8_t* p_Data);

 /**@brief		Write a single byte into the user RAM.
  * @param p_Device	Pointer to RV3028 device structure.
  * @param Address	RAM address (1 or 2).
  * @param Data		Data byte.
  * @return		Communication error code.
  */
 rv3028_error_t RV3028_SetRAM(rv3028_t* p_Device, uint8_t Address, uint8_t Data);

 /**@brief		Read a single byte into the user RAM.
  * @param p_Device	Pointer to RV3028 device structure.
  * @param Address	RAM address (1 or 2).
  * @param p_Data	Pointer to data byte.
  * @return		Communication error code.
  */
 rv3028_error_t RV3028_GetRAM(rv3028_t* p_Device, uint8_t Address, uint8_t* p_Data);

 /**@brief		Write a byte into the GP bits register.
  * @param p_Device	Pointer to RV3028 device structure.
  * @param Data		Data byte.
  * @return		Communication error code.
  */
 rv3028_error_t RV3028_SetGP(rv3028_t* p_Device, uint8_t Data);

 /**@brief		Modify the bits in the GP bits register.
  * @param p_Device	Pointer to RV3028 device structure.
  * @param Mask		Bit modification mask.
  * @param Value	New value for masked bits.
  * @return		Communication error code.
  */
 rv3028_error_t RV3028_ModifyGP(rv3028_t* p_Device, uint8_t Mask, uint8_t Value);

 /**@brief		Read a byte from the GP bits register.
  * @param p_Device	Pointer to RV3028 device structure.
  * @param p_Data	Pointer to data byte.
  * @return		Communication error code.
  */
 rv3028_error_t RV3028_GetGP(rv3028_t* p_Device, uint8_t* p_Data);

 /**@brief		Write a Unix time into the RTC
  * @param p_Device	Pointer to RV3028 device structure.
  * @param Time		Unix timestamp.
  * @return		Communication error code.
  */
 rv3028_error_t RV3028_SetUnixTime(rv3028_t* p_Device, uint32_t Time);

 /**@brief		Read the Unix time from the RTC.
  * @param p_Device	Pointer to RV3028 device structure.
  * @param p_Time	Pointer to Unix timestamp.
  * @return		Error code
  */
 rv3028_error_t RV3028_GetUnixTime(rv3028_t* p_Device, uint32_t* p_Time);

 /**@brief		Set the time of the RTC
  * @param p_Device	Pointer to RV3028 device structure.
  * @param p_Time	Pointer to time structure.
  * @return		Communication error code.
  */
 rv3028_error_t RV3028_SetTime(rv3028_t* p_Device, struct tm* p_Time);

 /**@brief		Read the time from the RTC.
  * @param p_Device	Pointer to RV3028 device structure.
  * @param p_Time	Pointer to time structure.
  * @return		Communication error code.
  */
 rv3028_error_t RV3028_GetTime(rv3028_t* p_Device, struct tm* p_Time);

 /**@brief		Enable the time stamp function of the RTC.
  * @param p_Device	Pointer to RV3028 device structure.
  * @param Mode		Time stamp source mode.
  * @param OverWrite	Set to #true to enable the overwriting of an existing time stamp.
  * @return		Communication error code.
  */
 rv3028_error_t RV3028_EnableTS(rv3028_t* p_Device, rv3028_ts_src_t Mode, bool OverWrite);

 /**@brief		Disable the time stamp function of the RTC.
  * @param p_Device	Pointer to RV3028 device structure.
  * @return		Communication error code.
  */
 rv3028_error_t RV3028_DisableTS(rv3028_t* p_Device);

 /**@brief		Get the time from the time stamp register.
  * @param p_Device	Pointer to RV3028 device structure.
  * @param p_Time	Pointer to time structure.
  * @param p_Count	Pointer to the corresponding event occurrences.
  * @return		Communication error code.
  */
 rv3028_error_t RV3028_GetTS(rv3028_t* p_Device, struct tm* p_Time, uint8_t* p_Count);

 /**@brief		Enable and configure the alarm.
  * @param p_Device	Pointer to RV3028 device structure.
  * @param p_Alarm	Pointer to alarm configuration structure.
  * @return		Communication error code.
  */
 rv3028_error_t RV3028_EnableAlarm(rv3028_t* p_Device, rv3028_alarm_t* p_Alarm);

 /**@brief		Disable the minutes alarm.
  * @param p_Device	Pointer to RV3028 device structure.
  * @param Minutes	Set to #true to disable the minutes alarms.
  * @param Hours	Set to #true to disable the hours alarms.
  * @param Days		Set to #true to disable the weekdays / date alarms.
  * @return		Communication error code.
  */
 rv3028_error_t RV3028_DisableAlarm(rv3028_t* p_Device, bool Minutes, bool Hours, bool Days);

#endif /* RV3028_H_ */