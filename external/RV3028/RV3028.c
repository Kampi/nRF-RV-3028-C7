/*****************************************************************************/
/**
* @file RV3028.c
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
* Errors and commissions should be reported to DanielKampert@kampis-elektroecke.de
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

#include "RV3028.h"

#define RV3028_REG_SECONDS                  0x00
#define RV3028_REG_MINUTES                  0x01
#define RV3028_REG_HOURS		    0x02
#define RV3028_REG_WEEKDAY                  0x03
#define RV3028_REG_DATE			    0x04
#define RV3028_REG_MONTH		    0x05
#define RV3028_REG_YEAR			    0x06
#define RV3028_REG_MINUTES_ALARM	    0x07
#define RV3028_REG_HOURS_ALARM		    0x08
#define RV3028_REG_WEEKDAY_ALARM	    0x09
#define RV3028_REG_TIMER_VALUE0             0x0A
#define RV3028_REG_TIMER_VALUE1             0x0B
#define RV3028_REG_TIMER_STATUS0            0x0C
#define RV3028_REG_TIMER_STATUS1            0x0D
#define RV3028_REG_STATUS                   0x0E
#define RV3028_REG_CONTROL1                 0x0F
#define RV3028_REG_CONTROL2                 0x10
#define RV3028_REG_GPBITS                   0x11
#define RV3028_REG_CLOCK_INT_MASK           0x12
#define RV3028_REG_EVENTCONTROL             0x13
#define RV3028_REG_COUNT_TS                 0x14
#define RV3028_REG_SECONDS_TS               0x15
#define RV3028_REG_MINUTES_TS               0x16
#define RV3028_REG_HOURS_TS		    0x17
#define RV3028_REG_DATE_TS		    0x18
#define RV3028_REG_MONTH_TS                 0x19
#define RV3028_REG_YEAR_TS                  0x1A
#define RV3028_REG_UNIXTIME0                0x1B
#define RV3028_REG_UNIXTIME1                0x1C
#define RV3028_REG_UNIXTIME2                0x1D
#define RV3028_REG_UNIXTIME3                0x1E
#define RV3028_REG_USER_RAM1                0x1F
#define RV3028_REG_USER_RAM2                0x20
#define RV3028_REG_PASSWORD0                0x21
#define RV3028_REG_PASSWORD1                0x22
#define RV3028_REG_PASSWORD2                0x23
#define RV3028_REG_PASSWORD3                0x24
#define RV3028_REG_EEADDRESS                0x25
#define RV3028_REG_EEDATA                   0x26
#define RV3028_REG_EECOMMAND                0x27
#define RV3028_REG_ID			    0x28

#define RV3028_EEPROM_EEPWE		    0x30
#define RV3028_EEPROM_PASSWORD0		    0x31
#define RV3028_EEPROM_PASSWORD1		    0x32
#define RV3028_EEPROM_PASSWORD2		    0x33
#define RV3028_EEPROM_PASSWORD3		    0x34
#define RV3028_EEPROM_CLKOUT		    0x35
#define RV3028_EEPROM_OFFSET		    0x36
#define RV3028_EEPROM_BACKUP		    0x37

#define RV3028_EECMD_FIRST                  0x00
#define RV3028_EECMD_UPDATE                 0x11
#define RV3028_EECMD_REFRESH                0x12
#define RV3028_EECMD_WRITE                  0x21
#define RV3028_EECMD_READ                   0x22

#define RV3028_BIT_AE_WD		    0x07
#define RV3028_BIT_AE_H			    0x07
#define RV3028_BIT_AE_M			    0x07
#define RV3028_BIT_TSE			    0x07
#define RV3028_BIT_EEBUSY                   0x07
#define RV3028_BIT_CLKOE                    0x07
#define RV3028_BIT_EHL			    0x06
#define RV3028_BIT_BSIE			    0x06
#define RV3028_BIT_CLKSY		    0x06
#define RV3028_BIT_TCE			    0x05
#define RV3028_BIT_BSF			    0x05
#define RV3028_BIT_WADA			    0x05
#define RV3028_BIT_AMPM			    0x05
#define RV3028_BIT_FEDE			    0x04
#define RV3028_BIT_ET			    0x04
#define RV3028_BIT_AIE			    0x03
#define RV3028_BIT_PORIE		    0x03
#define RV3028_BIT_EERD                     0x03
#define RV3028_BIT_AF			    0x02
#define RV3028_BIT_EIE			    0x02
#define RV3028_BIT_TSR			    0x02
#define RV3028_BIT_TSOW			    0x01
#define RV3028_BIT_EVF			    0x01
#define RV3028_BIT_12_24		    0x01
#define RV3028_BIT_PORF			    0x00
#define RV3028_BIT_RESET		    0x00
#define RV3028_BIT_TSS			    0x00

static rv3028_error_t ErrorCode;

/** @brief	    Convert a decimal value into a BCD value.
 *  @param Decimal  Decimal value.
 *  @return	    BCD value.
 */
static inline uint8_t DecimalToBCD(uint8_t Decimal)
{
   return ((Decimal / 0x0A) << 0x04) | (Decimal % 0x0A);
}

/** @brief	    Convert a BCD value into a decimal value.
 *  @param Decimal  BCD value.
 *  @return	    Decimal value.
 */
static inline uint8_t BCDToDecimal(uint8_t BCD)
{
   return ((BCD >> 0x04) * 0x0A) + (BCD & 0xF);
}

/** @brief		Read the content of one or more register from the RV3028.
 *  @param Reg_Addr	Register address.
 *  @param p_Reg_Data	Pointer to register data.
 *  @param Length	Data length.
 *  @param p_Device	Pointer to \ref rv3028_t device structure.
 *  @return		Communication error code.
 */
static rv3028_error_t RV3028_ReadRegister(uint8_t Reg_Addr, uint8_t* p_Reg_Data, uint32_t Length, rv3028_t* p_Device)
{
    if((p_Device == NULL) || (p_Device->p_Read == NULL) || (p_Reg_Data == NULL))
    {
	return RV3028_INVALID_PARAM;
    }
    else if(!p_Device->IsInitialized)
    {
	return RV3028_NOT_INITIALIZED;
    }

    return p_Device->p_Read(p_Device->DeviceAddr, Reg_Addr, p_Reg_Data, Length);
}

/** @brief		Write one or more data bytes into the RV3028.
 *  @param Reg_Addr	Register address.
 *  @param p_Reg_Data	Pointer to register data.
 *  @param Length	Data length.
 *  @param p_Device	Pointer to \ref rv3028_t device structure.
 *  @return		Communication error code.
 */
static rv3028_error_t RV3028_WriteRegister(uint8_t Reg_Addr, const uint8_t* p_Reg_Data, uint32_t Length, rv3028_t* p_Device)
{
    if((p_Device == NULL) || (p_Device->p_Write == NULL) || (p_Reg_Data == NULL))
    {
	return RV3028_INVALID_PARAM;
    }
    else if(!p_Device->IsInitialized)
    {
	return RV3028_NOT_INITIALIZED;
    }

    return p_Device->p_Write(p_Device->DeviceAddr, Reg_Addr, p_Reg_Data, Length);
}

/** @brief          Modify the value of a single register.
 *  @param Address  Register address.
 *  @param Mask	    Bit mask.
 *  @param Value    New value for masked bits.
 *  @param p_Device Pointer to \ref rv3028_t device structure.
 *  @return	    Communication error code.
 */
static rv3028_error_t RV3028_ModifyRegister(uint8_t Address, uint8_t Mask, uint8_t Value, rv3028_t* p_Device)
{
    uint8_t Temp = 0x00;
    uint8_t Addr_Temp = Address;

    ErrorCode = RV3028_ReadRegister(Addr_Temp, &Temp, sizeof(Temp), p_Device);
    if(ErrorCode != RV3028_NO_ERROR)
    {
	return ErrorCode;
    }

    Temp &= ~Mask;
    Temp |= Value & Mask;

    return RV3028_WriteRegister(Addr_Temp, &Temp, sizeof(Temp), p_Device);
}

/** @brief	    Wait as long as the EEPROM is busy.
 *  @param p_Device Pointer to \ref rv3028_t device structure.
 *  @return	    Communication error code.
 */
static rv3028_error_t RV3028_EEPROMWaitBusy(rv3028_t* p_Device)
{
    uint8_t Temp = 0x00;

    // Wait while the EEPROM is busy
    do
    {
	ErrorCode = RV3028_ReadRegister(RV3028_REG_STATUS, &Temp, 1, p_Device);
	if(ErrorCode != RV3028_NO_ERROR)
	{
	    return ErrorCode;
	}
    } while(Temp & (0x01 << RV3028_BIT_EEBUSY));

    return ErrorCode;
}

/** @brief          Execute an EEPROM command.
 *  @param Command  EEPROM command code.
 *  @param p_Device Pointer to \ref rv3028_t device structure.
 *  @return	    Communication error code.
 */
static rv3028_error_t RV3028_EEPROMCommand(uint8_t Command, rv3028_t* p_Device)
{
    uint8_t Temp[2] = {RV3028_EECMD_FIRST, Command};

    ErrorCode = RV3028_WriteRegister(RV3028_REG_EECOMMAND, &Temp[0], 1, p_Device);
    if(ErrorCode != RV3028_NO_ERROR)
    {
	return ErrorCode;
    }

    ErrorCode = RV3028_WriteRegister(RV3028_REG_EECOMMAND, &Temp[1], 1, p_Device);
    if(ErrorCode != RV3028_NO_ERROR)
    {
	return ErrorCode;
    }

    return RV3028_EEPROMWaitBusy(p_Device);
}

/** @brief          Enable or disable the auto refresh function of the RTC.
 *  @param Enable   Enable / Disable of the auto refresh function.
 *  @param p_Device Pointer to \ref rv3028_t device structure.
 *  @return	    Communication error code.
 */
static rv3028_error_t RV3028_EnableEERD(bool Enable, rv3028_t* p_Device)
{
    return RV3028_ModifyRegister(RV3028_REG_CONTROL1, (0x01 << RV3028_BIT_EERD), ~(Enable << RV3028_BIT_EERD), p_Device);
}

/** @brief          Execute the UPDATE command to copy the configuration from the RAM into the EEPROM.
 *  @param p_Device Pointer to \ref rv3028_t device structure.
 *  @return	    Communication error code.
 */
static rv3028_error_t RV3028_Update(rv3028_t* p_Device)
{
    ErrorCode = RV3028_EnableEERD(false, p_Device);
    if(ErrorCode != RV3028_NO_ERROR)
    {
	return ErrorCode;
    }

    ErrorCode = RV3028_EEPROMCommand(RV3028_EECMD_UPDATE, p_Device);
    if(ErrorCode != RV3028_NO_ERROR)
    {
	return ErrorCode;
    }

    return RV3028_EnableEERD(true, p_Device);
}

/** @brief          Execute the REFRESH command to copy the configuration from the EEPROM into the RAM.
 *  @param p_Device Pointer to \ref rv3028_t device structure.
 *  @return	    Communication error code.
 */
static rv3028_error_t RV3028_Refresh(rv3028_t* p_Device)
{
    ErrorCode = RV3028_EnableEERD(false, p_Device);
    if(ErrorCode != RV3028_NO_ERROR)
    {
	return ErrorCode;
    }

    ErrorCode = RV3028_EEPROMCommand(RV3028_EECMD_REFRESH, p_Device);
    if(ErrorCode != RV3028_NO_ERROR)
    {
	return ErrorCode;
    }

    return RV3028_EnableEERD(true, p_Device);
}

rv3028_error_t RV3028_Init(rv3028_init_t* p_Init, rv3028_t* p_Device)
{
    uint8_t Temp = 0x00;

    p_Device->IsInitialized = true;

    if(p_Init == NULL)
    {
	p_Device->IsInitialized = false;
	return RV3028_INVALID_PARAM;
    }

    ErrorCode = RV3028_ReadRegister(RV3028_REG_ID, &Temp, sizeof(Temp), p_Device);
    if(ErrorCode != RV3028_NO_ERROR)
    {
	p_Device->IsInitialized = false;
	return ErrorCode;
    }

    // Refresh the settings in the RAM with the settings from the EEPROM
    ErrorCode = RV3028_Refresh(p_Device);
    if(ErrorCode != RV3028_NO_ERROR)
    {
	return ErrorCode;
    }

    // Disable automatic refresh
    ErrorCode = RV3028_EnableEERD(false, p_Device);
    if(ErrorCode != RV3028_NO_ERROR)
    {
	return ErrorCode;
    }

    // Store the HID and VID
    p_Device->HID = (Temp & 0xF0) >> 0x04;
    p_Device->VID = Temp & 0x0F;

    // Configure the CLKOUT register
    ErrorCode = RV3028_ModifyRegister(RV3028_EEPROM_CLKOUT, 
				    (0x01 << RV3028_BIT_CLKOE) | (0x01 << RV3028_BIT_CLKSY) | (0x01 << RV3028_BIT_PORIE) | 0x07, 
				    (p_Init->EnableClkOut << RV3028_BIT_CLKOE) | (!p_Init->DisableSync << RV3028_BIT_CLKSY) | (p_Init->EnablePOR << RV3028_BIT_PORIE) | (p_Init->Frequency & 0x07), 
				    p_Device);
    if(ErrorCode != RV3028_NO_ERROR)
    {
	return ErrorCode;
    }

    p_Device->IsPOREnabled = p_Init->EnablePOR;
    p_Device->IsClkOutEnabled = p_Init->EnableClkOut;
    p_Device->Frequency = p_Init->Frequency;

    ErrorCode = RV3028_ModifyRegister(RV3028_EEPROM_BACKUP, 
				    (0x01 << RV3028_BIT_BSIE) | (0x01 << RV3028_BIT_TCE) | (0x01 << RV3028_BIT_FEDE) | (0x03 << 0x02) | 0x03, 
				    (p_Init->EnableBSIE << RV3028_BIT_BSIE) | (p_Init->EnableCharge << RV3028_BIT_TCE) | (0x01 << RV3028_BIT_FEDE) | ((p_Init->BatteryMode & 0x03) << 0x02) | (p_Init->Resistance & 0x03), 
				    p_Device);
    if(ErrorCode != RV3028_NO_ERROR)
    {
	return ErrorCode;
    }

    p_Device->IsBSIEEnabled = p_Init->EnableBSIE;
    p_Device->Resistance = p_Init->Resistance;
    p_Device->BatteryMode = p_Init->BatteryMode;
    p_Device->IsChargeEnabled = p_Init->EnableCharge;

    // Set the initial password
    if(p_Init->Password != 0x00)
    {
	// Check if the write protection is already enabled
	ErrorCode = RV3028_ReadRegister(RV3028_EEPROM_EEPWE, &Temp, sizeof(Temp), p_Device);
	if(ErrorCode != RV3028_NO_ERROR)
	{
	    return ErrorCode;
	}
    
	if(Temp == 0xFF)
	{
	    return RV3028_WP_ACTIVE;
	}

	// Store the reference password
	uint8_t PW_Temp[4] = {p_Init->Password & 0xFF, (p_Init->Password >> 0x08) & 0xFF, (p_Init->Password >> 0x10) & 0xFF, (p_Init->Password >> 0x18) & 0xFF};
	ErrorCode = RV3028_WriteRegister(RV3028_EEPROM_PASSWORD0, PW_Temp, sizeof(uint32_t), p_Device);
	if(ErrorCode != RV3028_NO_ERROR)
	{
	    return ErrorCode;
	}

	// Enable the password functionality
	Temp = 0xFF;
	ErrorCode = RV3028_WriteRegister(RV3028_EEPROM_EEPWE, &Temp, sizeof(Temp), p_Device);
	if(ErrorCode != RV3028_NO_ERROR)
	{
	    return ErrorCode;
	}

	p_Device->IsPasswordEnabled = true;

	// Lock the device
	ErrorCode = RV3028_UnlockWP(p_Device, 0x00);
	if(ErrorCode != RV3028_NO_ERROR)
	{
	    return ErrorCode;
	}
    }

    // Update the settings in the EEPROM
    ErrorCode = RV3028_Update(p_Device);
    if(ErrorCode != RV3028_NO_ERROR)
    {
	return ErrorCode;
    }

    // Load the settings from the EEPROM to make them active
    ErrorCode = RV3028_Refresh(p_Device);
    if(ErrorCode != RV3028_NO_ERROR)
    {
	return ErrorCode;
    }

    // Enable automatic refresh
    ErrorCode = RV3028_EnableEERD(true, p_Device);
    if(ErrorCode != RV3028_NO_ERROR)
    {
	return ErrorCode;
    }

    // Disable the alarms
    ErrorCode = RV3028_ModifyRegister(RV3028_REG_MINUTES_ALARM, 0x01 << RV3028_BIT_AE_M, 0x01 << RV3028_BIT_AE_M, p_Device);
    if(ErrorCode != RV3028_NO_ERROR)
    {
	return ErrorCode;
    }

    ErrorCode = RV3028_ModifyRegister(RV3028_REG_HOURS_ALARM, 0x01 << RV3028_BIT_AE_H, 0x01 << RV3028_BIT_AE_H, p_Device);
    if(ErrorCode != RV3028_NO_ERROR)
    {
	return ErrorCode;
    }

    ErrorCode = RV3028_ModifyRegister(RV3028_REG_WEEKDAY_ALARM, 0x01 << RV3028_BIT_AE_WD, 0x01 << RV3028_BIT_AE_WD, p_Device);
    if(ErrorCode != RV3028_NO_ERROR)
    {
	return ErrorCode;
    }

    p_Device->IsAlarmEnabled = false;

    // Configure the EVENT register
    ErrorCode = RV3028_ModifyRegister(RV3028_REG_EVENTCONTROL, 
				    (0x01 << RV3028_BIT_TSR) | (0x01 << RV3028_BIT_TSOW) | (0x01 << RV3028_BIT_TSS) | (0x01 << RV3028_BIT_EHL) | (0x03 << RV3028_BIT_ET),
				    (0x01 << RV3028_BIT_TSR) | (p_Init->EnableTSOverwrite << RV3028_BIT_TSOW) | ((p_Init->TSMode & 0x01) << RV3028_BIT_TSS) | (p_Init->EventHighLevel << RV3028_BIT_EHL) | ((p_Init->Filter & 0x03) << RV3028_BIT_ET),
				    p_Device);
    if(ErrorCode != RV3028_NO_ERROR)
    {
	return ErrorCode;
    }

    p_Device->IsEventHighLevel = p_Init->EventHighLevel;
    p_Device->Filter = p_Init->Filter;
    p_Device->IsTSOverwriteEnabled = p_Init->EnableTSOverwrite;
    p_Device->TSMode = p_Init->TSMode;

    // Configure the CONTROL2 register
    ErrorCode = RV3028_ClearFlags(p_Device, RV3028_FLAG_EVENT);
    if(ErrorCode != RV3028_NO_ERROR)
    {
	return ErrorCode;
    }

    ErrorCode = RV3028_ModifyRegister(RV3028_REG_CONTROL2, 
				    (0x01 << RV3028_BIT_TSE) | (0x01 << RV3028_BIT_EIE) | (0x01 << RV3028_BIT_12_24), 
				    (p_Init->EnableTS << RV3028_BIT_TSE) | (p_Init->EnableEventInt << RV3028_BIT_EIE) | (p_Init->HourMode << RV3028_BIT_12_24),
				    p_Device);
    if(ErrorCode != RV3028_NO_ERROR)
    {
	return ErrorCode;
    }

    p_Device->IsTSEnabled = p_Init->EnableTS;
    p_Device->HourMode = p_Init->HourMode;
    p_Device->IsEventIntEnabled = p_Init->EnableEventInt;

    // Set the initial time
    if(p_Init->p_CurrentTime != NULL)
    {
	ErrorCode = RV3028_SetTime(p_Device, p_Init->p_CurrentTime);
	if(ErrorCode != RV3028_NO_ERROR)
	{
	    return ErrorCode;
	}
    }

    // Set the initial Unix time
    if(p_Init->CurrentUnixTime != 0x00)
    {
	ErrorCode = RV3028_SetUnixTime(p_Device, p_Init->CurrentUnixTime);
	if(ErrorCode != RV3028_NO_ERROR)
	{
	    return ErrorCode;
	}
    }

    return ErrorCode;
}

rv3028_error_t RV3028_DisableWP(rv3028_t* p_Device, uint32_t Password)
{
    uint8_t Temp = 0x00;
    uint8_t PW_Temp[4] = {Password & 0xFF, (Password >> 0x08) & 0xFF, (Password >> 0x10) & 0xFF, (Password >> 0x18) & 0xFF};

    p_Device->IsInitialized = true;

    // Check if the write protection is already enabled
    ErrorCode = RV3028_ReadRegister(RV3028_EEPROM_EEPWE, &Temp, sizeof(Temp), p_Device);
    if(ErrorCode != RV3028_NO_ERROR)
    {
	return ErrorCode;
    }

    // Leave the function when no password protection is enabled
    if(Temp != 0xFF)
    {
	p_Device->IsPasswordEnabled = false;
	p_Device->IsInitialized = false;

	return ErrorCode;
    }

    // Write the password
    ErrorCode = RV3028_WriteRegister(RV3028_REG_PASSWORD0, PW_Temp, sizeof(uint32_t), p_Device);
    if(ErrorCode != RV3028_NO_ERROR)
    {
	return ErrorCode;
    }

    // Disable automatic refresh
    ErrorCode = RV3028_EnableEERD(false, p_Device);
    if(ErrorCode != RV3028_NO_ERROR)
    {
	return ErrorCode;
    }

    // Disable the write protection by setting EEPWE =/= 255
    Temp = 0x00;
    ErrorCode =  RV3028_WriteRegister(RV3028_EEPROM_EEPWE, &Temp, sizeof(Temp), p_Device);
    if(ErrorCode != RV3028_NO_ERROR)
    {
	return ErrorCode;
    }

    // Update the settings in the EEPROM
    ErrorCode = RV3028_Update(p_Device);
    if(ErrorCode != RV3028_NO_ERROR)
    {
	return ErrorCode;
    }

    p_Device->IsPasswordEnabled = false;
    p_Device->IsInitialized = false;

    return ErrorCode;
}

rv3028_error_t RV3028_UnlockWP(rv3028_t* p_Device, uint32_t Password)
{
    if(p_Device->IsPasswordEnabled == false)
    {
	return RV3028_NOT_READY;
    }

    uint8_t Temp[4] = {Password & 0xFF, (Password >> 0x08) & 0xFF, (Password >> 0x10) & 0xFF, (Password >> 0x18) & 0xFF};

    return RV3028_WriteRegister(RV3028_REG_PASSWORD0, Temp, sizeof(uint32_t), p_Device);
}

rv3028_error_t RV3028_Reset(rv3028_t* p_Device)
{
    return RV3028_ModifyRegister(RV3028_REG_CONTROL2, 0x01 << RV3028_BIT_RESET, 0x01 << RV3028_BIT_RESET, p_Device);
}

rv3028_error_t RV3028_Compensate(rv3028_t* p_Device, uint16_t Offset)
{
    uint8_t Temp = (Offset >> 0x01) & 0xFF;

    ErrorCode = RV3028_WriteRegister(RV3028_EEPROM_OFFSET, &Temp, sizeof(Temp), p_Device);
    if(ErrorCode != RV3028_NO_ERROR)
    {
	return ErrorCode;
    }

    ErrorCode = RV3028_ModifyRegister(RV3028_EEPROM_BACKUP, 0x01 << 0x07, (Offset & 0x01) << 0x07, p_Device);
    if(ErrorCode != RV3028_NO_ERROR)
    {
	return ErrorCode;
    }

    return RV3028_Update(p_Device);
}

rv3028_error_t RV3028_GetFlags(rv3028_t* p_Device, uint8_t* p_Flags)
{
    uint8_t Temp;

    if(p_Flags == NULL)
    {
	return RV3028_INVALID_PARAM;
    }

    ErrorCode = RV3028_ReadRegister(RV3028_REG_STATUS, &Temp, sizeof(Temp), p_Device);
    if(ErrorCode != RV3028_NO_ERROR)
    {
	return ErrorCode;
    }

    *p_Flags = Temp & 0x3F;

    return ErrorCode;
}

rv3028_error_t RV3028_ClearFlags(rv3028_t* p_Device, rv3028_flags_t Mask)
{
    return RV3028_ModifyRegister(RV3028_REG_STATUS, Mask & 0x3F, 0x00, p_Device);
}

rv3028_error_t RV3028_EnablePOR(rv3028_t* p_Device, bool Enable)
{
    uint8_t Temp = 0x00;

    ErrorCode = RV3028_ModifyRegister(RV3028_EEPROM_CLKOUT, 0x01 << RV3028_BIT_PORIE, Enable << RV3028_BIT_PORIE, p_Device);
    if(ErrorCode != RV3028_NO_ERROR)
    {
	return ErrorCode;
    }

    ErrorCode = RV3028_Update(p_Device);
    if(ErrorCode != RV3028_NO_ERROR)
    {
	return ErrorCode;
    }

    p_Device->IsPOREnabled = Enable;

    return ErrorCode;
}

rv3028_error_t RV3028_CheckBatterySwitch(rv3028_t* p_Device, bool* p_Active)
{
    uint8_t Temp;

    if(p_Device->IsChargeEnabled == false)
    {
	return RV3028_NOT_READY;
    }

    if(p_Active == NULL)
    {
	return RV3028_INVALID_PARAM;
    }

    ErrorCode = RV3028_ReadRegister(RV3028_REG_STATUS, &Temp, sizeof(Temp), p_Device);
    if(ErrorCode != RV3028_NO_ERROR)
    {
	return ErrorCode;
    }

    *p_Active = (Temp & (0x01 << RV3028_BIT_BSF)) >> RV3028_BIT_BSF;

    return ErrorCode;
}

rv3028_error_t RV3028_EnableClkOut(rv3028_t* p_Device, bool Enable, bool DisableSync)
{
    ErrorCode = RV3028_ModifyRegister(RV3028_EEPROM_CLKOUT, 
					(0x01 << RV3028_BIT_CLKOE) | (0x01 << RV3028_BIT_CLKSY), 
					Enable << RV3028_BIT_CLKOE | (!DisableSync << RV3028_BIT_CLKSY), 
					p_Device);
    if(ErrorCode != RV3028_NO_ERROR)
    {
	return ErrorCode;
    }

    p_Device->IsClkOutEnabled = Enable;

    return ErrorCode;
}

rv3028_error_t RV3028_SetClkOut(rv3028_t* p_Device, rv3028_clkout_t Frequency)
{
    if(p_Device->IsClkOutEnabled == false)
    {
	return RV3028_NOT_READY;
    }

    ErrorCode = RV3028_ModifyRegister(RV3028_EEPROM_CLKOUT, 0x07, Frequency & 0x07, p_Device);
    if(ErrorCode != RV3028_NO_ERROR)
    {
	return ErrorCode;
    }

    p_Device->Frequency = Frequency;

    return ErrorCode;
}

rv3028_error_t RV3028_SetResistance(rv3028_t* p_Device, rv3028_tcr_t Resistance)
{
    if(p_Device->IsChargeEnabled == false)
    {
	return RV3028_NOT_READY;
    }

    ErrorCode = RV3028_ModifyRegister(RV3028_EEPROM_BACKUP, 0x03, Resistance & 0x07, p_Device);
    if(ErrorCode != RV3028_NO_ERROR)
    {
	return ErrorCode;
    }

    p_Device->Resistance = Resistance;

    return ErrorCode;
}

rv3028_error_t RV3028_SetHourMode(rv3028_t* p_Device, rv3028_hourmode_t Mode)
{
    ErrorCode = RV3028_ModifyRegister(RV3028_REG_CONTROL2, 0x01 << RV3028_BIT_12_24, (Mode & 0x01) << RV3028_BIT_12_24, p_Device);
    if(ErrorCode != RV3028_NO_ERROR)
    {
	return ErrorCode;
    }

    p_Device->HourMode = Mode;

    return ErrorCode;
}

rv3028_error_t RV3028_SetEEPROM(rv3028_t* p_Device, uint8_t Address, uint8_t Data)
{
    ErrorCode = RV3028_EnableEERD(false, p_Device);
    if(ErrorCode != RV3028_NO_ERROR)
    {
	return ErrorCode;
    }

    ErrorCode = RV3028_WriteRegister(RV3028_REG_EEADDRESS, &Address, sizeof(Address), p_Device);
    if(ErrorCode != RV3028_NO_ERROR)
    {
	return ErrorCode;
    }
 
    ErrorCode = RV3028_WriteRegister(RV3028_REG_EEDATA, &Data, sizeof(Data), p_Device);
    if(ErrorCode != RV3028_NO_ERROR)
    {
	return ErrorCode;
    }

    ErrorCode = RV3028_EEPROMCommand(RV3028_EECMD_WRITE, p_Device);
    if(ErrorCode != RV3028_NO_ERROR)
    {
	return ErrorCode;
    }

    return RV3028_EnableEERD(true, p_Device);
}

rv3028_error_t RV3028_GetEEPROM(rv3028_t* p_Device, uint8_t Address, uint8_t* p_Data)
{
    if(p_Data == NULL)
    {
	return RV3028_INVALID_PARAM;
    }

    ErrorCode = RV3028_EnableEERD(false, p_Device);
    if(ErrorCode != RV3028_NO_ERROR)
    {
	return ErrorCode;
    }
 
    ErrorCode = RV3028_WriteRegister(RV3028_REG_EEADDRESS, &Address, sizeof(Address), p_Device);
    if(ErrorCode != RV3028_NO_ERROR)
    {
	return ErrorCode;
    }

    ErrorCode = RV3028_EEPROMCommand(RV3028_EECMD_READ, p_Device);
    if(ErrorCode != RV3028_NO_ERROR)
    {
	return ErrorCode;
    }

    ErrorCode = RV3028_ReadRegister(RV3028_REG_EEDATA, p_Data, sizeof(uint8_t), p_Device);
    if(ErrorCode != RV3028_NO_ERROR)
    {
	return ErrorCode;
    }

    return RV3028_EnableEERD(true, p_Device);
}

rv3028_error_t RV3028_SetRAM(rv3028_t* p_Device, uint8_t Address, uint8_t Data)
{
    if(Address > 0x02)
    {
	return RV3028_INVALID_PARAM;
    }

    return RV3028_WriteRegister(RV3028_REG_USER_RAM1 + Address, &Data, sizeof(uint8_t), p_Device);
}

rv3028_error_t RV3028_GetRAM(rv3028_t* p_Device, uint8_t Address, uint8_t* Data)
{
    if(Address > 0x02)
    {
	return RV3028_INVALID_PARAM;
    }

    return RV3028_ReadRegister(RV3028_REG_USER_RAM1 + Address, Data, sizeof(uint8_t), p_Device);;
}

rv3028_error_t RV3028_SetGP(rv3028_t* p_Device, uint8_t Data)
{
    return RV3028_WriteRegister(RV3028_REG_GPBITS, &Data, sizeof(uint8_t), p_Device);
}

rv3028_error_t RV3028_ModifyGP(rv3028_t* p_Device, uint8_t Mask, uint8_t Value)
{
    return RV3028_ModifyRegister(RV3028_REG_GPBITS, Mask, Value, p_Device);
}

rv3028_error_t RV3028_GetGP(rv3028_t* p_Device, uint8_t* p_Data)
{
    return RV3028_ReadRegister(RV3028_REG_GPBITS, p_Data, sizeof(uint8_t), p_Device);
}

rv3028_error_t RV3028_SetUnixTime(rv3028_t* p_Device, uint32_t Time)
{
    uint8_t Temp[4] = {Time & 0xFF, (Time >> 0x08) & 0xFF, (Time >> 0x10) & 0xFF, (Time >> 0x18) & 0xFF};

    return RV3028_WriteRegister(RV3028_REG_UNIXTIME0, Temp, sizeof(uint32_t), p_Device);
}

rv3028_error_t RV3028_GetUnixTime(rv3028_t* p_Device, uint32_t* p_Time)
{
    uint8_t Temp[4] = {0x00, 0x00, 0x00, 0x00};

    ErrorCode = RV3028_ReadRegister(RV3028_REG_UNIXTIME0, Temp, sizeof(uint32_t), p_Device);
    if(ErrorCode != RV3028_NO_ERROR)
    {
	return ErrorCode;
    }

    *p_Time = ((uint32_t)(Temp[3] << 0x18)) | ((uint32_t)(Temp[2] << 0x10)) | ((uint32_t)(Temp[1] << 0x08)) | Temp[0];

    return RV3028_NO_ERROR;
}

rv3028_error_t RV3028_SetTime(rv3028_t* p_Device, struct tm* p_Time)
{
    uint8_t Temp[7];

    if(p_Time == NULL)
    {
	return RV3028_INVALID_PARAM;
    }

    Temp[0] = DecimalToBCD(p_Time->tm_sec);
    Temp[1] = DecimalToBCD(p_Time->tm_min);
    Temp[2] = DecimalToBCD(p_Time->tm_hour);
    Temp[3] = DecimalToBCD(p_Time->tm_wday);
    Temp[4] = DecimalToBCD(p_Time->tm_mday);
    Temp[5] = DecimalToBCD(p_Time->tm_mon);
    Temp[6] = DecimalToBCD(p_Time->tm_year);

    return RV3028_WriteRegister(RV3028_REG_SECONDS, Temp, sizeof(Temp), p_Device);
}

rv3028_error_t RV3028_GetTime(rv3028_t* p_Device, struct tm* p_Time)
{
    uint8_t Temp[7];

    if(p_Time == NULL)
    {
	return RV3028_INVALID_PARAM;
    }

    ErrorCode = RV3028_ReadRegister(RV3028_REG_SECONDS, Temp, sizeof(Temp), p_Device);
    if(ErrorCode != RV3028_NO_ERROR)
    {
	return ErrorCode;
    }

    p_Time->tm_sec = BCDToDecimal(Temp[0]);
    p_Time->tm_min = BCDToDecimal(Temp[1]);
    p_Time->tm_hour = BCDToDecimal(Temp[2]);
    p_Time->tm_wday = BCDToDecimal(Temp[3]);
    p_Time->tm_mday = BCDToDecimal(Temp[4]);
    p_Time->tm_mon = BCDToDecimal(Temp[5]);
    p_Time->tm_year = BCDToDecimal(Temp[6]);

    return RV3028_NO_ERROR;
}

rv3028_error_t RV3028_EnableTS(rv3028_t* p_Device, rv3028_ts_src_t Mode, bool OverWrite)
{
    // Clear the TSE and EIE bits in the CONTROL2 register
    ErrorCode = RV3028_ModifyRegister(RV3028_REG_CONTROL2, 
				    (0x01 << RV3028_BIT_TSE) | (0x01 << RV3028_BIT_EIE),
				    (0x00 << RV3028_BIT_TSE) | (0x00 << RV3028_BIT_EIE), 
				    p_Device);
    if(ErrorCode != RV3028_NO_ERROR)
    {
	return ErrorCode;
    }

    // Clear the EVF and BSF bits in the STATUS register
    ErrorCode = RV3028_ModifyRegister(RV3028_REG_STATUS, 
				    (0x01 << RV3028_BIT_BSF) | (0x01 << RV3028_BIT_EVF),
				    (0x00 << RV3028_BIT_BSF) | (0x00 << RV3028_BIT_EVF), 
				    p_Device);
    if(ErrorCode != RV3028_NO_ERROR)
    {
	return ErrorCode;
    }

    // Initialize the time stamp function and reset the time stamp register
    ErrorCode = RV3028_ModifyRegister(RV3028_REG_EVENTCONTROL, 
				    (0x01 << RV3028_BIT_TSR) | (0x01 << RV3028_BIT_TSOW) |  (0x01 << RV3028_BIT_TSS),
				    (0x01 << RV3028_BIT_TSR) | (OverWrite << RV3028_BIT_TSOW) | ((Mode & 0x01) << RV3028_BIT_TSS), 
				    p_Device);
    if(ErrorCode != RV3028_NO_ERROR)
    {
	return ErrorCode;
    }

    p_Device->TSMode = Mode;
    p_Device->IsTSOverwriteEnabled = OverWrite;

    // Enable the time stamp function by setting TSE in the CONTROL2 register
    ErrorCode = RV3028_ModifyRegister(RV3028_REG_CONTROL2, 0x01 << RV3028_BIT_TSE, 0x01 << RV3028_BIT_TSE, p_Device);
    if(ErrorCode != RV3028_NO_ERROR)
    {
	return ErrorCode;
    }

    p_Device->IsTSEnabled = true;

    return RV3028_NO_ERROR;
}

rv3028_error_t RV3028_DisableTS(rv3028_t* p_Device)
{
    if(p_Device->IsTSEnabled == false)
    {
	return RV3028_NOT_READY;
    }

    ErrorCode = RV3028_ModifyRegister(RV3028_REG_CONTROL2, 0x01 << RV3028_BIT_TSE, 0x00 << RV3028_BIT_TSE, p_Device);
    if(ErrorCode != RV3028_NO_ERROR)
    {
	return ErrorCode;
    }

    p_Device->IsTSEnabled = false;

    return RV3028_NO_ERROR;
}

rv3028_error_t RV3028_GetTS(rv3028_t* p_Device, struct tm* p_Time, uint8_t* p_Count)
{
    uint8_t Temp[7];

    if(p_Device->IsTSEnabled == false)
    {
	return RV3028_NOT_READY;
    }
    else if((p_Time == NULL) || (p_Count == NULL))
    {
	return RV3028_INVALID_PARAM;
    }

    ErrorCode = RV3028_ReadRegister(RV3028_REG_COUNT_TS, Temp, sizeof(Temp), p_Device);
    if(ErrorCode != RV3028_NO_ERROR)
    {
	return ErrorCode;
    }

    // Clear the EVT flag when overwrite is not set
    if(p_Device->IsTSOverwriteEnabled == false)
    {
	ErrorCode = RV3028_ClearFlags(p_Device, RV3028_FLAG_EVENT);
	if(ErrorCode != RV3028_NO_ERROR)
	{
	    return ErrorCode;
	}
    }

    *p_Count = Temp[0];
    p_Time->tm_sec = BCDToDecimal(Temp[1]);
    p_Time->tm_min = BCDToDecimal(Temp[2]);
    p_Time->tm_hour = BCDToDecimal(Temp[3]);
    p_Time->tm_mday = BCDToDecimal(Temp[4]);
    p_Time->tm_mon = BCDToDecimal(Temp[5]);
    p_Time->tm_year = BCDToDecimal(Temp[6]);

    return RV3028_NO_ERROR;
}

rv3028_error_t RV3028_EnableAlarm(rv3028_t* p_Device, rv3028_alarm_t* p_Alarm)
{
    // Clear the AIE bit in the CONTROL2 register
    ErrorCode = RV3028_ModifyRegister(RV3028_REG_CONTROL2, 0x01 << RV3028_BIT_AIE, 0x00 << RV3028_BIT_AIE, p_Device);
    if(ErrorCode != RV3028_NO_ERROR)
    {
	return ErrorCode;
    }

    // Clear the AF bit in the STATUS register
    ErrorCode = RV3028_ModifyRegister(RV3028_REG_STATUS, 0x01 << RV3028_BIT_AF, 0x00 << RV3028_BIT_AF, p_Device);
    if(ErrorCode != RV3028_NO_ERROR)
    {
	return ErrorCode;
    }

    ErrorCode = RV3028_ModifyRegister(RV3028_REG_CONTROL1, 0x01 << RV3028_BIT_WADA, p_Alarm->UseDateAlarm << RV3028_BIT_WADA, p_Device);
    if(ErrorCode != RV3028_NO_ERROR)
    {
	return ErrorCode;
    }

    // Configure the minutes alarm
    if(p_Alarm->EnableMinutesAlarm)
    {
	uint8_t Temp = DecimalToBCD(p_Alarm->Minutes);

	ErrorCode = RV3028_WriteRegister(RV3028_REG_MINUTES_ALARM, &Temp, sizeof(Temp), p_Device);
	if(ErrorCode != RV3028_NO_ERROR)
	{
	    return ErrorCode;
	}

	p_Device->IsAlarmEnabled = true;
    }

    // Configure the hours alarm
    if(p_Alarm->EnableHoursAlarm)
    {
	uint8_t Temp = 0x00;

	if(p_Device->HourMode == RV3028_HOURMODE_24)
	{
	    Temp = DecimalToBCD(p_Alarm->Hours & 0x3F);
	}
	else
	{
	    Temp = DecimalToBCD(p_Alarm->Hours & 0x1F);
	    Temp |= p_Alarm->PM << RV3028_BIT_AMPM;
	}

	ErrorCode = RV3028_WriteRegister(RV3028_REG_HOURS_ALARM, &Temp, sizeof(Temp), p_Device);
	if(ErrorCode != RV3028_NO_ERROR)
	{
	    return ErrorCode;
	}

	p_Device->IsAlarmEnabled = true;
    }

    // Configure the weekday / date alarm
    if(p_Alarm->EnableDayAlarm)
    {
	uint8_t Temp = 0x00;

	if(p_Alarm->UseDateAlarm)
	{
	    Temp = DecimalToBCD(p_Alarm->Day & 0x3F);
	}
	else
	{
	    Temp = DecimalToBCD(p_Alarm->Day & 0x07);
	}

	ErrorCode = RV3028_WriteRegister(RV3028_REG_WEEKDAY_ALARM, &Temp, sizeof(Temp), p_Device);
	if(ErrorCode != RV3028_NO_ERROR)
	{
	    return ErrorCode;
	}

	p_Device->IsAlarmEnabled = true;
    }

    if(p_Alarm->EnableInterrupts)
    {
	ErrorCode = RV3028_ModifyRegister(RV3028_REG_CONTROL2, 0x01 << RV3028_BIT_AIE, 0x00 << RV3028_BIT_AIE, p_Device);
    }

    return ErrorCode;
}

rv3028_error_t RV3028_DisableAlarm(rv3028_t* p_Device, bool Minutes, bool Hours, bool Days)
{
    uint8_t Temp = 0x00;
    bool IsMinuteAlarmEnabled = p_Device->IsAlarmEnabled;
    bool IsHourAlarmEnabled = p_Device->IsAlarmEnabled;
    bool IsDayAlarmEnabled = p_Device->IsAlarmEnabled;

    if(p_Device->IsAlarmEnabled == false)
    {
	return RV3028_NOT_READY;
    }

    if(Minutes)
    {
	ErrorCode = RV3028_ModifyRegister(RV3028_REG_MINUTES_ALARM, 0x01 << RV3028_BIT_AE_M, 0x01 << RV3028_BIT_AE_M, p_Device);
	if(ErrorCode != RV3028_NO_ERROR)
	{
	    return ErrorCode;
	}

	IsMinuteAlarmEnabled = false;
    }

    if(Hours)
    {
	ErrorCode = RV3028_ModifyRegister(RV3028_REG_HOURS_ALARM, 0x01 << RV3028_BIT_AE_H, 0x01 << RV3028_BIT_AE_H, p_Device);
	if(ErrorCode != RV3028_NO_ERROR)
	{
	    return ErrorCode;
	}

	IsHourAlarmEnabled = false;
    }

    if(Days)
    {
	ErrorCode = RV3028_ModifyRegister(RV3028_REG_WEEKDAY_ALARM, 0x01 << RV3028_BIT_AE_WD, 0x01 << RV3028_BIT_AE_WD, p_Device);
	if(ErrorCode != RV3028_NO_ERROR)
	{
	    return ErrorCode;
	}

	IsDayAlarmEnabled = false;
    }


    p_Device->IsAlarmEnabled = IsMinuteAlarmEnabled & IsHourAlarmEnabled & IsDayAlarmEnabled;

    return RV3028_WriteRegister(RV3028_REG_MINUTES_ALARM, &Temp, sizeof(uint8_t), p_Device);
}
