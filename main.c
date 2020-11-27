#include <stdio.h>
#include "boards.h"
#include "app_error.h"
#include "app_util_platform.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "RV3028.h"

#define PASSWORD		0x20

struct tm CurrentTime = 
{
    .tm_sec = 0,
    .tm_min = 27,
    .tm_hour = 15,
    .tm_wday = 4,
    .tm_mday = 26,
    .tm_mon = 11,
    .tm_year = 20,
};

rv3028_error_t ErrorCode;
rv3028_t RTC;
rv3028_init_t RTC_Init = {
    // Use this settings to enable the battery backup
    .BatteryMode = RV3028_BAT_DSM,
    .Resistance = RV3028_TCT_3K,
    .EnableBSIE = true,
    .EnableCharge = true,

    // Use this settings to configure the time stamp function
    //.TSMode = RV3028_TS_BAT,
    .EnableTS = true,
    //.EnableTSOverwrite = true,

    // Use this settings to enable the clock output
    .Frequency = RV3028_CLKOUT_8KHZ,
    .EnableClkOut = true,

    // Use this settings for the event input configuration
    .EnableEventInt = true,
    .EventHighLevel = false,
    .Filter = RV3028_FILTER_256HZ,

    // Set the current time
    .HourMode = RV3028_HOURMODE_24,
    .p_CurrentTime = &CurrentTime,

    // Use this settings for the Power On Reset interrupt
    .EnablePOR = false,

    // Use this settings for the password function
    .Password = PASSWORD,
};

static const nrf_drv_twi_t TWI = NRF_DRV_TWI_INSTANCE(0);

void TWI_Init(void)
{
    uint8_t Dummy;
    const nrf_drv_twi_config_t Config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    APP_ERROR_CHECK(nrf_drv_twi_init(&TWI, &Config, NULL, NULL));
    nrf_drv_twi_enable(&TWI);
}

rv3028_error_t RV3028_Write(uint8_t Device_Addr, uint8_t Reg_Addr, const uint8_t* p_Reg_Data, uint32_t Length)
{
    uint8_t Temp[8] = {Reg_Addr};

    for(uint8_t i = 0x01; i < (Length + 0x01); i++)
    {
	Temp[i] = *(p_Reg_Data++);
    }

    APP_ERROR_CHECK(nrf_drv_twi_tx(&TWI, Device_Addr, Temp, Length + 0x01, false));
    while(nrf_drv_twi_is_busy(&TWI));

    return RV3028_NO_ERROR;
}

rv3028_error_t RV3028_Read(uint8_t Device_Addr, uint8_t Reg_Addr, uint8_t* p_Reg_Data, uint32_t Length)
{
    uint8_t Register = Reg_Addr;

    APP_ERROR_CHECK(nrf_drv_twi_tx(&TWI, Device_Addr, &Register, sizeof(uint8_t), true));
    while(nrf_drv_twi_is_busy(&TWI));
    APP_ERROR_CHECK(nrf_drv_twi_rx(&TWI, Device_Addr, p_Reg_Data, Length));
    while(nrf_drv_twi_is_busy(&TWI));

    return RV3028_NO_ERROR;
}

rv3028_error_t RV3028_Interface(rv3028_t* p_Device)
{
    if(p_Device == NULL)
    {
	return RV3028_INVALID_PARAM;
    }

    p_Device->p_Read = RV3028_Read;
    p_Device->p_Write = RV3028_Write;
    p_Device->DeviceAddr = RV3028_ADDRESS;

    return RV3028_NO_ERROR;
}

int main(void)
{
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("--- RV3028 RTC example ---");
    TWI_Init();

    if(RV3028_Interface(&RTC) == RV3028_NO_ERROR)
    {
	RV3028_DisableWP(&RTC, PASSWORD);

	NRF_LOG_INFO("RTC initialized...");
	ErrorCode = RV3028_Init(&RTC_Init, &RTC);
	if(ErrorCode == RV3028_NO_ERROR)
	{
	    NRF_LOG_INFO("RTC initialized...");
	    NRF_LOG_INFO("  HID: %u", RTC.HID);
	    NRF_LOG_INFO("  VID: %u", RTC.VID);

	    nrf_delay_ms(1000);
	    NRF_LOG_INFO("  Unlocking device...");
	    RV3028_UnlockWP(&RTC, PASSWORD);
	    RV3028_EnableClkOut(&RTC, false, false);

	    while(true)
	    {
		uint8_t Status;
		uint8_t TSCount;
		struct tm LastTS;

		// Get the status flags
		RV3028_GetFlags(&RTC, &Status);
		NRF_LOG_INFO("  Status: 0x%x", Status);

		// Check for a Power On Reset and clear the flag
		if(Status & RV3028_FLAG_POR)
		{
		    NRF_LOG_INFO("  Power On Reset...");
		    RV3028_ClearFlags(&RTC, RV3028_FLAG_POR);
		}
		else if(Status & RV3028_FLAG_BATTERY)
		{
		    RV3028_ClearFlags(&RTC, RV3028_FLAG_BATTERY);
		    NRF_LOG_INFO("  Battery switchover occured...");

		    if(RTC.IsTSEnabled)
		    {
			RV3028_GetTS(&RTC, &LastTS, &TSCount);
			NRF_LOG_INFO("  Last time stamp: %u:%u:%u", LastTS.tm_hour, LastTS.tm_min, LastTS.tm_sec);
		    }
		}
		else if(Status & RV3028_FLAG_EVENT)
		{
		    NRF_LOG_INFO("  Event...");
		    RV3028_ClearFlags(&RTC, RV3028_FLAG_EVENT);

		    if(RTC.IsTSEnabled)
		    {
			RV3028_GetTS(&RTC, &LastTS, &TSCount);
			NRF_LOG_INFO("  Last time stamp: %u:%u:%u", LastTS.tm_hour, LastTS.tm_min, LastTS.tm_sec);
		    }
		}

		RV3028_GetTime(&RTC, &CurrentTime);
		NRF_LOG_INFO("  Current time: %u:%u:%u", CurrentTime.tm_hour, CurrentTime.tm_min, CurrentTime.tm_sec);

		NRF_LOG_FLUSH();

		nrf_delay_ms(1000);
	    }
	}
	else
	{
	    NRF_LOG_INFO("Can not initialize RTC. Error: %u", ErrorCode);
	    NRF_LOG_FLUSH();
	}
    }
}