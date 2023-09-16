// by wuwbobo2021 <https://github.com/wuwbobo2021>, <wuwbobo@outlook.com>
// If you have found bugs in this program, please pull an issue, or contact me.

#ifndef BL0940_H
#define BL0940_H

#ifdef __cplusplus
extern "C" {
#else
#include <stdbool.h>
#endif

#include <stdint.h>

typedef enum {
	BL0940_Filter_AC_Pass = 0b00, //default
	BL0940_Filter_DC_Pass = 0b10,
	BL0940_Filter_All = 0b11
} BL0940_Filter;

typedef enum {
	BL0940_Av_Time_400ms = 0, //default
	BL0940_Av_Time_800ms = 1
} BL0940_Av_Time;

typedef enum {
	BL0940_AC_Freq_50Hz = 0, //default
	BL0940_AC_Freq_60Hz = 1
} BL0940_AC_Freq;

typedef enum {
	BL0940_Error_None,
	BL0940_Error_Read_Len,
	BL0940_Error_Read_Crc,
	BL0940_Error_Write,
	BL0940_Error_Write_Reg
} BL0940_Error;

typedef struct {
	uint8_t uart_port_num;
	
	float voltage_divider; // = (R2 + R1)/R1, the input is voltage on R1
	float r_shunt_ohm; //for example: 0.001 ohm
	
	// call bl0940_apply_settings() after changing values below
	BL0940_Filter setting_filter;
	BL0940_Av_Time setting_av_time;
	BL0940_AC_Freq setting_ac_freq;
	
	// values below are set by this module
	BL0940_Error error;
	
	float voltage;
	float current;
	float phase_angle; //degree
	float power_factor;
	float power;
	float energy; //kWh
	float temp_internal; //celsius
} BL0940;

// implement them as synchronous functions outside this module. 4800,8,N,1.
// it is better to clear the Rx buffer before sending data in bl0940_uart_send().
extern bool bl0940_uart_send(uint8_t port_num, const void* data, uint8_t cnt);
extern uint8_t bl0940_uart_receive(uint8_t port_num, void* data, uint8_t cnt, uint16_t timeout_ms);

extern bool bl0940_apply_settings(BL0940* bl0940);
extern bool bl0940_get_readings(BL0940* bl0940);

#ifdef __cplusplus
}
#endif

#endif  //BL0940_H
