// by wuwbobo2021 <https://github.com/wuwbobo2021>, <wuwbobo@outlook.com>
// If you have found bugs in this program, please pull an issue, or contact me.

#include "bl0940.h"

#include <math.h>
#define PI		3.14159265358979

#define BL0940_Head_Read                    0x58
#define BL0940_Head_Write                   0xA8

#define BL0940_Reg_Addr_Write_Protect       0x1A
#define BL0940_Unlock_User_Reg              0x55 //write to the write protect register

#define BL0940_Reg_Addr_Mode_Settings       0x18
#define BL0940_Reg_Mode_Pos_I_Filter        0
#define BL0940_Reg_Mode_Pos_V_Filter        4
#define BL0940_Reg_Mode_Pos_Av_Time         8
#define BL0940_Reg_Mode_Pos_AC_Freq         9

#define BL0940_Get_Readings                 0xAA //at the position of register index
#define BL0940_Head_Readings                0x55
#define BL0940_Len_Readings                 35

// each of them takes 3 bytes (little endian)
#define BL0940_Readings_Pos_I_Fast           1 //TODO
#define BL0940_Readings_Pos_I                4
#define BL0940_Readings_Pos_V               10
#define BL0940_Readings_Pos_W               16
#define BL0940_Readings_Pos_Pulses          22
#define BL0940_Readings_Pos_Temp_Internal   28
#define BL0940_Readings_Pos_Temp_External   31

#define BL0940_Vref                         1.218

#define BL0940_Coefficient_V                (BL0940_Vref/79931.0/1000)
#define BL0940_Coefficient_I                (BL0940_Vref/324004.0/1000)
#define BL0940_Coefficient_W                (BL0940_Vref*BL0940_Vref/4046.0/1000.0/1000)
#define BL0940_Coefficient_kWh              (BL0940_Coefficient_W * 1638.4*256.0/3600000)

#define BL0940_Reg_Addr_Angle               0x0C
#define BL0940_Freq_Sample                  1000000

#define BL0940_Timeout_ms                   500

static uint32_t read_register(BL0940* bl0940, uint8_t reg_addr);
static bool write_register(BL0940* bl0940, uint8_t reg_addr, uint32_t val);
static uint16_t bytes_sum(void* data, uint8_t cnt);
static uint8_t get_sum_byte(uint16_t sum);
static void set_triple_bytes(uint8_t* data, uint32_t val);
static uint32_t get_triple_bytes(const uint8_t* data);
static void set_bits(uint32_t* p_reg, uint32_t val, uint8_t pos, uint8_t size);

bool bl0940_apply_settings(BL0940* bl0940)
{
	bl0940->error = BL0940_Error_None;
	
	uint32_t reg_mode = read_register(bl0940, BL0940_Reg_Addr_Mode_Settings);
	if (bl0940->error) return false;
	
	set_bits(&reg_mode, bl0940->setting_filter, BL0940_Reg_Mode_Pos_I_Filter, 2);
	set_bits(&reg_mode, bl0940->setting_filter, BL0940_Reg_Mode_Pos_V_Filter, 2);
	set_bits(&reg_mode, bl0940->setting_av_time, BL0940_Reg_Mode_Pos_Av_Time, 1);
	set_bits(&reg_mode, bl0940->setting_ac_freq, BL0940_Reg_Mode_Pos_AC_Freq, 1);
	
	return write_register(bl0940, BL0940_Reg_Addr_Mode_Settings, reg_mode);
}

bool bl0940_get_readings(BL0940* bl0940)
{
	bl0940->error = BL0940_Error_None;
	
	uint8_t tx_data[] = {BL0940_Head_Read, BL0940_Get_Readings};
	if (! bl0940_uart_send(bl0940->uart_port_num, tx_data, sizeof(tx_data))) {
		bl0940->error = BL0940_Error_Write; return false;
	}
	
	uint8_t rx_data[BL0940_Len_Readings]; uint8_t cnt_rx;
    cnt_rx = bl0940_uart_receive(bl0940->uart_port_num, rx_data,
	                             sizeof(rx_data), BL0940_Timeout_ms);
	if (cnt_rx < sizeof(rx_data)) {
		bl0940->error = BL0940_Error_Read_Len; return false;
	}
	
	uint16_t sum = BL0940_Head_Read;
	sum += bytes_sum(rx_data, sizeof(rx_data) - 1);
	uint8_t* p_crc = &(rx_data[sizeof(rx_data) - 1]);
	if (*p_crc != get_sum_byte(sum)) {
		bl0940->error = BL0940_Error_Read_Crc; return false;
	}
	
	bl0940->voltage =   get_triple_bytes(rx_data + BL0940_Readings_Pos_V)
	                  * BL0940_Coefficient_V * bl0940->voltage_divider;
	
	bl0940->current =   get_triple_bytes(rx_data + BL0940_Readings_Pos_I)
	                  * BL0940_Coefficient_I / bl0940->r_shunt_ohm;
	
	bl0940->power =   get_triple_bytes(rx_data + BL0940_Readings_Pos_W)
	                * BL0940_Coefficient_W * bl0940->voltage_divider / bl0940->r_shunt_ohm;
	
	bl0940->energy =   get_triple_bytes(rx_data + BL0940_Readings_Pos_Pulses)
	                 * BL0940_Coefficient_kWh * bl0940->voltage_divider / bl0940->r_shunt_ohm;
	
	bl0940->temp_internal = get_triple_bytes(rx_data + BL0940_Readings_Pos_Temp_Internal);
	bl0940->temp_internal = (170.0/448)*(bl0940->temp_internal/2 - 32) - 45;
	
	float data_angle = read_register(bl0940, BL0940_Reg_Addr_Angle) & 0xFFFF;
	if (bl0940->error) return false;
	
	data_angle *= (bl0940->setting_ac_freq == BL0940_AC_Freq_50Hz ? 50 : 60);
	data_angle /= BL0940_Freq_Sample;
	bl0940->phase_angle = 360.0*data_angle;
	bl0940->power_factor = cos(2*PI*data_angle);
	
	return true;
}

static uint32_t read_register(BL0940* bl0940, uint8_t reg_addr)
{
	uint8_t tx_data[] = {BL0940_Head_Read, reg_addr};
	if (! bl0940_uart_send(bl0940->uart_port_num, tx_data, sizeof(tx_data))) {
		bl0940->error = BL0940_Error_Write; return 0;
	}
	
	uint8_t rx_data[4]; uint8_t cnt_rx;
	cnt_rx = bl0940_uart_receive(bl0940->uart_port_num, rx_data,
	                             sizeof(rx_data), BL0940_Timeout_ms);
	if (cnt_rx < sizeof(rx_data)) {
		bl0940->error = BL0940_Error_Read_Len; return 0;
	}
	
	uint16_t sum = 0;
	sum += bytes_sum(tx_data, sizeof(tx_data));
	sum += bytes_sum(rx_data, sizeof(rx_data) - 1);
	
	uint8_t* p_crc = &(rx_data[sizeof(rx_data) - 1]);
	if (*p_crc != get_sum_byte(sum)) {
		bl0940->error = BL0940_Error_Read_Crc; return 0;
	}
	
	return get_triple_bytes(rx_data);
}

static bool write_register(BL0940* bl0940, uint8_t reg_addr, uint32_t val)
{
	if (reg_addr != BL0940_Reg_Addr_Write_Protect)
		write_register(bl0940, BL0940_Reg_Addr_Write_Protect, BL0940_Unlock_User_Reg);
	
	uint8_t tx_data[6] = {BL0940_Head_Write, reg_addr};
	set_triple_bytes(tx_data + 2, val);
	uint8_t* p_crc = &(tx_data[sizeof(tx_data) - 1]);
	*p_crc = get_sum_byte(bytes_sum(tx_data, sizeof(tx_data) - 1));
	
	if (! bl0940_uart_send(bl0940->uart_port_num, tx_data, sizeof(tx_data))) {
		bl0940->error = BL0940_Error_Write; return false;
	}
	
	if (reg_addr == BL0940_Reg_Addr_Write_Protect)
		return true;
	
	uint32_t reread = read_register(bl0940, reg_addr);
	if (reread != val) {
		bl0940->error = BL0940_Error_Write_Reg; return false;
	}
	
	write_register(bl0940, BL0940_Reg_Addr_Write_Protect, 0x00);
	return true;
}

static uint16_t bytes_sum(void* data, uint8_t cnt)
{
	if (data == (void*)0 || cnt == 0) return 0;
	
	uint8_t* p = (uint8_t*)data;
	uint16_t sum = 0;
	for (int i = 0; i < cnt; i++)
		sum += p[i];
	return sum;
}

static uint8_t get_sum_byte(uint16_t sum)
{
	return ~(uint8_t)(sum & 0xFF);
}

static uint32_t get_triple_bytes(const uint8_t* data)
{
	if (data == (void*)0) return 0;
	
	return ((uint32_t)data[2] << 16) | 
		   ((uint32_t)data[1] << 8) | 
		   ((uint32_t)data[0]);
}

static void set_triple_bytes(uint8_t* data, uint32_t val)
{
	if (data == (void*)0) return;
	
	data[0] = val & 0xFF;
	data[1] = (val >> 8) & 0xFF;
	data[2] = (val >> 16) & 0xFF;
}

static void set_bits(uint32_t* p_reg, uint32_t val, uint8_t pos, uint8_t size)
{
	if (p_reg == (void*)0) return;
	if (pos >= 32) return;
	if (pos + size > 32) size = 32 - pos;

	// in the mask, set positions to be changed to 0, set others to 1
	uint32_t mask = 0;
	if (pos + size < 32)
		mask = UINT32_MAX << (pos + size); //'1's at left
	if (pos > 0)
		mask |= UINT32_MAX >> (32 - pos); //'1's at right

	*p_reg &= mask;
	*p_reg |= (val << pos) & (~mask);
}
