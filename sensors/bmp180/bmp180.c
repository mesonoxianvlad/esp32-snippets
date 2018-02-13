#include <driver/i2c.h>
#include <esp_log.h>
#include <freertos/task.h>
#include "sdkconfig.h"
#include "bmp180.h"
#include "limits.h"
#include "math.h"

const gpio_num_t I2C_MASTER_SCL_IO      =	GPIO_NUM_19;               /* Master clock gpio pin */
const gpio_num_t I2C_MASTER_SDA_IO      =	GPIO_NUM_18;               /* Master data gpio pin */
const i2c_port_t I2C_MASTER_PORT        =	I2C_NUM_0;
const uint32_t I2C_MASTER_FREQ_HZ       =	100000;
const bmp180_mode_t BMP180_MODE 		= 	BMP180_MODE_ULTRAHIGHRES;	   /* Resolution of pressure sensor */
const int BMP180_DEBUG = 1;

static char tag[] = "bmp180";
static bmp180_cal_data_t cal_data;

static void i2c_master_init() {
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_PORT, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_PORT, conf.mode, 0, 0, 0));
}

static void bmp180_init_cal_data() {
	cal_data.cal_AC1 = read_16(BMP180_REGISTER_CAL_AC1);
	cal_data.cal_AC2 = read_16(BMP180_REGISTER_CAL_AC2);
	cal_data.cal_AC3 = read_16(BMP180_REGISTER_CAL_AC3);

	cal_data.cal_AC4 = (uint16_t) read_16(BMP180_REGISTER_CAL_AC4);
	cal_data.cal_AC5 = (uint16_t) read_16(BMP180_REGISTER_CAL_AC5);
	cal_data.cal_AC6 = (uint16_t) read_16(BMP180_REGISTER_CAL_AC6);

	cal_data.cal_B1 = read_16(BMP180_REGISTER_CAL_B1);
	cal_data.cal_B2 = read_16(BMP180_REGISTER_CAL_B2);
	cal_data.cal_MB = read_16(BMP180_REGISTER_CAL_MB);
	cal_data.cal_MC = read_16(BMP180_REGISTER_CAL_MC);
	cal_data.cal_MD = read_16(BMP180_REGISTER_CAL_MD);

	ESP_LOGI(tag, "Calibration: AC1: %hi, AC2: %hi, AC3: %hi, AC4: %hu, AC5: %hu, AC6: %hu, B1: %hi, B2: %hi, MB: %hi, MC: %hi, MD: %hi",
		cal_data.cal_AC1, cal_data.cal_AC2, cal_data.cal_AC3, cal_data.cal_AC4, cal_data.cal_AC5, cal_data.cal_AC6, 
		cal_data.cal_B1, cal_data.cal_B2, cal_data.cal_MB, cal_data.cal_MC, cal_data.cal_MD);
}

static void read_n_bytes(uint8_t reg, uint8_t n, uint8_t bytes[]) {
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, BMP180_SET_WRITE_ADDRESS, I2C_MASTER_ACK);
	i2c_master_write_byte(cmd, reg, I2C_MASTER_ACK);
	i2c_master_stop(cmd);
	ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, 1000/portTICK_PERIOD_MS));
	i2c_cmd_link_delete(cmd);

	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, BMP180_SET_READ_ADDRESS, I2C_MASTER_ACK);

	for (int i = 0; i < n; i++) {
		i2c_master_read_byte(cmd, &(bytes[i]), i == n - 1 ? I2C_MASTER_NACK : I2C_MASTER_ACK);
	}

	i2c_master_stop(cmd);
	ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, 1000/portTICK_PERIOD_MS));
	i2c_cmd_link_delete(cmd);
}

static uint8_t read_8(uint8_t reg) {
	uint8_t bytes[1];
	read_n_bytes(reg, 1, bytes);
	return bytes[0];
}

static int16_t read_16(uint8_t reg) {
	uint8_t bytes[2];
	read_n_bytes(reg, 2, bytes);
	return bytes[0] << 8 | bytes[1];
}

static int32_t read_24(uint8_t reg) {
	uint8_t bytes[3];
	read_n_bytes(reg, 3, bytes);
	return bytes[0] << 16 | bytes[1] << 8 | bytes[0];
}

static void set_read_command(uint8_t reg) {
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, BMP180_SET_WRITE_ADDRESS, I2C_MASTER_ACK);
	i2c_master_write_byte(cmd, BMP180_REGISTER_CONTROL, I2C_MASTER_ACK);
	i2c_master_write_byte(cmd, reg, I2C_MASTER_ACK);
	i2c_master_stop(cmd);
	ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, 1000/portTICK_PERIOD_MS));
	i2c_cmd_link_delete(cmd);
}

static int32_t bmp180_read_uncompensated_temp() {
	set_read_command(BMP180_REGISTER_READTEMPCMD);

	/* Delay at least 4.5ms for temp register to be set */
	vTaskDelay(10/portTICK_PERIOD_MS);

	//ESP_LOGI(tag, "\nTEMP: \tMSB: %i\t LSB: %i\n", msb, lsb);
	return read_16(BMP180_REGISTER_TEMPDATA);
}

static int32_t bmp180_read_uncompensated_pressure() {
	set_read_command(BMP180_REGISTER_READPRESSURECMD);

	uint8_t wait_ms;

	switch(BMP180_MODE) {
	case BMP180_MODE_ULTRALOWPOWER:
		wait_ms = 5;
		break;
	case BMP180_MODE_STANDARD:
		wait_ms = 8;
		break;
	case BMP180_MODE_HIGHRES:
		wait_ms = 14;
		break;
	case BMP180_MODE_ULTRAHIGHRES:
	default:
		wait_ms = 26;
		break;
	}
	vTaskDelay(wait_ms/portTICK_PERIOD_MS);

	int32_t u_pressure;
	if (BMP180_MODE == BMP180_MODE_ULTRAHIGHRES)
		u_pressure = read_24(BMP180_REGISTER_PRESSUREDATA);
	else 
		u_pressure = read_16(BMP180_REGISTER_PRESSUREDATA) << 8;

	return u_pressure >> (8 - BMP180_MODE);
}

static int32_t get_b5(int32_t u_temp) {
  int32_t X1 = (u_temp - (int32_t) cal_data.cal_AC6) * ((int32_t) cal_data.cal_AC5) >> 15;
  int32_t X2 = ((int32_t) cal_data.cal_MC << 11) / (X1 + (int32_t) cal_data.cal_MD);
  return X1 + X2;
}

/* Calculates true tempterature in degrees C, based on algorithm in BMP180 DataSheet */
static double calculate_calibrated_temp(int32_t b5) {
	int32_t temp_tenths = ((b5 + 8) >> 4);
	return temp_tenths / 10.0;
}

/* Calculates true pressure in Pascals, based on algorithm in BMP180 DataSheet */
static int32_t calculate_calibrated_pressure(int32_t u_pressure, int32_t b5) {
	int32_t B6 = b5 - 4000;
	int32_t X1 = ((int32_t) cal_data.cal_B2 * ( (B6 * B6) >> 12 )) >> 11;
  	int32_t X2 = ((int32_t) cal_data.cal_AC2 * B6) >> 11;
  	int32_t X3 = X1 + X2;
  	int32_t B3 = (((((int32_t) cal_data.cal_AC1) * 4 + X3) << BMP180_MODE) + 2) / 4;

	//ESP_LOGI(tag, "\nStage 1: \nB6: %i\nX1: %i\nX2: %i\nX3: %i\nB3: %i\n", B6, X1, X2, X3, B3);

	X1 = ((int32_t) cal_data.cal_AC3 * B6) >> 13;
  	X2 = ((int32_t) cal_data.cal_B1 * ((B6 * B6) >> 12)) >> 16;
  	X3 = ((X1 + X2) + 2) >> 2;

	//ESP_LOGI(tag, "\nStage 2: \nX1: %i\nX2: %i\nX3: %i\n", X1, X2, X3);

  	uint32_t B4 = ((uint32_t) cal_data.cal_AC4 * (uint32_t)(X3 + 32768)) >> 15;
  	uint32_t B7 = ((uint32_t) u_pressure - B3) * (uint32_t)( 50000 >> BMP180_MODE);

	//ESP_LOGI(tag, "\nStage 3: \nB4: %i\nB7: %i\n", B4, B7);

	int32_t p;
	if (B7 < 0x80000000) {
    	p = (B7 * 2) / B4;
  	} else {
    	p = (B7 / B4) * 2;
  	}

  	X1 = (p >> 8) * (p >> 8);
  	X1 = (X1 * 3038) >> 16;
  	X2 = (-7357 * p) >> 16;

	return p + ((X1 + X2 + (int32_t)3791) >> 4);
}

static float get_altitude(int32_t pressure, float pressure_at_sea_level) {
  return 44330 * (1.0 - pow((float) pressure / pressure_at_sea_level, 0.1903));;
}

static float calculate_sea_level_pressure(int32_t absolute_pressure, float altitude) {
	return ((float) absolute_pressure) / (pow(1 - (altitude / 44330.0), 5.255));
}

static void bmp180_get_data() {
	int32_t u_temp = bmp180_read_uncompensated_temp();
	int32_t u_pressure = bmp180_read_uncompensated_pressure();

	int32_t b5 = get_b5(u_temp);
	double c_temp = calculate_calibrated_temp(b5);
	int32_t c_pressure = calculate_calibrated_pressure(u_pressure, b5);

	float altitude = get_altitude(c_pressure, 101300);
	float sea_level_pressure = calculate_sea_level_pressure(c_pressure, 10.0);
	
	ESP_LOGI(tag, "\nTemp (C): %.02f\nPressure (Pa): %d\nPressure (hPa): %.02f\nAltitude (m): %.02f\nPressure at sea level: %f", c_temp, c_pressure, (float) c_pressure / 100.0, altitude, sea_level_pressure);
}

static esp_err_t bmp180_is_available() {
	uint8_t msb = read_8(0xD0);

	if(msb == 0x55)
		return ESP_OK;
	
	return ESP_ERR_NOT_FOUND;
}

void app_main() {
	i2c_master_init();

	// Assert sensor is connected
	ESP_ERROR_CHECK(bmp180_is_available());

    bmp180_init_cal_data();
    
	while(1) {
		bmp180_get_data();
		vTaskDelay((5000) / portTICK_PERIOD_MS);
	}

	vTaskDelete(NULL);
}