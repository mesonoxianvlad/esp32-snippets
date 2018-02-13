#include <freertos/FreeRTOS.h>

static void i2c_master_init();
static void bmp180_init_cal_data();

static void read_n_bytes(uint8_t, uint8_t, uint8_t*);
static uint8_t read_8(uint8_t);
static int16_t read_16(uint8_t);
static int32_t read_24(uint8_t);

static void set_read_command(uint8_t);
static int32_t bmp180_read_uncompensated_temp();
static int32_t bmp180_read_uncompensated_pressure();

static int32_t get_b5(int32_t);
static double calculate_calibrated_temp(int32_t);
static int32_t calculate_calibrated_pressure(int32_t, int32_t);

static float get_altitude(int32_t, float);
static float calculate_sea_level_pressure(int32_t, float);
static void bmp180_get_data();
static esp_err_t bmp180_is_available();

/*
    @brief Enum of addresses regarding BMP180 sensor
*/
enum
{
	/* I2C address of sensor */
	BMP180_ADDRESS					   = 0x77,

	/* Calibration register addresses */
	BMP180_REGISTER_CAL_AC1            = 0xAA,  /* 16 bits, short */
	BMP180_REGISTER_CAL_AC2            = 0xAC,  /* 16 bits, short */
	BMP180_REGISTER_CAL_AC3            = 0xAE,  /* 16 bits, short */
	BMP180_REGISTER_CAL_AC4            = 0xB0,  /* 16 bits, unsigned short */
	BMP180_REGISTER_CAL_AC5            = 0xB2,  /* 16 bits, unsigned short */
	BMP180_REGISTER_CAL_AC6            = 0xB4,  /* 16 bits, unsigned short */
	BMP180_REGISTER_CAL_B1             = 0xB6,  /* 16 bits, short */
	BMP180_REGISTER_CAL_B2             = 0xB8,  /* 16 bits, short */
	BMP180_REGISTER_CAL_MB             = 0xBA,  /* 16 bits, short */
	BMP180_REGISTER_CAL_MC             = 0xBC,  /* 16 bits, short */
	BMP180_REGISTER_CAL_MD             = 0xBE,  /* 16 bits, short */

	/* Chip info and soft reset registers */
	BMP180_REGISTER_CHIPID             = 0xD0,
	BMP180_REGISTER_VERSION            = 0xD1,
	BMP180_REGISTER_SOFTRESET          = 0xE0,

	BMP180_REGISTER_CONTROL            = 0xF4,
	BMP180_SET_WRITE_ADDRESS           = 0xEE,
	BMP180_SET_READ_ADDRESS            = 0xEF,

	/* Sensor data registers */
	BMP180_REGISTER_TEMPDATA           = 0xF6,
	BMP180_REGISTER_PRESSUREDATA       = 0xF6,
	BMP180_REGISTER_READTEMPCMD        = 0x2E,
	BMP180_REGISTER_READPRESSURECMD    = 0x34
};

/*
    @brief Pressure sampling accuracy modes.
    Refer to the datasheet below for details
    https://cdn-shop.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf
*/
typedef enum
{
  BMP180_MODE_ULTRALOWPOWER          = 0,   /* 1 sample, 4.5ms conversion time, 3uA */
  BMP180_MODE_STANDARD               = 1,   /* 2 samples, 7.5ms conversion time, 5uA */
  BMP180_MODE_HIGHRES                = 2,   /* 4 samples, 13.5ms conversion time, 7uA */
  BMP180_MODE_ULTRAHIGHRES           = 3    /* 8 samples, 25.5ms conversion time, 12uA */
} bmp180_mode_t;

typedef struct
{
    int16_t cal_AC1;
	int16_t cal_AC2;
	int16_t cal_AC3;
	uint16_t cal_AC4;
	uint16_t cal_AC5;
	uint16_t cal_AC6;
	int16_t cal_B1;
	int16_t cal_B2;
	int16_t cal_MB;
	int16_t cal_MC;
	int16_t cal_MD;

} bmp180_cal_data_t;