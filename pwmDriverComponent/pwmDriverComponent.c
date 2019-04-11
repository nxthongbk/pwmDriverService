#include "legato.h"
#include "interfaces.h"
#include "i2c-utils.h"
#include <stdio.h>
#include <math.h>

#define PWM_DEFAULT_I2C_ADDR	0x7f

#define PWM_MODE1_REG	0x00
#define PWM_MODE2_REG	0x01
#define PWM_LED0_ON_L	0x06
#define PWM_ALL_LED_ON_L	0xfa
#define PWM_PRESCALE	0xfe

#define PWM_SLEEP_BIT	0x10
#define PWM_EXTCLK_BIT	0x40  
#define PWM_RESTART_BIT	0x80 

#define I2C_HUB_PORT_RASPI	0x08
#define I2C_HUB_PORT_IOT	0x01
#define I2C_HUB_PORT_GPIO	0x04
#define I2C_HUB_PORT_USB_HUB	0x02
#define I2C_HUB_PORT_ALL	0x0F

uint16_t	min_pulse_width = 0;
uint16_t	max_pulse_width = 0;
uint16_t	max_servo_degree = 0;

char laser_sensor_i2c_bus[256] = "/dev/i2c-0";

/**
 * Function: Configure I2C hub to enable I2C bus that connected to led matrix
 * Params:  - hub_address: I2C address of hub
 *      - port: Bus ports
 **/
int i2c_hub_select_port(uint8_t hub_address, uint8_t port)
{
	int result;
	int i2c_fd = open(laser_sensor_i2c_bus, O_RDWR);
	if (i2c_fd < 0) {
		LE_ERROR("i2cSendByte: failed to open %s\n", 
			laser_sensor_i2c_bus);
	}
	if (ioctl(i2c_fd, I2C_SLAVE_FORCE, hub_address) < 0) {
		LE_ERROR("Could not set address to 0x%02x: %s\n",
			hub_address,strerror(errno));
		return -1;
	}
	int writeResult = i2c_smbus_write_byte(i2c_fd, port);
	if (writeResult < 0) {
		LE_ERROR("smbus write failed with error %d\n", writeResult);
		result = -1;
	} else {
		result = 0;
	}
	close(i2c_fd);
	return result;
}

int I2C_write_byte(uint8_t addr, uint8_t reg, uint8_t data)
{
	int res = 0;
	int i2c_fd = open(laser_sensor_i2c_bus, O_RDWR);
	if (i2c_fd < 0) {
		printf("i2cSendByte: failed to open %s \n", 
			laser_sensor_i2c_bus);
	}
	if (ioctl(i2c_fd, I2C_SLAVE_FORCE, addr) < 0) {
		printf("Could not set address to 0x%02x: %s\n",
			addr, strerror(errno));
		close(i2c_fd);
		return -1;
	}
	res = i2c_smbus_write_byte_data(i2c_fd, reg, data);
	close(i2c_fd);
	return res;
}

int I2C_write_bytes(uint8_t addr, uint8_t reg, uint8_t length, uint8_t* data)
{
	int res = 0;
	int i2c_fd = open(laser_sensor_i2c_bus, O_RDWR);
	if (i2c_fd < 0) {
		printf("i2cSendByte: failed to open %s \n", 
				laser_sensor_i2c_bus);
	}
	if (ioctl(i2c_fd, I2C_SLAVE_FORCE, addr) < 0) {
		printf("Could not set address to 0x%02x: %s\n",
			addr, strerror(errno));
		close(i2c_fd);
		return -1;
	}
	res = i2c_smbus_write_i2c_block_data(i2c_fd, reg, length, data);
	close(i2c_fd);
	return res;
}

void restart()
{
	I2C_write_byte(PWM_DEFAULT_I2C_ADDR, PWM_MODE1_REG, PWM_RESTART_BIT);
	usleep(10);

	// Set to sleep first, to turn off internal oscillator
	I2C_write_byte(PWM_DEFAULT_I2C_ADDR, PWM_MODE1_REG, PWM_SLEEP_BIT);
	usleep(2);

	// Write logic 1 to both SLEEP bit and EXTCLK bit, the switch is now made. Enable the external oscillator
	I2C_write_byte(PWM_DEFAULT_I2C_ADDR, PWM_MODE1_REG, PWM_EXTCLK_BIT | PWM_SLEEP_BIT);

	// Turn on oscillator now
	I2C_write_byte(PWM_DEFAULT_I2C_ADDR, PWM_MODE1_REG, 0x00);
	usleep(2);
}


void I2C_read_byte(uint8_t addr, uint8_t reg, uint8_t *data)
{
	int i2c_fd = open(laser_sensor_i2c_bus, O_RDWR);
	if (i2c_fd < 0) {
		printf("i2cSendByte: failed to open %s \n", 
				laser_sensor_i2c_bus);
	}
	if (ioctl(i2c_fd, I2C_SLAVE_FORCE, addr) < 0) {
		printf("Could not set address to 0x%02x: %s\n",
			addr, strerror(errno));
		close(i2c_fd);
	}
	*data = i2c_smbus_read_byte_data(i2c_fd, reg);
	close(i2c_fd);
}

void set_frequency(uint16_t freq)
{
	if (freq < 24) 
		freq = 24;
	else if (freq > 1526) 
		freq = 1526;
	uint8_t data = 0;
	float prescaleval = (25000000.0/4096.0)/freq - 1;
	uint8_t pre_scale = floor(prescaleval + 0.5);

	I2C_read_byte(PWM_DEFAULT_I2C_ADDR, PWM_MODE1_REG, &data);

	// PRE_SCALE can only be set when SLEEP is logic 1
	// go to sleep
	I2C_write_byte(PWM_DEFAULT_I2C_ADDR, PWM_MODE1_REG, (data & 0x7F) | 0x10);
	usleep(2);
	I2C_write_byte(PWM_DEFAULT_I2C_ADDR, PWM_PRESCALE, pre_scale);
	
	I2C_write_byte(PWM_DEFAULT_I2C_ADDR, PWM_MODE1_REG, data);
	usleep(2);

	I2C_write_byte(PWM_DEFAULT_I2C_ADDR, PWM_MODE1_REG, data|0xa0);
	usleep(2);
}

void led_init()
{
	restart();
	set_frequency(1000);
	i2c_hub_select_port(0x71, I2C_HUB_PORT_ALL);
}


void set_pwm(uint8_t pin, uint16_t led_on, uint16_t led_off)
{
	if (pin < 1) 
		pin = 1;
	else if (pin > 16) 
		pin=16;
	pin = pin - 1;

	uint8_t buffer[4];

	buffer[0] = led_on;
	buffer[1] = led_on >> 8;
	buffer[2] = led_off;
	buffer[3] = led_off >> 8;

	I2C_write_bytes(PWM_DEFAULT_I2C_ADDR, PWM_LED0_ON_L+4*pin, 4, buffer);
}

void set_servo_pulse_range(uint16_t min_pulse, uint16_t max_pulse, uint16_t max_degree)
{
	float min, max;
	min = (2.46 * min_pulse)/10.0 - 1;
	max = (2.46 * max_pulse)/10.0 - 1;
	min_pulse_width = min;
	max_pulse_width = max;
	max_servo_degree = max_degree;
}
void ma_pwm_set_angle(uint8_t pin, uint16_t angle)
{
	if (angle > max_servo_degree) 
		angle = max_servo_degree;
	set_pwm(pin, 0, angle*((max_pulse_width-min_pulse_width)/max_servo_degree)+min_pulse_width);
}

void ma_pwm_servo_init()
{
	restart();
	set_servo_pulse_range(500, 2500, 180);
	set_frequency(50);
	i2c_hub_select_port(0x71, I2C_HUB_PORT_ALL);
}

COMPONENT_INIT
{
	LE_INFO("PWM Driver Start");
}