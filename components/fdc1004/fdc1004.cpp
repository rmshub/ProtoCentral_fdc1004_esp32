//////////////////////////////////////////////////////////////////////////////////////////
//
//    ESP-IDF library for the FDC1004 capacitance sensor breakout board
//
//    Author: rmshub
//
//    Based on original code written by Ashwin Whitchurch
//
//    This software is licensed under the MIT License(http://opensource.org/licenses/MIT).
//
//   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
//   NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
//   IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
//   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//   SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
//   For information on how to use, visit https://github.com/rmshub/ProtoCentral_fdc1004_esp32
/////////////////////////////////////////////////////////////////////////////////////////

#include "fdc1004.hpp"

#define FDC1004_UPPER_BOUND ((int16_t)0x4000)
#define FDC1004_LOWER_BOUND (-1 * FDC1004_UPPER_BOUND)

uint8_t MEAS_CONFIG[] = {0x08, 0x09, 0x0A, 0x0B};
uint8_t MEAS_MSB[] = {0x00, 0x02, 0x04, 0x06};
uint8_t MEAS_LSB[] = {0x01, 0x03, 0x05, 0x07};
uint8_t SAMPLE_DELAY[] = {11, 11, 6, 3};
static const char *TAG = "fdc1004";

FDC1004::FDC1004(uint16_t rate, int sda_io, int scl_io)
{
	this->_addr = 0b1010000;
	this->_rate = rate;
	this->_sda_gpio = sda_io;
	this->_scl_gpio = scl_io;
}

void FDC1004::begin()
{
	uint32_t i2c_clk;
	if (_rate == FDC1004_100HZ)
	{
		i2c_clk = 100000u;
	}
	else if (_rate == FDC1004_200HZ)
	{
		i2c_clk = 200000u;
	}
	else
	{
		i2c_clk = 400000u;
	}

	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = _sda_gpio;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_io_num = _scl_gpio;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = i2c_clk;
	esp_err_t err = i2c_param_config(I2C_NUM_1, &conf);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "fdc1004 begin error %s", esp_err_to_name(err));
	}
	err = i2c_driver_install(I2C_NUM_1, conf.mode, 0u, 0u, 0u);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "i2c_driver_install error %s", esp_err_to_name(err));
	}
}

void FDC1004::write16(uint8_t reg, uint16_t data)
{
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, _addr << 1 | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, reg, true);
	i2c_master_write_byte(cmd, (uint8_t)(data >> 8), true);
	i2c_master_write_byte(cmd, (uint8_t)(data & 0xFF), true);
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_NUM_1, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
}

uint16_t FDC1004::read16(uint8_t reg)
{
	uint8_t msb_data;
	uint8_t lsb_data;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (_addr << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, reg, true);
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_NUM_1, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);

	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (_addr << 1) | I2C_MASTER_READ, true);
	i2c_master_read_byte(cmd, &msb_data, I2C_MASTER_ACK);
	i2c_master_read_byte(cmd, &lsb_data, I2C_MASTER_NACK);
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_NUM_1, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	return ((msb_data << 8u) | lsb_data);
}

// configure a measurement
uint8_t FDC1004::configureMeasurementSingle(uint8_t measurement, uint8_t channel, uint8_t capdac)
{
	// Verify data
	if (!FDC1004_IS_MEAS(measurement) || !FDC1004_IS_CHANNEL(channel) || capdac > FDC1004_CAPDAC_MAX)
	{
		ESP_LOGE(TAG, "bad configuration");
		return 1;
	}

	// build 16 bit configuration
	uint16_t configuration_data;
	configuration_data = ((uint16_t)channel) << 13; // CHA
	configuration_data |= ((uint16_t)0x04) << 10;	// CHB disable / CAPDAC enable
	configuration_data |= ((uint16_t)capdac) << 5;	// CAPDAC value
	write16(MEAS_CONFIG[measurement], configuration_data);
	return 0;
}

uint8_t FDC1004::triggerSingleMeasurement(uint8_t measurement, uint8_t rate)
{
	// verify data
	if (!FDC1004_IS_MEAS(measurement) || !FDC1004_IS_RATE(rate))
	{
		ESP_LOGE(TAG, "bad trigger request");
		return 1;
	}
	uint16_t trigger_data;
	trigger_data = ((uint16_t)rate) << 10;	  // sample rate
	trigger_data |= 0 << 8;					  // repeat disabled
	trigger_data |= (1 << (7 - measurement)); // 0 > bit 7, 1 > bit 6, etc
	write16(FDC_REGISTER, trigger_data);
	return 0;
}

/**
 * Check if measurement is done, and read the measurement into value if so.
 * value should be at least 4 bytes long (24 bit measurement)
 */
uint8_t FDC1004::readMeasurement(uint8_t measurement, uint16_t *value)
{
	if (!FDC1004_IS_MEAS(measurement))
	{
		ESP_LOGE(TAG, "bad read request");
		return 1;
	}

	// check if measurement is complete
	uint16_t fdc_register = read16(FDC_REGISTER);
	if (!(fdc_register & (1 << (3 - measurement))))
	{
		ESP_LOGE(TAG, "measurement not completed");
		return 2;
	}

	// read the value
	uint16_t msb = read16(MEAS_MSB[measurement]);
	uint16_t lsb = read16(MEAS_LSB[measurement]);
	value[0] = msb;
	value[1] = lsb;
	return 0;
}

/**
 * take a measurement, uses the measurement register equal to the channel number
 */
uint8_t FDC1004::measureChannel(uint8_t channel, uint8_t capdac, uint16_t *value)
{
	uint8_t measurement = channel; // 4 measurement configs, 4 channels, seems fair
	if (configureMeasurementSingle(measurement, channel, capdac))
	{
		return 1;
	}
	if (triggerSingleMeasurement(measurement, this->_rate))
	{
		return 1;
	}
	vTaskDelay(SAMPLE_DELAY[this->_rate] / portTICK_RATE_MS);
	return readMeasurement(measurement, value);
}

/**
 *  function to get the capacitance from a channel.
 */
int32_t FDC1004::getCapacitance(uint8_t channel)
{
	fdc1004_measurement_t value;
	uint8_t result = getRawCapacitance(channel, &value);
	if (result)
		return 0x80000000;

	int32_t capacitance = ((int32_t)ATTOFARADS_UPPER_WORD) * ((int32_t)value.value); // attofarads
	capacitance /= 1000;															 // femtofarads
	capacitance += ((int32_t)FEMTOFARADS_CAPDAC) * ((int32_t)value.capdac);
	return capacitance;
}