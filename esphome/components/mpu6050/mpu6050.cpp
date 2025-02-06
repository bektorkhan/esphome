#include "mpu6050.h"
#include "esphome/core/log.h"

namespace esphome {
namespace mpu6050 {

static const char *const TAG = "mpu6050";

const uint8_t MPU6050_REGISTER_WHO_AM_I = 0x75;
const uint8_t MPU6050_REGISTER_POWER_MANAGEMENT_1 = 0x6B;
const uint8_t MPU6050_REGISTER_USER_CTRL_CFG = 0x6A;
const uint8_t MPU6050_REGISTER_INT_PIN_CFG = 0x37;
const uint8_t MPU6050_REGISTER_GYRO_CONFIG = 0x1B;
const uint8_t MPU6050_REGISTER_ACCEL_CONFIG = 0x1C;
const uint8_t MPU6050_REGISTER_ACCEL_XOUT_H = 0x3B;
const uint8_t MPU6050_CLOCK_SOURCE_X_GYRO = 0b001;
const uint8_t MPU6050_SCALE_2000_DPS = 0b11;
const float MPU6050_SCALE_DPS_PER_DIGIT_2000 = 0.060975f;
const uint8_t MPU6050_RANGE_2G = 0b00;
const float MPU6050_RANGE_PER_DIGIT_2G = 0.000061f;
const uint8_t MPU6050_BIT_I2C_BYPASS_EN = 1;
const uint8_t MPU6050_BIT_I2C_MST_EN = 5;
const uint8_t MPU6050_BIT_SLEEP_ENABLED = 6;
const uint8_t MPU6050_BIT_TEMPERATURE_DISABLED = 3;
const float GRAVITY_EARTH = 9.80665f;

/** Set I2C bypass enabled status.
 * When this bit is equal to 1 and I2C_MST_EN (Register 106 bit[5]) is equal to
 * 0, the host application processor will be able to directly access the
 * auxiliary I2C bus of the MPU-60X0. When this bit is equal to 0, the host
 * application processor will not be able to directly access the auxiliary I2C
 * bus of the MPU-60X0 regardless of the state of I2C_MST_EN (Register 0x6A 106
 * bit[5]).
 * @param enabled New I2C bypass enabled status
 * @see MPU6050_RA_INT_PIN_CFG  0x37
 * @see MPU6050_INTCFG_I2C_BYPASS_EN_BIT  bit[1]
 */

void MPU6050Component::setI2CBypassEnabled(bool enabled) {
    // I2Cdev::writeBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled, wireObj);  from https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU6050
  uint8_t int_pin_cfg;
  if (!this->read_byte(MPU6050_REGISTER_INT_PIN_CFG, &int_pin_cfg)) {
    this->mark_failed();
    return;
  }
  if (!enabled ) {
    int_pin_cfg &= ~(1 << MPU6050_BIT_I2C_BYPASS_EN);
  }
  else
  {
    int_pin_cfg |= (1 << MPU6050_BIT_I2C_BYPASS_EN);
  }
  if (!this->write_byte(MPU6050_REGISTER_INT_PIN_CFG, int_pin_cfg)) {
    this->mark_failed();
    return;
  }
 }

/** Set I2C Master Mode enabled status.
 * @param enabled New I2C Master Mode enabled status
 * @see getI2CMasterModeEnabled()
 * @see MPU6050_RA_USER_CTRL 0x6A
 * @see MPU6050_USERCTRL_I2C_MST_EN_BIT  bit[5]
 */


void MPU6050Component::setI2CMasterModeEnabled(bool enabled) {
    // I2Cdev::writeBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled, wireObj);  from https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU6050
  uint8_t i2c_mst_en;
  if (!this->read_byte(MPU6050_REGISTER_USER_CTRL_CFG, &i2c_mst_en)) {
    this->mark_failed();
    return;
  }
  if (!enabled ) {
    i2c_mst_en &= ~(1 << MPU6050_BIT_I2C_MST_EN);
  }
  else
  {
    i2c_mst_en |= (1 << MPU6050_BIT_I2C_MST_EN);
  }
  if (!this->write_byte(MPU6050_REGISTER_USER_CTRL_CFG, i2c_mst_en)) {
    this->mark_failed();
    return;
  }
}

/** Set sleep mode status.
 * @param enabled New sleep mode enabled status
 * @see getSleepEnabled()
 * @see MPU6050_RA_PWR_MGMT_1 0x6B
 * @see MPU6050_PWR1_SLEEP_BIT  bit[6]
 */

void MPU6050Component::setSleepEnabled(bool enabled) {
    // I2Cdev::writeBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled, wireObj);  from https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU6050
  // Setup power management
  uint8_t power_management;
  if (!this->read_byte(MPU6050_REGISTER_POWER_MANAGEMENT_1, &power_management)) {
    this->mark_failed();
    return;
  }
  if (!enabled ) {
    power_management &= ~(1 << MPU6050_BIT_SLEEP_ENABLED);
  }
  else
  {
    power_management |= (1 << MPU6050_BIT_SLEEP_ENABLED);
  }
  if (!this->write_byte(MPU6050_REGISTER_POWER_MANAGEMENT_1, power_management)) {
    this->mark_failed();
    return;
  }
 }
  
void MPU6050Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up MPU6050...");
  uint8_t who_am_i;
  if (!this->read_byte(MPU6050_REGISTER_WHO_AM_I, &who_am_i) ||
      (who_am_i != 0x68 && who_am_i != 0x70 && who_am_i != 0x98)) {
    this->mark_failed();
    return;
  }

  // **** Mod for GY-87 Setup USER_CTRL to disable I2CMasterMode ****
  uint8_t i2c_mst_en;
  if (!this->read_byte(MPU6050_REGISTER_USER_CTRL_CFG, &i2c_mst_en)) {
    this->mark_failed();
    return;
  }
  // Diable I2CMasterMode
  // i2c_mst_en &= ~(1 << MPU6050_BIT_I2C_MST_EN);
  i2c_mst_en &= 0b11011111;
  if (!this->write_byte(MPU6050_REGISTER_USER_CTRL_CFG, i2c_mst_en)) {
    this->mark_failed();
    return;
  }
  // **** Mod for GY-87 Setup INT_PIN_CFG to enable I2CBypass ****
  uint8_t int_pin_cfg;
  if (!this->read_byte(MPU6050_REGISTER_INT_PIN_CFG, &int_pin_cfg)) {
    this->mark_failed();
    return;
  }
  // Enable I2CByPass
  // int_pin_cfg |= (1 << MPU6050_BIT_I2C_BYPASS_EN);
  int_pin_cfg |= 0b00000010;
  if (!this->write_byte(MPU6050_REGISTER_INT_PIN_CFG, int_pin_cfg)) {
    this->mark_failed();
    return;
  }

  ESP_LOGV(TAG, "  Setting up Power Management...");
  // Setup power management
  uint8_t power_management;
  if (!this->read_byte(MPU6050_REGISTER_POWER_MANAGEMENT_1, &power_management)) {
    this->mark_failed();
    return;
  }
  ESP_LOGV(TAG, "  Input power_management: 0b" BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(power_management));
  // Set clock source - X-Gyro
  power_management &= 0b11111000;
  power_management |= MPU6050_CLOCK_SOURCE_X_GYRO;
  // Disable sleep
  power_management &= 0b10111111;    // ~(1 << MPU6050_BIT_SLEEP_ENABLED);
  // Enable temperature
  power_management &= 0b11110111;               // ~(1 << MPU6050_BIT_TEMPERATURE_DISABLED);
  ESP_LOGV(TAG, "  Output power_management: 0b" BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(power_management));
  if (!this->write_byte(MPU6050_REGISTER_POWER_MANAGEMENT_1, power_management)) {
    this->mark_failed();
    return;
  }

  ESP_LOGV(TAG, "  Setting up Gyro Config...");
  // Set scale - 2000DPS
  uint8_t gyro_config;
  if (!this->read_byte(MPU6050_REGISTER_GYRO_CONFIG, &gyro_config)) {
    this->mark_failed();
    return;
  }
  ESP_LOGV(TAG, "  Input gyro_config: 0b" BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(gyro_config));
  gyro_config &= 0b11100111;
  gyro_config |= MPU6050_SCALE_2000_DPS << 3;
  ESP_LOGV(TAG, "  Output gyro_config: 0b" BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(gyro_config));
  if (!this->write_byte(MPU6050_REGISTER_GYRO_CONFIG, gyro_config)) {
    this->mark_failed();
    return;
  }

  ESP_LOGV(TAG, "  Setting up Accel Config...");
  // Set range - 2G
  uint8_t accel_config;
  if (!this->read_byte(MPU6050_REGISTER_ACCEL_CONFIG, &accel_config)) {
    this->mark_failed();
    return;
  }
  ESP_LOGV(TAG, "    Input accel_config: 0b" BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(accel_config));
  accel_config &= 0b11100111;
  accel_config |= (MPU6050_RANGE_2G << 3);
  ESP_LOGV(TAG, "    Output accel_config: 0b" BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(accel_config));
  if (!this->write_byte(MPU6050_REGISTER_ACCEL_CONFIG, accel_config)) {
    this->mark_failed();
    return;
  }
}
void MPU6050Component::dump_config() {
  ESP_LOGCONFIG(TAG, "MPU6050:");
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with MPU6050 failed!");
  }
  LOG_UPDATE_INTERVAL(this);
  LOG_SENSOR("  ", "Acceleration X", this->accel_x_sensor_);
  LOG_SENSOR("  ", "Acceleration Y", this->accel_y_sensor_);
  LOG_SENSOR("  ", "Acceleration Z", this->accel_z_sensor_);
  LOG_SENSOR("  ", "Gyro X", this->gyro_x_sensor_);
  LOG_SENSOR("  ", "Gyro Y", this->gyro_y_sensor_);
  LOG_SENSOR("  ", "Gyro Z", this->gyro_z_sensor_);
  LOG_SENSOR("  ", "Temperature", this->temperature_sensor_);
}

void MPU6050Component::update() {
  ESP_LOGV(TAG, "    Updating MPU6050...");
  uint16_t raw_data[7];
  if (!this->read_bytes_16(MPU6050_REGISTER_ACCEL_XOUT_H, raw_data, 7)) {
    this->status_set_warning();
    return;
  }
  auto *data = reinterpret_cast<int16_t *>(raw_data);

  float accel_x = data[0] * MPU6050_RANGE_PER_DIGIT_2G * GRAVITY_EARTH;
  float accel_y = data[1] * MPU6050_RANGE_PER_DIGIT_2G * GRAVITY_EARTH;
  float accel_z = data[2] * MPU6050_RANGE_PER_DIGIT_2G * GRAVITY_EARTH;

  float temperature = data[3] / 340.0f + 36.53f;

  float gyro_x = data[4] * MPU6050_SCALE_DPS_PER_DIGIT_2000;
  float gyro_y = data[5] * MPU6050_SCALE_DPS_PER_DIGIT_2000;
  float gyro_z = data[6] * MPU6050_SCALE_DPS_PER_DIGIT_2000;

  ESP_LOGD(TAG,
           "Got accel={x=%.3f m/s², y=%.3f m/s², z=%.3f m/s²}, "
           "gyro={x=%.3f °/s, y=%.3f °/s, z=%.3f °/s}, temp=%.3f°C",
           accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, temperature);

  if (this->accel_x_sensor_ != nullptr)
    this->accel_x_sensor_->publish_state(accel_x);
  if (this->accel_y_sensor_ != nullptr)
    this->accel_y_sensor_->publish_state(accel_y);
  if (this->accel_z_sensor_ != nullptr)
    this->accel_z_sensor_->publish_state(accel_z);

  if (this->temperature_sensor_ != nullptr)
    this->temperature_sensor_->publish_state(temperature);

  if (this->gyro_x_sensor_ != nullptr)
    this->gyro_x_sensor_->publish_state(gyro_x);
  if (this->gyro_y_sensor_ != nullptr)
    this->gyro_y_sensor_->publish_state(gyro_y);
  if (this->gyro_z_sensor_ != nullptr)
    this->gyro_z_sensor_->publish_state(gyro_z);

  this->status_clear_warning();
}
float MPU6050Component::get_setup_priority() const { return setup_priority::DATA; }

}  // namespace mpu6050
}  // namespace esphome
