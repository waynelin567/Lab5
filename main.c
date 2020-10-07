// Robot Template app
//
// Framework for creating applications that control the Kobuki robot

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "app_error.h"
#include "app_timer.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_spi.h"

#include "buckler.h"
#include "display.h"
#include "kobukiActuator.h"
#include "kobukiSensorPoll.h"
#include "kobukiSensorTypes.h"
#include "kobukiUtilities.h"
#include "lsm9ds1.h"

// I2C manager
NRF_TWI_MNGR_DEF(twi_mngr_instance, 5, 0);

typedef enum {
  OFF,
  DRIVING,
  TURNING,
  BACKING,
  TURNING_45,
} robot_state_t;

static float measure_distance(uint16_t current_encoder, uint16_t previous_encoder)
{
  float ret = 0;
  const float CONVERSION = 0.0006108;
  if (current_encoder >= previous_encoder) 
  {
    ret = (float)(current_encoder - previous_encoder) * CONVERSION;
  }
  else
  {
    ret = (float)(65535+current_encoder-previous_encoder)*CONVERSION;
  }
  return ret;
}
static float measure_reverse_distance(uint16_t current_encoder, uint16_t previous_encoder)
{
  float ret = 0;
  const float CONVERSION = 0.0006108;
  if (current_encoder <= previous_encoder) 
  {
    ret = (float)(current_encoder - previous_encoder) * CONVERSION;
  }
  else
  {
    ret = -(float)((65535-current_encoder)+previous_encoder)*CONVERSION;
  }
  return ret;
}
bool bumpedIntoSth(KobukiSensors_t* sensors, bool* isRight)
{
  *isRight = sensors->bumps_wheelDrops.bumpRight;
  return sensors->bumps_wheelDrops.bumpLeft ||
         sensors->bumps_wheelDrops.bumpCenter ||
         sensors->bumps_wheelDrops.bumpRight;
}
int main(void) {
  ret_code_t error_code = NRF_SUCCESS;

  // initialize RTT library
  error_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(error_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  printf("Log initialized!\n");

  // initialize LEDs
  nrf_gpio_pin_dir_set(23, NRF_GPIO_PIN_DIR_OUTPUT);
  nrf_gpio_pin_dir_set(24, NRF_GPIO_PIN_DIR_OUTPUT);
  nrf_gpio_pin_dir_set(25, NRF_GPIO_PIN_DIR_OUTPUT);

  // initialize display
  nrf_drv_spi_t spi_instance = NRF_DRV_SPI_INSTANCE(1);
  nrf_drv_spi_config_t spi_config = {
    .sck_pin = BUCKLER_LCD_SCLK,
    .mosi_pin = BUCKLER_LCD_MOSI,
    .miso_pin = BUCKLER_LCD_MISO,
    .ss_pin = BUCKLER_LCD_CS,
    .irq_priority = NRFX_SPI_DEFAULT_CONFIG_IRQ_PRIORITY,
    .orc = 0,
    .frequency = NRF_DRV_SPI_FREQ_4M,
    .mode = NRF_DRV_SPI_MODE_2,
    .bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST
  };
  error_code = nrf_drv_spi_init(&spi_instance, &spi_config, NULL, NULL);
  APP_ERROR_CHECK(error_code);
  display_init(&spi_instance);
  display_write("Hello, Human!", DISPLAY_LINE_0);
  printf("Display initialized!\n");

  // initialize i2c master (two wire interface)
  nrf_drv_twi_config_t i2c_config = NRF_DRV_TWI_DEFAULT_CONFIG;
  i2c_config.scl = BUCKLER_SENSORS_SCL;
  i2c_config.sda = BUCKLER_SENSORS_SDA;
  i2c_config.frequency = NRF_TWIM_FREQ_100K;
  error_code = nrf_twi_mngr_init(&twi_mngr_instance, &i2c_config);
  APP_ERROR_CHECK(error_code);
  lsm9ds1_init(&twi_mngr_instance);
  printf("IMU initialized!\n");

  // initialize Kobuki
  kobukiInit();
  printf("Kobuki initialized!\n");

  // configure initial state
  robot_state_t state = OFF;
  KobukiSensors_t sensors = {0};

  float dist = 0;
  float angle = 0;
  bool isRight = false;
  uint16_t lastEncoder = 0;
  // loop forever, running state machine
  while (1) {
    // read sensors from robot
    kobukiSensorPoll(&sensors);

    // delay before continuing
    // Note: removing this delay will make responses quicker, but will result
    //  in printf's in this loop breaking JTAG
    nrf_delay_ms(1);

    // handle states
    switch(state) {
      case OFF: {
        // transition logic
        if (is_button_pressed(&sensors)) {
          state = DRIVING;
          lastEncoder = sensors.leftWheelEncoder;
        } else {
          // perform state-specific actions here
          display_write("OFF", DISPLAY_LINE_0);
          kobukiDriveDirect(0, 0);
          state = OFF;
        }
        break; // each case needs to end with break!
      }

      case DRIVING: {

        // transition logic
        if (is_button_pressed(&sensors)) {
          state = OFF;
          dist = 0;  
          kobukiDriveDirect(0, 0);
        }
        else if (bumpedIntoSth(&sensors, &isRight))
        {
          state = BACKING;
          dist = 0;
          kobukiDriveDirect(-70, -70);
          nrf_delay_ms(100);
          kobukiSensorPoll(&sensors);
          lastEncoder = sensors.leftWheelEncoder;
        }
        else if (dist >= 0.5)
        {
          state = TURNING;
          lsm9ds1_start_gyro_integration();
          dist = 0;
          angle = 0;
          kobukiDriveDirect(100, -100);
        } 
        else {
          // perform state-specific actions here
          display_write("DRIVING", DISPLAY_LINE_0);

          dist += measure_distance(sensors.leftWheelEncoder, lastEncoder);
          lastEncoder = sensors.leftWheelEncoder;
          kobukiDriveDirect(100, 100);
          state = DRIVING;

          char buf[16];
          snprintf(buf, 16, "%f", dist);
          display_write(buf, DISPLAY_LINE_1);
        }
        break; // each case needs to end with break!
      }

      case TURNING:
      {
        if (is_button_pressed(&sensors)) {
          state = OFF;
          lsm9ds1_stop_gyro_integration();
          angle = 0;
          dist = 0;  
          kobukiDriveDirect(0, 0);
        }
        else if (bumpedIntoSth(&sensors, &isRight))
        {
          state = BACKING;
          dist = 0;
          angle = 0;
          kobukiDriveDirect(-70, -70);
          nrf_delay_ms(100);
          kobukiSensorPoll(&sensors);
          lastEncoder = sensors.leftWheelEncoder;
          lsm9ds1_stop_gyro_integration();
        }
        else if (angle <= -90)
        {
          state = DRIVING;
          lsm9ds1_stop_gyro_integration();
          angle = 0;
          dist = 0;  
          kobukiDriveDirect(100, 100);
          nrf_delay_ms(100);
          kobukiSensorPoll(&sensors);
          lastEncoder = sensors.leftWheelEncoder;
        }
        else 
        {
          display_write("TURNING", DISPLAY_LINE_0);
          kobukiDriveDirect(100, -100);
          angle = lsm9ds1_read_gyro_integration().z_axis;
          state = TURNING;

          char buf[16];
          snprintf(buf, 16, "%f", angle);
          display_write(buf, DISPLAY_LINE_1);
        }
        break;
      }

      case BACKING:
      {
        // transition logic
        if (is_button_pressed(&sensors)) {
          state = OFF;
          dist = 0;  
          kobukiDriveDirect(0, 0);
        }
        else if (dist <= -0.1)
        {
          state = TURNING_45;
          lsm9ds1_start_gyro_integration();
          dist = 0;
          angle = 0;
          kobukiDriveDirect(0, 0);
        } 
        else {
          // perform state-specific actions here
          display_write("BACKING", DISPLAY_LINE_0);

          dist += measure_reverse_distance(sensors.leftWheelEncoder, lastEncoder);
          lastEncoder = sensors.leftWheelEncoder;
          kobukiDriveDirect(-70, -70);
          state = BACKING;
          
          char buf[16];
          snprintf(buf, 16, "%f", dist);
          display_write(buf, DISPLAY_LINE_1);
        }
        break; // each case needs to end with break!
      }
      // add other cases here

      case TURNING_45:
      {
        if (is_button_pressed(&sensors)) {
          state = OFF;
          lsm9ds1_stop_gyro_integration();
          angle = 0;
          dist = 0;  
          kobukiDriveDirect(0, 0);
        }
        else if (abs(angle) >= 45)
        {
          state = DRIVING;
          lsm9ds1_stop_gyro_integration();
          angle = 0;
          dist = 0;  
          kobukiDriveDirect(100, 100);
          nrf_delay_ms(100);
          kobukiSensorPoll(&sensors);
          lastEncoder = sensors.leftWheelEncoder;
        }
        else 
        {
          display_write("TURNING_45", DISPLAY_LINE_0);
          if (!isRight) kobukiDriveDirect(60, -60);
          else kobukiDriveDirect(-60, 60);
          angle = lsm9ds1_read_gyro_integration().z_axis;
          state = TURNING_45;

          char buf[16];
          snprintf(buf, 16, "%f", angle);
          display_write(buf, DISPLAY_LINE_1);
        }
        break;
      }
    }
  }
}

