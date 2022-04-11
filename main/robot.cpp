#include "robot.h"
#include "mpu6050.h"
#include "esp_log.h"

static const char TAG[] = "robot";

#define ROBOT_CHECK(a, str, ret) if(!(a)) { \
        ESP_LOGE(TAG,"%s:%d (%s):%s", __FILE__, __LINE__, __FUNCTION__, str); \
        return (ret); \
    }



extern const unsigned char gImage_1[3200];

esp_err_t Robot::robot_init()
{
    esp_err_t ret = robot_gpio_init();
    ROBOT_CHECK(ret == ESP_OK, "gpio init failed", ret);

    ret = robot_i2c_bus_init();
    ROBOT_CHECK(ret == ESP_OK, "i2c init failed", ret);

    ret = robot_spi_bus_init();
    ROBOT_CHECK(ret == ESP_OK, "spi init failed", ret);

//    /********* Need to adjust parameters for specific hardware *********/
//    joint[ANY] = JointStatus_t{0, -180, 180, 90};
//    // Head
//    joint[1] = JointStatus_t{2, 70, 95, 0, -15, 15, true};
//    // Left arm roll
//    joint[2] = JointStatus_t{4, -9, 3, 0, 0, 30, false};
//    // Left arm pitch
//    joint[3] = JointStatus_t{6, -16, 117, 0, -20, 180, false};
//    // Right arm roll
//    joint[4] = JointStatus_t{8, 133, 141, 0, 0, 30, true};
//    // Right arm pitch
//    joint[5] = JointStatus_t{10, 15, 150, 0, -20, 180, true};
//    // Body
//    joint[6] = JointStatus_t{12, 0, 180, 0, -90, 90, false};
//    /********* Need to adjust parameters for specific hardware *********/

	//LCD屏幕初始化
	m_pLcd = new RobotLCD(m_spi2_bus_handle, GPIO_NUM_10);
    //手势传感器初始化
    m_pPAJ7620 = new PAJ7620(m_i2c0_bus_handle, PAJ7620_ID);
    m_pPAJ7620->init();
    //6轴陀螺仪初始化
    m_mpu6050 = mpu6050_create(m_i2c0_bus_handle, MPU6050_I2C_ADDRESS);
//
//    //usb初始化
//	 m_pUsbRobot = new UsbRobot();
//	 m_pUsbRobot->init();


    ESP_LOGI(TAG,"Init Done ...");
    return ESP_OK;
}


esp_err_t Robot::robot_gpio_init(void)
{
    return ESP_OK;
}

esp_err_t Robot::robot_i2c_bus_init(void)
{
    i2c_config_t board_i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_NUM_18,
		.scl_io_num = GPIO_NUM_17,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
    };
    board_i2c_conf.master.clk_speed = 1000000;
    i2c_bus_handle_t handle = i2c_bus_create(I2C_NUM_0, &board_i2c_conf);
    ROBOT_CHECK(handle != NULL, "i2c_bus create failed", ESP_FAIL);
    m_i2c0_bus_handle = handle;
    ESP_LOGI(TAG, "i2c_bus 0 create succeed");
    return ESP_OK;
}

esp_err_t Robot::robot_spi_bus_init(void)
{
    spi_config_t bus_conf = {
        .miso_io_num = GPIO_NUM_13,
        .mosi_io_num = GPIO_NUM_11,
        .sclk_io_num = GPIO_NUM_12,
    };
    m_spi2_bus_handle = spi_bus_create(SPI2_HOST, &bus_conf);
    ROBOT_CHECK(m_spi2_bus_handle != NULL, "spi_bus2 create failed", ESP_FAIL);
    ESP_LOGI(TAG, "spi_bus 2 create succeed");
    return ESP_OK;
}
