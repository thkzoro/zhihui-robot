#include "robot.h"

extern const unsigned char gImage_1[3200];

int Robot::do_init()
{
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
    LcdIoCfg lcdIoCfg;

    memset(&lcdIoCfg, 0, sizeof(lcdIoCfg));
    lcdIoCfg.spi_id = SPI2_HOST;
    lcdIoCfg.spi_clk = GPIO_NUM_12;
    lcdIoCfg.spi_mosi = GPIO_NUM_11;
    lcdIoCfg.spi_miso = GPIO_NUM_13;
    lcdIoCfg.spi_cs = GPIO_NUM_10;
    lcdIoCfg.lcd_res = GPIO_NUM_9;
    lcdIoCfg.lcd_dc = GPIO_NUM_1;
    lcdIoCfg.lcd_blk = GPIO_NUM_0;

    m_pLcd = new RoundLcd(lcdIoCfg, RoundLcd::DEGREE_0);
    m_pLcd->LCD_Init();
    m_pLcd->LCD_Fill(0,0,LCD_W,LCD_H,WHITE);
    for(int j=0;j<3;j++) {
        for(int i=0;i<6;i++) {
        	m_pLcd->LCD_ShowPicture(40*i,120+j*40,40,40,gImage_1);
        }
    }
    m_pLcd->LCD_ShowString(32,80,(const u8*)("LCD_Diameter:"),RED,WHITE,16,0);

    //手势传感器初始化
    PAJ7620IoCfg paj7620IoCfg;
    memset(&paj7620IoCfg, 0, sizeof(paj7620IoCfg));
    paj7620IoCfg.i2c_id = 0;
    paj7620IoCfg.i2c_sda = GPIO_NUM_18;
    paj7620IoCfg.i2c_scl = GPIO_NUM_17;
    m_pPAJ7620 = new PAJ7620(paj7620IoCfg);
    m_pPAJ7620->init();

    //usb初始化
	 m_pUsbRobot = new UsbRobot();
	 m_pUsbRobot->init();

    return 0;
}
