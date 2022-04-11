#ifndef ROUND_LCD_H
#define ROUND_LCD_H

#include <stdint.h>
#include "driver/spi_master.h"
//#include "hal/spi_types.h"
#include "hal/gpio_types.h"
#include <string.h>


typedef  uint8_t    u8;
typedef  uint16_t   u16;
typedef  uint32_t   u32;

#define USE_HORIZONTAL 0  //设置横屏或者竖屏显示 0或1为竖屏 2或3为横屏


#define LCD_W 240
#define LCD_H 240

//-----------------LCD端口定义----------------

#define LCD_SCLK_Clr() //SCL=SCLK
#define LCD_SCLK_Set()

#define LCD_MOSI_Clr() //SDA=MOSI
#define LCD_MOSI_Set()

#define LCD_RES_Clr()   gpio_set_level(m_ioCfg.lcd_res, 0)//RES
#define LCD_RES_Set()   gpio_set_level(m_ioCfg.lcd_res, 1)

#define LCD_DC_Clr()   gpio_set_level(m_ioCfg.lcd_dc, 0)//DC
#define LCD_DC_Set()   gpio_set_level(m_ioCfg.lcd_dc, 1)

#define LCD_CS_Clr()   //CS
#define LCD_CS_Set()

#define LCD_BLK_Clr()  gpio_set_level(m_ioCfg.lcd_blk, 0)//BLK
#define LCD_BLK_Set()  gpio_set_level(m_ioCfg.lcd_blk, 1)

//画笔颜色
#define WHITE         	 0xFFFF
#define BLACK         	 0x0000
#define BLUE           	 0x001F
#define BRED             0XF81F
#define GRED 			 0XFFE0
#define GBLUE			 0X07FF
#define RED           	 0xF800
#define MAGENTA       	 0xF81F
#define GREEN         	 0x07E0
#define CYAN          	 0x7FFF
#define YELLOW        	 0xFFE0
#define BROWN 			 0XBC40 //棕色
#define BRRED 			 0XFC07 //棕红色
#define GRAY  			 0X8430 //灰色
#define DARKBLUE      	 0X01CF	//深蓝色
#define LIGHTBLUE      	 0X7D7C	//浅蓝色
#define GRAYBLUE       	 0X5458 //灰蓝色
#define LIGHTGREEN     	 0X841F //浅绿色
#define LGRAY 			 0XC618 //浅灰色(PANNEL),窗体背景色
#define LGRAYBLUE        0XA651 //浅灰蓝色(中间层颜色)
#define LBBLUE           0X2B12 //浅棕蓝色(选择条目的反色)


//lcd管脚配置
struct LcdIoCfg{
    //spi
    spi_host_device_t spi_id;
    gpio_num_t spi_clk;
    gpio_num_t spi_mosi;
    gpio_num_t spi_miso;
    gpio_num_t spi_cs;

    gpio_num_t lcd_res;
    gpio_num_t lcd_dc;
    gpio_num_t lcd_blk;
};

//芯片型号GC9A01
class RoundLcd {
public:
    enum Orientation_t {
        DEGREE_0,
        DEGREE_90,
        DEGREE_180,
        DEGREE_270
    };
    RoundLcd(LcdIoCfg &ioCfg, Orientation_t _orientation) { memcpy(&m_ioCfg, &ioCfg, sizeof(ioCfg)); }
    ~RoundLcd();
    //LCD 功能函数
    void LCD_Fill(u16 xsta,u16 ysta,u16 xend,u16 yend,u16 color);//指定区域填充颜色
    void LCD_DrawPoint(u16 x,u16 y,u16 color);//在指定位置画一个点
    void LCD_DrawLine(u16 x1,u16 y1,u16 x2,u16 y2,u16 color);//在指定位置画一条线
    void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2,u16 color);//在指定位置画一个矩形
    void Draw_Circle(u16 x0,u16 y0,u8 r,u16 color);//在指定位置画一个圆

    void LCD_ShowChar(u16 x,u16 y,u8 num,u16 fc,u16 bc,u8 sizey,u8 mode);//显示一个字符
    void LCD_ShowString(u16 x,u16 y,const u8 *p,u16 fc,u16 bc,u8 sizey,u8 mode);//显示字符串
    u32 mypow(u8 m,u8 n);//求幂
    void LCD_ShowIntNum(u16 x,u16 y,u16 num,u8 len,u16 fc,u16 bc,u8 sizey);//显示整数变量
    void LCD_ShowFloatNum1(u16 x,u16 y,float num,u8 len,u16 fc,u16 bc,u8 sizey);//显示两位小数变量

    void LCD_ShowPicture(u16 x,u16 y,u16 length,u16 width,const u8 pic[]);//显示图片

    //底层
    bool gpio_init();
    bool spi_init();
    void LCD_Init(void);//LCD初始化
    void LCD_Writ_Bus(u8 data);//模拟SPI时序
    void LCD_WR_DATA8(u8 data);//写入一个字节
    void LCD_WR_DATA(u16 data);//写入两个字节
    void LCD_WR_REG(u8 data);//写入一个指令
    void LCD_Address_Set(u16 x1,u16 y1,u16 x2,u16 y2);//设置坐标函数

private:
    LcdIoCfg m_ioCfg;
    spi_device_handle_t m_spi_handle;
};

#endif //ROUND_LCD_H
