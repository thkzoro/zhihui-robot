#include "round_lcd.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
# include <string.h>
#include "lcdfontlib.h"
#include "lcdpicturelib.h"

#define GPIO_HANDSHAKE 2
#define delay(x)	vTaskDelay(x / portTICK_PERIOD_MS)
#define delay_ms(x) delay(x)

extern const unsigned char ascii_1206[][12];
extern const unsigned char ascii_1608[][16];
extern const unsigned char ascii_2412[][48];
extern const unsigned char ascii_3216[][64];
/******************************************************************************
      函数说明：在指定区域填充颜色
      入口数据：xsta,ysta   起始坐标
                xend,yend   终止坐标
								color       要填充的颜色
      返回值：  无
******************************************************************************/
void RoundLcd::LCD_Fill(u16 xsta,u16 ysta,u16 xend,u16 yend,u16 color)
{
    u16 i,j;
    LCD_Address_Set(xsta,ysta,xend-1,yend-1);//设置显示范围
    for(i=ysta;i<yend;i++)
    {
        for(j=xsta;j<xend;j++)
        {
            LCD_WR_DATA(color);
        }
    }
}

/******************************************************************************
      函数说明：在指定位置画点
      入口数据：x,y 画点坐标
                color 点的颜色
      返回值：  无
******************************************************************************/
void RoundLcd::LCD_DrawPoint(u16 x,u16 y,u16 color)
{
    LCD_Address_Set(x,y,x,y);//设置光标位置
    LCD_WR_DATA(color);
}


/******************************************************************************
      函数说明：画线
      入口数据：x1,y1   起始坐标
                x2,y2   终止坐标
                color   线的颜色
      返回值：  无
******************************************************************************/
void RoundLcd::LCD_DrawLine(u16 x1,u16 y1,u16 x2,u16 y2,u16 color)
{
    u16 t;
    int xerr=0,yerr=0,delta_x,delta_y,distance;
    int incx,incy,uRow,uCol;
    delta_x=x2-x1; //计算坐标增量
    delta_y=y2-y1;
    uRow=x1;//画线起点坐标
    uCol=y1;
    if(delta_x>0)incx=1; //设置单步方向
    else if (delta_x==0)incx=0;//垂直线
    else {incx=-1;delta_x=-delta_x;}
    if(delta_y>0)incy=1;
    else if (delta_y==0)incy=0;//水平线
    else {incy=-1;delta_y=-delta_y;}
    if(delta_x>delta_y)distance=delta_x; //选取基本增量坐标轴
    else distance=delta_y;
    for(t=0;t<distance+1;t++)
    {
        LCD_DrawPoint(uRow,uCol,color);//画点
        xerr+=delta_x;
        yerr+=delta_y;
        if(xerr>distance)
        {
            xerr-=distance;
            uRow+=incx;
        }
        if(yerr>distance)
        {
            yerr-=distance;
            uCol+=incy;
        }
    }
}


/******************************************************************************
      函数说明：画矩形
      入口数据：x1,y1   起始坐标
                x2,y2   终止坐标
                color   矩形的颜色
      返回值：  无
******************************************************************************/
void RoundLcd::LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2,u16 color)
{
    LCD_DrawLine(x1,y1,x2,y1,color);
    LCD_DrawLine(x1,y1,x1,y2,color);
    LCD_DrawLine(x1,y2,x2,y2,color);
    LCD_DrawLine(x2,y1,x2,y2,color);
}


/******************************************************************************
      函数说明：画圆
      入口数据：x0,y0   圆心坐标
                r       半径
                color   圆的颜色
      返回值：  无
******************************************************************************/
void RoundLcd::Draw_Circle(u16 x0,u16 y0,u8 r,u16 color)
{
    int a,b;
    a=0;b=r;
    while(a<=b)
    {
        LCD_DrawPoint(x0-b,y0-a,color);             //3
        LCD_DrawPoint(x0+b,y0-a,color);             //0
        LCD_DrawPoint(x0-a,y0+b,color);             //1
        LCD_DrawPoint(x0-a,y0-b,color);             //2
        LCD_DrawPoint(x0+b,y0+a,color);             //4
        LCD_DrawPoint(x0+a,y0-b,color);             //5
        LCD_DrawPoint(x0+a,y0+b,color);             //6
        LCD_DrawPoint(x0-b,y0+a,color);             //7
        a++;
        if((a*a+b*b)>(r*r))//判断要画的点是否过远
        {
            b--;
        }
    }
}

/******************************************************************************
      函数说明：显示单个字符
      入口数据：x,y显示坐标
                num 要显示的字符
                fc 字的颜色
                bc 字的背景色
                sizey 字号
                mode:  0非叠加模式  1叠加模式
      返回值：  无
******************************************************************************/
void RoundLcd::LCD_ShowChar(u16 x,u16 y,u8 num,u16 fc,u16 bc,u8 sizey,u8 mode)
{
    u8 temp,sizex,t,m=0;
    u16 i,TypefaceNum;//一个字符所占字节大小
    u16 x0=x;
    sizex=sizey/2;
    TypefaceNum=(sizex/8+((sizex%8)?1:0))*sizey;
    num=num-' ';    //得到偏移后的值
    LCD_Address_Set(x,y,x+sizex-1,y+sizey-1);  //设置光标位置
    for(i=0;i<TypefaceNum;i++)
    {
        if(sizey==12)temp=ascii_1206[num][i];		       //调用6x12字体
        else if(sizey==16)temp=ascii_1608[num][i];		 //调用8x16字体
        else if(sizey==24)temp=ascii_2412[num][i];		 //调用12x24字体
        else if(sizey==32)temp=ascii_3216[num][i];		 //调用16x32字体
        else return;
        for(t=0;t<8;t++)
        {
            if(!mode)//非叠加模式
            {
                if(temp&(0x01<<t))LCD_WR_DATA(fc);
                else LCD_WR_DATA(bc);
                m++;
                if(m%sizex==0)
                {
                    m=0;
                    break;
                }
            }
            else//叠加模式
            {
                if(temp&(0x01<<t))LCD_DrawPoint(x,y,fc);//画一个点
                x++;
                if((x-x0)==sizex)
                {
                    x=x0;
                    y++;
                    break;
                }
            }
        }
    }
}


/******************************************************************************
      函数说明：显示字符串
      入口数据：x,y显示坐标
                *p 要显示的字符串
                fc 字的颜色
                bc 字的背景色
                sizey 字号
                mode:  0非叠加模式  1叠加模式
      返回值：  无
******************************************************************************/
void RoundLcd::LCD_ShowString(u16 x,u16 y,const u8 *p,u16 fc,u16 bc,u8 sizey,u8 mode)
{
    while(*p!='\0')
    {
        LCD_ShowChar(x,y,*p,fc,bc,sizey,mode);
        x+=sizey/2;
        p++;
    }
}


/******************************************************************************
      函数说明：显示数字
      入口数据：m底数，n指数
      返回值：  无
******************************************************************************/
u32 RoundLcd::mypow(u8 m,u8 n)
{
    u32 result=1;
    while(n--)result*=m;
    return result;
}


/******************************************************************************
      函数说明：显示整数变量
      入口数据：x,y显示坐标
                num 要显示整数变量
                len 要显示的位数
                fc 字的颜色
                bc 字的背景色
                sizey 字号
      返回值：  无
******************************************************************************/
void RoundLcd::LCD_ShowIntNum(u16 x,u16 y,u16 num,u8 len,u16 fc,u16 bc,u8 sizey)
{
    u8 t,temp;
    u8 enshow=0;
    u8 sizex=sizey/2;
    for(t=0;t<len;t++)
    {
        temp=(num/mypow(10,len-t-1))%10;
        if(enshow==0&&t<(len-1))
        {
            if(temp==0)
            {
                LCD_ShowChar(x+t*sizex,y,' ',fc,bc,sizey,0);
                continue;
            }else enshow=1;

        }
        LCD_ShowChar(x+t*sizex,y,temp+48,fc,bc,sizey,0);
    }
}


/******************************************************************************
      函数说明：显示两位小数变量
      入口数据：x,y显示坐标
                num 要显示小数变量
                len 要显示的位数
                fc 字的颜色
                bc 字的背景色
                sizey 字号
      返回值：  无
******************************************************************************/
void RoundLcd::LCD_ShowFloatNum1(u16 x,u16 y,float num,u8 len,u16 fc,u16 bc,u8 sizey)
{
    u8 t,temp,sizex;
    u16 num1;
    sizex=sizey/2;
    num1=num*100;
    for(t=0;t<len;t++)
    {
        temp=(num1/mypow(10,len-t-1))%10;
        if(t==(len-2))
        {
            LCD_ShowChar(x+(len-2)*sizex,y,'.',fc,bc,sizey,0);
            t++;
            len+=1;
        }
        LCD_ShowChar(x+t*sizex,y,temp+48,fc,bc,sizey,0);
    }
}


/******************************************************************************
      函数说明：显示图片
      入口数据：x,y起点坐标
                length 图片长度
                width  图片宽度
                pic[]  图片数组
      返回值：  无
******************************************************************************/
void RoundLcd::LCD_ShowPicture(u16 x,u16 y,u16 length,u16 width,const u8 pic[])
{
    u16 i,j;
    u32 k=0;
    LCD_Address_Set(x,y,x+length-1,y+width-1);
    for(i=0;i<length;i++)
    {
        for(j=0;j<width;j++)
        {
            LCD_WR_DATA8(pic[k*2]);
            LCD_WR_DATA8(pic[k*2+1]);
            k++;
        }
    }
}

/******************************************************************************
      函数说明：LCD串行数据写入函数
      入口数据：data  要写入的串行数据
      返回值：  无
******************************************************************************/
void RoundLcd::LCD_Writ_Bus(u8 data)
{
    spi_transaction_t t;
    t.length = sizeof(data);
    t.tx_buffer = &data;
    t.rx_buffer = &data;
    spi_device_transmit(m_spi_handle, &t);
}


/******************************************************************************
      函数说明：LCD写入数据
      入口数据：data 写入的数据
      返回值：  无
******************************************************************************/
void RoundLcd::LCD_WR_DATA8(u8 data)
{
    LCD_Writ_Bus(data);
}


/******************************************************************************
      函数说明：LCD写入数据
      入口数据：data 写入的数据
      返回值：  无
******************************************************************************/
void RoundLcd::LCD_WR_DATA(u16 data)
{
    LCD_Writ_Bus(data>>8);
    LCD_Writ_Bus(data);
}


/******************************************************************************
      函数说明：LCD写入命令
      入口数据：data 写入的命令
      返回值：  无
******************************************************************************/
void RoundLcd::LCD_WR_REG(u8 data)
{
    LCD_DC_Clr();//写命令
    LCD_Writ_Bus(data);
    LCD_DC_Set();//写数据
}


/******************************************************************************
      函数说明：设置起始和结束地址
      入口数据：x1,x2 设置列的起始和结束地址
                y1,y2 设置行的起始和结束地址
      返回值：  无
******************************************************************************/
void RoundLcd::LCD_Address_Set(u16 x1,u16 y1,u16 x2,u16 y2)
{
    LCD_WR_REG(0x2a);//列地址设置
    LCD_WR_DATA(x1);
    LCD_WR_DATA(x2);
    LCD_WR_REG(0x2b);//行地址设置
    LCD_WR_DATA(y1);
    LCD_WR_DATA(y2);
    LCD_WR_REG(0x2c);//储存器写
}

bool RoundLcd::gpio_init()
{
    gpio_config_t io_conf;
    //zero-initialize the config structure.
    memset(&io_conf, 0, sizeof(io_conf));
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //disable pull-up mode
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    //disable pull-down mode
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //bit mask of the pins that you want to set
    io_conf.pin_bit_mask = ((1ULL<<m_ioCfg.lcd_blk) | (1ULL<<m_ioCfg.lcd_dc) | (1ULL<<m_ioCfg.lcd_res));

    //configure GPIO with the given settings
    gpio_config(&io_conf);

    return true;
}

bool RoundLcd::spi_init()
{
    esp_err_t ret;
    //Configuration for the SPI bus
    spi_bus_config_t busCfg;

    memset(&busCfg, 0, sizeof(busCfg));

    busCfg.mosi_io_num = m_ioCfg.spi_mosi;
    busCfg.miso_io_num = m_ioCfg.spi_miso;
    busCfg.sclk_io_num = m_ioCfg.spi_clk;
    busCfg.quadwp_io_num=-1;
    busCfg.quadhd_io_num=-1;

    //Configuration for the SPI device on the other side of the bus
    spi_device_interface_config_t devCfg;

    memset(&devCfg, 0, sizeof(devCfg));
    devCfg.command_bits = 0;
    devCfg.address_bits = 0;
    devCfg.dummy_bits = 0;
    devCfg.mode = 0;
    devCfg.duty_cycle_pos = 128;        //50% duty cycle
    devCfg.cs_ena_posttrans = 3;        //Keep the CS low 3 cycles after transaction, to stop slave from missing the last bit when CS has less propagation delay than CLK
    devCfg.clock_speed_hz = 5000000;
    devCfg.spics_io_num = m_ioCfg.spi_cs;
    devCfg.queue_size = 3;

    //Initialize the SPI bus and add the device we want to send stuff to.
    ret = spi_bus_initialize(m_ioCfg.spi_id, &busCfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        return false;
    }

    ret = spi_bus_add_device(m_ioCfg.spi_id, &devCfg, &m_spi_handle);
    if (ret != ESP_OK) {
        return false;
    }

    return true;
}

void RoundLcd::LCD_Init(void)
{
    spi_init();
    gpio_init();

    LCD_RES_Clr();//复位
    delay_ms(100);
    LCD_RES_Set();
    delay_ms(100);

    LCD_BLK_Set();//打开背光
    delay_ms(100);

    LCD_WR_REG(0xEF);
    LCD_WR_REG(0xEB);
    LCD_WR_DATA8(0x14);

    LCD_WR_REG(0xFE);
    LCD_WR_REG(0xEF);

    LCD_WR_REG(0xEB);
    LCD_WR_DATA8(0x14);

    LCD_WR_REG(0x84);
    LCD_WR_DATA8(0x40);

    LCD_WR_REG(0x85);
    LCD_WR_DATA8(0xFF);

    LCD_WR_REG(0x86);
    LCD_WR_DATA8(0xFF);

    LCD_WR_REG(0x87);
    LCD_WR_DATA8(0xFF);

    LCD_WR_REG(0x88);
    LCD_WR_DATA8(0x0A);

    LCD_WR_REG(0x89);
    LCD_WR_DATA8(0x21);

    LCD_WR_REG(0x8A);
    LCD_WR_DATA8(0x00);

    LCD_WR_REG(0x8B);
    LCD_WR_DATA8(0x80);

    LCD_WR_REG(0x8C);
    LCD_WR_DATA8(0x01);

    LCD_WR_REG(0x8D);
    LCD_WR_DATA8(0x01);

    LCD_WR_REG(0x8E);
    LCD_WR_DATA8(0xFF);

    LCD_WR_REG(0x8F);
    LCD_WR_DATA8(0xFF);


    LCD_WR_REG(0xB6);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x20);

    LCD_WR_REG(0x36);
    if(USE_HORIZONTAL==0)LCD_WR_DATA8(0x08);
    else if(USE_HORIZONTAL==1)LCD_WR_DATA8(0xC8);
    else if(USE_HORIZONTAL==2)LCD_WR_DATA8(0x68);
    else LCD_WR_DATA8(0xA8);

    LCD_WR_REG(0x3A);
    LCD_WR_DATA8(0x05);


    LCD_WR_REG(0x90);
    LCD_WR_DATA8(0x08);
    LCD_WR_DATA8(0x08);
    LCD_WR_DATA8(0x08);
    LCD_WR_DATA8(0x08);

    LCD_WR_REG(0xBD);
    LCD_WR_DATA8(0x06);

    LCD_WR_REG(0xBC);
    LCD_WR_DATA8(0x00);

    LCD_WR_REG(0xFF);
    LCD_WR_DATA8(0x60);
    LCD_WR_DATA8(0x01);
    LCD_WR_DATA8(0x04);

    LCD_WR_REG(0xC3);
    LCD_WR_DATA8(0x13);
    LCD_WR_REG(0xC4);
    LCD_WR_DATA8(0x13);

    LCD_WR_REG(0xC9);
    LCD_WR_DATA8(0x22);

    LCD_WR_REG(0xBE);
    LCD_WR_DATA8(0x11);

    LCD_WR_REG(0xE1);
    LCD_WR_DATA8(0x10);
    LCD_WR_DATA8(0x0E);

    LCD_WR_REG(0xDF);
    LCD_WR_DATA8(0x21);
    LCD_WR_DATA8(0x0c);
    LCD_WR_DATA8(0x02);

    LCD_WR_REG(0xF0);
    LCD_WR_DATA8(0x45);
    LCD_WR_DATA8(0x09);
    LCD_WR_DATA8(0x08);
    LCD_WR_DATA8(0x08);
    LCD_WR_DATA8(0x26);
    LCD_WR_DATA8(0x2A);

    LCD_WR_REG(0xF1);
    LCD_WR_DATA8(0x43);
    LCD_WR_DATA8(0x70);
    LCD_WR_DATA8(0x72);
    LCD_WR_DATA8(0x36);
    LCD_WR_DATA8(0x37);
    LCD_WR_DATA8(0x6F);


    LCD_WR_REG(0xF2);
    LCD_WR_DATA8(0x45);
    LCD_WR_DATA8(0x09);
    LCD_WR_DATA8(0x08);
    LCD_WR_DATA8(0x08);
    LCD_WR_DATA8(0x26);
    LCD_WR_DATA8(0x2A);

    LCD_WR_REG(0xF3);
    LCD_WR_DATA8(0x43);
    LCD_WR_DATA8(0x70);
    LCD_WR_DATA8(0x72);
    LCD_WR_DATA8(0x36);
    LCD_WR_DATA8(0x37);
    LCD_WR_DATA8(0x6F);

    LCD_WR_REG(0xED);
    LCD_WR_DATA8(0x1B);
    LCD_WR_DATA8(0x0B);

    LCD_WR_REG(0xAE);
    LCD_WR_DATA8(0x77);

    LCD_WR_REG(0xCD);
    LCD_WR_DATA8(0x63);


    LCD_WR_REG(0x70);
    LCD_WR_DATA8(0x07);
    LCD_WR_DATA8(0x07);
    LCD_WR_DATA8(0x04);
    LCD_WR_DATA8(0x0E);
    LCD_WR_DATA8(0x0F);
    LCD_WR_DATA8(0x09);
    LCD_WR_DATA8(0x07);
    LCD_WR_DATA8(0x08);
    LCD_WR_DATA8(0x03);

    LCD_WR_REG(0xE8);
    LCD_WR_DATA8(0x34);

    LCD_WR_REG(0x62);
    LCD_WR_DATA8(0x18);
    LCD_WR_DATA8(0x0D);
    LCD_WR_DATA8(0x71);
    LCD_WR_DATA8(0xED);
    LCD_WR_DATA8(0x70);
    LCD_WR_DATA8(0x70);
    LCD_WR_DATA8(0x18);
    LCD_WR_DATA8(0x0F);
    LCD_WR_DATA8(0x71);
    LCD_WR_DATA8(0xEF);
    LCD_WR_DATA8(0x70);
    LCD_WR_DATA8(0x70);

    LCD_WR_REG(0x63);
    LCD_WR_DATA8(0x18);
    LCD_WR_DATA8(0x11);
    LCD_WR_DATA8(0x71);
    LCD_WR_DATA8(0xF1);
    LCD_WR_DATA8(0x70);
    LCD_WR_DATA8(0x70);
    LCD_WR_DATA8(0x18);
    LCD_WR_DATA8(0x13);
    LCD_WR_DATA8(0x71);
    LCD_WR_DATA8(0xF3);
    LCD_WR_DATA8(0x70);
    LCD_WR_DATA8(0x70);

    LCD_WR_REG(0x64);
    LCD_WR_DATA8(0x28);
    LCD_WR_DATA8(0x29);
    LCD_WR_DATA8(0xF1);
    LCD_WR_DATA8(0x01);
    LCD_WR_DATA8(0xF1);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x07);

    LCD_WR_REG(0x66);
    LCD_WR_DATA8(0x3C);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0xCD);
    LCD_WR_DATA8(0x67);
    LCD_WR_DATA8(0x45);
    LCD_WR_DATA8(0x45);
    LCD_WR_DATA8(0x10);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x00);

    LCD_WR_REG(0x67);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x3C);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x01);
    LCD_WR_DATA8(0x54);
    LCD_WR_DATA8(0x10);
    LCD_WR_DATA8(0x32);
    LCD_WR_DATA8(0x98);

    LCD_WR_REG(0x74);
    LCD_WR_DATA8(0x10);
    LCD_WR_DATA8(0x85);
    LCD_WR_DATA8(0x80);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x4E);
    LCD_WR_DATA8(0x00);

    LCD_WR_REG(0x98);
    LCD_WR_DATA8(0x3e);
    LCD_WR_DATA8(0x07);

    LCD_WR_REG(0x35);
    LCD_WR_REG(0x21);

    LCD_WR_REG(0x11);
    delay_ms(120);
    LCD_WR_REG(0x29);
    delay_ms(20);
}
