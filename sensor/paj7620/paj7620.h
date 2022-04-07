#ifndef __PAJ7620_H__
#define __PAJ7620_H__
#include "hal/gpio_types.h"
#include <string.h>

#define PAJ7620_VAL(val, maskbit)		( val << maskbit )
#define PAJ7620_ADDR_BASE				0x00
#define PAJ7620_REGITER_BANK_SEL		(PAJ7620_ADDR_BASE + 0xEF)	//W //BANK选择寄存器

#define PAJ7620_ID  0x73    //设备地址

//BANK0 寄存器组
#define PAJ7620_ADDR_SUSPEND_CMD            (PAJ7620_ADDR_BASE + 0X03) //W  //设置设备挂起
#define PAJ7620_ADDR_SET_INT_FLAG1          (PAJ7620_ADDR_BASE + 0X41) //RW //设置手势检测中断寄存器1
#define PAJ7620_ADDR_SET_INT_FLAG2          (PAJ7620_ADDR_BASE + 0X42) //RW //设置手势检测中断寄存器2
#define PAJ7620_ADDR_GET_INT_FLAG1          (PAJ7620_ADDR_BASE + 0X43) //R  //获取手势检测中断标志寄存器1(获取手势结果)
#define PAJ7620_ADDR_GET_INT_FLAG2          (PAJ7620_ADDR_BASE + 0X44) //R  //获取手势检测中断标志寄存器2(获取手势结果)
#define PAJ7620_ADDR_GET_STATE              (PAJ7620_ADDR_BASE + 0X45) //R  //获取手势检测工作状态
#define PAJ7620_ADDR_SET_HIGH_THRESHOLD     (PAJ7620_ADDR_BASE + 0x69) //RW //设置滞后高阀值（仅在接近检测模式下）
#define PAJ7620_ADDR_SET_LOW_THRESEHOLD     (PAJ7620_ADDR_BASE + 0X6A) //RW //设置滞后低阀值
#define PAJ7620_ADDR_GET_APPROACH_STATE     (PAJ7620_ADDR_BASE + 0X6B) //R  //获取接近状态 （1：PS data>= PS threshold ,0:PS data<= Low threshold）
#define PAJ7620_ADDR_GET_GESTURE_DATA       (PAJ7620_ADDR_BASE + 0X6C) //R  //获取接近数据
#define PAJ7620_ADDR_GET_OBJECT_BRIGHTNESS  (PAJ7620_ADDR_BASE + 0XB0) //获取被照物体亮度（最大255）
#define PAJ7620_ADDR_GET_OBJECT_SIZE_1      (PAJ7620_ADDR_BASE + 0XB1) //获取被照物体大小低八位（bit7:0）(最大900)
#define PAJ7620_ADDR_GET_OBJECT_SIZE_2      (PAJ7620_ADDR_BASE + 0XB2) //获取被照物体大小高四位（bit3:0）

//BANK1 寄存器组
#define PAJ7620_ADDR_SET_PS_GAIN         (PAJ7620_ADDR_BASE + 0X44) //RW //设置检测增益大小 (0:1x gain 1:2x gain)
#define PAJ7620_ADDR_SET_IDLE_S1_STEP_0  (PAJ7620_ADDR_BASE + 0x67) //RW //设置S1的响应因子
#define PAJ7620_ADDR_SET_IDLE_S1_STEP_1  (PAJ7620_ADDR_BASE + 0x68) //RW
#define PAJ7620_ADDR_SET_IDLE_S2_STEP_0  (PAJ7620_ADDR_BASE + 0X69) //RW //设置S2的响应因子
#define PAJ7620_ADDR_SET_IDLE_S2_STEP_1  (PAJ7620_ADDR_BASE + 0X6A) //RW
#define PAJ7620_ADDR_SET_OP_TO_S1_STEP_0 (PAJ7620_ADDR_BASE + 0X6B) //RW //设置OP到S1的过度时间
#define PAJ7620_ADDR_SET_OP_TO_S1_STEP_1 (PAJ7620_ADDR_BASE + 0X6C) //RW
#define PAJ7620_ADDR_SET_S1_TO_S2_STEP_0 (PAJ7620_ADDR_BASE + 0X6D) //RW //设置S1到S2的过度时间
#define PAJ7620_ADDR_SET_S1_TO_S2_STEP_1 (PAJ7620_ADDR_BASE + 0X6E) //RW
#define PAJ7620_ADDR_OPERATION_ENABLE    (PAJ7620_ADDR_BASE + 0X72) //RW //设置PAJ7620U2使能寄存器

// PAJ7620_REGITER_BANK_SEL
#define PAJ7620_BANK0	PAJ7620_VAL(0,0)
#define PAJ7620_BANK1	PAJ7620_VAL(1,0)

// PAJ7620_ADDR_SUSPEND_CMD
#define PAJ7620_I2C_WAKEUP	PAJ7620_VAL(1,0)
#define PAJ7620_I2C_SUSPEND	PAJ7620_VAL(0,0)

// PAJ7620_ADDR_OPERATION_ENABLE
#define PAJ7620_ENABLE		PAJ7620_VAL(1,0)
#define PAJ7620_DISABLE		PAJ7620_VAL(0,0)

#define GES_RIGHT_FLAG				PAJ7620_VAL(1,0)
#define GES_LEFT_FLAG				PAJ7620_VAL(1,1)
#define GES_UP_FLAG					PAJ7620_VAL(1,2)
#define GES_DOWN_FLAG				PAJ7620_VAL(1,3)
#define GES_FORWARD_FLAG			PAJ7620_VAL(1,4)
#define GES_BACKWARD_FLAG			PAJ7620_VAL(1,5)
#define GES_CLOCKWISE_FLAG			PAJ7620_VAL(1,6)
#define GES_COUNT_CLOCKWISE_FLAG	PAJ7620_VAL(1,7)
#define GES_WAVE_FLAG				PAJ7620_VAL(1,0)


#define INIT_REG_ARRAY_SIZE (sizeof(initRegisterArray)/sizeof(initRegisterArray[0]))

//管脚配置
struct PAJ7620IoCfg{
    //spi
    int i2c_id;
    gpio_num_t i2c_sda;
    gpio_num_t i2c_scl;
    gpio_num_t paj7620_int; //中断引脚
};

class PAJ7620 {
public:
    PAJ7620(PAJ7620IoCfg &ioCfg) {memcpy(&m_ioCfg, &ioCfg, sizeof(ioCfg));}
    ~PAJ7620() {}

    bool i2c_init();
    uint8_t init(void);
    void selectBank(int bank);
    uint8_t writeReg(uint8_t addr, uint8_t cmd);
    uint8_t readReg(uint8_t addr, uint8_t qty, uint8_t data[]);

private:
    PAJ7620IoCfg m_ioCfg;
};
#endif
