#ifndef LINUX_SENSOR_ROUND_LCD_H
#define LINUX_SENSOR_ROUND_LCD_H

#include <cstdint>


class RoundLcd
{
public:
    RoundLcd()        
    {
       
    }

    typedef enum Orientation_t
    {
        DEGREE_0,
        DEGREE_90,
        DEGREE_180,
        DEGREE_270
    } Orientation_t;


    typedef enum ColorMode_t
    {
        BIT_12,
        BIT_16,
        BIT_18
    } ColorMode_t;

    void spi_init();
    void gpio_init();
    void Init(Orientation_t _orientation);

    void SetWindow(uint16_t _startX, uint16_t _endX, uint16_t _startY, uint16_t _endY);

    void WriteFrameBuffer(uint8_t* _buffer, uint32_t _len, bool _isAppend = false);

    void SetBackLight(float _val = 1.0f);

    volatile bool isBusy = false;

    void LCD_ShowPicture(uint16_t x, uint16_t y, uint16_t length, uint16_t width, const uint8_t pic[]);
private:
    void ChipSelect(bool _enable);

    void Reset(bool _enable);

    void SetDataOrCommand(bool _isData);

    void WriteCommand(uint8_t _cmd);

    void Write1Byte(uint8_t _data);

    void WriteData(uint8_t* _data, uint32_t _len, bool _useDma = false);

    Orientation_t orientation = DEGREE_0;
    ColorMode_t colorMode = BIT_18;
};


#endif //LINUX_SENSOR_ROUND_LCD_H
