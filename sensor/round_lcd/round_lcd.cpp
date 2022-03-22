#include "round_lcd.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "driver/gpio.h"



#define delay(x)	vTaskDelay(x / portTICK_PERIOD_MS);

#define GPIO_HANDSHAKE 2
#define GPIO_MOSI GPIO_NUM_11
#define GPIO_MISO GPIO_NUM_13
#define GPIO_SCLK GPIO_NUM_12
#define GPIO_CS GPIO_NUM_10
#define SENDER_HOST SPI2_HOST

#define GPIO_OUTPUT_IO_BLK    GPIO_NUM_0
#define GPIO_OUTPUT_IO_DC     GPIO_NUM_1
#define GPIO_OUTPUT_IO_RES    GPIO_NUM_9
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_IO_BLK) | (1ULL<<GPIO_OUTPUT_IO_DC) | (1ULL<<GPIO_OUTPUT_IO_RES))

spi_device_handle_t spi_handle;

void RoundLcd::spi_init()
{
    const char *TAG_spi = "spi";
    esp_err_t ret;
    

    //Configuration for the SPI bus
    spi_bus_config_t buscfg={
        .mosi_io_num=GPIO_MOSI,
        .miso_io_num=GPIO_MISO,
        .sclk_io_num=GPIO_SCLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1
    };

    //Configuration for the SPI device on the other side of the bus
    spi_device_interface_config_t devcfg={
        .command_bits=0,
        .address_bits=0,
        .dummy_bits=0,
        .mode=0,
        .duty_cycle_pos=128,        //50% duty cycle
        .cs_ena_posttrans=3,        //Keep the CS low 3 cycles after transaction, to stop slave from missing the last bit when CS has less propagation delay than CLK
        .clock_speed_hz=5000000,
        .spics_io_num=GPIO_CS,
        .queue_size=3
    };

    //Initialize the SPI bus and add the device we want to send stuff to.
    ret=spi_bus_initialize(SENDER_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGI(TAG_spi, "spi bus init failed");
        return;
    }
    
    ret=spi_bus_add_device(SENDER_HOST, &devcfg, &spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGI(TAG_spi, "spi dev init failed");
        return;
    }

//    ret=spi_bus_remove_device(handle);

}

void RoundLcd::gpio_init()
{
    //zero-initialize the config structure.
   gpio_config_t io_conf = {};
   //disable interrupt
   io_conf.intr_type = GPIO_INTR_DISABLE;
   //set as output mode
   io_conf.mode = GPIO_MODE_OUTPUT;
   //bit mask of the pins that you want to set,e.g.GPIO18/19
   io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
   //disable pull-down mode
   io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
   //disable pull-up mode
   io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
   //configure GPIO with the given settings
   gpio_config(&io_conf);

}

void RoundLcd::Init(Orientation_t _orientation)
{
    ChipSelect(true);

    delay(5);
    Reset(true);
    delay(10);
    Reset(false);
    delay(120);

    /* Initial Sequence */
    WriteCommand(0xEF);

    WriteCommand(0xEB);
    Write1Byte(0x14);

    WriteCommand(0xFE);
    WriteCommand(0xEF);

    WriteCommand(0xEB);
    Write1Byte(0x14);

    WriteCommand(0x84);
    Write1Byte(0x40);

    WriteCommand(0x85);
    Write1Byte(0xFF);

    WriteCommand(0x86);
    Write1Byte(0xFF);

    WriteCommand(0x87);
    Write1Byte(0xFF);

    WriteCommand(0x88);
    Write1Byte(0x0A);

    WriteCommand(0x89);
    Write1Byte(0x21);

    WriteCommand(0x8A);
    Write1Byte(0x00);

    WriteCommand(0x8B);
    Write1Byte(0x80);

    WriteCommand(0x8C);
    Write1Byte(0x01);

    WriteCommand(0x8D);
    Write1Byte(0x01);

    WriteCommand(0x8E);
    Write1Byte(0xFF);

    WriteCommand(0x8F);
    Write1Byte(0xFF);


    WriteCommand(0xB6);
    Write1Byte(0x00);
    Write1Byte(0x00);

    WriteCommand(0x36);
    switch (_orientation)
    {
        case DEGREE_0:
            Write1Byte(0x18);
            break;
        case DEGREE_90:
            Write1Byte(0x28);
            break;
        case DEGREE_180:
            Write1Byte(0x48);
            break;
        case DEGREE_270:
            Write1Byte(0x88);
            break;
    }

    WriteCommand(0x3A); // COLOR_MODE
    switch (colorMode)
    {
        case BIT_12:
            Write1Byte(0x03);
            break;
        case BIT_16:
            Write1Byte(0x05);
            break;
        case BIT_18:
            Write1Byte(0x06);
            break;
    }

    WriteCommand(0x90);
    Write1Byte(0x08);
    Write1Byte(0x08);
    Write1Byte(0x08);
    Write1Byte(0x08);

    WriteCommand(0xBD);
    Write1Byte(0x06);

    WriteCommand(0xBC);
    Write1Byte(0x00);

    WriteCommand(0xFF);
    Write1Byte(0x60);
    Write1Byte(0x01);
    Write1Byte(0x04);

    WriteCommand(0xC3);
    Write1Byte(0x13);
    WriteCommand(0xC4);
    Write1Byte(0x13);

    WriteCommand(0xC9);
    Write1Byte(0x22);

    WriteCommand(0xBE);
    Write1Byte(0x11);

    WriteCommand(0xE1);
    Write1Byte(0x10);
    Write1Byte(0x0E);

    WriteCommand(0xDF);
    Write1Byte(0x21);
    Write1Byte(0x0c);
    Write1Byte(0x02);

    WriteCommand(0xF0);
    Write1Byte(0x45);
    Write1Byte(0x09);
    Write1Byte(0x08);
    Write1Byte(0x08);
    Write1Byte(0x26);
    Write1Byte(0x2A);

    WriteCommand(0xF1);
    Write1Byte(0x43);
    Write1Byte(0x70);
    Write1Byte(0x72);
    Write1Byte(0x36);
    Write1Byte(0x37);
    Write1Byte(0x6F);

    WriteCommand(0xF2);
    Write1Byte(0x45);
    Write1Byte(0x09);
    Write1Byte(0x08);
    Write1Byte(0x08);
    Write1Byte(0x26);
    Write1Byte(0x2A);

    WriteCommand(0xF3);
    Write1Byte(0x43);
    Write1Byte(0x70);
    Write1Byte(0x72);
    Write1Byte(0x36);
    Write1Byte(0x37);
    Write1Byte(0x6F);

    WriteCommand(0xED);
    Write1Byte(0x1B);
    Write1Byte(0x0B);

    WriteCommand(0xAE);
    Write1Byte(0x77);

    WriteCommand(0xCD);
    Write1Byte(0x63);

    WriteCommand(0x70);
    Write1Byte(0x07);
    Write1Byte(0x07);
    Write1Byte(0x04);
    Write1Byte(0x0E);
    Write1Byte(0x0F);
    Write1Byte(0x09);
    Write1Byte(0x07);
    Write1Byte(0x08);
    Write1Byte(0x03);

    WriteCommand(0xE8);
    Write1Byte(0x34);

    WriteCommand(0x62);
    Write1Byte(0x18);
    Write1Byte(0x0D);
    Write1Byte(0x71);
    Write1Byte(0xED);
    Write1Byte(0x70);
    Write1Byte(0x70);
    Write1Byte(0x18);
    Write1Byte(0x0F);
    Write1Byte(0x71);
    Write1Byte(0xEF);
    Write1Byte(0x70);
    Write1Byte(0x70);

    WriteCommand(0x63);
    Write1Byte(0x18);
    Write1Byte(0x11);
    Write1Byte(0x71);
    Write1Byte(0xF1);
    Write1Byte(0x70);
    Write1Byte(0x70);
    Write1Byte(0x18);
    Write1Byte(0x13);
    Write1Byte(0x71);
    Write1Byte(0xF3);
    Write1Byte(0x70);
    Write1Byte(0x70);

    WriteCommand(0x64);
    Write1Byte(0x28);
    Write1Byte(0x29);
    Write1Byte(0xF1);
    Write1Byte(0x01);
    Write1Byte(0xF1);
    Write1Byte(0x00);
    Write1Byte(0x07);

    WriteCommand(0x66);
    Write1Byte(0x3C);
    Write1Byte(0x00);
    Write1Byte(0xCD);
    Write1Byte(0x67);
    Write1Byte(0x45);
    Write1Byte(0x45);
    Write1Byte(0x10);
    Write1Byte(0x00);
    Write1Byte(0x00);
    Write1Byte(0x00);

    WriteCommand(0x67);
    Write1Byte(0x00);
    Write1Byte(0x3C);
    Write1Byte(0x00);
    Write1Byte(0x00);
    Write1Byte(0x00);
    Write1Byte(0x01);
    Write1Byte(0x54);
    Write1Byte(0x10);
    Write1Byte(0x32);
    Write1Byte(0x98);

    WriteCommand(0x74);
    Write1Byte(0x10);
    Write1Byte(0x85);
    Write1Byte(0x80);
    Write1Byte(0x00);
    Write1Byte(0x00);
    Write1Byte(0x4E);
    Write1Byte(0x00);

    WriteCommand(0x98);
    Write1Byte(0x3e);
    Write1Byte(0x07);

    WriteCommand(0x35);
    WriteCommand(0x21);

    WriteCommand(0x11);
    delay(120);
    WriteCommand(0x29);
    delay(20);

    ChipSelect(false);

    SetBackLight(1);
}


void RoundLcd::SetWindow(uint16_t _startX, uint16_t _endX, uint16_t _startY, uint16_t _endY)
{
    ChipSelect(true);

    uint8_t data[4];

    WriteCommand(0x2A); // COL_ADDR_SET
    data[0] = (_startX >> 8) & 0xFF;
    data[1] = _startX & 0xFF;
    data[2] = (_endX >> 8) & 0xFF;
    data[3] = _endX & 0xFF;
    WriteData(data, sizeof(data));

    WriteCommand(0x2B); // ROW_ADDR_SET
    data[0] = (_startY >> 8) & 0xFF;
    data[1] = _startY & 0xFF;
    data[2] = (_endY >> 8) & 0xFF;
    data[3] = _endY & 0xFF;
    WriteData(data, sizeof(data));

    ChipSelect(false);
}


void RoundLcd::WriteFrameBuffer(uint8_t* _buffer, uint32_t _len, bool _isAppend)
{
    isBusy = true;

    ChipSelect(true);
    _isAppend ?
    WriteCommand(0x3C) : // MEM_WR_CONT
    WriteCommand(0x2C);  // MEM_WR
    WriteData(_buffer, _len, true);

    // need to wait DMA transmit finish if used DMA
    ChipSelect(false);
}


void RoundLcd::ChipSelect(bool _enable)
{
//    _enable ? LCD_CS_GPIO_Port->BSRR = (uint32_t) LCD_CS_Pin << 16U :
//            LCD_CS_GPIO_Port->BSRR = LCD_CS_Pin;
}


void RoundLcd::Reset(bool _enable)
{
    _enable ? gpio_set_level(GPIO_OUTPUT_IO_RES, 0): gpio_set_level(GPIO_OUTPUT_IO_RES, 1);
}


void RoundLcd::SetDataOrCommand(bool _isData)
{
    _isData ? gpio_set_level(GPIO_OUTPUT_IO_DC, 1): gpio_set_level(GPIO_OUTPUT_IO_DC, 0);
}


void RoundLcd::WriteCommand(uint8_t _cmd)
{
    SetDataOrCommand(false);
     spi_transaction_t t;
    t.length = sizeof(_cmd);
    t.tx_buffer = &_cmd;
    t.rx_buffer = &_cmd;
    spi_device_transmit(spi_handle, &t);
}


void RoundLcd::Write1Byte(uint8_t _data)
{
    SetDataOrCommand(true);
    spi_transaction_t t;
    t.length = sizeof(_data);
    t.tx_buffer = &_data;
    t.rx_buffer = &_data;
    spi_device_transmit(spi_handle, &t);
}


void RoundLcd::WriteData(uint8_t* _data, uint32_t _len, bool _useDma)
{
    SetDataOrCommand(true);
    spi_transaction_t t;
    t.length = _len;
    t.tx_buffer = _data;
    t.rx_buffer = _data;
    spi_device_transmit(spi_handle, &t);
}


void RoundLcd::SetBackLight(float _val)
{
    if (_val < 0) _val = 0;
    else if (_val > 1.0f) _val = 1.0f;

    gpio_set_level(GPIO_OUTPUT_IO_BLK, _val);
}

void RoundLcd::showFixImage()
{
   
}
