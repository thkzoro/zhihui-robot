#ifndef ESP_IDF_V4_4_LCDFONTLIB_H
#define ESP_IDF_V4_4_LCDFONTLIB_H
#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    unsigned char Index[2];
    unsigned char Msk[24];
}typFNT_GB12;

typedef struct {
    unsigned char Index[2];
    unsigned char Msk[32];
}typFNT_GB16;

typedef struct {
    unsigned char Index[2];
    unsigned char Msk[72];
}typFNT_GB24;

typedef struct {
    unsigned char Index[2];
    unsigned char Msk[128];
}typFNT_GB32;

unsigned char *get_ascii_1206_font(int ascii);
unsigned char *get_ascii_1608_font(int ascii);
unsigned char *get_ascii_2412_font(int ascii);
unsigned char *get_ascii_3216_font(int ascii);

//unsigned char *get_GB12_font(int GB);
//unsigned char *get_GB16_font(int GB);
//unsigned char *get_GB24_font(int GB);
//unsigned char *get_GB32_font(int GB);

//中文字体需要自行取字模
#ifdef __cplusplus
}
#endif

#endif //ESP_IDF_V4_4_LCDFONTLIB_H
