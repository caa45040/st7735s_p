
#ifndef _NA_ST7735_P_H_
#define _NA_ST7735_P_H_

#include <Adafruit_GFX.h>

#ifndef NO_ADAFRUIT_NA_ST7735_P_COLOR_COMPATIBILITY
#define BLACK NA_ST7735_P_BLACK     ///< Draw 'off' pixels
#define WHITE NA_ST7735_P_WHITE     ///< Draw 'on' pixels
#define RED NA_ST7735_P_RED     ///< Draw 'on' pixels
//#define INVERSE NA_ST7735_P_INVERSE ///< Invert pixels
#endif
/// fit into the SSD1306_ naming scheme
#define NA_ST7735_P_BLACK 0x0000   ///< Draw 'off' pixels
#define NA_ST7735_P_WHITE 0xffff   ///< Draw 'on' pixels
#define NA_ST7735_P_RED 0b1111100000000000 //red
 
///< Draw 'on' pixels
//#define NA_ST7735_P_INVERSE 2 ///< Invert pixels


class NA_ST7735_P : public Adafruit_GFX {
public:

  NA_ST7735_P(uint8_t w, uint8_t h);

  ~NA_ST7735_P(void);

  bool begin(void);
  void display(void);
  void clearDisplay(void);
  void drawPixel(int16_t x, int16_t y, uint16_t color);
  bool getPixel(int16_t x, int16_t y);
  uint16_t *getBuffer(void);

  void GPIO_8BIT(uint8_t s);
  void LCD_Write_CMD(uint8_t ww);
  void LCD_Write_Data(uint8_t ii);
  void TXDT144TF_ST7735S_Init(void);

  void HS_GPIO_8BIT_A(uint8_t s);
  void HS_GPIO_8BIT_B(uint8_t s);
  void HS_GPIO_WR(int o_a);

protected:


  uint16_t *buffer; ///< Buffer data used for display buffer. Allocated when
                   ///< begin method is called.


uint16_t y_shift[160]={
0,128,256,384,512,640,768,896,1024,1152,
1280,1408,1536,1664,1792,1920,2048,2176,2304,2432,
2560,2688,2816,2944,3072,3200,3328,3456,3584,3712,
3840,3968,4096,4224,4352,4480,4608,4736,4864,4992,
5120,5248,5376,5504,5632,5760,5888,6016,6144,6272,
6400,6528,6656,6784,6912,7040,7168,7296,7424,7552,
7680,7808,7936,8064,8192,8320,8448,8576,8704,8832,
8960,9088,9216,9344,9472,9600,9728,9856,9984,10112,
10240,10368,10496,10624,10752,10880,11008,11136,11264,11392,
11520,11648,11776,11904,12032,12160,12288,12416,12544,12672,
12800,12928,13056,13184,13312,13440,13568,13696,13824,13952,
14080,14208,14336,14464,14592,14720,14848,14976,15104,15232,
15360,15488,15616,15744,15872,16000,16128,16256,16384,16512,
16640,16768,16896,17024,17152,17280,17408,17536,17664,17792,
17920,18048,18176,18304,18432,18560,18688,18816,18944,19072,
19200,19328,19456,19584,19712,19840,19968,20096,20224,20352
};



};

#endif // _NA_ST7735_P_H_
