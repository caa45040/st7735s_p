
#ifdef __AVR__
#include <avr/pgmspace.h>
#elif defined(ESP8266) || defined(ESP32) || defined(ARDUINO_ARCH_RP2040)
#include <pgmspace.h>
#else
#define pgm_read_byte(addr)                                                    \
  (*(const unsigned char *)(addr)) ///< PROGMEM workaround for non-AVR
#endif

#if !defined(__ARM_ARCH) && !defined(ENERGIA) && !defined(ESP8266) &&          \
    !defined(ESP32) && !defined(__arc__)
#include <util/delay.h>
#endif

#include "hh.h"
#include <Adafruit_GFX.h>



#define NA_ST7735_P_swap(a, b)                                                     \
  (((a) ^= (b)), ((b) ^= (a)), ((a) ^= (b))) ///< No-temp-var swap operation

/*
//GPIOの設定1　開始

//GPIO
#define GPIO_A0_P A0
#define GPIO_A1_P A1
#define GPIO_A2_P A2
#define GPIO_A3_P A3

//GPIO
#define GPIO_A0(s) digitalWrite(GPIO_A0_P,s)
#define GPIO_A1(s) digitalWrite(GPIO_A1_P,s)
#define GPIO_A2(s) digitalWrite(GPIO_A2_P,s)
#define GPIO_A3(s) digitalWrite(GPIO_A3_P,s)

#define GPIO_RD_P     GPIO_A3_P //RD=1
#define GPIO_WR_P     GPIO_A2_P //WR=1
#define GPIO_RS_P     GPIO_A1_P //RS=0
#define GPIO_RESET_P  GPIO_A0_P //RESET=1


#define GPIO_RD(y)     GPIO_A3(y) //RD=1
#define GPIO_WR(y)     GPIO_A2(y) //WR=1
#define GPIO_RS(y)     GPIO_A1(y) //RS=0
#define GPIO_RESET(y)  GPIO_A0(y) //RESET=1


#define GPIO_DB7  7
#define GPIO_DB6  6
#define GPIO_DB5  5
#define GPIO_DB4  4
#define GPIO_DB3  3
#define GPIO_DB2  2
#define GPIO_DB1  A5
#define GPIO_DB0  A4

//GPIOの設定1　終了
*/

/*

//GPIOの設定2　開始

//GPIO
#define GPIO_A4_P A4
#define GPIO_A5_P A5
#define GPIO_D3_P 3
#define GPIO_D2_P 2

//GPIO
#define GPIO_A4(s) digitalWrite(GPIO_A4_P,s)
#define GPIO_A5(s) digitalWrite(GPIO_A5_P,s)
#define GPIO_D3(s) digitalWrite(GPIO_D3_P,s)
#define GPIO_D2(s) digitalWrite(GPIO_D2_P,s)


#define GPIO_RD_P     GPIO_A4_P //RD=1
#define GPIO_WR_P     GPIO_D3_P //WR=1
#define GPIO_RS_P     GPIO_A5_P //RS=0
#define GPIO_RESET_P  GPIO_D2_P //RESET=1

#define GPIO_RD(y)    GPIO_A4(y) //RD=1
#define GPIO_WR(y)    GPIO_D3(y) //WR=0
#define GPIO_RS(y)    GPIO_A5(y) //RS=0
#define GPIO_RESET(y) GPIO_D2(y) //RESET=1


#define GPIO_DB7  A0
#define GPIO_DB6   7
#define GPIO_DB5  A1
#define GPIO_DB4   6
#define GPIO_DB3  A2
#define GPIO_DB2   5
#define GPIO_DB1  A3
#define GPIO_DB0   4

//GPIOの設定2　終了

*/


///*

//GPIOの設定3　開始

//GPIO
#define GPIO_D32_P 32
#define GPIO_D33_P 33
#define GPIO_D19_P 19
#define GPIO_D18_P 18

//GPIO
#define GPIO_D32(s) digitalWrite(GPIO_D32_P,s)
#define GPIO_D33(s) digitalWrite(GPIO_D33_P,s)
#define GPIO_D19(s) digitalWrite(GPIO_D19_P,s)
#define GPIO_D18(s) digitalWrite(GPIO_D18_P,s)


#define GPIO_RD_P     GPIO_D33_P //RD=1
#define GPIO_WR_P     GPIO_D18_P //WR=1
#define GPIO_RS_P     GPIO_D32_P //RS=0
#define GPIO_RESET_P  GPIO_D19_P //RESET=1

#define GPIO_RD(y)    GPIO_D33(y) //RD=1
#define GPIO_WR(y)    GPIO_D18(y) //WR=0
#define GPIO_RS(y)    GPIO_D32(y) //RS=0
#define GPIO_RESET(y) GPIO_D19(y) //RESET=1


#define GPIO_DB7  14
#define GPIO_DB6  4
#define GPIO_DB5  27
#define GPIO_DB4  16
#define GPIO_DB3  26
#define GPIO_DB2  17
#define GPIO_DB1  25
#define GPIO_DB0  5

//GPIOの設定3　終了

//*/


void NA_ST7735_P::GPIO_8BIT(uint8_t s)
{
  digitalWrite( GPIO_DB7, (s >> 7) & 1);
  digitalWrite( GPIO_DB6, (s >> 6) & 1);
  digitalWrite( GPIO_DB5, (s >> 5) & 1);
  digitalWrite( GPIO_DB4, (s >> 4) & 1);
  digitalWrite( GPIO_DB3, (s >> 3) & 1);
  digitalWrite( GPIO_DB2, (s >> 2) & 1);
  digitalWrite( GPIO_DB1, (s >> 1) & 1);
  digitalWrite( GPIO_DB0, s & 1);
} //GPIO_8BIT

//コマンドの書き込み
void NA_ST7735_P::LCD_Write_CMD(uint8_t a)
{
  GPIO_RS(0); //A0=0;
  GPIO_8BIT(a);//P1=a; data
  GPIO_WR(0);//WRB=0;
  GPIO_WR(1);//WRB=1;
} //LCD_Write_CMD


//データ書き込み
void NA_ST7735_P::LCD_Write_Data(uint8_t a)
{
  GPIO_RS(1);//A0=1;
  GPIO_8BIT(a);//P1=a; data
  GPIO_WR(0);//WRB=0;
  GPIO_WR(1);//WRB=1;
} //LCD_Write_Data


//液晶の初期化処理
void NA_ST7735_P::TXDT144TF_ST7735S_Init(void)
{

  //----------  ST7735S Reset Sequence  --------//

  GPIO_RESET(1);//LCD_RESET=1;

  delay(1); //Delay 1ms

  GPIO_RESET(0);//LCD_RESET=0;

  delay(1); //Delay 1ms

  GPIO_RESET(1);//LCD_RESET=1;

  delay(120); //Delay 120ms

  LCD_Write_CMD(0x01);//SOFTWARE RESET
  delay(50);

  LCD_Write_CMD(0x11);//SLEEP OUT
  delay(200);

  LCD_Write_CMD(0x29);//display on
  delay(100);

  LCD_Write_CMD(0x3a);//Interface pixel format
  LCD_Write_Data(0x05);//16bit mode
  delay(100);

  LCD_Write_CMD(0x36);//RGB-RGR format
  LCD_Write_Data(0x08);//RGB mode
  delay(100);

} //TXDT144TF_ST7735S_Init



NA_ST7735_P::NA_ST7735_P(uint8_t w, uint8_t h)
    : Adafruit_GFX(w, h), buffer(NULL)

{
}


//バッファのクリア
NA_ST7735_P::~NA_ST7735_P(void) {
  if (buffer) {
    free(buffer);
    buffer = NULL;
  }
}//~NA_ST7735_P



//初期処理
bool NA_ST7735_P::begin(void) {

  if ((!buffer) && !(buffer = (uint16_t *)malloc(WIDTH * HEIGHT * 2)   ))
    return false;

  //バッファーのクリア
  clearDisplay();

  //ポートのモード設定
  //アウトプットモード
  pinMode(GPIO_DB7, OUTPUT);
  pinMode(GPIO_DB6, OUTPUT);
  pinMode(GPIO_DB5, OUTPUT);
  pinMode(GPIO_DB4, OUTPUT);
  pinMode(GPIO_DB3, OUTPUT);
  pinMode(GPIO_DB2, OUTPUT);

  pinMode(GPIO_RESET_P, OUTPUT);
  pinMode(GPIO_RS_P, OUTPUT);
  pinMode(GPIO_WR_P, OUTPUT);
  pinMode(GPIO_RD_P, OUTPUT);
  pinMode(GPIO_DB0, OUTPUT);
  pinMode(GPIO_DB1, OUTPUT);

  //ポートの初期化
  GPIO_RD(1);//RD=1
  GPIO_WR(1);//WR=1
  GPIO_RS(0);//RS=0
  GPIO_RESET(1);//RESET=1

  delay(500); //0.5秒待つ

  //液晶の初期化処理
  TXDT144TF_ST7735S_Init();

  //画面の書き込み開始
  display();

  return true; // Success
}//begin




//点の表示
void NA_ST7735_P::drawPixel(int16_t x, int16_t y, uint16_t color) {
  if ((x >= 0) && (x < width()) && (y >= 0) && (y < height())) {
    // Pixel is in-bounds. Rotate coordinates if needed.
    switch (getRotation()) {
    case 1:
      NA_ST7735_P_swap(x, y);
      x = WIDTH - x - 1;
      break;
    case 2:
      x = WIDTH - x - 1;
      y = HEIGHT - y - 1;
      break;
    case 3:
      NA_ST7735_P_swap(x, y);
      y = HEIGHT - y - 1;
      break;
    }//end switch

    //ドットのカラーの設定
    buffer[ y_shift[y] | x ] = color;

  }//if
}//drawPixel

//バッファのクリア
void NA_ST7735_P::clearDisplay(void) {
  memset(buffer, 0, WIDTH * HEIGHT * 2 );
}




bool NA_ST7735_P::getPixel(int16_t x, int16_t y) {
  if ((x >= 0) && (x < width()) && (y >= 0) && (y < height())) {
    // Pixel is in-bounds. Rotate coordinates if needed.
    switch (getRotation()) {
    case 1:
      NA_ST7735_P_swap(x, y);
      x = WIDTH - x - 1;
      break;
    case 2:
      x = WIDTH - x - 1;
      y = HEIGHT - y - 1;
      break;
    case 3:
      NA_ST7735_P_swap(x, y);
      y = HEIGHT - y - 1;
      break;
    }
    return (buffer[ y_shift[y] | x ] );
  }
  return false; // Pixel out of bounds
}



uint16_t *NA_ST7735_P::getBuffer(void) { return buffer; }


#define GPIO_0to31SET_REG   *((volatile unsigned long *)GPIO_OUT_W1TS_REG)
#define GPIO_0to31CLR_REG   *((volatile unsigned long *)GPIO_OUT_W1TC_REG)


//aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
void NA_ST7735_P::HS_GPIO_8BIT_A(uint8_t s)
{
  
int work_a = 0;
//------------------------------------------------------
//7bit
work_a = work_a |( ( (s >> 7) & 1) << 14) ;
//------------------------------------------------------
//6bit
work_a = work_a |( ( (s >> 6) & 1) << 4);
//------------------------------------------------------
//5bit
work_a = work_a |( ( (s >> 5) & 1) << 27);
//------------------------------------------------------
//4bit
work_a = work_a |( ( (s >> 4) & 1) << 16);
//------------------------------------------------------
//3bit
work_a = work_a |( ( (s >> 3) & 1) << 26);
//------------------------------------------------------
//2bit
work_a = work_a |( ( (s >> 2) & 1) << 17);
//------------------------------------------------------
//1bit
work_a = work_a |( ( (s >> 1) & 1) << 25);
//------------------------------------------------------
//0bit
work_a = work_a |( ( s & 1) << 5);

  //[0-31]レジスターの書き込み
 
  //セットレジスターの設定
  GPIO_0to31SET_REG = GPIO_0to31SET_REG | work_a;

//delayMicroseconds(3);

  //リセットレジスターの設定　反転させてマスクして足しこむ
  GPIO_0to31CLR_REG = GPIO_0to31CLR_REG |

(

(  (1 << 14)|(1 << 4)|(1 << 27)|(1 << 16) | (1 << 26)|(1 << 17)|(1 << 25)|(1 << 5)  )

 & (~work_a)

);

//delayMicroseconds(3);

} //GPIO_8BIT
//aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa

//bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb
void NA_ST7735_P::HS_GPIO_8BIT_B(uint8_t s)
{
//------------------------------------------------------
   //7bit
   if( ( (s >> 7) & 1) == 0 ) {
     GPIO_0to31CLR_REG = GPIO_0to31CLR_REG | (1<<14);
   } else {
     GPIO_0to31SET_REG = GPIO_0to31SET_REG | (1<<14);
   } //end if
//------------------------------------------------------
  //6bit
  if( ( (s >> 6) & 1) == 0 ) {
     GPIO_0to31CLR_REG = GPIO_0to31CLR_REG | (1<<4);
   } else {
     GPIO_0to31SET_REG = GPIO_0to31SET_REG | (1<<4);
   } //end if
//------------------------------------------------------
   //5bit
   if( ( (s >> 5) & 1) == 0 ) {
     GPIO_0to31CLR_REG = GPIO_0to31CLR_REG | (1<<27);
   } else {
     GPIO_0to31SET_REG = GPIO_0to31SET_REG | (1<<27);
   } //end if
//------------------------------------------------------
   //4bit
   if( ( (s >> 4) & 1) == 0 ) {
     GPIO_0to31CLR_REG = GPIO_0to31CLR_REG | (1<<16);
   } else {
     GPIO_0to31SET_REG = GPIO_0to31SET_REG | (1<<16);
   } //end if
//------------------------------------------------------
   //3bit
   if( ( (s >> 3) & 1) == 0 ) {
     GPIO_0to31CLR_REG = GPIO_0to31CLR_REG | (1<<26);
   } else {
     GPIO_0to31SET_REG = GPIO_0to31SET_REG | (1<<26);
   } //end if
//------------------------------------------------------
   //2bit
   if( ( (s >> 2) & 1) == 0 ) {
     GPIO_0to31CLR_REG = GPIO_0to31CLR_REG | (1<<17);
   } else {
     GPIO_0to31SET_REG = GPIO_0to31SET_REG | (1<<17);
   } //end if
//------------------------------------------------------
   //1bit
   if( ( (s >> 1) & 1) == 0 ) {
     GPIO_0to31CLR_REG = GPIO_0to31CLR_REG | (1<<25);
   } else {
     GPIO_0to31SET_REG = GPIO_0to31SET_REG | (1<<25);
   } //end if
//------------------------------------------------------
   //0bit
   if( ( (s >> 0) & 1) == 0 ) {
     GPIO_0to31CLR_REG = GPIO_0to31CLR_REG | (1<<5);
   } else {
     GPIO_0to31SET_REG = GPIO_0to31SET_REG | (1<<5);
   } //end if  
} //GPIO_8BIT


//bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb
//bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb


//セット
#define GPIO_0to31SET_REG_18  (GPIO_0to31SET_REG=GPIO_0to31SET_REG|(1<<GPIO_D18_P))

//クリア
#define GPIO_0to31CLR_REG_18  (GPIO_0to31CLR_REG=GPIO_0to31CLR_REG|(1<<GPIO_D18_P))


//WR信号を出力
void NA_ST7735_P::HS_GPIO_WR(int o_a) {


   //ゼロの場合は、クリア　それ以外は、セット
   if( o_a == 0 ) {

     GPIO_0to31CLR_REG = GPIO_0to31CLR_REG | (1<<18);

   } else {

     GPIO_0to31SET_REG = GPIO_0to31SET_REG | (1<<18);

   } //end if

} // end GPIO_WR


//画面のリフレッシュ
void NA_ST7735_P::display(void) {

#if defined(ESP8266)
  // ESP8266 needs a periodic yield() call to avoid watchdog reset.
  // With the limited size of SSD1306 displays, and the fast bitrate
  // being used (1 MHz or more), I think one yield() immediately before
  // a screen write and one immediately after should cover it.  But if
  // not, if this becomes a problem, yields() might be added in the
  // 32-byte transfer condition below.
  yield();
#endif

//buffer = (uint16_t*)yu; //とりま 仮想VRAMに実態を設定


//  uint16_t count = WIDTH * ((HEIGHT + 7) / 8);
//  uint16_t *ptr = buffer;

//    while (count--) {
//
//      //WIRE_WRITE(*ptr++);
//    }

//どつかから拾ってきたint を　文字列　にするアルゴリズム
//int a=0x01234567;
//char s[4];
//int *p;
//p=(int*)s;
//*p=a;

  //画面の書き込み開始
  LCD_Write_CMD(0x2C); //memory write

    GPIO_RS(1);//A0=1; //データモード

////debug start
//        HS_GPIO_8BIT_A(0xff); delay(3000);
//        HS_GPIO_8BIT_A(0x00); delay(1000);
//
//
//        for(int s=0;s<8;s++){
//                        HS_GPIO_8BIT_A(1 << s);delay(1000);
//        }
//        
//        HS_GPIO_8BIT_A(0x00); delay(1000);
////debug end

  char vg[2];
  uint16_t *ptr;  //ポインター型
  ptr = (uint16_t*)vg; //ポインターに実態の文字列を強制型変換して入れる

  for (int i = 0; i < (HEIGHT * WIDTH); i++) {
 
      *ptr=buffer[i]; //ptr と　vg は、ポインタでリンクしている

        //GPIO_8BIT(vg[1]); //汎用
        //なぜかウェートが必要
        GPIO_0to31SET_REG = GPIO_0to31SET_REG | (1<<5);
        GPIO_0to31SET_REG = GPIO_0to31SET_REG | (1<<5);
        GPIO_0to31SET_REG = GPIO_0to31SET_REG | (1<<5);
        HS_GPIO_8BIT_A(vg[1]); //8ビット出力 esp32

        //GPIO_WR(0);//WRB=0; 10
        //GPIO_WR(1);//WRB=1; 10
        GPIO_0to31CLR_REG_18;
        GPIO_0to31SET_REG_18;


        //GPIO_8BIT(vg[0]); //汎用
        HS_GPIO_8BIT_A(vg[0]); //8ビット出力 esp32

        //GPIO_WR(0);//WRB=0; 10
        //GPIO_WR(1);//WRB=1; 10
        GPIO_0to31CLR_REG_18;
        GPIO_0to31SET_REG_18;
        
  }//i

#if defined(ESP8266)
  yield();
#endif
}//display
