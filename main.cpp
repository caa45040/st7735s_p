
//OLED_SSD1306_BITMAP_DV67_color_UNO


//ヘッダーファイル
#include <Adafruit_GFX.h>
#include "hh.h"

//定義
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 160 // OLED display height, in pixels
//#define SCREEN_HEIGHT 80 // OLED display height, in pixels stm32G071
NA_ST7735_P display(SCREEN_WIDTH, SCREEN_HEIGHT);


// ビットマップデータ
uint8_t databytes[8] = 

{

0b01100110,
0b10101111,
0b10111111,
0b11011111,

0b01111110,
0b01111110,
0b00111100,
0b00011000

};


//初期化
void setup() {

  // I2Cアドレスは使用するディスプレイに合わせて変更する
  display.begin();

}//setup


//メインループ
void loop() {

  // 画面表示をクリア
  display.clearDisplay();

  //ビットマップの表示
  display.drawBitmap(0, 0, databytes, 8, 8,  WHITE);



  // テキストサイズを設定
  display.setTextSize(3);
  // テキスト色を設定
  //display.setTextColor(WHITE);
  display.setTextColor(RED);
  // テキストの開始位置を設定
  display.setCursor(20, 20);


  // 1行目に46を表示
  display.println("123");

  // 画面の左側に長方形(塗りつぶしなし)を描画
  // display.drawRect(左上x, 左上y, 幅, 高さ, 線の色)
  //display.drawRect(0, 0, 127, 80, WHITE);
  display.drawRect(0, 0, 127, 159, WHITE);

  // 描画バッファの内容を画面に表示
  display.display();

  delay(1000); //1秒待つ


unsigned long p_time;
p_time = millis();

  // 画面表示をクリア
  display.clearDisplay();
  // テキストサイズを設定
  display.setTextSize(3);
  // テキスト色を設定
  display.setTextColor(WHITE);
  // テキストの開始位置を設定
  display.setCursor(20, 20);

  // 1行目に46を表示
  display.println("456");
  // 描画バッファの内容を画面に表示
  display.display();

p_time = millis() - p_time;

  
  delay(2000); //1秒待つ


  // 画面表示をクリア
  display.clearDisplay();
  // テキストの開始位置を設定
  display.setCursor(20, 20);

  // 1行目に46を表示
  //display.println("time");
  display.print(p_time);
  display.println("ms");
  
  // 描画バッファの内容を画面に表示
  display.display();

  delay(3000); //1秒待つ

}//loop
