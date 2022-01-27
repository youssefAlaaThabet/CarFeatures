#if 1
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"
#include <Adafruit_GFX.h>
#include <MCUFRIEND_kbv.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
MCUFRIEND_kbv tft;
#include <TouchScreen.h>
#define MINPRESSURE 200
#define MAXPRESSURE 1000
//
//light
#define light 48
#define light2 49
//
const int SW_pin=42;
const int X_pin=14;
const int Y_pin=15;
//
//#define X_pin 8
//#define Y_pin 9
//#define SW_pin 42
#define A 33 //33
#define B 32//32
#define C 36 //36
#define D 37//37
#define E 38 //38
#define F 34//34
#define G 35//35
//
SemaphoreHandle_t Sem;
void Task1 (void *pvParameters); // screen with sound 
void Task2 (void *pvParameters);// light and control  indicator 
///
// Use pins 2 and 3 to communicate with DFPlayer Mini
static const uint8_t PIN_MP3_TX = 50; // Connects to module's RX 
static const uint8_t PIN_MP3_RX = 51; // Connects to module's TX 
SoftwareSerial softwareSerial(PIN_MP3_RX, PIN_MP3_TX);

// Create the Player object
DFRobotDFPlayerMini player;


//
// ALL Touch panels and wiring is DIFFERENT
// copy-paste results from TouchScreen_Calibr_native.ino
const int XP=8,XM=A2,YP=A3,YM=9; //240x320 ID=0x9341
const int TS_LEFT=137,TS_RT=894,TS_TOP=93,TS_BOT=893;


TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);
bool b =true;
Adafruit_GFX_Button on_btn, off_btn,Pause_btn, play_btn;

int pixel_x, pixel_y;     //Touch_getXY() updates global vars
bool Touch_getXY(void)
{
    TSPoint p = ts.getPoint();
    pinMode(YP, OUTPUT);      //restore shared pins
    pinMode(XM, OUTPUT);
    digitalWrite(YP, HIGH);   //because TFT control pins
    digitalWrite(XM, HIGH);
    bool pressed = (p.z > MINPRESSURE && p.z < MAXPRESSURE);
    if (pressed) {
        pixel_x = map(p.x, TS_LEFT, TS_RT, 0, tft.width()); //.kbv makes sense to me
        pixel_y = map(p.y, TS_TOP, TS_BOT, 0, tft.height());
    }
    return pressed;
}

#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF
int x =1;
bool paused=false;
void setup(void)
{
  uint16_t ID = tft.readID();
    Serial.print("TFT ID = 0x");
    Serial.println(ID, HEX);
    Serial.println("Calibrate for your Touch Panel");
    if (ID == 0xD3D3) ID = 0x9486; // write-only shield
    tft.begin(ID);
    tft.setRotation(0);            //PORTRAIT
    tft.fillScreen(BLACK);
    on_btn.initButton(&tft,  60, 290, 100, 40, WHITE, CYAN, BLACK, "Next", 2);
    off_btn.initButton(&tft, 180, 290, 100, 40, WHITE, CYAN, BLACK, "Previous", 2);
    //
    Pause_btn.initButton(&tft,60, 200,100,40, WHITE, CYAN, BLACK, "Pause", 2);
    play_btn.initButton(&tft, 180, 200, 100, 40, WHITE, CYAN, BLACK, "Play", 2);
    //
    on_btn.drawButton(false);
    off_btn.drawButton(false);
    Pause_btn.drawButton(false);
    play_btn.drawButton(false);
  // pinMode(22, OUTPUT);
   //pinMode(X_pin,INPUT);
     // pinMode(Y_pin,INPUT);

     //
     pinMode(light,OUTPUT);
          pinMode(light2,OUTPUT);

     //

    pinMode(SW_pin,INPUT);
   digitalWrite(A,HIGH);
  digitalWrite(B,HIGH);
  digitalWrite(C,LOW);
  digitalWrite(D,LOW);
  digitalWrite(E,HIGH);
    digitalWrite(F,HIGH);
  digitalWrite(G,HIGH);
      //softwareSerial.begin(9600);
      // player.pause();
    Serial.begin(9600);
   Sem = xSemaphoreCreateMutex();

   xTaskCreate(Task1,"Task1",1280,NULL,1,NULL);
   xTaskCreate(Task2,"Task2",1280,NULL,2,NULL);

 
}
void print_float_at(float val, int x, int y)
{
    char buf[10];
    int16_t x1, y1, w, h;
    tft.getTextBounds("000.0", x, y, &x1, &y1, &w, &h); 
    dtostrf(val, 5, 1, buf);   //e.g. 123.4
    tft.fillRect(x1, y1, w, h, YELLOW);
    tft.setTextColor(RED);
    tft.setCursor(x, y);
    tft.print(buf);
}
void Task1(void *pvParameters){
 
   Serial.println("This is Taskbbb1");
  const TickType_t xDelays = pdMS_TO_TICKS(500);
   Serial.println("pThis is Task1");
   
  while(1){
       Serial.println("pThis is Task1");

   xSemaphoreTake(Sem,portMAX_DELAY);
     tft.setTextColor(WHITE);
    tft.fillRect(0, 230, 240, 3, BLUE);
    tft.fillRect(0, 317, 240, 3, BLUE);
    tft.fillRect(0, 0, 240, 3, BLUE);
    tft.fillRect(237, 0, 3, 320, BLUE);
    tft.fillRect(0, 0, 3, 320, BLUE);
    tft.setCursor(65, 240);
    tft.setTextSize(2);
    //tft.drawText("  Song:" +x);
    tft.print("song:");
    tft.setCursor(150, 240);
    print_float_at(x,150 ,240);

    bool down = Touch_getXY();
    on_btn.press(down && on_btn.contains(pixel_x, pixel_y));
    off_btn.press(down && off_btn.contains(pixel_x, pixel_y));
     Pause_btn.press(down && Pause_btn.contains(pixel_x, pixel_y));
    play_btn.press(down && play_btn.contains(pixel_x, pixel_y));
    if (on_btn.justReleased())
        on_btn.drawButton();
    if (Pause_btn.justReleased())
        Pause_btn.drawButton();
    if (off_btn.justReleased())
        off_btn.drawButton();
    if (play_btn.justReleased())
        play_btn.drawButton();
    if (Pause_btn.justPressed()) {
        Pause_btn.drawButton(true);
        player.volume(30);
        player.pause();
        paused=true;
          delay(pdMS_TO_TICKS(6000));

   
    }
    if (play_btn.justPressed()) {
        play_btn.drawButton(true);
         digitalWrite(22, HIGH);
         if(b){
          softwareSerial.begin(9600);
           player.begin(softwareSerial);
           }
         player.volume(30);
         player.start();
         b=false;
                 paused=false;

           delay(pdMS_TO_TICKS(6000));

    }
    if (on_btn.justPressed()) {
      if(!paused){
        on_btn.drawButton(true);
                Serial.print("cliked1");
                player.volume(30);
                player.next();
                if(x==10){
                  x=1;
                }
                else{
                x++;
                }
                  delay(pdMS_TO_TICKS(6000));
      }
    }
    if (off_btn.justPressed()) {
      if(!paused){
        off_btn.drawButton(true);
        player.volume(30);
          if(x>1){
        player.previous();
      
         x--;
          }
        Serial.print("cliked2");
        digitalWrite(22, LOW);
          delay(pdMS_TO_TICKS(6000));

    }
    }  
     xSemaphoreGive(Sem);

    


 

  }
    
}

void Task2(void *pvParameters){

            
  while(1){
       Serial.println("This is Task2");

        xSemaphoreTake(Sem,portMAX_DELAY);
        delay(pdMS_TO_TICKS(10000));
              // Serial.println(analogRead(X_pin));
      // Serial.println(analogRead(Y_pin));
        
 if(analogRead(A14)>=0 &&analogRead(A14)<50&& analogRead(A15)>450 &&analogRead(A15)<550 ){// Down
   digitalWrite(A,HIGH);
  digitalWrite(B,HIGH);
  digitalWrite(C,LOW);
  digitalWrite(D,LOW);
  digitalWrite(E,HIGH);
    digitalWrite(F,HIGH);
  digitalWrite(G,HIGH);
  delay(pdMS_TO_TICKS(60000));
  }
  if(analogRead(A14)<=1023 && analogRead(A14)>=950 && analogRead(A15)>450 &&analogRead(A15)<550 ){// Up
    digitalWrite(A,HIGH);
  digitalWrite(B,HIGH);
  digitalWrite(C,HIGH);
  digitalWrite(D,HIGH);
  digitalWrite(E,HIGH);
    digitalWrite(F,HIGH);
  digitalWrite(G,LOW);
    delay(pdMS_TO_TICKS(60000));

  }
   if(analogRead(A15)>=0  &&analogRead(A15)<50&& analogRead(A14)>450 &&analogRead(A14)<550 ){ // left
    digitalWrite(A,HIGH);
  digitalWrite(B,HIGH);
  digitalWrite(C,HIGH);
  digitalWrite(D,LOW);
  digitalWrite(E,HIGH);
    digitalWrite(F,HIGH);
  digitalWrite(G,LOW);
    delay(pdMS_TO_TICKS(60000));

  }
   if(analogRead(A15)<=1023 && analogRead(A15)>=950 && analogRead(A14)>450 &&analogRead(A14)<550 ){ // right
    digitalWrite(A,HIGH);
  digitalWrite(B,HIGH);
  digitalWrite(C,HIGH);
  digitalWrite(D,LOW);
  digitalWrite(E,HIGH);
    digitalWrite(F,HIGH);
  digitalWrite(G,HIGH);
    delay(pdMS_TO_TICKS(60000));

  }
  Serial.println(analogRead(A13));
  if(analogRead(A13)<350&&analogRead(A13)>160){
    digitalWrite(light,HIGH);
    digitalWrite(light2,LOW);
        delay(pdMS_TO_TICKS(6000));
  }
   if(analogRead(A13)<150){
    digitalWrite(light,HIGH);
    digitalWrite(light2,HIGH);

  }
  if(analogRead(A13)>400){
     digitalWrite(light,LOW);
      digitalWrite(light2,LOW);

         delay(pdMS_TO_TICKS(6000));

  }

          xSemaphoreGive(Sem);
         // vTaskDelay(pdMS_TO_TICKS(10000));



  }
    
}

void loop(void)
{


  
    
   

}
#endif
