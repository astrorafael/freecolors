/* ************************************************************************** */ 
/*                           ORIGINAL COPYRIGHT TEXT                          */ 
/* ************************************************************************** */ 

/*
  This sketch reads the sensor and creates a color bar graph on a tiny TFT

  Designed specifically to work with the Adafruit AS7262 breakout and 160x18 tft
  ----> http://www.adafruit.com/products/3779
  ----> http://www.adafruit.com/product/3533
  
  These sensors use I2C to communicate. The device's I2C address is 0x49
  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!
  
  Written by Dean Miller for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
  */

/* ************************************************************************** */ 
/*                             HARDWARE CONNECTIONS                           */ 
/* ************************************************************************** */ 

/*

              Arduino Nano        AS7262 Spectral Sensor
              ============        ======================
                    /
                   |  A4 (SDA)  -----> SDA 
              I2C  |  A5 (SCL)  -----> SCL 
                   \
                   /
                   |   3.3V    <====> 3.3V 
              Pwr  |   GND     <====> GND
                   \

              Arduino Nano        miniTFTWing
              ============        ===========
                   /
                   | D11 (MOSI) -----> MOSI
              SPI <  D13 (SCK)  -----> SCLK
                   |     D5     -----> CS
                   |     D6     -----> DC
                   \
                   /
                   |  A4 (SDA)  -----> SDA (+10k pullup)
              I2C  |  A5 (SCL)  -----> SCL (+10k pullup)
                   \
                   /
                   |   3.3V    <====> 3.3V 
              Pwr  |   GND     <====> GND
                   \


*/

/* ************************************************************************** */ 
/*                           INCLUDE HEADERS SECTION                          */
/* ************************************************************************** */ 

// FreeRTOS modules
#include <Arduino_FreeRTOS.h> // This must be the first include file if using FreeRTOS
#include <semphr.h>

// Adafruit Spectral Sensor library
#include <Adafruit_AS726x.h>

// Adafruit Graphics libraries
#include <Adafruit_GFX.h>          // Core graphics library
#include <Adafruit_ST7735.h>       // Hardware-specific library
#include <Adafruit_miniTFTWing.h>  // Seesaw library for the miniTFT Wing display

// Support for the Git version tags
#include "git-version.h"


/* ************************************************************************** */ 
/*                                DEFINEs SECTION                             */
/* ************************************************************************** */ 

// ---------------------------------------------------------
// Define which Arduino nano pins will control the TFT Reset, 
// SPI Chip Select (CS) and SPI Data/Command DC
// ----------------------------------------------------------

#define TFT_RST -1  // miniTFTwing uses the seesaw chip for resetting to save a pin
#define TFT_CS   5 // Arduino nano D5 pin
#define TFT_DC   6 // Arduini nano D6 pin


// Short delay in screens (milliseconds)
#define SHORT_DELAY 200

// -----------------------------------------
// Some predefined colors for the 16 bit TFT
// -----------------------------------------

#define BLACK   0x0000
#define GRAY    0x8410
#define WHITE   0xFFFF
#define RED     0xF800
#define ORANGE  0xFA60
#define YELLOW  0xFFE0  
#define LIME    0x07FF
#define GREEN   0x07E0
#define CYAN    0x07FF
#define AQUA    0x04FF
#define BLUE    0x001F
#define MAGENTA 0xF81F
#define PINK    0xF8FF

/* ************************************************************************** */ 
/*                        CUSTOM CLASES & DATA TYPES                          */
/* ************************************************************************** */ 

typedef struct {
  float    calibratedValues[AS726x_NUM_CHANNELS];
  uint16_t rawValues[AS726x_NUM_CHANNELS];
  uint8_t  gain;           // device gain multiplier
  uint8_t  exposure;       // device integration time in steps of 2.8 ms
  uint8_t  temperature;    // device internal temperature
} sensor_info_t;

typedef struct {
  uint8_t backlight;    // miniTFTWing backlight value in percentage
} tft_info_t;

// Menu action function pointer as a typedef
typedef void (*menu_action_t)(void);

/* ************************************************************************** */ 
/*                          GLOBAL VARIABLES SECTION                          */
/* ************************************************************************** */ 

// The Adafruit SeeSaw chip that controls the TFT by I2C
Adafruit_miniTFTWing ss;

// The Adafruit TFT display object based on ST7735
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

// buffer to hold raw & calibrated values as well as exposure time and gain
tft_info_t tft_info;

//create the 6 channel spectral sensor object
Adafruit_AS726x ams;

// buffer to hold raw & calibrated values as well as exposure time and gain
sensor_info_t sensor_info;

SemaphoreHandle_t xBinarySemaphoreKey = NULL;

/* ************************************************************************** */ 
/*                      GUI STATE MACHINE DECLARATIONS                        */
/* ************************************************************************** */ 

// Events generated by user
enum gui_events {
  GUI_NO_EVENT      = 0,
  GUI_KEY_A_PRESSED,
  GUI_KEY_B_PRESSED,
  GUI_JOY_PRESSED,
  GUI_JOY_UP,
  GUI_JOY_DOWN,
  GUI_JOY_LEFT,
  GUI_JOY_RIGHT
};

// TFT Screens as states
enum gui_state {
  GUI_LIGHT_SCREEN     = 0,
  GUI_GAIN_SCREEN,
  GUI_EXPOSURE_SCREEN,
  GUI_READINGS_SCREEN
};

// --------------------------------------------
// State Machine Actions (Forward declarations)
// --------------------------------------------

static void act_idle();
static void act_gain_enter();
static void act_gain_up();
static void act_gain_down();
static void act_light_enter();
static void act_light_up();
static void act_light_down();
static void act_exposure_enter();
static void act_exposure_up();
static void act_exposure_down();
static void act_readings_enter();


// Action to execute as a function of current state and event
// This table is held in Flash memory to save precious RAM
// Use of PROGMEM and pgm_xxx() functions is necessary
static menu_action_t get_action(uint8_t state, uint8_t event)
{
  static const menu_action_t menu_action[][4] PROGMEM = {
    // BACKLIGHT SCREEN | GAIN SCREEN      |   EXPOSURE SCREEN    | READINGS SCREEN
    //------------------+------------------+----------------------+-----------------
    { act_idle,           act_idle,           act_idle,               act_readings_enter }, // GUI_NO_EVENT
    { act_light_up,       act_gain_up,        act_exposure_up,        act_idle           }, // GUI_KEY_A_PRESSED
    { act_light_down,     act_gain_down,      act_exposure_down,      act_idle           }, // GUI_KEY_B_PRESSED
    { act_idle,           act_idle,           act_idle,               act_idle           }, // GUI_JOY_PRESSED
    { act_light_up,       act_gain_up,        act_exposure_up,        act_idle           }, // GUI_JOY_UP
    { act_light_down,     act_gain_down,      act_exposure_down,      act_idle           }, // GUI_JOY_DOWN
    { act_readings_enter, act_light_enter,    act_gain_enter,         act_exposure_enter }, // GUI_JOY_LEFT
    { act_gain_enter,     act_exposure_enter, act_readings_enter,     act_light_enter    }  // GUI_JOY_RIGHT
  };
  return (menu_action_t) pgm_read_ptr(&menu_action[event][state]);
}

/* -------------------------------------------------------------------------- */ 

// Next state to proceed as a function of current state and event
// This table is held in Flash memory to save precious RAM
// Use of PROGMEM and pgm_xxx() functions is necessary
static uint8_t get_next_screen(uint8_t state, uint8_t event)
{
  static const PROGMEM uint8_t next_screen[][4] = {
    // BACKLIGHT SCREEN | GAIN SCREEN      |   EXPOSURE SCREEN    | READINGS SCREEN
    //------------------+------------------+----------------------+-----------------
      { GUI_LIGHT_SCREEN,    GUI_GAIN_SCREEN,     GUI_EXPOSURE_SCREEN,   GUI_READINGS_SCREEN }, // GUI_NO_EVENT
      { GUI_LIGHT_SCREEN,    GUI_GAIN_SCREEN,     GUI_EXPOSURE_SCREEN,   GUI_READINGS_SCREEN }, // GUI_KEY_A_PRESSED
      { GUI_LIGHT_SCREEN,    GUI_GAIN_SCREEN,     GUI_EXPOSURE_SCREEN,   GUI_READINGS_SCREEN }, // GUI_KEY_B_PRESSED
      { GUI_LIGHT_SCREEN,    GUI_GAIN_SCREEN,     GUI_EXPOSURE_SCREEN,   GUI_READINGS_SCREEN }, // GUI_JOY_PRESSED
      { GUI_LIGHT_SCREEN,    GUI_GAIN_SCREEN,     GUI_EXPOSURE_SCREEN,   GUI_READINGS_SCREEN }, // GUI_JOY_UP
      { GUI_LIGHT_SCREEN,    GUI_GAIN_SCREEN,     GUI_EXPOSURE_SCREEN,   GUI_READINGS_SCREEN }, // GUI_JOY_DOWN
      { GUI_READINGS_SCREEN, GUI_LIGHT_SCREEN,    GUI_GAIN_SCREEN,       GUI_EXPOSURE_SCREEN }, // GUI_JOY_LEFT
      { GUI_GAIN_SCREEN,     GUI_EXPOSURE_SCREEN, GUI_READINGS_SCREEN,   GUI_LIGHT_SCREEN    }  // GUI_JOY_RIGHT
    };
  return pgm_read_byte(&next_screen[event][state]);
}



/* ************************************************************************** */ 
/*                              HELPER FUNCTIONS                              */
/* ************************************************************************** */ 

// Reads miniTFTWing buttons & joystick and produces events
static uint8_t read_buttons()
{
  extern Adafruit_miniTFTWing ss;

  uint8_t event = GUI_NO_EVENT;

  // miniTFT wing buttons;
  uint32_t buttons;

  // read buttons via the I2C SeeSaw chip in miniTFTWing
  // These buttons are active-low logic
  buttons = ss.readButtons();


  if ((buttons & TFTWING_BUTTON_A) == 0) {
       //Serial.println("A pressed");
       event = GUI_KEY_A_PRESSED;
  } else if ((buttons & TFTWING_BUTTON_B) == 0) {
       //Serial.println("B pressed");
       event = GUI_KEY_B_PRESSED;
  } else if ((buttons & TFTWING_BUTTON_UP) == 0) {
       //Serial.println("Joy up");
       event = GUI_JOY_UP;
  } else if ((buttons & TFTWING_BUTTON_DOWN) == 0) {
       //Serial.println("Joy down");
       event = GUI_JOY_DOWN;
  } else if ((buttons & TFTWING_BUTTON_LEFT) == 0) {
       //Serial.println("Joy left");
       event = GUI_JOY_LEFT;
  } else if ((buttons & TFTWING_BUTTON_RIGHT) == 0) {
       //Serial.println("Joy right");
       event = GUI_JOY_RIGHT;
  } else if ((buttons & TFTWING_BUTTON_SELECT) == 0) {
       //Serial.println("Joy select");
       event = GUI_JOY_PRESSED;
  }
  return event;
}

/* ************************************************************************** */ 

static int read_sensor()
{
  extern Adafruit_AS726x ams;
  extern sensor_info_t sensor_info;
  uint8_t freshData = 0;

  if(ams.dataReady()) {
    freshData = 1;
    ams.readCalibratedValues(sensor_info.calibratedValues);
    ams.readRawValues(sensor_info.rawValues);
    sensor_info.temperature = ams.readTemperature();
    Serial.print('+');
  } else {
    Serial.print('-');
  }
  return freshData;
}

/* ************************************************************************** */ 

static void display_bars()
{
  //extern sensor_info_t   sensor_info;
  //extern Adafruit_ST7735 tft;
  uint16_t barWidth = (tft.width()) / AS726x_NUM_CHANNELS;
  bool     refresh = false;

  // array of predefined bar colors
  // This table is held in Flash memory to save precious RAM
  // Use of PROGMEM and pgm_xxx() functions is necessary
  static const PROGMEM uint16_t colors[AS726x_NUM_CHANNELS] = {
      MAGENTA,
      BLUE,
      GREEN,
      YELLOW,
      ORANGE,
      RED
  };

  // Display bar buffers, used to minimize redrawings
  static uint16_t height[AS726x_NUM_CHANNELS][2];
  static uint8_t  curBuf = 0;                     // current buffer 
  return;

}

/* ************************************************************************** */ 


static void display_gain()
{
  //extern sensor_info_t sensor_info;

  // strings to display on TFT
  // It is not worth to place these strings in Flash
  static const char* gain_table[] = {
    "1x",
    "3.7x",
    "16x",
    "64x"
  };

  tft.fillScreen(ST7735_BLACK);
  // Display the "Gain" sttring in TFT
  tft.setTextSize(3); // 3x the original font
  tft.setCursor(tft.height()/3, 0);
  tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
  tft.print("Gain");
  // Display the gain value string in TFT
  //tft.setCursor(tft.height()/3, tft.width()/3);
  //tft.setTextColor(ST7735_YELLOW, ST7735_BLACK);
  //tft.print(gain_table[sensor_info.gain]);
  
}

/* ************************************************************************** */ 


static void display_backlight()
{
  //extern tft_info_t tft_info;
  
  tft.fillScreen(ST7735_BLACK);
  // Display the "Gain" sttring in TFT
  tft.setTextSize(3); // 3x the original font
  tft.setCursor(0, 0);
  tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
  tft.print("Baklight");
  // Display the gain value string in TFT
  //tft.setCursor(tft.height()/3, tft.width()/3);
  //tft.setTextColor(ST7735_YELLOW, ST7735_BLACK);
  //tft.print(tft_info.backlight); tft.print(" %");
  
}

/* ************************************************************************** */ 

static void display_exposure()
{
  //extern sensor_info_t sensor_info;

  tft.fillScreen(ST7735_BLACK);
  // Display the "Gain" sttring in TFT
  tft.setTextSize(3);
  tft.setCursor(0,0);
  tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
  tft.print("Exposure");
  // Display the exposure value string in TFT
  //tft.setCursor(0, tft.width()/3);
  //tft.setTextColor(ST7735_YELLOW, ST7735_BLACK);
  //tft.print(sensor_info.exposure*EXPOSURE_UNIT,1); tft.print(" ms");
}


/* ************************************************************************** */ 
/*                      STATE MACHINE ACTION FUNCTIONS                        */
/* ************************************************************************** */ 

static void act_idle()
{
  //read_sensor();
}

/* ------------------------------------------------------------------------- */ 

static void act_exposure_enter()
{ 
  display_exposure();
  vTaskDelay(SHORT_DELAY / portTICK_PERIOD_MS );
}

/* ------------------------------------------------------------------------- */ 

static void act_exposure_up()
{
  //extern sensor_info_t sensor_info;
  //extern Adafruit_AS726x ams;
  //int exposure = sensor_info.exposure + EXPOSURE_STEPS;

  //sensor_info.exposure = constrain(exposure, 1, 255);
  //ams.setIntegrationTime(sensor_info.exposure); 
  display_exposure();
  vTaskDelay(SHORT_DELAY / portTICK_PERIOD_MS );
}

/* ------------------------------------------------------------------------- */ 

static void act_exposure_down()
{
  //extern sensor_info_t sensor_info;
  //extern Adafruit_AS726x ams;
  //int exposure = sensor_info.exposure - EXPOSURE_STEPS;

  //sensor_info.exposure = constrain(exposure, 1, 255);
  //ams.setIntegrationTime(sensor_info.exposure); 
  display_exposure();
  vTaskDelay(SHORT_DELAY / portTICK_PERIOD_MS );
}


/* ------------------------------------------------------------------------- */ 

static void act_light_enter()
{ 
  display_backlight();
  vTaskDelay(SHORT_DELAY / portTICK_PERIOD_MS );
}

/* ------------------------------------------------------------------------- */ 

static void act_light_up()
{
  extern Adafruit_miniTFTWing ss;
  extern tft_info_t tft_info;
  int   backlight;

  backlight = tft_info.backlight + 10;
  tft_info.backlight = constrain(backlight, 10, 100);
  ss.setBacklight(65535-(tft_info.backlight*65535)/100); 
  display_backlight();
  vTaskDelay(SHORT_DELAY / portTICK_PERIOD_MS );
}

/* ------------------------------------------------------------------------- */ 

static void act_light_down()
{
  extern Adafruit_miniTFTWing ss;
  extern tft_info_t tft_info;
  int    backlight;

  backlight = tft_info.backlight - 10;
  tft_info.backlight = constrain(backlight, 10, 100);
  ss.setBacklight(65535-(tft_info.backlight*65535)/100); 
  display_backlight();
  vTaskDelay(SHORT_DELAY / portTICK_PERIOD_MS );
}

/* ------------------------------------------------------------------------- */ 

static void act_gain_enter()
{ 
  display_gain();
  vTaskDelay(SHORT_DELAY / portTICK_PERIOD_MS );
}

/* ------------------------------------------------------------------------- */ 

static void act_gain_up()
{
  //extern sensor_info_t sensor_info;
  //extern Adafruit_AS726x ams;

  //sensor_info.gain = (sensor_info.gain + 1) & 0b11;
  //ams.setGain(sensor_info.gain); 
  display_gain();
  vTaskDelay(SHORT_DELAY / portTICK_PERIOD_MS );
}

/* ------------------------------------------------------------------------- */ 

static void act_gain_down()
{
  //extern sensor_info_t sensor_info;
  //extern Adafruit_AS726x ams;

  //sensor_info.gain = (sensor_info.gain - 1) & 0b11;
  //ams.setGain(sensor_info.gain); 
  display_gain();
  vTaskDelay(SHORT_DELAY / portTICK_PERIOD_MS );
}

/* ------------------------------------------------------------------------- */ 

static void act_readings_enter()
{
  uint8_t freshData;
  freshData = read_sensor();
  if (freshData)
    display_bars();
  else
    display_bars();
    vTaskDelay(SHORT_DELAY / portTICK_PERIOD_MS );
}


/* ************************************************************************** */ 
/*                              TASKS FUNCTIONS                               */
/* ************************************************************************** */ 


void TaskGUI( void * pvParameters )  // This is a Task.
{
  (void) pvParameters;  // unusued
  uint8_t  screen = GUI_GAIN_SCREEN; // The current screen

  for (;;) // A Task shall never return or exit.
  {
    menu_action_t   action;
    uint8_t         event;
   
    event  = read_buttons();
    //Serial.print(F("State: "));  Serial.print(screen); Serial.print(F(" Event: ")); Serial.println(event);
    action = get_action(screen, event);
    screen = get_next_screen(screen, event);
    action();  // execute the action
  }
}

/* ----------------------------------------------------------------------- */

void TaskSensor( void * pvParameters )  // This is a Task.
{
  (void) pvParameters;  // unusued
  Serial.println(F("Hi there!"));

  for (;;) // A Task shall never return or exit.
  {
    ams.setConversionType(ONE_SHOT);
    xSemaphoreTake(xBinarySemaphoreKey, portMAX_DELAY);
    read_sensor();
  }
}

/* ----------------------------------------------------------------------- */

void MyInterruptServiceRoutine(void)
{
  BaseType_t xHigherPriorityTaskWoken;

  // toggles LED
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  xSemaphoreGiveFromISR(xBinarySemaphoreKey, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken) {
      taskYIELD();
  }
}

/* ************************************************************************** */ 
/*                              SETUP FUNCTIONS                               */
/* ************************************************************************** */ 


static void setup_sensor()
{
  extern sensor_info_t sensor_info;
  extern Adafruit_AS726x ams;
  
  // finds the 6 channel chip
  if(!ams.begin()){
    Serial.println(F("could not connect to sensor! Please check your wiring."));
    while(1);
  }
  // as initialized by the AS7262 library
  // Note that in MODE 2, the exposure time is actually doubled
  sensor_info.gain     = GAIN_64X;
  sensor_info.exposure = 50;
  // one shot conversions 
  ams.setConversionType(ONE_SHOT);
  ams.enableInterrupt();
  // MCU related configuration
  pinMode(2, INPUT_PULLUP); // Ext INT0
  pinMode(LED_BUILTIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(2), MyInterruptServiceRoutine, FALLING);
  Serial.println(F("AS7262 initialized"));
}


static void setup_tft()
{
  extern Adafruit_miniTFTWing ss;
  extern Adafruit_ST7735     tft;
  extern tft_info_t          tft_info;
  
  // acknowledges the Seesaw chip before sending commands to the TFT display
  if (!ss.begin()) {
    Serial.println(F("seesaw couldn't be found!"));
    while(1); // stops forever if cannto initialize
  }

  ss.tftReset();   // reset the display via a seesaw command
  ss.setBacklight(TFTWING_BACKLIGHT_ON/2);  // turn on the backlight
  tft_info.backlight = 50;
  //ss.setBacklightFreq(10);  // turn on the backlight

  tft.initR(INITR_MINI160x80);   // initialize a ST7735S chip, mini display
  tft.setRotation(3);            
  tft.fillScreen(ST7735_BLACK);
  
  Serial.println(F("TFT initialized"));
}

static void setup_tasks()
{
  //TaskHandle_t xTaskHandle;
  xBinarySemaphoreKey = xSemaphoreCreateBinary();

  xTaskCreate(
    TaskGUI,                  // Funtion to execute
    (const portCHAR *) "Blink", // A name just for humans
    192,                        // Stack size in words. Checked & adjusted by reading the Stack Highwater
    NULL,                       // pointer to parameters struct (pvParameters)
    1,                          // Priority. 3 = highest, 0 = lowest.
    NULL                        // pointer to task handle just created
  );

  xTaskCreate(
    TaskSensor,                  // Funtion to execute
    (const portCHAR *) "AS7262", // A name just for humans
    192,                        // Stack size in words. Checked & adjusted by reading the Stack Highwater
    NULL,                       // pointer to parameters struct (pvParameters)
    3,                          // Priority. 3 = highest, 0 = lowest.
    NULL                        // pointer to task handle just created
  );

  //vTaskSuspend(xTaskHandle);
}

/* ************************************************************************** */ 

void setup() 
{
  Serial.begin(9600);
  // wait for serial port to connect. 
  // Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
  while (!Serial) { ; }
  Serial.println(F("Sketch version: " GIT_VERSION));
  setup_tft();
  setup_sensor();
  setup_tasks();
}


/* ************************************************************************** */ 
/*                                MAIN SECTION                                */
/* ************************************************************************** */ 

/*
In the Arduino port of FreeRTOS, the IDLE HOOK is the loop() function
The IDLE HOOK is enable by setting configUSE_IDLE_HOOK to 1 within 
FreeRTOSConfig.h

The idle task runs at the very lowest priority, so such an idle hook function 
will only get executed when there are no tasks of higher priority that are 
able to run. This makes the idle hook function an ideal place to put 
the processor into a low power state - providing an automatic power saving 
whenever there is no processing to be performed.

The idle hook is called repeatedly as long as the idle task is running. 
It is paramount that the idle hook function does not call any API functions
that could cause it to block. Also, if the application makes use of the 
vTaskDelete() API function then the idle task hook must be allowed to 
periodically return (this is because the idle task is responsible for 
cleaning up the resources that were allocated by the RTOS kernel to the task 
that has been deleted)
*/

void loop()
{
}