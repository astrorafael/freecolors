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
              EXT0              <----- INT
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
#include <timers.h>
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


/* ************************************************************************** */ 
/*                      GUI STATE MACHINE DECLARATIONS                        */
/* ************************************************************************** */ 

// --------------------------------------------
// State Machine Actions (Forward declarations)
// --------------------------------------------



/* ************************************************************************** */ 
/*                              GUI TASK                                      */
/* ************************************************************************** */ 

/* ************************************************************************** */ 
/*                         GUI TASK HELPER FUNCTIONS                          */
/* ************************************************************************** */ 


/* ************************************************************************** */ 
/*                      STATE MACHINE ACTION FUNCTIONS                        */
/* ************************************************************************** */ 


/* -------------------------------------------------------------------------- */
/*                     SENSOR TASK HELPER FUNCTIONS                           */
/* -------------------------------------------------------------------------- */


/* ************************************************************************** */ 
/*                                TIMER TASK CALLBACK                         */
/* ************************************************************************** */ 


static void callbackBlinkLED(TimerHandle_t xTimer)
{
  (void) xTimer;
  digitalWrite(LED_BUILTIN, ! digitalRead(LED_BUILTIN));
}

/* ************************************************************************** */ 
/*                              SETUP FUNCTIONS                               */
/* ************************************************************************** */ 

void setup_tasks()
{
  TimerHandle_t     xTimer;

  xTimer = xTimerCreate(
    "LED", 
    2000 / portTICK_PERIOD_MS, 
    pdTRUE, 
    0, 
    callbackBlinkLED
  );
  xTimerStart(xTimer,0);
}


void setup() 
{
  Serial.begin(9600);
  // wait for serial port to connect. 
  // Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
  while (!Serial);
  Serial.println(F("Sketch version: " ));
  setup_tasks();
  pinMode(LED_BUILTIN, OUTPUT);
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
