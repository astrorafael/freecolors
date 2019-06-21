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

// Support for the Git version tags
#include "git-version.h"


/* ************************************************************************** */ 
/*                                DEFINEs SECTION                             */
/* ************************************************************************** */ 



/* ************************************************************************** */ 
/*                        CUSTOM CLASES & DATA TYPES                          */
/* ************************************************************************** */ 



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
