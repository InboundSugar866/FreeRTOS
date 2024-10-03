#include <Arduino.h>
#include <Wire.h>               // Only needed for Arduino 1.6.5 and earlier
#include "SSD1306Wire.h"        // legacy: #include "SSD1306.h"

#define SDA 5
#define SCL 4
#define ADDRESSE 0x3c

// Initialize the OLED display using Arduino Wire:
SSD1306Wire display(ADDRESSE, SDA, SCL);   // ADDRESS, SDA, SCL  -  SDA and SCL usually populate automatically based on your board's pins_arduino.h e.g. https://github.com/esp8266/Arduino/blob/master/variants/nodemcu/pins_arduino.h

int compteur;
// put function declarations here:
void vTaskCompteur(void *pvParameters);


void setup() {
  // put your setup code here, to run once:
  display.init();
  compteur = 0;

  xTaskCreate(vTaskCompteur, "vTaskCompteur", 10000, NULL, 1, NULL);
}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:
void vTaskCompteur(void *pvParameters){
  
  const char *cTaskName = "Compteur";
  UBaseType_t uxPriority;
  uxPriority = uxTaskPriorityGet( NULL );

  TickType_t xLastWakeTimeConso;
  xLastWakeTimeConso = xTaskGetTickCount();

  for(;;){
    display.clear();


    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.drawString(64, 22, String(compteur));
    
    display.display();
    
    if( compteur + 1 < 60 ){
      compteur++;
    }else{
      compteur=0;
    }
    
    vTaskDelayUntil( &xLastWakeTimeConso, pdMS_TO_TICKS( 50 ) );
  }

}