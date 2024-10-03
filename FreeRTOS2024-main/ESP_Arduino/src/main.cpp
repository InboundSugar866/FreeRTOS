#include <Arduino.h>
#include <Wire.h>  // Only needed for Arduino 1.6.5 and earlier

#include "DHTesp.h"       // Click here to get the library: http://librarymanager/All#DHTesp
#include "SSD1306Wire.h"  // legacy: #include "SSD1306.h"
#include "driver/uart.h"
#include "freertos/semphr.h"
#include "hal/uart_types.h"

#define dhtPin 15  // Pin de transmission pour le capteur de température et d'humidité

#define SDA 5
#define SCL 4
#define ADDRESSE 0x3c

#include <WebServer.h>
#include <WiFiClient.h>

#include "stdio.h"
#include "string.h"

// Initialize the OLED display using Arduino Wire:
SSD1306Wire display(ADDRESSE, SDA, SCL);  // ADDRESS, SDA, SCL  -  SDA and SCL usually populate automatically based on your board's pins_arduino.h e.g. https://github.com/esp8266/Arduino/blob/master/variants/nodemcu/pins_arduino.h

DHTesp dht;

float sTemp = 0.0;
float sHum = 0.0;
SemaphoreHandle_t xTempHum = NULL;

uint8_t uCo2High = 0;
uint8_t uCo2Low = 0;
SemaphoreHandle_t xCo2 = NULL;

#define TXD_ECHANGE_PIN 27
#define RXD_ECHANGE_PIN 26
#define RX_BUF_SIZE 128

const uart_config_t uart_config = {
    .baud_rate = 9600,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_APB,
};

#define GPIO_SW1 26
SemaphoreHandle_t xSemRec = NULL;

void vTaskAffichage(void *pvParameters);
void vTaskTempHum(void *pvParameters);
void vTaskGetDataCO2(void *pvParameters);

void vTaskWebLoop(void *pvParameters);
// Initialize Web Server
WebServer server(80);
// modifiez ces deux constantes pour qu'elles contiennent les caractéristiques de votre réseau Wifi
#define ssid "Galaxy A14 8128"  // le nom (SSID) de votre réseau WiFi
#define password "patatra006C"  // votre mot de passe WiFi
void gestionPage();
String construitPage(String temp, String hum, String co2);
String sTemperature, sHumidity, sCo2;

// void vTaskProd(void *pvParameters);
void vTaskConso(void *pvParameters);

#define uN 10
typedef struct {
  int type;
  String value;
} donnee;

donnee buffer[uN];
int uIProd;
int uIConso;
SemaphoreHandle_t xSemProd = NULL;
SemaphoreHandle_t xSemConso = NULL;
SemaphoreHandle_t xMutProd = NULL;

void vTaskValidation(void *pvParameters);
void IRAM_ATTR vInterrupt();

// temporalité
SemaphoreHandle_t xSemAffich = NULL;
SemaphoreHandle_t xSemTime = NULL;
void vTaskTimeUpdate(void *pvParameters);

void IRAM_ATTR vInterrupt() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  // Serial.println("interruptSW1()");
  xSemaphoreGiveFromISR(xSemRec, &xHigherPriorityTaskWoken);
  xSemaphoreGiveFromISR(xSemAffich, &xHigherPriorityTaskWoken);
  xSemaphoreGiveFromISR(xSemTime, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken != pdFALSE)
    portYIELD_FROM_ISR();
}

SemaphoreHandle_t xSemTimeVar = NULL;
unsigned long lActivation;

void vTaskTimeUpdate(void *pvParameters) {
  TickType_t xLastWakeTimeConso;
  xLastWakeTimeConso = xTaskGetTickCount();
  for (;;) {
    xSemaphoreTake(xSemTime, portMAX_DELAY);
    xSemaphoreTake(xSemTimeVar, portMAX_DELAY);
    lActivation = millis();
    // Serial.println("changement de lActivation : " + String(lActivation));
    xSemaphoreGive(xSemTimeVar);
    vTaskDelayUntil(&xLastWakeTimeConso, pdMS_TO_TICKS(100));
  }
}

void vTaskValidation(void *pvParameters) {
  TickType_t xLastWakeTimeConso;
  xLastWakeTimeConso = xTaskGetTickCount();

  for (;;) {
    xSemaphoreTake(xSemRec, portMAX_DELAY);
    Serial.println("Reception de donnée!!!!!");

    vTaskDelayUntil(&xLastWakeTimeConso, pdMS_TO_TICKS(250));
  }
}

/*
void vTaskProd(void *pvParameters) {
  const char *cTaskName = "Producteur";

  float sLocalTemp;
  float sLocalHum;

  uint8_t uLocalCo2High;
  uint8_t uLocalCo2Low;
  uint16_t uLocalCo2Total;

  TickType_t xLastWakeTimeProd;
  xLastWakeTimeProd = xTaskGetTickCount();

  for (;;) {
    xSemaphoreTake(xSemProd, portMAX_DELAY);

    xSemaphoreTake(xTempHum, portMAX_DELAY);
    sLocalTemp = sTemp;
    sLocalHum = sHum;
    xSemaphoreGive(xTempHum);

    xSemaphoreTake(xCo2, portMAX_DELAY);
    uLocalCo2High = uCo2High;
    uLocalCo2Low = uCo2Low;
    xSemaphoreGive(xCo2);

    uLocalCo2Total = uLocalCo2High * 256 + uLocalCo2Low;

    buffer[uIProd].uCo2 = uLocalCo2Total;
    buffer[uIProd].sTemp = sLocalTemp;
    buffer[uIProd].sHum = sLocalHum;

    Serial.println(String(cTaskName) + " - index " + String(uIProd) +
                   " (" + String(buffer[uIProd].uCo2) + " " + String(buffer[uIProd].sHum) + " " + String(buffer[uIProd].sTemp));
    uIProd = (uIProd + 1) % uN;

    xSemaphoreGive(xSemConso);

    vTaskDelayUntil(&xLastWakeTimeProd, pdMS_TO_TICKS(125));
  }
}

void vTaskConso(void *pvParameters) {
  const char *cTaskName = "Consommateur";

  TickType_t xLastWakeTimeProd;
  xLastWakeTimeProd = xTaskGetTickCount();

  for (;;) {
    xSemaphoreTake(xConso, portMAX_DELAY);

    lastData = buffer[uIConso];

    // Serial.println(String(cTaskName) + " - index " + String(uIProd) +
    //                " (" + String(buffer[uIProd].uCo2) + " " + String(buffer[uIProd].sHum) + " " + String(buffer[uIProd].sTemp));
    uIConso = (uIConso + 1) % uN;

    xSemaphoreGive(xSemProd);

    vTaskDelayUntil(&xLastWakeTimeProd, pdMS_TO_TICKS(125));
  }
}
*/

void setup() {
  // Arduino init
  Serial.begin(115200);
  Serial.println();
  Serial.println("DHT ESP32 example with tasks");

  dht.setup(dhtPin, DHTesp::DHT22);
  Serial.println("DHT initiated");

  display.init();
  Serial.println("Display initiated");

  xTempHum = xSemaphoreCreateMutex();
  xCo2 = xSemaphoreCreateMutex();

  uart_driver_install(UART_NUM_2, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
  uart_param_config(UART_NUM_2, &uart_config);
  uart_set_pin(UART_NUM_2, TXD_ECHANGE_PIN, RXD_ECHANGE_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

  // Interruption de tâche

  attachInterrupt(digitalPinToInterrupt(GPIO_SW1), vInterrupt, RISING);
  xSemRec = xSemaphoreCreateBinary();

  xSemAffich = xSemaphoreCreateBinary();
  xSemTime = xSemaphoreCreateBinary();

  xSemTimeVar = xSemaphoreCreateMutex();
  xTaskCreate(vTaskTimeUpdate, "vTaskTimeUpdate", 10000, NULL, 1, NULL);

  // Modele Prod-Conso
  uIProd = 0;
  uIConso = 0;
  xSemProd = xSemaphoreCreateCounting(uN, uN);
  xMutProd = xSemaphoreCreateMutex();
  xSemConso = xSemaphoreCreateCounting(uN, 0);

  /*
      Initialisation du serveur WEB
  */
  // Initialisation des E/S
  gpio_config_t io_conf;
  // disable interrupt
  io_conf.intr_type = GPIO_INTR_DISABLE;
  // set as output mode
  io_conf.mode = GPIO_MODE_OUTPUT;
  // bit mask of the pins that you want to set,e.g.GPIO18
  io_conf.pin_bit_mask = GPIO_SEL_14;
  // disable pull-down mode
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  // disable pull-up mode
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  // configure GPIO with the given settings
  gpio_config(&io_conf);

  IPAddress ip;

  // initialisation de la communication WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    printf(".");
  }

  printf("\nMaintenant connecte a %s\n", ssid);
  printf("Adresse IP: ");
  ip = WiFi.localIP();
  printf("%u.%u.%u.%u\n", ip & 0x000000FF, (ip & 0x0000FF00) / 256, (ip & 0x00FF0000) / (256 * 256), (ip & 0xFF000000) / (256 * 256 * 256));

  // On indique le nom de la fonction qui gère l'interraction avec la page web
  server.on("/", gestionPage);
  server.begin();
  printf("Serveur HTTP en fonction\n");

  xTaskCreate(vTaskGetDataCO2, "vTaskGetDataCO2", 10000, NULL, 7, NULL);
  xTaskCreate(vTaskAffichage, "vTaskAffichage", 10000, NULL, 5, NULL);
  xTaskCreate(vTaskTempHum, "vTaskTempHum", 10000, NULL, 3, NULL);
  xTaskCreate(vTaskWebLoop, "vTaskWebLoop", 10000, NULL, 10, NULL);

  // xTaskCreate(vTaskValidation, "vTaskValidation", 10000, NULL, 1, NULL);
}

void loop() {
}

/*void vTaskConso(void *pvParameters) {
  TickType_t xLastWakeTimeConso;
  xLastWakeTimeConso = xTaskGetTickCount();

  donnee dataConso;

  for (;;) {
    xSemaphoreTake(xSemConso, portMAX_DELAY);
    dataConso = buffer[uIConso];
    uIConso = (uIConso + 1) % uN;
    xSemaphoreGive(xSemProd);

    Serial.println("CONSO - " + String(dataConso.type) + " : " + dataConso.value);
    vTaskDelayUntil(&xLastWakeTimeConso, pdMS_TO_TICKS(100));
  }
}*/

void vTaskTempHum(void *pvParameters) {
  const char *cTaskName = "Affichage";
  UBaseType_t uxPriority;
  uxPriority = uxTaskPriorityGet(NULL);

  TickType_t xLastWakeTimeConso;
  xLastWakeTimeConso = xTaskGetTickCount();

  for (;;) {
    TempAndHumidity newValues = dht.getTempAndHumidity();
    // Check if any reads failed and exit early (to try again).
    if (dht.getStatus() != 0) {
      Serial.println("DHT11 error status: " + String(dht.getStatusString()));
    }

    xSemaphoreTake(xTempHum, portMAX_DELAY);
    sTemp = newValues.temperature;
    sHum = newValues.humidity;
    xSemaphoreGive(xTempHum);

    xSemaphoreTake(xSemProd, portMAX_DELAY);  // Ajout de la température dans le buffer
    xSemaphoreTake(xMutProd, portMAX_DELAY);  // Ajout de la température dans le buffer
    buffer[uIProd].type = 0;                  // Ajout de la température dans le buffer
    buffer[uIProd].value = String(sTemp);     // Ajout de la température dans le buffer
    xSemaphoreGive(xSemConso);                // Ajout de la température dans le buffer
    Serial.println("PROD - " + String(buffer[uIProd].type) + " : " + buffer[uIProd].value);
    uIProd = (uIProd + 1) % uN;  // Ajout de la température dans le buffer
    xSemaphoreGive(xMutProd);    // Ajout de la température dans le buffer

    xSemaphoreTake(xSemProd, portMAX_DELAY);  // Ajout de l'humidité dans le buffer
    xSemaphoreTake(xMutProd, portMAX_DELAY);  // Ajout de l'humidité dans le buffer
    buffer[uIProd].type = 1;                  // Ajout de l'humidité dans le buffer
    buffer[uIProd].value = String(sHum);      // Ajout de l'humidité dans le buffer
    xSemaphoreGive(xSemConso);                // Ajout de l'humidité dans le buffer
    Serial.println("PROD - " + String(buffer[uIProd].type) + " : " + buffer[uIProd].value);
    uIProd = (uIProd + 1) % uN;  // Ajout de l'humidité dans le buffer
    xSemaphoreGive(xMutProd);    // Ajout de l'humidité dans le buffer

    // Serial.println("New values - T:" + String(newValues.temperature) + " H:" + String(newValues.humidity));

    vTaskDelayUntil(&xLastWakeTimeConso, pdMS_TO_TICKS(10000));
  }
}

void vTaskGetDataCO2(void *pvParameters) {
  TickType_t xLastWakeTimeConso;
  xLastWakeTimeConso = xTaskGetTickCount();

  uint8_t *uDataReceive = (uint8_t *)malloc(RX_BUF_SIZE + 1);

  uint16_t uCo2Total;
  int rxBytes;
  for (;;) {
    rxBytes = uart_read_bytes(UART_NUM_2, uDataReceive, RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);  // Pour une raison qui nous échappe, l'acquisition des données est très longue

    xSemaphoreTake(xSemRec, portMAX_DELAY);

    xSemaphoreTake(xCo2, portMAX_DELAY);
    uCo2High = uDataReceive[0];
    uCo2Low = uDataReceive[1];
    xSemaphoreGive(xCo2);

    uCo2Total = uCo2High * 256 + uCo2Low;

    xSemaphoreTake(xSemProd, portMAX_DELAY);
    xSemaphoreTake(xMutProd, portMAX_DELAY);
    buffer[uIProd].type = 2;
    buffer[uIProd].value = String(uCo2Total);
    xSemaphoreGive(xSemConso);
    Serial.println("PROD - " + String(buffer[uIProd].type) + " : " + buffer[uIProd].value);
    uIProd = (uIProd + 1) % uN;
    xSemaphoreGive(xMutProd);

    // printf("rxBytes: %d\n", rxBytes);
    // printf("%d %d\n\n", uDataReceive[0], uDataReceive[1]);
    vTaskDelayUntil(&xLastWakeTimeConso, pdMS_TO_TICKS(3000));
  }
}

void vTaskAffichage(void *pvParameters) {
  const char *cTaskName = "Affichage";
  UBaseType_t uxPriority;
  uxPriority = uxTaskPriorityGet(NULL);

  TickType_t xLastWakeTimeConso;
  xLastWakeTimeConso = xTaskGetTickCount();

  // float sLocalTemp;
  // float sLocalHum;

  /*
    uint8_t uLocalCo2High;
    uint8_t uLocalCo2Low;
    uint16_t uCo2Total;
  */

  String cLocalTemp = "attente";
  String cLocalHum = "attente";
  String cLocalCo2 = "attente";

  donnee dataConso;

  // uint8_t *data = (uint8_t *)malloc(RX_BUF_SIZE + 1);

  int rxBytes;

  for (;;) {
    Serial.println("Affichage - boucle");
    display.clear();

    xSemaphoreTake(xSemAffich, portMAX_DELAY);

    /*
    xSemaphoreTake(xTempHum, portMAX_DELAY);
    sLocalTemp = sTemp;
    sLocalHum = sHum;
    xSemaphoreGive(xTempHum);
    */

    /*
        xSemaphoreTake(xCo2, portMAX_DELAY);
        uLocalCo2High = uCo2High;
        uLocalCo2Low = uCo2Low;
        xSemaphoreGive(xCo2);


    uCo2Total = uLocalCo2High * 256 + uLocalCo2Low;
    */
    /*
        xSemaphoreTake(xSemConso, portMAX_DELAY);
        dataConso = buffer[uIConso];
        uIConso = (uIConso + 1) % uN;
        xSemaphoreGive(xSemProd);

        Serial.println("CONSO - " + String(dataConso.type) + " : " + dataConso.value);

        switch (dataConso.type) {
          case 0:
            cLocalTemp = dataConso.value;
            break;

          case 1:
            cLocalHum = dataConso.value;
            break;

          case 2:
            cLocalCo2 = dataConso.value;
            break;

          default:
            break;
        }
    */
    // Serial.println("c : - T:" + String(sLocalTemp) + " H:" + String(sLocalHum));
    // Serial.println("c : - Co2:" + String(uCo2Total));

    // display.setTextAlignment(TEXT_ALIGN_CENTER);
    // display.drawString(64, 22, String("T : " + cLocalTemp + " H : " + cLocalHum));
    //     display.drawString(64, 22, String("T : " + String(sLocalTemp) + " H : " + String(sLocalHum)));

    /*rxBytes = uart_read_bytes(UART_NUM_2, data, RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);

    printf("rxBytes: %d\n", rxBytes);
    printf("%d %d\n\n", data[0], data[1]);

    display.drawString(64, 22, String(data[0]) + " " + String(data[1]));
    display.drawString(64, 32, "rxBytes " + String(rxBytes));*/
    // display.drawString(64, 32, String("Co2 : " + cLocalCo2));
    //  display.drawString(64, 32, String("Co2 : " + String(uCo2Total)));

    // display.drawString(64, 10, " hello world ");

    xSemaphoreTake(xSemTimeVar, portMAX_DELAY);
    Serial.println("Timer:" + String(millis() - lActivation) + "b:" + String(millis() - lActivation < 15000));

    if (millis() - lActivation < 15000) {
      xSemaphoreGive(xSemAffich);

      xSemaphoreTake(xSemConso, portMAX_DELAY);
      dataConso = buffer[uIConso];
      uIConso = (uIConso + 1) % uN;
      xSemaphoreGive(xSemProd);

      Serial.println("CONSO - " + String(dataConso.type) + " : " + dataConso.value);

      switch (dataConso.type) {
        case 0:
          cLocalTemp = dataConso.value;
          sTemperature = String(cLocalTemp);
          break;

        case 1:
          cLocalHum = dataConso.value;
          sHumidity = String(cLocalHum);
          break;

        case 2:
          cLocalCo2 = dataConso.value;
          sCo2 = String(cLocalCo2);
          break;

        default:
          break;
      }

      display.setTextAlignment(TEXT_ALIGN_CENTER);
      display.drawString(64, 22, String("T : " + cLocalTemp + " H : " + cLocalHum));

      display.drawString(64, 32, String("Co2 : " + cLocalCo2));

      display.drawString(64, 10, " hello world ");

      printf("1\n");
      printf("actual %lu\n", millis());
      printf("event %lu\n", lActivation);
    } else {
      printf("2\n");
      sTemperature = String("NULL");
      sHumidity = String("NULL");
      sCo2 = String("NULL");
    }
    display.display();
    xSemaphoreGive(xSemTimeVar);

    vTaskDelayUntil(&xLastWakeTimeConso, pdMS_TO_TICKS(125));
  }
}

String construitPage(String temp, String hum, String co2) {
  String page = "<!DOCTYPE html>";
  page += "<html lang='fr'>";
  page += "<head>";
  page += "    <meta charset='UTF-8'>";
  page += "<meta http-equiv='refresh' content='3'>";
  page += "    <meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  page += "    <title>ESP 32 - météo</title>";
  page += "    <style>";
  page += "        body {";
  page += "            font-family: Arial, sans-serif;";
  page += "            background-color: #f5f5f5;";
  page += "            margin: 0;";
  page += "            padding: 0;";
  page += "        }";
  page += "        .weather-app {";
  page += "            max-width: 400px;";
  page += "            margin: 0 auto;";
  page += "            padding: 20px;";
  page += "            background-color: #fff;";
  page += "            border-radius: 8px;";
  page += "            box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);";
  page += "        }";
  page += "        h1 {";
  page += "            font-size: 24px;";
  page += "            margin-bottom: 16px;";
  page += "        }";
  page += "        .weather-data {";
  page += "            font-size: 16px;";
  page += "        }";
  page += "        .weather-data p {";
  page += "            margin: 8px 0;";
  page += "        }";
  page += "    </style>";
  page += "</head>";
  page += "<body>";
  page += "    <div class='weather-app'>";
  page += "        <h1>Information météo</h1>";
  page += "        <div class='weather-data'>";
  page += "            <p>Température: <span id='temperature'>" + temp + " °C</span></p>";
  page += "            <p>Humidité: <span id='humidity'>" + hum + " %</span></p>";
  page += "            <p>CO2: <span id='co2'>" + co2 + " ppm</span></p>";
  page += "        </div>";
  page += "    </div>";
  page += "</body>";
  page += "</html>";
  return page;
}

/*  La fonction gestionPage modifie les caractéristiques */

void gestionPage() {
  server.send(200, "text/html", construitPage(sTemperature, sHumidity, sCo2));
}

void vTaskWebLoop(void *pvParameters) {
  TickType_t xLastWakeTimeConso;
  xLastWakeTimeConso = xTaskGetTickCount();
  for (;;) {
    if (WiFi.status() == WL_CONNECTED)
      server.handleClient();
    else
      printf("Serveur déconnecté...\n");
  }
  vTaskDelayUntil(&xLastWakeTimeConso, pdMS_TO_TICKS(5000));
}