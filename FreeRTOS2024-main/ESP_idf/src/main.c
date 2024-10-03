#include <stdio.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "hal/uart_types.h"
#include "sdkconfig.h"
#include "ultrasonic.h"

#define TXD_ECHANGE_PIN 27
#define RXD_ECHANGE_PIN 26
#define TXD_CO2_PIN 17
#define RXD_CO2_PIN 16

#define RX_BUF_SIZE 1024  // Taille de la RAM alloué pour l'échange de donnée via UART
const uart_config_t uart_config = {
    .baud_rate = 9600,                      // Configuration UART pour échange de donnée
    .data_bits = UART_DATA_8_BITS,          // Configuration UART pour échange de donnée
    .parity = UART_PARITY_DISABLE,          // Configuration UART pour échange de donnée
    .stop_bits = UART_STOP_BITS_1,          // Configuration UART pour échange de donnée
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,  // Configuration UART pour échange de donnée
    .source_clk = UART_SCLK_APB,            // Configuration UART pour échange de donnée
};

#define ECHO_GPIO 32         // PIN pour le capteur Ultra-Son
#define TRIGGER_GPIO 33      // PIN pour le capteur Ultra-Son
#define MAX_DISTANCE_CM 500  // Maximum of 5 meters

ultrasonic_sensor_t sensor = {    // Configuration du capteur à ultra son
    .trigger_pin = TRIGGER_GPIO,  // Configuration du capteur à ultra son
    .echo_pin = ECHO_GPIO};       // Configuration du capteur à ultra son

#define GPIO_SW_SIGNAL GPIO_NUM_18  // PIN pour envoyer une interruption de tâche

void vTaskCO2(void *pvParameters);
void vTaskUltraSon(void *pvParameters);
void vTaskSendData(void *pvParameters);
void interruptSW();

uint8_t uHigh = 0;    // Initialisation des diffèrentes variables
uint8_t uLow = 0;     // Initialisation des diffèrentes variables
float sDistance = 0;  // Initialisation des diffèrentes variables

SemaphoreHandle_t xSemDataCO2 = NULL;
SemaphoreHandle_t xSemDataUltraSon = NULL;
SemaphoreHandle_t xSemValidationEnvoie = NULL;

void app_main() {
  // We won't use a buffer for sending data.
  uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);                             // Inititialisation de l'UART pour échanger avec le capteur de CO2
  uart_param_config(UART_NUM_1, &uart_config);                                                 // Inititialisation de l'UART pour échanger avec le capteur de CO2
  uart_set_pin(UART_NUM_1, TXD_CO2_PIN, RXD_CO2_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);  // Inititialisation de l'UART pour échanger avec le capteur de CO2

  uart_driver_install(UART_NUM_2, RX_BUF_SIZE * 2, 0, 0, NULL, 0);                                     // Inititialisation de l'UART pour échanger avec l'autre carte
  uart_param_config(UART_NUM_2, &uart_config);                                                         // Inititialisation de l'UART pour échanger avec l'autre carte
  uart_set_pin(UART_NUM_2, TXD_ECHANGE_PIN, RXD_ECHANGE_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);  // Inititialisation de l'UART pour échanger avec l'autre carte

  ultrasonic_init(&sensor);
  xSemDataUltraSon = xSemaphoreCreateMutex();

  xSemDataCO2 = xSemaphoreCreateMutex();

  gpio_install_isr_service(0);                                                // Initialisation de l'interruption de tâche
  gpio_isr_handler_add(GPIO_SW_SIGNAL, interruptSW, (void *)GPIO_SW_SIGNAL);  // Initialisation de l'interruption de tâche
  gpio_set_intr_type(GPIO_SW_SIGNAL, GPIO_INTR_NEGEDGE);                      // Initialisation de l'interruption de tâche

  xSemValidationEnvoie = xSemaphoreCreateBinary();

  xTaskCreate(vTaskCO2, "vTaskCO2", 10000, NULL, 5, NULL);
  xTaskCreate(vTaskSendData, "vTaskSendData", 10000, NULL, 3, NULL);
  xTaskCreate(vTaskUltraSon, "vTaskUltraSon", 10000, NULL, 5, NULL);
}

void vTaskCO2(void *pvParameters) {
  // const char *cTaskName = "Capteur_CO2";
  // UBaseType_t uxPriority;
  // uxPriority = uxTaskPriorityGet(NULL);

  TickType_t xLastWakeTimeConso;
  xLastWakeTimeConso = xTaskGetTickCount();

  uint8_t *uDataTx = (uint8_t *)malloc(RX_BUF_SIZE + 1);  // Donnée que nous allons envoyé par le bus
  uint8_t *uDataRx = (uint8_t *)malloc(RX_BUF_SIZE + 1);  // Donnée que nous allons envoyé par le bus

  uDataTx[0] = 0xFF;  // Initialisation du capteur pour que la range detection se fasse entre 0 et 10 000
  uDataTx[1] = 0x01;  // Initialisation du capteur pour que la range detection se fasse entre 0 et 10 000
  uDataTx[2] = 0x99;  // Initialisation du capteur pour que la range detection se fasse entre 0 et 10 000
  uDataTx[3] = 0x00;  // Initialisation du capteur pour que la range detection se fasse entre 0 et 10 000
  uDataTx[4] = 0x00;  // Initialisation du capteur pour que la range detection se fasse entre 0 et 10 000
  uDataTx[5] = 0x00;  // Initialisation du capteur pour que la range detection se fasse entre 0 et 10 000
  uDataTx[6] = 0x27;  // Initialisation du capteur pour que la range detection se fasse entre 0 et 10 000
  uDataTx[7] = 0x10;  // Initialisation du capteur pour que la range detection se fasse entre 0 et 10 000
  uDataTx[8] = 0x2f;  // Initialisation du capteur pour que la range detection se fasse entre 0 et 10 000

  int txBytes = uart_write_bytes(UART_NUM_1, uDataTx, 9);                                      // Envoie de donnée : initialisation du capteur
  int rxBytes = uart_read_bytes(UART_NUM_1, uDataRx, RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);  // Reception

  for (;;) {
    uDataTx[0] = 0xFF;  // Commande envoyé au capteur
    uDataTx[1] = 0x01;  // Commande envoyé au capteur
    uDataTx[2] = 0x86;  // Commande envoyé au capteur
    uDataTx[3] = 0x00;  // Commande envoyé au capteur
    uDataTx[4] = 0x00;  // Commande envoyé au capteur
    uDataTx[5] = 0x00;  // Commande envoyé au capteur
    uDataTx[6] = 0x00;  // Commande envoyé au capteur
    uDataTx[7] = 0x00;  // Commande envoyé au capteur
    uDataTx[8] = 0x79;  // Commande envoyé au capteur

    txBytes = uart_write_bytes(UART_NUM_1, uDataTx, 9);                                      // Envoie de la commande
    rxBytes = uart_read_bytes(UART_NUM_1, uDataRx, RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);  // Récupération du résultat obtenu

    xSemaphoreTake(xSemDataCO2, portMAX_DELAY);
    uHigh = uDataRx[2];
    uLow = uDataRx[3];
    xSemaphoreGive(xSemDataCO2);

    vTaskDelayUntil(&xLastWakeTimeConso, pdMS_TO_TICKS(5000));
  }
}

void vTaskUltraSon(void *pvParameters) {
  // const char *cTaskName = "Capteur_CO2";
  // UBaseType_t uxPriority;
  // uxPriority = uxTaskPriorityGet(NULL);

  TickType_t xLastWakeTimeConso;
  xLastWakeTimeConso = xTaskGetTickCount();

  float sLocalDistance;

  for (;;) {
    ultrasonic_measure(&sensor, MAX_DISTANCE_CM, &sLocalDistance);  // Acquisition de la distance
    printf("%f\n", sLocalDistance);

    if (sLocalDistance < 0.5) {  // Vérification si une personne se trouve à 50 centimètre du capteur

      xSemaphoreGive(xSemValidationEnvoie);

      /*
            gpio_set_level(GPIO_SW_SIGNAL, 1);  // Envoie du signal montant-descendant pour déclencher l'interruption de tâche assigné
            printf("allumée\n");                // Debug
            vTaskDelay(pdMS_TO_TICKS(125));     // Envoie du signal montant-descendant pour déclencher l'interruption de tâche assigné
            gpio_set_level(GPIO_SW_SIGNAL, 0);  // Envoie du signal montant-descendant pour déclencher l'interruption de tâche assigné
      */
    } else {
      printf("pas allumée\n");
    }

    vTaskDelayUntil(&xLastWakeTimeConso, pdMS_TO_TICKS(300));
  }
}

void vTaskSendData(void *pvParameters) {
  // const char *cTaskName = "Envoie_de_donnée";
  // UBaseType_t uxPriority;
  // uxPriority = uxTaskPriorityGet(NULL);

  TickType_t xLastWakeTimeConso;
  xLastWakeTimeConso = xTaskGetTickCount();

  uint8_t *uDataSend = (uint8_t *)malloc(RX_BUF_SIZE + 1);

  uint8_t uLocalHigh;
  uint8_t uLocalLow;

  int txBytes;

  for (;;) {
    xSemaphoreTake(xSemValidationEnvoie, portMAX_DELAY);

    xSemaphoreTake(xSemDataCO2, portMAX_DELAY);
    uLocalHigh = uHigh;
    uLocalLow = uLow;
    xSemaphoreGive(xSemDataCO2);

    uDataSend[0] = uLocalHigh;
    uDataSend[1] = uLocalLow;

    txBytes = uart_write_bytes(UART_NUM_2, uDataSend, 2);  // Envoie des données par UART

    printf("Envoie des données\n");

    /*
        printf("txBytes: %d\n", txBytes);  // DEBUG
        for (int i = 0; i < 2; i++) {      // DEBUG
          printf("%d ", uDataSend[i]);     // DEBUG
        }  // DEBUG
        printf("\n\n");  // DEBUG
        */

    vTaskDelayUntil(&xLastWakeTimeConso, pdMS_TO_TICKS(250));
  }
}

void interruptSW() {
  /*
  Fonction qui transmet envoie une interruption de tâche aux diffèrents contrôleur
  */

  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  // xSemaphoreGiveFromISR(xSemValidationEnvoie, &xHigherPriorityTaskWoken);

  if (xHigherPriorityTaskWoken != pdFALSE) {
    portYIELD_FROM_ISR();
  }
}