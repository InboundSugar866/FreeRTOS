#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdbool.h>
#include <stdio.h>

#include "driver/gpio.h"
#include "freertos/semphr.h"
#include "ultrasonic.h"

#define ECHO_GPIO 32
#define TRIGGER_GPIO 33
#define MAX_DISTANCE_CM 500  // Maximum of 5 meters

#define GPIO_SW1 GPIO_NUM_16

SemaphoreHandle_t xBSemUltrason = NULL;
SemaphoreHandle_t xMutexDistance = NULL;
SemaphoreHandle_t xBSemValidation = NULL;

float distance;

void vTaskUltrason(void *pvParameters);
void vTaskPeriodic(void *pvParameters);
void vTaskOK(void *pvParameters);
static void interruptSW1();

void app_main() {
  gpio_install_isr_service(0);
  gpio_isr_handler_add(GPIO_SW1, interruptSW1, NULL);
  gpio_set_intr_type(GPIO_SW1, GPIO_INTR_NEGEDGE);

  xBSemUltrason = xSemaphoreCreateBinary();
  xMutexDistance = xSemaphoreCreateMutex();
  xBSemValidation = xSemaphoreCreateBinary();

  gpio_set_level(GPIO_SW1, 0);

  // attachInterruptFromISR(digitalPinToInterrupt(GPIO_SW1), interruptSW1,
  // FALLING);

  xTaskCreate(vTaskUltrason, "vTaskUltrason", 10000, NULL, 3, NULL);
  xTaskCreate(vTaskPeriodic, "vTaskPeriodic", 10000, NULL, 5, NULL);
  xTaskCreate(vTaskOK, "vTaskOK", 10000, NULL, 8, NULL);
}

void vTaskPeriodic(void *pvParameters) {
  const char *pcTaskName = "Task periodique";
  UBaseType_t uxPriority;
  uxPriority = uxTaskPriorityGet(NULL);

  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  float distanceLocal = 0;

  for (;;) {
    printf("[TASK %s - Priority %d] en attente...\n", pcTaskName, uxPriority);

    xSemaphoreTake(xMutexDistance, portMAX_DELAY);
    distanceLocal = distance;
    xSemaphoreGive(xMutexDistance);

    printf("[TASK %s - Priority %d] Distance : %0.04f m\n", pcTaskName, uxPriority, distanceLocal);

    if (distanceLocal < 0.06) {
      xSemaphoreGive(xBSemValidation);
    }

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(500));
  }
}

void vTaskUltrason(void *pvParameters) {
  const char *cTaskName = "Ultrason";
  UBaseType_t uxPriority;
  uxPriority = uxTaskPriorityGet(NULL);

  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  // float distance; global variable
  float distanceLocal;

  ultrasonic_sensor_t sensor = {
      .trigger_pin = TRIGGER_GPIO,
      .echo_pin = ECHO_GPIO};

  ultrasonic_init(&sensor);

  for (;;) {
    printf("[TASK %s - Priority %d] en attente...\n", cTaskName, uxPriority);

    xSemaphoreTakeFromISR(xBSemUltrason, portMAX_DELAY);

    // esp_err_t res =
    ultrasonic_measure(&sensor, MAX_DISTANCE_CM, &distanceLocal);

    xSemaphoreTake(xMutexDistance, portMAX_DELAY);
    distance = distanceLocal;
    xSemaphoreGive(xMutexDistance);

    printf("[TASK %s - Priority %d] Mesure d'ultrason réalisé\n", cTaskName, uxPriority);

    /*if (res == ESP_OK)
    {
      printf("Distance: %0.04f m\n", distanceLocal);
    } // Print error
    else
    {
      printf("Error %d: ", res);
      switch (res)
      {
      case ESP_ERR_ULTRASONIC_PING:
        printf("Cannot ping (device is in invalid state)\n");
        break;
      case ESP_ERR_ULTRASONIC_PING_TIMEOUT:
        printf("Ping timeout (no device found)\n");
        break;
      case ESP_ERR_ULTRASONIC_ECHO_TIMEOUT:
        printf("Echo timeout (i.e. distance too big)\n");
        break;
      default:
        printf("%s\n", esp_err_to_name(res));
      }
    }*/

    gpio_set_level(GPIO_SW1, 1);
    vTaskDelay(pdMS_TO_TICKS(125));
    gpio_set_level(GPIO_SW1, 0);

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(300));
  }
}

void vTaskOK(void *pvParameters) {
  const char *pcTaskName = "Task Validation";
  UBaseType_t uxPriority;
  uxPriority = uxTaskPriorityGet(NULL);

  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    printf("[TASK %s - Priority %d] en attente...\n", pcTaskName, uxPriority);

    xSemaphoreTake(xBSemValidation, portMAX_DELAY);

    printf("[TASK %s - Priority %d] SUCCESS !!!!\n", pcTaskName, uxPriority);

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
  }
}

void interruptSW1() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  printf("interruptSW1()\n");

  xSemaphoreGiveFromISR(xBSemUltrason, &xHigherPriorityTaskWoken);

  if (xHigherPriorityTaskWoken != pdFALSE) {
    portYIELD_FROM_ISR();
  }
}