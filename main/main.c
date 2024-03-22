#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "ssd1306.h"
#include "gfx.h"

// Definições dos pinos
const int TRIG_PIN = 12;
const int ECHO_PIN = 13;

// Definições para o display OLED
const uint LED_OLED = 20;
const uint BTN_OLED = 28;

// Variáveis para cálculo do tempo e distância
float sound_speed = 0.0343;

// Recursos do FreeRTOS
QueueHandle_t xQueueTime;
SemaphoreHandle_t xSemaphoreTrigger;
QueueHandle_t xQueueDistance;

TaskHandle_t oledTaskHandle = NULL;

// Tempo máximo de espera pelo eco em microssegundos
const int ECHO_TIMEOUT = 300000; // 30ms

// Função callback para o pino do echo
void gpio_callback(uint gpio, uint32_t events) {
    static int time_init;
    if (events == GPIO_IRQ_EDGE_RISE) {
        time_init = to_us_since_boot(get_absolute_time());
    } else if (events == GPIO_IRQ_EDGE_FALL) {
        int time_end = to_us_since_boot(get_absolute_time());
        int time_diff = time_end - time_init;
        xQueueSendFromISR(xQueueTime, &time_diff, NULL);
        xSemaphoreGiveFromISR(xSemaphoreTrigger, NULL);
    }
}

// Task responsável por gerar o trigger
void trigger_task(void *pvParameters) {
    while (1) {
        gpio_put(TRIG_PIN, 1);
        sleep_us(10);
        gpio_put(TRIG_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Task que faz a leitura do tempo que o pino echo ficou levantado
void echo_task(void *pvParameters) {
    int time_diff;
    int timeout = 0;
    float distance;
    while (1) {
        if (xQueueReceive(xQueueTime, &time_diff, 0) == pdPASS) {
            distance = (time_diff * sound_speed) / 2;
            xQueueSend(xQueueDistance, &distance, 0);
            timeout = 0;
        } else {
            timeout++;
            if (timeout > ECHO_TIMEOUT) {
                distance = -1;
                xQueueSend(xQueueDistance, &distance, 0);
                xSemaphoreGive(xSemaphoreTrigger); // Libera o semáforo quando ocorre uma falha no sensor
                vTaskPrioritySet(oledTaskHandle, 2); // Aumenta a prioridade da tarefa oled_task
            }
        }
    }
}

// Task que exibe a informação da distância no display
void oled_task(void *pvParameters) {
    ssd1306_init();
    ssd1306_t disp;
    gfx_init(&disp, 128, 32);

    float distance;
    char buffer[20];
    while (1) {
        if (xSemaphoreTake(xSemaphoreTrigger, portMAX_DELAY) == pdTRUE) {
            if (xQueueReceive(xQueueDistance, &distance, 0) == pdPASS) {
                gfx_clear_buffer(&disp);
                if (distance > 0) {
                    snprintf(buffer, sizeof(buffer), "Distancia: %.2f cm", distance);
                    gfx_draw_string(&disp, 0, 0, 1, buffer);
                    int bar_length = (int)(distance / 2); // Ajuste conforme necessário
                    gfx_draw_line(&disp, 0, 20, bar_length, 20);
                } else {
                    gfx_draw_string(&disp, 0, 0, 1, "Falha no sensor");
                }
                gfx_show(&disp);
            }
        }
    }
}

int main() {
    stdio_init_all();
    gpio_init(ECHO_PIN);
    gpio_set_dir(ECHO_PIN, GPIO_IN);
    gpio_set_irq_enabled_with_callback(ECHO_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    gpio_init(TRIG_PIN);
    gpio_set_dir(TRIG_PIN, GPIO_OUT);

    // Inicialização dos recursos do FreeRTOS
    xQueueTime = xQueueCreate(10, sizeof(int));
    xSemaphoreTrigger = xSemaphoreCreateBinary();
    xQueueDistance = xQueueCreate(10, sizeof(float));

    // Criação das tasks
    xTaskCreate(trigger_task, "Trigger Task", 256, NULL, 1, NULL);
    xTaskCreate(echo_task, "Echo Task", 256, NULL, 1, NULL);
    xTaskCreate(oled_task, "OLED Task", 1024, NULL, 1, NULL);
    xTaskCreate(oled_task, "OLED Task", 1024, NULL, 1, &oledTaskHandle);

    // Inicialização do scheduler
    vTaskStartScheduler();

    while (true) {}
}