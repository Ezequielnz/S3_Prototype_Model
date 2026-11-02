// Stubs mínimos para compilar la lógica de adquisición/DSP del firmware en una PC (Linux).
#pragma once
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <deque>
#include <vector>

#define IRAM_ATTR
typedef int BaseType_t;
typedef uint32_t TickType_t;
#define pdTRUE 1
#define pdFALSE 0
#define pdMS_TO_TICKS(x) (x)
#define portMAX_DELAY 0xFFFFFFFF
typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(x)
#define portEXIT_CRITICAL(x)
typedef void *TaskHandle_t;
typedef void *adc_continuous_handle_t;
typedef int adc_channel_t;

// Reloj simulado
static int64_t g_ahora_us = 0;
static inline int64_t esp_timer_get_time() { return g_ahora_us; }

// Colas: la de red envía de verdad por UDP; las demás no se usan en el simulador.
struct SimCola { int id; size_t item; };
typedef SimCola *QueueHandle_t;
void sim_enviar_paquete(const void *item);   // definida en sim_nodo.cpp
static inline int xQueueSend(QueueHandle_t q, const void *item, int) {
  if (q && q->id == 3) sim_enviar_paquete(item);
  return pdTRUE;
}
static inline int xQueueReceive(QueueHandle_t, void *, int) { return pdFALSE; }
static inline void vTaskDelay(int) {}

struct SimSerial {
  explicit operator bool() const { return true; }
  int availableForWrite() { return 1000; }
  void printf(const char *f, ...) { va_list a; va_start(a, f); vprintf(f, a); va_end(a); }
  void println(const char *s) { puts(s); }
};
static SimSerial Serial;
