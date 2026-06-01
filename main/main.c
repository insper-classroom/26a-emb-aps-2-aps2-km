/**
 * subway_controller_rtos.c - Controle com RTOS para Subway Surfers
 * 
 * Estrutura com FreeRTOS:
 * - Task_IMU: Lê acelerômetro e envia comandos de direção para a fila
 * - Task_Botoes: Lê botões com debounce e envia comandos para a fila
 * - Task_Comunicacao: Processa fila e envia comandos pela USB Serial
 * - Task_LED: Gerencia LED de conexão/status
 * 
 * Conexões:
 * - MPU6050 SDA -> GPIO2, SCL -> GPIO3
 * - Botão LIGA/DESLIGA -> GPIO8
 * - Botão DESLIZE_ESQUERDA_2X -> GPIO9
 * - Botão HOVER (Espaço) -> GPIO10
 * - Botão PULAR -> GPIO11
 * - Botão DESLIZAR -> GPIO12
 * - Potenciômetro -> GPIO26 (ADC0)
 * - LED Conexão -> GPIO25
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// ========== CONFIGURAÇÕES DOS PINOS ==========
#define I2C_PORT i2c1
#define I2C_SDA_PIN 2
#define I2C_SCL_PIN 3

#define BTN_LIGA_PIN 8                   // Botão liga/desliga (GPIO8)
#define BTN_DESLIZE_ESQUERDA_2X_PIN 9    // Botão deslizar 2x esquerda
#define BTN_HOVER_PIN 10                 // Botão Hoverboard (Espaço)
#define BTN_PULAR_PIN 11
#define BTN_DESLIZAR_PIN 12
#define LED_CONEXAO_PIN 25               // LED de indicação de conexão
#define POT_PIN 26                       // ADC0

// ========== CONFIGURAÇÕES DO MPU6050 ==========
#define MPU6050_ADDR 0x68
#define I2C_BAUD_RATE 400000
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B
#define WHO_AM_I 0x75

// ========== PARÂMETROS DE SENSIBILIDADE ==========
#define THRESHOLD_MIN 3000
#define THRESHOLD_MAX 10000
#define DEADZONE_BASE 1500
#define CALIBRATION_SAMPLES 50

// ========== CONFIGURAÇÕES RTOS ==========
#define TASK_STACK_SIZE 256
#define TASK_IMU_PRIORITY 2
#define TASK_BOTOES_PRIORITY 2
#define TASK_COMUNICACAO_PRIORITY 2
#define TASK_LED_PRIORITY 1

#define QUEUE_LENGTH 32
#define BTN_DEBOUNCE_MS 50
#define LOOP_DELAY_MS 30

// ========== TIPOS DE DADOS ==========

// Comandos que podem ser enviados para o PC
typedef enum {
    CMD_ESQUERDA = 'L',
    CMD_DIREITA = 'R',
    CMD_PULAR = 'U',
    CMD_DESLIZAR = 'D',
    CMD_HOVER = 'H',        // Hoverboard (Espaço)
    CMD_NEUTRO = 'S'
} comando_t;

// Estrutura para mensagens na fila
typedef struct {
    comando_t cmd;
    uint32_t timestamp_ms;
} mensagem_t;

// Estrutura para parâmetros de sensibilidade (protegida por semáforo)
typedef struct {
    int16_t threshold;
    int16_t deadzone;
} sensibilidade_t;

// ========== HANDLES GLOBAIS ==========
static QueueHandle_t xComandoQueue = NULL;
static SemaphoreHandle_t xSensibilidadeMutex = NULL;
static sensibilidade_t g_sensibilidade = {THRESHOLD_MIN, DEADZONE_BASE};
static int16_t g_neutral_x = 0;
static SemaphoreHandle_t xNeutralMutex = NULL;

// Controle do botão liga/desliga
static bool sistema_ligado = true;

// ========== FUNÇÕES DO MPU6050 ==========

void mpu6050_write_register(uint8_t reg, uint8_t data) {
    uint8_t buf[2] = {reg, data};
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, buf, 2, false);
}

int16_t mpu6050_read_16bit(uint8_t reg) {
    uint8_t buf[2];
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU6050_ADDR, buf, 2, false);
    return (int16_t)((buf[0] << 8) | buf[1]);
}

int16_t read_accel_x(void) {
    return mpu6050_read_16bit(ACCEL_XOUT_H);
}

bool mpu6050_init(void) {
    uint8_t who_am_i = 0;
    uint8_t reg = WHO_AM_I;
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU6050_ADDR, &who_am_i, 1, false);
    
    if (who_am_i != 0x68) {
        return false;
    }
    mpu6050_write_register(PWR_MGMT_1, 0x00);
    return true;
}

// ========== FUNÇÃO PARA ENVIAR 2X ESQUERDA ==========
void enviar_duas_esquerdas(void) {
    // Envia primeiro comando de ESQUERDA
    mensagem_t msg1 = {.cmd = CMD_ESQUERDA, .timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS};
    xQueueSend(xComandoQueue, &msg1, 0);
    printf("[BOTAO] 1a ESQUERDA enviada\n");
    
    // Aguarda para garantir que o primeiro comando foi processado
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // Envia comando NEUTRO para "soltar" a tecla entre os movimentos
    mensagem_t msg_neutro = {.cmd = CMD_NEUTRO, .timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS};
    xQueueSend(xComandoQueue, &msg_neutro, 0);
    
    // Aguarda para dar tempo do personagem parar
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // Envia segundo comando de ESQUERDA
    mensagem_t msg2 = {.cmd = CMD_ESQUERDA, .timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS};
    xQueueSend(xComandoQueue, &msg2, 0);
    printf("[BOTAO] 2a ESQUERDA enviada\n");
    
    // Aguarda e envia neutro novamente
    vTaskDelay(pdMS_TO_TICKS(50));
    xQueueSend(xComandoQueue, &msg_neutro, 0);
    
    printf("[BOTAO] DUAS ESQUERDAS concluido!\n");
}

// ========== TASK: IMU ESTÁVEL E FUNCIONAL (COM PAUSA) ==========

void task_imu(void *pvParameters) {
    TickType_t xLastWakeTime;
    
    // Calibração (só executa uma vez no início)
    int16_t neutral;
    int32_t sum = 0;
    printf("[IMU] Calibrando... Mantenha o controle PARADO!\n");
    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        sum += read_accel_x();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    neutral = (int16_t)(sum / CALIBRATION_SAMPLES);
    
    if (xSemaphoreTake(xNeutralMutex, portMAX_DELAY) == pdTRUE) {
        g_neutral_x = neutral;
        xSemaphoreGive(xNeutralMutex);
    }
    
    printf("[IMU] Calibrado! Neutro = %d\n", neutral);
    
    comando_t last_sent = CMD_NEUTRO;
    TickType_t last_send_time = 0;
    
    // VALORES TESTADOS E FUNCIONAIS
    int16_t SENS_THRESHOLD = 4500;
    int16_t SENS_DEADZONE = 1200;
    
    printf("[IMU] Threshold = %d, Deadzone = %d\n", SENS_THRESHOLD, SENS_DEADZONE);
    printf("[IMU] Controle parado NAO deve enviar comandos!\n");
    
    while (1) {
        // Se o sistema estiver desligado, apenas espera
        if (!sistema_ligado) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;  // Pula o resto do loop
        }
        
        xLastWakeTime = xTaskGetTickCount();
        
        int16_t accel_x = read_accel_x();
        int32_t deviation = (int32_t)accel_x - (int32_t)neutral;
        
        // MOSTRA VALORES REAIS
        static int debug_count = 0;
        debug_count++;
        if (debug_count >= 50) {
            debug_count = 0;
            printf("[INFO] accel=%d, neutro=%d, desvio=%d (threshold=%d)\n", 
                   accel_x, neutral, deviation, SENS_THRESHOLD);
        }
        
        // Lógica com zona morta
        comando_t desired_cmd = CMD_NEUTRO;
        
        if (deviation > SENS_THRESHOLD) {
            desired_cmd = CMD_DIREITA;
            printf("[MOVE] >>> DIREITA! desvio=%d\n", deviation);
        } else if (deviation < -SENS_THRESHOLD) {
            desired_cmd = CMD_ESQUERDA;
            printf("[MOVE] <<< ESQUERDA! desvio=%d\n", deviation);
        }
        
        // Só envia se mudou o estado
        if (desired_cmd != last_sent) {
            if (desired_cmd != CMD_NEUTRO) {
                mensagem_t msg = {.cmd = desired_cmd, .timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS};
                xQueueSend(xComandoQueue, &msg, 0);
                last_send_time = xTaskGetTickCount();
                printf("[ENVIO] Comando: %c\n", desired_cmd);
            } else if (last_sent != CMD_NEUTRO) {
                mensagem_t msg = {.cmd = CMD_NEUTRO, .timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS};
                xQueueSend(xComandoQueue, &msg, 0);
                printf("[ENVIO] Comando: S (parado)\n");
            }
            last_sent = desired_cmd;
        }
        
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(LOOP_DELAY_MS));
    }
    vTaskDelete(NULL);
}

// ========== TASK: LEITURA DOS BOTÕES ==========

void task_botoes(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    // Estados anteriores para detecção de borda
    bool last_liga = true;
    bool last_deslize_esquerda_2x = true;
    bool last_hover = true;
    bool last_pular = true;
    bool last_deslizar = true;
    
    // Timestamps para debounce
    uint32_t last_liga_time = 0;
    uint32_t last_deslize_esquerda_2x_time = 0;
    uint32_t last_hover_time = 0;
    uint32_t last_pular_time = 0;
    uint32_t last_deslizar_time = 0;
    
    while (1) {
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        
        // Botão LIGA/DESLIGA (GP8)
        bool current_liga = gpio_get(BTN_LIGA_PIN);
        if (!current_liga && last_liga && (now - last_liga_time) > BTN_DEBOUNCE_MS) {
            last_liga_time = now;
            sistema_ligado = !sistema_ligado;
            
            if (sistema_ligado) {
                printf("[SISTEMA] LIGADO\n");
                // Opcional: enviar um comando de heartbeatao ligar
                mensagem_t msg = {.cmd = CMD_NEUTRO, .timestamp_ms = now};
                xQueueSend(xComandoQueue, &msg, 0);
            } else {
                printf("[SISTEMA] DESLIGADO\n");
                // Quando desliga, envia comando NEUTRO para parar qualquer movimento
                mensagem_t msg = {.cmd = CMD_NEUTRO, .timestamp_ms = now};
                xQueueSend(xComandoQueue, &msg, 0);
                // Solta todas as teclas via Python (o Python vai receber S)
            }
        }
        last_liga = current_liga;
        
        // Só processa os outros botoes se o sistema estiver ligado
        if (sistema_ligado) {
            // Botão DESLIZE ESQUERDA 2X (GP9)
            bool current_deslize_esquerda_2x = gpio_get(BTN_DESLIZE_ESQUERDA_2X_PIN);
            if (!current_deslize_esquerda_2x && last_deslize_esquerda_2x && 
                (now - last_deslize_esquerda_2x_time) > BTN_DEBOUNCE_MS) {
                last_deslize_esquerda_2x_time = now;
                enviar_duas_esquerdas();
            }
            last_deslize_esquerda_2x = current_deslize_esquerda_2x;
            
            // Botão HOVER (GP10) - Espaço
            bool current_hover = gpio_get(BTN_HOVER_PIN);
            if (!current_hover && last_hover && (now - last_hover_time) > BTN_DEBOUNCE_MS) {
                last_hover_time = now;
                mensagem_t msg = {.cmd = CMD_HOVER, .timestamp_ms = now};
                xQueueSend(xComandoQueue, &msg, 0);
                printf("[BOTAO] HOVERBOARD (ESPACO)!\n");
            }
            last_hover = current_hover;
            
            // Botão PULAR (GP11)
            bool current_pular = gpio_get(BTN_PULAR_PIN);
            if (!current_pular && last_pular && (now - last_pular_time) > BTN_DEBOUNCE_MS) {
                last_pular_time = now;
                mensagem_t msg = {.cmd = CMD_PULAR, .timestamp_ms = now};
                xQueueSend(xComandoQueue, &msg, 0);
                printf("[BOTAO] PULAR!\n");
            }
            last_pular = current_pular;
            
            // Botão DESLIZAR (GP12)
            bool current_deslizar = gpio_get(BTN_DESLIZAR_PIN);
            if (!current_deslizar && last_deslizar && (now - last_deslizar_time) > BTN_DEBOUNCE_MS) {
                last_deslizar_time = now;
                mensagem_t msg = {.cmd = CMD_DESLIZAR, .timestamp_ms = now};
                xQueueSend(xComandoQueue, &msg, 0);
                printf("[BOTAO] DESLIZAR!\n");
            }
            last_deslizar = current_deslizar;
        } else {
            // Se o sistema está desligado, atualiza os estados anteriores mesmo assim
            // para não disparar comandos quando religar
            last_deslize_esquerda_2x = gpio_get(BTN_DESLIZE_ESQUERDA_2X_PIN);
            last_hover = gpio_get(BTN_HOVER_PIN);
            last_pular = gpio_get(BTN_PULAR_PIN);
            last_deslizar = gpio_get(BTN_DESLIZAR_PIN);
        }
        
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20));
    }
    vTaskDelete(NULL);
}

// ========== TASK: COMUNICAÇÃO SERIAL ==========

void task_comunicacao(void *pvParameters) {
    mensagem_t msg;
    comando_t last_sent = CMD_NEUTRO;
    TickType_t last_heartbeat = xTaskGetTickCount();
    
    while (1) {
        if (xQueueReceive(xComandoQueue, &msg, pdMS_TO_TICKS(100))) {
            // Só envia comandos se o sistema estiver ligado
            // Exceto comandos especiais se houver
            if (sistema_ligado || msg.cmd == CMD_NEUTRO) {
                printf("%c\n", msg.cmd);
                
                switch(msg.cmd) {
                    case CMD_ESQUERDA: printf("  ⬅️ ESQUERDA\n"); break;
                    case CMD_DIREITA: printf("  ➡️ DIREITA\n"); break;
                    case CMD_PULAR: printf("  🔼 PULAR\n"); break;
                    case CMD_DESLIZAR: printf("  🔽 DESLIZAR\n"); break;
                    case CMD_HOVER: printf("  🛹 HOVERBOARD (ESPACO)\n"); break;
                    case CMD_NEUTRO: break;
                }
                last_sent = msg.cmd;
            }
        }
        
        // Heartbeat a cada 5 segundos (indica que o controle está vivo)
        if ((xTaskGetTickCount() - last_heartbeat) > pdMS_TO_TICKS(5000)) {
            last_heartbeat = xTaskGetTickCount();
        }
    }
    vTaskDelete(NULL);
}

// ========== TASK: LED DE STATUS ==========

void task_led(void *pvParameters) {
    bool led_state = false;
    
    while (1) {
        if (sistema_ligado) {
            // Pisca lentamente quando ligado
            led_state = !led_state;
            gpio_put(LED_CONEXAO_PIN, led_state);
            vTaskDelay(pdMS_TO_TICKS(500));
        } else {
            // Apagado quando desligado
            gpio_put(LED_CONEXAO_PIN, 0);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
    vTaskDelete(NULL);
}

// ========== CONFIGURAÇÕES DE HARDWARE ==========

void setup_hardware(void) {
    // I2C
    i2c_init(I2C_PORT, I2C_BAUD_RATE);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    
    // Botões (pull-up interno)
    gpio_init(BTN_LIGA_PIN);
    gpio_set_dir(BTN_LIGA_PIN, GPIO_IN);
    gpio_pull_up(BTN_LIGA_PIN);
    
    gpio_init(BTN_DESLIZE_ESQUERDA_2X_PIN);
    gpio_set_dir(BTN_DESLIZE_ESQUERDA_2X_PIN, GPIO_IN);
    gpio_pull_up(BTN_DESLIZE_ESQUERDA_2X_PIN);
    
    gpio_init(BTN_HOVER_PIN);
    gpio_set_dir(BTN_HOVER_PIN, GPIO_IN);
    gpio_pull_up(BTN_HOVER_PIN);
    
    gpio_init(BTN_PULAR_PIN);
    gpio_set_dir(BTN_PULAR_PIN, GPIO_IN);
    gpio_pull_up(BTN_PULAR_PIN);
    
    gpio_init(BTN_DESLIZAR_PIN);
    gpio_set_dir(BTN_DESLIZAR_PIN, GPIO_IN);
    gpio_pull_up(BTN_DESLIZAR_PIN);
    
    // LED
    gpio_init(LED_CONEXAO_PIN);
    gpio_set_dir(LED_CONEXAO_PIN, GPIO_OUT);
    gpio_put(LED_CONEXAO_PIN, 1);
    
    // ADC
    adc_init();
    adc_gpio_init(POT_PIN);
    adc_select_input(0);
}

// ========== FUNÇÃO PRINCIPAL ==========

int main(void) {
    stdio_init_all();
    sleep_ms(2000);
    
    printf("\n========================================\n");
    printf("   SUBWAY SURFERS CONTROLLER - RTOS\n");
    printf("   Estrutura: Tasks + Filas + Semáforos\n");
    printf("========================================\n\n");
    
    setup_hardware();
    
    if (!mpu6050_init()) {
        printf("ERRO: MPU6050 nao encontrado!\n");
        while (1) { tight_loop_contents(); }
    }
    printf("MPU6050 inicializado!\n");
    
    xComandoQueue = xQueueCreate(QUEUE_LENGTH, sizeof(mensagem_t));
    configASSERT(xComandoQueue != NULL);
    
    xSensibilidadeMutex = xSemaphoreCreateMutex();
    configASSERT(xSensibilidadeMutex != NULL);
    
    xNeutralMutex = xSemaphoreCreateMutex();
    configASSERT(xNeutralMutex != NULL);
    
    xTaskCreate(task_imu, "IMU_Task", TASK_STACK_SIZE, NULL, TASK_IMU_PRIORITY, NULL);
    xTaskCreate(task_botoes, "Botoes_Task", TASK_STACK_SIZE, NULL, TASK_BOTOES_PRIORITY, NULL);
    xTaskCreate(task_comunicacao, "Com_Task", TASK_STACK_SIZE, NULL, TASK_COMUNICACAO_PRIORITY, NULL);
    xTaskCreate(task_led, "LED_Task", TASK_STACK_SIZE, NULL, TASK_LED_PRIORITY, NULL);
    
    printf("\n✅ RTOS iniciado! Tasks criadas:\n");
    printf("   - IMU_Task (le acelerometro)\n");
    printf("   - Botoes_Task (le botoes: LIGA, GP9, HOVER, PULAR, DESLIZAR)\n");
    printf("   - ADC_Task (le potenciometro)\n");
    printf("   - Com_Task (envia comandos)\n");
    printf("   - LED_Task (gerencia LED)\n");
    printf("\n🎮 Controle pronto!\n");
    printf("📌 GPIO8 = LIGA/DESLIGA\n");
    printf("📌 GPIO9 = DESLIZAR 2X ESQUERDA\n");
    printf("📌 GPIO10 = HOVERBOARD (Espaco)\n");
    printf("📌 GPIO11 = PULAR (UP)\n");
    printf("📌 GPIO12 = DESLIZAR (DOWN)\n\n");
    
    vTaskStartScheduler();
    
    while (1) { tight_loop_contents(); }
    return 0;
}