#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "hardware/irq.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"


// ========== CONFIGURAÇÕES DOS PINOS ==========
#define I2C_PORT        i2c1
#define I2C_SDA_PIN     2
#define I2C_SCL_PIN     3

#define BTN_LIGA_PIN                8
#define BTN_DESLIZE_ESQUERDA_2X_PIN 9
#define BTN_HOVER_PIN               10
#define BTN_PULAR_PIN               11
#define BTN_DESLIZAR_PIN            12

#define LED_CONEXAO_PIN   25
#define LED_CALIBRADO_PIN 16
#define POT_PIN           26

// ========== CONFIGURAÇÕES MPU6050 ==========
#define MPU6050_ADDR  0x68
#define I2C_BAUD_RATE 400000
#define PWR_MGMT_1    0x6B
#define ACCEL_XOUT_H  0x3B
#define WHO_AM_I      0x75

// ========== PARÂMETROS DE SENSIBILIDADE ==========
#define CALIBRATION_SAMPLES 50
#define SENS_THRESHOLD      4500
#define SENS_DEADZONE       1200

// ========== CONFIGURAÇÕES HC-06 ==========
#define HC06_NAME      "SUBWAY_CTRL"
#define HC06_PIN       "1234"
#define HC06_UART_ID   uart1
#define HC06_BAUD_RATE 115200
#define HC06_TX_PIN    4    // Pico TX -> HC-06 RX
#define HC06_RX_PIN    5

// ========== CONFIGURAÇÕES RTOS ==========
#define TASK_STACK_SIZE           512
#define TASK_IMU_PRIORITY         2
#define TASK_BOTOES_PRIORITY      2
#define TASK_COMUNICACAO_PRIORITY 3   // mais alta: não pode perder comandos
#define TASK_LED_PRIORITY         1

#define QUEUE_CMDS_LENGTH    32   // fila de comandos (IMU + botões)
#define QUEUE_BT_TX_LENGTH   128  // fila de bytes para TX Bluetooth
#define BTN_DEBOUNCE_MS      50
#define LOOP_IMU_MS          30

// ========== CONFIGURAÇÕES SMP ==========
#define CORE_0 (1 << 0)
#define CORE_1 (1 << 1)

// ========== TIPOS ==========
typedef enum {
    CMD_ESQUERDA = 'L',
    CMD_DIREITA  = 'R',
    CMD_PULAR    = 'U',
    CMD_DESLIZAR = 'D',
    CMD_HOVER    = 'H',
    CMD_NEUTRO   = 'S'
} comando_t;

typedef struct {
    comando_t cmd;
    uint32_t  timestamp_ms;
} mensagem_t;

// ========== HANDLES GLOBAIS ==========
static QueueHandle_t xComandoQueue  = NULL;  // fila de comandos do jogo
static QueueHandle_t xBtTxQueue     = NULL;  // fila de bytes para o HC-06
static QueueHandle_t xSistemaQueue  = NULL;  // fila de 1 elemento: bool sistema_ligado

// ========== HELPERS: ESTADO DO SISTEMA ==========
// Lê o estado atual sem remover da fila (peek não-bloqueante)
static bool sistema_esta_ligado(void) {
    bool estado = true;
    xQueuePeek(xSistemaQueue, &estado, 0);
    return estado;
}

// Atualiza o estado: esvazia a fila e reinsere o novo valor
static void sistema_set_ligado(bool ligado) {
    bool dummy;
    xQueueReceive(xSistemaQueue, &dummy, 0);  // esvazia
    xQueueSend(xSistemaQueue, &ligado, 0);    // insere novo valor
}

// ========== BLUETOOTH: IRQ RX ==========
// ISR mínima: lê apenas um byte por interrupção — sem loop.
static void uart_bt_rx_handler(void) {
    if (uart_is_readable(HC06_UART_ID)) {
        uart_getc(HC06_UART_ID);  // lê um byte e sai; próximo byte dispara novo IRQ
    }
}

static void init_bt_uart(void) {
    uart_init(HC06_UART_ID, HC06_BAUD_RATE);
    gpio_set_function(HC06_TX_PIN, UART_FUNCSEL_NUM(HC06_UART_ID, HC06_TX_PIN));
    gpio_set_function(HC06_RX_PIN, UART_FUNCSEL_NUM(HC06_UART_ID, HC06_RX_PIN));
    uart_set_baudrate(HC06_UART_ID, HC06_BAUD_RATE);
    uart_set_hw_flow(HC06_UART_ID, false, false);
    uart_set_format(HC06_UART_ID, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(HC06_UART_ID, false);

    int UART_IRQ = (HC06_UART_ID == uart0) ? UART0_IRQ : UART1_IRQ;
    irq_set_exclusive_handler(UART_IRQ, uart_bt_rx_handler);
    irq_set_enabled(UART_IRQ, true);
    uart_set_irq_enables(HC06_UART_ID, true, false);
}

// ========== TASK: TX BLUETOOTH ==========
// Consome xBtTxQueue e envia byte a byte pela UART do HC-06.
static void task_bt_tx(void *pvParameters) {
    uint8_t ch;
    while (1) {
        if (xQueueReceive(xBtTxQueue, &ch, portMAX_DELAY) == pdTRUE) {
            uart_putc_raw(HC06_UART_ID, ch);
        }
    }
    vTaskDelete(NULL);
}

// Helper: enfileira string na fila BT (não bloqueia)
static void bt_enqueue_str(const char *str) {
    while (*str) {
        xQueueSend(xBtTxQueue, (const void *)str, 0);
        str++;
    }
}

// ========== MPU6050 ==========
static void mpu6050_write_register(uint8_t reg, uint8_t data) {
    uint8_t buf[2] = {reg, data};
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, buf, 2, false);
}

static int16_t mpu6050_read_16bit(uint8_t reg) {
    uint8_t buf[2];
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU6050_ADDR, buf, 2, false);
    return (int16_t)((buf[0] << 8) | buf[1]);
}

static int16_t read_accel_x(void) {
    return mpu6050_read_16bit(ACCEL_XOUT_H);
}

static bool mpu6050_init(void) {
    uint8_t who_am_i = 0;
    uint8_t reg = WHO_AM_I;
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU6050_ADDR, &who_am_i, 1, false);
    if (who_am_i != 0x68) return false;
    mpu6050_write_register(PWR_MGMT_1, 0x00);
    return true;
}

// ========== HELPER: 2X ESQUERDA ==========
static void enviar_duas_esquerdas(void) {
    mensagem_t msg   = {.cmd = CMD_ESQUERDA, .timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS};
    mensagem_t neutro = {.cmd = CMD_NEUTRO,  .timestamp_ms = msg.timestamp_ms};

    xQueueSend(xComandoQueue, &msg, 0);
    printf("[BOTAO] 1a ESQUERDA\n");
    vTaskDelay(pdMS_TO_TICKS(50));

    xQueueSend(xComandoQueue, &neutro, 0);
    vTaskDelay(pdMS_TO_TICKS(50));

    msg.timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    xQueueSend(xComandoQueue, &msg, 0);
    printf("[BOTAO] 2a ESQUERDA\n");
    vTaskDelay(pdMS_TO_TICKS(50));

    neutro.timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    xQueueSend(xComandoQueue, &neutro, 0);
    printf("[BOTAO] DUAS ESQUERDAS concluido!\n");
}

// ========== TASK: IMU ==========
// neutral é variável LOCAL — não precisa de global nem mutex.
static void task_imu(void *pvParameters) {
    // Calibração inicial
    int32_t sum = 0;
    printf("[IMU] Calibrando... Mantenha o controle PARADO!\n");
    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        sum += read_accel_x();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    int16_t neutral = (int16_t)(sum / CALIBRATION_SAMPLES);  // local: só task_imu usa

    for (int i = 0; i < 3; i++) {
        gpio_put(LED_CALIBRADO_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(150));
        gpio_put(LED_CALIBRADO_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(150));
    }
    printf("[IMU] Calibrado! Neutro = %d | Threshold = %d\n", neutral, SENS_THRESHOLD);

    comando_t  last_sent = CMD_NEUTRO;
    TickType_t xLastWakeTime;
    int        debug_count = 0;

    while (1) {
        // Usa helper para ler estado via fila (sem global)
        if (!sistema_esta_ligado()) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        xLastWakeTime = xTaskGetTickCount();

        int16_t accel_x   = read_accel_x();
        int32_t deviation = (int32_t)accel_x - (int32_t)neutral;

        // Debug a cada ~1.5 s
        if (++debug_count >= 50) {
            debug_count = 0;
            printf("[IMU] accel=%d neutro=%d desvio=%d\n", accel_x, neutral, deviation);
        }

        comando_t desired = CMD_NEUTRO;
        if      (deviation >  SENS_THRESHOLD) desired = CMD_DIREITA;
        else if (deviation < -SENS_THRESHOLD) desired = CMD_ESQUERDA;

        if (desired != last_sent) {
            mensagem_t msg = {
                .cmd          = (desired != CMD_NEUTRO) ? desired : CMD_NEUTRO,
                .timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS
            };
            xQueueSend(xComandoQueue, &msg, 0);
            printf("[IMU] Enviando: %c\n", msg.cmd);
            last_sent = desired;
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(LOOP_IMU_MS));
    }
}

// ========== TASK: BOTÕES ==========
static void task_botoes(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();

    bool last_liga            = true;
    bool last_deslize_esq_2x  = true;
    bool last_hover           = true;
    bool last_pular           = true;
    bool last_deslizar        = true;

    uint32_t t_liga = 0, t_esq2x = 0, t_hover = 0, t_pular = 0, t_deslizar = 0;

    while (1) {
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

        // --- LIGA/DESLIGA ---
        bool cur_liga = gpio_get(BTN_LIGA_PIN);
        if (!cur_liga && last_liga && (now - t_liga) > BTN_DEBOUNCE_MS) {
            t_liga = now;
            // Lê estado atual via fila e inverte
            bool atual = sistema_esta_ligado();
            sistema_set_ligado(!atual);
            mensagem_t msg = {.cmd = CMD_NEUTRO, .timestamp_ms = now};
            xQueueSend(xComandoQueue, &msg, 0);
            printf("[SISTEMA] %s\n", !atual ? "LIGADO" : "DESLIGADO");
        }
        last_liga = cur_liga;

        // Usa helper para ler estado via fila (sem global)
        if (sistema_esta_ligado()) {
            // --- DESLIZE 2X ESQUERDA ---
            bool cur_esq2x = gpio_get(BTN_DESLIZE_ESQUERDA_2X_PIN);
            if (!cur_esq2x && last_deslize_esq_2x && (now - t_esq2x) > BTN_DEBOUNCE_MS) {
                t_esq2x = now;
                enviar_duas_esquerdas();
            }
            last_deslize_esq_2x = cur_esq2x;

            // --- HOVER ---
            bool cur_hover = gpio_get(BTN_HOVER_PIN);
            if (!cur_hover && last_hover && (now - t_hover) > BTN_DEBOUNCE_MS) {
                t_hover = now;
                mensagem_t msg = {.cmd = CMD_HOVER, .timestamp_ms = now};
                xQueueSend(xComandoQueue, &msg, 0);
                printf("[BOTAO] HOVERBOARD\n");
            }
            last_hover = cur_hover;

            // --- PULAR ---
            bool cur_pular = gpio_get(BTN_PULAR_PIN);
            if (!cur_pular && last_pular && (now - t_pular) > BTN_DEBOUNCE_MS) {
                t_pular = now;
                mensagem_t msg = {.cmd = CMD_PULAR, .timestamp_ms = now};
                xQueueSend(xComandoQueue, &msg, 0);
                printf("[BOTAO] PULAR\n");
            }
            last_pular = cur_pular;

            // --- DESLIZAR ---
            bool cur_deslizar = gpio_get(BTN_DESLIZAR_PIN);
            if (!cur_deslizar && last_deslizar && (now - t_deslizar) > BTN_DEBOUNCE_MS) {
                t_deslizar = now;
                mensagem_t msg = {.cmd = CMD_DESLIZAR, .timestamp_ms = now};
                xQueueSend(xComandoQueue, &msg, 0);
                printf("[BOTAO] DESLIZAR\n");
            }
            last_deslizar = cur_deslizar;
        } else {
            // Atualiza estados para não disparar ao religar
            last_deslize_esq_2x = gpio_get(BTN_DESLIZE_ESQUERDA_2X_PIN);
            last_hover           = gpio_get(BTN_HOVER_PIN);
            last_pular           = gpio_get(BTN_PULAR_PIN);
            last_deslizar        = gpio_get(BTN_DESLIZAR_PIN);
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20));
    }
    vTaskDelete(NULL);
}

// ========== TASK: COMUNICAÇÃO (USB Serial + Bluetooth) ==========
static void task_comunicacao(void *pvParameters) {
    mensagem_t msg;
    char buf[4];  // ex: "L\n\0"

    while (1) {
        if (xQueueReceive(xComandoQueue, &msg, pdMS_TO_TICKS(100))) {
            // Usa helper para ler estado via fila (sem global)
            if (sistema_esta_ligado() || msg.cmd == CMD_NEUTRO) {

                // 1) Envia pela USB Serial (lido pelo subway_controller.py)
                printf("%c\n", msg.cmd);

                // 2) Envia pelo Bluetooth (mesmo protocolo: "<CHAR>\n")
                snprintf(buf, sizeof(buf), "%c\n", msg.cmd);
                bt_enqueue_str(buf);

                // Debug visual no terminal USB
                switch (msg.cmd) {
                    case CMD_ESQUERDA: printf("  [BT+USB] ESQUERDA\n");   break;
                    case CMD_DIREITA:  printf("  [BT+USB] DIREITA\n");    break;
                    case CMD_PULAR:    printf("  [BT+USB] PULAR\n");      break;
                    case CMD_DESLIZAR: printf("  [BT+USB] DESLIZAR\n");   break;
                    case CMD_HOVER:    printf("  [BT+USB] HOVERBOARD\n"); break;
                    case CMD_NEUTRO:   break;
                }
            }
        }
    }
    vTaskDelete(NULL);
}

// ========== TASK: LED ==========
static void task_led(void *pvParameters) {
    bool led_state = false;
    while (1) {
        // Usa helper para ler estado via fila (sem global)
        if (sistema_esta_ligado()) {
            led_state = !led_state;
            gpio_put(LED_CONEXAO_PIN, led_state);
            vTaskDelay(pdMS_TO_TICKS(500));  // pisca lento = ativo
        } else {
            gpio_put(LED_CONEXAO_PIN, 0);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
    vTaskDelete(NULL);
}

// ========== HARDWARE ==========
static void setup_hardware(void) {
    // I2C (MPU6050)
    i2c_init(I2C_PORT, I2C_BAUD_RATE);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    // Botões
    const uint btn_pins[] = {
        BTN_LIGA_PIN, BTN_DESLIZE_ESQUERDA_2X_PIN,
        BTN_HOVER_PIN, BTN_PULAR_PIN, BTN_DESLIZAR_PIN
    };
    for (int i = 0; i < 5; i++) {
        gpio_init(btn_pins[i]);
        gpio_set_dir(btn_pins[i], GPIO_IN);
        gpio_pull_up(btn_pins[i]);
    }

    // LEDs
    gpio_init(LED_CONEXAO_PIN);
    gpio_set_dir(LED_CONEXAO_PIN, GPIO_OUT);
    gpio_put(LED_CONEXAO_PIN, 1);

    gpio_init(LED_CALIBRADO_PIN);
    gpio_set_dir(LED_CALIBRADO_PIN, GPIO_OUT);
    gpio_put(LED_CALIBRADO_PIN, 0);

    // ADC
    adc_init();
    adc_gpio_init(POT_PIN);
    adc_select_input(0);
}

// ========== MAIN ==========
int main(void) {
    stdio_init_all();
    sleep_ms(2000);  // aguarda terminal USB

    printf("\n========================================\n");
    printf("   SUBWAY SURFERS CONTROLLER - RTOS + BT\n");
    printf("   IMU MPU6050 | Bluetooth HC-06\n");
    printf("========================================\n\n");

    setup_hardware();

    printf("[BT] Inicializando HC-06...\n");
    init_bt_uart();
    printf("[BT] HC-06 pronto (UART1 @ %d baud)\n\n", HC06_BAUD_RATE);

    if (!mpu6050_init()) {
        printf("ERRO: MPU6050 nao encontrado! Verifique I2C.\n");
        while (1) { tight_loop_contents(); }
    }
    printf("[IMU] MPU6050 inicializado!\n\n");

    // --- Filas ---
    xComandoQueue = xQueueCreate(QUEUE_CMDS_LENGTH, sizeof(mensagem_t));
    xBtTxQueue    = xQueueCreate(QUEUE_BT_TX_LENGTH, sizeof(uint8_t));
    // Fila de 1 elemento para o estado liga/desliga (substitui a global)
    xSistemaQueue = xQueueCreate(1, sizeof(bool));
    configASSERT(xComandoQueue != NULL);
    configASSERT(xBtTxQueue    != NULL);
    configASSERT(xSistemaQueue != NULL);

    // Estado inicial: ligado = true
    bool estado_inicial = true;
    xQueueSend(xSistemaQueue, &estado_inicial, 0);

    // --- Tasks ---
    TaskHandle_t hImu, hBotoes, hCom, hBtTx, hLed;

    xTaskCreate(task_imu,         "IMU",    TASK_STACK_SIZE, NULL, TASK_IMU_PRIORITY,           &hImu);
    xTaskCreate(task_botoes,      "Botoes", TASK_STACK_SIZE, NULL, TASK_BOTOES_PRIORITY,         &hBotoes);
    xTaskCreate(task_comunicacao, "Com",    TASK_STACK_SIZE, NULL, TASK_COMUNICACAO_PRIORITY,    &hCom);
    xTaskCreate(task_bt_tx,       "BtTX",   TASK_STACK_SIZE, NULL, TASK_COMUNICACAO_PRIORITY,    &hBtTx);
    xTaskCreate(task_led,         "LED",    256,             NULL, TASK_LED_PRIORITY,             &hLed);

    // Core 0: leitura de sensores e botões (tempo real, baixa latência)
    vTaskCoreAffinitySet(hImu,    CORE_0);
    vTaskCoreAffinitySet(hBotoes, CORE_0);

    // Core 1: comunicação e saídas (não bloqueia leitura dos sensores)
    vTaskCoreAffinitySet(hCom,   CORE_1);
    vTaskCoreAffinitySet(hBtTx,  CORE_1);
    vTaskCoreAffinitySet(hLed,   CORE_1);

    printf("✅ Tasks criadas (SMP: Core0=IMU+Botoes | Core1=Com+BT+LED)\n");
    printf("📌 GP2/3  = I2C MPU6050\n");
    printf("📌 GP4/5  = UART HC-06 (Bluetooth)\n");
    printf("📌 GP8    = LIGA/DESLIGA\n");
    printf("📌 GP9    = 2X ESQUERDA\n");
    printf("📌 GP10   = HOVER (Espaço)\n");
    printf("📌 GP11   = PULAR\n");
    printf("📌 GP12   = DESLIZAR\n\n");
    printf("🎮 Pronto! Conecte via Bluetooth ou USB.\n\n");

    vTaskStartScheduler();
    while (1) { tight_loop_contents(); }
    return 0;
}