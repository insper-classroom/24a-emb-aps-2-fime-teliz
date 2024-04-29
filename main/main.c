/*
 * LED blink with FreeRTOS
 */
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>
#include "mpu6050.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "hardware/i2c.h"


#include <string.h>
#include <Fusion.h>


#include "pico/stdlib.h"
#include <stdio.h>
#include <stdlib.h>

#include "hc05.h"
#include "hc05.c"

#define deadzone 225
#define SAMPLE_PERIOD 0.05f

#define uart uart0

const int UART_TX_PIN = 0;
const int UART_RX_PIN = 1;


volatile int ADC_X = 26;
volatile int ADC_Y = 27;
const int MPU_ADDRESS = 0x68;
const int I2C_SDA_GPIO = 4;
const int I2C_SCL_GPIO = 5;
const int BTN_MACRO = 20;

const char IMU_X_HW_ID = 1;
const char IMU_Y_HW_ID = 2;
const char ADC_HW_ID = 3;
const char EOP = -1;

const int ENCA_PIN = 17;
const int ENCB_PIN = 16;

const int BTN_LINE = 18;
const int BTN_LOCK = 19;

volatile int camera_lock_flag = 0;
volatile int jurumeu_flag = 0;
volatile int state_line = 0;

typedef struct adc {
    int axis;
    int val;
} adc_t;

typedef struct movement {
    int x;
    int y;
} movement_t;


QueueHandle_t xQueueAdc;
QueueHandle_t xQueueIMU;
QueueHandle_t xQueueBtn;


movement_t *movement;


void uart_task(void *p) {
    adc_t adcData;
    adc_t imuData;
    int btnData;
    uart_init(uart0, 115200);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    
    while (1) {
        if (xQueueReceive(xQueueAdc, &adcData, 1)) {
            int axis = adcData.axis;
            int val = adcData.val;

            uart_putc_raw(uart, ADC_HW_ID);
            uart_putc_raw(uart, axis);
            uart_putc_raw(uart, val);
            uart_putc_raw(uart, EOP);
        }
        if (xQueueReceive(xQueueIMU, &imuData, 10)) {
            int val = imuData.val;
            int msb = val >> 8; 
            int lsb = val & 0xFF;
            
            if (imuData.axis == 0 && camera_lock_flag == 0) {
            uart_putc_raw(uart, IMU_X_HW_ID);
                uart_putc_raw(uart, msb);
                uart_putc_raw(uart, lsb);
                uart_putc_raw(uart, EOP);
        
            } else if (jurumeu_flag ==0){
                uart_putc_raw(uart, IMU_Y_HW_ID);
                uart_putc_raw(uart, imuData.val);
                uart_putc_raw(uart, EOP);
                
            }
        }
        if (xQueueReceive(xQueueBtn, &btnData, 1)) {
            // se data for 0 ou 1, entao é o botao de macro
            // se data for 31, entao é o botao de linha
            // se data for 11 ou 12, entao é o botao de rotação

            
            if (btnData == 0 || btnData == 1){
                // printf("MACRO\n");
                uart_putc_raw(uart, 5);
                uart_putc_raw(uart, btnData);
                uart_putc_raw(uart, EOP);
            } else if (btnData == 235){
                // printf("LINE\n");
                uart_putc_raw(uart, 4);
                uart_putc_raw(uart, state_line);
                uart_putc_raw(uart, EOP);
            } else if (btnData == 49){
                // printf("RIGHT\n");
                uart_putc_raw(uart, 6);
                uart_putc_raw(uart, 1);
                uart_putc_raw(uart, EOP);
            } else if (btnData == 12544){
                // printf("LEFT\n");
                uart_putc_raw(uart, 6);
                uart_putc_raw(uart, 2);
                uart_putc_raw(uart, EOP);
            }
        }

    }
    vTaskDelay(pdMS_TO_TICKS(10));
}

void x_adc_task(void *p) {
    adc_t data;
    adc_init();
    adc_gpio_init(ADC_X);
    int esto_mandando_dado = 0;

    while (1) {
        adc_select_input(0); // Select ADC input 0 (GPIO26)
        float result = adc_read();

        result = result - 2048;
        result = result / 8;

        if (abs(result) < deadzone) {
            data.val = 0; // resultado pode ser 2 ou -2 (vai ser lido pelo python)
            if (esto_mandando_dado == 1){
                xQueueSend(xQueueAdc, &data, portMAX_DELAY);
                esto_mandando_dado = 0;
            }
        }else{            
            data.val = 2*result/abs(result); // resultado pode ser 2 ou -2 (vai ser lido pelo python)
            data.axis = 1;
            xQueueSend(xQueueAdc, &data, portMAX_DELAY);
            esto_mandando_dado = 1;
        }
        
        // printf("X: %d\n", data.val);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void y_adc_task(void *p) {
    adc_t data;
    adc_init();
    adc_gpio_init(ADC_Y);
    int estou_mandando_dado = 0;

    while (1) {
        adc_select_input(1); // Select ADC input 1 (GPIO27)
        float result = adc_read();

        result = result - 2048;
        result = result / 8;

        
        if (abs(result) < deadzone) {
            data.val = 0; // resultado pode ser 2 ou -2 (vai ser lido pelo python)
            if (estou_mandando_dado == 1){
                xQueueSend(xQueueAdc, &data, portMAX_DELAY);
                estou_mandando_dado = 0;
            }
        }else{            
            data.val = 2*result/abs(result); // resultado pode ser 2 ou -2 (vai ser lido pelo python)
            xQueueSend(xQueueAdc, &data, portMAX_DELAY);
            estou_mandando_dado = 1;
        }
        data.axis = 0;
        
        // printf("Y: %d\n", data.val);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}



void hc06_task(void *p) {
    int connected=1;
    uart_init(hc05_UART_ID, hc05_BAUD_RATE);
    gpio_set_function(hc05_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(hc05_RX_PIN, GPIO_FUNC_UART);
    
    hc05_init("vara_de_pesca", "0000");

    while (true) {
        //uart_puts(HC06_UART_ID, "OLAAA ");
        if (connected == 0){
            if (hc05_check_connection()){
            ////printf("Connected\n");
            connected = 1;
            } else {
                //printg("Not connected\n");

            };
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        else{
            adc_t data_ADC_HC06;
            xQueueAdc = xQueueCreate(10, sizeof(adc_t));
            xQueueReceive(xQueueAdc, &data_ADC_HC06, portMAX_DELAY);

            adc_t data_IMU_HC06;
            xQueueIMU = xQueueCreate(10, sizeof(adc_t));
            xQueueReceive(xQueueIMU, &data_IMU_HC06, portMAX_DELAY);


        }

    }
}
static void mpu6050_reset() {
    uint8_t resetBuffer[] = {0x6B, 0x00};
    i2c_write_blocking(i2c_default, MPU_ADDRESS, resetBuffer, 2, false);

    uint8_t writeBuffer[2];
    writeBuffer[0] = MPUREG_ACCEL_CONFIG;
    writeBuffer[1] = 0 << 3;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, writeBuffer, 2, false);

    writeBuffer[0] = MPUREG_ACCEL_CONFIG;
    writeBuffer[1] = 0 << 4;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, writeBuffer, 2, false);

    writeBuffer[0] = MPUREG_GYRO_CONFIG;
    writeBuffer[1] = 0 << 3;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, writeBuffer, 2, false);

    writeBuffer[0] = MPUREG_GYRO_CONFIG;
    writeBuffer[1] = 0 << 4;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, writeBuffer, 2, false);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    uint8_t readBuffer[6];

    uint8_t val = 0x3B;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, readBuffer, 6, false);

    for (int i = 0; i < 3; i++) {
        accel[i] = (readBuffer[i * 2] << 8 | readBuffer[(i * 2) + 1]);
    }

    val = 0x43;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, readBuffer, 6, false);

    for (int i = 0; i < 3; i++) {
        gyro[i] = (readBuffer[i * 2] << 8 | readBuffer[(i * 2) + 1]);
    }

    val = 0x41;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, readBuffer, 2, false);

    *temp = readBuffer[0] << 8 | readBuffer[1];
}



void mpu6050_task(void *p) {
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_GPIO);
    gpio_pull_up(I2C_SCL_GPIO);

    adc_t imuData;

    mpu6050_reset();
    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);

    int16_t acceleration[3], gyro[3], temp;
    int count = 0;
    int oldatar;
    int newdatar;
    int oldatay;
    int newdatay;
    int delta_roll;
    int delta_yaw;

    while (1) {
        mpu6050_read_raw(acceleration, gyro, &temp);
        FusionVector gyroscope = {
            .axis.x = gyro[0] / 131.0f, // Conversão para graus/s
            .axis.y = gyro[1] / 131.0f,
            .axis.z = gyro[2] / 131.0f,
        };

        FusionVector accelerometer = {
            .axis.x = acceleration[0] / 16384.0f, // Conversão para g
            .axis.y = acceleration[1] / 16384.0f,
            .axis.z = acceleration[2] / 16384.0f,
        };      

        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);

        const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

        if(count == 0){
            oldatar = (int) euler.angle.roll;
        }else if(count == 3){
            newdatar = (int) euler.angle.roll;
            delta_roll = newdatar - oldatar;
            count = -1;
        }
        count++;
        if(abs(newdatar) > 75){
            imuData.val = 1;
        }
        else{
            imuData.val = 0;
        }
        imuData.axis = 1;
        xQueueSend(xQueueIMU, &imuData, portMAX_DELAY);
        imuData.val = euler.angle.yaw;
        imuData.axis = 0;
        // printf("Yaw: %d\n", (int) imuData.val);
        xQueueSend(xQueueIMU, &imuData, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}



void task_macro( void *p) {
    int status = 0;
    gpio_init(BTN_MACRO);
    gpio_set_dir(BTN_MACRO, GPIO_IN);
    gpio_pull_up(BTN_MACRO);
    while(1){
        if (gpio_get(BTN_MACRO) == 0){
            jurumeu_flag = 1;
            status = 1;
            // printf("MACRO\n");
            xQueueSend(xQueueBtn, &status, portMAX_DELAY);
            vTaskDelay(pdMS_TO_TICKS(1000));

            status = 0;
            // printf("macro\n");
            xQueueSend(xQueueBtn, &status, portMAX_DELAY);
            vTaskDelay(pdMS_TO_TICKS(500));
        }else{
            jurumeu_flag = 0;
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}
void scroll_task(void *p) {
    static const int8_t state_table[] = {
        0, -1,  1,  0,
        1,  0,  0, -1,
        -1,  0,  0,  1,
        0,  1, -1,  0
    };
    uint8_t enc_state = 0; // Current state of the encoder
    int8_t last_encoded = 0; // Last encoded state
    int8_t encoded;
    int sum;
    int last_sum = 0; // Last non-zero sum to filter out noise
    int debounce_counter = 0; // Debounce counter

    // Initialize GPIO pins for the encoder
    gpio_init(ENCA_PIN);
    gpio_init(ENCB_PIN);

    gpio_set_dir(ENCA_PIN, GPIO_IN);
    gpio_set_dir(ENCB_PIN, GPIO_IN);

    gpio_pull_up(ENCA_PIN);  // Enable internal pull-up
    gpio_pull_up(ENCB_PIN);  // Enable internal pull-up

    last_encoded = (gpio_get(ENCA_PIN) << 1) | gpio_get(ENCB_PIN);

    while (1) {
        encoded = (gpio_get(ENCA_PIN) << 1) | gpio_get(ENCB_PIN);
        enc_state = (enc_state << 2) | encoded;
        sum = state_table[enc_state & 0x0f];

        if (sum != 0) {
            if (sum == last_sum) {
                if (++debounce_counter > 1) {  // Check if the same movement is read consecutively
                    if (sum == 1) {
                        // printf("RIGHT\n");
                        xQueueSend(xQueueBtn, 11, 10);
                        vTaskDelay(pdMS_TO_TICKS(100));
                    } else if (sum == -1) {
                        // printf("LEFT\n");
                        xQueueSend(xQueueBtn, 12, 10);
                        vTaskDelay(pdMS_TO_TICKS(100));
                    }
                    debounce_counter = 0;  // Reset the counter after confirming the direction
                }
            } else {
                debounce_counter = 0;  // Reset the counter if the direction changes
            }
            last_sum = sum;  // Update last_sum to the current sum
        }

        vTaskDelay(pdMS_TO_TICKS(1)); // Poll every 1 ms to improve responsiveness
    }
}
void btn_lock_task(void *p) {
    gpio_init(BTN_LOCK);
    gpio_set_dir(BTN_LOCK, GPIO_IN);
    gpio_pull_up(BTN_LOCK);
    while(1){
        if (gpio_get(BTN_LOCK) == 0){
            if (camera_lock_flag == 0){
                // printf("LOCK\n");
                camera_lock_flag = 1;
            } else {
                camera_lock_flag = 0;
                // printf("UNLOCK\n");
            }
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void btn_line_task(void *p) {
    gpio_init(BTN_LINE);
    gpio_set_dir(BTN_LINE, GPIO_IN);
    gpio_pull_up(BTN_LINE);
    while(1){
        if (gpio_get(BTN_LINE) == 0 && state_line == 0){
            // printf("LINE\n");
            xQueueSend(xQueueBtn, 0x04, portMAX_DELAY);
            state_line = 1;
        }
        if (gpio_get(BTN_LINE) == 1 && state_line == 1){
            xQueueSend(xQueueBtn, 0x04, portMAX_DELAY);
            state_line = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

int main() {
    stdio_init_all();

    //////printf("Start bluetooth task\n");

    xTaskCreate(uart_task, "UART_Task", 4096, NULL, 1, NULL);
/* coloque as tasks inativas aqui
    xTaskCreate(hc05_task, "HC_Task 1", 4096, NULL, 1, NULL);

*/
    
    xTaskCreate(x_adc_task, "ADC_Task 1", 4096, NULL, 1, NULL);
    xTaskCreate(y_adc_task, "ADC_Task 2", 4096, NULL, 1, NULL);
    xTaskCreate(mpu6050_task, "MPU6050_Task", 4096, NULL, 1, NULL);
    xTaskCreate(task_macro, "Macro_Task", 4096, NULL, 1, NULL);
    xTaskCreate(btn_line_task, "Line_Task", 4096, NULL, 1, NULL);
    xTaskCreate(btn_lock_task, "Lock_Task", 4096, NULL, 1, NULL);
    xTaskCreate(scroll_task, "Rotate_Task", 4096, NULL, 1, NULL);



    

    xQueueAdc = xQueueCreate(32, sizeof(adc_t));
    xQueueIMU = xQueueCreate(32, sizeof(adc_t));
    xQueueBtn = xQueueCreate(32, sizeof(int));



    vTaskStartScheduler();

    while (true)
        ;
}
