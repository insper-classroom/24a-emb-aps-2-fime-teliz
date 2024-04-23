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

#include "hc06.h"

#define deadzone 220
#define SAMPLE_PERIOD 0.1f

volatile int ADC_X = 26;
volatile int ADC_Y = 27;
const int MPU_ADDRESS = 0x68;
const int I2C_SDA_GPIO = 4;
const int I2C_SCL_GPIO = 5;

const char IMU_X_HW_ID = 1;
const char IMU_Y_HW_ID = 2;
const char ADC_HW_ID = 3;
const char EOP = -1;

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
movement_t *movement;


void uart_task(void *p) {
    adc_t adcData;
    adc_t imuData;
    while (1) {
        if (xQueueReceive(xQueueAdc, &adcData, portMAX_DELAY)) {
            int axis = adcData.axis;
            int val = adcData.val;

            uart_putc_raw(uart0, ADC_HW_ID);
            uart_putc_raw(uart0, axis);
            uart_putc_raw(uart0, val);
            uart_putc_raw(uart0, EOP);
        }
        if (xQueueReceive(xQueueIMU, &imuData, portMAX_DELAY)) {
            int val = imuData.val;
            int msb = val >> 8; 
            int lsb = val & 0xFF;

            uart_putc_raw(uart0, IMU_X_HW_ID);
            uart_putc_raw(uart0, imuData.axis);
            uart_putc_raw(uart0, msb);
            uart_putc_raw(uart0, lsb);
            uart_putc_raw(uart0, EOP);
        }
    }
}

void x_adc_task(void *p) {
    adc_t data;
    adc_init();
    adc_gpio_init(ADC_X);

    while (1) {
        adc_select_input(0); // Select ADC input 0 (GPIO26)
        float result = adc_read();

        result = result - 2048;
        result = result / 8;

        if (abs(result) < deadzone) {
            data.val = 0; // resultado pode ser 2 ou -2 (vai ser lido pelo python)
        }else{            
            data.val = 2*result/abs(result); // resultado pode ser 2 ou -2 (vai ser lido pelo python)

        }
        data.axis = 1;
        xQueueSend(xQueueAdc, &data, portMAX_DELAY);
        //printf("X: %d\n", data.val);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void y_adc_task(void *p) {
    adc_t data;
    adc_init();
    adc_gpio_init(ADC_Y);

    while (1) {
        adc_select_input(1); // Select ADC input 1 (GPIO27)
        float result = adc_read();

        result = result - 2048;
        result = result / 8;

        
        if (abs(result) < deadzone) {
            data.val = 0; // resultado pode ser 2 ou -2 (vai ser lido pelo python)
        }else{            
            data.val = 2*result/abs(result); // resultado pode ser 2 ou -2 (vai ser lido pelo python)

        }
        data.axis = 0;
        xQueueSend(xQueueAdc, &data, portMAX_DELAY);
        //printf("Y: %d\n", data.val);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}



void hc06_task(void *p) {
    int connected=1;
    uart_init(HC06_UART_ID, HC06_BAUD_RATE);
    gpio_set_function(HC06_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(HC06_RX_PIN, GPIO_FUNC_UART);
    
    //hc06_init("HC06", "6");

    while (true) {
        //uart_puts(HC06_UART_ID, "OLAAA ");
        if (connected == 0){
            if (hc06_check_connection()){
            printf("Connected\n");
            connected = 1;
            } else {
                printf("Not connected\n");

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

       // printf("Roll %0.1f, Pitch %0.1f, Yaw %0.1f\n", euler.angle.roll, euler.angle.pitch, euler.angle.yaw);
        imuData.val = (int) euler.angle.roll;
        imuData.axis = 1;
     //   printf("Roll %0.1f\n", euler.angle.roll);

        xQueueSend(xQueueIMU, &imuData, portMAX_DELAY);

        imuData.val = (int) euler.angle.yaw;
        imuData.axis = 0;
        //printf("Yaw %0.1f\n", euler.angle.yaw);
        xQueueSend(xQueueIMU, &imuData, portMAX_DELAY);
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void task_tes( void *p) {
    int pino_meio = 14;
    int pino_fim = 15;

    gpio_init(pino_meio);
    gpio_init(pino_fim);
    gpio_set_dir(pino_meio, GPIO_IN);
    gpio_set_dir(pino_fim, GPIO_IN);
    while(1){
        if (gpio_get(pino_meio)){
            printf("Meio\n");
        }
        if (gpio_get(pino_fim)){
            printf("Fim\n");
        }
    }
}
int main() {
    stdio_init_all();

    printf("Start bluetooth task\n");
/*
    xTaskCreate(task_tes, "Task Teste", 4096, NULL, 1, NULL);
    xTaskCreate(mpu6050_task, "MPU6050_Task", 4096, NULL, 1, NULL);
*/
    xQueueAdc = xQueueCreate(32, sizeof(adc_t));
    xQueueIMU = xQueueCreate(32, sizeof(adc_t));
    xTaskCreate(x_adc_task, "ADC_Task 1", 4096, NULL, 1, NULL);
    xTaskCreate(y_adc_task, "ADC_Task 2", 4096, NULL, 1, NULL);
    xTaskCreate(hc06_task, "UART_Task 1", 4096, NULL, 1, NULL);


    vTaskStartScheduler();

    while (true)
        ;
}
