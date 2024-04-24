#include "hc05.h"

bool hc05_check_connection() {
    char str[32];
    int i = 0;
    uart_puts(hc05_UART_ID, "AT\r\n");
    while (uart_is_readable_within_us(hc05_UART_ID, 1000)) {
        str[i++] = uart_getc(hc05_UART_ID);
    }
    str[i] = '\0';

    if (strstr(str, "OK") > 0)
        return true;
    else
        return false;
}

bool hc05_set_name(char name[]) {
    char str[32];
    int i = 0;

    sprintf(str, "AT+NAME=%s\r\n", name);
    uart_puts(hc05_UART_ID, str);
    while (uart_is_readable_within_us(hc05_UART_ID, 1000)) {
        str[i++] = uart_getc(hc05_UART_ID);
    }
    str[i] = '\0';

    if (strstr(str, "OK") > 0)
        return true;
    else
        return false;
}

bool hc05_set_pin(char pin[]) {
    char str[32];
    int i = 0;

    sprintf(str, "AT+PSWD=\"%s\"\r\n", pin);
    uart_puts(hc05_UART_ID, str);
    while (uart_is_readable_within_us(hc05_UART_ID, 1000)) {
        str[i++] = uart_getc(hc05_UART_ID);
    }
    str[i] = '\0';

    if (strstr(str, "OK") > 0)
        return true;
    else
        return false;
}

bool hc05_set_at_mode(int on){
    gpio_put(hc05_PIN, on);
}

bool hc05_init(char name[], char pin[]) {
    hc05_set_at_mode(1);
    printf("check connection\n");
    while (hc05_check_connection() == false) {
        printf("not connected\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
    printf("Connected \n");

    
    printf("set name\n");
    while (hc05_set_name(name) == false) {
        printf("set name failed\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
    printf("name OK\n");

    
    printf("set pin\n");
    while (hc05_set_pin(pin) == false) {
        printf("set pin failed\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
    printf("pin OK\n");
    hc05_set_at_mode(0);
}
