



#include "ULTRASONICO.h"

#define timmer htim4 // reemplazan por el timmer que utilizen

extern TIM_HandleTypeDef timmer;

void ultrasonic_init(void) {
    HAL_TIM_Base_Start(&timmer);
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET); // ponggo el trigger en bajo
}


uint16_t ultrasonic_measure_distance(void) {

    uint32_t pMillis = 0;
    uint32_t Value1 = 0;
    uint32_t Value2 = 0;
    uint16_t Distance = 0;

    // Generar pulso de 10us
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);
    __HAL_TIM_SET_COUNTER(&timmer, 0);
    while (__HAL_TIM_GET_COUNTER(&timmer) < 10); // 10 us
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);

    // Esperar flanco de subida
    pMillis = HAL_GetTick();
    while (!HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN)) {
        if ((HAL_GetTick() - pMillis) > 20) return 0; // timeout
    }
    Value1 = __HAL_TIM_GET_COUNTER(&timmer);

    // Esperar flanco de bajada
    pMillis = HAL_GetTick();
    while (HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN)) {
        if ((HAL_GetTick() - pMillis) > 20) return 0; // timeout
    }
    Value2 = __HAL_TIM_GET_COUNTER(&timmer);

    // Calcular distancia
    Distance = (Value2 - Value1) * 0.034f / 2;

    return Distance;
}
