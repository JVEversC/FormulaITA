/*
 * ============================================================
 * GPS ATGM336H - STM32 Blue Pill (STM32F103C8T6)
 * main.c - projeto STM32CubeIDE
 * ============================================================
 *
 * IMPORTANTE: este arquivo substitui o main.c inteiro do
 * projeto gerado pelo CubeMX. Ele NÃO define:
 *   - HAL_UART_MspInit           -> continua em stm32f1xx_hal_msp.c
 *   - USART1_IRQHandler/USART2_IRQHandler -> continuam em stm32f1xx_it.c
 * Não mexa nesses dois arquivos, eles já foram gerados
 * corretamente pelo CubeMX a partir da configuração das UARTs.
 *
 * Periféricos:
 *   USART2 (PA2 = TX, PA3 = RX) -> módulo GPS ATGM336H, 9600 bps, IRQ habilitada
 *   USART1 (PA9 = TX, PA10 = RX) -> USB-TTL / debug, 115200 bps
 * ============================================================
 */

#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#ifndef PI
#define PI 3.14159265358979323846
#endif

#define EARTH_RADIUS 6371000.0

#define NMEA_BUF_SIZE   100
#define GGA_MAX_FIELDS  15
#define VTG_MAX_FIELDS  12


// ============================================================
// HANDLES DOS PERIFÉRICOS
// ============================================================

UART_HandleTypeDef huart1;   // USB-TTL / debug
UART_HandleTypeDef huart2;   // GPS


// ============================================================
// DADOS DO GPS
// ============================================================

static float latitude = 0.0f;
static float longitude = 0.0f;
static float altitude = 0.0f;

static int satellites = 0;

static float velocity_kmh = 0.0f;
static float velocity_ms = 0.0f;

static float acceleration = 0.0f;

static double distance = 0.0;


// ============================================================
// DADOS ANTERIORES
// ============================================================

static float previousLatitude = 0.0f;
static float previousLongitude = 0.0f;

static float previousVelocity = 0.0f;

static uint32_t previousTime = 0;

static uint8_t firstPosition = 1;
static uint8_t firstVelocity = 1;


// ============================================================
// RECEPÇÃO NMEA (INTERRUPÇÃO)
// ============================================================

static uint8_t  rxByte;
static char     rxBuffer[NMEA_BUF_SIZE];
static uint16_t rxIndex = 0;

static char     sentence[NMEA_BUF_SIZE];
static volatile uint8_t sentenceReady = 0;


// ============================================================
// PROTÓTIPOS (gerados pelo CubeMX)
// ============================================================

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);


// ============================================================
// PROTÓTIPOS (nossos)
// ============================================================

static void processNMEA(char *sentence);
static void processGGA(char *sentence);
static void processVTG(char *sentence);
static void printData(void);

static int   splitFields(char *sentence, char *fields[], int maxFields);
static float convertCoordinate(float coordinate, char direction);
static double calculateDistance(float lat1, float lon1, float lat2, float lon2);
static void debugPrint(const char *msg);


// ============================================================
// MAIN
// ============================================================

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();

    debugPrint("\r\nATGM336H iniciado!\r\n\r\n");

    // Arma a primeira recepção por interrupção
    HAL_UART_Receive_IT(&huart2, &rxByte, 1);

    while (1)
    {
        if (sentenceReady)
        {
            processNMEA(sentence);
            sentenceReady = 0;
        }
    }
}


// ============================================================
// CALLBACK DE RECEPÇÃO (USART2 - GPS)
// ============================================================
// Isso NÃO conflita com stm32f1xx_it.c: o IRQHandler que está
// lá dentro chama HAL_UART_IRQHandler(&huart2), que por sua
// vez chama esta callback quando o byte termina de chegar.

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        char c = (char)rxByte;

        // Começo de uma sentença NMEA
        if (c == '$')
        {
            rxIndex = 0;
            rxBuffer[rxIndex++] = c;
        }
        else if (rxIndex > 0 && rxIndex < (NMEA_BUF_SIZE - 1))
        {
            rxBuffer[rxIndex++] = c;
        }

        // Final da sentença
        if (c == '\n' && rxIndex > 0 && !sentenceReady)
        {
            rxBuffer[rxIndex] = '\0';

            strncpy(sentence, rxBuffer, NMEA_BUF_SIZE - 1);
            sentence[NMEA_BUF_SIZE - 1] = '\0';

            sentenceReady = 1;
            rxIndex = 0;
        }

        // Rearma a recepção do próximo byte
        HAL_UART_Receive_IT(&huart2, &rxByte, 1);
    }
}


// ============================================================
// PROCESSAMENTO NMEA
// ============================================================

static void processNMEA(char *sentence)
{
    if (strncmp(sentence, "$GNGGA", 6) == 0)
    {
        processGGA(sentence);
    }
    else if (strncmp(sentence, "$GNVTG", 6) == 0)
    {
        processVTG(sentence);
    }
}


// ============================================================
// SEPARA OS CAMPOS DA SENTENÇA
// ============================================================
// Modifica a string original, trocando cada ',' por '\0' e
// guardando o ponteiro de início de cada campo em fields[].
// Preserva campos vazios (",," -> "" no meio).

static int splitFields(char *sentence, char *fields[], int maxFields)
{
    int field = 0;

    fields[field++] = sentence;

    for (char *p = sentence; *p != '\0'; p++)
    {
        if (*p == ',')
        {
            *p = '\0';

            if (field < maxFields)
            {
                fields[field++] = p + 1;
            }
        }

        if (*p == '\r' || *p == '\n')
        {
            *p = '\0';
        }
    }

    return field;
}


// ============================================================
// PROCESSA GGA
// ============================================================

static void processGGA(char *sentence)
{
    char *fields[GGA_MAX_FIELDS] = {0};

    int fieldCount = splitFields(sentence, fields, GGA_MAX_FIELDS);

    if (fieldCount <= 9)
    {
        return;
    }


    // --------------------------------------------------------
    // Verifica FIX
    // --------------------------------------------------------

    int fixQuality = atoi(fields[6]);

    if (fixQuality == 0)
    {
        debugPrint("Sem sinal de SAT.\r\n");
        return;
    }


    // --------------------------------------------------------
    // Satélites
    // --------------------------------------------------------

    satellites = atoi(fields[7]);


    // --------------------------------------------------------
    // Latitude
    // --------------------------------------------------------

    float rawLatitude = atof(fields[2]);

    char latDirection = (fields[3][0] != '\0') ? fields[3][0] : 'N';

    latitude = convertCoordinate(rawLatitude, latDirection);


    // --------------------------------------------------------
    // Longitude
    // --------------------------------------------------------

    float rawLongitude = atof(fields[4]);

    char lonDirection = (fields[5][0] != '\0') ? fields[5][0] : 'E';

    longitude = convertCoordinate(rawLongitude, lonDirection);


    // --------------------------------------------------------
    // Altitude
    // --------------------------------------------------------

    altitude = atof(fields[9]);


    // ========================================================
    // DISTÂNCIA
    // ========================================================

    if (!firstPosition)
    {
        double deltaDistance = calculateDistance(
            previousLatitude,
            previousLongitude,
            latitude,
            longitude
        );

        if (deltaDistance < 100.0)
        {
            distance += deltaDistance;
        }
    }

    previousLatitude = latitude;
    previousLongitude = longitude;

    firstPosition = 0;
}


// ============================================================
// PROCESSA VTG
// ============================================================

static void processVTG(char *sentence)
{
    char *fields[VTG_MAX_FIELDS] = {0};

    int fieldCount = splitFields(sentence, fields, VTG_MAX_FIELDS);

    if (fieldCount <= 7 || fields[7][0] == '\0')
    {
        return;
    }


    // --------------------------------------------------------
    // Velocidade em km/h
    // --------------------------------------------------------

    float newVelocityKmh = atof(fields[7]);
    float newVelocityMs  = newVelocityKmh / 3.6f;


    // ========================================================
    // ACELERAÇÃO
    // ========================================================

    uint32_t currentTime = HAL_GetTick();

    if (!firstVelocity)
    {
        float dt = (currentTime - previousTime) / 1000.0f;

        if (dt > 0.0f)
        {
            float rawAcceleration = (newVelocityMs - previousVelocity) / dt;

            const float alpha = 0.2f;

            acceleration = alpha * rawAcceleration
                          + (1.0f - alpha) * acceleration;
        }
    }

    previousVelocity = newVelocityMs;
    previousTime = currentTime;

    firstVelocity = 0;

    velocity_kmh = newVelocityKmh;
    velocity_ms  = newVelocityMs;


    // ========================================================
    // MOSTRA TUDO
    // ========================================================

    printData();
}


// ============================================================
// MOSTRA DADOS (via USART1 / USB-TTL)
// ============================================================

static void printData(void)
{
    char buf[80];

    debugPrint("\r\n--------------------------------\r\n");

    snprintf(buf, sizeof(buf), "Latitude: %.6f deg\r\n", latitude);
    debugPrint(buf);

    snprintf(buf, sizeof(buf), "Longitude: %.6f deg\r\n", longitude);
    debugPrint(buf);

    snprintf(buf, sizeof(buf), "Altitude: %.2f m\r\n", altitude);
    debugPrint(buf);

    snprintf(buf, sizeof(buf), "Satelites: %d\r\n", satellites);
    debugPrint(buf);

    snprintf(buf, sizeof(buf), "Velocidade: %.2f km/h\r\n", velocity_kmh);
    debugPrint(buf);

    snprintf(buf, sizeof(buf), "Velocidade: %.2f m/s\r\n", velocity_ms);
    debugPrint(buf);

    snprintf(buf, sizeof(buf), "Distancia: %.2f m\r\n", distance);
    debugPrint(buf);

    snprintf(buf, sizeof(buf), "Aceleracao: %.3f m/s^2\r\n", acceleration);
    debugPrint(buf);

    debugPrint("--------------------------------\r\n");
}


// ============================================================
// ENVIA STRING PELA USART1
// ============================================================

static void debugPrint(const char *msg)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}


// ============================================================
// CONVERSÃO NMEA -> GRAUS DECIMAIS
// ============================================================

static float convertCoordinate(float coordinate, char direction)
{
    int degrees = (int)(coordinate / 100.0f);

    float minutes = coordinate - degrees * 100.0f;

    float decimal = degrees + minutes / 60.0f;

    if (direction == 'S' || direction == 'W')
    {
        decimal *= -1.0f;
    }

    return decimal;
}


// ============================================================
// DISTÂNCIA HAVERSINE
// ============================================================

static double calculateDistance(float lat1, float lon1, float lat2, float lon2)
{
    double lat1Rad = lat1 * PI / 180.0;
    double lat2Rad = lat2 * PI / 180.0;

    double deltaLat = (lat2 - lat1) * PI / 180.0;
    double deltaLon = (lon2 - lon1) * PI / 180.0;

    double a = sin(deltaLat / 2.0) * sin(deltaLat / 2.0)
             + cos(lat1Rad) * cos(lat2Rad)
             * sin(deltaLon / 2.0) * sin(deltaLon / 2.0);

    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));

    return EARTH_RADIUS * c;
}


// ============================================================
// CLOCK: HSE 8 MHz -> 72 MHz via PLL (padrão do Blue Pill)
// ============================================================

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                 | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}


// ============================================================
// GPIO
// ============================================================

static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();
}


// ============================================================
// USART1 - USB-TTL / DEBUG (PA9 = TX, PA10 = RX), 115200 bps
// ============================================================

static void MX_USART1_UART_Init(void)
{
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
}


// ============================================================
// USART2 - GPS (PA2 = TX, PA3 = RX), 9600 bps
// ============================================================

static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 9600;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
}


// ============================================================
// ERROR HANDLER
// ============================================================

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}
