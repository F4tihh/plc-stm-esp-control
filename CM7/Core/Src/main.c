/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lan8742.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "lwip/sockets.h"
#include "lwip/netif.h"
#include "lwip/ip4_addr.h"
#include "lwip/etharp.h"
#include "stm32h7xx_hal_eth.h"

extern volatile uint32_t eth_pbuf_count;
extern volatile uint32_t eth_input_ok_count;
extern volatile uint32_t eth_input_fail_count;
extern volatile uint32_t eth_rx_irq_count;
extern volatile uint32_t eth_tx_irq_count;
extern volatile uint32_t eth_err_irq_count;

extern ETH_HandleTypeDef heth;
extern lan8742_Object_t LAN8742;
extern struct netif gnetif;
/* USER CODE END Includes */

/* USER CODE BEGIN PD */
/* #define DUAL_CORE_BOOT_SYNC_SEQUENCE */

#define PLC_IP_ADDR      "192.168.1.100"
#define PLC_MODBUS_PORT  502
#define PLC_UNIT_ID      1

#define HR_SERVO_HIZ          0   /* 40001 */
#define HR_SERVO_HIZ_GELEN    1   /* 40002 */
#define HR_SERVO_HIZ_GIDEN    2   /* 40003 */
#define HR_SERVO_TORK         3   /* 40004 */
#define HR_SERVO_TORK_GELEN   4   /* 40005 */
#define HR_SERVO_TORK_GIDEN   5   /* 40006 */
#define HR_SERVO_MEVCUT_HIZ   6   /* 40007 */

/* GMSuite tablosunda: EKRAN_STOP=2, EKRAN_START=3, SISTEM_STARTLI=4.
   Modbus genelde 1-based gösterir, gerçek coil adresi = görünen adres - 1. */
#define COIL_EKRAN_STOP       1
#define COIL_EKRAN_START      2
#define COIL_SISTEM_STARTLI   3

#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U)
#endif
#endif
/* USER CODE END PD */

UART_HandleTypeDef huart3;

osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 4096,
  .priority = (osPriority_t) osPriorityNormal,
};

void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
void uart_send_text(const char *text);
void modbus_read_all_registers(void);
void modbus_motion_test_once(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void uart_send_text(const char *text)
{
  HAL_UART_Transmit(&huart3, (uint8_t *)text, strlen(text), HAL_MAX_DELAY);
}

static void CPU_CACHE_Enable(void)
{
  // Test asamasinda cache kapali
  // SCB_EnableICache();
  // SCB_EnableDCache();
}

static int modbus_open_socket(void)
{
  int sock;
  struct sockaddr_in server;

  sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock < 0)
  {
    uart_send_text("SOCKET FAIL\r\n");
    return -1;
  }

  memset(&server, 0, sizeof(server));
  server.sin_family = AF_INET;
  server.sin_port = htons(PLC_MODBUS_PORT);
  server.sin_addr.s_addr = inet_addr(PLC_IP_ADDR);

  if (connect(sock, (struct sockaddr *)&server, sizeof(server)) < 0)
  {
    uart_send_text("PLC CONNECT FAIL\r\n");
    closesocket(sock);
    return -1;
  }

  return sock;
}

static int modbus_write_single_register(uint16_t reg_addr, uint16_t value)
{
  int sock = modbus_open_socket();
  if (sock < 0) return -1;

  uint8_t req[12];
  uint8_t resp[32];
  char msg[128];

  req[0] = 0x00; req[1] = 0x10;
  req[2] = 0x00; req[3] = 0x00;
  req[4] = 0x00; req[5] = 0x06;
  req[6] = PLC_UNIT_ID;
  req[7] = 0x06;
  req[8] = (reg_addr >> 8) & 0xFF;
  req[9] = reg_addr & 0xFF;
  req[10] = (value >> 8) & 0xFF;
  req[11] = value & 0xFF;

  int sent = send(sock, req, sizeof(req), 0);
  if (sent <= 0)
  {
    uart_send_text("WRITE REG SEND FAIL\r\n");
    closesocket(sock);
    return -1;
  }

  int len = recv(sock, resp, sizeof(resp), 0);
  closesocket(sock);

  snprintf(msg, sizeof(msg), "WRITE REG addr=%u value=%u recv=%d\r\n", reg_addr, value, len);
  uart_send_text(msg);

  if (len >= 12 && resp[7] == 0x06) return 0;
  return -1;
}

static int modbus_write_single_coil(uint16_t coil_addr, uint8_t state)
{
  int sock = modbus_open_socket();
  if (sock < 0) return -1;

  uint8_t req[12];
  uint8_t resp[32];
  char msg[128];

  req[0] = 0x00; req[1] = 0x20;
  req[2] = 0x00; req[3] = 0x00;
  req[4] = 0x00; req[5] = 0x06;
  req[6] = PLC_UNIT_ID;
  req[7] = 0x05;
  req[8] = (coil_addr >> 8) & 0xFF;
  req[9] = coil_addr & 0xFF;
  req[10] = state ? 0xFF : 0x00;
  req[11] = 0x00;

  int sent = send(sock, req, sizeof(req), 0);
  if (sent <= 0)
  {
    uart_send_text("WRITE COIL SEND FAIL\r\n");
    closesocket(sock);
    return -1;
  }

  int len = recv(sock, resp, sizeof(resp), 0);
  closesocket(sock);

  snprintf(msg, sizeof(msg), "WRITE COIL addr=%u state=%u recv=%d\r\n", coil_addr, state, len);
  uart_send_text(msg);

  if (len >= 12 && resp[7] == 0x05) return 0;
  return -1;
}

static void plc_start(void)
{
  uart_send_text("PLC START veriliyor\r\n");
  modbus_write_single_coil(COIL_EKRAN_START, 1);
  osDelay(100);
  modbus_write_single_coil(COIL_SISTEM_STARTLI, 1);
}

static void plc_stop(void)
{
  uart_send_text("PLC STOP veriliyor\r\n");
  modbus_write_single_register(HR_SERVO_HIZ_GIDEN, 0);
  modbus_write_single_register(HR_SERVO_TORK_GIDEN, 0);
  osDelay(100);
  modbus_write_single_coil(COIL_SISTEM_STARTLI, 0);
  osDelay(100);
  modbus_write_single_coil(COIL_EKRAN_START, 0);
}

static void plc_set_speed_torque(int16_t speed, uint16_t torque)
{
  uint16_t speed_u16 = (uint16_t)speed;
  char msg[128];

  snprintf(msg, sizeof(msg), "SET speed=%d torque=%u\r\n", speed, torque);
  uart_send_text(msg);

  modbus_write_single_register(HR_SERVO_TORK, torque);
  osDelay(100);
  modbus_write_single_register(HR_SERVO_HIZ, speed_u16);
  osDelay(100);

  modbus_write_single_register(HR_SERVO_TORK_GIDEN, torque);
  osDelay(100);
  modbus_write_single_register(HR_SERVO_HIZ_GIDEN, speed_u16);
}

void modbus_motion_test_once(void)
{
  uart_send_text("\r\n===== MOTION TEST BASLADI =====\r\n");

  uart_send_text("BASLANGIC TEMIZLIK\r\n");
  plc_stop();
  osDelay(1500);

  plc_start();
  osDelay(500);

  uart_send_text("ILERI/SARMA: 5 saniye\r\n");
  plc_set_speed_torque(200, 30);
  osDelay(5000);

  uart_send_text("DUR: 2 saniye\r\n");
  plc_set_speed_torque(0, 0);
  osDelay(2000);

  uart_send_text("GERI TEST: 5 saniye\r\n");

  /*
    Negatif hiz yerine pozitif hiz deniyoruz.
    Cunku PLC ladder negatif hizi kabul etmiyor veya hemen sifirliyor olabilir.
  */
  plc_set_speed_torque(200, 30);
  osDelay(5000);

  uart_send_text("FINAL STOP\r\n");
  plc_stop();

  uart_send_text("===== MOTION TEST BITTI =====\r\n\r\n");
}

void modbus_read_all_registers(void)
{
  int sock;
  struct sockaddr_in server;
  uint8_t request[12];
  uint8_t response[128];
  char msg[256];

  sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock < 0)
  {
    uart_send_text("SOCKET FAIL\r\n");
    return;
  }

  memset(&server, 0, sizeof(server));
  server.sin_family = AF_INET;
  server.sin_port = htons(PLC_MODBUS_PORT);
  server.sin_addr.s_addr = inet_addr(PLC_IP_ADDR);

  if (connect(sock, (struct sockaddr*)&server, sizeof(server)) < 0)
  {
    uart_send_text("PLC CONNECT FAIL\r\n");
    closesocket(sock);
    return;
  }

  request[0] = 0x00; request[1] = 0x01;
  request[2] = 0x00; request[3] = 0x00;
  request[4] = 0x00; request[5] = 0x06;
  request[6] = PLC_UNIT_ID;
  request[7] = 0x03;
  request[8]  = 0x00;
  request[9]  = 0x00;
  request[10] = 0x00;
  request[11] = 0x07;

  int sent = send(sock, request, 12, 0);

  if (sent <= 0)
  {
    uart_send_text("MODBUS SEND FAIL\r\n");
    closesocket(sock);
    return;
  }

  int len = recv(sock, response, sizeof(response), 0);

  if (len >= 23 && response[7] == 0x03)
  {
    uint16_t r40001 = ((uint16_t)response[9]  << 8) | response[10];
    uint16_t r40002 = ((uint16_t)response[11] << 8) | response[12];
    uint16_t r40003 = ((uint16_t)response[13] << 8) | response[14];
    uint16_t r40004 = ((uint16_t)response[15] << 8) | response[16];
    uint16_t r40005 = ((uint16_t)response[17] << 8) | response[18];
    uint16_t r40006 = ((uint16_t)response[19] << 8) | response[20];
    uint16_t r40007 = ((uint16_t)response[21] << 8) | response[22];

    uart_send_text("\r\n===== PLC MODBUS READ =====\r\n");
    snprintf(msg, sizeof(msg), "40001 SERVO_HIZ        : %u\r\n", r40001); uart_send_text(msg);
    snprintf(msg, sizeof(msg), "40002 Servo_hiz_gelen  : %u\r\n", r40002); uart_send_text(msg);
    snprintf(msg, sizeof(msg), "40003 Servo_hiz_giden  : %u\r\n", r40003); uart_send_text(msg);
    snprintf(msg, sizeof(msg), "40004 SERVO_TORKU      : %u\r\n", r40004); uart_send_text(msg);
    snprintf(msg, sizeof(msg), "40005 Servo_tork_gelen : %u\r\n", r40005); uart_send_text(msg);
    snprintf(msg, sizeof(msg), "40006 Servo_tork_giden : %u\r\n", r40006); uart_send_text(msg);
    snprintf(msg, sizeof(msg), "40007 Servo_mevcut_hiz : %u\r\n", r40007); uart_send_text(msg);
    uart_send_text("===========================\r\n");
  }
  else
  {
    snprintf(msg, sizeof(msg), "MODBUS RECV HATA / LEN: %d\r\n", len);
    uart_send_text(msg);
  }

  closesocket(sock);
}

/* USER CODE END 0 */

int main(void)
{
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  int32_t timeout;
#endif

  MPU_Config();
  //CPU_CACHE_Enable();

#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if (timeout < 0) { Error_Handler(); }
#endif

  HAL_Init();
  SystemClock_Config();

#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  __HAL_RCC_HSEM_CLK_ENABLE();
  HAL_HSEM_FastTake(HSEM_ID_0);
  HAL_HSEM_Release(HSEM_ID_0, 0);
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
  if (timeout < 0) { Error_Handler(); }
#endif

  MX_GPIO_Init();
  MX_USART3_UART_Init();

  uart_send_text("\r\n\r\nSTM32 guc verdi\r\n");

  osKernelInitialize();
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  if (defaultTaskHandle == NULL) uart_send_text("TASK CREATE FAIL\r\n");
  else uart_send_text("TASK CREATE OK\r\n");

  osKernelStart();

  while (1) {}
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) Error_Handler();
}

static void MX_USART3_UART_Init(void)
{
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  if (HAL_UART_Init(&huart3) != HAL_OK) Error_Handler();
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) Error_Handler();
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) Error_Handler();
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
}

void StartDefaultTask(void *argument)
{
  char msg[128];
  uint32_t phy_bsr = 0;
  uint8_t link_now = 0;
  uint8_t link_prev = 0xFF;
  uint8_t motion_done = 0;

  uart_send_text("TASK BASLADI\r\n");
  uart_send_text("LWIP init giriyor\r\n");

  MX_LWIP_Init();

  uart_send_text("LWIP init cikti\r\n");
  uart_send_text("LWIP init tamam\r\n");

  snprintf(msg, sizeof(msg),
           "STM MAC: %02X:%02X:%02X:%02X:%02X:%02X\r\n",
           gnetif.hwaddr[0], gnetif.hwaddr[1], gnetif.hwaddr[2],
           gnetif.hwaddr[3], gnetif.hwaddr[4], gnetif.hwaddr[5]);
  uart_send_text(msg);

  osDelay(3000);

  if (!motion_done)
  {
    motion_done = 1;
    modbus_motion_test_once();
  }

  for(;;)
  {
    if (HAL_ETH_ReadPHYRegister(&heth, 0, 0x01, &phy_bsr) != HAL_OK)
      uart_send_text("PHY reg read hata\r\n");

    link_now = (phy_bsr & (1 << 2)) ? 1 : 0;

    if (link_now != link_prev)
    {
      if (link_now) uart_send_text("LINK DEGISTI -> UP\r\n");
      else uart_send_text("LINK DEGISTI -> DOWN\r\n");
      link_prev = link_now;
    }

    snprintf(msg, sizeof(msg), "PHY BSR (0x01): 0x%04lX\r\n", phy_bsr); uart_send_text(msg);
    if (link_now) uart_send_text("BSR link bit: 1\r\n"); else uart_send_text("BSR link bit: 0\r\n");
    if (netif_is_link_up(&gnetif)) uart_send_text("ETH LINK UP\r\n"); else uart_send_text("ETH LINK DOWN\r\n");
    if (netif_is_up(&gnetif)) uart_send_text("NETIF UP\r\n"); else uart_send_text("NETIF DOWN\r\n");

    snprintf(msg, sizeof(msg), "RX IRQ: %lu  TX IRQ: %lu  ERR IRQ: %lu\r\n",
             eth_rx_irq_count, eth_tx_irq_count, eth_err_irq_count); uart_send_text(msg);

    snprintf(msg, sizeof(msg), "PBUF: %lu  INPUT_OK: %lu  INPUT_FAIL: %lu\r\n",
             eth_pbuf_count, eth_input_ok_count, eth_input_fail_count); uart_send_text(msg);

    snprintf(msg, sizeof(msg), "IP: %s\r\n", ip4addr_ntoa(netif_ip4_addr(&gnetif))); uart_send_text(msg);
    uart_send_text("---------------------\r\n");

    modbus_read_all_registers();
    osDelay(1000);
  }
}

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};
  HAL_MPU_Disable();

  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
