/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
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
#include <errno.h>
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

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
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

#define HR_SERVO_MESAFE            (40311 - 40001)   // 310
#define HR_EKRAN_SERVO_SABIT_HIZ   (40313 - 40001)   // 312
#define HR_EKRAN_SERVO_SABIT_TORK  (40314 - 40001)   // 313

#define HR_GERCEK_METRE_RAW   (42011 - 40001)   // 2010, Real, 2 register

#define COIL_EKRAN_METRE_AKTIF     (5 - 1)           // 4
#define COIL_EKRAN_YON_SECIMI      (6 - 1)           // 5
#define COIL_METRE_SIFIRLAMA       (7 - 1)           // 6
#define COIL_EKRAN_DEGISKEN_MOD    (8 - 1)           // 7
#define COIL_ISLEM_BITTI           (11 - 1)          // 10

/* GMSuite tablosunda: EKRAN_STOP=2, EKRAN_START=3, SISTEM_STARTLI=4.
   Modbus genelde 1-based gösterir, gerçek coil adresi = görünen adres - 1. */
#define COIL_EKRAN_STOP       1
#define COIL_EKRAN_START      2
#define COIL_SISTEM_STARTLI   3

/* Set 1 only when debugging raw UART2 traffic (very chatty on USART3). */
#define ESP_UART_BYTE_TRACE   0

/* Modbus TCP response wait (ms). recv() returns -1 on timeout instead of blocking forever. */
#define MODBUS_RECV_TIMEOUT_MS 2000

/* Max wait for TCP connect (non-blocking connect + select). Avoids indefinite block when PLC is off. */
#define MODBUS_CONNECT_TIMEOUT_MS  4000

/* Minimum gap between identical UART alerts (ms) — avoids flooding USART3 when PLC is down. */
#define MODBUS_UART_PLC_FAIL_MIN_MS   4000
#define MODBUS_UART_ETH_DOWN_MIN_MS   5000
#define MODBUS_UART_SOCKET_FAIL_MIN_MS 8000
#define MODBUS_UART_IO_FAIL_MIN_MS    4000

/* Main loop: longer delay after repeated Modbus read failures (back off TCP retries). */
#define MODBUS_LOOP_DELAY_OK_MS       1000
#define MODBUS_LOOP_DELAY_FAIL_MS     2500
#define MODBUS_LOOP_FAIL_STREAK_LONG  4U

/* Print PHY/netif/counter/IP banner only every N loop iterations (Modbus read still each cycle). */
#define DEFAULT_TASK_VERBOSE_INTERVAL  5

#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U)
#endif
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 4096,
  .priority = (osPriority_t) osPriorityNormal,
};

/* USER CODE BEGIN PV */

static int current_speed = 100;
static int current_torque = 5;

static uint8_t esp_rx_byte;
static char esp_cmd_buffer[32];
static uint8_t esp_cmd_index = 0;

static uint32_t modbus_uart_last_plc_fail_ms;
static uint32_t modbus_uart_last_eth_down_ms;
static uint32_t modbus_uart_last_sock_fail_ms;
static uint32_t modbus_uart_last_io_fail_ms;

/* Gate START command to avoid overlapping sequences */
static volatile uint8_t plc_busy = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
void uart_send_text(const char *text);
int modbus_read_all_registers(void);
void modbus_motion_test_once(void);
int plc_read_one_holding_register(uint16_t raw_reg_addr, uint16_t *out_value);
int plc_read_one_coil(uint16_t raw_coil_addr, uint8_t *out_state);
void plc_diagnostic_read_all(void);
void plc_reset_meter(void);
void plc_set_fixed_speed_torque_distance(uint16_t fixed_speed, uint16_t fixed_torque, uint16_t distance);
void plc_start_fixed_mode_from_esp(void);
void plc_stop_from_esp(void);
static int modbus_write_single_register(uint16_t reg_addr,
                                        uint16_t value);

static int modbus_eth_link_ready(void);

static int plc_read_float_holding(uint16_t raw_reg_addr, float *out_value);
static void telemetry_send_meter_to_esp(void);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

static void plc_apply_live_torque(void)
{
  char msg[128];

  snprintf(msg, sizeof(msg),
           "LIVE TORQUE UPDATE -> TORQUE=%d\r\n",
           current_torque);
  uart_send_text(msg);

  if (!modbus_eth_link_ready())
  {
    uart_send_text("TORQUE UPDATE FAIL -> ETH/PLC LINK DOWN\r\n");
    return;
  }

  if (modbus_write_single_register(HR_EKRAN_SERVO_SABIT_TORK,
                                   current_torque) == 0)
  {
    uart_send_text("TORQUE UPDATE SENT\r\n");
  }
  else
  {
    uart_send_text("TORQUE UPDATE FAIL\r\n");
  }
}

static void plc_apply_live_speed(void)
{
  char msg[128];

  snprintf(msg, sizeof(msg),
           "LIVE SPEED UPDATE -> SPEED=%d\r\n",
           current_speed);
  uart_send_text(msg);

  if (!modbus_eth_link_ready())
  {
    uart_send_text("SPEED UPDATE FAIL -> ETH/PLC LINK DOWN\r\n");
    return;
  }

  if (modbus_write_single_register(HR_EKRAN_SERVO_SABIT_HIZ,
                                   current_speed) == 0)
  {
    uart_send_text("SPEED UPDATE SENT\r\n");
  }
  else
  {
    uart_send_text("SPEED UPDATE FAIL\r\n");
  }
}

void uart_send_text(const char *text)
{
  HAL_UART_Transmit(&huart3, (uint8_t *)text, strlen(text), HAL_MAX_DELAY);
}

static void esp_send_text(const char *text)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)text, strlen(text), HAL_MAX_DELAY);
}

static void CPU_CACHE_Enable(void)
{
  // Test asamasinda cache kapali
  // SCB_EnableICache();
  // SCB_EnableDCache();
}

static int modbus_eth_link_ready(void)
{
  return (netif_is_link_up(&gnetif) && netif_is_up(&gnetif)) ? 1 : 0;
}

static void modbus_uart_plc_fail_throttled(void)
{
  uint32_t t = osKernelGetTickCount();
  if ((t - modbus_uart_last_plc_fail_ms) >= MODBUS_UART_PLC_FAIL_MIN_MS)
  {
    modbus_uart_last_plc_fail_ms = t;
    uart_send_text("PLC CONNECT FAIL\r\n");
  }
}

static void modbus_uart_eth_down_throttled(void)
{
  uint32_t t = osKernelGetTickCount();
  if ((t - modbus_uart_last_eth_down_ms) >= MODBUS_UART_ETH_DOWN_MIN_MS)
  {
    modbus_uart_last_eth_down_ms = t;
    uart_send_text("MODBUS: ETH link/netif down\r\n");
  }
}

static void modbus_uart_socket_fail_throttled(void)
{
  uint32_t t = osKernelGetTickCount();
  if ((t - modbus_uart_last_sock_fail_ms) >= MODBUS_UART_SOCKET_FAIL_MIN_MS)
  {
    modbus_uart_last_sock_fail_ms = t;
    uart_send_text("SOCKET FAIL\r\n");
  }
}

static void modbus_uart_io_fail_throttled(void)
{
  uint32_t t = osKernelGetTickCount();
  if ((t - modbus_uart_last_io_fail_ms) >= MODBUS_UART_IO_FAIL_MIN_MS)
  {
    modbus_uart_last_io_fail_ms = t;
    uart_send_text("MODBUS IO fail\r\n");
  }
}

static int modbus_open_socket(void)
{
  int sock;
  int conn_rc;
  int sel;
  struct sockaddr_in server;
  fd_set wfds;
  struct timeval tvsel;
  struct timeval tvrecv;
  unsigned long nb = 1U;
  unsigned long zb = 0U;
  socklen_t solen;
  int soerr;

  if (!modbus_eth_link_ready())
  {
    modbus_uart_eth_down_throttled();
    return -1;
  }

  sock = socket(AF_INET, SOCK_STREAM, 0);

  if (sock < 0)
  {
      modbus_uart_socket_fail_throttled();
      return -1;
  }

  struct timeval tv_short;
  tv_short.tv_sec = 1;
  tv_short.tv_usec = 0;

  setsockopt(sock,
             SOL_SOCKET,
             SO_RCVTIMEO,
             &tv_short,
             sizeof(tv_short));

  setsockopt(sock,
             SOL_SOCKET,
             SO_SNDTIMEO,
             &tv_short,
             sizeof(tv_short));

  memset(&server, 0, sizeof(server));
  server.sin_family = AF_INET;
  server.sin_port = htons(PLC_MODBUS_PORT);
  server.sin_addr.s_addr = inet_addr(PLC_IP_ADDR);

  if (ioctlsocket(sock, FIONBIO, &nb) != 0)
  {
    (void)closesocket(sock);
    return -1;
  }

  conn_rc = connect(sock, (struct sockaddr *)&server, sizeof(server));
  if (conn_rc < 0)
  {
    if ((errno != EINPROGRESS) && (errno != EWOULDBLOCK))
    {
      modbus_uart_plc_fail_throttled();
      (void)closesocket(sock);
      return -1;
    }

    FD_ZERO(&wfds);
    FD_SET(sock, &wfds);
    tvsel.tv_sec = MODBUS_CONNECT_TIMEOUT_MS / 1000;
    tvsel.tv_usec = (MODBUS_CONNECT_TIMEOUT_MS % 1000) * 1000;
    sel = select(sock + 1, NULL, &wfds, NULL, &tvsel);
    if (sel <= 0)
    {
      modbus_uart_plc_fail_throttled();
      (void)closesocket(sock);
      return -1;
    }
  }

  solen = (socklen_t)sizeof(soerr);
  soerr = 0;
  if (getsockopt(sock, SOL_SOCKET, SO_ERROR, &soerr, &solen) != 0)
  {
    modbus_uart_plc_fail_throttled();
    (void)closesocket(sock);
    return -1;
  }
  if (soerr != 0)
  {
    modbus_uart_plc_fail_throttled();
    (void)closesocket(sock);
    return -1;
  }

  if (ioctlsocket(sock, FIONBIO, &zb) != 0)
  {
    (void)closesocket(sock);
    return -1;
  }

  tvrecv.tv_sec = MODBUS_RECV_TIMEOUT_MS / 1000;
  tvrecv.tv_usec = (MODBUS_RECV_TIMEOUT_MS % 1000) * 1000;
  (void)setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tvrecv, (socklen_t)sizeof(tvrecv));

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
    modbus_uart_io_fail_throttled();
    (void)closesocket(sock);
    return -1;
  }

  int len = recv(sock, resp, sizeof(resp), 0);

  if (len <= 0)
  {
    uart_send_text("MODBUS WRITE RECV TIMEOUT\r\n");

    (void)closesocket(sock);

    return -1;
  }

  (void)closesocket(sock);

  if (len >= 12 && resp[7] == 0x06)
  {
    snprintf(msg, sizeof(msg),
             "WRITE REG addr=%u value=%u recv=%d\r\n",
             reg_addr,
             value,
             len);

    uart_send_text(msg);

    return 0;
  }

  modbus_uart_io_fail_throttled();
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
    modbus_uart_io_fail_throttled();
    (void)closesocket(sock);
    return -1;
  }

  int len = recv(sock, resp, sizeof(resp), 0);

  if (len <= 0)
  {
    uart_send_text("MODBUS COIL RECV TIMEOUT\r\n");

    (void)closesocket(sock);

    return -1;
  }

  (void)closesocket(sock);

  if (len >= 12 && resp[7] == 0x05)
  {
    snprintf(msg, sizeof(msg),
             "WRITE COIL addr=%u state=%u recv=%d\r\n",
             coil_addr,
             state,
             len);

    uart_send_text(msg);

    return 0;
  }

  modbus_uart_io_fail_throttled();
  return -1;
}

static int plc_read_float_holding(uint16_t raw_reg_addr, float *out_value)
{
  int sock;
  uint8_t request[12];
  uint8_t response[32];

  if (out_value == NULL)
    return -1;

  sock = modbus_open_socket();
  if (sock < 0)
    return -1;

  request[0] = 0x00; request[1] = 0x41;
  request[2] = 0x00; request[3] = 0x00;
  request[4] = 0x00; request[5] = 0x06;
  request[6] = PLC_UNIT_ID;
  request[7] = 0x03;
  request[8]  = (raw_reg_addr >> 8) & 0xFF;
  request[9]  = raw_reg_addr & 0xFF;
  request[10] = 0x00;
  request[11] = 0x02;   // Real = 2 register

  int sent = send(sock, request, sizeof(request), 0);
  if (sent <= 0)
  {
    (void)closesocket(sock);
    return -1;
  }

  int len = recv(sock, response, sizeof(response), 0);

  if (len <= 0)
  {
    (void)closesocket(sock);
    return -1;
  }

  (void)closesocket(sock);

  if (len >= 13 && response[7] == 0x03 && response[8] == 0x04)
  {
    uint16_t w1 = ((uint16_t)response[9]  << 8) | response[10];
    uint16_t w2 = ((uint16_t)response[11] << 8) | response[12];

    uint32_t raw_swapped = ((uint32_t)w2 << 16) | w1;

    float val;
    memcpy(&val, &raw_swapped, sizeof(float));

    *out_value = val;
    return 0;
  }

  return -1;
}

static void telemetry_send_meter_to_esp(void)
{
  float metre = 0.0f;
  char msg[64];

  if (plc_read_float_holding(HR_GERCEK_METRE_RAW, &metre) == 0)
  {
    int metre_x100 = (int)(metre * 100.0f);

    snprintf(msg, sizeof(msg),
             "MTR,%d.%02d\r\n",
             metre_x100 / 100,
             metre_x100 % 100);

    esp_send_text(msg);
  }
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

  // 1) Önce hareketi durduracak bitleri kapat
  modbus_write_single_coil(COIL_EKRAN_START, 0);
  osDelay(150);

  modbus_write_single_coil(COIL_SISTEM_STARTLI, 0);
  osDelay(150);

  // 2) Sonra hız/tork komutlarını sıfırla
  modbus_write_single_register(HR_SERVO_HIZ_GIDEN, 0);
  osDelay(150);

  modbus_write_single_register(HR_SERVO_TORK_GIDEN, 0);
  osDelay(150);

  uart_send_text("PLC STOP komutlari tamamlandi\r\n");
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

  uart_send_text("GERI/SARMA TEST: 5+ saniye\r\n");

  /*
    Su an sistemde calisan yon bu.
    ESP'den gelen REWIND komutunda da bunu kullanacagiz.
  */
  plc_set_speed_torque(1000, 100);
  osDelay(7000);

  uart_send_text("FINAL STOP\r\n");
  plc_stop();

  uart_send_text("===== MOTION TEST BITTI =====\r\n\r\n");
}

static void plc_rewind_from_esp(void)
{
  /* TEMP: old fixed-mode torque test removed.
     Keep entry point so UART parser stays stable. */
  plc_start_fixed_mode_from_esp();
}

void plc_reset_meter(void)
{
  uart_send_text("PLC: METRE SIFIRLAMA\r\n");
  modbus_write_single_coil(COIL_METRE_SIFIRLAMA, 1);
  osDelay(200);
  modbus_write_single_coil(COIL_METRE_SIFIRLAMA, 0);
  osDelay(200);
}

void plc_set_fixed_speed_torque_distance(uint16_t fixed_speed, uint16_t fixed_torque, uint16_t distance)
{
  char msg[128];

  /* Fixed mode: EKRAN_DEGISKEN_MOD = 0 */
  modbus_write_single_coil(COIL_EKRAN_DEGISKEN_MOD, 0);
  osDelay(80);

  snprintf(msg, sizeof(msg), "PLC: FIXED hiz=%u tork=%u mesafe=%u\r\n",
           (unsigned int)fixed_speed, (unsigned int)fixed_torque, (unsigned int)distance);
  uart_send_text(msg);

  modbus_write_single_register(HR_EKRAN_SERVO_SABIT_HIZ, fixed_speed);
  osDelay(80);
  modbus_write_single_register(HR_EKRAN_SERVO_SABIT_TORK, fixed_torque);
  osDelay(80);
  modbus_write_single_register(HR_SERVO_MESAFE, distance);
  osDelay(120);
}

void plc_start_fixed_mode_from_esp(void)
{

  current_torque = 5;
  current_speed = 100;

  uart_send_text("\r\nESP KOMUT GELDI -> START\r\n");

  modbus_write_single_register(HR_EKRAN_SERVO_SABIT_HIZ, 100);
  osDelay(200);

  modbus_write_single_register(HR_EKRAN_SERVO_SABIT_TORK, current_torque);
  osDelay(200);

  modbus_write_single_coil(COIL_EKRAN_START, 1);
  osDelay(200);

  modbus_write_single_coil(COIL_SISTEM_STARTLI, 1);
  osDelay(200);

  uart_send_text("PLC START DONE\r\n");
}

void plc_stop_from_esp(void)
{
  uart_send_text("\r\nESP KOMUT GELDI -> STOP\r\n");
  plc_busy = 0;
  plc_stop();
  uart_send_text("PLC: STOP DONE\r\n");
}

static void esp_check_uart_command(void)
{
  static uint32_t esp_last_start_ms = 0;

  if (HAL_UART_Receive(&huart2, &esp_rx_byte, 1, 1) != HAL_OK)
  {
    return;
  }

#if ESP_UART_BYTE_TRACE
  char dbg[40];
  snprintf(dbg, sizeof(dbg), "UART2 BYTE: 0x%02X '%c'\r\n",
           (unsigned int)esp_rx_byte,
           (esp_rx_byte >= 32 && esp_rx_byte <= 126) ? (char)esp_rx_byte : '.');
  uart_send_text(dbg);
#endif

  if (esp_rx_byte == 'S')
  {
    uart_send_text("\r\nUART CMD -> S / STOP\r\n");
    plc_busy = 0;
    plc_stop_from_esp();

    memset(esp_cmd_buffer, 0, sizeof(esp_cmd_buffer));
    esp_cmd_index = 0;
    return;
  }

  if (esp_rx_byte == 'A')
  {
    current_torque += 5;

    if (current_torque > 50)
      current_torque = 50;

    uart_send_text("TORQUE PLUS\r\n");

    plc_apply_live_torque();

    return;
  }

  if (esp_rx_byte == 'Z')
  {
    current_torque -= 5;

    if (current_torque < 0)
      current_torque = 0;

    uart_send_text("TORQUE MINUS\r\n");

    plc_apply_live_torque();

    return;
  }

  if (esp_rx_byte == 'K')
  {
    current_speed += 100;

    if (current_speed > 2000)
      current_speed = 2000;

    uart_send_text("SPEED PLUS\r\n");

    plc_apply_live_speed();

    return;
  }

  if (esp_rx_byte == 'M')
  {
    current_speed -= 100;

    if (current_speed < 0)
      current_speed = 0;

    uart_send_text("SPEED MINUS\r\n");

    plc_apply_live_speed();

    return;
  }

  if (esp_rx_byte == 'R')
  {
    uint32_t now_ms = osKernelGetTickCount();

    if ((now_ms - esp_last_start_ms) < 1000U)
    {
      uart_send_text("START IGNORED - COOLDOWN\r\n");
      return;
    }

    if (plc_busy)
    {
      uart_send_text("PLC BUSY - START IGNORED\r\n");
      return;
    }

    uart_send_text("\r\nUART CMD -> R / START\r\n");
    esp_last_start_ms = now_ms;

    plc_busy = 1;
    plc_start_fixed_mode_from_esp();
    plc_busy = 0;

    memset(esp_cmd_buffer, 0, sizeof(esp_cmd_buffer));
    esp_cmd_index = 0;
    return;
  }

  if (esp_rx_byte == '\r' || esp_rx_byte == '\n' || esp_rx_byte == 0x00)
  {
    return;
  }

  uart_send_text("UART BYTE IGNORED\r\n");
}

int modbus_read_all_registers(void)
{
  int sock;
  uint8_t request[12];
  uint8_t response[128];
  char msg[256];

  sock = modbus_open_socket();
  if (sock < 0)
  {
    return -1;
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
    modbus_uart_io_fail_throttled();
    (void)closesocket(sock);
    return -1;
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
    (void)closesocket(sock);
    return 0;
  }

  modbus_uart_io_fail_throttled();
  (void)closesocket(sock);
  return -1;
}

int plc_read_one_holding_register(uint16_t raw_reg_addr, uint16_t *out_value)
{
  int sock;
  uint8_t request[12];
  uint8_t response[64];

  if (out_value == NULL)
  {
    return -1;
  }

  sock = modbus_open_socket();
  if (sock < 0)
  {
    return -1;
  }

  request[0] = 0x00; request[1] = 0x31;
  request[2] = 0x00; request[3] = 0x00;
  request[4] = 0x00; request[5] = 0x06;
  request[6] = PLC_UNIT_ID;
  request[7] = 0x03; /* Read Holding Registers */
  request[8]  = (raw_reg_addr >> 8) & 0xFF;
  request[9]  = raw_reg_addr & 0xFF;
  request[10] = 0x00;
  request[11] = 0x01; /* qty=1 */

  int sent = send(sock, request, (int)sizeof(request), 0);
  if (sent <= 0)
  {
    modbus_uart_io_fail_throttled();
    (void)closesocket(sock);
    return -1;
  }

  int len = recv(sock, response, (int)sizeof(response), 0);
  (void)closesocket(sock);

  if (len >= 11 && response[7] == 0x03 && response[8] == 0x02)
  {
    *out_value = ((uint16_t)response[9] << 8) | response[10];
    return 0;
  }

  modbus_uart_io_fail_throttled();
  return -1;
}

int plc_read_one_coil(uint16_t raw_coil_addr, uint8_t *out_state)
{
  int sock;
  uint8_t request[12];
  uint8_t response[64];

  if (out_state == NULL)
  {
    return -1;
  }

  sock = modbus_open_socket();
  if (sock < 0)
  {
    return -1;
  }

  request[0] = 0x00; request[1] = 0x32;
  request[2] = 0x00; request[3] = 0x00;
  request[4] = 0x00; request[5] = 0x06;
  request[6] = PLC_UNIT_ID;
  request[7] = 0x01; /* Read Coils */
  request[8]  = (raw_coil_addr >> 8) & 0xFF;
  request[9]  = raw_coil_addr & 0xFF;
  request[10] = 0x00;
  request[11] = 0x01; /* qty=1 */

  int sent = send(sock, request, (int)sizeof(request), 0);
  if (sent <= 0)
  {
    modbus_uart_io_fail_throttled();
    (void)closesocket(sock);
    return -1;
  }

  int len = recv(sock, response, (int)sizeof(response), 0);
  (void)closesocket(sock);

  if (len >= 10 && response[7] == 0x01 && response[8] >= 0x01)
  {
    *out_state = (response[9] & 0x01) ? 1U : 0U;
    return 0;
  }

  modbus_uart_io_fail_throttled();
  return -1;
}

void plc_diagnostic_read_all(void)
{
  typedef struct
  {
    uint16_t human_addr;
    const char *name;
  } plc_hr_item_t;

  typedef struct
  {
    uint16_t human_coil;
    const char *name;
  } plc_coil_item_t;

  static const plc_hr_item_t hrs[] =
  {
    { 40001, "SERVO_HIZ" },
    { 40002, "SERVO_HIZ_GELEN" },
    { 40003, "SERVO_HIZ_GIDEN" },
    { 40004, "SERVO_TORK" },
    { 40005, "SERVO_TORK_GELEN" },
    { 40006, "SERVO_TORK_GIDEN" },
    { 40007, "SERVO_MEVCUT_HIZ" },

    { 40311, "SERVO_MESAFE" },
    { 40312, "HR_40312" },
    { 40313, "HR_40313" },
    { 40314, "HR_40314" },
    { 40317, "HR_40317" },

    { 42001, "R_SAYAC" },
    { 42015, "HR_42015" },
    { 42017, "HR_42017" },
    { 42019, "HR_42019" },
    { 42021, "HR_42021" },
    { 42023, "HR_42023" },
    { 42025, "HR_42025" },
    { 42027, "HR_42027" },
    { 42029, "HR_42029" },
    { 42031, "HR_42031" },
    { 42033, "HR_42033" },
    { 42037, "HR_42037" },
    { 42039, "HR_42039" },
    { 42041, "HR_42041" },
  };

  static const plc_coil_item_t coils[] =
  {
    { 2,  "COIL_2"  },
    { 3,  "COIL_3"  },
    { 4,  "COIL_4"  },
    { 5,  "COIL_5"  },
    { 6,  "METRE_SIFIRLAMA" },
    { 7,  "EKRAN_DEGISKEN_MOD" },
    { 8,  "COIL_8"  },
    { 9,  "COIL_9"  },
    { 10, "ISLEM_BITTI" },
    { 11, "COIL_11" },
    { 12, "COIL_12" },
    { 13, "COIL_13" },
  };

  char msg[192];
  uart_send_text("\r\n===== PLC DIAGNOSTIC READ (TEMP) =====\r\n");

  for (uint32_t i = 0; i < (uint32_t)(sizeof(hrs) / sizeof(hrs[0])); i++)
  {
    uint16_t val = 0;
    uint16_t raw = (uint16_t)(hrs[i].human_addr - 40001U);
    int rc = plc_read_one_holding_register(raw, &val);
    if (rc == 0)
    {
      snprintf(msg, sizeof(msg), "HR %u (%5u) %-20s = %u\r\n",
               (unsigned int)hrs[i].human_addr,
               (unsigned int)raw,
               hrs[i].name,
               (unsigned int)val);
    }
    else
    {
      snprintf(msg, sizeof(msg), "HR %u (%5u) %-20s = READ_FAIL\r\n",
               (unsigned int)hrs[i].human_addr,
               (unsigned int)raw,
               hrs[i].name);
    }
    uart_send_text(msg);
  }

  uart_send_text("----- COILS -----\r\n");

  for (uint32_t i = 0; i < (uint32_t)(sizeof(coils) / sizeof(coils[0])); i++)
  {
    uint8_t st = 0;
    uint16_t raw = (uint16_t)(coils[i].human_coil - 1U);
    int rc = plc_read_one_coil(raw, &st);
    if (rc == 0)
    {
      snprintf(msg, sizeof(msg), "C  %u (%5u) %-20s = %u\r\n",
               (unsigned int)coils[i].human_coil,
               (unsigned int)raw,
               coils[i].name,
               (unsigned int)st);
    }
    else
    {
      snprintf(msg, sizeof(msg), "C  %u (%5u) %-20s = READ_FAIL\r\n",
               (unsigned int)coils[i].human_coil,
               (unsigned int)raw,
               coils[i].name);
    }
    uart_send_text(msg);
  }

  uart_send_text("======================================\r\n");
}

/* USER CODE END 0 */

int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  int32_t timeout;
#endif

  MPU_Config();
  //CPU_CACHE_Enable();

#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if (timeout < 0)
  {
    Error_Handler();
  }
#endif

  HAL_Init();

  SystemClock_Config();

#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  __HAL_RCC_HSEM_CLK_ENABLE();
  HAL_HSEM_FastTake(HSEM_ID_0);
  HAL_HSEM_Release(HSEM_ID_0,0);

  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
  if (timeout < 0)
  {
    Error_Handler();
  }
#endif

  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  uart_send_text("\r\n\r\nSTM32 guc verdi\r\n");
  uart_send_text("USART2 ESP KOMUT HATTI HAZIR\r\n");
  /* USER CODE END 2 */

  osKernelInitialize();

  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  if (defaultTaskHandle == NULL)
  {
    uart_send_text("TASK CREATE FAIL\r\n");
  }
  else
  {
    uart_send_text("TASK CREATE OK\r\n");
  }

  osKernelStart();

  while (1)
  {
  }
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
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

  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
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

  /* USER CODE BEGIN StartDefaultTask */
  /*
    Artik otomatik hareket testi yapmiyoruz.
    ESP32 tarafindan USART2 uzerinden "REWIND\n" gelirse calisacak.
  */

  for(;;)
  {
    static uint32_t last_verbose_ms = 0;
    static uint32_t next_modbus_ms = 0;
    static uint32_t mb_fail_streak = 0;
    uint32_t now = osKernelGetTickCount();
    uint8_t verbose_phy;

    /* Poll ESP UART frequently (no long blocking delays). */
    esp_check_uart_command();

    /* PHY/link checks */
    if (HAL_ETH_ReadPHYRegister(&heth, 0, 0x01, &phy_bsr) != HAL_OK)
    {
      uart_send_text("PHY reg read hata\r\n");
    }

    link_now = (phy_bsr & (1 << 2)) ? 1 : 0;

    if (link_now != link_prev)
    {
      if (link_now)
        uart_send_text("LINK DEGISTI -> UP\r\n");
      else
        uart_send_text("LINK DEGISTI -> DOWN\r\n");

      link_prev = link_now;
    }

    /* Keep verbose banner similar to old cadence (~every N seconds). */
    verbose_phy = ((now - last_verbose_ms) >= (DEFAULT_TASK_VERBOSE_INTERVAL * 1000U));
    if (verbose_phy)
    {
      last_verbose_ms = now;

      snprintf(msg, sizeof(msg), "PHY BSR (0x01): 0x%04lX\r\n", phy_bsr);
      uart_send_text(msg);

      if (link_now)
        uart_send_text("BSR link bit: 1\r\n");
      else
        uart_send_text("BSR link bit: 0\r\n");

      if (netif_is_link_up(&gnetif))
        uart_send_text("ETH LINK UP\r\n");
      else
        uart_send_text("ETH LINK DOWN\r\n");

      if (netif_is_up(&gnetif))
        uart_send_text("NETIF UP\r\n");
      else
        uart_send_text("NETIF DOWN\r\n");

      snprintf(msg, sizeof(msg), "RX IRQ: %lu  TX IRQ: %lu  ERR IRQ: %lu\r\n",
               eth_rx_irq_count, eth_tx_irq_count, eth_err_irq_count);
      uart_send_text(msg);

      snprintf(msg, sizeof(msg), "PBUF: %lu  INPUT_OK: %lu  INPUT_FAIL: %lu\r\n",
               eth_pbuf_count, eth_input_ok_count, eth_input_fail_count);
      uart_send_text(msg);

      snprintf(msg, sizeof(msg), "IP: %s\r\n", ip4addr_ntoa(netif_ip4_addr(&gnetif)));
      uart_send_text(msg);

      uart_send_text("---------------------\r\n");
    }

    /* Periodic Modbus reads disabled for stability (no diagnostic/read-all calls). */
    if ((int32_t)(now - next_modbus_ms) >= 0)
    {
      int mb = 0;
      (void)mb;
      mb_fail_streak = 0;
      next_modbus_ms = now + 5000U;
    }

    static uint32_t last_meter_ms = 0;

    if ((now - last_meter_ms) >= 1000U)
    {
      last_meter_ms = now;
      telemetry_send_meter_to_esp();
    }

    /* Small chunk delay so UART polling stays responsive. */
    osDelay(10);
  }
  /* USER CODE END StartDefaultTask  */
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
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
