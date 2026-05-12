/* USER CODE BEGIN Header */
/**
******************************************************************************
* @file : main.c
* @brief : Main program body
******************************************************************************
*/
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* MPU6050 */
#define MPU6050_ADDR_68              (0x68 << 1)
#define MPU6050_ADDR_69              (0x69 << 1)

#define MPU6050_WHO_AM_I_REG         0x75
#define MPU6050_PWR_MGMT_1_REG       0x6B
#define MPU6050_ACCEL_XOUT_H_REG     0x3B
#define MPU6050_GYRO_CONFIG_REG      0x1B
#define MPU6050_ACCEL_CONFIG_REG     0x1C

/* RPLIDAR */
#define RPLIDAR_SYNC_BYTE            0xA5
#define RPLIDAR_CMD_GET_INFO         0x50
#define RPLIDAR_CMD_STOP             0x25
#define RPLIDAR_CMD_SCAN             0x20

/* PuTTY command */
#define CTRL_C_BYTE                  0x03

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* PC / PuTTY command variables */
uint8_t pc_cmd = 0;
uint8_t output_paused = 0;

/* MPU6050 variables */
uint8_t who_am_i = 0;
uint16_t mpu_addr = MPU6050_ADDR_68;
HAL_StatusTypeDef mpu_status;

/* Raw IMU values */
volatile int16_t accel_x_raw = 0;
volatile int16_t accel_y_raw = 0;
volatile int16_t accel_z_raw = 0;

volatile int16_t temp_raw = 0;

volatile int16_t gyro_x_raw = 0;
volatile int16_t gyro_y_raw = 0;
volatile int16_t gyro_z_raw = 0;

/* Scaled IMU values */
volatile int32_t accel_x_mg = 0;
volatile int32_t accel_y_mg = 0;
volatile int32_t accel_z_mg = 0;

volatile int32_t gyro_x_dps_x100 = 0;
volatile int32_t gyro_y_dps_x100 = 0;
volatile int32_t gyro_z_dps_x100 = 0;

volatile int32_t temp_c_x100 = 0;

uint8_t imu_data[14];
char uart_buf[220];

/* RPLIDAR variables */
uint8_t lidar_info_rx[27];
uint8_t lidar_desc[7];
uint8_t lidar_node[5];
char debug_msg[220];

volatile uint8_t lidar_scan_running = 0;
volatile uint32_t lidar_valid_samples = 0;
volatile uint32_t lidar_print_counter = 0;

/* Timing variables */
uint32_t last_imu_read = 0;
uint32_t last_imu_print = 0;
uint32_t last_led_toggle = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);

/* USER CODE BEGIN PFP */

/* Debug / PC command */
void Debug_Print(char *msg);
void Check_PC_Command(void);

/* MPU6050 */
uint8_t MPU6050_Detect(void);
void MPU6050_Init(void);
HAL_StatusTypeDef MPU6050_Read_All(void);
void MPU6050_Recover_I2C(void);
void UART_Print_IMU(void);

/* RPLIDAR */
void RPLIDAR_Stop(void);
void RPLIDAR_Clear_UART_Buffer(void);
void RPLIDAR_GetInfo_Test(void);
uint8_t RPLIDAR_Start_Scan(void);
uint8_t RPLIDAR_Read_Valid_Node(uint8_t *node, uint32_t timeout_ms);
void RPLIDAR_Print_Node(uint8_t *node);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void Debug_Print(char *msg)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 1000);
}

void Check_PC_Command(void)
{
    /*
      PuTTY Ctrl+C sends byte 0x03.
      Ctrl+C once  -> pause output and stop LIDAR scan.
      Ctrl+C again -> resume output and restart LIDAR scan.
    */

    if (HAL_UART_Receive(&huart2, &pc_cmd, 1, 0) == HAL_OK)
    {
        if (pc_cmd == CTRL_C_BYTE)
        {
            if (output_paused == 0)
            {
                output_paused = 1;

                Debug_Print("\r\n--- CTRL+C received: stopping LIDAR and pausing output ---\r\n");

                RPLIDAR_Stop();
                lidar_scan_running = 0;

                Debug_Print("--- OUTPUT PAUSED. Press Ctrl+C again to resume. ---\r\n");
            }
            else
            {
                output_paused = 0;

                Debug_Print("\r\n--- CTRL+C received: restarting LIDAR and resuming output ---\r\n");

                if (!RPLIDAR_Start_Scan())
                {
                    Debug_Print("RPLIDAR scan restart failed. Check motor power.\r\n");
                }

                Debug_Print("--- OUTPUT RESUMED. ---\r\n");
            }
        }
    }
}

/* ========================= MPU6050 FUNCTIONS ========================= */

uint8_t MPU6050_Detect(void)
{
    who_am_i = 0;

    mpu_status = HAL_I2C_Mem_Read(
        &hi2c1,
        MPU6050_ADDR_68,
        MPU6050_WHO_AM_I_REG,
        I2C_MEMADD_SIZE_8BIT,
        &who_am_i,
        1,
        100
    );

    if (mpu_status == HAL_OK && who_am_i == 0x68)
    {
        mpu_addr = MPU6050_ADDR_68;
        return 1;
    }

    who_am_i = 0;

    mpu_status = HAL_I2C_Mem_Read(
        &hi2c1,
        MPU6050_ADDR_69,
        MPU6050_WHO_AM_I_REG,
        I2C_MEMADD_SIZE_8BIT,
        &who_am_i,
        1,
        100
    );

    if (mpu_status == HAL_OK && who_am_i == 0x68)
    {
        mpu_addr = MPU6050_ADDR_69;
        return 1;
    }

    return 0;
}

void MPU6050_Init(void)
{
    uint8_t data;

    /* Wake up MPU6050 */
    data = 0x00;
    HAL_I2C_Mem_Write(
        &hi2c1,
        mpu_addr,
        MPU6050_PWR_MGMT_1_REG,
        I2C_MEMADD_SIZE_8BIT,
        &data,
        1,
        100
    );

    HAL_Delay(100);

    /* Gyroscope full scale: +/- 250 deg/s */
    data = 0x00;
    HAL_I2C_Mem_Write(
        &hi2c1,
        mpu_addr,
        MPU6050_GYRO_CONFIG_REG,
        I2C_MEMADD_SIZE_8BIT,
        &data,
        1,
        100
    );

    /* Accelerometer full scale: +/- 2g */
    data = 0x00;
    HAL_I2C_Mem_Write(
        &hi2c1,
        mpu_addr,
        MPU6050_ACCEL_CONFIG_REG,
        I2C_MEMADD_SIZE_8BIT,
        &data,
        1,
        100
    );
}

HAL_StatusTypeDef MPU6050_Read_All(void)
{
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read(
        &hi2c1,
        mpu_addr,
        MPU6050_ACCEL_XOUT_H_REG,
        I2C_MEMADD_SIZE_8BIT,
        imu_data,
        14,
        100
    );

    if (status != HAL_OK)
    {
        return status;
    }

    accel_x_raw = (int16_t)((imu_data[0] << 8) | imu_data[1]);
    accel_y_raw = (int16_t)((imu_data[2] << 8) | imu_data[3]);
    accel_z_raw = (int16_t)((imu_data[4] << 8) | imu_data[5]);

    temp_raw = (int16_t)((imu_data[6] << 8) | imu_data[7]);

    gyro_x_raw = (int16_t)((imu_data[8] << 8) | imu_data[9]);
    gyro_y_raw = (int16_t)((imu_data[10] << 8) | imu_data[11]);
    gyro_z_raw = (int16_t)((imu_data[12] << 8) | imu_data[13]);

    accel_x_mg = ((int32_t)accel_x_raw * 1000) / 16384;
    accel_y_mg = ((int32_t)accel_y_raw * 1000) / 16384;
    accel_z_mg = ((int32_t)accel_z_raw * 1000) / 16384;

    gyro_x_dps_x100 = ((int32_t)gyro_x_raw * 100) / 131;
    gyro_y_dps_x100 = ((int32_t)gyro_y_raw * 100) / 131;
    gyro_z_dps_x100 = ((int32_t)gyro_z_raw * 100) / 131;

    temp_c_x100 = (((int32_t)temp_raw * 100) / 340) + 3653;

    return HAL_OK;
}

void MPU6050_Recover_I2C(void)
{
    Debug_Print("MPU6050 I2C read failed, recovering...\r\n");

    HAL_I2C_DeInit(&hi2c1);
    HAL_Delay(50);

    MX_I2C1_Init();
    HAL_Delay(100);

    if (MPU6050_Detect())
    {
        MPU6050_Init();
        Debug_Print("MPU6050 recovered\r\n");
    }
    else
    {
        Debug_Print("MPU6050 still not detected\r\n");
    }
}

void UART_Print_IMU(void)
{
    int len;

    len = snprintf(
        uart_buf,
        sizeof(uart_buf),
        "IMU | ACC mg: X=%ld Y=%ld Z=%ld | GYRO dps: X=%ld.%02ld Y=%ld.%02ld Z=%ld.%02ld | TEMP=%ld.%02ld C\r\n",
        accel_x_mg,
        accel_y_mg,
        accel_z_mg,

        gyro_x_dps_x100 / 100,
        labs(gyro_x_dps_x100 % 100),

        gyro_y_dps_x100 / 100,
        labs(gyro_y_dps_x100 % 100),

        gyro_z_dps_x100 / 100,
        labs(gyro_z_dps_x100 % 100),

        temp_c_x100 / 100,
        labs(temp_c_x100 % 100)
    );

    if (len > 0)
    {
        HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, len, 100);
    }
}

/* ========================= RPLIDAR FUNCTIONS ========================= */

void RPLIDAR_Stop(void)
{
    uint8_t stop_cmd[2] = {RPLIDAR_SYNC_BYTE, RPLIDAR_CMD_STOP};

    HAL_UART_Transmit(&huart3, stop_cmd, 2, 100);
    HAL_Delay(100);
}

void RPLIDAR_Clear_UART_Buffer(void)
{
    uint8_t dump;

    while (HAL_UART_Receive(&huart3, &dump, 1, 5) == HAL_OK)
    {
        /* discard old bytes */
    }
}

void RPLIDAR_GetInfo_Test(void)
{
    uint8_t get_info_cmd[2] = {RPLIDAR_SYNC_BYTE, RPLIDAR_CMD_GET_INFO};

    memset(lidar_info_rx, 0, sizeof(lidar_info_rx));

    Debug_Print("\r\nSending RPLIDAR STOP...\r\n");
    RPLIDAR_Stop();
    RPLIDAR_Clear_UART_Buffer();

    Debug_Print("Sending RPLIDAR GET_INFO...\r\n");
    HAL_UART_Transmit(&huart3, get_info_cmd, 2, 100);

    HAL_StatusTypeDef status = HAL_UART_Receive(&huart3, lidar_info_rx, 27, 1000);

    if (status == HAL_OK)
    {
        Debug_Print("RPLIDAR response received:\r\n");

        for (int i = 0; i < 27; i++)
        {
            snprintf(debug_msg, sizeof(debug_msg), "%02X ", lidar_info_rx[i]);
            Debug_Print(debug_msg);
        }

        Debug_Print("\r\n");

        if (lidar_info_rx[0] == 0xA5 && lidar_info_rx[1] == 0x5A)
        {
            Debug_Print("RPLIDAR GET_INFO looks valid!\r\n");
        }
        else
        {
            Debug_Print("Received data, but header is not A5 5A.\r\n");
        }
    }
    else
    {
        Debug_Print("No RPLIDAR response. Check wiring, power, USART3 pins, and baud rate.\r\n");
    }
}

uint8_t RPLIDAR_Start_Scan(void)
{
    uint8_t scan_cmd[2] = {RPLIDAR_SYNC_BYTE, RPLIDAR_CMD_SCAN};

    Debug_Print("\r\nStarting continuous RPLIDAR scan...\r\n");

    RPLIDAR_Stop();
    RPLIDAR_Clear_UART_Buffer();

    HAL_UART_Transmit(&huart3, scan_cmd, 2, 100);

    memset(lidar_desc, 0, sizeof(lidar_desc));

    HAL_StatusTypeDef status = HAL_UART_Receive(&huart3, lidar_desc, 7, 1000);

    if (status != HAL_OK)
    {
        Debug_Print("No SCAN response descriptor received.\r\n");
        lidar_scan_running = 0;
        return 0;
    }

    Debug_Print("SCAN descriptor:\r\n");

    for (int i = 0; i < 7; i++)
    {
        snprintf(debug_msg, sizeof(debug_msg), "%02X ", lidar_desc[i]);
        Debug_Print(debug_msg);
    }

    Debug_Print("\r\n");

    if (lidar_desc[0] != 0xA5 || lidar_desc[1] != 0x5A)
    {
        Debug_Print("Invalid SCAN descriptor header.\r\n");
        lidar_scan_running = 0;
        return 0;
    }

    Debug_Print("RPLIDAR continuous scan started\r\n");

    lidar_scan_running = 1;
    lidar_valid_samples = 0;
    lidar_print_counter = 0;

    return 1;
}

uint8_t RPLIDAR_Read_Valid_Node(uint8_t *node, uint32_t timeout_ms)
{
    uint32_t start_time = HAL_GetTick();
    uint8_t b = 0;

    while ((HAL_GetTick() - start_time) < timeout_ms)
    {
        if (HAL_UART_Receive(&huart3, &b, 1, 2) == HAL_OK)
        {
            node[0] = node[1];
            node[1] = node[2];
            node[2] = node[3];
            node[3] = node[4];
            node[4] = b;

            uint8_t start_bit = node[0] & 0x01;
            uint8_t inverted_start_bit = (node[0] >> 1) & 0x01;
            uint8_t check_bit = node[1] & 0x01;

            /*
              Standard RPLIDAR node validity:
              bit0 of byte0 and bit1 of byte0 must be opposite.
              bit0 of byte1 must be 1.
            */
            if ((start_bit != inverted_start_bit) && (check_bit == 1))
            {
                return 1;
            }
        }
    }

    return 0;
}

void RPLIDAR_Print_Node(uint8_t *node)
{
    uint8_t quality = node[0] >> 2;

    uint16_t angle_q6 = ((uint16_t)node[1] | ((uint16_t)node[2] << 8)) >> 1;
    uint16_t distance_q2 = ((uint16_t)node[3] | ((uint16_t)node[4] << 8));

    if (distance_q2 == 0)
    {
        return;
    }

    uint16_t angle_int = angle_q6 / 64;
    uint16_t angle_frac = ((angle_q6 % 64) * 100) / 64;

    uint16_t distance_int = distance_q2 / 4;
    uint16_t distance_frac = (distance_q2 % 4) * 25;
    /* Reject bad / misaligned / unrealistic samples */
    if (angle_int >= 360)
    {
        return;
    }

    if (distance_int < 100 || distance_int > 6000)
    {
        return;
    }

    if (quality < 10)
    {
        return;
    }

    snprintf(
        debug_msg,
        sizeof(debug_msg),
        "LIDAR | Sample=%lu | Angle=%u.%02u deg | Distance=%u.%02u mm | Quality=%u\r\n",
        lidar_valid_samples,
        angle_int,
        angle_frac,
        distance_int,
        distance_frac,
        quality
    );

    Debug_Print(debug_msg);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();

  /* USER CODE BEGIN 2 */

  HAL_Delay(1000);

  Debug_Print("\r\nSTM32 started\r\n");
  Debug_Print("Press Ctrl+C in PuTTY to pause/stop. Press Ctrl+C again to resume.\r\n");

  if (MPU6050_Detect())
  {
      MPU6050_Init();
      Debug_Print("MPU6050 detected and initialized\r\n");
  }
  else
  {
      Debug_Print("MPU6050 NOT detected\r\n");
  }

  RPLIDAR_GetInfo_Test();

  if (!RPLIDAR_Start_Scan())
  {
      Debug_Print("RPLIDAR scan did not start. Check motor power.\r\n");
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /*
      Check if user pressed Ctrl+C in PuTTY.
    */
    Check_PC_Command();

    /*
      If paused, keep program alive but stop printing and stop scanning.
      Press Ctrl+C again to resume.
    */
    if (output_paused)
    {
        if (HAL_GetTick() - last_led_toggle >= 1000)
        {
            last_led_toggle = HAL_GetTick();
            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        }

        HAL_Delay(10);
        continue;
    }

    /*
      Keep reading LIDAR continuously.
      We only print every 20th valid sample because USART2 at 115200
      cannot print every LIDAR sample fast enough.
    */
    if (lidar_scan_running)
    {
        if (RPLIDAR_Read_Valid_Node(lidar_node, 10))
        {
            lidar_valid_samples++;
            lidar_print_counter++;

            if (lidar_print_counter >= 20)
            {
                RPLIDAR_Print_Node(lidar_node);
                lidar_print_counter = 0;
            }
        }
    }

    /*
      Read IMU every 100 ms.
    */
    if (HAL_GetTick() - last_imu_read >= 100)
    {
        last_imu_read = HAL_GetTick();

        if (mpu_status == HAL_OK && who_am_i == 0x68)
        {
            if (MPU6050_Read_All() != HAL_OK)
            {
                MPU6050_Recover_I2C();
            }
        }
    }

    /*
      Print IMU once per second.
    */
    if (HAL_GetTick() - last_imu_print >= 1000)
    {
        last_imu_print = HAL_GetTick();

        if (mpu_status == HAL_OK && who_am_i == 0x68)
        {
            UART_Print_IMU();
        }
    }

    /*
      Blink LED without blocking the loop.
    */
    if (HAL_GetTick() - last_led_toggle >= 500)
    {
        last_led_toggle = HAL_GetTick();
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    }

    /* USER CODE END 3 */
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks */
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

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
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

  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 460800;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /* Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /* Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();

  while (1)
  {
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
      HAL_Delay(100);
  }
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  * where the assert_param error has occurred.
  */
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
