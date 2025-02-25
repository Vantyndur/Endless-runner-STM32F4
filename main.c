/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f429i_discovery_lcd.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DMA2D_HandleTypeDef hdma2d;

I2C_HandleTypeDef hi2c3;

LTDC_HandleTypeDef hltdc;

SPI_HandleTypeDef hspi5;

SDRAM_HandleTypeDef hsdram1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA2D_Init(void);
static void MX_FMC_Init(void);
static void MX_I2C3_Init(void);
static void MX_LTDC_Init(void);
static void MX_SPI5_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
struct Object {
	int x;
	int y;
	int width;
	int height;
};
static struct Object ObjectInit(int x, int y, int width, int height){
	struct Object obj;
	obj.x = x;
	obj.y = y;
	obj.width = width;
	obj.height = height;
	return obj;
}
void ClearObjectFromDisplay(struct Object object) {
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_FillRect(object.x, object.y, object.width, object.height);
}
int checkIfObjectOnArea(struct Object object, int minX, int maxX, int minY, int maxY){
	if (object.x <= maxX && object.x + object.width >=minX && object.y <= maxY && object.y + object.height>= minY){
			return 1;
		}
		return 0;
}
int checkIfObjectOnScreen(struct Object object){
	if (checkIfObjectOnArea(object, 0, 240 , 0, 320)){
		return 1;
	}
	return 0;
}
void updatePosition(struct Object* object,int inx, int iny) {
	if (iny != object->y || inx != object->x) {
		if (checkIfObjectOnScreen(*object)) {
			ClearObjectFromDisplay(*object);
		}
		object->y = iny;
		object->x = inx;
		if(checkIfObjectOnScreen(*object)) {
			BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
			BSP_LCD_FillRect(object->x, object->y, object->width, object->height);
		}
	}
}

struct Object FloorObjects[5];
void DrawFloor(int scroll) {


	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_FillRect(120, 270, 240, 5);
	// grudki
	for (int i = 0; i < 5; i++) {
		if (FloorObjects[i].x + scroll + FloorObjects[i].width / 2 < 0) {
			updatePosition(&FloorObjects[i], 240 + FloorObjects[i].width / 2 , FloorObjects[i].y);
		}
		else {
			updatePosition(&FloorObjects[i], FloorObjects[i].x + scroll, FloorObjects[i].y);
		}

	}
	//--------
}

struct Object Obstacles[5];
int obstaclesSize = 5;
int obstaclesCount;
int minObstacleDistance;
int maxObstacleDistance;
int minObstacleHeight;
int maxObstacleHeight;
int obstacleWidth;
int CreateObstacle(int x, int y, int width , int height) {
		if (obstaclesCount < obstaclesSize) {
			struct Object newObstacle = ObjectInit(x, y, width, height);
			Obstacles[obstaclesCount] = newObstacle;
			obstaclesCount++;
			return 1;
		}
	return 0;
}
void DeleteObstacle(int index) {
	if (index <= obstaclesCount - 1) {
		if (index < obstaclesSize) {
			for (int i = index + 1; i < 5; i++) {
				Obstacles[i - 1] = Obstacles[i];
			}
		}
	}
	obstaclesCount--;
}
void CreateRandomObstacle() {
	//podmienić funkcją z stm32
	int randomX = rand() % (maxObstacleDistance - minObstacleDistance + 1) + minObstacleDistance;
	int randomHeight = rand() % (maxObstacleHeight - minObstacleHeight + 1) + minObstacleHeight;
	//-------------------------
	if (obstaclesCount > 0) {
		//CreateObstacle(Obstacles[obstaclesCount - 1].x + randomX, (270 - (randomHeight / 2)), obstacleWidth, randomHeight);
		CreateObstacle(Obstacles[obstaclesCount - 1].x + randomX, (270 - randomHeight ), obstacleWidth, randomHeight);
	}
	else {
		//CreateObstacle(250, (270 - (randomHeight / 2)), obstacleWidth, randomHeight);
		CreateObstacle(250, (270 - randomHeight ), obstacleWidth, randomHeight);

	}
}
void DrawObstacles(int scroll) {
	for (int i = 0; i < obstaclesSize; i++) {
		if (Obstacles[i].x + scroll + Obstacles[i].width / 2 < 0) {
			ClearObjectFromDisplay(Obstacles[i]);
			DeleteObstacle(i);
			CreateRandomObstacle();
		}
		else {
			updatePosition(&Obstacles[i], Obstacles[i].x + scroll, Obstacles[i].y);
		}
	}
}

struct Player{
	//kotwica lewy górny róg
	int x;
	int y;
	float jumpY;
	int state;
	int height;
	//szerokość i wysokość dla sprawdzania kolizji
	int colWidth;
	int colHeight;
	//obiekty składające się na gracza
	int countOfParts;
	struct Object parts[9];
	struct Object useableParts[9];

};

struct Player PlayerInit(int x, int y,int height, int colWidth, int colHeight, int countOfParts ,struct Object parts[]){
	struct Player player;
	player.x = x;
	player.y = y;
	player.jumpY = 0.0;
	player.state = 0;
	player.height = height;
	player.colWidth = colWidth;
	player.colHeight = colHeight;
	player.countOfParts = countOfParts;
	for(int i =0; i<countOfParts; i++){
		player.parts[i] = parts[i];
	}
	for(int i =0; i<countOfParts; i++){
			player.useableParts[i] = parts[i];
		}
	return player;
}
struct Player player;
void updatePlayerPosition(int index) {

	ClearObjectFromDisplay(player.useableParts[index]);
	player.useableParts[index].x = player.parts[index].x + player.x;
	player.useableParts[index].y = player.parts[index].y + player.y;
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_FillRect(player.useableParts[index].x, player.useableParts[index].y, player.useableParts[index].width, player.useableParts[index].height);

}
void DrawPlayer(int inY){
	player.y = inY;
	for(int i =0;i<player.countOfParts; i++){
		updatePlayerPosition(i);
	}
}
void startPlayerJump(){
	player.state = 1;
}
void PlayerJump(int minY, int maxY, float Ystep){
	if(player.state == 1){
		//unoszenie

		if(player.y  <= maxY){
			player.state = -1; // zaczyna spadać
			DrawPlayer(player.y + (int)(Ystep));
		}else{
			DrawPlayer(player.y - (int)(Ystep));
		}
	}else if(player.state == -1){
		//spadanie

		if(player.y >= minY-5){
			player.state = 0; // koniec skoku
			DrawPlayer(minY);
			player.jumpY = 0.0;
		}else{
			DrawPlayer(player.y + (int)(Ystep));
		}
	} else{
		DrawPlayer(minY);
	}
}
float yStep;
int maxJumpHeight;
void GameInit() {
	//podłoga
	struct Object F0 = ObjectInit(220, 290, 10, 10);
	struct Object F1 = ObjectInit(160, 300, 8, 12);
	struct Object F2 = ObjectInit(130, 280, 20, 5);
	struct Object F3 = ObjectInit(80, 310, 10, 15);
	struct Object F4 = ObjectInit(20, 285, 8, 6);
	FloorObjects[0] = F0;
	FloorObjects[1] = F1;
	FloorObjects[2] = F2;
	FloorObjects[3] = F3;
	FloorObjects[4] = F4;
	//end podłoga

	//przeszkody
	minObstacleDistance = 70;
	maxObstacleDistance = 240;
	obstacleWidth = 5;
	minObstacleHeight = 30;
	maxObstacleHeight = 70;
	obstaclesCount = 0;
	for (int i = 0; i < obstaclesSize; i++) {
		CreateRandomObstacle();
	}
	//end przeszkody

	//gracz:
	struct Object playerParts[9];

	struct Object P1 = ObjectInit(2,0,20,20);
	struct Object P2 = ObjectInit(8,20,8,6);
	struct Object P3 = ObjectInit(0,26,24,35);
	struct Object P4 = ObjectInit(0,61,8,30);
	struct Object P5 = ObjectInit(16,61,8,30);
	struct Object P6 = ObjectInit(-8,26,8,6);
	struct Object P7 = ObjectInit(24,26,8,6);
	struct Object P8 = ObjectInit(-8,32,6,29);
	struct Object P9 = ObjectInit(26,32,6,29);
	playerParts[0] = P1;
	playerParts[1] = P2;
	playerParts[2] = P3;
	playerParts[3] = P4;
	playerParts[4] = P5;
	playerParts[5] = P6;
	playerParts[6] = P7;
	playerParts[7] = P8;
	playerParts[8] = P9;

	//player = PlayerInit(80,179, 91, 24, 76, 9,playerParts);

	player = PlayerInit(80,179, 91, 24, 76, 9,playerParts);
	maxJumpHeight = maxObstacleHeight + 30;
	yStep = maxJumpHeight/(minObstacleDistance/2); // maxJumpHeight / minimum distance to jump carefully;
}

int checkCollision(){
	for(int i = 0; i<obstaclesCount; i++){
		if(Obstacles[i].x > player.x + player.colWidth+1){
			return 0;
		} else{
			if(checkIfObjectOnArea(Obstacles[i], player.x, player.x+player.colWidth, player.y, player.y + player.colHeight)){
				return 1;
			}
		}
	}
	return 0;
}
void DrawScore(int score){
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	char str[100];
	sprintf(str, "score: %d", score);
	BSP_LCD_DisplayStringAtLine(0, (uint8_t*)&str);
}
void DrawEndScreen(int score, int* HighScore){
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	char str0[100];
	sprintf(str0, "Congratulations!");
	BSP_LCD_DisplayStringAtLine(4,(uint8_t*)&str0);
	char str1[100];
	sprintf(str1, "Your score: %d", score);
	BSP_LCD_DisplayStringAtLine(4, (uint8_t*)&str1);
	if(score > *HighScore){
		char str2[100];
		sprintf(str2,"You've just set new High Score!");
		BSP_LCD_DisplayStringAtLine( 6, (uint8_t*)&str2);
		*HighScore = score;
	} else{
		char str2[100];
		sprintf(str2,"High Score!:%d", *HighScore);
		BSP_LCD_DisplayStringAtLine(6, (uint8_t*)&str2);
	}
	char str3[100];
	sprintf(str3,"Click the button to play again.");
	BSP_LCD_DisplayStringAtLine(8, (uint8_t*)&str3);
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

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA2D_Init();
  MX_FMC_Init();
  MX_I2C3_Init();
  MX_LTDC_Init();
  MX_SPI5_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  BSP_LCD_Init();
    BSP_LCD_LayerDefaultInit(LCD_BACKGROUND_LAYER, LCD_FRAME_BUFFER);
    BSP_LCD_Clear(LCD_COLOR_BLACK);
    GameInit();
      int maxScore = 0;
      int score = 0;
      int scroll = -1;
      int delay = 50;
      int gameOn = 1;
      player.state = 0;
  while (1)
  {
	  if(gameOn){
	  		  DrawFloor(scroll);
	  		  DrawObstacles(scroll);
	  		  // when button is clicked
	  		  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET  ){
	  			  if(player.state == 0){
	  				  player.state = 1;
	  				HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_13);
	  			  }
	  			  // zgaszenie diody
	  		  }
	  		  //--------------
	  		  PlayerJump(270-player.height, 270-(maxJumpHeight+player.height), yStep);
	  		  //PlayerJump(270, 270-(maxJumpHeight+player.height), 50);
	  		  if(checkCollision()){
	  			  gameOn = 0;
	  			  BSP_LCD_Clear(LCD_COLOR_WHITE);
	  		  }else{
	  			  score = score + 1;
	  		  }
	  		  //----------------
	  		  DrawScore(score);
	  		  HAL_Delay(delay);
	  	  } else {
	  		  DrawEndScreen(score, &maxScore);
	  		  while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET){}
	  		  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET){
	  			  BSP_LCD_Clear(LCD_COLOR_BLACK);
	  			  gameOn = 1;
	  			  GameInit();
	  			  score = 0;
	  			  HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_13);// zgaszenie diody
	  		  }
	  	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DMA2D Initialization Function
  * @param None
  * @retval None
  */
static void MX_DMA2D_Init(void)
{

  /* USER CODE BEGIN DMA2D_Init 0 */

  /* USER CODE END DMA2D_Init 0 */

  /* USER CODE BEGIN DMA2D_Init 1 */

  /* USER CODE END DMA2D_Init 1 */
  hdma2d.Instance = DMA2D;
  hdma2d.Init.Mode = DMA2D_M2M;
  hdma2d.Init.ColorMode = DMA2D_OUTPUT_RGB565;
  hdma2d.Init.OutputOffset = 0;
  hdma2d.LayerCfg[1].InputOffset = 0;
  hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_RGB565;
  hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  hdma2d.LayerCfg[1].InputAlpha = 0;
  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DMA2D_Init 2 */

  /* USER CODE END DMA2D_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief LTDC Initialization Function
  * @param None
  * @retval None
  */
static void MX_LTDC_Init(void)
{

  /* USER CODE BEGIN LTDC_Init 0 */

  /* USER CODE END LTDC_Init 0 */

  LTDC_LayerCfgTypeDef pLayerCfg = {0};
  LTDC_LayerCfgTypeDef pLayerCfg1 = {0};

  /* USER CODE BEGIN LTDC_Init 1 */

  /* USER CODE END LTDC_Init 1 */
  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync = 7;
  hltdc.Init.VerticalSync = 3;
  hltdc.Init.AccumulatedHBP = 14;
  hltdc.Init.AccumulatedVBP = 5;
  hltdc.Init.AccumulatedActiveW = 254;
  hltdc.Init.AccumulatedActiveH = 325;
  hltdc.Init.TotalWidth = 260;
  hltdc.Init.TotalHeigh = 327;
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 0;
  hltdc.Init.Backcolor.Red = 0;
  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = 0;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = 0;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
  pLayerCfg.Alpha = 0;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg.FBStartAdress = 0;
  pLayerCfg.ImageWidth = 0;
  pLayerCfg.ImageHeight = 0;
  pLayerCfg.Backcolor.Blue = 0;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg1.WindowX0 = 0;
  pLayerCfg1.WindowX1 = 0;
  pLayerCfg1.WindowY0 = 0;
  pLayerCfg1.WindowY1 = 0;
  pLayerCfg1.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
  pLayerCfg1.Alpha = 0;
  pLayerCfg1.Alpha0 = 0;
  pLayerCfg1.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg1.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg1.FBStartAdress = 0;
  pLayerCfg1.ImageWidth = 0;
  pLayerCfg1.ImageHeight = 0;
  pLayerCfg1.Backcolor.Blue = 0;
  pLayerCfg1.Backcolor.Green = 0;
  pLayerCfg1.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg1, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LTDC_Init 2 */

  /* USER CODE END LTDC_Init 2 */

}

/**
  * @brief SPI5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI5_Init(void)
{

  /* USER CODE BEGIN SPI5_Init 0 */

  /* USER CODE END SPI5_Init 0 */

  /* USER CODE BEGIN SPI5_Init 1 */

  /* USER CODE END SPI5_Init 1 */
  /* SPI5 parameter configuration*/
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI5_Init 2 */

  /* USER CODE END SPI5_Init 2 */

}

/* FMC initialization function */
static void MX_FMC_Init(void)
{

  /* USER CODE BEGIN FMC_Init 0 */

  /* USER CODE END FMC_Init 0 */

  FMC_SDRAM_TimingTypeDef SdramTiming = {0};

  /* USER CODE BEGIN FMC_Init 1 */

  /* USER CODE END FMC_Init 1 */

  /** Perform the SDRAM1 memory initialization sequence
  */
  hsdram1.Instance = FMC_SDRAM_DEVICE;
  /* hsdram1.Init */
  hsdram1.Init.SDBank = FMC_SDRAM_BANK2;
  hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
  hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
  hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
  hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_1;
  hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_DISABLE;
  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_DISABLE;
  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_0;
  /* SdramTiming */
  SdramTiming.LoadToActiveDelay = 16;
  SdramTiming.ExitSelfRefreshDelay = 16;
  SdramTiming.SelfRefreshTime = 16;
  SdramTiming.RowCycleDelay = 16;
  SdramTiming.WriteRecoveryTime = 16;
  SdramTiming.RPDelay = 16;
  SdramTiming.RCDDelay = 16;

  if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FMC_Init 2 */

  /* USER CODE END FMC_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PG13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
