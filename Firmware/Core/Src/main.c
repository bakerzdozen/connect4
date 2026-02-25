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
#include <stdint.h>
#include <stdbool.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "core_cm4.h"
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
PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */
#define LED_ROWS			6
#define LED_COLS			7
#define LED_COUNT			(LED_ROWS * LED_COLS)

typedef enum _led_mode {
	LED_OFF,
	LED_PLAYER1,
	LED_PLAYER2,
	LED_BOTH
} ledMode_t;

typedef enum _game_state {
	GAME_INIT,
	GAME_PLAYER1,
	GAME_PLAYER2,
	GAME_WIN_PLAYER1,
	GAME_WIN_PLAYER2,
	GAME_DRAW
} gameState_t;

typedef struct _col_driver {
	GPIO_TypeDef* dat2Port;
	GPIO_TypeDef* dat1Port;
	GPIO_TypeDef* clk2Port;
	GPIO_TypeDef* clk1Port;
	uint16_t dat1Pin;
	uint16_t dat2Pin;
	uint16_t clk1Pin;
	uint16_t clk2Pin;
} colDriver_t;

typedef struct _pos {
	uint8_t col;
	uint8_t row;
} pos_t;

#define HUMAN_PLAYER		LED_PLAYER1
#define COMPUTER_PLAYER		LED_PLAYER2

ledMode_t leds[LED_COUNT] = {0};
colDriver_t column_drivers[LED_COLS] = {0};

pos_t winning_pieces[7];
uint8_t winning_pieces_count = 0;

const int8_t ROW_DELTA[] = {-1, -1, 0, 1};
const int8_t COL_DELTA[] = {0, 1, 1, 1};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USB_PCD_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void driver_init()
{
	// Column 1
	// Ports
	column_drivers[0].clk1Port = GPIOA;
	column_drivers[0].dat1Port = GPIOA;
	column_drivers[0].clk2Port = GPIOA;
	column_drivers[0].dat2Port = GPIOA;
	// Pins
	column_drivers[0].clk1Pin = GPIO_PIN_4;
	column_drivers[0].dat1Pin = GPIO_PIN_5;
	column_drivers[0].clk2Pin = GPIO_PIN_6;
	column_drivers[0].dat2Pin = GPIO_PIN_7;

	// Column 2
	// Ports
	column_drivers[1].clk1Port = GPIOB;
	column_drivers[1].dat1Port = GPIOB;
	column_drivers[1].clk2Port = GPIOB;
	column_drivers[1].dat2Port = GPIOB;
	// Pins
	column_drivers[1].clk1Pin = GPIO_PIN_0;
	column_drivers[1].dat1Pin = GPIO_PIN_1;
	column_drivers[1].clk2Pin = GPIO_PIN_2;
	column_drivers[1].dat2Pin = GPIO_PIN_10;

	// Column 3
	// Ports
	column_drivers[2].clk1Port = GPIOB;
	column_drivers[2].dat1Port = GPIOB;
	column_drivers[2].clk2Port = GPIOB;
	column_drivers[2].dat2Port = GPIOB;
	// Pins
	column_drivers[2].clk1Pin = GPIO_PIN_11;
	column_drivers[2].dat1Pin = GPIO_PIN_12;
	column_drivers[2].clk2Pin = GPIO_PIN_13;
	column_drivers[2].dat2Pin = GPIO_PIN_14;

	// Column 4
	// Ports
	column_drivers[3].clk1Port = GPIOB;
	column_drivers[3].dat1Port = GPIOA;
	column_drivers[3].clk2Port = GPIOA;
	column_drivers[3].dat2Port = GPIOA;
	// Pins
	column_drivers[3].clk1Pin = GPIO_PIN_15;
	column_drivers[3].dat1Pin = GPIO_PIN_8;
	column_drivers[3].clk2Pin = GPIO_PIN_9;
	column_drivers[3].dat2Pin = GPIO_PIN_10;

	// Column 5
	// Ports
	column_drivers[4].clk1Port = GPIOA;
	column_drivers[4].dat1Port = GPIOA;
	column_drivers[4].clk2Port = GPIOA;
	column_drivers[4].dat2Port = GPIOB;
	// Pins
	column_drivers[4].clk1Pin = GPIO_PIN_13;
	column_drivers[4].dat1Pin = GPIO_PIN_14;
	column_drivers[4].clk2Pin = GPIO_PIN_15;
	column_drivers[4].dat2Pin = GPIO_PIN_3;

	// Column 6
	// Ports
	column_drivers[5].clk1Port = GPIOB;
	column_drivers[5].dat1Port = GPIOB;
	column_drivers[5].clk2Port = GPIOB;
	column_drivers[5].dat2Port = GPIOB;
	// Pins
	column_drivers[5].clk1Pin = GPIO_PIN_4;
	column_drivers[5].dat1Pin = GPIO_PIN_5;
	column_drivers[5].clk2Pin = GPIO_PIN_6;
	column_drivers[5].dat2Pin = GPIO_PIN_7;

	// Column 7
	// Ports
	column_drivers[6].clk1Port = GPIOB;
	column_drivers[6].dat1Port = GPIOC;
	column_drivers[6].clk2Port = GPIOC;
	column_drivers[6].dat2Port = GPIOC;
	// Pins
	column_drivers[6].clk1Pin = GPIO_PIN_9;
	column_drivers[6].dat1Pin = GPIO_PIN_13;
	column_drivers[6].clk2Pin = GPIO_PIN_14;
	column_drivers[6].dat2Pin = GPIO_PIN_15;
//
//	// Set all clocks to HIGH
//	for (uint8_t col = 0; col < LED_COLS; col++) {
//		HAL_GPIO_TogglePin(column_drivers[col].clk1Port, column_drivers[col].clk1Pin);
//		HAL_GPIO_TogglePin(column_drivers[col].clk2Port, column_drivers[col].clk2Pin);
//	}

	// Set the ENABLE line to high
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
}

static void output_col(uint8_t col_no)
{
	colDriver_t driver = column_drivers[col_no];

	uint8_t row = 0;

	for (; row < LED_ROWS; row++) {
		// Set the data then pulse the clock
		ledMode_t p1stat = leds[row * LED_COLS + col_no];
		ledMode_t p2stat = leds[((LED_ROWS - 1) - row) * LED_COLS + col_no];

		// No output for either
		HAL_GPIO_WritePin(driver.dat1Port, driver.dat1Pin, p1stat == LED_PLAYER1);
		HAL_GPIO_WritePin(driver.dat2Port, driver.dat2Pin, p2stat == LED_PLAYER2);

		HAL_GPIO_TogglePin(driver.clk1Port, driver.clk1Pin);
		HAL_GPIO_TogglePin(driver.clk2Port, driver.clk2Pin);

		HAL_GPIO_TogglePin(driver.clk1Port, driver.clk1Pin);
		HAL_GPIO_TogglePin(driver.clk2Port, driver.clk2Pin);
	}

	while (row++ <= 8) {

		HAL_GPIO_TogglePin(driver.clk1Port, driver.clk1Pin);
		HAL_GPIO_TogglePin(driver.clk2Port, driver.clk2Pin);

		HAL_GPIO_TogglePin(driver.clk1Port, driver.clk1Pin);
		HAL_GPIO_TogglePin(driver.clk2Port, driver.clk2Pin);
	}
}

void place_piece(ledMode_t player, uint8_t col_no)
{
	if (col_no >= LED_COLS) {
		return;
	}

	// Start at row 0 and increment until there is already a piece
	uint8_t row = 0;

	for (; row < LED_ROWS; row++) {
		ledMode_t state = leds[row * LED_COLS + col_no];

		if (state != LED_OFF) {
			break;
		} else if (row > 0) {
			// Remove previous LED
			leds[(row - 1) * LED_COLS + col_no] = LED_OFF;
		}

		leds[row * LED_COLS + col_no] = player;

		// Update column
		output_col(col_no);

		HAL_Delay(500);
	}
}

int8_t get_next_row(uint8_t col)
{
	int8_t row = (LED_ROWS - 1);

	for (; row >= 0; row--) {
		if (leds[row * LED_COLS + col] == LED_OFF) {
			break;
		}
	}
	return row;
}

void set_winning_pieces(ledMode_t player, uint8_t col) {
	// We need to find the direction of the winning pieces
	uint8_t max = 0;
	uint8_t dir_pos = 0;

	int8_t row = get_next_row(col);

	if (row < 0) {
		row = 0;
	} else if (row < (LED_ROWS - 1)) {
		row++;
	}

	for (uint8_t dir = 0; dir < sizeof(ROW_DELTA); dir++) {
		int8_t dRow = ROW_DELTA[dir];
		int8_t dCol = COL_DELTA[dir];

		int8_t currRow = row + dRow;
		int8_t currCol = col + dCol;

		// Loop until last matching piece
		while ((currRow < LED_ROWS) && (currRow >= 0) &&
			   (currCol < LED_COLS) && (currCol >= 0)) {
			if (leds[currRow * LED_COLS + currCol] != player) {
				break;
			}

			currRow += dRow;
			currCol += dCol;
		}

		// Decrease currRow & currCol to last known position
		currRow -= dRow;
		currCol -= dCol;

		dRow = -dRow;
		dCol = -dCol;

		uint8_t continuous = 0;

		while ((currRow < LED_ROWS) && (currRow >= 0) &&
			   (currCol < LED_COLS) && (currCol >= 0)) {

			if (leds[currRow * LED_COLS + currCol] != player) {
				break;
			}

			continuous++;

			currRow += dRow;
			currCol += dCol;
		}

		if (continuous > max) {
			max = continuous;
			dir_pos = dir;
		}
	}

	int8_t dRow = ROW_DELTA[dir_pos];
	int8_t dCol = COL_DELTA[dir_pos];

	int8_t currRow = row + dRow;
	int8_t currCol = col + dCol;

	// Loop until last matching piece
	while ((currRow < LED_ROWS) && (currRow >= 0) &&
		   (currCol < LED_COLS) && (currCol >= 0)) {
		if (leds[currRow * LED_COLS + currCol] != player) {
			break;
		}

		currRow += dRow;
		currCol += dCol;
	}

	// Decrease currRow & currCol to last known position
	currRow -= dRow;
	currCol -= dCol;

	dRow = -dRow;
	dCol = -dCol;

	winning_pieces_count = 0;

	while ((currRow < LED_ROWS) && (currRow >= 0) &&
		   (currCol < LED_COLS) && (currCol >= 0)) {

		if (leds[currRow * LED_COLS + currCol] != player) {
			break;
		}

		winning_pieces[winning_pieces_count].col = currCol;
		winning_pieces[winning_pieces_count++].row = currRow;

		currRow += dRow;
		currCol += dCol;
	}
}

uint8_t eval_position(ledMode_t player, uint8_t col, uint8_t row) {
	ledMode_t opposite = (player == LED_PLAYER1) ? LED_PLAYER2 : LED_PLAYER1;

	uint8_t max = 0;

	for (uint8_t dir = 0; dir < sizeof(ROW_DELTA); dir++) {
		int8_t dRow = ROW_DELTA[dir];
		int8_t dCol = COL_DELTA[dir];

		int8_t currRow = row + dRow;
		int8_t currCol = col + dCol;

		// Loop until last matching piece
		while ((currRow < LED_ROWS) && (currRow >= 0) &&
			   (currCol < LED_COLS) && (currCol >= 0)) {
			if (leds[currRow * LED_COLS + currCol] == opposite) {
				break;
			}

			currRow += dRow;
			currCol += dCol;
		}

		// Decrease currRow & currCol to last known position
		currRow -=dRow;
		currCol -= dCol;

		dRow = -dRow;
		dCol = -dCol;

		uint8_t availCount = 0;
		uint8_t totalCount = 0;
		uint8_t continuous = 0;
		uint8_t maxContinuous = 0;

		while ((currRow < LED_ROWS) && (currRow >= 0) &&
			   (currCol < LED_COLS) && (currCol >= 0)) {

			ledMode_t mode = leds[currRow * LED_COLS + currCol];

			if (mode == opposite) {
				break;
			}

			totalCount++;

			if ((mode == player) || ((currRow == row) && (currCol == col))) {
				availCount++;
				if (++continuous > maxContinuous) {
					maxContinuous = continuous;
				}
			} else {
				// We need to check if this move is currently valid! Add an extra point if it is valid
				if (get_next_row(currCol) == currRow) {
					availCount++;
				}
				continuous = 0;
			}

			currRow += dRow;
			currCol += dCol;
		}

		availCount += (maxContinuous << 4);

		if ((totalCount >= 4) && (availCount > max)) {
			max = availCount;
		}
	}

	return max;
}

uint8_t eval_move(ledMode_t player, uint8_t col, uint8_t row)
{
	if ((col >= LED_COLS) || (row >= LED_ROWS)) {
		return 0;
	}

	ledMode_t opposition;

	if (player == LED_PLAYER1) {
		opposition = LED_PLAYER2;
	} else if (player == LED_PLAYER2) {
		opposition = LED_PLAYER1;
	} else {
		return 0;
	}

	uint8_t myScore = eval_position(player, col, row);
	uint8_t opScore = eval_position(opposition, col, row);

	if ((myScore >> 4) >= 4) {
		return 0xFF;
	} else if ((opScore >> 4) >= 4) {
		return 0xFE;
	} else if ((opScore >> 4) >= 2) {
		return myScore + opScore / 2;
	}

	return myScore;
}

uint8_t get_next_move()
{
	uint8_t max = 0;
	uint8_t sel_col = 3;

	for (uint8_t col = 0; col < LED_COLS; col++) {
		int8_t row = get_next_row(col);
		if (row < 0) {
			continue;
		}
		uint8_t val = eval_move(COMPUTER_PLAYER, col, row);
		if (val > max) {
			max = val;
			sel_col = col;
		} else if (val == max) {
			// Select the piece closest to the centre
			uint8_t centre = LED_COLS / 2;
			int8_t colDiff = centre - col;
			if (colDiff < 0) {
				colDiff = -colDiff;
			}
			int8_t selDiff = centre - sel_col;
			if (selDiff < 0) {
				selDiff = -selDiff;
			}
			if (colDiff < selDiff) {
				sel_col = col;
			}
		}
	}

	return sel_col;
}

bool has_won(uint8_t col, ledMode_t player)
{
	if (col >= LED_COLS) {
		return false;
	}

	int8_t row = get_next_row(col);

	if (row < 0) {
		row = 0;
	} else if (row < (LED_ROWS - 1)) {
		row++;
	}

	return (eval_position(player, col, row) >> 4) >= 4;
}

int8_t get_cursor()
{
	int8_t valid_col = -1;
	for (uint8_t col = 0; col < LED_COLS; col++) {
		// Check if the column has a valid number
		if (get_next_row(col) >= 0) {
			// Valid - choose if it is closer to the centre
			uint8_t centre = LED_COLS / 2;
			int8_t colDiff = centre - col;
			if (colDiff < 0) {
				colDiff = -colDiff;
			}
			int8_t selDiff = centre - valid_col;
			if (selDiff < 0) {
				selDiff = -selDiff;
			}

			if (colDiff == 0) {
				return col;
			} else if (colDiff < selDiff) {
				valid_col = col;
			}
		}
	}

	return valid_col;
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

  //  MX_USB_PCD_Init();
  /* USER CODE BEGIN 2 */

  driver_init();

//  colDriver_t driver = column_drivers[0];

  // Set the data
//  HAL_GPIO_WritePin(driver.dat1Port, driver.dat1Pin, 1);
//
//  HAL_Delay(1);
////  for (uint16_t i = 0; i < 1000; i++) {
////  	  NOP();
////  }
//
//  for (uint8_t i = 0; i < 17; i++) {
//	  // Toggle the clock
//	    HAL_GPIO_TogglePin(driver.clk1Port, driver.clk1Pin);
//	  //  for (uint16_t i = 0; i < 1000; i++) {
//	  //	  NOP();
//	  //  }
//	    HAL_Delay(1);
//  }


//  place_piece(LED_PLAYER1, 0);
//  place_piece(LED_PLAYER2, 0);
//  place_piece(LED_PLAYER1, 1);
//  place_piece(LED_PLAYER2, 1);
//  place_piece(LED_PLAYER1, 2);
//  place_piece(LED_PLAYER2, 2);
//  place_piece(LED_PLAYER1, 3);
//  place_piece(LED_PLAYER2, 3);
//  place_piece(LED_PLAYER1, 4);
//  place_piece(LED_PLAYER2, 4);
//  place_piece(LED_PLAYER1, 5);
//  place_piece(LED_PLAYER2, 5);
//  place_piece(LED_PLAYER1, 6);
//  place_piece(LED_PLAYER2, 6);

  uint32_t ticks = 0;

  uint8_t leftButtonCount = 0;
  uint8_t rightButtonCount = 0;
  uint8_t selectButtonCount = 0;

  gameState_t state = GAME_PLAYER1;

  int8_t blink_col = 3;
  int8_t prev_blink_col = 3;

  bool blink_on = true;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (ticks < (UINT32_MAX - 10))
  {
    /* USER CODE END WHILE */
	  GPIO_PinState leftButton = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);
	  GPIO_PinState rightButton = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
	  GPIO_PinState selectButton = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8);

	  if (leftButton == GPIO_PIN_SET) {
		  if (++leftButtonCount >= 2) {
			  leftButtonCount = 2;

			  leftButton = GPIO_PIN_RESET;
			  rightButton = GPIO_PIN_RESET;
			  selectButton = GPIO_PIN_RESET;
		  }
	  } else if (leftButtonCount > 0) {
		  if ((state == GAME_PLAYER1) && (--leftButtonCount == 0)) {
			  while (--blink_col >= 0) {
				  if (leds[blink_col] == LED_OFF) {
					  break;
				  }
			  }
			  if (blink_col < 0) {
				  blink_col = prev_blink_col;
			  } else {
				  // We have changed the blink column
				  // Clear previous blink
				  leds[prev_blink_col] = LED_OFF;
				  output_col(prev_blink_col);
				  prev_blink_col = blink_col;
			  }
		  }
	  }

	  if (rightButton == GPIO_PIN_SET) {
		  if (++rightButtonCount >= 2) {
			  rightButtonCount = 2;

			  leftButton = GPIO_PIN_RESET;
			  rightButton = GPIO_PIN_RESET;
			  selectButton = GPIO_PIN_RESET;
		  }
	  } else if (rightButtonCount > 0) {
		  if ((state == GAME_PLAYER1) && (--rightButtonCount == 0)) {
			  while (++blink_col < LED_COLS) {
				  if (leds[blink_col] == LED_OFF) {
					  break;
				  }
			  }
			  if (blink_col >= LED_COLS) {
				  blink_col = prev_blink_col;
			  } else {
				  // Clear previous blink
				  leds[prev_blink_col] = LED_OFF;
				  output_col(prev_blink_col);
				  prev_blink_col = blink_col;
			  }
		  }
	  }
	  if (selectButton == GPIO_PIN_SET) {

		  if (++selectButtonCount >= 2) {
			  selectButtonCount = 2;


			  leftButton = GPIO_PIN_RESET;
			  rightButton = GPIO_PIN_RESET;
			  selectButton = GPIO_PIN_RESET;
		  }
	  } else if (selectButtonCount > 0) {
		  if (--selectButtonCount == 0) {
			  if (state == GAME_WIN_PLAYER1 || state == GAME_WIN_PLAYER2) {
				  // Go back to start state
				  blink_col = 3;
				  prev_blink_col = 3;
				  for (uint8_t i = 0; i < LED_COUNT; i++) {
					  leds[i] = LED_OFF;
				  }
				  for (uint8_t i = 0; i < LED_COLS; i++) {
					  output_col(i);
				  }
				  state = GAME_PLAYER1;
				  winning_pieces_count = 0;
			  } else {
				  // Remove the blinking!!
				  leds[blink_col] = LED_OFF;
				  // Select the current column
				  place_piece(HUMAN_PLAYER, blink_col);

				  // Evaluate if the user has won
				  if (has_won(blink_col, HUMAN_PLAYER)) {
					  // We won
					  state = GAME_WIN_PLAYER1;
					  set_winning_pieces(HUMAN_PLAYER, blink_col);
				  } else {
					  // This is where the "computer" should make its move
					  uint8_t col = get_next_move();

					  place_piece(COMPUTER_PLAYER, col);

					  // Evaluate if the computer has won
					  if (has_won(col, COMPUTER_PLAYER)) {
						  // Computer has won
						  state = GAME_WIN_PLAYER2;
						  set_winning_pieces(COMPUTER_PLAYER, col);
					  } else {
						  // Check where the 'cursor' should be
						  int8_t cursor_pos = get_cursor();

						  if (cursor_pos < 0) {
							  // We have a tie
							  // Set the STAT LED
//							  HAL_GPIO_WritePin(GPIOB)	``
						  } else {
							  blink_col = cursor_pos;
							  prev_blink_col = cursor_pos;
						  }
					  }
				  }
			  }
		  }
	  }

	  if (ticks % 5 == 0) {
		  // Every 500ms
		  if (state == GAME_PLAYER1) {
			  // Blink on position
			  leds[blink_col] = blink_on * HUMAN_PLAYER;
			  // Output the blink
			  output_col(blink_col);
		  } else if ((state == GAME_WIN_PLAYER1) || (state == GAME_WIN_PLAYER2)) {
			  ledMode_t mode = (state == GAME_WIN_PLAYER1) ? LED_PLAYER1 : LED_PLAYER2;
			  // Flash the winning pieces
			  uint8_t refresh_bitmask = 0x00;
			  for (uint8_t i = 0; i < winning_pieces_count; i++) {
				  leds[winning_pieces[i].row * LED_COLS + winning_pieces[i].col] = mode * blink_on;
				  refresh_bitmask |= 1 << winning_pieces[i].col;
			  }

			  for (uint8_t i = 0; i < LED_COLS; i++) {
				  if (refresh_bitmask & (1 << i)) {
					  output_col(i);
				  }
			  }
		  }
		  // Change blink
		  blink_on = !blink_on;
	  }

	  HAL_Delay(100);
	  ticks++;
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA3 PA4 PA5
                           PA6 PA7 PA8 PA9
                           PA10 PA13 PA14 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10
                           PB11 PB12 PB13 PB14
                           PB15 PB3 PB4 PB5
                           PB6 PB7 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
