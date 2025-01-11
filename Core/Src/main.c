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
#include "usb_speaker.h"
#include "volume_ctrl.h"
#include "speaker_settings.h"
#include "ring_buf.h"
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
I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi3_tx;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
speaker_settings_t speaker_settings;

// Buffer for speaker data
ring_buf_t spk_ring_buffer;
uint32_t *spk_ring_buffer_storage;

// Buffer for speaker data
uint8_t spk_usb_read_buf[SAMPLE_BUFFER_SIZE * 2 * 4 * 2]; // Max size of audio sample is  2 * 4. 2 Channels, 4 byte width sample
i2s_32b_audio_sample spk_32b_i2s_buffer[SAMPLE_BUFFER_SIZE];
i2s_16b_audio_sample spk_16b_i2s_buffer[SAMPLE_BUFFER_SIZE];
uint32_t spk_i2s_buf[SAMPLE_BUFFER_SIZE * 2];

void led_blinking_task(void);

int32_t usb_to_i2s_32b_sample_convert(int32_t sample, uint32_t volume_db);

int16_t usb_to_i2s_16b_sample_convert(int16_t sample, uint32_t volume_db);

HAL_StatusTypeDef refresh_i2s_spk(void);
void refresh_i2s_connections(void);
HAL_StatusTypeDef MyHAL_I2S_Init(I2S_HandleTypeDef *hi2s);
void MyHAL_I2S_MspInit(I2S_HandleTypeDef *hi2s);

void usb_speaker_mute_handler(int8_t bChannelNumber, int8_t mute_in);
void usb_speaker_volume_handler(int8_t bChannelNumber, int16_t volume_in);
void usb_speaker_current_sample_rate_handler(uint32_t current_sample_rate_in);
void usb_speaker_current_resolution_handler(uint8_t current_resolution_in);
void usb_speaker_current_status_set_handler(uint32_t blink_interval_ms_in);

void usb_speaker_tud_audio_rx_done_pre_read_handler(uint8_t rhport,
		uint16_t n_bytes_received, uint8_t func_id, uint8_t ep_out,
		uint8_t cur_alt_setting);

int spk_machine_i2s_write_stream(uint32_t *buf_in, uint32_t size);
void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s);
void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s);
uint32_t feed_spk_dma(uint32_t *dma_buffer_p,
		uint32_t sizeof_half_dma_buffer_in_bytes);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S3_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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
	MX_DMA_Init();
	MX_I2S3_Init();
	MX_USB_OTG_FS_PCD_Init();
	/* USER CODE BEGIN 2 */

	speaker_settings.sample_rate = I2S_SPK_RATE_DEF;
	speaker_settings.resolution = CFG_TUD_AUDIO_FUNC_1_FORMAT_1_RESOLUTION_RX;
	speaker_settings.blink_interval_ms = BLINK_NOT_MOUNTED;
	speaker_settings.status_updated = false;

	usb_speaker_set_mute_set_handler(usb_speaker_mute_handler);
	usb_speaker_set_volume_set_handler(usb_speaker_volume_handler);
	usb_speaker_set_current_sample_rate_set_handler(
			usb_speaker_current_sample_rate_handler);
	usb_speaker_set_current_resolution_set_handler(
			usb_speaker_current_resolution_handler);
	usb_speaker_set_current_status_set_handler(
			usb_speaker_current_status_set_handler);

	usb_speaker_set_tud_audio_rx_done_pre_read_set_handler(
			usb_speaker_tud_audio_rx_done_pre_read_handler);

	usb_speaker_init();
	refresh_i2s_connections();

	for (int i = 0; i < (CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1); i++) {
		speaker_settings.volume[i] = DEFAULT_VOLUME;
		speaker_settings.mute[i] = 0;
		speaker_settings.volume_db[i] = vol_to_db_convert_enc(
				speaker_settings.mute[i], speaker_settings.volume[i]);
	}

	for (int i = 0; i < (CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX); i++) {
		speaker_settings.volume_mul_db[i] = speaker_settings.volume_db[0]
				* speaker_settings.volume_db[i + 1];
	}

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		usb_speaker_task();

		led_blinking_task();

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

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
	RCC_OscInitStruct.PLL.PLLM = 25;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2S3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2S3_Init(void) {

	/* USER CODE BEGIN I2S3_Init 0 */

	/* USER CODE END I2S3_Init 0 */

	/* USER CODE BEGIN I2S3_Init 1 */

	/* USER CODE END I2S3_Init 1 */
	hi2s3.Instance = SPI3;
	hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
	hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
	hi2s3.Init.DataFormat = I2S_DATAFORMAT_32B;
	hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
	hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_44K;
	hi2s3.Init.CPOL = I2S_CPOL_LOW;
	hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
	hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
	if (HAL_I2S_Init(&hi2s3) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2S3_Init 2 */

	/* USER CODE END I2S3_Init 2 */

}

/**
 * @brief USB_OTG_FS Initialization Function
 * @param None
 * @retval None
 */
static void MX_USB_OTG_FS_PCD_Init(void) {

	/* USER CODE BEGIN USB_OTG_FS_Init 0 */

	/* USER CODE END USB_OTG_FS_Init 0 */

	/* USER CODE BEGIN USB_OTG_FS_Init 1 */

	/* USER CODE END USB_OTG_FS_Init 1 */
	hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
	hpcd_USB_OTG_FS.Init.dev_endpoints = 4;
	hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
	hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
	hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
	hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
	hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
	hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
	hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
	hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
	if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USB_OTG_FS_Init 2 */

	/* USER CODE END USB_OTG_FS_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Stream5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(ONBOARD_LED_GPIO_Port, ONBOARD_LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : ONBOARD_LED_Pin */
	GPIO_InitStruct.Pin = ONBOARD_LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(ONBOARD_LED_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void refresh_i2s_connections(void) {
	HAL_I2S_StateTypeDef dma_state;

	// Stop microphone DMA
	dma_state = HAL_I2S_GetState(&hi2s3);
	if (dma_state != HAL_I2S_STATE_READY) {
		HAL_StatusTypeDef stop_status = HAL_I2S_DMAStop(&hi2s3);
		if (stop_status != HAL_OK) {
			Error_Handler();
		}
	}

	HAL_StatusTypeDef tx_i2s_status = refresh_i2s_spk();
	if (tx_i2s_status != HAL_OK) {
		Error_Handler();
	}

	if (spk_ring_buffer_storage != NULL) {
		free(spk_ring_buffer_storage);
	}

	speaker_settings.samples_in_i2s_frame_min = (speaker_settings.sample_rate)
			/ 1000;
	speaker_settings.samples_in_i2s_frame_max = (speaker_settings.sample_rate
			+ 999) / 1000;

	// Ring buffer contains 2 ms data
	spk_ring_buffer_storage = m_new(uint32_t,
			speaker_settings.samples_in_i2s_frame_max * 2 * 2);
	ringbuf_init(&spk_ring_buffer, spk_ring_buffer_storage,
			speaker_settings.samples_in_i2s_frame_max * 2 * 2);

	// Clear memories
	memset(spk_i2s_buf, 0x0, sizeof(spk_i2s_buf));

	uint16_t num_of_samples_in_buffer =
			speaker_settings.samples_in_i2s_frame_min * 2;

	// Run transmit DMA
	HAL_StatusTypeDef tx_status = HAL_I2S_Transmit_DMA(&hi2s3, spk_i2s_buf,
			num_of_samples_in_buffer);
	if (tx_status != HAL_OK) {
		Error_Handler();
	}
}

HAL_StatusTypeDef refresh_i2s_spk(void) {
	HAL_StatusTypeDef status = HAL_I2S_DeInit(&hi2s3);
	if (status != HAL_OK)
		return status;

	hi2s3.Instance = SPI3;
	hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
	hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
	hi2s3.Init.DataFormat =
			(speaker_settings.resolution == 16) ?
			I2S_DATAFORMAT_16B :
													I2S_DATAFORMAT_32B; //I2S_DATAFORMAT_32B;
	hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
	hi2s3.Init.AudioFreq = speaker_settings.sample_rate; //I2S_AUDIOFREQ_48K;
	hi2s3.Init.CPOL = I2S_CPOL_LOW;
	hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
	hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
	if (MyHAL_I2S_Init(&hi2s3) != HAL_OK) {
		Error_Handler();
	}

	return HAL_OK;
}

//-------------------------
// callback functions
//-------------------------

void usb_speaker_mute_handler(int8_t bChannelNumber, int8_t mute_in) {
	speaker_settings.mute[bChannelNumber] = mute_in;
	speaker_settings.volume_db[bChannelNumber] = vol_to_db_convert_enc(
			speaker_settings.mute[bChannelNumber],
			speaker_settings.volume[bChannelNumber]);

	for (int i = 0; i < (CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX); i++) {
		speaker_settings.volume_mul_db[i] = speaker_settings.volume_db[0]
				* speaker_settings.volume_db[i + 1];
	}

	speaker_settings.status_updated = true;
}

void usb_speaker_volume_handler(int8_t bChannelNumber, int16_t volume_in) {
	speaker_settings.volume[bChannelNumber] = volume_in;
	speaker_settings.volume_db[bChannelNumber] = vol_to_db_convert_enc(
			speaker_settings.mute[bChannelNumber],
			speaker_settings.volume[bChannelNumber]);

	for (int i = 0; i < (CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX); i++) {
		speaker_settings.volume_mul_db[i] = speaker_settings.volume_db[0]
				* speaker_settings.volume_db[i + 1];
	}

	speaker_settings.status_updated = true;
}

void usb_speaker_current_sample_rate_handler(uint32_t current_sample_rate_in) {
	speaker_settings.sample_rate = current_sample_rate_in;
	refresh_i2s_connections();
	speaker_settings.status_updated = true;
}

void usb_speaker_current_resolution_handler(uint8_t current_resolution_in) {
	speaker_settings.resolution = current_resolution_in;
	refresh_i2s_connections();
	speaker_settings.status_updated = true;
}

void usb_speaker_current_status_set_handler(uint32_t blink_interval_ms_in) {
	speaker_settings.blink_interval_ms = blink_interval_ms_in;
	speaker_settings.status_updated = true;
}

void usb_speaker_tud_audio_rx_done_pre_read_handler(uint8_t rhport,
		uint16_t n_bytes_received, uint8_t func_id, uint8_t ep_out,
		uint8_t cur_alt_setting) {
	uint32_t volume_db_left = speaker_settings.volume_mul_db[0];
	uint32_t volume_db_right = speaker_settings.volume_mul_db[1];

	if (speaker_settings.blink_interval_ms == BLINK_STREAMING) {
		// Speaker data size received in the last frame
		uint16_t usb_spk_data_size = tud_audio_n_read(func_id, spk_usb_read_buf,
				n_bytes_received);
		//uint16_t usb_spk_data_size = tud_audio_read(spk_usb_read_buf,
		//		n_bytes_received);
		uint16_t usb_sample_count = 0;

		if (speaker_settings.resolution == 16) {
			int16_t *in = (int16_t*) spk_usb_read_buf;
			usb_sample_count = usb_spk_data_size / 4; // 4 bytes per sample 2b left, 2b right

			//if (usb_sample_count >= current_settings.samples_in_i2s_frame_min) {
			for (int i = 0; i < usb_sample_count; i++) {
				int16_t left = in[i * 2 + 0];
				int16_t right = in[i * 2 + 1];

				left = usb_to_i2s_16b_sample_convert(left, volume_db_left);
				right = usb_to_i2s_16b_sample_convert(right, volume_db_right);

				spk_16b_i2s_buffer[i].left = left;
				spk_16b_i2s_buffer[i].right = right;
			}
			spk_machine_i2s_write_stream(&spk_16b_i2s_buffer[0],
					usb_sample_count); // Number of words
			//}
		} else {
			int32_t *in = (int32_t*) spk_usb_read_buf;
			usb_sample_count = usb_spk_data_size / 8; // 8 bytes per sample 4b left, 4b right

			//if (usb_sample_count >= current_settings.samples_in_i2s_frame_min) {
			for (int i = 0; i < usb_sample_count; i++) {
				int32_t left = in[i * 2 + 0];
				int32_t right = in[i * 2 + 1];

				left = usb_to_i2s_32b_sample_convert(left, volume_db_left);
				right = usb_to_i2s_32b_sample_convert(right, volume_db_right);

				spk_32b_i2s_buffer[i].left = left;
				spk_32b_i2s_buffer[i].right = right;
			}
			spk_machine_i2s_write_stream(&spk_32b_i2s_buffer[0],
					usb_sample_count * 2); // Number of words
			//}
		}
	}
}

int32_t usb_to_i2s_32b_sample_convert(int32_t sample, uint32_t volume_db) {
	int64_t sample_tmp = (int64_t) sample * (int64_t) volume_db;
	sample_tmp = sample_tmp >> (15 + 15);
	return (int32_t) sample_tmp;
	//return (int32_t)sample;
}

int16_t usb_to_i2s_16b_sample_convert(int16_t sample, uint32_t volume_db) {
	int64_t sample_tmp = (int64_t) sample * (int64_t) volume_db;
	sample_tmp = sample_tmp >> (15 + 15);
	return (int16_t) sample_tmp;
	//return (int16_t)sample;
}

//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+
void led_blinking_task(void) {
	static uint32_t start_ms = 0;
	static bool led_state = false;

	uint32_t cur_time_ms = board_millis();

	// Blink every interval ms
	if (cur_time_ms - start_ms < speaker_settings.blink_interval_ms)
		return;
	start_ms += speaker_settings.blink_interval_ms;

	board_led_write(led_state);
	led_state = 1 - led_state;
}

int spk_machine_i2s_write_stream(uint32_t *buf_in, uint32_t size) {
	if (size == 0) {
		return 0;
	}

	// copy audio samples from the app buffer to the ring buffer
	// loop, reading samples until the app buffer is emptied
	// for uasyncio mode, the loop will make an early exit if the ring buffer becomes full

	// Not allow buffer overflow
	uint16_t available_space = ringbuf_available_space(&spk_ring_buffer);
	if (available_space <= size) {
		return 0;
	}

	if (speaker_settings.resolution != 16) {
		for (uint32_t a_index = 0; a_index < size; a_index++) {
			if (ringbuf_push_half_word_swap(&spk_ring_buffer,
					buf_in[a_index]) == false) {
				return a_index;
			}
		}
	} else {
		for (uint32_t a_index = 0; a_index < size; a_index++) {
			if (ringbuf_push(&spk_ring_buffer, buf_in[a_index]) == false) {
				return a_index;
			}
		}
	}

	return size;
}

void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
	int num_of_words_feed =
			(speaker_settings.resolution != 16) ?
					speaker_settings.samples_in_i2s_frame_min :
					speaker_settings.samples_in_i2s_frame_min / 2;
	feed_spk_dma(&spk_i2s_buf[0], num_of_words_feed);
}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s) {
	int num_of_words_feed =
			(speaker_settings.resolution != 16) ?
					speaker_settings.samples_in_i2s_frame_min :
					speaker_settings.samples_in_i2s_frame_min / 2;
	feed_spk_dma(&spk_i2s_buf[num_of_words_feed], num_of_words_feed);
}

uint32_t feed_spk_dma(uint32_t *dma_buffer_p,
		uint32_t sizeof_half_dma_buffer_in_words) {
	// when data exists, copy samples from ring buffer
	uint32_t available_data_words = ringbuf_available_data(&spk_ring_buffer);
	if (available_data_words > sizeof_half_dma_buffer_in_words) {
		available_data_words = sizeof_half_dma_buffer_in_words;
	}

	if (available_data_words >= sizeof_half_dma_buffer_in_words) {
		for (uint32_t i = 0; i < sizeof_half_dma_buffer_in_words; i++) {
			ringbuf_pop(&spk_ring_buffer, &dma_buffer_p[i]);
		}
		return available_data_words;
	} else {
		// underflow.  clear buffer to transmit "silence" on the I2S bus
		memset(dma_buffer_p, 0,
				sizeof_half_dma_buffer_in_words * sizeof(uint32_t));
		return sizeof_half_dma_buffer_in_words;
	}
}

void MyHAL_I2S_IRQHandler(I2S_HandleTypeDef *hi2s)
{
  /* Call the IrqHandler ISR set during HAL_I2S_INIT */
  hi2s->IrqHandlerISR(hi2s);
}

HAL_StatusTypeDef MyHAL_I2S_Init(I2S_HandleTypeDef *hi2s) {
	uint32_t i2sdiv;
	uint32_t i2sodd;
	uint32_t packetlength;
	uint32_t tmp;
	uint32_t i2sclk;
#if defined (SPI_I2S_FULLDUPLEX_SUPPORT)
	uint16_t tmpreg;
#endif

	/* Check the I2S handle allocation */
	if (hi2s == NULL) {
		return HAL_ERROR;
	}

	/* Check the I2S parameters */
	assert_param(IS_I2S_ALL_INSTANCE(hi2s->Instance));
	assert_param(IS_I2S_MODE(hi2s->Init.Mode));
	assert_param(IS_I2S_STANDARD(hi2s->Init.Standard));
	assert_param(IS_I2S_DATA_FORMAT(hi2s->Init.DataFormat));
	assert_param(IS_I2S_MCLK_OUTPUT(hi2s->Init.MCLKOutput));
	assert_param(IS_I2S_AUDIO_FREQ(hi2s->Init.AudioFreq));
	assert_param(IS_I2S_CPOL(hi2s->Init.CPOL));
	assert_param(IS_I2S_CLOCKSOURCE(hi2s->Init.ClockSource));

	if (hi2s->State == HAL_I2S_STATE_RESET) {
		/* Allocate lock resource and initialize it */
		hi2s->Lock = HAL_UNLOCKED;

		/* Initialize Default I2S IrqHandler ISR */
		hi2s->IrqHandlerISR = MyHAL_I2S_IRQHandler;

#if (USE_HAL_I2S_REGISTER_CALLBACKS == 1U)
    /* Init the I2S Callback settings */
    hi2s->TxCpltCallback       = HAL_I2S_TxCpltCallback;          /* Legacy weak TxCpltCallback       */
    hi2s->RxCpltCallback       = HAL_I2S_RxCpltCallback;          /* Legacy weak RxCpltCallback       */
#if defined (SPI_I2S_FULLDUPLEX_SUPPORT)
    hi2s->TxRxCpltCallback     = HAL_I2SEx_TxRxCpltCallback;      /* Legacy weak TxRxCpltCallback     */
#endif /* SPI_I2S_FULLDUPLEX_SUPPORT */
    hi2s->TxHalfCpltCallback   = HAL_I2S_TxHalfCpltCallback;      /* Legacy weak TxHalfCpltCallback   */
    hi2s->RxHalfCpltCallback   = HAL_I2S_RxHalfCpltCallback;      /* Legacy weak RxHalfCpltCallback   */
#if defined (SPI_I2S_FULLDUPLEX_SUPPORT)
    hi2s->TxRxHalfCpltCallback = HAL_I2SEx_TxRxHalfCpltCallback;  /* Legacy weak TxRxHalfCpltCallback */
#endif /* SPI_I2S_FULLDUPLEX_SUPPORT */
    hi2s->ErrorCallback        = HAL_I2S_ErrorCallback;           /* Legacy weak ErrorCallback        */

    if (hi2s->MspInitCallback == NULL)
    {
      hi2s->MspInitCallback = HAL_I2S_MspInit; /* Legacy weak MspInit  */
    }

    /* Init the low level hardware : GPIO, CLOCK, NVIC... */
    hi2s->MspInitCallback(hi2s);
#else
		/* Init the low level hardware : GPIO, CLOCK, CORTEX...etc */
		MyHAL_I2S_MspInit(hi2s);
#endif /* USE_HAL_I2S_REGISTER_CALLBACKS */
	}

	hi2s->State = HAL_I2S_STATE_BUSY;

	/*----------------------- SPIx I2SCFGR & I2SPR Configuration ----------------*/
	/* Clear I2SMOD, I2SE, I2SCFG, PCMSYNC, I2SSTD, CKPOL, DATLEN and CHLEN bits */
	CLEAR_BIT(hi2s->Instance->I2SCFGR,
			(SPI_I2SCFGR_CHLEN | SPI_I2SCFGR_DATLEN | SPI_I2SCFGR_CKPOL | SPI_I2SCFGR_I2SSTD | SPI_I2SCFGR_PCMSYNC | SPI_I2SCFGR_I2SCFG | SPI_I2SCFGR_I2SE | SPI_I2SCFGR_I2SMOD));
	hi2s->Instance->I2SPR = 0x0002U;

	/*----------------------- I2SPR: I2SDIV and ODD Calculation -----------------*/
	/* If the requested audio frequency is not the default, compute the prescaler */
	if (hi2s->Init.AudioFreq != I2S_AUDIOFREQ_DEFAULT) {
		/* Check the frame length (For the Prescaler computing) ********************/
		if (hi2s->Init.DataFormat == I2S_DATAFORMAT_16B) {
			/* Packet length is 16 bits */
			packetlength = 16U;
		} else {
			/* Packet length is 32 bits */
			packetlength = 32U;
		}

		/* I2S standard */
		if (hi2s->Init.Standard <= I2S_STANDARD_LSB) {
			/* In I2S standard packet length is multiplied by 2 */
			packetlength = packetlength * 2U;
		}

		/* Get the source clock value **********************************************/
#if defined(I2S_APB1_APB2_FEATURE)
    if (IS_I2S_APB1_INSTANCE(hi2s->Instance))
    {
      i2sclk = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_I2S_APB1);
    }
    else
    {
      i2sclk = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_I2S_APB2);
    }
#else
		i2sclk = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_I2S);
#endif /* I2S_APB1_APB2_FEATURE */

		/* Compute the Real divider depending on the MCLK output state, with a floating point */
		if (hi2s->Init.MCLKOutput == I2S_MCLKOUTPUT_ENABLE) {
			/* MCLK output is enabled */
			if (hi2s->Init.DataFormat != I2S_DATAFORMAT_16B) {
				tmp = (uint32_t) (((((i2sclk / (packetlength * 4U)) * 10U)
						/ hi2s->Init.AudioFreq)) + 5U);
			} else {
				tmp = (uint32_t) (((((i2sclk / (packetlength * 8U)) * 10U)
						/ hi2s->Init.AudioFreq)) + 5U);
			}
		} else {
			/* MCLK output is disabled */
			tmp = (uint32_t) (((((i2sclk / packetlength) * 10U)
					/ hi2s->Init.AudioFreq)) + 5U);
		}

		/* Remove the flatting point */
		tmp = tmp / 10U;

		/* Check the parity of the divider */
		i2sodd = (uint32_t) (tmp & (uint32_t) 1U);

		/* Compute the i2sdiv prescaler */
		i2sdiv = (uint32_t) ((tmp - i2sodd) / 2U);

		/* Get the Mask for the Odd bit (SPI_I2SPR[8]) register */
		i2sodd = (uint32_t) (i2sodd << 8U);
	} else {
		/* Set the default values */
		i2sdiv = 2U;
		i2sodd = 0U;
	}

	/* Test if the divider is 1 or 0 or greater than 0xFF */
	if ((i2sdiv < 2U) || (i2sdiv > 0xFFU)) {
		/* Set the error code and execute error callback*/
		SET_BIT(hi2s->ErrorCode, HAL_I2S_ERROR_PRESCALER);
		return HAL_ERROR;
	}

	/*----------------------- SPIx I2SCFGR & I2SPR Configuration ----------------*/

	/* Write to SPIx I2SPR register the computed value */
	hi2s->Instance->I2SPR = (uint32_t) ((uint32_t) i2sdiv
			| (uint32_t) (i2sodd | (uint32_t) hi2s->Init.MCLKOutput));

	/* Clear I2SMOD, I2SE, I2SCFG, PCMSYNC, I2SSTD, CKPOL, DATLEN and CHLEN bits */
	/* And configure the I2S with the I2S_InitStruct values                      */
	MODIFY_REG(hi2s->Instance->I2SCFGR,
			(SPI_I2SCFGR_CHLEN | SPI_I2SCFGR_DATLEN | SPI_I2SCFGR_CKPOL | SPI_I2SCFGR_I2SSTD | SPI_I2SCFGR_PCMSYNC | SPI_I2SCFGR_I2SCFG | SPI_I2SCFGR_I2SE | SPI_I2SCFGR_I2SMOD),
			(SPI_I2SCFGR_I2SMOD | hi2s->Init.Mode | hi2s->Init.Standard | hi2s->Init.DataFormat | hi2s->Init.CPOL));

#if defined(SPI_I2SCFGR_ASTRTEN)
  if ((hi2s->Init.Standard == I2S_STANDARD_PCM_SHORT) || ((hi2s->Init.Standard == I2S_STANDARD_PCM_LONG)))
  {
    /* Write to SPIx I2SCFGR */
    SET_BIT(hi2s->Instance->I2SCFGR, SPI_I2SCFGR_ASTRTEN);
  }
#endif /* SPI_I2SCFGR_ASTRTEN */

#if defined (SPI_I2S_FULLDUPLEX_SUPPORT)

	/* Configure the I2S extended if the full duplex mode is enabled */
	assert_param(IS_I2S_FULLDUPLEX_MODE(hi2s->Init.FullDuplexMode));

	if (hi2s->Init.FullDuplexMode == I2S_FULLDUPLEXMODE_ENABLE) {
		/* Set FullDuplex I2S IrqHandler ISR if FULLDUPLEXMODE is enabled */
		hi2s->IrqHandlerISR = HAL_I2SEx_FullDuplex_IRQHandler;

		/* Clear I2SMOD, I2SE, I2SCFG, PCMSYNC, I2SSTD, CKPOL, DATLEN and CHLEN bits */
		CLEAR_BIT(I2SxEXT(hi2s->Instance)->I2SCFGR,
				(SPI_I2SCFGR_CHLEN | SPI_I2SCFGR_DATLEN | SPI_I2SCFGR_CKPOL | SPI_I2SCFGR_I2SSTD | SPI_I2SCFGR_PCMSYNC | SPI_I2SCFGR_I2SCFG | SPI_I2SCFGR_I2SE | SPI_I2SCFGR_I2SMOD));
		I2SxEXT(hi2s->Instance)->I2SPR = 2U;

		/* Get the I2SCFGR register value */
		tmpreg = I2SxEXT(hi2s->Instance)->I2SCFGR;

		/* Get the mode to be configured for the extended I2S */
		if ((hi2s->Init.Mode == I2S_MODE_MASTER_TX)
				|| (hi2s->Init.Mode == I2S_MODE_SLAVE_TX)) {
			tmp = I2S_MODE_SLAVE_RX;
		} else /* I2S_MODE_MASTER_RX ||  I2S_MODE_SLAVE_RX */
		{
			tmp = I2S_MODE_SLAVE_TX;
		}

		/* Configure the I2S Slave with the I2S Master parameter values */
		tmpreg |=
				(uint16_t) ((uint16_t) SPI_I2SCFGR_I2SMOD | (uint16_t) tmp
						| (uint16_t) hi2s->Init.Standard
						| (uint16_t) hi2s->Init.DataFormat
						| (uint16_t) hi2s->Init.CPOL);

		/* Write to SPIx I2SCFGR */
		WRITE_REG(I2SxEXT(hi2s->Instance)->I2SCFGR, tmpreg);
	}
#endif /* SPI_I2S_FULLDUPLEX_SUPPORT */

	hi2s->ErrorCode = HAL_I2S_ERROR_NONE;
	hi2s->State = HAL_I2S_STATE_READY;

	return HAL_OK;
}

void MyHAL_I2S_MspInit(I2S_HandleTypeDef *hi2s) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };
	if (hi2s->Instance == SPI3) {
		/* USER CODE BEGIN SPI3_MspInit 0 */

		/* USER CODE END SPI3_MspInit 0 */

		/** Initializes the peripherals clock
		 */
		if (speaker_settings.sample_rate == 48000) {
			/** Initializes the peripherals clock
			 */
			PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
			PeriphClkInitStruct.PLLI2S.PLLI2SN = 258;
			PeriphClkInitStruct.PLLI2S.PLLI2SM = 25;
			PeriphClkInitStruct.PLLI2S.PLLI2SR = 3;
			if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
				Error_Handler();
			}
		}

		if (speaker_settings.sample_rate == 44100) {
			/** Initializes the peripherals clock
			 */
			PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
			PeriphClkInitStruct.PLLI2S.PLLI2SN = 271;
			PeriphClkInitStruct.PLLI2S.PLLI2SM = 25;
			PeriphClkInitStruct.PLLI2S.PLLI2SR = 6;
			if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
				Error_Handler();
			}
		}

		/* Peripheral clock enable */
		__HAL_RCC_SPI3_CLK_ENABLE();

		__HAL_RCC_GPIOB_CLK_ENABLE();
		__HAL_RCC_GPIOA_CLK_ENABLE();
		/**I2S3 GPIO Configuration
		 PB10     ------> I2S3_MCK
		 PA15     ------> I2S3_WS
		 PB3     ------> I2S3_CK
		 PB5     ------> I2S3_SD
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_3 | GPIO_PIN_5;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = GPIO_PIN_15;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		/* I2S3 DMA Init */
		/* SPI3_TX Init */
		hdma_spi3_tx.Instance = DMA1_Stream5;
		hdma_spi3_tx.Init.Channel = DMA_CHANNEL_0;
		hdma_spi3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
		hdma_spi3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_spi3_tx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_spi3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
		hdma_spi3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
		hdma_spi3_tx.Init.Mode = DMA_CIRCULAR;
		hdma_spi3_tx.Init.Priority = DMA_PRIORITY_HIGH;
		hdma_spi3_tx.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
		hdma_spi3_tx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;
		hdma_spi3_tx.Init.MemBurst = DMA_MBURST_SINGLE;
		hdma_spi3_tx.Init.PeriphBurst = DMA_PBURST_SINGLE;
		if (HAL_DMA_Init(&hdma_spi3_tx) != HAL_OK) {
			Error_Handler();
		}

		__HAL_LINKDMA(hi2s, hdmatx, hdma_spi3_tx);

		/* USER CODE BEGIN SPI3_MspInit 1 */

		/* USER CODE END SPI3_MspInit 1 */

	}
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
