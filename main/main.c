/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
// #include "led_strip.h"
#include "sdkconfig.h"
// #include "driver/spi_master.h"
#include <esp_task_wdt.h>

#include "esp_timer.h"	// for measurement of function execution time

#include "rom/ets_sys.h"        // for us delay

#include "freertos/timers.h"	// for display timer
#include "freertos/semphr.h"	// for mutex

// Only on a different type of ESP32:
// #include "components/driver/parlio/include/driver/parlio_tx.h"

#include "rlcd_lib/ls012b7dd06.h"
// #include "rlcd_lib/i2s_parallel_driver/i2s_parallel.h"
// #include "rlcd_lib/ls012b7dd06_hal.h"
#include "rlcd_lib/graphics.h"

#define LCD_FRAME_UPDATE_PERIOD 500//40

// 
// To do:
// 	-	BCK flip-flop flips its logic sometimes - add some reset logic
// 		hardware or software with hardware input

// Display log on the LCD:
// https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/log.html
// vprintf_like_t esp_log_set_vprintf(vprintf_like_t func)

/* ESP32 strapping pins info
	Dual Edge Asymmetric Waveform - Active Low

	strapping pins:
	32, 33, 20, 21, 26, 12, 13, 14, 15,
	28, 29, 30, 31

	try:
	19, 22, 1, 3

	safe:
	25, 27, 

	despite pull-down in high state are pins:
	0, 2, 4

	BCK - GCK x
	VA - GSP x
	VB - BSP x
*/

static const char app_tag[] = "RLCD test";

static TimerHandle_t lcd_timer_handle = NULL;

static SemaphoreHandle_t mutex_handle;

bool is_lcd_enabled = false;

void task_LCDSendFrame( void* pvParameters ){
	if( is_lcd_enabled ){
		if( xSemaphoreTake( mutex_handle, 0 ) == pdTRUE ){
			vTaskDelay( 10 / portTICK_PERIOD_MS );
			rlcd_sendFrame();
			// testTransmit();

			xSemaphoreGive( mutex_handle );
		}
		else {
			ESP_LOGW( app_tag, "Could not transmit frame, mutex taken." );
		}
	}
	vTaskDelete(NULL);
}

void task_LCDResume( void* pvParameters ){
	if( is_lcd_enabled == false ){
		if( xSemaphoreTake( mutex_handle, 0 ) == pdTRUE ){
			ESP_LOGI( app_tag, "Resuming LCD." );
			
			rlcd_resume();
			is_lcd_enabled = true;

			xSemaphoreGive( mutex_handle );
		}
		else {
			ESP_LOGW( app_tag, "Could not resume LCD, mutex taken." );
		}
	}
	vTaskDelete(NULL);
}

void task_LCDSuspend( void* pvParameters ){
	if( is_lcd_enabled ){
		if( xSemaphoreTake( mutex_handle, 0 ) == pdTRUE ){
			ESP_LOGI( app_tag, "Suspending LCD." );
			
			rlcd_suspend();
			is_lcd_enabled = false;

			xSemaphoreGive( mutex_handle );
		}
		else {
			ESP_LOGW( app_tag, "Could not suspend LCD, mutex taken." );
		}
	}
	vTaskDelete(NULL);
}

void task_LCDFillWhite( void* pvParameters ){
	if( is_lcd_enabled ){
		if( xSemaphoreTake( mutex_handle, 0 ) == pdTRUE ){
			ESP_LOGI( app_tag, "Filling white." );
			
			rlcd_fillImageWhite();

			rlcd_drawLine( 31, 31, 127, 127, COLOUR_RED );

			// rlcd_drawChar( 100, 100, 'A', COLOUR_MAGENTA, COLOUR_CYAN, 1 );
			// rlcd_drawChar( 120, 100, 'b', COLOUR_RED, COLOUR_WHITE, 2 );
			

			rlcd_updateImageBuf();

			xSemaphoreGive( mutex_handle );
		}
		else {
			ESP_LOGW( app_tag, "Could fill LCD image, mutex taken." );
		}
	}
	vTaskDelete(NULL);
}

void task_LCDFillRed( void* pvParameters ){
	if( is_lcd_enabled ){
		if( xSemaphoreTake( mutex_handle, 0 ) == pdTRUE ){
			ESP_LOGI( app_tag, "Filling red." );

			rlcd_fillImageColour( COLOUR_RED );
			rlcd_updateImageBuf();

			xSemaphoreGive( mutex_handle );
		}
		else {
			ESP_LOGW( app_tag, "Could fill LCD image, mutex taken." );
		}
	}
	vTaskDelete(NULL);
}

void task_LCDFillGreen( void* pvParameters ){
	if( is_lcd_enabled ){
		if( xSemaphoreTake( mutex_handle, 0 ) == pdTRUE ){
			ESP_LOGI( app_tag, "Filling green." );
			
			rlcd_fillImageColour( COLOUR_GREEN );
			rlcd_updateImageBuf();

			xSemaphoreGive( mutex_handle );
		}
		else {
			ESP_LOGW( app_tag, "Could fill LCD image, mutex taken." );
		}
	}
	vTaskDelete(NULL);
}

void task_LCDFillBlue( void* pvParameters ){
	if( is_lcd_enabled ){
		if( xSemaphoreTake( mutex_handle, 0 ) == pdTRUE ){
			ESP_LOGI( app_tag, "Filling blue." );

			rlcd_fillImageColour( COLOUR_BLUE );
			rlcd_updateImageBuf();

			xSemaphoreGive( mutex_handle );
		}
		else {
			ESP_LOGW( app_tag, "Could fill LCD image, mutex taken." );
		}
	}
	vTaskDelete(NULL);
}


// 
// Colour test.
// Draw 8*8 squares with every possible colour.
// 
void task_LCDColourTest( void* pvParameters ){
	if( is_lcd_enabled ){
		if( xSemaphoreTake( mutex_handle, 0 ) == pdTRUE ){
			ESP_LOGI( app_tag, "Displaying colour test." );

			const int16_t square_size = 16;
			const int16_t x_start = ( RLCD_DISP_W - 8*square_size ) / 2;
			const int16_t y_start = x_start;

			int16_t x = x_start, y = y_start;

			rlcd_fillImageColour( COLOUR_BLACK );

			for( uint8_t i = 1; i < 64; i++ ){
				rlcd_drawFillRect( x, y,
									square_size, square_size,
									i );
				
				x += square_size;

				if( (i % 8 == 0) && (i != 0) ){
					y += square_size;
					x = x_start;
				}
			}
			// And the last square black
			rlcd_drawFillRect( x, y,
							   square_size, square_size,
							   0 );


			uint64_t start = esp_timer_get_time();
			rlcd_updateImageBuf();
			uint64_t end = esp_timer_get_time();

			ESP_LOGI( app_tag, "task_LCDColourTest(): rlcd_updateImageBuf() took %llu us.", (end - start) );

			xSemaphoreGive( mutex_handle );
		}
		else {
			ESP_LOGW( app_tag, "Could perform LCD colour test, mutex taken." );
		}
	}
	vTaskDelete(NULL);
}

void lcdTimerCallback( TimerHandle_t xTimer ){
	xTaskCreate( task_LCDSendFrame, "RLCD_rlcd_display_task", 2048, NULL, 20, NULL );
}

// #define TEST_BUF_SIZE 76800
// uint8_t test_buf[TEST_BUF_SIZE] = {0};

// void swapDataPairs(){
// 	uint8_t temp1, temp2;
//     for( uint32_t i = 0; i < TEST_BUF_SIZE; i += 4 ){
//         temp1 = test_buf[i];
//         temp2 = test_buf[i+1];

//         test_buf[i] = test_buf[i+2];
//         test_buf[i+1] = test_buf[i+3];

//         test_buf[i+2] = temp1;
//         test_buf[i+3] = temp2;
//     }
// }

#define MEASURE_TIME_TASK_PRIORITY 10

// void task_measureFuncTime( void* pvParameters ){
// 	const unsigned MEASUREMENT_CNT = 5000;

// 	uint64_t start = esp_timer_get_time();
// 	for( int retries = 0; retries < MEASUREMENT_CNT; retries++ ){
// 		swapDataPairs();
// 	}
// 	uint64_t end = esp_timer_get_time();

// 	ESP_LOGI( app_tag, "task_measureFuncTime(): swapDataPairs() %u iterations took %llu milliseconds (%llu microseconds per invocation) at task priotity %d\n",
//            MEASUREMENT_CNT, (end - start)/1000, (end - start)/MEASUREMENT_CNT, MEASURE_TIME_TASK_PRIORITY );
// }

void task_measureFuncTime2( void* pvParameters ){
	const unsigned MEASUREMENT_CNT = 100;

	uint64_t start = esp_timer_get_time();
	for( int retries = 0; retries < MEASUREMENT_CNT; retries++ ){
		rlcd_updateImageBuf();
	}
	uint64_t end = esp_timer_get_time();

	ESP_LOGI( app_tag, "task_measureFuncTime(): rlcd_updateImageBuf() %u iterations took %llu milliseconds (%llu microseconds per invocation) at task priotity %d\n",
           MEASUREMENT_CNT, (end - start)/1000, (end - start)/MEASUREMENT_CNT, MEASURE_TIME_TASK_PRIORITY );
}

void app_main(void) {

	// esp_log_set_vprintf( rlcd_vprintf_func );

    // Not needed, WDT init done automatically for OS tasks.
    // const esp_task_wdt_config_t wdt_config = {
    //     .timeout_ms = 1,
    //     .idle_core_mask = ???,
    //     .trigger_panic = true,
    // }
    // esp_task_wdt_init( &wdt_config ); //enable panic so ESP32 restarts

	// Measure functions
	// xTaskCreate( task_measureFuncTime, "task_measureFuncTime", 2048, NULL, MEASURE_TIME_TASK_PRIORITY, NULL );
	// xTaskCreate( task_measureFuncTime2, "task_measureFuncTime2", 2048, NULL, MEASURE_TIME_TASK_PRIORITY, NULL );


	rlcd_init();
	rlcd_fillImageWhite();

	// xTaskCreate( task_measureFuncTime2, "task_measureFuncTime2", 2048, NULL, MEASURE_TIME_TASK_PRIORITY, NULL );

	// Create a mutex for LCD routines
	mutex_handle = xSemaphoreCreateMutex();

	is_lcd_enabled = true;

	// Create LCD-frame-updating timer
	lcd_timer_handle = xTimerCreate(
							"LCD timer",
							LCD_FRAME_UPDATE_PERIOD / portTICK_PERIOD_MS,
							pdTRUE,
							(void*)0,
							lcdTimerCallback
						);

	if( lcd_timer_handle == NULL ){
		ESP_LOGW( app_tag, "Could not create LCD timer." );
	}
	else {
		vTaskDelay( 1000 / portTICK_PERIOD_MS );
		xTimerStart( lcd_timer_handle, portMAX_DELAY );
	}

    // esp_task_wdt_add(NULL); //add current thread to WDT watch

    ESP_LOGI( app_tag, "Program configured to test RLCD interface." );
	
	ESP_LOGI( app_tag, "LCD should be enabled." );
	ESP_LOGI( app_tag, "Press D to suspend display or E to resume it." );

	// Display start / stop terminal
	while(1){
		char in_char = getchar();
		switch( in_char ){
			case 'e':
			case 'E':
				xTaskCreate( task_LCDResume, "RLCD_rlcd_resume_task", 2048, NULL, 10, NULL );
				break;
			case 'd':
			case 'D':
				xTaskCreate( task_LCDSuspend, "RLCD_rlcd_suspend_task", 2048, NULL, 10, NULL );
				break;
			case 'w':
			case 'W':
				xTaskCreate( task_LCDFillWhite, "RLCD_rlcd_fill_white_task", 2048, NULL, 10, NULL );
				break;
			case 'r':
			case 'R':
				xTaskCreate( task_LCDFillRed, "RLCD_rlcd_fill_red_task", 2048, NULL, 10, NULL );
				break;
			case 'g':
			case 'G':
				xTaskCreate( task_LCDFillGreen, "RLCD_rlcd_fill_green_task", 2048, NULL, 10, NULL );
				break;
			case 'b':
			case 'B':
				xTaskCreate( task_LCDFillBlue, "RLCD_rlcd_fill_blue_task", 2048, NULL, 10, NULL );
				break;
			case 'c':
			case 'C':
				xTaskCreate( task_LCDColourTest, "RLCD_rlcd_colour_test_task", 2048, NULL, 10, NULL );
				break;
			default:
				in_char = 0;
		}

		if( (in_char) && (in_char != 'd') && (in_char != 'D') ){
			xTaskCreate( task_LCDSendFrame, "RLCD_rlcd_display_task", 2048, NULL, 20, NULL );
			in_char = 0;
		}

		vTaskDelay( 50 / portTICK_PERIOD_MS );
	}

	// a) and c) the same and
	// b) and d) the same
	// so tx/rx_msb_right = 1/0 makes no difference in the output signals
	// a) differs from b) by having 2 additional clock edges before and after transmitted data

	// the same for e)-h) which are the same as a)-d)

	// So tx/rx_msb_right belong to the serial mode flags
	// and tx/rx_right_first, when set, make WS active low.

    // while (1) {
        // ESP_LOGI( app_tag, "Turning the LED %s!", s_led_state == true ? "ON" : "OFF" );

		/* // Interactive config selector
		char in_char = getchar();//fgetc( stdin );
		switch( in_char ){
			
			case 'a':
			case 'A':
				ESP_LOGI( app_tag, "Loading I2S config %c.", in_char );
				i2s_stop( &I2S1 );
				cfg.tx_right_first = 0;
				cfg.rx_right_first = 0;
				cfg.tx_msb_right = 0;
				cfg.rx_msb_right = 0;
				cfg.tx_chan_mod = 1;
				cfg.rx_chan_mod = 1;
				rlcd_init( &cfg );
				break;

			case 'r':
			case 'R':

			case 'b':
			case 'B':
				ESP_LOGI( app_tag, "Loading I2S config %c.", in_char );
				i2s_stop( &I2S1 );
				cfg.tx_right_first = 1;
				cfg.rx_right_first = 1;
				cfg.tx_msb_right = 0;
				cfg.rx_msb_right = 0;
				cfg.tx_chan_mod = 1;
				cfg.rx_chan_mod = 1;
				rlcd_init( &cfg );
				break;
			case 'c':
			case 'C':
				ESP_LOGI( app_tag, "Loading I2S config %c.", in_char );
				i2s_stop( &I2S1 );
				cfg.tx_right_first = 0;
				cfg.rx_right_first = 0;
				cfg.tx_msb_right = 1;
				cfg.rx_msb_right = 1;
				cfg.tx_chan_mod = 1;
				cfg.rx_chan_mod = 1;
				rlcd_init( &cfg );
				break;
			case 'd':
			case 'D':
				ESP_LOGI( app_tag, "Loading I2S config %c.", in_char );
				i2s_stop( &I2S1 );
				cfg.tx_right_first = 1;
				cfg.rx_right_first = 1;
				cfg.tx_msb_right = 1;
				cfg.rx_msb_right = 1;
				cfg.tx_chan_mod = 1;
				cfg.rx_chan_mod = 1;
				rlcd_init( &cfg );
				break;
			

			case 'e':
			case 'E':
				ESP_LOGI( app_tag, "Loading I2S config %c.", in_char );
				i2s_stop( &I2S1 );
				cfg.tx_right_first = 0;
				cfg.rx_right_first = 0;
				cfg.tx_msb_right = 0;
				cfg.rx_msb_right = 0;
				cfg.tx_chan_mod = 2;
				cfg.rx_chan_mod = 2;
				rlcd_init( &cfg );
				break;

			case 'f':
			case 'F':
				ESP_LOGI( app_tag, "Loading I2S config %c.", in_char );
				i2s_stop( &I2S1 );
				cfg.tx_right_first = 1;
				cfg.rx_right_first = 1;
				cfg.tx_msb_right = 0;
				cfg.rx_msb_right = 0;
				cfg.tx_chan_mod = 2;
				cfg.rx_chan_mod = 2;
				rlcd_init( &cfg );
				break;
			case 'g':
			case 'G':
				ESP_LOGI( app_tag, "Loading I2S config %c.", in_char );
				i2s_stop( &I2S1 );
				cfg.tx_right_first = 0;
				cfg.rx_right_first = 0;
				cfg.tx_msb_right = 1;
				cfg.rx_msb_right = 1;
				cfg.tx_chan_mod = 2;
				cfg.rx_chan_mod = 2;
				rlcd_init( &cfg );
				break;
			case 'h':
			case 'H':
				ESP_LOGI( app_tag, "Loading I2S config %c.", in_char );
				i2s_stop( &I2S1 );
				cfg.tx_right_first = 1;
				cfg.rx_right_first = 1;
				cfg.tx_msb_right = 1;
				cfg.rx_msb_right = 1;
				cfg.tx_chan_mod = 2;
				cfg.rx_chan_mod = 2;
				rlcd_init( &cfg );
				break;
			
			case 'i':
			case 'I':
				ESP_LOGI( app_tag, "Loading I2S config %c.", in_char );
				i2s_stop( &I2S1 );
				cfg.tx_right_first = 0;
				cfg.rx_right_first = 0;
				cfg.tx_msb_right = 0;
				cfg.rx_msb_right = 0;
				cfg.tx_chan_mod = 3;
				cfg.rx_chan_mod = 3;
				rlcd_init( &cfg );
				break;

			case 'j':
			case 'J':
				ESP_LOGI( app_tag, "Loading I2S config %c.", in_char );
				i2s_stop( &I2S1 );
				cfg.tx_right_first = 1;
				cfg.rx_right_first = 1;
				cfg.tx_msb_right = 0;
				cfg.rx_msb_right = 0;
				cfg.tx_chan_mod = 3;
				cfg.rx_chan_mod = 3;
				rlcd_init( &cfg );
				break;
			case 'k':
			case 'K':
				ESP_LOGI( app_tag, "Loading I2S config %c.", in_char );
				i2s_stop( &I2S1 );
				cfg.tx_right_first = 0;
				cfg.rx_right_first = 0;
				cfg.tx_msb_right = 1;
				cfg.rx_msb_right = 1;
				cfg.tx_chan_mod = 3;
				cfg.rx_chan_mod = 3;
				rlcd_init( &cfg );
				break;
			case 'l':
			case 'L':
				ESP_LOGI( app_tag, "Loading I2S config %c.", in_char );
				i2s_stop( &I2S1 );
				cfg.tx_right_first = 1;
				cfg.rx_right_first = 1;
				cfg.tx_msb_right = 1;
				cfg.rx_msb_right = 1;
				cfg.tx_chan_mod = 3;
				cfg.rx_chan_mod = 3;
				rlcd_init( &cfg );
				break;

			
			case 'm':
			case 'M':
				ESP_LOGI( app_tag, "Loading I2S config %c.", in_char );
				i2s_stop( &I2S1 );
				cfg.tx_right_first = 0;
				cfg.rx_right_first = 0;
				cfg.tx_msb_right = 0;
				cfg.rx_msb_right = 0;
				cfg.tx_chan_mod = 4;
				cfg.rx_chan_mod = 4;
				rlcd_init( &cfg );
				break;

			case 'n':
			case 'N':
				ESP_LOGI( app_tag, "Loading I2S config %c.", in_char );
				i2s_stop( &I2S1 );
				cfg.tx_right_first = 1;
				cfg.rx_right_first = 1;
				cfg.tx_msb_right = 0;
				cfg.rx_msb_right = 0;
				cfg.tx_chan_mod = 4;
				cfg.rx_chan_mod = 4;
				rlcd_init( &cfg );
				break;
			case 'o':
			case 'O':
				ESP_LOGI( app_tag, "Loading I2S config %c.", in_char );
				i2s_stop( &I2S1 );
				cfg.tx_right_first = 0;
				cfg.rx_right_first = 0;
				cfg.tx_msb_right = 1;
				cfg.rx_msb_right = 1;
				cfg.tx_chan_mod = 4;
				cfg.rx_chan_mod = 4;
				rlcd_init( &cfg );
				break;
			case 'p':
			case 'P':
				ESP_LOGI( app_tag, "Loading I2S config %c.", in_char );
				i2s_stop( &I2S1 );
				cfg.tx_right_first = 1;
				cfg.rx_right_first = 1;
				cfg.tx_msb_right = 1;
				cfg.rx_msb_right = 1;
				cfg.tx_chan_mod = 4;
				cfg.rx_chan_mod = 4;
				rlcd_init( &cfg );
				break;

			default:
				break;
		}
		*/

		// ESP_LOGI( app_tag, "Starting transmission..." );
		// rlcd_sendFrame();
		
		// int active_dma_desc_idx = -1;
		// getInvalidFlag( &I2S1, &active_dma_desc_idx );
		// // if( getInvalidFlag( &active_dma_desc_idx ) )
		// 	// ESP_LOGW( app_tag, "Got invalid DMA descriptor no %d.", active_dma_desc_idx );
		
        // esp_task_wdt_reset();

		// ets_delay_us( 14 );

        // vTaskDelay(10 / portTICK_PERIOD_MS);
        // i2s_setStopSignal();
        // rlcd_testGPIOs( s_led_state );
        
        /* Toggle the LED state */
        // s_led_state = !s_led_state;

		// for( uint8_t i=0; i < 4; i++ )
		// 	rlcd_sendFrame();

		// vTaskDelay( 40 / portTICK_PERIOD_MS );

		// rlcd_suspend();

        // vTaskDelay( 5000 / portTICK_PERIOD_MS );

		// rlcd_resume();
    // }
}

/* void printBin( uint8_t n ){
	printf("0b");
	for( uint8_t i=0; i<8; i++ ){
		if (n & 0b10000000)
			printf("1");
		else
			printf("0");

		n <<= 1;
	}
	printf("\n");
}
*/
/* void colourTest( void ){
    printf( "sizeof(lcd_colour_t) = %lld\n", sizeof(lcd_colour_t) );

	lcd_colour_t col;
	col.bits = 0;
	printf( "Init:\t" );
	printBin( col.bits );

	col.r = 1;
	printf( "r=1:\t" );
	printBin( col.bits );
	col.r = 2;
	printf( "r=2:\t" );
	printBin( col.bits );
	col.r = 3;
	printf( "r=3:\t" );
	printBin( col.bits );

	col.r = 0;
	col.g = 1;
	printf( "g=1:\t" );
	printBin( col.bits );
	col.g = 2;
	printf( "g=2:\t" );
	printBin( col.bits );
	col.g = 3;
	printf( "g=3:\t" );
	printBin( col.bits );

	col.g = 0;
	col.b = 1;
	printf( "b=1:\t" );
	printBin( col.bits );
	col.b = 2;
	printf( "b=2:\t" );
	printBin( col.bits );
	col.b = 3;
	printf( "b=3:\t" );
	printBin( col.bits );
}
*/