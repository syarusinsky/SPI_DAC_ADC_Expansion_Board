#include "../lib/STM32f302x8-HAL/llpd/include/LLPD.hpp"

#include <math.h>

// to disassemble -- arm-none-eabi-objdump -S --disassemble main_debug.elf > disassembled.s

#define SYS_CLOCK_FREQUENCY 72000000

// global variables
constexpr unsigned int bufferSize = 256;
volatile uint16_t adcInVal[bufferSize];
volatile uint16_t dacOutVal[bufferSize];
volatile unsigned int adcDacIncr = 0;
volatile bool audioReady = false;

// peripheral defines
#define ADC_SPI_NUM 		SPI_NUM::SPI_3
#define ADC_CS_PORT 		GPIO_PORT::A
#define ADC_CS_PIN 		GPIO_PIN::PIN_15
#define ADC_SCK_PORT 		GPIO_PORT::B
#define ADC_SCK_PIN 		GPIO_PIN::PIN_3
#define ADC_MISO_PORT 		GPIO_PORT::B
#define ADC_MISO_PIN 		GPIO_PIN::PIN_4
#define ADC_MOSI_PORT 		GPIO_PORT::B
#define ADC_MOSI_PIN 		GPIO_PIN::PIN_5
#define DAC_SPI_NUM 		SPI_NUM::SPI_2
#define DAC_CS_PORT 		GPIO_PORT::B
#define DAC_CS_PIN 		GPIO_PIN::PIN_12
#define DAC_SCK_PORT 		GPIO_PORT::B
#define DAC_SCK_PIN 		GPIO_PIN::PIN_13
#define DAC_MISO_PORT 		GPIO_PORT::B
#define DAC_MISO_PIN  		GPIO_PIN::PIN_14
#define DAC_MOSI_PORT 		GPIO_PORT::B
#define DAC_MOSI_PIN 		GPIO_PIN::PIN_15
#define SLAVE_SPI_NUM 		SPI_NUM::SPI_1
#define SLAVE_CS_PORT 		GPIO_PORT::A
#define SLAVE_CS_PIN 		GPIO_PIN::PIN_4
#define SLAVE_SCK_PORT 		GPIO_PORT::A
#define SLAVE_SCK_PIN 		GPIO_PIN::PIN_5
#define SLAVE_MISO_PORT 	GPIO_PORT::A
#define SLAVE_MISO_PIN 		GPIO_PIN::PIN_6
#define SLAVE_MOSI_PORT 	GPIO_PORT::A
#define SLAVE_MOSI_PIN 		GPIO_PIN::PIN_7

// these pins are unconnected on SPI_DAC_ADC_Expansion board, so we disable them as per the ST recommendations
void disableUnusedPins()
{
	LLPD::gpio_output_setup( GPIO_PORT::C, GPIO_PIN::PIN_13, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::C, GPIO_PIN::PIN_14, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::C, GPIO_PIN::PIN_15, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );

	LLPD::gpio_output_setup( GPIO_PORT::A, GPIO_PIN::PIN_0, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::A, GPIO_PIN::PIN_1, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::A, GPIO_PIN::PIN_2, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::A, GPIO_PIN::PIN_3, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::A, GPIO_PIN::PIN_8, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::A, GPIO_PIN::PIN_9, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::A, GPIO_PIN::PIN_10, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::A, GPIO_PIN::PIN_11, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::A, GPIO_PIN::PIN_12, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );

	LLPD::gpio_output_setup( GPIO_PORT::B, GPIO_PIN::PIN_0, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::B, GPIO_PIN::PIN_1, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::B, GPIO_PIN::PIN_2, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::B, GPIO_PIN::PIN_6, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::B, GPIO_PIN::PIN_7, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::B, GPIO_PIN::PIN_8, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::B, GPIO_PIN::PIN_9, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
}

int main(void)
{
	// set system clock to PLL with HSE (16MHz / 2) as input, so 72MHz system clock speed
	LLPD::rcc_clock_setup( RCC_CLOCK_SOURCE::EXTERNAL, true, RCC_PLL_MULTIPLY::BY_9, SYS_CLOCK_FREQUENCY );

	// prescale APB1 by 2, since the maximum clock speed is 36MHz
	LLPD::rcc_set_periph_clock_prescalers( RCC_AHB_PRES::BY_1, RCC_APB1_PRES::AHB_BY_2, RCC_APB2_PRES::AHB_BY_1 );

	// enable all gpio clocks
	LLPD::gpio_enable_clock( GPIO_PORT::A );
	LLPD::gpio_enable_clock( GPIO_PORT::B );
	LLPD::gpio_enable_clock( GPIO_PORT::C );
	LLPD::gpio_enable_clock( GPIO_PORT::F );

	// disable the unused pins
	disableUnusedPins();

	// setup cs pins
	LLPD::gpio_output_setup( ADC_CS_PORT, ADC_CS_PIN, GPIO_PUPD::NONE, GPIO_OUTPUT_TYPE::PUSH_PULL, GPIO_OUTPUT_SPEED::HIGH );
	LLPD::gpio_output_setup( DAC_CS_PORT, DAC_CS_PIN, GPIO_PUPD::NONE, GPIO_OUTPUT_TYPE::PUSH_PULL, GPIO_OUTPUT_SPEED::HIGH );

	// set cs pins high
	LLPD::gpio_output_set( ADC_CS_PORT, ADC_CS_PIN, true );
	LLPD::gpio_output_set( DAC_CS_PORT, DAC_CS_PIN, true );

	// spi init (36MHz SPI2/SPI3 apb2 source, while 72MHz SPI1 apb1 source)
	LLPD::spi_master_init( DAC_SPI_NUM, SPI_BAUD_RATE::APB1CLK_DIV_BY_2, SPI_CLK_POL::LOW_IDLE, SPI_CLK_PHASE::SECOND,
				SPI_DUPLEX::FULL, SPI_FRAME_FORMAT::MSB_FIRST, SPI_DATA_SIZE::BITS_8 );
	LLPD::spi_master_init( ADC_SPI_NUM, SPI_BAUD_RATE::APB1CLK_DIV_BY_2, SPI_CLK_POL::LOW_IDLE, SPI_CLK_PHASE::FIRST,
				SPI_DUPLEX::FULL, SPI_FRAME_FORMAT::MSB_FIRST, SPI_DATA_SIZE::BITS_8 );
	LLPD::spi1_dma_slave_init( SLAVE_SPI_NUM, SPI_CLK_POL::LOW_IDLE, SPI_CLK_PHASE::FIRST, SPI_DUPLEX::FULL,
					SPI_FRAME_FORMAT::MSB_FIRST, SPI_DATA_SIZE::BITS_16, true );

	// adc mosi pin needs to be held high
	LLPD::gpio_output_setup( ADC_MOSI_PORT, ADC_MOSI_PIN, GPIO_PUPD::PULL_UP, GPIO_OUTPUT_TYPE::PUSH_PULL, GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_set( ADC_MOSI_PORT, ADC_MOSI_PIN, true );

	// audio timer setup (for 40 kHz sampling rate at 72 MHz system clock)
	LLPD::tim6_counter_setup( 0, 1800, 40000 );
	LLPD::tim6_counter_enable_interrupts();

	// audio timer start
	LLPD::tim6_counter_start();

	// first conversion to be safe
	LLPD::gpio_output_set( ADC_CS_PORT, ADC_CS_PIN, true );
	LLPD::tim6_delay( 1000000 );

	// begin spi1 dma
	LLPD::spi1_dma_slave_start( (void*) &adcInVal, (void*) &dacOutVal, bufferSize );

	// ready to start sending audio
	audioReady = true;

	while ( true )
	{
	}
}

extern "C" void TIM6_DAC_IRQHandler (void)
{
	if ( (! LLPD::tim6_isr_handle_delay()) && audioReady ) // if not currently in a delay function,...
	{
		LLPD::gpio_output_set( ADC_CS_PORT, ADC_CS_PIN, false );
		const uint8_t adcHighByte = LLPD::spi_master_send_and_recieve( ADC_SPI_NUM, 0b11111111 );
		const uint8_t adcLowByte = LLPD::spi_master_send_and_recieve( ADC_SPI_NUM, 0b11111111 );
		adcInVal[adcDacIncr] = ( adcHighByte << 8 ) | adcLowByte;
		LLPD::gpio_output_set( ADC_CS_PORT, ADC_CS_PIN, true );
		LLPD::gpio_output_set( DAC_CS_PORT, DAC_CS_PIN, false );
		const uint16_t dacVal = dacOutVal[adcDacIncr];
		adcDacIncr = ( adcDacIncr + 1 ) % bufferSize;
		LLPD::spi_master_send_and_recieve( DAC_SPI_NUM, 0b0 );
		LLPD::spi_master_send_and_recieve( DAC_SPI_NUM, (dacVal >> 8) );
		LLPD::spi_master_send_and_recieve( DAC_SPI_NUM, (dacVal & 0xFF) );
		LLPD::gpio_output_set( DAC_CS_PORT, DAC_CS_PIN, true );
	}

	LLPD::tim6_counter_clear_interrupt_flag();
}
