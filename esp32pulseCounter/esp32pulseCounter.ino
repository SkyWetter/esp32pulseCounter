/*

Pulse Counter ARDUINO firmware

adapted by Andy Vanier from rdheiliger commented on Apr 12, 2017's 
https://github.com/espressif/arduino-esp32/issues/176

Sept 28, 2018

*/


// preprocessor directives
extern "C" {
#include "soc/pcnt_struct.h"
}
#include "driver/pcnt.h"

#define PCNT_TEST_UNIT PCNT_UNIT_0							// pulse counter 0 of 7
#define PCNT_H_LIM_VAL 32767								// high limit value 
#define PCNT_L_LIM_VAL -1									// low limit value

// global variables
byte pulsePin = 5;											// pulse pin input
int16_t flow0Counter = 0;
int16_t Pulses = 0;
int16_t x;

// timer shit
hw_timer_t * timer = NULL;									// typedef struct hw_timer_s hw_timer_t;--> hw_timer_t * timerBegin(uint8_t timer, uint16_t divider, bool countUp);
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;		// typedef struct  --> * - Uninitialized (invalid)*portMUX_FREE_VAL - Mux is free, can be locked by either CPU CORE_ID_PRO / CORE_ID_APP - Mux is locked to the particular core 	 * Any value other than portMUX_FREE_VAL, CORE_ID_PRO, CORE_ID_APP indicates corruption 	 * If mux is unlocked, count should be zero.* If mux is locked, count is non - zero & represents the number of recursive locks on the mux.


// variables for data proccessing functions
volatile byte state = LOW;
volatile byte state2 = LOW;
volatile byte state_tmr = 0;								// used in onTimer() function's case statement
volatile byte value_ready = 0;								// used in onTimer() function's case statement


void setup() 
{

	pinMode(pulsePin, INPUT);								// pin to read pulse frequency
	Serial.begin(115200);									// open serial monitor at 115200 BAUD
	Serial.println("");										// print nothing

	initPcnt();												// init pulseCounter function

	// timer declared like this:  -->  hw_timer_t * timer = NULL;	
	timer = timerBegin(0, 80, true);						// hw_timer_t * timerBegin(uint8_t timer, uint16_t divider, bool countUp);  -->  Use 1st timer of 4 (counted from zero). Set 80 divider for prescaler 
	timerAttachInterrupt(timer, &onTimer, true);			// void timerAttachInterrupt(hw_timer_t *timer, void(*fn)(void), bool edge);  -->  Attach onTimer function to our timer.
	timerAlarmWrite(timer, 1000000, true);					// void timerAlarmWrite(hw_timer_t *timer, uint64_t interruptAt, bool autoreload);  -->  Set alarm to call onTimer function every second (value in microseconds).  -->  Repeat the alarm (third parameter)
	timerAlarmEnable(timer);								// void timerAlarmEnable(hw_timer_t *timer);

}


void loop() 
{

	readPcntCounter_0();

	if (value_ready == 1) 
	{
		Serial.printf("Pulses: %d\r\n", Pulses);
		Serial.printf("Frequency %f Hz\r\n", (float)Pulses);
	}

	delay(100);

}


void readPcntCounter_0()
{

	// * @brief Get pulse counter value * @param pcnt_unit  Pulse Counter unit number * @param count Pointer to accept counter value  -->  esp_err_t pcnt_get_counter_value(pcnt_unit_t pcnt_unit, int16_t* count);
	if (pcnt_get_counter_value(PCNT_TEST_UNIT, &flow0Counter) == ESP_OK)
	{
		Serial.println("pcnt unit 0...");
	}

	delay(10);

	Serial.print("flow0Counts = ");
	Serial.println(flow0Counter);

}


// (ISR) |) alarm timer - IRAM_ATTTR assures this interrupt function will run from RAM and not slower FLASH
void IRAM_ATTR onTimer()
{

	portENTER_CRITICAL_ISR(&timerMux);								 // declare by calling the portENTER_CRITICAL_ISR and portExit_CRITICAL_ISR macros. They also both receive as input the address of the global portMUX_TYPE variable.

	switch (state_tmr)
	{

	case 0:
		pcnt_counter_clear(PCNT_TEST_UNIT);
		pcnt_counter_resume(PCNT_TEST_UNIT);
		value_ready = 0;
		state_tmr = 1;
		break;

	case 1:
		pcnt_counter_pause(PCNT_TEST_UNIT);							 // * @brief Pause PCNT counter of PCNT unit
		pcnt_get_counter_value(PCNT_TEST_UNIT, &flow0Counter);		 // * @brief Get pulse counter value  -->  esp_err_t pcnt_get_counter_value(pcnt_unit_t pcnt_unit, int16_t* count);
		Pulses = flow0Counter;
		flow0Counter = 0;
		value_ready = 1;
		state_tmr = 2;
		break;

	case 2:
		// if you have been thru all states reset state_tmr
		state_tmr = 0;
		break;

	default:
		break;
	}
	portEXIT_CRITICAL_ISR(&timerMux);

}


// This is first run in setup()  -->  pcnt config convers the unit and channel of Pcnt  -->  GPIO of pulsePin and pulseInGate
void initPcnt()							
{

	Serial.println("init pulse counter... ");

	// setup pcnt_config_t
	pcnt_config_t pcnt_config = 
	{
		pulsePin,													// = GPIO5 Pulse input gpio_num, if you want to use gpio16, pulse_gpio_num = 16, a negative value will be ignored
		PCNT_PIN_NOT_USED,											//  = -1 Control signal input gpio_num, a negative value will be ignored
		PCNT_MODE_KEEP,												// = 0 PCNT low control mode
		PCNT_MODE_KEEP,												// PCNT high control mode
		PCNT_COUNT_INC,												// PCNT positive edge count mode
		PCNT_COUNT_DIS,												// PCNT negative edge count mode
		PCNT_H_LIM_VAL,												// Maximum counter value
		PCNT_L_LIM_VAL,												// Minimum counter value
		PCNT_TEST_UNIT,												// PCNT unit number
		PCNT_CHANNEL_0,												// the PCNT channel
	};

	//Setting up of particular channel is then done by calling a function pcnt_unit_config() with above pcnt_config_t as the input parameter.
	if (pcnt_unit_config(&pcnt_config) == ESP_OK)					//init unit
		Serial.println("Config Unit_0 = ESP_OK");

	// filter value
	pcnt_set_filter_value(PCNT_TEST_UNIT, 1);
	pcnt_filter_enable(PCNT_TEST_UNIT);

	// * @brief Disable PCNT interrupt for PCNT unit  * @param pcnt_unit PCNT unit number  * @return  *     - ESP_OK Success *     - ESP_ERR_INVALID_ARG Parameter error  -->  esp_err_t pcnt_intr_disable(pcnt_unit_t pcnt_unit);
	pcnt_intr_disable(PCNT_TEST_UNIT);		
	
	// * @brief Disable PCNT event of PCNT unit  * @param unit PCNT unit number  * @param evt_type Watch point event type. All enabled events share the same interrupt (one interrupt per pulse counter unit). * @return *     - ESP_OK Success *     - ESP_ERR_INVALID_ARG Parameter error  -->  esp_err_t pcnt_event_disable(pcnt_unit_t unit, pcnt_evt_type_t evt_type);
	pcnt_event_disable(PCNT_TEST_UNIT, PCNT_EVT_L_LIM);				
	pcnt_event_disable(PCNT_TEST_UNIT, PCNT_EVT_H_LIM);		
	pcnt_event_disable(PCNT_TEST_UNIT, PCNT_EVT_THRES_0);
	pcnt_event_disable(PCNT_TEST_UNIT, PCNT_EVT_THRES_1);
	pcnt_event_disable(PCNT_TEST_UNIT, PCNT_EVT_ZERO);
		
	pcnt_counter_pause(PCNT_TEST_UNIT);								// Pause counter
	pcnt_counter_clear(PCNT_TEST_UNIT);								// Reset counter value

	//  * @brief Enable PCNT interrupt for PCNT unit * @note Each Pulse counter unit has five watch point events that share the same interrupt. *        Configure events with pcnt_event_enable() and pcnt_event_disable() * @param pcnt_unit PCNT unit number
	pcnt_intr_enable(PCNT_TEST_UNIT);

	pcnt_counter_resume(PCNT_TEST_UNIT);							// Resume counting

}


