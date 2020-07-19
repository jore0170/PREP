/***************************************************************************//**
 * @file main.c
 * @brief Simple RAIL application which includes hal
 * @copyright Copyright 2017 Silicon Laboratories, Inc. http://www.silabs.com
 ******************************************************************************/
#include "rail.h"
#include "hal_common.h"
#include "rail_config.h"
#include "rail_types.h"

#include "em_chip.h"

#if defined(HAL_CONFIG)
#include "hal-config.h"
#endif

#include "bsp.h"
#include "retargetserial.h"
#include "gpiointerrupt.h"
#include <stdio.h>

#define DEBUG_LEVEL	1
#if DEBUG_LEVEL
#define initLog()     RETARGET_SerialInit()
#define flushLog()    RETARGET_SerialFlush()
#define printLog(...) printf(__VA_ARGS__)
#else
#define initLog()
#define flushLog()
#define printLog(...)
#endif

typedef struct ButtonArray {
	GPIO_Port_TypeDef port;
	unsigned int pin;
} ButtonArray_t;

static const ButtonArray_t buttonArray[BSP_BUTTON_COUNT] = BSP_BUTTON_INIT;

void gpioCallback(uint8_t pin);

void peripheralInit() {
	initLog();
	// RETARGET_SerialCrLf(1);
	for (int i = 0; i < BSP_BUTTON_COUNT; i++) {
		GPIO_PinModeSet(buttonArray[i].port, buttonArray[i].pin,
				gpioModeInputPull, 1);
	}
	GPIOINT_Init();
	GPIOINT_CallbackRegister(buttonArray[0].pin, gpioCallback);
	// GPIOINT_CallbackRegister(buttonArray[1].pin, gpioCallback);
	GPIO_IntConfig(buttonArray[0].port, buttonArray[0].pin, false, true, true);
	// GPIO_IntConfig(buttonArray[1].port, buttonArray[1].pin, false, true, true);

}

// Prototypes
void RAILCb_Generic(RAIL_Handle_t railHandle, RAIL_Events_t events);

RAIL_Handle_t railHandle;

static RAIL_Config_t railCfg = { .eventsCallback = &RAILCb_Generic, };

#define PAYLOAD_LENGTH 16
#define BUFFER_LENGTH 512
#define TX_BUFFER_SIZE BUFFER_LENGTH
#define RX_BUFFER_SIZE BUFFER_LENGTH
static uint8_t txBuffer[TX_BUFFER_SIZE];

void initRadio() {
	halInit();
	railHandle = RAIL_Init(&railCfg, NULL);
	if (railHandle == NULL) {
		while (1)
			;
	}
	RAIL_ConfigCal(railHandle, RAIL_CAL_ALL);

	// Set us to a valid channel for this config and force an update in the main
	// loop to restart whatever action was going on
	RAIL_ConfigChannels(railHandle, channelConfigs[0], NULL);

	// Initialize the PA now that the HFXO is up and the timing is correct
	RAIL_TxPowerConfig_t txPowerConfig = {
#if HAL_PA_2P4_LOWPOWER
			.mode = RAIL_TX_POWER_MODE_2P4_LP,
#else
			.mode = RAIL_TX_POWER_MODE_2P4_HP,
#endif
			.voltage = BSP_PA_VOLTAGE, .rampTime = HAL_PA_RAMP, };
#if RAIL_FEAT_SUBGIG_RADIO
	if (channelConfigs[0]->configs[0].baseFrequency < 1000000UL) {
		// Use the Sub-GHz PA if required
		txPowerConfig.mode = RAIL_TX_POWER_MODE_SUBGIG;
	}
#endif
	if (RAIL_ConfigTxPower(railHandle, &txPowerConfig) != RAIL_STATUS_NO_ERROR) {
		// Error: The PA could not be initialized due to an improper configuration.
		// Please ensure your configuration is valid for the selected part.
		while (1)
			;
	}
	RAIL_SetTxPower(railHandle, HAL_PA_POWER);
    /*
	RAIL_StateTransitions_t transitions = { RAIL_RF_STATE_RX, RAIL_RF_STATE_RX };
	RAIL_SetRxTransitions(railHandle, &transitions);
	RAIL_SetTxTransitions(railHandle, &transitions);

	RAIL_ConfigEvents(railHandle, RAIL_EVENTS_ALL,
			RAIL_EVENTS_TX_COMPLETION | RAIL_EVENTS_RX_COMPLETION
					| RAIL_EVENT_CAL_NEEDED);
    */
	RAIL_SetTxFifo(railHandle, txBuffer, 0, TX_BUFFER_SIZE);
	static const RAIL_DataConfig_t railDataConfig = {
			.txSource = TX_PACKET_DATA,  // not sure what to put here, is it compatible with start_txstream?
			.rxSource = RX_IQDATA_FILTLSB, //if rx_packet_data, works fine, alt is RX_IQDATA_FILTLSB
			.txMethod = FIFO_MODE, //doesn't do anything
			.rxMethod = FIFO_MODE, };
	RAIL_Status_t status = RAIL_ConfigData(railHandle, &railDataConfig);
	printLog("RAIL_ConfigData status: %d\n", status);
	RAIL_SetTxFifoThreshold(railHandle, TX_BUFFER_SIZE - TX_BUFFER_SIZE / 10);
	RAIL_SetRxFifoThreshold(railHandle, RX_BUFFER_SIZE - RX_BUFFER_SIZE / 10);
	RAIL_ConfigEvents(railHandle,
			RAIL_EVENTS_ALL, // RAIL_EVENT_TX_FIFO_ALMOST_EMPTY | RAIL_EVENT_RX_FIFO_ALMOST_FULL,
			RAIL_EVENT_TX_FIFO_ALMOST_EMPTY | RAIL_EVENT_RX_FIFO_ALMOST_FULL);
}

static uint8_t rxBuffer[BUFFER_LENGTH];
volatile bool received_data = false;
volatile int rcvd_length = 0;
volatile int available = 0;
volatile bool startTx = false;
volatile bool buttonPressed = false;

void gpioCallback(uint8_t pin) {
	startTx = !startTx;
	buttonPressed = true;
}

int main(void) {
	CHIP_Init();
	initRadio();
	peripheralInit();
	RAIL_StartRx(railHandle, 0, NULL);
	printLog("RAIL example\n");
	flushLog();
	// RETARGET_SerialFlush();
	while (1) {
		if (buttonPressed) {
			if (startTx) {
				int result = RAIL_StartTxStream(railHandle, 0,
						RAIL_STREAM_CARRIER_WAVE);
				printLog("start tx result: %d\r\n", result);
			} else {
				int result = RAIL_StopTxStream(railHandle);
				printLog("stop tx result: %d\r\n", result);
			}
			buttonPressed = false;

		}
		if (received_data) {
			// printf("RX %d available: %d ", rcvd_length, available);
			/*
			for (int i = 0; i < rcvd_length; i++) {
				printf(" 0x%02X", rxBuffer[i]);
			}
			*/
			for (int i=0; i<4; i++) {
				RETARGET_WriteChar(0xFF);
			}
			RETARGET_WriteChar(rcvd_length & 0xFF);
			RETARGET_WriteChar((rcvd_length>>8) & 0xFF);


			for (int i=0; i<rcvd_length; i++) {
				RETARGET_WriteChar(rxBuffer[i]);
			}
			// printf("\r\n");
			RETARGET_SerialFlush();
			received_data = false;
		}
	}
	return 0;
}

void RAILCb_Generic(RAIL_Handle_t railHandle, RAIL_Events_t events) {
	if (events & RAIL_EVENT_CAL_NEEDED) {
		RAIL_Calibrate(railHandle, NULL, RAIL_CAL_ALL_PENDING);
	}
	if (events & RAIL_EVENTS_TX_COMPLETION) {
		BSP_LedToggle(0);
	}
	if (events & RAIL_EVENTS_RX_COMPLETION) {
		BSP_LedToggle(1);
		printLog("Should not have gotten here, RAIL_EVENTS_RX_COMPLETION... how?\r\n");
	}
	if (events & RAIL_EVENT_RX_FIFO_ALMOST_FULL) {
		BSP_LedToggle(0);
		available = RAIL_GetRxFifoBytesAvailable(railHandle);
		available = available &0xFFC;
		rcvd_length = RAIL_ReadRxFifo(railHandle, rxBuffer, available);
		available = RAIL_GetRxFifoBytesAvailable(railHandle);
		// RAIL_ResetFifo(railHandle, false, true);
		received_data = true;
	}
}
