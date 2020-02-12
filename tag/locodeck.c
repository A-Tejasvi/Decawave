/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2016 Bitcraze AB
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * locodeck.c: Dwm1000 deck driver.
 */

#define DEBUG_MODULE "DWM"

#include <stdint.h>
#include <string.h>

/*******Added by Kedar*******/
#include "Arduino.h"
#include <SPI.h>
/****************************/

#include "locodeck.h"
#include "lpsTdoa3Tag.h"

// LOCO deck alternative IRQ and RESET pins(IO_2, IO_3) instead of default (RX1, TX1), leaving UART1 free for use
#define GPIO_PIN_IRQ 	  9
#define GPIO_PIN_RESET 	2
#define CS_PIN          SS


#define DEFAULT_RX_TIMEOUT 10000
#define ANTENNA_OFFSET 154.6   // In meter

// The anchor position can be set using parameters
// As an option you can set a static position in this file and set
// combinedAnchorPositionOk to enable sending the anchor rangings to the Kalman filter

static lpsAlgoOptions_t algoOptions = {
  // .userRequestedMode is the wanted algorithm, available as a parameter
#if LPS_TDOA_ENABLE
  .userRequestedMode = lpsMode_TDoA2,
#elif LPS_TDOA3_ENABLE
  .userRequestedMode = lpsMode_TDoA3,
#elif defined(LPS_TWR_ENABLE)
  .userRequestedMode = lpsMode_TWR,
#else
  .userRequestedMode = lpsMode_auto,
#endif
  // .currentRangingMode is the currently running algorithm, available as a log
  // lpsMode_auto is an impossible mode which forces initialization of the requested mode
  // at startup
  .currentRangingMode = lpsMode_auto,
  .modeAutoSearchActive = true,
  .modeAutoSearchDoInitialize = true,
};

struct {
  uwbAlgorithm_t *algorithm;
  char *name;
} algorithmsList[LPS_NUMBER_OF_ALGORITHMS + 1] = {
  // [lpsMode_TWR] = {.algorithm = &uwbTwrTagAlgorithm, .name="TWR"},
  // [lpsMode_TDoA2] = {.algorithm = &uwbTdoa2TagAlgorithm, .name="TDoA2"},
  [lpsMode_TDoA3] = {.algorithm = &uwbTdoa3TagAlgorithm, .name="TDoA3"},
};

// #if LPS_TDOA_ENABLE
// static uwbAlgorithm_t *algorithm = &uwbTdoa2TagAlgorithm;
// #elif LPS_TDOA3_ENABLE
static uwbAlgorithm_t *algorithm = &uwbTdoa3TagAlgorithm;
// #else
// static uwbAlgorithm_t *algorithm = &uwbTwrTagAlgorithm;
// #endif

static bool isInit = false;

static dwDevice_t dwm_device;
static dwDevice_t *dwm = &dwm_device;


static uint32_t timeout;

static void txCallback(dwDevice_t *dev)
{
  timeout = algorithm->onEvent(dev, eventPacketSent);
}

static void rxCallback(dwDevice_t *dev)
{
  timeout = algorithm->onEvent(dev, eventPacketReceived);
}

static void rxTimeoutCallback(dwDevice_t * dev) {
  timeout = algorithm->onEvent(dev, eventReceiveTimeout);
}

// This function is called from the memory sub system that runs in a different
// task, protect it from concurrent calls from this task
// TODO krri Break the dependency, do not call directly from other modules into the deck driver
bool locoDeckGetAnchorPosition(const uint8_t anchorId, point_t* position)
{
  if (!isInit) {
    return false;
  }
  bool result = algorithm->getAnchorPosition(anchorId, position);
  return result;
}

// This function is called from the memory sub system that runs in a different
// task, protect it from concurrent calls from this task
// TODO krri Break the dependency, do not call directly from other modules into the deck driver
uint8_t locoDeckGetAnchorIdList(uint8_t unorderedAnchorList[], const int maxListSize) {
  if (!isInit) {
    return 0;
  }
  uint8_t result = algorithm->getAnchorIdList(unorderedAnchorList, maxListSize);
  return result;
}

// This function is called from the memory sub system that runs in a different
// task, protect it from concurrent calls from this task
// TODO krri Break the dependency, do not call directly from other modules into the deck driver
uint8_t locoDeckGetActiveAnchorIdList(uint8_t unorderedAnchorList[], const int maxListSize) {
  if (!isInit) {
    return 0;
  }
  uint8_t result = algorithm->getActiveAnchorIdList(unorderedAnchorList, maxListSize);
  return result;
}

static bool switchToMode(const lpsMode_t newMode) {
  bool result = false;

  if (lpsMode_auto != newMode && newMode <= LPS_NUMBER_OF_ALGORITHMS) {
    algoOptions.currentRangingMode = newMode;
    algorithm = algorithmsList[algoOptions.currentRangingMode].algorithm;

    algorithm->init(dwm);
    timeout = algorithm->onEvent(dwm, eventTimeout);

    result = true;
  }

  return result;
}

static void autoModeSearchTryMode(const lpsMode_t newMode, const uint32_t now) {
  // // Set up next time to check
  // algoOptions.nextSwitchTick = now + LPS_AUTO_MODE_SWITCH_PERIOD;
  // switchToMode(newMode);
}


/**************************************/
/*******Remove xTaskGetTickCount*******/
/**************************************/

static lpsMode_t autoModeSearchGetNextMode() {
  // lpsMode_t newMode = algoOptions.currentRangingMode + 1;
  // if (newMode > LPS_NUMBER_OF_ALGORITHMS) {
  //   newMode = lpsMode_TWR;
  // }

  return 1;
}

// static void processAutoModeSwitching() {
//   uint32_t now = xTaskGetTickCount();

//   if (algoOptions.modeAutoSearchActive) {
//     if (algoOptions.modeAutoSearchDoInitialize) {
//       autoModeSearchTryMode(lpsMode_TDoA2, now);
//       algoOptions.modeAutoSearchDoInitialize = false;
//     } else {
//       if (now > algoOptions.nextSwitchTick) {
//         if (algorithm->isRangingOk()) {
//           // We have found an algorithm, stop searching and lock to it.
//           algoOptions.modeAutoSearchActive = false;
//           DEBUG_PRINT("Automatic mode: detected %s\n", algorithmsList[algoOptions.currentRangingMode].name);
//         } else {
//           lpsMode_t newMode = autoModeSearchGetNextMode();
//           autoModeSearchTryMode(newMode, now);
//         }
//       }
//     }
//   }
// }

/**************************************/
/**************************************/
/**************************************/

static void resetAutoSearchMode() {
  algoOptions.modeAutoSearchActive = true;
  algoOptions.modeAutoSearchDoInitialize = true;
}

// static void handleModeSwitch() {
//   if (algoOptions.userRequestedMode == lpsMode_auto) {
//     processAutoModeSwitching();
//   } else {
//     resetAutoSearchMode();
//     if (algoOptions.userRequestedMode != algoOptions.currentRangingMode) {
//       if (switchToMode(algoOptions.userRequestedMode)) {
//         DEBUG_PRINT("Switching to mode %s\n", algorithmsList[algoOptions.currentRangingMode].name);
//       }
//     }
//   }
// }

/**************************************/
/**************************************/
/**************************************/

static void uwbTask(void* parameters) {
  // lppShortQueue = xQueueCreate(10, sizeof(lpsLppShortPacket_t));

  algoOptions.currentRangingMode = lpsMode_TDoA3;

  while(1) {
    // handleModeSwitch();
    dwHandleInterrupt(dwm);
  //   if (xSemaphoreTake(irqSemaphore, timeout / portTICK_PERIOD_MS)) {
  //     do{
  //       xSemaphoreTake(algoSemaphore, portMAX_DELAY);
  //       dwHandleInterrupt(dwm);
  //       xSemaphoreGive(algoSemaphore);
  //     } while(digitalRead(GPIO_PIN_IRQ) != 0);
  //   } else {
  //     xSemaphoreTake(algoSemaphore, portMAX_DELAY);
  //     timeout = algorithm->onEvent(dwm, eventTimeout);
  //     xSemaphoreGive(algoSemaphore);
  //   }
  }
}

/**************************************/
/**************************************/
/**************************************/

static lpsLppShortPacket_t lppShortPacket;

// bool lpsSendLppShort(uint8_t destId, void* data, size_t length)
// {
//   bool result = false;

//   if (isInit)
//   {
//     lppShortPacket.dest = destId;
//     lppShortPacket.length = length;
//     memcpy(lppShortPacket.data, data, length);
//     result = xQueueSend(lppShortQueue, &lppShortPacket,0) == pdPASS;
//   }

//   return !result;
// }

// bool lpsGetLppShort(lpsLppShortPacket_t* shortPacket)
// {
//   return xQueueReceive(lppShortQueue, shortPacket, 0) == pdPASS;
// }

static uint8_t spiTxBuffer[196];
static uint8_t spiRxBuffer[196];
static uint16_t spiSpeed = 20000000L;

/************ Low level ops for libdw **********/

/***************************************************************/
/*********Check spiWrite and spiRead for bugs in memcpy*********/
/***************************************************************/

static void spiWrite(dwDevice_t* dev, const void *header, size_t headerLength,
                                      const void* data, size_t dataLength)
{
  SPI.spiBeginTransaction(SPISettings(spiSpeed, MSBFIRST, SPI_MODE0));
  digitalWrite(CS_PIN, LOW);

  memcpy(spiTxBuffer, header, headerLength);
  memcpy(spiTxBuffer + headerLength, data, dataLength);

  for (int i = 0; i < headerLength + dataLength; i++){
    SPI.transfer(spiTxBuffer[i]);
  }

  delayMicroseconds(5);
  digitalWrite(CS_PIN, HIGH);
  SPI.spiEndTransaction();
}

static void spiRead(dwDevice_t* dev, const void *header, size_t headerLength,
                                     void* data, size_t dataLength)
{
  SPI.spiBeginTransaction(SPISettings(spiSpeed, MSBFIRST, SPI_MODE0));
  digitalWrite(CS_PIN, LOW);

  memcpy(spiTxBuffer, header, headerLength);
  memset(spiTxBuffer+headerLength, 0, dataLength);

  for (int i = 0; i< headerLength; i++){
    SPI.transfer(spiTxBuffer[i]);
  }

  for (int i = 0; i< dataLength; i++){
    spiRxBuffer[i] = SPI.transfer(0x00);
  }

  memcpy(data, spiRxBuffer, dataLength);

  delayMicroseconds(5);
  digitalWrite(CS_PIN, HIGH);
  SPI.spiEndTransaction();
}

static void spiSetSpeed(dwDevice_t* dev, dwSpiSpeed_t speed)
{
  if (speed == dwSpiSpeedLow)
  {
    spiSpeed = 2000000L;
  }
  else if (speed == dwSpiSpeedHigh)
  {
    spiSpeed = 20000000L;
  }
}

static void delayms(dwDevice_t* dev, unsigned int time)
{
  delay(time);
}

static dwOps_t dwOps = {
  .spiRead = spiRead,
  .spiWrite = spiWrite,
  .spiSetSpeed = spiSetSpeed,
  .delayms = delayms,
};

/*********** Deck driver initialization ***************/
//static void dwm1000Init() {
void dwm1000Init() {
  SPI.begin();
  // Init IRQ input, Reset output, CS pin output
  pinMode(GPIO_PIN_IRQ, INPUT);
  pinMode(GPIO_PIN_RESET, OUTPUT);
  pinMode(CS_PIN, OUTPUT);

  // Reset the DW1000 chip
  digitalWrite(GPIO_PIN_RESET, LOW);
  delay(10);
  digitalWrite(GPIO_PIN_RESET, HIGH);
  delay(10);

  // Initialize the driver
  dwInit(dwm, &dwOps);       // Init libdw

  int result = dwConfigure(dwm);
  if (result != 0) {
    isInit = false;
    return;
  }

  dwTime_t delay = {.full = 0};
  dwSetAntenaDelay(dwm, delay);

  dwAttachSentHandler(dwm, txCallback);
  dwAttachReceivedHandler(dwm, rxCallback);
  dwAttachReceiveTimeoutHandler(dwm, rxTimeoutCallback);

  dwNewConfiguration(dwm);
  dwSetDefaults(dwm);

  dwEnableMode(dwm, MODE_SHORTDATA_MID_ACCURACY);
  //dwEnableMode(dwm, MODE_SHORTDATA_FAST_ACCURACY);

  dwSetChannel(dwm, CHANNEL_2);
  dwUseSmartPower(dwm, true);
  dwSetPreambleCode(dwm, PREAMBLE_CODE_64MHZ_9);

  dwSetReceiveWaitTimeout(dwm, DEFAULT_RX_TIMEOUT);
  dwCommitConfiguration(dwm);
  
  attachInterrupt(digitalPinToInterrupt(GPIO_PIN_IRQ), uwbTask, RISING);
  
  isInit = true;
}

uint16_t locoDeckGetRangingState() {
  return algoOptions.rangingState;
}

void locoDeckSetRangingState(const uint16_t newState) {
  algoOptions.rangingState = newState;
}


// static bool dwm1000Test()
// {
//   if (!isInit) {
//     DEBUG_PRINT("Error while initializing DWM1000\n");
//   }

//   return isInit;
// }
