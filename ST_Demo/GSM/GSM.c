/*
This library handles all interaction with the GSM Module.

Version 0.1 / March 2017

*** How to Use ****

--- Functions ---
- Required Function Calls -
gsmInit() - call at startup.
gsm_MS_Init() - call at startup (required for some functionality / modules).
gsm_GPRS_Init() - call at startup (TBD, requires GSM_GPRS module - not done yet).
gsm1msPing() - call at 1ms intervals (from an interrupt). This is used to
  update the timers used in this module.
gsmPoll() - call as often as possible.
- Power -
void gsmPowerSetOnOff(char power_on) - instructs the library to
  turn the module on or off (on by default)
- Date/Time (RTCC) -
void gsmDateTimeRead() - instructs the library to read the GSM
  modules RTCC (an event is fired once the RTCC has been read)
void gsmDateTimeWrite() - instructs the library to write to the GSM
  modules RTCC (an event is fired, in order to get the current date/time,
  just before the write occurs)
- Network Registration -
char gsmReady() - indicates if the module is registered on the network
- GPRS (Requires GSM_GPRS Module) - Once done, the functions will be
void gsmGprsHttpGet(char* url) - initiate a HTTP GET operation
void gsmGprsHttpPost(char* url, char* postdata) - initiate a HTTP POST operation
char gsmGprsPending() - indicates if a GPRS operation is pending
gsmGprsCancel() - cancels a pending GPRS operation
- External Functions -
extern void gsmEvent(char GsmEventType)
  Event data is available through the following event-specific variables:
    char* pstrGsmEventOriginatorID
    char* pstrGsmEventData
    TDateTime dtmGsmEvent
  GsmEventType can be:
    gsmevntDateTimeRead
      Date/Time read from the GSM module
        (initiated by gsmDateTimeRead())
      dtmGsmEvent contains the data
    gsmevntDateTimeWrite
      Date/Time about to be written to the GSM module
        (initiated by gsmDateTimeWrite())
      dtmGsmEvent should be loaded with the date/time to be written
    gsmevntIMEI_Read
      IMEI read from the GSM module
      pstrGsmEventData points to the read IMEI
    gsmevntPIN_Request
      GSM module is requesting the SIM PIN code
        (ignore event if no pin needed)
      pstrGsmEventData should be loaded with the SIM PIN code
    gsmevntPIN_Fail
      Provided PIN code was incorrect
    gsmevntMissedCall
      A missed call has been received
      pstrGsmEventOriginatorID points to the caller ID   
    gsmevntGprsFailed (requires GSM_GPRS module)
      Failed to complete GPRS operation
    gsmevntGprsHttpResultErr (requires GSM_GPRS module)
      HTTP operation returned a result code other than 200
        (e.g. 404 - page not found)
      pstrGsmEventData points to the result code
    gsmevntGprsHttpResponseLine (requires GSM_GPRS module to be completed)
      HTTP operation result (fired for each line received)
      pstrGsmEventData points to the result data
*** Debugging Features ***
#ifdef gsm_echo_int_rx
extern gsmUartRxEcho(char *UartRxLine) - echoes UART line received
#endif
#ifdef gsm_debug_state
extern char *gsmDebugStateStrPtr - string to hold the debug output (64 chars)
extern void gsmDebugStateStrReady() - called when the string is ready
#endif

*** Notes ***
bitGSM_Stat_On_State can be used to determine which state on the GSM_Stat pin
  is considered to be "on"

*** Operation ***
--- UART RX ---
Characters are received and added to a string buffer. When "new line" characters
are received a flag is set to indicate that a line of communication has been
received and is ready to be inspected / processed.
A timer exists which will clear (reset) the string if a "new line" is not
received within 100ms.

--- State Machine ---
Most of the software works on a state machine.
States are changed using the gsmSetStateNext routine, or a number of other
specialised setState... routines.
A "delay" state is available, which delays for a certain amount of time
before moving to the next state.
Provision is also made for a timeout condition, which will default to
a desired state if it is not cancelled within a certain amount of time.

*** Version History ***
--- v0.1    (2017/03/22) ---
Original release
*/

#define gsm_async_uart_rx // Enable asynchronous RX from the GSM module
                          // (using gsm1msPing())
                          // This can result in occasional delays of up to 1ms
                          // when gsmPoll() is called,
                          // but is preferrable if it is not possible to call
                          // gsmPoll() frequently

#include "GSM.h"

//<String_Functions>
#include "Str.h"
//</String_Functions>

//<Debugging>
#ifdef gsm_echo_int_rx
extern void gsmUartRxEcho(char *UartRxLine);
#endif
#ifdef gsm_debug_state
extern char *gsmDebugStateStrPtr;
extern void gsmDebugStateStrReady();
#endif
//</Debugging>

// ---------- UART Tx (Optionally Non-Blocking) ----------
volatile GPIO_TypeDef *GSM_Pwr_Key = (GPIO_TypeDef *)(GSM_Pwr_Key_Port);
#ifdef gsm_reset_en
volatile GPIO_TypeDef *GSM_Reset = (GPIO_TypeDef *)(GSM_Reset_Port);
#endif
volatile GPIO_TypeDef *GSM_Stat = (GPIO_TypeDef *)(GSM_Stat_Port);
//Define the UART used in the project 
UART_HandleTypeDef UartGSMHandle;

// UART Communication
//#define gsm_blocking_uart_tx //Remove for Non-blocking UART Tx
#ifndef gsm_blocking_uart_tx
#define cGsmUartTxQueMaxSize  8
unsigned int wrdGsmUartTxStrCharQueSw; // Bitfield indicating whether the next
                                       // qued item is a string or a char
                                       // (1 indicates a string, 0 a char)
char bytGsmUartTxStrCharQueSwPos = 0;
char *pcharGsmUartTxStrQue[cGsmUartTxQueMaxSize];
char bytGsmUartTxStrQueSize = 0;
char bytGsmUartTxStrQuePos = 0;
char charGsmUartTxCharQue[cGsmUartTxQueMaxSize];
char bytGsmUartTxCharQueSize = 0;
char bytGsmUartTxCharQuePos = 0;
#endif

char UART_Tx_Idle(void){
  if(UART_CheckIdleState(&UartGSMHandle) == HAL_OK)
  {
    return 1;
  }
  return 0;    
}

void UART_Write(char *pData){
  HAL_UART_Transmit(&UartGSMHandle,(uint8_t *) pData, sizeof(pData), TimeOut_TX);
}

char UART_Read(void){
  char pData[2];
  HAL_UART_Receive(&UartGSMHandle,(uint8_t *) pData,1,TimeOut_RX);
  return pData[0];
}

bit UART_Data_Ready(void){
  if(__HAL_UART_GET_FLAG(&UartGSMHandle, UART_FLAG_RXNE) == SET) {
    return 1;
  }
  return 0;
}

void UART_GSM_Init(void){
  
  UartGSMHandle.Instance        = USART_GSM;

  UartGSMHandle.Init.BaudRate   = USART_GSM_BAUDRATE;
  UartGSMHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartGSMHandle.Init.StopBits   = UART_STOPBITS_1;
  UartGSMHandle.Init.Parity     = UART_PARITY_NONE;
  UartGSMHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  UartGSMHandle.Init.Mode       = UART_MODE_TX_RX;
  HAL_UART_Init(&UartGSMHandle);
}
void gsmUART_Write_Text(char *UART_text) {
  #ifndef gsm_blocking_uart_tx
  // Add a string to the UART Tx que
  if (*UART_text != 0) {
  //if (bytGsmUartTxStrQueSize < cGsmUartTxQueMaxSize) {
    pcharGsmUartTxStrQue[bytGsmUartTxStrQueSize] = UART_text;
    bytGsmUartTxStrQueSize++;
    wrdGsmUartTxStrCharQueSw <<= 1;
    wrdGsmUartTxStrCharQueSw |= 1;
    //wrdGsmUartTxStrCharQueSw.B0 = 1;
    bytGsmUartTxStrCharQueSwPos++;
  //}
  }
  #else
    UART_Write_Text(UART_text);
  #endif
}

void gsmUART_Write(char data_) {
  #ifndef gsm_blocking_uart_tx
  // Add a single character to the UART Tx que
  //if (bytGsmUartTxCharQueSize < cGsmUartTxQueMaxSize) {
    charGsmUartTxCharQue[bytGsmUartTxCharQueSize] = data_;
    bytGsmUartTxCharQueSize++;
    wrdGsmUartTxStrCharQueSw <<= 1;
    //wrdGsmUartTxStrCharQueSw.B0 = 0; // Should not be necessary
    bytGsmUartTxStrCharQueSwPos++;
  //}
  #else
    UART_Write(data_)
  #endif
}

#ifndef gsm_blocking_uart_tx
static char gsmUartTx() {
  // Process the UART Tx Que
  // Returns 1 if there are still que items to be processed, 0 if not
  unsigned int sw;
  if (bytGsmUartTxStrQueSize || bytGsmUartTxCharQueSize) {
    if (UART_Tx_Idle()) {
      sw = wrdGsmUartTxStrCharQueSw >> (bytGsmUartTxStrCharQueSwPos - 1);
      if (sw & 1) {
        // String is next in que
        UART_Write((char *)pcharGsmUartTxStrQue[bytGsmUartTxStrQuePos]);
        pcharGsmUartTxStrQue[bytGsmUartTxStrQuePos]++; // Increment position within string
        if (!*pcharGsmUartTxStrQue[bytGsmUartTxStrQuePos]) { // End of string
          bytGsmUartTxStrQuePos++; // Increment que position
          bytGsmUartTxStrCharQueSwPos--;
          if (bytGsmUartTxStrQuePos == bytGsmUartTxStrQueSize) {
            // End of que
            bytGsmUartTxStrQueSize = 0;
            bytGsmUartTxStrQuePos = 0;
          }
        }
      } else {
        // Char is next in que
        UART_Write((char *)charGsmUartTxCharQue[bytGsmUartTxCharQuePos]);
        bytGsmUartTxCharQuePos++; // Increment que position
        bytGsmUartTxStrCharQueSwPos--;
        if (bytGsmUartTxCharQuePos == bytGsmUartTxCharQueSize) {
          // End of que
          bytGsmUartTxCharQueSize = 0;
          bytGsmUartTxCharQuePos = 0;
        }
      }
    }
    return 1;
  } else {
    return 0;
  }
}
#endif

// ---------- END UART Tx (Optionally Non-Blocking) ----------

// ---------- UART Rx ----------

char charGsmUartRx; // Received character
char strGsmUartRxBuff[256]; // String of data / communication which has been rcvd
char *pstrGsmUartRxBuff = (char *)strGsmUartRxBuff; // Current pos. within the above string
bit bitGsmUartRxReset; // Clear UART Rx buffer on timeout
#ifdef gsm_debug_state
bit bitGsmUartRxCharsLost;
bit bitGsmUartRxBuffCleared;
#endif
// Marker for avoiding buffer overrun
char *pstrGsmUartRxBuffCutoff = (char *)strGsmUartRxBuff+sizeof(strGsmUartRxBuff);
//char *pstrUartRxLine = &strGsmUartRxBuff; // Start of line currently being received
bit bitGsmUartRxLineReady; // Indicates that a "line" of communcation has been
                       // received, and is ready to be inspected / processed.
                       // Should be reset / cleared after the communication
                       // has been inspected / processed, in order to allow
                       // further communication to be received.
char bytGsmUartRxLinesReady = 0;
char bytGsmUartRxQuietTimer = 0;
#ifdef gsm_async_uart_rx
bit bitGsmUartRxSync;
#endif

static void gsmUartRxLineReceived() {
  // New line received
  *(pstrGsmUartRxBuff - 1) = 0;  // Mark end of line
  bitGsmUartRxLineReady = 1; // Notify main thread that line is ready
  bytGsmUartRxLinesReady++; // Increment number of lines ready
  //pstrUartRxLine = pstrGsmUartRxBuff; // Mark start of line currently being received
}

static void gsmUartRx() {
  // Read character from UART
  charGsmUartRx = UART_Read();
  //charGsmUartRx = RCREG1;
  bytGsmUartRxQuietTimer = 0;
  bitGsmUartRxReset = 0; // Just in case this had been set in gsm1msPing();
  if (pstrGsmUartRxBuff /*!=*/ < pstrGsmUartRxBuffCutoff) { // Check for buffer overrun
    // Buffer is not yet full
    // Check for new line
    if ((charGsmUartRx == 10) && // If Cr received
        (*(pstrGsmUartRxBuff - 1) == 13) /*&& // and previous character was Lf
        (pstrGsmUartRxBuff != &strGsmUartRxBuff)*/) { // and not at the start of the buffer
      // New line received
      gsmUartRxLineReceived();
    } else { // Otherwise add character to buffer
      *pstrGsmUartRxBuff = charGsmUartRx;
      pstrGsmUartRxBuff++;
      if (pstrGsmUartRxBuff == pstrGsmUartRxBuffCutoff) { // If buffer is now full
        gsmUartRxLineReceived(); // Process as if line was ready
        #ifdef gsm_debug_state
        bitGsmUartRxCharsLost = 1;
        #endif
      }
    }
  } else {
    // Buffer full, discard characters
    if (!bitGsmUartRxLineReady) { // If buffer is full then
      gsmUartRxLineReceived(); // process as if line was ready
    }
    #ifdef gsm_debug_state
    bitGsmUartRxCharsLost = 1;
    #endif
  }
}

#ifdef gsm_async_uart_rx
static void gsmUartRxSync() {
  bitGsmUartRxSync = 1;
  while (bitGsmUartRxSync);
}
#endif

void gsmUartRxLineClear() {
  /*
  //pstrGsmUartRxBuff = pstrUartRxLine; // Clear the line currently being received
  #ifdef gsm_async_uart_rx
  gsmUartRxSync();
  #endif
  while ((*(pstrGsmUartRxBuff - 1) != 0) && (pstrGsmUartRxBuff > &strGsmUartRxBuff)) {
    pstrGsmUartRxBuff--;
  }
  */
}

void gsmUartRxBuffClear() {
  #ifdef gsm_async_uart_rx
  gsmUartRxSync();
  #endif
  #ifdef gsm_debug_state
  if (pstrGsmUartRxBuff != &strGsmUartRxBuff) {
    #ifdef gsm_async_uart_rx
    bitGsmUartRxBuffCleared = 1;
    #else
    strcpy(gsmDebugStateStrPtr, "UART Rx Buff Cleared. Contents:\r\n");
    gsmDebugStateStrReady();
    while (bitGsmUartRxLineReady) {
      strncpyExNewLine(gsmDebugStateStrPtr, strGsmUartRxBuff, 64);
      gsmDebugStateStrReady();
      gsmUartRxLineProcessed();
    }
    if (pstrGsmUartRxBuff != strGsmUartRxBuff) {
      *pstrGsmUartRxBuff = 0;
      strncpyExNewLine(gsmDebugStateStrPtr, strGsmUartRxBuff, 64);
      gsmDebugStateStrReady();
    }
    strcpy(gsmDebugStateStrPtr, "(End Contents)\r\n");
    gsmDebugStateStrReady();
    #endif
  }
  #endif
  bytGsmUartRxLinesReady = 0;
  bitGsmUartRxLineReady = 0;
  pstrGsmUartRxBuff = &strGsmUartRxBuff[0]; // Reset to the start of the buffer
  //pstrUartRxLine = &strGsmUartRxBuff;
  //strcpy(gsmDebugStateStrPtr, "UART Rx Buff Cleared.\r\n");
  //gsmUartRxEcho(gsmDebugStateStrPtr);
}

void gsmUartRxLineProcessed() {
  char bLineLen = 0;
  char *psRdPos;
  char *psWrPos;
  #ifdef gsm_async_uart_rx
  gsmUartRxSync();
  #endif
  #ifdef gsm_echo_int_rx
  //strcpy(gsmDebugStateStrPtr, "UART Rx Line Processed:\r\n");
  //gsmUartRxEcho(gsmDebugStateStrPtr);
  gsmUartRxEcho(&strGsmUartRxBuff);
  #endif
  /*#ifdef gsm_debug_state
  strcpy(gsmDebugStateStrPtr, "Shifting UART Rx Buffer\r\n");
  gsmDebugStateStrReady();
  #endif*/
  psRdPos = (char *)strGsmUartRxBuff;
  psWrPos = (char *)strGsmUartRxBuff;
  // Find the end of the current line
  while ((*psRdPos != 0) && (psRdPos < pstrGsmUartRxBuff)) {
    psRdPos++;
  }
  psRdPos++; // Start of next line
  bLineLen = psRdPos - psWrPos;
  // Shift buffer
  while (psRdPos < pstrGsmUartRxBuff) {
    *psWrPos = *psRdPos;
    psRdPos++;
    psWrPos++;
  }
  pstrGsmUartRxBuff -= bLineLen;
  //pstrUartRxLine -= bLineLen;
  bytGsmUartRxLinesReady--;
  if (bytGsmUartRxLinesReady == 0) {bitGsmUartRxLineReady = 0;}
}

// ---------- END UART Rx ----------

//<Events>
extern void gsmEvent(char GsmEventType);
char* pstrGsmEventOriginatorID;
char* pstrGsmEventData;
TDateTime dtmGsmEvent;
//</Events>

// -- Constants --

#ifdef __GNUC__
// Events
const char gsmevntDateTimeRead = 150;
const char gsmevntDateTimeWrite = 151;
const char gsmevntSignalQualityRead = 152;
const char gsmevntIMEI_Read = 20;
const char gsmevntPIN_Request = 30;
const char gsmevntPIN_Fail = 31;
const char gsmevntMissedCall = 70;
const char gsmevntMsgRcvd = 80;
const char gsmevntMsgDrafted = 81; // Message written to SIM card
const char gsmevntMsgDiscarded = 82; // Gave up on trying to write msg to SIM card
                                    // (SIM card memory full?)
const char gsmevntMsgSent = 83;
const char gsmevntMsgSendFailed = 84; // Gave up on trying to send message
                                      // (no airtime?)
const char gsmevntGprsFailed = 110;
const char gsmevntGprsHttpResultErr = 111;
const char gsmevntGprsHttpResponseLine = 112;
// GSM State Machine States
// Note: In order not to overrun the output buffer
//       (when gsm_debug_state is defined)
//       state names should not be longer than 26 characters
//       i.e. last char here -----> x
// General-Purpose
const char gsmstDelay = 1;
const char gsmstWaitingOK = 2;
const char gsmstCmdOK = 3;
const char gsmstDie = 9;
// Diversions
const char gsmstGetDateTimePre = 150;
const char gsmstGetDateTimeQuery = 151;
const char gsmstGetDateTimeResponse = 152;
const char gsmstSetDateTimePre = 160;
const char gsmstSetDateTime = 161;
// Power
const char gsmstPwrGsmOffPre = 10;
const char gsmstPwrGsmOff = 11;
const char gsmstPwringGsmOff = 12;
const char gsmstPwrGsmOn = 13;
const char gsmstPwringGsmOn = 14;
// Info
const char gsmstIMEIPre = 20;
const char gsmstIMEIQuery = 21;
const char gsmstIMEIResponse = 22;
// Pin
const char gsmstPinChkPre = 30;
const char gsmstPinChkQuery = 31;
const char gsmstPinChkResponse = 32;
const char gsmstPinPre = 33;
const char gsmstPinCmd = 34;
const char gsmstPinResponse = 35;
// Module Setup
const char gsmstSetup_MSHI = 40;
const char gsmstSetup_MSHO = 41;
const char gsmstEnableCLIP = 42;
const char gsmstSetMsgFrmtToTxt = 43;
const char gsmstSetMsgAlertOnPre = 44;
const char gsmstSetMsgAlertOn = 45;
// Network Registration
const char gsmstWaitRegPre = 50;
const char gsmstWaitRegQuery = 51;
const char gsmstWaitRegResponse = 52;
// Standby
const char gsmstStandbyPre = 60;
const char gsmstStandby = 61;
// Call
const char gsmstWaitingCLIP = 70;
const char gsmstWaitingNO_CARRIER = 71;
// Text Message - States 80-109
const gsmstMsgHook = 109;
// GPRS - States 110-139
const char gsmstGPRS_Hook = 110;
// Module Specific - States 200+


#ifdef gsm_debug_state                
const char cstr_gsmstDelay[] = "gsmstDelay";
const char cstr_gsmstWaitingOK[] = "gsmstWaitingOK";
const char cstr_gsmstCmdOK[] = "gsmstCmdOK";
const char cstr_gsmstDie[] = "gsmstDie";
const char cstr_gsmstGetDateTimePre[] = "gsmstGetDateTimePre";
const char cstr_gsmstGetDateTimeQuery[] = "gsmstGetDateTimeQuery";
const char cstr_gsmstGetDateTimeResponse[] = "gsmstGetDateTimeResponse";
const char cstr_gsmstSetDateTimePre[] = "gsmstSetDateTimePre";
const char cstr_gsmstSetDateTime[] = "gsmstSetDateTime";
const char cstr_gsmstPwrGsmOffPre[] = "gsmstPwrGsmOffPre";
const char cstr_gsmstPwrGsmOff[] = "gsmstPwrGsmOff";
const char cstr_gsmstPwringGsmOff[] = "gsmstPwringGsmOff";
const char cstr_gsmstPwrGsmOn[] = "gsmstPwrGsmOn";
const char cstr_gsmstPwringGsmOn[] = "gsmstPwringGsmOn";
const char cstr_gsmstIMEIPre[] = "gsmstIMEIPre";
const char cstr_gsmstIMEIQuery[] = "gsmstIMEIQuery";
const char cstr_gsmstIMEIResponse[] = "gsmstIMEIResponse";
const char cstr_gsmstPinChkPre[] = "gsmstPinChkPre";
const char cstr_gsmstPinChkQuery[] = "gsmstPinChkQuery";
const char cstr_gsmstPinChkResponse[] = "gsmstPinChkResponse";
const char cstr_gsmstPinPre[] = "gsmstPinPre";
const char cstr_gsmstPinCmd[] = "gsmstPinCmd";
const char cstr_gsmstPinResponse[] = "gsmstPinResponse";
const char cstr_gsmstSetup_MSHI[] = "gsmstSetup_MSHI";
const char cstr_gsmstSetup_MSHO[] = "gsmstSetup_MSHO";
const char cstr_gsmstEnableCLIP[] = "gsmstEnableCLIP";
const char cstr_gsmstSetMsgFrmtToTxt[] = "gsmstSetMsgFrmtToTxt";
const char cstr_gsmstSetMsgAlertOn[] = "gsmstSetMsgAlertOn";
const char cstr_gsmstSetMsgAlertOnPre[] = "gsmstSetMsgAlertOnPre";
const char cstr_gsmstWaitRegPre[] = "gsmstWaitRegPre";
const char cstr_gsmstWaitRegQuery[] = "gsmstWaitRegQuery";
const char cstr_gsmstWaitRegResponse[] = "gsmstWaitRegResponse";
const char cstr_gsmstStandbyPre[] = "gsmstStandbyPre";
const char cstr_gsmstStandby[] = "gsmstStandby";
const char cstr_gsmstWaitingCLIP[] = "gsmstWaitingCLIP";
const char cstr_gsmstWaitingNO_CARRIER[] = "gsmstWaitingNO_CARRIER";
const char cstr_gsmstMsgHook[] = "gsmstMsgHook";
const char cstr_gsmstGPRS_Hook[] = "gsmstGPRS_Hook";
#endif

#else /*__GNUC__*/
#ifdef gsm_debug_state                
#define cstr_gsmstDelay[]                       "gsmstDelay"
#define cstr_gsmstWaitingOK[]                   "gsmstWaitingOK"
#define cstr_gsmstCmdOK[]                       "gsmstCmdOK"
#define cstr_gsmstDie[]                         "gsmstDie"
#define cstr_gsmstGetDateTimePre[]              "gsmstGetDateTimePre"
#define cstr_gsmstGetDateTimeQuery[]            "gsmstGetDateTimeQuery"
#define cstr_gsmstGetDateTimeResponse[]         "gsmstGetDateTimeResponse"
#define cstr_gsmstSetDateTimePre[]              "gsmstSetDateTimePre"
#define cstr_gsmstSetDateTime[]                 "gsmstSetDateTime"
#define cstr_gsmstPwrGsmOffPre[]                "gsmstPwrGsmOffPre"
#define cstr_gsmstPwrGsmOff[]                   "gsmstPwrGsmOff"
#define cstr_gsmstPwringGsmOff[]                "gsmstPwringGsmOff"
#define cstr_gsmstPwrGsmOn[]                    "gsmstPwrGsmOn"
#define cstr_gsmstPwringGsmOn[]                 "gsmstPwringGsmOn"
#define cstr_gsmstIMEIPre[]                     "gsmstIMEIPre"
#define cstr_gsmstIMEIQuery[]                   "gsmstIMEIQuery"
#define cstr_gsmstIMEIResponse[]                "gsmstIMEIResponse"
#define cstr_gsmstPinChkPre[]                   "gsmstPinChkPre"
#define cstr_gsmstPinChkQuery[]                 "gsmstPinChkQuery"
#define cstr_gsmstPinChkResponse[]              "gsmstPinChkResponse"
#define cstr_gsmstPinPre[]                      "gsmstPinPre"
#define cstr_gsmstPinCmd[]                      "gsmstPinCmd"
#define cstr_gsmstPinResponse[]                 "gsmstPinResponse"
#define cstr_gsmstSetup_MSHI[]                  "gsmstSetup_MSHI"
#define cstr_gsmstSetup_MSHO[]                  "gsmstSetup_MSHO"
#define cstr_gsmstEnableCLIP[]                  "gsmstEnableCLIP"
#define cstr_gsmstSetMsgFrmtToTxt[]             "gsmstSetMsgFrmtToTxt"
#define cstr_gsmstSetMsgAlertOn[]               "gsmstSetMsgAlertOn"
#define cstr_gsmstSetMsgAlertOnPre[]            "gsmstSetMsgAlertOnPre"
#define cstr_gsmstWaitRegPre[]                  "gsmstWaitRegPre"
#define cstr_gsmstWaitRegQuery[]                "gsmstWaitRegQuery"
#define cstr_gsmstWaitRegResponse[]             "gsmstWaitRegResponse"
#define cstr_gsmstStandbyPre[]                  "gsmstStandbyPre"
#define cstr_gsmstStandby[]                     "gsmstStandby"
#define cstr_gsmstWaitingCLIP[]                 "gsmstWaitingCLIP"
#define cstr_gsmstWaitingNO_CARRIER[]           "gsmstWaitingNO_CARRIER"
#define cstr_gsmstMsgHook[]                     "gsmstMsgHook"
#define cstr_gsmstGPRS_Hook[]                   "gsmstGPRS_Hook"
#endif
#endif /*__GNUC__*/ 

#ifdef gsm_debug_state
static void gsm_strcatState(char *to, char state) {
  // Appends the current state name to the end of a string
  char handled = 0;
  if (!handled && p_gsm_MS_strcatState) {
    handled = p_gsm_MS_strcatState(to, state);
  }
  if (!handled && p_gsm_Msg_strcatState) {
    handled = p_gsm_Msg_strcatState(to, state);
  }
  if (!handled && p_gsm_GPRS_strcatState) {
    handled = p_gsm_GPRS_strcatState(to, state);
  }
  if (!handled) {
    switch (state) {
      case gsmstDelay: strcat(to, RomTxt30(&cstr_gsmstDelay)); break;
      case gsmstWaitingOK: strcat(to, RomTxt30(&cstr_gsmstWaitingOK)); break;
      case gsmstCmdOK: strcat(to, RomTxt30(&cstr_gsmstCmdOK)); break;
      case gsmstDie: strcat(to, RomTxt30(&cstr_gsmstDie)); break;
      case gsmstGetDateTimePre: strcat(to, RomTxt30(&cstr_gsmstGetDateTimePre)); break;
      case gsmstGetDateTimeQuery: strcat(to, RomTxt30(&cstr_gsmstGetDateTimeQuery)); break;
      case gsmstGetDateTimeResponse: strcat(to, RomTxt30(&cstr_gsmstGetDateTimeResponse)); break;
      case gsmstSetDateTimePre: strcat(to, RomTxt30(&cstr_gsmstSetDateTimePre)); break;
      case gsmstSetDateTime: strcat(to, RomTxt30(&cstr_gsmstSetDateTime)); break;
      case gsmstPwrGsmOffPre: strcat(to, RomTxt30(&cstr_gsmstPwrGsmOffPre)); break;
      case gsmstPwrGsmOff: strcat(to, RomTxt30(&cstr_gsmstPwrGsmOff)); break;
      case gsmstPwringGsmOff: strcat(to, RomTxt30(&cstr_gsmstPwringGsmOff)); break;
      case gsmstPwrGsmOn: strcat(to, RomTxt30(&cstr_gsmstPwrGsmOn)); break;
      case gsmstPwringGsmOn: strcat(to, RomTxt30(&cstr_gsmstPwringGsmOn)); break;
      case gsmstIMEIPre: strcat(to, RomTxt30(&cstr_gsmstIMEIPre)); break;
      case gsmstIMEIQuery: strcat(to, RomTxt30(&cstr_gsmstIMEIQuery)); break;
      case gsmstIMEIResponse: strcat(to, RomTxt30(&cstr_gsmstIMEIResponse)); break;
      case gsmstPinChkPre: strcat(to, RomTxt30(&cstr_gsmstPinChkPre)); break;
      case gsmstPinChkQuery: strcat(to, RomTxt30(&cstr_gsmstPinChkQuery)); break;
      case gsmstPinChkResponse: strcat(to, RomTxt30(&cstr_gsmstPinChkResponse)); break;
      case gsmstPinPre: strcat(to, RomTxt30(&cstr_gsmstPinPre)); break;
      case gsmstPinCmd: strcat(to, RomTxt30(&cstr_gsmstPinCmd)); break;
      case gsmstPinResponse: strcat(to, RomTxt30(&cstr_gsmstPinResponse)); break;
      case gsmstSetup_MSHI: strcat(to, RomTxt30(&cstr_gsmstSetup_MSHI)); break;
      case gsmstSetup_MSHO: strcat(to, RomTxt30(&cstr_gsmstSetup_MSHO)); break;
      case gsmstEnableCLIP: strcat(to, RomTxt30(&cstr_gsmstEnableCLIP)); break;
      case gsmstSetMsgFrmtToTxt: strcat(to, RomTxt30(&cstr_gsmstSetMsgFrmtToTxt)); break;
      case gsmstSetMsgAlertOn: strcat(to, RomTxt30(&cstr_gsmstSetMsgAlertOn)); break;
      case gsmstSetMsgAlertOnPre: strcat(to, RomTxt30(&cstr_gsmstSetMsgAlertOnPre)); break;
      case gsmstWaitRegPre: strcat(to, RomTxt30(&cstr_gsmstWaitRegPre)); break;
      case gsmstWaitRegQuery: strcat(to, RomTxt30(&cstr_gsmstWaitRegQuery)); break;
      case gsmstWaitRegResponse: strcat(to, RomTxt30(&cstr_gsmstWaitRegResponse)); break;
      case gsmstStandbyPre: strcat(to, RomTxt30(&cstr_gsmstStandbyPre)); break;
      case gsmstStandby: strcat(to, RomTxt30(&cstr_gsmstStandby)); break;
      case gsmstWaitingCLIP: strcat(to, RomTxt30(&cstr_gsmstWaitingCLIP)); break;
      case gsmstWaitingNO_CARRIER: strcat(to, RomTxt30(&cstr_gsmstWaitingNO_CARRIER)); break;
      case gsmstMsgHook: strcat(to, RomTxt30(&cstr_gsmstMsgHook)); break;
      case gsmstGPRS_Hook: strcat(to, RomTxt30(&cstr_gsmstGPRS_Hook)); break;
      default:
        strcat(to, "[not found]"); break;
    }
  }
}
#endif

// -- Variables --
// General-Purpose
unsigned int wrdGsmGPTmr = 0; // General-purpose timer (integer)
unsigned long dwdGsmGPTmr = 0; // General-purpose timer (long)
char bytGsmGPCtr = 0; // General-purpose counter
bit bitGsmGPFlag; // General-purpose flag   
char strGsmGP[22]; // General-purpose string
char* pstrGsmGP;
// UART Communication Strings
char strNewLine[] = "\r\n"; //{13, 10, 0};
char strAT[] = "AT";
char strOK[] = "OK";
char strSetOn[] = "=1";
char strERROR[] = "ERROR";
char strREADY[] = "READY";
char strCPIN[] = "+CPIN";
char strCREG[] = "+CREG";
char strCCLK[] = "+CCLK";
char strRING[] = "RING";
char strCLIP[] = "+CLIP";
char strNOCARRIER[] = "NO CARRIER";
char strCMTI[] = "+CMTI";
char strCSQ[] = "+CSQ";
// State Machine
char bytGsmState;   
// State Machine Module Specific
#ifdef gsm_debug_state
char (*p_gsm_MS_strcatState)(char *to, char state) = 0;
#endif
char (*p_gsm_MS_ProcessState)(char dummy) = 0;
// State Machine Delay
char bytGsmStateAfterDelay;
unsigned int wrdGsmDelayTime;
unsigned int wrdGsmDelayTmr = 0;
// State Machine Timeout
char bytGsmStateAfterTimeout = 0;
unsigned int wrdGsmTimeoutTime;
unsigned int wrdGsmTimeoutTmr = 0;
// State Machine Divert
char bytGsmStateAfterDivert = 0;
// State Machine Clock
bit bitGsmDateTimeReadPending;
bit bitGsmDateTimeWritePending;
// State Machine SMS
#ifdef gsm_debug_state
char (*p_gsm_Msg_strcatState)(char *to, char state) = 0;
#endif
char (*p_gsm_Msg_ProcessState)(char dummy) = 0;
char *pstrGsmMsgSendTxt;
char *pstrGsmMsgSendNum;
bit bitGsmMsgDelPending;
bit bitGsmMsgWritePending;
bit bitGsmMsgReadPending;
bit bitGsmMsgSendPending;
//unsigned int wrdGsmMsgWriteTmr;
bit bitGsmMsgJustArrived;
// State Machine GPRS
#ifdef gsm_debug_state
char (*p_gsm_GPRS_strcatState)(char *to, char state) = 0;
#endif
char (*p_gsm_GPRS_ProcessState)(char dummy) = 0;
bit bitGsmGprsPending;
bit bitGsmGprsInProgress;
char *pstrGsmGprsURL;
char *pstrGsmGprsData;
unsigned int wrdGsmGprsDataSize;
bit bitGsmGprsHttpKeepAlive;
bit bitGsmGprsRestartFlag; // Informs the GPRS module that the system has restarted
// State Machine Other
bit bitExpectGSM_On;
bit bitGSM_PowerOff; // Instructs the library to power the GSM module off
char bytGsmStateAfterOK = 0;
char bytGsmCmdOKCtr = 0;
char bytGsmStateAfterReg = 0;
char *pstrGsmCommand;
char bytGsmStateAfterCmdFail = 0;
char strGsmOrigOrDestID[15];
char bytGSM_StatTmr = 0; //GSM_Stat can sometimes dip off very briefly
                         //This is used to avoid "false" off readings
// Misc
bit bitGSM_Stat_On_State; // Determines what state of GSM_Stat is considered "on"
bit bitGSM_Ready; // Indicates if the module is registered on the network

static char gsmCheckStateDivert() {
  if (bitGSM_PowerOff) {
    return gsmstPwrGsmOffPre;
  } if (bitGsmDateTimeReadPending) {
    //bitGsmDateTimeReadPending = 0; // Now only done once successful
    return gsmstGetDateTimePre;
  } else if (bitGsmDateTimeWritePending) {
    return gsmstSetDateTimePre;
  } else {
    return 0;
  }
}

void gsmSetStateNext(char stateNext, char allowDivert) {
  /*if (stateNext == bytGsmStateAfterDivert) {
    // Returning from divert
    bytGsmStateAfterDivert = 0;
  }*/
  if (/*(bytGsmStateAfterDivert == 0) &&*/ (allowDivert)) {
    bytGsmStateAfterDivert = stateNext;
    stateNext = gsmCheckStateDivert();
    if (stateNext == 0) {
      stateNext = bytGsmStateAfterDivert; // No diversions currently pending
      //bytGsmStateAfterDivert = 0;
    }
  }
  // Diversions should only be allowed when the state machine is completely "free"
  // (no timeouts, counters, etc being used by any other state)
  // Diversion should never be allowed from within another diversion
  if (stateNext != bytGsmState) {
    #ifdef gsm_debug_state
    // Output the current and next states
    // Caution: If the state names are too long then they can overflow the
    //          buffer, with unpredictable results
    strcpy(gsmDebugStateStrPtr, "cur:");
    gsm_strcatState(gsmDebugStateStrPtr, bytGsmState);
    strcat(gsmDebugStateStrPtr, " nxt:");
    gsm_strcatState(gsmDebugStateStrPtr, stateNext);
    strcat(gsmDebugStateStrPtr, &strNewLine);
    gsmDebugStateStrReady();
    #endif
    bytGsmState = stateNext;
  }
}

void gsmSetStateTimeout(unsigned int time_ms, char stateAfterTimeout) {
  // Activates a timeout timer, which will jump to a certain state
  // if it runs out before being cancelled
  wrdGsmTimeoutTime = time_ms;
  bytGsmStateAfterTimeout = stateAfterTimeout;
  wrdGsmTimeoutTmr = 0;
}

void gsmCancelStateTimeout() {
  bytGsmStateAfterTimeout = 0; // Cancel timeout (if applicable)
}

void gsmSetStateDelay(unsigned int time_ms, char stateAfterDelay) {
  // Delays for a certain amount of time, before moving to the next state
  gsmSetStateNext(gsmstDelay, 0);
  wrdGsmDelayTime = time_ms;
  bytGsmStateAfterDelay = stateAfterDelay;
  wrdGsmDelayTmr = 0;
  #ifdef gsm_debug_state
  strcpy(gsmDebugStateStrPtr, "Delaying ");
  IntToStr(wrdGsmDelayTime, gsmDebugStateStrPtr + 9);
  strcat(gsmDebugStateStrPtr, "ms\r\n");
  gsmDebugStateStrReady();
  #endif
}

void gsmSetStateWaitOK(char stateAfterOK, unsigned int timeout,
                    char stateAfterTimeout) {
  gsmSetStateNext(gsmstWaitingOK, 0);
  bytGsmStateAfterOK = stateAfterOK;
  gsmSetStateTimeout(timeout, stateAfterTimeout);
  gsmUartRxLineClear(); // Make sure that new comms will be received
}

void gsmSetStateWaitReg(char stateAfterReg) {
  gsmSetStateNext(gsmstWaitRegPre, 1);
  bytGsmStateAfterReg = stateAfterReg;
  // If bytGsmStateAfterReg is set to zero then a default next state
  // will be selected in gsmstWaitRegPre
  // bytGsmStateAfterReg will also be reset to default after each successful
  // network registration check
  // Default is currently gsmstReadMsgRequest
}

void gsmSetStateCmdOK(char* cmd, char stateAfterOK, char stateAfterFail) {
  dwdGsmGPTmr = 0;
  bytGsmGPCtr = 0;
  pstrGsmCommand = cmd;
  bytGsmStateAfterOK = stateAfterOK;
  if (stateAfterFail == 0) {
    stateAfterFail = gsmstPwrGsmOffPre;
  }
  bytGsmStateAfterCmdFail = stateAfterFail;
  gsmSetStateNext(gsmstCmdOK, 0);
}

static char gsmExtractCallerId(char *source, char *dest) {
  // Copies to dest the value between the next two quotes in source
  // Returns 1 if successful, 0 if not
  char ctr = 0;
  char *destStart;
  destStart = dest;
  // Look for the first quote (") character in the string
  while ((*source != '"') && (*source != 0) && (ctr < 30)) {
    source++;
    ctr++;
  }
  if (*source == '"') { // If a quote (") character was found then
    source++;
    ctr = 0;
    // Copy from source to dest until the next quote character is found
    source += strcpyTillChar(source, dest, '"', 15);
    if (*source == '"') { // If a closing quote was found then
      return 1;
    } else { // Otherwise, if a closing quote was not found then
      *destStart = 0; // Mark the string as blank
    }
  } else { // Otherwise, if a starting quote was not found then
    *destStart = 0; // Mark the string as blank
  }
  return 0;
}

void gsmExtractDateTime(char *source) {
  // yy/MM/dd,hh:mm:ss
  source = strchr(source, '/');
  if (source != 0) {
    source -= 2;
    dtmGsmEvent.Year = StrToByte(source);
    source += 3;
    dtmGsmEvent.Month = StrToByte(source);
    source += 3;
    dtmGsmEvent.Day = StrToByte(source);
    source += 3;
    dtmGsmEvent.Hour = StrToByte(source);
    source += 3;
    dtmGsmEvent.Minute = StrToByte(source);
    source += 3;
    dtmGsmEvent.Second = StrToByte(source);
    source += 3;
  } else {
    dtmGsmEvent.Day = 0; // Mark as bad / not read
  }
}

static void gsmRTCC_GenerateDigits(char value, char *pos, char tail) {
  *pos = (value / 10) + 48;
  pos++;
  *pos = (value % 10) + 48;
  pos++;
  *pos = tail;
}

static char gsmMsgPending() {
  if (bitGsmMsgDelPending || bitGsmMsgWritePending || bitGsmMsgReadPending || bitGsmMsgSendPending) {
    return 1;
  }
  return 0;
}
 
void gsm1msPing() {  
  #ifdef gsm_async_uart_rx
  if (UART_Data_Ready()) {gsmUartRx();}
  if (UART_Data_Ready()) {gsmUartRx();}
  bitGsmUartRxSync = 0;
  #endif
  wrdGsmGPTmr++;
  dwdGsmGPTmr++;
  wrdGsmDelayTmr++;
  wrdGsmTimeoutTmr++;
  //wrdGsmMsgWriteTmr++;
  bytGSM_StatTmr++;
  if (bytGsmUartRxQuietTimer < 255) {
    bytGsmUartRxQuietTimer++;
  } 
  if (bytGsmUartRxQuietTimer == 100) {
    bitGsmUartRxReset = 1;
    /*#ifdef gsm_debug_state
    if (pstrGsmUartRxBuff != &strGsmUartRxBuff) {bitGsmUartRxBuffCleared = 1;}
    #endif  
    bytGsmUartRxQuietTimer = 0;
    bytGsmUartRxLinesReady = 0;
    bitGsmUartRxLineReady = 0;  
    pstrGsmUartRxBuff = &strGsmUartRxBuff; //"Clear the line"*/
  }
}      

void gsmInit() {
  //Setup
  //GSM_Pwr_Key_Dir = 0; // Should now be done externally
  #ifdef gsm_reset_en
  //GSM_Reset_Dir = 0; // Should now be done externally
  #endif
  //GSM_Stat_Dir = 1; // Should now be done externally
  bitGSM_Stat_On_State = 1;
  //UART1_Init(9600); //Enable UART communication //Must be done externally
  //RC1IE_bit = 1; //Enable interrupt on UART1 Rx
  //Startup
  bytGsmState = gsmstPwrGsmOn;
  bitGsmUartRxLineReady = 0;
  GSM_Pwr_Key->BRR = GSM_Pwr_Key_Pin;
  #ifdef gsm_reset_en
  GSM_Reset->BRR = GSM_Reset_Pin;
  #endif
//  HAL_Delay(100);
//  GSM_Pwr_Key->BSRR = GSM_Pwr_Key_Pin;
//  HAL_Delay(100);
//  GSM_Pwr_Key->BRR = GSM_Pwr_Key_Pin;
  while(GSM_Stat->IDR & GSM_Stat_Pin != GPIO_PIN_SET);
  
  bitGsmDateTimeReadPending = 0;
  bitGsmDateTimeWritePending = 0;
  bitExpectGSM_On = 0;
  bitGSM_PowerOff = 0;
  //strcpy(&strGsmOrigOrDestID, &strExpectedOriginatorID);
  bitGsmMsgDelPending = 0;   // Some of these are also cleared
  bitGsmMsgWritePending = 0; // after the module is swithced on
  bitGsmMsgReadPending = 0;
  bitGsmMsgSendPending = 0;
  bitGsmMsgJustArrived = 0;
  bitGsmGprsPending = 0;
  bitGsmGprsInProgress = 0;
  bitGsmGprsHttpKeepAlive = 0;
  bitGsmGprsRestartFlag = 0;
  bitGSM_Ready = 0;
  #ifdef gsm_async_uart_rx
  bitGsmUartRxSync = 0;
  #endif
  bitGsmUartRxReset = 0;
  #ifdef gsm_debug_state
  bitGsmUartRxCharsLost = 0;
  bitGsmUartRxBuffCleared = 0;
  #endif
}

void gsmPoll() {
  char handled = 0;
  /*while (UART_Data_Ready()) {
    gsmUartRx();
  }*/
  #ifndef gsm_async_uart_rx
  // The below check should be done externally
  // Only if not using mikroE UART_Read routines
  // (mikroE UART_Read routine does it automatically)
  /*if (OERR1_bit) { // Check for UART Rx overflow
    CREN1_bit = 0;
    CREN1_bit = 1;
    #ifdef gsm_debug_state
    bitGsmUartRxCharsLost = 1;
    #endif
  }*/
  //if (UART_Data_Ready()) {gsmUartRx();}
  //if (UART_Data_Ready()) {gsmUartRx();}
  while (UART_Data_Ready()) {gsmUartRx();}
  #endif
  if (bitGsmUartRxReset) {
    gsmUartRxBuffClear();
    //bytGsmUartRxQuietTimer = 0;
    bitGsmUartRxReset = 0;
  }
  #ifdef gsm_debug_state
  if (bitGsmUartRxCharsLost) {
    strcpy(gsmDebugStateStrPtr, "UART Rx Chars Lost\r\n");
    gsmDebugStateStrReady();    
    bitGsmUartRxCharsLost = 0;
  }
  if (bitGsmUartRxBuffCleared) {
    strcpy(gsmDebugStateStrPtr, "UART Rx Buff Cleared\r\n");
    gsmDebugStateStrReady();    
    bitGsmUartRxBuffCleared = 0;  
  }
  #endif  
  #ifndef gsm_blocking_uart_tx
  // Transmit any qued UART communication until there is none left
  if (gsmUartTx()) {return;}  
  #endif
  // Process the current state
  if (!handled && p_gsm_MS_ProcessState) {
    handled = p_gsm_MS_ProcessState(0);
  }
  if (!handled && p_gsm_Msg_ProcessState) {
    handled = p_gsm_Msg_ProcessState(0);
  }
  if (!handled && p_gsm_GPRS_ProcessState) {
    handled = p_gsm_GPRS_ProcessState(0);
  }
  if (!handled) { // If the state has not already been processed,
                  // then process it here
    switch (bytGsmState) {
      case gsmstDelay:
        // -- General delay routine --
        // Entry from: any
        // Exit to: (bytGsmStateAfterDelay)
        // Associated routine: gsmSetStateDelay
        if (wrdGsmDelayTmr >= wrdGsmDelayTime) {
          #ifdef gsm_debug_state
          strcpy(gsmDebugStateStrPtr, "Delay complete\r\n");
          gsmDebugStateStrReady();
          #endif
          gsmSetStateNext(bytGsmStateAfterDelay, 0);
        }
        break;
      // --- GetDateTime Diversion ---
      case gsmstGetDateTimePre:
        // Entry from: (diversion)
        // Exit to: gsmstGetDateTimeQuery
        bytGsmGPCtr = 0; // Reset the general-purpose counter
        gsmSetStateNext(gsmstGetDateTimeQuery, 0);
        break;
      case gsmstGetDateTimeQuery:
        // -- Get date/time from the GSM module --
        // Entry from: gsmstGetDateTimePre, (timeout set by gsmstGetDateTimeQuery)
        // Exit to: gsmstGetDateTimeResponse, (return from diversion)
        gsmUartRxLineClear(); // Make sure new UART data will be received
        if (bytGsmGPCtr < 3) { // If we have been trying this for less than
                            // 3 times then
          gsmUART_Write_Text((char *)strAT);      // Request date/time
          gsmUART_Write_Text((char *)strCCLK);    // from the GSM module
          gsmUART_Write('?');
          gsmUART_Write_Text((char *)strNewLine);
          gsmSetStateNext(gsmstGetDateTimeResponse, 0); // then wait for a response
          gsmSetStateTimeout(500, gsmstGetDateTimeQuery); // for 500ms before asking
                                                       // again
        } else { // Otherwise (trying this for more than 3 times)
          gsmCancelStateTimeout(); // Cancel timeout (if applicable)
          gsmSetStateNext(bytGsmStateAfterDivert, 0); // Give up and carry on
          //bitGsmDateTimeReadPending = 0;
        }
        bytGsmGPCtr++;
        break;
      case gsmstGetDateTimeResponse:
        // -- Interpret response to date/time request --
        // Entry from: gsmstGetDateTimeQuery
        // Exit to: (return from diversion (after OK)), gsmstGetDateTimeQuery
        // Timeout to: gsmstGetDateTimeQuery
        if (bitGsmUartRxLineReady) { // If a line of communication has been rcvd
          wrdGsmTimeoutTmr = 0; //Reset timeout timer
          if (memcmp(&strGsmUartRxBuff, &strCCLK, 5) == 0) { // If it's the type
                                                          // of communication
                                                          // we're looking for
                                                          // then
            gsmCancelStateTimeout(); //Cancel timeout
            gsmExtractDateTime(&strGsmUartRxBuff[6]); //Extract date/time
            //if (dtmGsmEvent.Year > 10 && dtmGsmEvent.Month <= 12 && dtmGsmEvent.Day <= 31 &&
            //    dtmGsmEvent.Hour <= 24 && dtmGsmEvent.Minute <= 60 && dtmGsmEvent.Second <= 60) {
              // If the date/time extracted seem ok
              gsmEvent(gsmevntDateTimeRead); // Call the external routine
              bitGsmDateTimeReadPending = 0;
              // Proceed to next gsmst after "OK"
              gsmSetStateWaitOK(bytGsmStateAfterDivert, 250, bytGsmStateAfterDivert);
            //} else { // Otherwise
            //  // Try again
            //  //gsmSetStateNext(gsmstGetDateTimeQuery, 0);
            //  //gsmSetStateDelay(50, gsmstGetDateTimeQuery);
            //  gsmSetStateWaitOK(gsmstGetDateTimeQuery, 50, gsmstGetDateTimeQuery);
            //  // (delay / wait for ok to allow the GSM module to finish it's transmission)
            //}
          }
          gsmUartRxLineProcessed(); // Allow the next line of comms to be received
        }
        break;
      // --- End of GetDateTime Diversion ---
      // --- SetDateTime Diversion ---
      case gsmstSetDateTimePre:
        // Entry from: (diversion)
        // Exit to: gsmstSetDateTime
        bytGsmGPCtr = 0; // Reset the general-purpose counter
        gsmSetStateNext(gsmstSetDateTime, 0);
        break;
      case gsmstSetDateTime:
        // Entry from: gsmstSetDateTimePre
        // Exit to: (return from diversion), gsmstSetDateTime
        if (bytGsmGPCtr < 3) { // If we have been trying this for less than
                            // 3 times then
          gsmEvent(gsmevntDateTimeWrite); // Request the date/time to read
          // Build the command string
          pstrGsmGP = &strGsmGP[0];
          gsmRTCC_GenerateDigits(dtmGsmEvent.Year, pstrGsmGP, '/');
          pstrGsmGP += 3;
          gsmRTCC_GenerateDigits(dtmGsmEvent.Month, pstrGsmGP, '/');
          pstrGsmGP += 3;
          gsmRTCC_GenerateDigits(dtmGsmEvent.Day, pstrGsmGP, ',');
          pstrGsmGP += 3;
          gsmRTCC_GenerateDigits(dtmGsmEvent.Hour, pstrGsmGP, ':');
          pstrGsmGP += 3;
          gsmRTCC_GenerateDigits(dtmGsmEvent.Minute, pstrGsmGP, ':');
          pstrGsmGP += 3;
          gsmRTCC_GenerateDigits(dtmGsmEvent.Second, pstrGsmGP, '+');
          pstrGsmGP += 3;
          strcpy(pstrGsmGP, "00"); // Ignoring time-zone
          pstrGsmGP += 3;
          *pstrGsmGP = 0; // Mark end of string
          // Send the command
          gsmUART_Write_Text((char *)strAT);
          gsmUART_Write_Text((char *)strCCLK);
          gsmUART_Write('=');
          gsmUART_Write('"');
          gsmUART_Write_Text((char *)strGsmGP);
          gsmUART_Write('"');
          gsmUART_Write_Text((char *)strNewLine);
          gsmSetStateWaitOK(bytGsmStateAfterDivert, 250, gsmstSetDateTime);
          bitGsmDateTimeWritePending = 0;
        } else {
          bitGsmDateTimeWritePending = 1;
          gsmSetStateNext(bytGsmStateAfterDivert, 0); // Give up and carry on
        }
        bytGsmGPCtr++;
        break;
      // --- End of SetDateTime Diversion ---
      case gsmstPwrGsmOffPre:
        // Entry from: gsmstPwringGsmOff, gsmstSimInsertedQuery,
        //             gsmstWaitRegQuery, gsmstCmdOK, gsmCheckStateDivert(),
        //             gsmstPinChkQuery, gsmstPinCmd
        // Exit to: gsmstPwrGsmOff
        bytGsmGPCtr = 0;
        #ifdef gsm_reset_en
        if (GSM_Reset->IDR & GSM_Reset_Pin) == GPIO_PIN_SET)) {
          // Reset pin was activated (see gsmstPwringGsmOff),
          // deactivate it and then wait for the reset to be handled by the module
          GSM_Reset->BRR = GSM_Reset_Pin;
          gsmSetStateDelay(1000, gsmstPwrGsmOff);
        } else
        #endif
        {
          gsmSetStateNext(gsmstPwrGsmOff, 0);
        }
        break;
      case gsmstPwrGsmOff:
        // -- Check if GSM module is powered off / start power-off procedure --
        // Entry from: gsmstPwrGsmOffPre, gsmstPwringGsmOff
        // Exit to: gsmstPwrGsmOn
        if ((bit)(GSM_Stat->IDR & GSM_Stat_Pin) == bitGSM_Stat_On_State) { // If the GSM module is on then
          bitExpectGSM_On = 0;
          GSM_Pwr_Key->BSRR = GSM_Pwr_Key_Pin; // "Press" the Pwr_Key button
          dwdGsmGPTmr = 0;
          bitGsmGPFlag = 0;
          //gsmSetStateDelay(1500, gsmstPwringGsmOff); // for at least 1 second,
                                                     // then release it and check
                                                     // if the module
                                                     // has turned off
          gsmSetStateNext(gsmstPwringGsmOff, 0);
        } else { // If the GSM module is already off
          if (!bitGSM_PowerOff) {
            //bytGsmStateAfterDivert = 0;
            gsmSetStateNext(gsmstPwrGsmOn, 0); // Skip to powering it on
          }
        }
        break;
      case gsmstPwringGsmOff:
        // -- End GSM module power-off procedure / check if it is powered off --
        // Entry from: gsmstPwrGsmOff
        // Exit to: gsmstPwrGsmOff
        // (After delay)
        if ((GSM_Pwr_Key->IDR & GSM_Pwr_Key_Pin) == GPIO_PIN_SET) { // If the Pwr_Key button is "pressed" then
          if ((bitGsmGPFlag && ((bit)(GSM_Stat->IDR & GSM_Stat_Pin) != bitGSM_Stat_On_State) && (wrdGsmGPTmr > 100)) || (dwdGsmGPTmr > 1500)) {
            GSM_Pwr_Key->BRR = GSM_Pwr_Key_Pin; // "Release" the Pwr_Key button
            dwdGsmGPTmr = 0; // Reset the general-purpose timer (integer type)
          } else if ((bit)(GSM_Stat->IDR & GSM_Stat_Pin) != bitGSM_Stat_On_State) {
            if (!bitGsmGPFlag) {
              bitGsmGPFlag = 1;
              wrdGsmGPTmr = 0;
            }
          } else {
            bitGsmGPFlag = 0;
          }
        } else { // If the Pwr_Key button has already been released then
          if ((bit)(GSM_Stat->IDR & GSM_Stat_Pin) != bitGSM_Stat_On_State) { // If the module is now off then
            gsmSetStateDelay(5000, gsmstPwrGsmOff); // give it 5 seconds to rest
                                                 // then go back to the
                                                 // "check" routine (which
                                                 // will immediately
                                                 // redirect to the next
                                                 // routine which should be
                                                 // targeted after the GSM
                                                 // module is powered on)
          } else if (dwdGsmGPTmr > 20000) { // Otherwise, if the module has not
                                         // switched off after 20 seconds then
            #ifdef gsm_reset_en
            if (bytGsmGPCtr < 3) { // If we have tried less than 3 times then
            #endif
              gsmSetStateNext(gsmstPwrGsmOff, 0); // try switching it off again
              bytGsmGPCtr++;
            #ifdef gsm_reset_en
            } else { // Already tried powering off for 3 times, reset the module
              GSM_Reset->BSRR = GSM_Reset_Pin;
              gsmSetStateDelay(500, gsmstPwrGsmOffPre);
            }
            #endif
          }
        }
        break;
      case gsmstPwrGsmOn:
        // -- Check if GSM module is powered on / start power-on procedure --
        // Entry from: (startup), gsmstPwrGsmOff, gsmstPwringGsmOn,
        //             (unexpected module power-off)
        // Exit to: gsmstPwringGsmOn, gsmstIMEIPre
        if (bitGSM_PowerOff) {
          gsmSetStateNext(gsmstPwrGsmOff, 0);
        } else if ((bit)(GSM_Stat->IDR & GSM_Stat_Pin) != bitGSM_Stat_On_State) { // If the GSM module is off
          GSM_Pwr_Key->BSRR = GSM_Pwr_Key_Pin; // "Press" the Pwr_Key button
          gsmSetStateDelay(1000, gsmstPwringGsmOn); // for 1 second, then check if
                                                 // the module has turned on
        } else { // If the GSM module is already on then
          bitExpectGSM_On = 1;
          // Set default/startup values
          bitGsmMsgDelPending = 0;
          //bitGsmMsgWritePending = 0; // Now only done in gsmInit()
          bitGsmMsgReadPending = 0;
          bitGsmMsgSendPending = 0;
          bitGsmGprsRestartFlag = 1;
          bitGsmGprsInProgress = 0; // Failsafe (shouldn't be necessary)
          bitGSM_Ready = 0;
          dwdGsmGPTmr = 0; // Reset the general-purpose timer (long type)
          // skip to next step
          gsmSetStateNext(gsmstIMEIPre, 1);
        }
        break;
      case gsmstPwringGsmOn:
        // -- End GSM module power-on procedure / check if it is powered on --
        // Entry from: gsmstPwrGsmOn
        // Exit to: gsmstPwrGsmOn
        // (After delay)
        if ((GSM_Pwr_Key->IDR & GSM_Pwr_Key_Pin) == GPIO_PIN_SET) { // If the Pwr_Key button is "pressed" then
          GSM_Pwr_Key->BRR = GSM_Pwr_Key_Pin; // "Release" the Pwr_Key button
          wrdGsmGPTmr = 0; // Reset the general-purpose timer (integer type)
        } else { // If the Pwr_Key button has already been released then
          if ((bit)(GSM_Stat->IDR & GSM_Stat_Pin) == bitGSM_Stat_On_State) { // If the module is now on then
            gsmSetStateDelay(10000, gsmstPwrGsmOn); // give it 10 seconds to
                                                 // stabilise before
                                                 // going back to the "check"
                                                 // routine (which will
                                                 // immediately redirect to the
                                                 // next routine which should
                                                 // be targeted after the GSM
                                                 // module is powered on)
          } else if (wrdGsmGPTmr > 5000) { // Otherwise, if the module has not
                                        // switched on after 5 seconds then
            gsmSetStateNext(gsmstPwrGsmOn, 0); // try switching it on again
          }
        }
        break;
      case gsmstIMEIPre:
        // Entry from: gsmstPwrGsmOn
        // Exit to: gsmstIMEIQuery
        bytGsmGPCtr = 0; // Reset the general-purpose counter
        gsmSetStateNext(gsmstIMEIQuery, 0);
        break;
      case gsmstIMEIQuery:
        // Entry from: gsmstIMEIPre, (timeout set by gsmstIMEIQuery)
        // Exit to: gsmstIMEIResponse, gsmstPinChkPre
        gsmUartRxLineClear(); // Make sure new UART data will be received
        if (bytGsmGPCtr < 3) { // If we have been trying this for less than
                            // 3 times then
          gsmUART_Write_Text((char *)strAT);      // Request IMEI
          gsmUART_Write_Text("+CGSN");    // from the GSM module
          gsmUART_Write_Text((char *)strNewLine);
          gsmSetStateNext(gsmstIMEIResponse, 0); // then wait for a response
          gsmSetStateTimeout(500, gsmstIMEIQuery); // for 500ms before asking
                                                       // again
        } else { // Otherwise (trying this for more than 3 times)
          gsmCancelStateTimeout(); // Cancel timeout (if applicable)
          gsmSetStateNext(gsmstPinChkPre, 0); // Give up and carry on
        }
        bytGsmGPCtr++;
        break;
      case gsmstIMEIResponse:
        // Entry from: gsmstIMEIQuery
        // Exit to: gsmstPinChkPre, gsmstIMEIQuery
        // Timeout to: gsmstIMEIQuery
        if (bitGsmUartRxLineReady) { // If a line of communication has been rcvd
          wrdGsmTimeoutTmr = 0; //Reset timeout timer
          if (isnumeric((char *)strGsmUartRxBuff)) { // If it's numeric then
            gsmCancelStateTimeout(); //Cancel timeout
            pstrGsmEventData = (char *)strGsmUartRxBuff;
            gsmEvent(gsmevntIMEI_Read); // Call the external routine
            // Proceed to next gsmst after "OK"
            gsmSetStateWaitOK(gsmstPinChkPre, 250, gsmstPinChkPre);
          }
          gsmUartRxLineProcessed(); // Allow the next line of comms to be received
        }
        break;
      case gsmstPinChkPre:
        // Entry from: gsmstIMEIQuery, gsmstIMEIResponse
        // Exit to: gsmstIMEIQuery
        bytGsmGPCtr = 0; // Reset the general-purpose counter
        gsmSetStateNext(gsmstPinChkQuery, 0);
        break;
      case gsmstPinChkQuery:
        // Entry from: gsmstPinChkPre
        // Exit to: gsmstPinChkResponse, gsmstPwrGsmOffPre
        gsmUartRxLineClear(); // Make sure new UART data will be received
        if (bytGsmGPCtr < 3) { // If we have been trying this for less than
                            // 3 times then
          gsmUART_Write_Text((char *)strAT);      // Request PIN status
          gsmUART_Write_Text((char *)strCPIN);    // from the GSM module
          gsmUART_Write('?');
          gsmUART_Write_Text((char *)strNewLine);
          gsmSetStateNext(gsmstPinChkResponse, 0); // then wait for a response
          gsmSetStateTimeout(500, gsmstPinChkQuery); // for 500ms before asking
                                                       // again
        } else { // Otherwise (trying this for more than 3 times)
          gsmCancelStateTimeout(); // Cancel timeout (if applicable)
          gsmSetStateNext(gsmstPwrGsmOffPre, 0); // Restart the module
        }
        bytGsmGPCtr++;
        break;
      case gsmstPinChkResponse:
        // Entry from: gsmstPinChkQuery
        // Exit to: gsmstSetup_MSHI, gsmstPinPre, gsmstPwrGsmOffPre
        // Timeout to: gsmstPinChkQuery
        if (bitGsmUartRxLineReady) { // If a line of communication has been rcvd
          wrdGsmTimeoutTmr = 0; //Reset timeout timer
          if (memcmp(&strGsmUartRxBuff, &strCPIN, 5) == 0) { // If it's the type
                                                          // of communication
                                                          // we're looking for
                                                          // then
            gsmCancelStateTimeout(); //Cancel timeout
            if (memcmp(&strGsmUartRxBuff + 7, &strREADY, 5) == 0) {
              // No PIN required
              // Proceed to next gsmst after "OK"
              gsmSetStateWaitOK(gsmstSetup_MSHI, 250, gsmstSetup_MSHI);
            } else if (memcmp(&strGsmUartRxBuff + 11, "PIN", 3) == 0) {
              // PIN must be entered
              gsmSetStateWaitOK(gsmstPinPre, 250, gsmstPinPre); // Enter PIN
            } else if (memcmp(&strGsmUartRxBuff + 11, "PUK", 3) == 0) {
              // PUK required
              // (User should remove the SIM card, unblock the PUK, and try again)
              gsmSetStateWaitOK(gsmstPwrGsmOffPre, 250, gsmstPwrGsmOffPre);
            } else {
              // Unrecognised response
              // Try again
              gsmSetStateWaitOK(gsmstPinChkQuery, 250, gsmstPinChkQuery);
            }
          }
          gsmUartRxLineProcessed(); // Allow the next line of comms to be received
        }
        break;
      case gsmstPinPre:
        // Entry from: gsmstPinChkResponse
        // Exit to: gsmstIMEIQuery
        bytGsmGPCtr = 0; // Reset the general-purpose counter
        gsmSetStateNext(gsmstPinCmd, 0);
        break;
      case gsmstPinCmd:
        // Entry from: gsmstPinPre
        // Exit to: gsmstPinResponse, gsmstPwrGsmOffPre
        gsmUartRxLineClear(); // Make sure new UART data will be received
        if (bytGsmGPCtr < 3) { // If we have been trying this for less than
                            // 3 times then
          strGsmGP[0] = 0;
          pstrGsmEventData = (char *)strGsmGP;
          gsmEvent(gsmevntPIN_Request);    // Event to request PIN
          gsmUART_Write_Text((char *)strAT);      // Enter
          gsmUART_Write_Text((char *)strCPIN);    // PIN
          gsmUART_Write('=');
          gsmUART_Write_Text(pstrGsmEventData);
          gsmUART_Write_Text((char *)strNewLine);
          gsmSetStateNext(gsmstPinResponse, 0); // then wait for a response
          gsmSetStateTimeout(2500, gsmstPinCmd); // for 2500ms before trying
                                                       // again
        } else { // Otherwise (trying this for more than 3 times)
          gsmCancelStateTimeout(); // Cancel timeout (if applicable)
          gsmSetStateNext(gsmstPwrGsmOffPre, 0); // Restart the module
        }
        bytGsmGPCtr++;
        break;
      case gsmstPinResponse:
        // Entry from: gsmstPinCmd
        // Exit to: gsmstSetup_MSHI, gsmstDie
        // Timeout to: gsmstPinCmd
        if (bitGsmUartRxLineReady) { // If a line of communication has been rcvd
          wrdGsmTimeoutTmr = 0; //Reset timeout timer
          if (memcmp(&strGsmUartRxBuff, &strOK, 2) == 0) {
            // PIN OK
            gsmCancelStateTimeout(); //Cancel timeout
            //gsmSetStateNext(gsmstSetup_MSHI, 1);
            gsmSetStateDelay(5000, gsmstSetup_MSHI); // Give SIM time to initialise
            gsmSetStateNext(gsmstDelay, 1); // Allow divert
          } else if (memcmp(&strGsmUartRxBuff, &strERROR, 5) == 0) {
            // PIN incorrect
            gsmCancelStateTimeout(); //Cancel timeout
            gsmEvent(gsmevntPIN_Fail); // Event to notify pin fail
            gsmSetStateNext(gsmstDie, 1); // Die
          }
          gsmUartRxLineProcessed(); // Allow the next line of comms to be received
        }
        break;
      case gsmstSetup_MSHI:
        // * Module-specific code hook in *
        // Entry from: gsmstPinChkResponse, gsmstPinResponse
        // Exit to: gsmstSetup_MSHO
        gsmSetStateNext(gsmstSetup_MSHO, 1);
        break;
      case gsmstSetup_MSHO:
        // * Module-specific code hook out *
        // Entry from: gsmstSetup_MSHI
        // Exit to: gsmstEnableCLIP
        gsmSetStateNext(gsmstEnableCLIP, 1);
        break;
      case gsmstEnableCLIP:
        // -- Turn on CLIP (Caller Line Identity Presentation) --
        // Entry from: gsmstSetup_MSHO,
        //             (timeout set by gsmstEnableCLIP)
        // Exit to: gsmstSetMsgFrmtToTxt (after gsmstWaitOK)
        strcpy((char *)strGsmGP, (char *)strAT); // Send the command to turn CLIP on
        strcpy((char *)strGsmGP +2, (char *)strCLIP);
        strcpy((char *)strGsmGP +7, (char *)strSetOn);
        gsmSetStateCmdOK((char *)strGsmGP, gsmstSetMsgFrmtToTxt, 0);
        break;
      case gsmstSetMsgFrmtToTxt:
        // -- Set SMS format to text mode --
        // Entry from: gsmstEnableCLIP, (timeout set by gsmstSetMsgFrmtToTxt)
        // Exit to: gsmstSetMsgAlertOn (after gsmstWaitOK)
        gsmSetStateCmdOK("AT+CMGF=1", gsmstSetMsgAlertOnPre, 0); // Set SMS format to text mode
        break;
      case gsmstSetMsgAlertOnPre:
        // Entry from: gsmstSetMsgFrmtToTxt
        // Exit to: gsmstSetMsgAlertOn
        bytGsmGPCtr = 0;
        bitGsmGPFlag = 0;
        gsmSetStateNext(gsmstSetMsgAlertOn, 1);
        break;
      case gsmstSetMsgAlertOn:
        // -- Set SMS arrival alerts on --
        // Entry from: gsmstSetMsgAlertOnPre, (timeout set by gsmstSetMsgAlertOn)
        // Exit to: gsmstWaitRegPre (after gsmstWaitOK)
        if (bitGsmGPFlag) {
          gsmSetStateDelay(5000, gsmstSetMsgAlertOn);
          bitGsmGPFlag = 0;
          bytGsmGPCtr++;
        } else {
          if (bytGsmGPCtr < 5) {
            gsmSetStateCmdOK("AT+CNMI=2,1", gsmstWaitRegPre, gsmstSetMsgAlertOn); // Enable new message indications
            bitGsmGPFlag = 1;
          } else {
            gsmSetStateNext(gsmstPwrGsmOffPre, 0);
          }
        }
        break;
      case gsmstWaitRegPre:
        // Entry from: gsmstSetMsgAlertOn, gsmstStandby, gsmstDeleteMsg,
        //             gsmstWriteMsgAbortWtngOK, any
        // Exit to: gsmstWaitRegQuery
        dwdGsmGPTmr = 0; // Reset the general-purpose timer (long type)
        gsmSetStateNext(gsmstWaitRegQuery, 0);
        if (bytGsmStateAfterReg == 0) {
          bytGsmStateAfterReg = gsmstStandbyPre;
          bitGsmMsgReadPending = 1;
          //bytGsmStateAfterReg = gsmstReadMsgRequest;
        }
        break;
      case gsmstWaitRegQuery:
        // -- Request network registration status --
        // Entry from: gsmstWaitRegPre, (timeout set by gsmstWaitRegQuery)
        // Exit to: gsmstWaitRegResponse, gsmstPwrGsmOffPre
        gsmUartRxLineClear(); // Make sure new UART data will be received
        if (dwdGsmGPTmr < 60000) { // If we have been trying this for less than a
                                // minute then
          gsmUART_Write_Text((char *)strAT);     // Request network registration
          gsmUART_Write_Text((char *)strCREG);   // status from the GSM module
          gsmUART_Write('?');
          gsmUART_Write_Text((char *)strNewLine);
          gsmSetStateNext(gsmstWaitRegResponse, 0); // then wait for a response
          gsmSetStateTimeout(500, gsmstWaitRegQuery); // for 500ms before asking
                                                   // again
        } else { // Otherwise (trying this for longer than a minute)
          gsmCancelStateTimeout(); // Cancel timeout (if applicable)
          gsmSetStateNext(gsmstPwrGsmOffPre, 0); // Restart the module
        }
        break;
      case gsmstWaitRegResponse:
        // -- Process response to network registration status request --
        // Entry from: gsmstWaitRegQuery
        // Exit to: (bytGsmStateAfterReg) - gsmstStandbyPre by default,
        //          gsmstWaitRegQuery
        // Timeout to: gsmstWaitRegQuery
        if (bitGsmUartRxLineReady) { // If a line of communication has been rcvd
          wrdGsmTimeoutTmr = 0; //Reset timeout timer
          if (memcmp(&strGsmUartRxBuff, &strCREG, 5) == 0) { // If it's the type
                                                        // of communication
                                                        // we're looking for
                                                        // then
            gsmCancelStateTimeout(); //Cancel timeout
            if ((*((char *)strGsmUartRxBuff + 9) == '1') ||
                (*((char *)strGsmUartRxBuff + 9) == '5')) {
              // If registered then proceed to next gsmst, after "OK"
              gsmSetStateWaitOK(bytGsmStateAfterReg, 250, bytGsmStateAfterReg);
              bytGsmStateAfterReg = 0; // Reset to default
              bitGSM_Ready = 1;
            } else { // If not registered then
              gsmSetStateDelay(5000, gsmstWaitRegQuery); // wait 5 seconds before
                                                      // checking again
              bitGSM_Ready = 0;
            }
          }
          gsmUartRxLineProcessed(); // Allow the next line of comms to be received
        }
        break;
      case gsmstStandbyPre:
        // Entry from: gsmstWaitingCLIP, (timeout set by gsmstStandby),
        //             (timeout set by gsmstWaitingCLIP), gsmstSendMsg,
        //             gsmstWaitingNO_CARRIER, gsmstReadMsgHeader
        //             (timeout set by gsmstReadMsgRequest),
        //             (timeout set by gsmstReadMsgHeader),
        //             gsmstReadMsgWaitingBlank, gsmstReadMsgWaitingOK,
        //             gsmstWaitRegResponse
        // Exit to: gsmstStandby
        wrdGsmGPTmr = 0;
        dwdGsmGPTmr = 0;
        bitGsmMsgJustArrived = 0;
        bitGsmGprsInProgress = 0; // Failsafe (shouldn't be necessary)
        gsmSetStateNext(gsmstStandby, 0);
        break;
      case gsmstStandby:
        // -- Wait for activity --
        // Entry from: gsmstStandbyPre
        // Exit to: gsmstWaitingCLIP, gsmstReadMsgRequest, gsmstWriteMsgPre,
        //          gsmstSendMsgPre, gsmstDeleteMsgPre, gsmstWaitRegPre
        // Note that the order of the items in the if .. else if block
        // is important
        if (bitGsmUartRxLineReady) { // If a line of communication has been rcvd
          dwdGsmGPTmr = 0; // Reset the general-purpose timer (long type)
          if (strcmp((char *)strGsmUartRxBuff, (char *)strRING) == 0) { // If "RING" was received
            gsmSetStateNext(gsmstWaitingCLIP, 0); // Check for caller ID
            gsmSetStateTimeout(500, gsmstStandbyPre); // for up to 500ms, before
                                                   // coming back to standby
          } else if (memcmp(&strGsmUartRxBuff, &strCMTI, 5) == 0) { // If msg arrvd
            if (!gsmMsgPending()) {
              bitGsmMsgJustArrived = 1;
              bitGsmMsgReadPending = 1;
              gsmSetStateNext(gsmstMsgHook, 0); // Read the message
            }
          }
          gsmUartRxLineProcessed(); // Allow new comms to be received
        } else if (gsmMsgPending()) {
          // Message (SMS) action pending
          gsmSetStateNext(gsmstMsgHook, 1);
        } else if (bitGsmGprsPending) {
          // GPRS
          gsmSetStateNext(gsmstGPRS_Hook, 1);
        } else if (gsmCheckStateDivert()) {
          // Divert pending
          gsmSetStateNext(gsmstWaitRegPre, 1);
        } else if (dwdGsmGPTmr >= 60000) { // If there is no activity for more
                                        // than a minute then
          gsmSetStateNext(gsmstWaitRegPre, 1);   // Check that we're still registered
                                           // on the network
        }
        /*if (wrdGsmGPTmr >= 1000) { // Flash the relay 5 LED at 1 seconds intervals
          LATD5_bit = !LATD5_bit;
          wrdGsmGPTmr = 0;
        }*/
        break;
      case gsmstWaitingCLIP:
        // -- Check for CLIP --
        // Entry from: gsmstStandby
        // Exit to: gsmstWaitingNO_CARRIER
        // Timeout to: gsmstStandbyPre
        if (bitGsmUartRxLineReady) { // If a line of communication has been rcvd
          if (memcmp(&strGsmUartRxBuff, &strCLIP, 5) == 0) { // If it's "+CLIP"
            // Try to extract the caller ID
            if (gsmExtractCallerId((char *)strGsmUartRxBuff, (char *)strGsmOrigOrDestID) == 1) {
              #ifdef gsm_debug_state
              strcpy(gsmDebugStateStrPtr, "Incoming call from ");
              strcat(gsmDebugStateStrPtr, &strGsmOrigOrDestID);
              strcat(gsmDebugStateStrPtr, &strNewLine);
              gsmDebugStateStrReady();
              #endif
              gsmCancelStateTimeout();
              /*// Check if the caller ID is what we expected
              if (strcmp((char *)strGsmOrigOrDestID, (char *)strExpectedOriginatorID) == 0) {
                LATD0_bit = !LATD0_bit; // Toggle PortD.0
              }*/
              pstrGsmEventOriginatorID = (char *)strGsmOrigOrDestID;
              gsmEvent(gsmevntMissedCall); // Call the external routine
              gsmSetStateNext(gsmstWaitingNO_CARRIER, 0); // Wait for end of call
              gsmSetStateTimeout(5000, gsmstStandbyPre);
            }
          }
          gsmUartRxLineProcessed(); // Allow new comms to be received
        }
        break;
      case gsmstWaitingNO_CARRIER:
        // -- Wait for ringing to end --
        // Entry from: gsmstWaitingCLIP
        // Exit to: gsmstStandbyPre
        // Timeout to: gsmstStandbyPre
        if (bitGsmUartRxLineReady) { // If a line of communication has been rcvd
          if (strcmp((char *)strGsmUartRxBuff, (char *)strRING) == 0) { // If still ringing
            wrdGsmTimeoutTmr = 0; // Reset timeout
          } else if (strcmp((char *)strGsmUartRxBuff, (char *)strNOCARRIER) == 0) {
            // If "NO CARRIER" received then
            gsmSetStateNext(gsmstStandbyPre, 1); // Go back to standby
            gsmCancelStateTimeout(); // Cancel the timeout
          }
          gsmUartRxLineProcessed(); // Allow new comms to be received
        }
        break;
      case gsmstMsgHook:
        // * Message (SMS) code hook *
        // Entry from: gsmstStandby
        // Exit to: gsmstStandbyPre
        if (bitGsmMsgWritePending) {
          gsmEvent(gsmevntMsgDiscarded); // Fail if the module does not hook in
        }
        bitGsmMsgWritePending = 0;
        bitGsmMsgReadPending = 0;
        bitGsmMsgSendPending = 0;
        bitGsmMsgDelPending = 0;
        gsmSetStateNext(gsmstStandbyPre, 1);
        break;
      case gsmstGPRS_Hook:
        // * GPRS code hook *
        // Entry from: gsmstStandby
        // Exit to: gsmstStandbyPre
        gsmEvent(gsmevntGprsFailed); // Fail if the module does not hook in
        bitGsmGprsPending = 0;
        gsmSetStateNext(gsmstStandbyPre, 1);
        break;
      case gsmstWaitingOK:
        // -- Waiting for "OK" --
        // Entry from: any
        // Exit to: (bytGsmStateAfterOK)
        // Timeout to: (bytGsmStateAfterTimeout)
        if (bitGsmUartRxLineReady) { // If a line of communication has been rcvd
          if (strcmp((char *)strGsmUartRxBuff,(char *) strOK) == 0) { // If "OK" was received
            gsmSetStateNext(bytGsmStateAfterOK, 0);
            gsmCancelStateTimeout(); // Cancel timeout (if applicable)
          }
          gsmUartRxLineProcessed(); // Allow new comms to be received
        }
        break;
      case gsmstCmdOK:
        // -- Issue a command and then wait for "OK" --
        // Entry from: any
        // Exit to: (bytGsmStateAfterOK)
        // Fail to: (bytGsmStateAfterCmdFail)
        gsmUartRxLineClear(); // Make sure that new UART data will be received
        //if (dwdGsmGPTmr < 10000) { // If we have been trying this for less than
        //                           // 10 seconds then
        if (bytGsmCmdOKCtr < 3) { // If we have tried this less than 3 times then
          //gsmUART_Write_Text((char *)strAT); // Send the command
          gsmUART_Write_Text((char *)pstrGsmCommand);
          gsmUART_Write_Text((char *)strNewLine);
          // then wait for a response for 1 second before trying again
          gsmSetStateWaitOK(bytGsmStateAfterOK, 1000, gsmstCmdOK);
          bytGsmCmdOKCtr++;
        } else { // Otherwise (trying this for longer than 10 seconds)
          gsmSetStateNext(bytGsmStateAfterCmdFail, 0); // Fail
        }
        break;
      case gsmstDie:
        if (gsmCheckStateDivert()) {
          gsmSetStateNext(gsmstDie, 1);
        }
        break;
      default:
        // Something has gone wrong, and the state is not recognised
        #ifdef gsm_debug_state
        strcpy(gsmDebugStateStrPtr, "State ");
        ByteToStr(bytGsmState, gsmDebugStateStrPtr + 6);
        strcpy(gsmDebugStateStrPtr + 9, " not defined.\r\n");
        gsmDebugStateStrReady();
        #endif
        gsmSetStateNext(gsmstPwrGsmOn, 0);
        break;
    } // End of state machine switch
  }
  // Restart if the GSM module is powered down
  if (bitExpectGSM_On && ((bit)(GSM_Stat->IDR & GSM_Stat_Pin) != bitGSM_Stat_On_State)) {
    if (bytGSM_StatTmr > 100) {
      bitExpectGSM_On = 0;
      gsmSetStateDelay(2500, gsmstPwrGsmOn);
      gsmCancelStateTimeout();
    }
  } else {
    bytGSM_StatTmr = 0; // Reset
  }
  // Timeout Timer
  if (bytGsmStateAfterTimeout != 0) {
    if (wrdGsmTimeoutTmr >= wrdGsmTimeoutTime) {
      #ifdef gsm_debug_state
      strcpy(gsmDebugStateStrPtr, "Timed out\r\n");
      gsmDebugStateStrReady();
      #endif
      gsmSetStateNext(bytGsmStateAfterTimeout, 0);
      gsmCancelStateTimeout();
      gsmUartRxLineClear(); // Make sure new comms will be received
    }
  }
}

void gsmPowerSetOnOff(char power_on) {
  if (power_on) { bitGSM_PowerOff = 0; } else { bitGSM_PowerOff = 1; }
}

char gsmReady() { // Indicates if the module is registered on the network
  //return bitGSM_Ready;
  if (bitGSM_Ready) { return 1; } else { return 0; }
}

void gsmDateTimeRead() {
  bitGsmDateTimeReadPending = 1;
}

void gsmDateTimeWrite() {
  bitGsmDateTimeWritePending = 1;
}

void gsmMsgSend(char *Message, char *DestinationID) {
  pstrGsmMsgSendTxt = Message;
  pstrGsmMsgSendNum = DestinationID;
  bitGsmMsgWritePending = 1;
}

char gsmMsgSendPending() {
  //return bitGsmMsgWritePending;
  if (bitGsmMsgWritePending) { return 1; } else { return 0; }
}

void gsmMsgSendCancel() {
  bitGsmMsgWritePending = 0;
}

char gsmMsgJustArrived() {
  //return bitGsmMsgJustArrived;
  if (bitGsmMsgJustArrived) { return 1; } else { return 0; }
}

char gsmGprsHttpGet(char* url) {
  if (!bitGsmGprsInProgress) {
    pstrGsmGprsURL = url;
    wrdGsmGprsDataSize = 0;
    bitGsmGprsPending = 1;
    return 1;
  } else {
    return 0;
  }
}

char gsmGprsHttpPost(char* url, char* postdata) {
  if (!bitGsmGprsInProgress) {
    pstrGsmGprsURL = url;
    pstrGsmGprsData = postdata;
    wrdGsmGprsDataSize = strlen(postdata);
    bitGsmGprsPending = 1;
    return 1;
  } else {
    return 0;
  }
}

char gsmGprsPending() {
  //return bitGsmGprsPending;
  if (bitGsmGprsPending || bitGsmGprsInProgress) { return 1; } else { return 0; }
}

void gsmGprsCancel() {
  bitGsmGprsPending = 0;
}

void gsmGprsSetHttpKeepAlive(char keepalive) {
  if (keepalive) { bitGsmGprsHttpKeepAlive = 1; } else { bitGsmGprsHttpKeepAlive = 0; }
}