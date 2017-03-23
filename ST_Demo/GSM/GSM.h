#ifndef __GSM_H
#define __GSM_H
//#define gsm_echo_int_rx // Enable echoing of communication with the GSM module to the external interface
//#define gsm_debug_state // Enable outputting of state machine debug msgs

//#define gsm_reset_en

// --- GSM ---
#ifndef bit
#define bit _Bool
#endif 

#include "stm32l4xx_hal.h"
#include "string.h"

#define GPIO_GSM_Pwr_Key_CLK_ENABLE()   __HAL_RCC_GPIOE_CLK_ENABLE()
#define GSM_Pwr_Key_Port GPIOE
#define GSM_Pwr_Key_Pin  GPIO_PIN_10 //this will use GPIOB Pin8 as the handler
extern volatile GPIO_TypeDef *GSM_Pwr_Key;// = (GPIO_TypeDef *)(GSM_Pwr_Key_Port);

#ifdef gsm_reset_en
#define GPIO_GSM_Reset_CLK_ENABLE() __HAL_RCC_GPIOE_CLK_ENABLE()
#define GSM_Reset_Port GPIOE
#define GSM_Reset_Pin  GPIO_PIN_11 //this will use GPIOB Pin9 as the handler
extern volatile GPIO_TypeDef *GSM_Reset;// = (GPIO_TypeDef *)(GSM_Reset_Port);
#endif
#define GPIO_GSM_Stat_CLK_ENABLE()   __HAL_RCC_GPIOE_CLK_ENABLE() // Indicates if the module is powered on
#define GSM_Stat_Port GPIOE
#define GSM_Stat_Pin  GPIO_PIN_12 //this will use GPIOB Pin7 as the handler
extern volatile GPIO_TypeDef *GSM_Stat;// = (GPIO_TypeDef *)(GSM_Stat_Port);

#define USART_GSM          UART4     
#define USART_GSM_BAUDRATE 9600
#define TimeOut_TX 1000
#define TimeOut_RX 1000

extern UART_HandleTypeDef UartGSMHandle;

#define USART_GSM_CLK_ENABLE()              __HAL_RCC_UART4_CLK_ENABLE()
#define USART_GSM_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define USART_GSM_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define USART_GSM_TX_PIN                    GPIO_PIN_0
#define USART_GSM_TX_GPIO_PORT              GPIOA
#define USART_GSM_RX_PIN                    GPIO_PIN_1
#define USART_GSM_RX_GPIO_PORT              GPIOA
#define USART_GSM_TX_AF                     GPIO_AF8_UART4
#define USART_GSM_RX_AF                     GPIO_AF8_UART4

#ifdef __GNUC__
extern const char gsmevntDateTimeRead;
extern const char gsmevntDateTimeWrite;
extern const char gsmevntSignalQualityRead;
extern const char gsmevntIMEI_Read;
extern const char gsmevntPIN_Request;
extern const char gsmevntPIN_Fail;
extern const char gsmevntMissedCall;
extern const char gsmevntMsgRcvd;
extern const char gsmevntMsgDrafted;
extern const char gsmevntMsgDiscarded;
extern const char gsmevntMsgSent;
extern const char gsmevntMsgSendFailed;
extern const char gsmevntGprsFailed;
extern const char gsmevntGprsHttpResultErr;
extern const char gsmevntGprsHttpResponseLine;

#ifndef struct_DateTime
typedef struct DateTime {
  char Year, Month, Day, Hour, Minute, Second;
} TDateTime;
#define struct_DateTime
#endif
extern void gsmEvent(char GsmEventType);
extern char* pstrGsmEventOriginatorID;
extern char* pstrGsmEventData;
extern TDateTime dtmGsmEvent;

extern void gsmInit();
extern void gsm1msPing();
extern void gsmPoll();
extern void gsmPowerSetOnOff(char power_on);
extern char gsmReady();
extern void gsmDateTimeRead();
extern void gsmDateTimeWrite();
extern void gsmMsgSend(char *Message, char *DestinationID);
extern char gsmMsgSendPending();
extern void gsmMsgSendCancel();
extern char gsmMsgJustArrived();
extern char gsmGprsHttpGet(char* url);
extern char gsmGprsHttpPost(char* url, char* postdata);
extern char gsmGprsPending();
extern void gsmGprsCancel();
extern void gsmGprsSetHttpKeepAlive(char keepalive);

#ifdef gsm_debug_state
extern char *gsmDebugStateStrPtr;
extern void gsmDebugStateStrReady();
#endif

extern const char gsmevntMsgRcvd;
extern const char gsmevntMsgDrafted;
extern const char gsmevntMsgDiscarded;
extern const char gsmevntMsgSent;
extern const char gsmevntMsgSendFailed;

extern const char gsmstPwrGsmOffPre;
extern const char gsmstPwrGsmOff;
extern const char gsmstPwringGsmOff;
extern const char gsmstPwrGsmOn;
extern const char gsmstPwringGsmOn;
extern const char gsmstPinChkPre;
extern const char gsmstPinChkQuery;
extern const char gsmstSetup_MSHI;
extern const char gsmstSetup_MSHO;
extern const char gsmstWaitRegPre;
extern const char gsmstStandbyPre;
extern const char gsmstMsgHook;
extern const char gsmstGPRS_Hook;

#else

#define gsmevntDateTimeRead             150
#define gsmevntDateTimeWrite            151
#define gsmevntSignalQualityRead        152
#define gsmevntIMEI_Read                20
#define gsmevntPIN_Request              30
#define gsmevntPIN_Fail                 31
#define gsmevntMissedCall               70
#define gsmevntMsgRcvd                  80
#define gsmevntMsgDrafted               81
#define gsmevntMsgDiscarded             82
#define gsmevntMsgSent                  83
#define gsmevntMsgSendFailed            84
#define gsmevntGprsFailed               110
#define gsmevntGprsHttpResultErr        111
#define gsmevntGprsHttpResponseLine     112

#ifndef struct_DateTime
typedef struct DateTime {
  char Year, Month, Day, Hour, Minute, Second;
} TDateTime;
#define struct_DateTime
#endif
extern void gsmEvent(char GsmEventType);
extern char* pstrGsmEventOriginatorID;
extern char* pstrGsmEventData;
extern TDateTime dtmGsmEvent;

extern void gsmInit();
extern void gsm1msPing();
extern void gsmPoll();
extern void gsmPowerSetOnOff(char power_on);
extern char gsmReady();
extern void gsmDateTimeRead();
extern void gsmDateTimeWrite();
extern void gsmMsgSend(char *Message, char *DestinationID);
extern char gsmMsgSendPending();
extern void gsmMsgSendCancel();
extern char gsmMsgJustArrived();
extern char gsmGprsHttpGet(char* url);
extern char gsmGprsHttpPost(char* url, char* postdata);
extern char gsmGprsPending();
extern void gsmGprsCancel();
extern void gsmGprsSetHttpKeepAlive(char keepalive);

#ifdef gsm_debug_state
extern char *gsmDebugStateStrPtr;
extern void gsmDebugStateStrReady();
#endif

#define gsmstDelay  1
#define gsmstWaitingOK  2
#define gsmstCmdOK  3
#define gsmstDie  9
// Diversions
#define gsmstGetDateTimePre  150
#define gsmstGetDateTimeQuery  151
#define gsmstGetDateTimeResponse  152
#define gsmstSetDateTimePre  160
#define gsmstSetDateTime     161

#define gsmstPwrGsmOffPre       10
#define gsmstPwrGsmOff          11
#define gsmstPwringGsmOff       12
#define gsmstPwrGsmOn           13
#define gsmstPwringGsmOn        14

#define gsmstIMEIPre  20
#define gsmstIMEIQuery  21
#define gsmstIMEIResponse  22

#define gsmstPinChkPre          30
#define gsmstPinChkQuery        31
#define gsmstPinChkResponse  32
#define gsmstPinPre  33
#define gsmstPinCmd  34
#define gsmstPinResponse  35

#define gsmstSetup_MSHI         40
#define gsmstSetup_MSHO         41
#define gsmstEnableCLIP  42
#define gsmstSetMsgFrmtToTxt  43
#define gsmstSetMsgAlertOnPre  44
#define gsmstSetMsgAlertOn  45


#define gsmstWaitRegPre         50
#define gsmstWaitRegQuery  51
#define gsmstWaitRegResponse  52

#define gsmstStandbyPre         60
#define gsmstStandby  61

#define gsmstWaitingCLIP  70
#define gsmstWaitingNO_CARRIER  71

#define gsmstMsgHook            109
#define gsmstGPRS_Hook          110
#endif /* __GNUC__ */

char UART_Tx_Idle(void);
void UART_Write(char *pData);
char UART_Read(void);
bit UART_Data_Ready(void);
void UART_GSM_Init(void);

extern unsigned int wrdGsmGPTmr;
extern unsigned long dwdGsmGPTmr;
extern char bytGsmGPCtr;
extern bit bitGsmGPFlag;
extern char strGsmGP[];
extern char* pstrGsmGP;
extern char charGsmUartRx;
extern char strGsmUartRxBuff[];
extern bit bitGsmUartRxLineReady;
extern char bytGsmUartRxQuietTimer;
extern char bytGsmState;
extern unsigned int wrdGsmTimeoutTime;
extern unsigned int wrdGsmTimeoutTmr;
extern char bytGsmStateAfterTimeout;
extern char strAT[];
extern char strOK[];
extern char strERROR[];
extern char strNewLine[];
extern bit bitGSM_Stat_On_State;
extern char *pstrGsmMsgSendTxt;
extern char *pstrGsmMsgSendNum;
extern bit bitGsmMsgDelPending;
extern bit bitGsmMsgWritePending;
extern bit bitGsmMsgReadPending;
extern bit bitGsmMsgSendPending;
extern bit bitGsmMsgJustArrived;
extern bit bitGsmGprsPending;
extern bit bitGsmGprsInProgress;
extern char *pstrGsmGprsURL;
extern char *pstrGsmGprsData;
extern unsigned int wrdGsmGprsDataSize;
extern bit bitGsmGprsHttpKeepAlive;
extern bit bitGsmGprsRestartFlag;
extern char strGsmOrigOrDestID[];
extern bit bitGSM_PowerOff;
extern bit bitExpectGSM_On;
extern bit bitGSM_Stat_On_State;

extern void gsmUartRxLineClear();
extern void gsmUartRxLineProcessed();
extern void gsmUART_Write_Text(char *UART_text);
extern void gsmUART_Write(char data_);
extern void gsmSetStateNext(char stateNext, char allowDivert);
extern void gsmSetStateTimeout(unsigned int time_ms, char stateAfterTimeout);
extern void gsmCancelStateTimeout();
extern void gsmSetStateDelay(unsigned int time_ms, char stateAfterDelay);
extern void gsmSetStateCmdOK(char* cmd, char stateAfterOK, char stateAfterFail);
extern void gsmSetStateWaitOK(char stateAfterOK, unsigned int timeout,
                              char stateAfterTimeout);
extern void gsmSetStateWaitReg(char stateAfterReg);
extern void gsmExtractDateTime(char *source);

// --- Modules ---

extern void gsm_MS_Init();
extern char (*p_gsm_MS_ProcessState)(char dummy);
#ifdef gsm_debug_state
extern char (*p_gsm_MS_strcatState)(char *to, char state);
#endif

extern void gsm_Msg_Init();
extern char (*p_gsm_Msg_ProcessState)(char dummy);
#ifdef gsm_debug_state
extern char (*p_gsm_Msg_strcatState)(char *to, char state);
#endif

extern void gsm_GPRS_Init();
extern char (*p_gsm_GPRS_ProcessState)(char dummy);
#ifdef gsm_debug_state
extern char (*p_gsm_GPRS_strcatState)(char *to, char state);
#endif
#endif /*#ifndef __GSM_H*/