#include "GSM.h"
#include "Str.h"

// -- Constants --
// GSM State Machine States
// Note: In order not to overrun the output buffer
//       (when gsm_debug_state is defined)
//       state names should not be longer than 26 characters
//       i.e. last char here -----> x
#ifdef __GNUC__
const char gsmstEnableLTS = 200;
#else
#define gsmstEnableLTS 200
#endif /*__GNUC__*/

#ifdef gsm_debug_state
#ifdef __GNUC__
const char cstr_gsmstEnableLTS[] = "gsmstEnableLTS";
#else
#define cstr_gsmstEnableLTS[]  "gsmstEnableLTS"
#endif
#endif

#ifdef gsm_debug_state
static char gsm_MS_strcatState(char *to, char state) {
  switch (state) {
    case gsmstEnableLTS: strcat(to, RomTxt30(&cstr_gsmstEnableLTS)); break;
    default: return 0; break;
  }
  return 1;
}
#endif

static char gsm_MS_ProcessState(char dummy) {
  switch (bytGsmState) {
    case gsmstSetup_MSHI:
      // Entry from: (overload)
      // Exit to: gsmstEnableLTS
      gsmSetStateNext(gsmstEnableLTS, 1);
      break;
    case gsmstEnableLTS:
      // -- Turn on LTS (Local TimeStamp) (Receive Time from Network) --
      // Entry from: gsmstSetup_MSHI,
      //             (timeout set by gsmstEnableLTS)
      // Exit to: gsmstSetup_MSHO (after gsmstWaitOK)
      gsmSetStateCmdOK("AT+CTZU=2", gsmstSetup_MSHO, 0); // Send the command to turn LTS on
      break;
    default:
      return 0; // State was not processed here
      break;
  }
  return 1; // State was processed here
}

void gsm_MS_Init() {
  p_gsm_MS_ProcessState = &gsm_MS_ProcessState;
  #ifdef gsm_debug_state
  p_gsm_MS_strcatState = &gsm_MS_strcatState;
  #endif
}