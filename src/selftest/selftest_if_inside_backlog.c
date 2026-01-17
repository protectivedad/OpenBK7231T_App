#ifdef WINDOWS

#include "selftest_local.h"

void Test_IF_Inside_Backlog() {
	// reset whole device
	SIM_ClearOBK(0);

	// CW bulb requires two PWMs - first on channel 1, second on channel 2
	// NOTE: we also support case where first PWM is on channel 0, and second on 1
	PIN_SetPinRoleForPinIndex(24, IOR_PWM);
	PIN_SetPinChannelForPinIndex(24, 1);

	PIN_SetPinRoleForPinIndex(26, IOR_PWM);
	PIN_SetPinChannelForPinIndex(26, 2);




}


#endif
