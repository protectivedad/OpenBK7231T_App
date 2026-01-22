#ifdef WINDOWS

#include "selftest_local.h"

// This will allow us to treat multiple PWMs on a single channel as one PWM channel.
// Thanks to this users can turn for example RGB LED controller
// into high power 3-outputs single colors LED controller
void Test_TwoPWMsOneChannel_Test1() {
	int pwmCount;

	// reset whole device
	SIM_ClearOBK(0);

	PIN_SetPinChannelForPinIndex(9, 0);
	PIN_SetPinRoleForPinIndex(9, IOR_PWM);

	PIN_SetPinChannelForPinIndex(11, 0);
	PIN_SetPinRoleForPinIndex(11, IOR_PWM);

	pwmCount = PWM_count();
	// two PWMs on one channel counts as one PWM
	SELFTEST_ASSERT(pwmCount == 1);

	PIN_SetPinChannelForPinIndex(12, 0);
	PIN_SetPinRoleForPinIndex(12, IOR_PWM);

	pwmCount = PWM_count();
	// three PWMs on one channel counts as one PWM
	SELFTEST_ASSERT(pwmCount == 1);

	pwmCount = PWM_count();
	PIN_SetPinChannelForPinIndex(12, 1);
	// now we have two channels with 3 pwms
	SELFTEST_ASSERT(pwmCount == 2);
}
void Test_TwoPWMsOneChannel() {
	Test_TwoPWMsOneChannel_Test1();


}


#endif
