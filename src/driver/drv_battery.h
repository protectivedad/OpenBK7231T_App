#ifndef __DRV_BATTERY_H__
#define __DRV_BATTERY_H__

uint32_t Battery_frameworkRequest(uint32_t obkfRequest, uint32_t arg);
void Battery_onEverySecond();
void Battery_quickTick();
void Batt_AppendInformationToHTTPIndexPage(http_request_t *request, int bPreState);

// battery public void 
uint32_t Battery_lastreading(uint32_t type);

// read last value of battery driver value
enum {
	OBK_BATT_VOLTAGE,
	OBK_BATT_LEVEL
};

#endif // __DRV_BATTERY_H__