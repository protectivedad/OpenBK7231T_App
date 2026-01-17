#ifndef __DRV_BATTERY_H__
#define __DRV_BATTERY_H__

void Batt_Init();
void Batt_OnEverySecond();
void Batt_AppendInformationToHTTPIndexPage(http_request_t *request, int bPreState);
void Batt_StopDriver();
void Battery_reserveIORoles(int driverIndex);

// battery public void 
int Battery_lastreading(int type);

// read last value of battery driver value
enum {
	OBK_BATT_VOLTAGE,
	OBK_BATT_LEVEL
};

#endif // __DRV_BATTERY_H__