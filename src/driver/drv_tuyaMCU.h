

void TuyaMCU_Send(byte *data, int size);
void TuyaMCU_Send_RawBuffer(byte *data, int len);
bool TuyaMCU_IsChannelUsedByTuyaMCU(int channelIndex);
void TuyaMCU_EnableAutomaticSending(bool enable);
