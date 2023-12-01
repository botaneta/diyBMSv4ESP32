#ifndef DIYBMS_PYLON_CANBUS_H_
#define DIYBMS_PYLON_CANBUS_H_

#include "defines.h"
#include "Rules.h"
#include <driver/twai.h>

void pylon_message_356();
void pylon_message_35e();
void pylon_message_351();
void pylon_message_355();
void pylon_message_359();
void pylon_message_35c();



const uint8_t DEFAULT_DEVICE_ID_ADDRESS=1; 
void pylonHV_message_0x4210(bool extend);
void pylonHV_message_0x4220(bool extend);
void pylonHV_message_0x4230(bool extend);
void pylonHV_message_0x4240(bool extend);
void pylonHV_message_0x4250(bool extend);
void pylonHV_message_0x4260(bool extend);
void pylonHV_message_0x4270(bool extend);
void pylonHV_message_0x4280(bool extend);
void pylonHV_message_0x4290(bool extend);
void pylonHV_message_0x42A0(bool extend);

void pylonHV_message_0x7320(bool extend);
void pylonHV_message_0x7330(bool extend);
void pylonHV_message_0x7310(bool extend);
void pylonHV_message_0x7340(bool extend);

void pylonHV_send_message_info(bool extend);
void pylonHV_send_message_status(bool extend);

extern uint8_t TotalNumberOfCells();
extern Rules rules;
extern currentmonitoring_struct currentMonitor;
extern diybms_eeprom_settings mysettings;
extern std::string hostname;
extern ControllerState _controller_state;
extern uint32_t canbus_messages_failed_sent;
extern uint32_t canbus_messages_sent;
extern uint32_t canbus_messages_received;

extern void send_canbus_message(uint32_t identifier, const uint8_t *buffer,const uint8_t length);

#endif