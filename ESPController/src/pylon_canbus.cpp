/*
 ____  ____  _  _  ____  __  __  ___
(  _ \(_  _)( \/ )(  _ \(  \/  )/ __)
 )(_) )_)(_  \  /  ) _ < )    ( \__ \
(____/(____) (__) (____/(_/\/\_)(___/

  (c) 2022 Stuart Pittaway

This code communicates emulates a PYLON TECH BATTERY using CANBUS @ 500kbps and 11 bit addresses.

*/

#define USE_ESP_IDF_LOG 1
static constexpr const char *const TAG = "diybms-pylon";

#include "pylon_canbus.h"

// 0x351 – Battery voltage + current limits
void pylon_message_351()
{
  struct data351
  {
    uint16_t battery_charge_voltage;
    // positive number
    int16_t battery_charge_current_limit;
    // negative number
    int16_t battery_discharge_current_limit;
    uint16_t battery_discharge_voltage;
  };

  data351 data;

  // If we pass ZERO's to SOFAR inverter it appears to ignore them
  // so send 0.1V and 0.1Amps instead to indicate "stop"
  data.battery_discharge_voltage = mysettings.dischargevolt;

  uint16_t default_charge_voltage = 1;         // 0.1V
  int16_t default_charge_current_limit = 1;    // 0.1A
  int16_t default_discharge_current_limit = 1; // 0.1A

  if (mysettings.canbusinverter == CanBusInverter::INVERTER_DEYE)
  {
    // FOR DEYE INVERTERS APPLY DIFFERENT LOGIC TO PREVENT "W31" ERRORS
    // ISSUE #216
    default_charge_voltage = rules.lowestBankVoltage / 100;
    default_charge_current_limit = 0;
    default_discharge_current_limit = 0;
  }

  //  Defaults (tell inverter to do nothing/stop charge/discharge)
  data.battery_charge_voltage = default_charge_voltage;
  data.battery_charge_current_limit = default_charge_current_limit;
  data.battery_discharge_current_limit = default_discharge_current_limit;

  if (rules.IsChargeAllowed(&mysettings))
  {
    if (rules.numberOfBalancingModules > 0 && mysettings.stopchargebalance == true)
    {
      // Balancing is active, so stop charging (do nothing here)
    }
    else
    {
      // Default - normal behaviour (apply charging voltage and current)
      data.battery_charge_voltage = rules.DynamicChargeVoltage();
      data.battery_charge_current_limit = rules.DynamicChargeCurrent();
    }
  }

  if (rules.IsDischargeAllowed(&mysettings))
  {
    // Set discharge current limits in normal operation
    data.battery_discharge_current_limit = mysettings.dischargecurrent;
  }

  send_canbus_message(0x351, (uint8_t *)&data, sizeof(data351));
}
// 0x355 – 1A 00 64 00 – State of Health (SOH) / State of Charge (SOC)
void pylon_message_355()
{
  if (_controller_state != ControllerState::Running)
    return;

  struct data355
  {
    uint16_t stateofchargevalue;
    uint16_t stateofhealthvalue;
  };

  // Only send CANBUS message if we have a current monitor enabled & valid
  if (mysettings.currentMonitoringEnabled && currentMonitor.validReadings && (mysettings.currentMonitoringDevice == CurrentMonitorDevice::DIYBMS_CURRENT_MON_MODBUS || mysettings.currentMonitoringDevice == CurrentMonitorDevice::DIYBMS_CURRENT_MON_INTERNAL))
  {
    data355 data;
    // 0 SOC value un16 1 %
    data.stateofchargevalue = rules.StateOfChargeWithRulesApplied(&mysettings, currentMonitor.stateofcharge);

    //  2 SOH value un16 1 %
    // TODO: Need to determine this based on age of battery/cycles etc.
    data.stateofhealthvalue = 100;

    send_canbus_message(0x355, (uint8_t *)&data, sizeof(data355));
  }
}

// 0x359 – 00 00 00 00 0A 50 4E – Protection & Alarm flags
void pylon_message_359()
{
  struct data359
  {
    // Protection - Table 1
    uint8_t byte0;
    // Protection - Table 2
    uint8_t byte1;
    // Warnings - Table
    uint8_t byte2;
    // Warnings - Table 4
    uint8_t byte3;
    // Quantity of banks in parallel
    uint8_t byte4;
    uint8_t byte5;
    uint8_t byte6;
    // Online address of banks in parallel - Table 5
    uint8_t byte7;
  };

  data359 data;

  memset(&data, 0, sizeof(data359));

  if (_controller_state == ControllerState::Running)
  {
    // bit 0 = unused
    //(bit 1) Battery high voltage alarm
    data.byte0 |= ((rules.ruleOutcome(Rule::BankOverVoltage) || rules.ruleOutcome(Rule::CurrentMonitorOverVoltage)) ? B00000010 : 0);

    //(bit 2) Battery low voltage alarm
    data.byte0 |= ((rules.ruleOutcome(Rule::BankUnderVoltage) || rules.ruleOutcome(Rule::CurrentMonitorUnderVoltage)) ? B00000100 : 0);

    //(bit 3) Battery high temperature alarm
    if (rules.moduleHasExternalTempSensor)
    {
      data.byte0 |= (rules.ruleOutcome(Rule::ModuleOverTemperatureExternal) ? B00001000 : 0);
    }
    // (bit 4) Battery low temperature alarm
    if (rules.moduleHasExternalTempSensor)
    {
      data.byte0 |= (rules.ruleOutcome(Rule::ModuleUnderTemperatureExternal) ? B00010000 : 0);
    }
    // bit 5 = unused
    // bit 6 = unused
    // bit 7 = Discharge over current

    // Byte2, Warnings - Table 3
    data.byte2 = 0;

    // WARNING:Battery high voltage
    if (rules.highestBankVoltage / 100 > mysettings.chargevolt)
    {
      data.byte2 |= B00000010;
    }

    // WARNING:Battery low voltage
    // dischargevolt=490, lowestbankvoltage=48992 (scale down 100)
    if (rules.lowestBankVoltage / 100 < mysettings.dischargevolt)
    {
      data.byte2 |= B00000100;
    }

    // WARNING: Battery high temperature
    if (rules.moduleHasExternalTempSensor && rules.highestExternalTemp > mysettings.chargetemphigh)
    {
      data.byte2 |= B00001000;
    }

    // WARNING: Battery low temperature
    if (rules.moduleHasExternalTempSensor && rules.lowestExternalTemp < mysettings.chargetemplow)
    {
      data.byte2 |= B00010000;
    }
  }

  // byte3,table4, Bit 3 = Internal communication failure
  data.byte3 |= ((rules.ruleOutcome(Rule::BMSError) || rules.ruleOutcome(Rule::EmergencyStop)) ? B00001000 : 0);
  data.byte3 |= ((_controller_state != ControllerState::Running) ? B00001000 : 0);

  if (mysettings.currentMonitoringEnabled && currentMonitor.validReadings)
  {
    // Pylon can have multiple battery each of 74Ah capacity, so emulate this based on total Ah capacity
    // this drives the inverter to assume certain charge/discharge parameters based on number of battery banks installed
    // Set inverter to use "Pylontech US3000C 3.5kWh" in its settings (these are 74Ah each)
    data.byte4 = max((uint8_t)1, (uint8_t)round(mysettings.nominalbatcap / 74.0));
  }
  else
  {
    // Default 1 battery
    data.byte4 = 1;
  }

  data.byte5 = 0x50; // P
  data.byte6 = 0x4e; // N

  send_canbus_message(0x359, (uint8_t *)&data, sizeof(data359));
}

// 0x35C – C0 00 – Battery charge request flags
void pylon_message_35c()
{
  struct data35c
  {
    uint8_t byte0;
  };

  data35c data;

  // bit 0/1/2/3 unused
  // bit 4 Force charge 2
  // bit 5 Force charge 1
  // bit 6 Discharge enable
  // bit 7 Charge enable
  data.byte0 = 0;

  if (rules.IsChargeAllowed(&mysettings))
  {
    data.byte0 = data.byte0 | B10000000;
  }

  if (rules.IsDischargeAllowed(&mysettings))
  {
    data.byte0 = data.byte0 | B01000000;
  }

  send_canbus_message(0x35c, (uint8_t *)&data, sizeof(data35c));
}

// 0x35E – 50 59 4C 4F 4E 20 20 20 – Manufacturer name ("PYLON ")
void pylon_message_35e()
{
  // Send 8 byte "magic string" PYLON (with 3 trailing spaces)
  // const char pylon[] = "\x50\x59\x4c\x4f\x4e\x20\x20\x20";
  uint8_t pylon[] = {0x50, 0x59, 0x4c, 0x4f, 0x4e, 0x20, 0x20, 0x20};
  send_canbus_message(0x35e, (uint8_t *)&pylon, sizeof(pylon) - 1);
}

// Battery voltage - 0x356 – 4e 13 02 03 04 05 – Voltage / Current / Temp
void pylon_message_356()
{
  struct data356
  {
    int16_t voltage;
    int16_t current;
    int16_t temperature;
  };

  data356 data;

  // If current shunt is installed, use the voltage from that as it should be more accurate
  if (mysettings.currentMonitoringEnabled && currentMonitor.validReadings)
  {
    data.voltage = currentMonitor.modbus.voltage * 100.0;
    data.current = currentMonitor.modbus.current * 10;
  }
  else
  {
    // Use highest bank voltage calculated by controller and modules
    data.voltage = rules.highestBankVoltage / 10;
    data.current = 0;
  }

  // Temperature 0.1 C using external temperature sensor
  if (rules.moduleHasExternalTempSensor)
  {
    data.temperature = (int16_t)rules.highestExternalTemp * (int16_t)10;
  }
  else
  {
    // No external temp sensors
    data.temperature = 0;
  }

  send_canbus_message(0x356, (uint8_t *)&data, sizeof(data356));
}





///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////// PROTOCOL PYLONTECH HIGH VOLTAGE  /////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////

///////                                                                         ///////
///////   CAN MESSAGE OF REQUEST HOST MESSAGE 0x4200: 02 00 00 00 00 00 00 00   ///////
///////                                                                         ///////

/* message info hardware and software */
void pylonHV_message_0x7310(bool extend){
  uint8_t data[8];
  data[0]=0x01; //0:null, 1:ver.A, 2:ver.B, other:reserved
  data[1]=0x00; //reserve
  data[2]=0x10; //hardware version V
  data[3]=0x02; //hardware version R
  data[4]=0x04; //software version v major
  data[5]=0x05; //software version v minor
  data[6]=0x34; //software version  
  data[7]=0x0C; //software version
  uint32_t address=0x731;
  if(extend)address=0x7310 + DEFAULT_DEVICE_ID_ADDRESS;
  send_canbus_message(address, data, 8);
}

/* message all cells, number of modules, cells for module, nominal voltage, capacity */
void pylonHV_message_0x7320(bool extend){
  uint8_t data[8];
  data[0]=mysettings.totalNumberOfSeriesModules; //number of cells of system
  data[1]=0x00;
  data[2]=mysettings.totalNumberOfBanks;//number of module
  data[3]=mysettings.totalNumberOfSeriesModules / mysettings.totalNumberOfBanks; //number cells for module
  uint16_t nominal_voltage= mysettings.cellmaxmv * mysettings.totalNumberOfSeriesModules / 1000;
  data[4]=nominal_voltage & 0xFF;
  data[5]=nominal_voltage >> 8;
  uint16_t capacity=mysettings.nominalbatcap; //Ah
  data[6]=capacity & 0xFF;
  data[7]=capacity >>8;
  uint32_t address=0x732;
  if(extend)address=0x7320 + DEFAULT_DEVICE_ID_ADDRESS;
  send_canbus_message(address, data, 8);
}

/* message 1/2 name of Maker*/
void pylonHV_message_0x7330(bool extend){
  uint8_t data[8];
  data[0]='P';
  data[1]='Y';
  data[2]='L';
  data[3]='O';
  data[4]='N';
  data[5]='T';
  data[6]='E';
  data[7]='C';
  uint32_t address=0x733;
  if(extend)address=0x7330 + DEFAULT_DEVICE_ID_ADDRESS;
  send_canbus_message(address, data, 8);
}

/* message 2/2 name of Maker*/
void pylonHV_message_0x7340(bool extend){
  uint8_t data[8];
  data[0]='H';
  data[1]=0x00;
  data[2]=0x00;
  data[3]=0x00;
  data[4]=0x00;
  data[5]=0x00;
  data[6]=0x00;
  data[7]=0x00;
  uint32_t address=0x734;
  if(extend)address=0x7340 + DEFAULT_DEVICE_ID_ADDRESS;
  send_canbus_message(address, data, 8);
}

void pylonHV_send_message_info(bool extend){
  if (_controller_state != ControllerState::Running) return;
  pylonHV_message_0x7320(extend);
  delay(10);
  pylonHV_message_0x7330(extend);
  delay(10);
  pylonHV_message_0x7310(extend);
  delay(10);
  pylonHV_message_0x7340(extend);
}


///////                                                                         ///////
///////   CAN MESSAGE OF REQUEST HOST MESSAGE 0x4200: 00 00 00 00 00 00 00 00   ///////
///////                                                                         ///////

/* Voltage, current, temperature, soc, soh of pack battery system*/
void pylonHV_message_0x4210(bool extend){
  uint8_t data[8];
  uint16_t voltage=0;  //resolution 0.1V
  int16_t current=30000; // offset= 30000 =0.0A  scale 0.1A ;29985= -1.5A 30028=2.8A
  int16_t temperature=1000; // offset 1000  =0.0ºC     -2.0ºC = 980 
  uint16_t stateofchargevalue=0;
  uint16_t stateofhealthvalue=0;

  if (_controller_state != ControllerState::Running) return;


  // Only send CANBUS message if we have a current monitor enabled & valid
  if (mysettings.currentMonitoringEnabled && currentMonitor.validReadings && (mysettings.currentMonitoringDevice == CurrentMonitorDevice::DIYBMS_CURRENT_MON_MODBUS || mysettings.currentMonitoringDevice == CurrentMonitorDevice::DIYBMS_CURRENT_MON_INTERNAL)){
    
     // 0 SOC value un16 1 %
    stateofchargevalue = rules.StateOfChargeWithRulesApplied(&mysettings, currentMonitor.stateofcharge);

    //  2 SOH value un16 1 %
    // TODO: Need to determine this based on age of battery/cycles etc.
    stateofhealthvalue = 100;
  }

  if (mysettings.currentMonitoringEnabled && currentMonitor.validReadings){
    voltage = currentMonitor.modbus.voltage * 10;
    current += currentMonitor.modbus.current * 10;
  }else{
    // Use highest bank voltage calculated by controller and modules
    voltage = rules.highestBankVoltage / 100;
    current += 0;
  }

  // Temperature 0.1 C using external temperature sensor
  if (rules.moduleHasExternalTempSensor){
    temperature += rules.highestExternalTemp;
  }else{
    // No external temp sensors
    temperature += 250; //default 25.0ºC
  }

  data[0]=voltage & 0xFF;
  data[1]=voltage >> 8;
  data[2]=current & 0xFF;
  data[3]=current >> 8;
  data[4]=temperature & 0xFF;
  data[5]=temperature >> 8;
  data[6]=stateofchargevalue & 0xFF;
  data[7]=stateofhealthvalue & 0xFF;

  uint32_t address=0x421;
  if(extend)address=0x4210 + DEFAULT_DEVICE_ID_ADDRESS;
  send_canbus_message(address, data, 8);
  ESP_LOGI("PYLON_HV", "Address:%04x::%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x", 
                  address, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7] );
}

/* Charge voltage, discharge voltage, charge current, discharge current */
void pylonHV_message_0x4220(bool extend){
  uint8_t data[8];
  uint16_t charge_voltage=0;  //resolution 0.1v
  uint16_t discharge_voltage=0;
  int16_t charge_current=30000;  //offset 3000A scale 0.1A 
  int16_t discharge_current=30000;

  charge_voltage = mysettings.chargevolt;
  discharge_voltage = mysettings.dischargevolt;

  if(rules.IsChargeAllowed(&mysettings)){
    
    if(rules.numberOfBalancingModules > 0 && mysettings.stopchargebalance == true){
      // Balancing is active, so stop charging (do nothing here)
    }else{
      // Default - normal behaviour (apply charging voltage and current)
      charge_voltage = rules.DynamicChargeVoltage();
      charge_current += rules.DynamicChargeCurrent();
    }

  }

  if(rules.IsDischargeAllowed(&mysettings)){
    // Set discharge current limits in normal operation
    discharge_current -= mysettings.dischargecurrent; //BOTANETA test signed
  }else{
    // default 0.0A
  }

  data[0]=charge_voltage & 0xFF;
  data[1]=charge_voltage >> 8;
  data[2]=discharge_voltage & 0xFF;
  data[3]=discharge_voltage >> 8;
  data[4]=charge_current & 0xFF;
  data[5]=charge_current >> 8;
  data[6]=discharge_current & 0xFF;
  data[7]=discharge_current >> 8;
  
  uint32_t address=0x422;
  if(extend)address=0x4220 + DEFAULT_DEVICE_ID_ADDRESS;
  send_canbus_message(address, data, 8);
  ESP_LOGI("PYLON_HV", "Address:%04x::%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x", 
                  address, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7] );
}


/* Voltage and id from cell maximun and minimun*/
void pylonHV_message_0x4230(bool extend){
  uint8_t data[8];
  uint8_t id_cell_vmax=rules.address_HighestCellVoltage;
  uint16_t cell_vmax=rules.highestCellVoltage;
  uint8_t id_cell_vmin=rules.address_LowestCellVoltage;
  uint16_t cell_vmin=rules.lowestCellVoltage;
  
  data[0]=cell_vmax & 0xFF;
  data[1]=cell_vmax >> 8;
  data[2]=cell_vmin & 0xFF;
  data[3]=cell_vmin >> 8;
  data[4]=id_cell_vmax;
  data[5]=0x00;
  data[6]=id_cell_vmin;
  data[7]=0x00;

  uint32_t address=0x423;
  if(extend)address=0x4230 + DEFAULT_DEVICE_ID_ADDRESS;
  send_canbus_message(address, data, 8);
  ESP_LOGI("PYLON_HV", "Address:%04x::%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x", 
                  address, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7] );
}

/* Temperature and id from cell maximun and minimun */
void pylonHV_message_0x4240(bool extend){
  uint8_t data[8];
  uint8_t id_cell_tmax=rules.address_highestExternalTemp;
  int8_t cell_tmax=rules.highestExternalTemp;
  uint8_t id_cell_tmin=rules.address_lowestExternalTemp;
  int8_t cell_tmin=rules.lowestExternalTemp;
  uint8_t series=TotalNumberOfCells();

  for(uint16_t idcell=0; idcell < series ; idcell++){
    
    if(cmi[idcell].internalTemp > cell_tmax ){
      cell_tmax = cmi[idcell].internalTemp;
      id_cell_tmax=idcell;
    }

    if(cmi[idcell].internalTemp < cell_tmin){
      cell_tmin = cmi[idcell].internalTemp;
      id_cell_tmin=idcell;
    }
  }

  //offset temp 100ºC scale 0.1ºC
  uint16_t tmax= 1000 + cell_tmax*10;
  uint16_t tmin= 1000 + cell_tmin*10;
  data[0]=tmax & 0xFF;
  data[1]=tmax >> 8;
  data[2]=tmin & 0xFF;
  data[3]=tmin >> 8;
  data[4]=id_cell_tmax;
  data[5]=0x00;
  data[6]=id_cell_tmin;
  data[7]=0x00;

  uint32_t address=0x424;
  if(extend)address=0x4240 + DEFAULT_DEVICE_ID_ADDRESS;
  send_canbus_message(address, data, 8);ESP_LOGI("PYLON_HV", "Address:%04x::%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x", 
                  address, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7] );
}

/* Status, nº_cycles, error, alarm, protection*/
void pylonHV_message_0x4250(bool extend){
  uint8_t data[8];
  uint8_t status=0x00;
  //b7 reserve
  //b6 reserve
  //b5 reserve
  //b4 0:null, 1:Balance charge request
  //b3 0:null, 1:Forced charge request
  //b2..0  0:Sleep, 1:Charge, 2:discharge, 3:idle, 4..7 reserve
  
  if(currentMonitor.modbus.current > 0.0)status=0x01;
  if(rules.getChargingMode() == ChargingMode::floating)status=0x03;
  if(currentMonitor.modbus.current < 0.0)status=0x02;

  uint16_t cycles=(currentMonitor.modbus.milliamphour_out / 1000) / mysettings.currentMonitoring_batterycapacity;

  uint8_t error=0x00;
  //b7 other error
  //b6 battery cell error
  //b5 relay check error,  RELAY_ERR
  //b4 input transposition error, RV_ERR 
  //b3 input over voltage error, DCOV_ERR
  //b2 internal comunication error, IN_COMM_ERR
  //b1 temperature sensor error, TMPR_ERR
  //b0 voltage sensor error, VOLT_ERR
  error = rules.ruleOutcome(Rule::EmergencyStop)? error | 0b10000000 : error;
  error = rules.ruleOutcome(Rule::BMSError)?      error | 0b00000100 : error;
  
  uint16_t alarm=0x0000;
  //b15 Reseerve
  //b14 Reserve
  //b13 Fan Alarm
  //b12 Terminal High Temperature Alarm
  //b11 MHV: Module High Voltage Alarm
  //b10 MLV: Module Low Voltage Alarm
  //b9 DOCA: Discharge Over Current Alarm
  //b8 COCA: Charge Over Current Alarm
  //b7 DHT:  Discharge Cell High Temperature Alarm
  //b6 DLT:  Discharge Cell Low Temperature Alarm
  //b5 CHT:  Charge Cell High Temperature Alarm
  //b4 CLT:  Charge Cell Low Temperature Alarm
  //b3 PHV:  Charge system High Voltage Alarm
  //b2 PLV:  Discharge system Low Voltage Alarm
  //b1 BHV:  Single Cell High Voltage Alarm
  //b0 BLV:  Single Cell Low Voltage Alarm

  if(rules.ruleOutcome(Rule::ModuleOverTemperatureExternal) ||
     rules.ruleOutcome(Rule::ModuleOverTemperatureInternal))alarm |= 0b0000000010100000; //b7 b5
  if(rules.ruleOutcome(Rule::ModuleUnderTemperatureExternal) ||
    rules.ruleOutcome(Rule::ModuleUnderTemperatureInternal))alarm |= 0b0000000001010000; //b6 b4   
  if(rules.ruleOutcome(Rule::CurrentMonitorOverCurrentAmps))alarm |= 0b0000001100000000; //b9 b8
  if(rules.ruleOutcome(Rule::BankOverVoltage))              alarm |= 0b0000100000001000; //b11 b3
  if(rules.ruleOutcome(Rule::BankUnderVoltage))             alarm |= 0b0000010000000100; //b10 b2
  if(rules.ruleOutcome(Rule::ModuleOverVoltage) || rules.highestCellVoltage > mysettings.cellmaxmv) alarm |= 0b0000000000000010; //b1
  if(rules.ruleOutcome(Rule::ModuleUnderVoltage) || rules.lowestCellVoltage < mysettings.cellminmv) alarm |= 0b0000000000000001; //b0

  uint16_t protection=0x0000;
  //b15 Reserve
  //b14 Reverse
  //b13 Reverse
  //b12 BUV2: Battery cell secondary undervoltage protection
  //b11 MOV: Module Over Voltage Protect
  //b10 MUV: Module Under Voltage Protect
  //b9 DOC:  Discharge Over Current Protect
  //b8 COC:  Charge Over Current Protect
  //b7 DOT:  Discharge Cell Over Temperature Protect
  //b6 DUT:  Discharge Cell Under Temperature Protect
  //b5 COT:  Charge Cell Over Temperature Protect
  //b4 CUT:  Charge Cell Under Temperature Protect
  //b3 POV:  Charge system Over Voltage Protect
  //b2 PUV:  Discharge system Under Voltage Protect
  //b1 BOV: Single Cell Over Voltage Protect
  //b0 BUV: Single Cell Under Voltage Protect

  if(rules.ruleOutcome(Rule::ModuleUnderVoltage)) protection |= 0b0001000000000000; //b12
  if(rules.ruleOutcome(Rule::BankOverVoltage))    protection |= 0b0000100000001000; //b11 b3
  if(rules.ruleOutcome(Rule::BankUnderVoltage))   protection |= 0b0000010000000100; //b10 b2
  if(rules.ruleOutcome(Rule::CurrentMonitorOverCurrentAmps))protection |= 0b0000001100000000; //b9 b8
  if(rules.ruleOutcome(Rule::ModuleOverTemperatureExternal) ||
     rules.ruleOutcome(Rule::ModuleOverTemperatureInternal))protection |= 0b0000000010100000; //b7 b5
  if(rules.ruleOutcome(Rule::ModuleUnderTemperatureExternal) ||
    rules.ruleOutcome(Rule::ModuleUnderTemperatureInternal))protection |= 0b0000000001010000; //b6 b4
   if(rules.ruleOutcome(Rule::ModuleOverVoltage) || rules.highestCellVoltage > mysettings.cellmaxmv) protection |= 0b0000000000000010; //b1
  if(rules.ruleOutcome(Rule::ModuleUnderVoltage) || rules.lowestCellVoltage < mysettings.cellminmv) protection |= 0b0000000000000001; //b0

  data[0]=status;
  data[1]=cycles & 0xFF;
  data[2]=cycles >> 8;
  data[3]=error;
  data[4]=alarm & 0xFF;
  data[5]=alarm >> 8;
  data[6]=protection & 0xFF;
  data[7]=protection >> 8;

  uint32_t address=0x425;
  if(extend)address=0x4250 + DEFAULT_DEVICE_ID_ADDRESS;
  send_canbus_message(address, data, 8);
  ESP_LOGI("PYLON_HV", "Address:%04x::%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x", 
                  address, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7] );
}


/* Module-pylontech max/min voltage, id_max/id_min */
void pylonHV_message_0x4260(bool extend){
  uint8_t data[8];
  uint16_t voltage=0xC3B4; //resolution 1mV, simulation 50100mV

  data[0]=voltage & 0xff;
  data[1]=voltage >> 8;
  data[2]=voltage & 0xff;
  data[3]=voltage >> 8;
  data[4]=0x01;
  data[5]=0x00;
  data[6]=0x01;
  data[7]=0x00;

  uint32_t address=0x426;
  if(extend)address=0x4260 + DEFAULT_DEVICE_ID_ADDRESS;
  send_canbus_message(address, data, 8);
  ESP_LOGI("PYLON_HV", "Address:%04x::%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x", 
                  address, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7] );
}

/* Module max/min temperature id_max/id_min */
void pylonHV_message_0x4270(bool extend){
  uint8_t data[8];
  uint8_t id_cell_tmax=rules.address_highestExternalTemp;
  int8_t cell_tmax=rules.highestExternalTemp;
  uint8_t id_cell_tmin=rules.address_lowestExternalTemp;
  int8_t cell_tmin=rules.lowestExternalTemp;
  uint8_t series=TotalNumberOfCells();
  uint8_t n_banks=mysettings.totalNumberOfBanks;

  for(uint16_t idcell=0; idcell < series ; idcell++){
    
    if(cmi[idcell].internalTemp > cell_tmax){
      cell_tmax=cmi[idcell].internalTemp;
      id_cell_tmax=idcell;
    }  

    if(cmi[idcell].internalTemp < cell_tmin){
      cell_tmin=cmi[idcell].internalTemp;
      id_cell_tmin=idcell;
    }
  }

  //offset temp 100ºC scale 0.1ºC
  uint16_t tmax= 1000 + cell_tmax*10;
  uint16_t tmin= 1000 + cell_tmin*10;

  data[0]=tmax & 0xFF;
  data[1]=tmax >> 8;
  data[2]=tmin & 0xFF;
  data[3]=tmin >> 8;
  data[4]=id_cell_tmax/(series/n_banks);
  data[5]=0x00;
  data[6]=id_cell_tmin/(series/n_banks);
  data[7]=0x00;

  uint32_t address=0x427;
  if(extend)address=0x4270 + DEFAULT_DEVICE_ID_ADDRESS;
  send_canbus_message(address, data, 8);
  ESP_LOGI("PYLON_HV", "Address:%04x::%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x", 
                  address, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7] );
}

/* Charge Discharge forbiden mark */
void pylonHV_message_0x4280(bool extend){
  uint8_t data[8]{0};
  uint8_t no_charge=0xAA;  // charge != 0xAA;
  uint8_t no_discharge=0xAA;

  data[0]=rules.IsChargeAllowed(&mysettings)? 0x00 : no_charge; 
  data[1]=rules.IsDischargeAllowed(&mysettings)? 0x00 : no_discharge;
  uint32_t address=0x428;
  if(extend)address=0x4280 + DEFAULT_DEVICE_ID_ADDRESS;
  send_canbus_message(address, data, 8);
  ESP_LOGI("PYLON_HV", "Address:%04x::%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x", 
                  address, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7] );
}

/* System error list */
void pylonHV_message_0x4290(bool extend){
  uint8_t data[8]{0};
  uint8_t error=0;
  //b5..b7 reserve
  //b4  chip error
  //b3  self-test error
  //b2  internal bus error
  //b1  BMIC error
  //b0  shutdown circuit error

  if(rules.ruleOutcome(Rule::BMSError))error |= 0b00010100;
  data[0]=error;

  uint32_t address=0x429;
  if(extend)address=0x4290 + DEFAULT_DEVICE_ID_ADDRESS;
  send_canbus_message(address, data, 8);
  ESP_LOGI("PYLON_HV", "Address:%04x::%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x", 
                  address, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7] );
}

/* Terminal max/min temperature id_terminal*/
void pylonHV_message_0x42A0(bool extend){
  struct data42A0{
    uint16_t terminal_max_temp=1350; //offset 100.0ºC resolution 0.1ºC, default 35.0ºC
    uint16_t terminal_min_temp=1350;
    uint16_t id_terminal_max_temp=0;
    uint16_t id_terminal_min_temp=0;
  };
  data42A0 data42a;
  uint32_t address=0x42A;
  if(extend)address=0x42A0 + DEFAULT_DEVICE_ID_ADDRESS;
  send_canbus_message(address,(uint8_t *)&data42a, sizeof(data42A0));
  uint8_t data[8];
  memcpy(&data, &data42a, 8);
  ESP_LOGI("PYLON_HV", "Address:%04x::%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x", 
                  address, ((uint8_t *)&data42a)[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7] );
}

void pylonHV_send_message_status(bool extend){
  if (_controller_state != ControllerState::Running)return;
  pylonHV_message_0x4210(extend);
  delay(10);
  pylonHV_message_0x4220(extend);
  delay(10);
  pylonHV_message_0x4230(extend);
  delay(10);
  pylonHV_message_0x4240(extend);
  delay(10);
  pylonHV_message_0x4250(extend);
  delay(10);
  pylonHV_message_0x4260(extend);
  delay(10);
  pylonHV_message_0x4270(extend);
  delay(10);
  pylonHV_message_0x4280(extend);
  delay(10);
  pylonHV_message_0x4290(extend);
  delay(10);
  pylonHV_message_0x42A0(extend);
}
