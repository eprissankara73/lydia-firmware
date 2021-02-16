#ifndef SYSTEM_TEMPLATE_H
#define	SYSTEM_TEMPLATE_H

#include <xc.h> // include processor files - each processor file is guarded.  

extern struct bme680_dev gas_sensor;
extern uint16_t meas_period;    
extern struct bme680_field_data data;

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */
    
void ConfigureBME680(void);    
void BME680_delay_ms(uint32_t period);
int8_t BME680_spi_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
int8_t BME680_spi_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);

float ReadBoardTemperature(uint8_t location);
uint16_t ReadBatteryVoltage (void);
void Heat_off(void);
void Heat_on(void);
void Cool_off(void);
void Cool_on(void);
void Fan_off(void);
void Fan_on(void);
void Application_Initialize(void);
uint8_t Temperature_average(float current_read_temperature);
void ReadParametersFromFRAM(void);
void SetScheduleForNow(void);
void OnEntryScheduleNone(void);
void OnExitScheduleNone(void);
void DuringScheduleNone(void);
void onEntryScheduleHome(void);
void OnExitScheduleHome(void);
void DuringScheduleHome(void);
void OnEntryScheduleDREvent(void);
void OnExitScheduleDREvent(void);
void DuringScheduleDREvent(void);
void OnEntryScheduleAway(void);
void OnExitScheduleAway(void);
void DuringScheduleAway(void);
int FindPreviousSchedule(int start_idx);
int32_t ComputeDurationNextDaySchedule(int32_t partial_mins, int start_idx);
int CountSchedulesInWday(int start_idx);
void ExecuteSchedule(void);
void PopulateDebugBuffers(uint8_t * t_buf, uint8_t* r_buf, uint8_t * s_buf, int buf_len, int transmit_buf_len);
uint8_t LoadTemperatureBufferForBLE(uint8_t * data, int data_len, uint8_t start_idx);
uint8_t LoadRuntimeBufferForBLE(uint8_t * data, int data_len, uint8_t start_idx);
uint8_t LoadSetpointBufferForBLE(uint8_t * data, int data_len, uint8_t start_idx);
void ProcessUserParameterSettingsData(uint8_t* data_arr, int start_idx);
uint8_t ProcessSchedules(uint8_t * data, int start_idx);
void ResetFRAM(void);
void CheckRTCCInitialized(void);
uint8_t findSubstring(char *str, char *substr);
uint8_t Temperature_average_with_kalman(float current_read_temperature);
float Temperature_estimate_kalman(float current_read_temperature);
int computeHeatCoolSetpoints(int buttonTypePress, uint8_t change_by);

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* SYSTEM_TEMPLATE_H */

