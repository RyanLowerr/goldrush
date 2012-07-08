#define F_CPU 16000000UL

/* Broadcast ID */
#define AX_BROADCAST_ID             254

/* Control Table: EEPROM Area */
#define AX_MODEL_NUMBER_L           0
#define AX_MODEL_NUMBER_H           1
#define AX_VERSION                  2
#define AX_ID                       3
#define AX_BAUD_RATE                4
#define AX_RETURN_DELAY_TIME        5
#define AX_CW_ANGLE_LIMIT_L         6
#define AX_CW_ANGLE_LIMIT_H         7
#define AX_CCW_ANGLE_LIMIT_L        8
#define AX_CCW_ANGLE_LIMIT_H        9
#define AX_LIMIT_TEMPERATURE        11
#define AX_LOW_LIMIT_VOLTAGE        12
#define AX_HIGH_LIMIT_VOLTAGE       13
#define AX_MAX_TORQUE_L             14
#define AX_MAX_TORQUE_H             15
#define AX_STATUS_RETURN_LEVEL      16
#define AX_ALARM_LED                17
#define AX_ALARM_SHUTDOWN           18
#define AX_DOWN_CALIBRATION_L       20
#define AX_DOWN_CALIBRATION_H       21
#define AX_UP_CALIBRATION_L         22
#define AX_UP_CALIBRATION_H         23

/* Control Table: RAM Area */
#define AX_TORQUE_ENABLE            24
#define AX_LED                      25
#define AX_CW_COMPLIANCE_MARGIN     26
#define AX_CCW_COMPLIANCE_MARGIN    27
#define AX_CW_COMPLIANCE_SLOPE      28
#define AX_CCW_COMPLIANCE_SLOPE     29
#define AX_GOAL_POSITION_L          30
#define AX_GOAL_POSITION_H          31
#define AX_GOAL_SPEED_L             32
#define AX_GOAL_SPEED_H             33
#define AX_TORQUE_LIMIT_L           34
#define AX_TORQUE_LIMIT_H           35
#define AX_PRESENT_POSITION_L       36
#define AX_PRESENT_POSITION_H       37
#define AX_PRESENT_SPEED_L          38
#define AX_PRESENT_SPEED_H          39
#define AX_PRESENT_LOAD_L           40
#define AX_PRESENT_LOAD_H           41
#define AX_PRESENT_VOLTAGE          42
#define AX_PRESENT_TEMPERATURE      43
#define AX_REGISTERED_INSTRUCTION   44
#define AX_MOVING                   46
#define AX_LOCK                     47
#define AX_PUNCH_L                  48
#define AX_PUNCH_H                  49

/* Status Return Levels */
#define AX_RETURN_NONE              0
#define AX_RETURN_READ              1
#define AX_RETURN_ALL               2

/* Instructions */
#define AX_INST_PING                1
#define AX_INST_READ_DATA           2
#define AX_INST_WRITE_DATA          3
#define AX_INST_REG_WRITE           4
#define AX_INST_ACTION              5
#define AX_INST_RESET               6
#define AX_INST_SYNC_WRITE          131

void ax12_init(long baud);
void ax12_tx(unsigned char data);
unsigned char ax12_rx(void);
void ax12_set_tx(void);
void ax12_set_rx(void);
void ax12_write(int id, int reg, int data);
void ax12_write2(int id, int reg, int data);
int ax12_read(int id, int reg, int length);
void ax12_reg_write(int id, int reg, int data);
void ax12_reg_write2(int id, int reg, int data);
void ax12_action(int id);
void ax12_ping(int id);
void ax12_reset(int id);
void ax12_sync_write(int reg, int L, int n, int value_list[]);
