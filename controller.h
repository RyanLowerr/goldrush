#define F_CPU 16000000UL

/* Instructions */
#define INST_READ					1
#define INST_WRITE				    2
#define INST_RESET				    3

/* Parameters */
#define PARAM_TRAVEL_X				0
#define PARAM_TRAVEL_Y				1
#define PARAM_TRAVEL_Z				2
#define PARAM_GAIT_MODE				3
#define PARAM_TURRET_PAN			4
#define PARAM_TURRET_TILT			5
#define PARAM_TURRET_MODE			6
#define PARAM_GUN_STATUS			7
#define PARAM_GUN_MODE				8
#define PARAM_POSE				    9
#define PARAM_INTERPOLATION_TIME	10

#define NUMBER_PARAMS				11
#define BUFFER_SIZE 				128

typedef struct{
	int paramaters[NUMBER_PARAMS];
	int temp_params[NUMBER_PARAMS];
	char buffer[BUFFER_SIZE];
	int buffer_start;
	int buffer_end;
} MechController;

MechController controller;

void controller_param_init(void);
void controller_init(long baud);
void controller_tx(unsigned char data);
unsigned char controller_rx(void);
void controller_buffer_write(char x);
int controller_buffer_read(void);
int controller_buffer_data_avaliable(void);
void controller_buffer_flush(void);
void controller_param_set(int id, int length, int params[]);
void controller_param_get(int id, int length, int params[]);
void controller_read(void);

