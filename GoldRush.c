#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>
#include "ax12.h"

#define set_bit(reg,bit) reg |= (1<<bit)
#define clear_bit(reg,bit) reg &= ~(1<<bit)

/* ------------------------------------------------------------------------------------------ */
/* Data Structures                                                                            */
/* ------------------------------------------------------------------------------------------ */

typedef struct{
  float x;
  float y;
  float z;
} pos;

typedef struct{
  int coxa;
  int femur;
  int tibia;
} servo;

typedef struct{
  float coxa;
  float femur;
  float tibia;
} angle;

typedef struct{
  pos quadrant;       // Quadrant the leg resides in.

  servo id;           // Servo IDs of the leg joints.
  servo center;       // Servo centers of the leg joints.
  servo direction;    // Servio direction of the leg joints.
  
  pos foot_endpoint;  // Endpoint of the foot measured from the robots origin to tip of the foot.
  pos foot_neutral;   // Neutral foot endpoint.
  pos foot_gait;      // Adjustments to the foot enpoint from the gait.

  pos body_endpoint;  // Endpoint of the body measured from the robots origin Coxa joint.
  pos body_neutral;   // Neutral body endpoint.
  pos body_gait;      // Adjustments to the body enpoint from the gait.
  pos body_rotation;  // Body rotation.
  pos body_gait_rot;  // Adjustments to the body rotation from the gait.

  angle ik;           //
} leg;

typedef struct{
  int lift_height;    // Height to lift legs in milimeters.
  int push_steps;     // Number of times a leg pushes or moves the body forward in the walk cycle.
  int steps;          // Number of steps in a walk cycle.
  int step;           // Step counter.
  int leg_order[4];   //
} gaits;

/* ------------------------------------------------------------------------------------------ */
/* Mech Definitions                                                                           */
/* ------------------------------------------------------------------------------------------ */

#define NUM_LEGS    4
#define NUM_DOF     3

// Leg position offset (mm)
#define X_OFFSET    67
#define Y_OFFSET    67
#define Z_OFFSET    0

// Leg segment dimensions (mm)
#define COXA_LENGTH    46
#define FEMUR_LENGTH   57
#define TIBIA_LENGTH   84

// Legs IDs
#define LEFT_FRONT    0
#define RIGHT_FRONT   1
#define RIGHT_REAR    2
#define LEFT_REAR     3

// Leg joint servo IDs
#define LF_COXA_ID    10
#define LF_FEMUR_ID   11
#define LF_TIBIA_ID   12
#define RF_COXA_ID    1
#define RF_FEMUR_ID   2
#define RF_TIBIA_ID   3
#define LR_COXA_ID    7
#define LR_FEMUR_ID   8
#define LR_TIBIA_ID   9
#define RR_COXA_ID    4
#define RR_FEMUR_ID   5
#define RR_TIBIA_ID   6

// Turret servo IDs
#define PAN_ID       13
#define TILT_ID      14

// Leg Joint min and max values
#define COXA_MIN     -180
#define COXA_MAX      180
#define FEMUR_MIN    -40
#define FEMUR_MAX     40
#define TIBIA_MIN    -40
#define TIBIA_MAX     40

// Turret min and max values
#define PAN_MIN     -80
#define PAN_MAX      80
#define TILT_MIN    -45
#define TILT_MAX     45

//
#define ENDPOINT_X    140
#define ENDPOINT_Y    120
#define ENDPOINT_Z    100

/* ------------------------------------------------------------------------------------------ */
/* Controller                                                                                 */
/* ------------------------------------------------------------------------------------------ */

/* Instructions */
#define INST_READ           1
#define INST_WRITE          2
#define INST_RESET          3

/* Parameters */
#define PARAM_TRAVEL_X      0
#define PARAM_TRAVEL_Y      1
#define PARAM_TRAVEL_Z      2
#define PARAM_GAIT_MODE     3
#define PARAM_TURRET_PAN    4
#define PARAM_TURRET_TILT   5
#define PARAM_TURRET_MODE   6
#define PARAM_GUN_STATUS    7
#define PARAM_GUN_MODE      8
#define PARAM_POSE          9

#define NUMBER_PARAMS       10
#define BUFFER_SIZE         128

typedef struct{
  int paramaters[NUMBER_PARAMS];
  int temp_params[NUMBER_PARAMS];
  char buffer[BUFFER_SIZE];
  int buffer_start;
  int buffer_end;
} MechController;

MechController controller;

void controller_param_init(void)
{
  controller.paramaters[PARAM_TRAVEL_X]           = 127;
  controller.paramaters[PARAM_TRAVEL_Y]           = 127;
  controller.paramaters[PARAM_TRAVEL_Z]           = 127;
  controller.paramaters[PARAM_GAIT_MODE]          = 0;
  controller.paramaters[PARAM_TURRET_PAN]         = 127;
  controller.paramaters[PARAM_TURRET_TILT]        = 127;
  controller.paramaters[PARAM_TURRET_MODE]        = 0;
  controller.paramaters[PARAM_GUN_STATUS]         = 0;
  controller.paramaters[PARAM_GUN_MODE]           = 0;
  controller.paramaters[PARAM_POSE]               = 0;
}

void controller_init(long baud)
{
  UBRR1H = ((F_CPU / 16 + baud / 2) / baud - 1) >> 8;
  UBRR1L = ((F_CPU / 16 + baud / 2) / baud - 1);
  set_bit(UCSR1B, TXEN1);
  set_bit(UCSR1B, RXEN1);
  set_bit(UCSR1B, RXCIE1);

  controller_param_init();
}

void controller_tx(unsigned char data)
{
  while (bit_is_clear(UCSR1A, UDRE1));
  UDR1 = data;
}

unsigned char controller_rx(void)
{
  while (bit_is_clear(UCSR1A, RXC1));
  return UDR1;
}

void controller_buffer_write(char x)
{
  if ((controller.buffer_end + 1) % BUFFER_SIZE != controller.buffer_start)
  {
    controller.buffer[controller.buffer_end] = x;
    controller.buffer_end = (controller.buffer_end + 1) % BUFFER_SIZE;
  }
}

int controller_buffer_read(void)
{
  char temp = controller.buffer[controller.buffer_start];
  controller.buffer_start = (controller.buffer_start + 1) % BUFFER_SIZE;
  return temp;
}

int controller_buffer_data_avaliable(void)
{
  return ((BUFFER_SIZE + controller.buffer_start - controller.buffer_end) % BUFFER_SIZE);
}

void controller_buffer_flush(void)
{
  controller.buffer_start = controller.buffer_end;
}

void controller_param_set(int id, int length, int params[])
{
  int i = 0;
  while(i < length)
  {
    controller.paramaters[params[i]] = params[i + 1];
    i += 2;
  }
}

void controller_param_get(int id, int length, int params[])
{
  int i = 0;
  int checksum = 0;
  controller_tx(0xFF);
  controller_tx(0xFF);
  controller_tx(id);
  controller_tx(INST_READ);
  controller_tx(length);
  while(i < length)
  {
    controller_tx(params[i]);
    controller_tx(controller.paramaters[params[i]]);
    checksum += params[i];
    checksum += controller.paramaters[params[i]];
  }
  controller_tx(checksum);
}

void controller_read(void)
{
  int index = -1;
  int checksum = 0;
  int id = 0;
  int length = 0;
  int instruction = 0;

  while(controller_buffer_data_avaliable() > 0)
  {
    // Searching for the first two 0xFF header bytes.
    if(index == -1)
    {
      // First header byte.
      if(controller_buffer_read() == 0xff)
      {
        // second header byte.
        if(controller_buffer_read() == 0xff)
        {
          index = 0;
          checksum = 0;

          id = controller_buffer_read();
          checksum += id;

          instruction = controller_buffer_read();
          checksum += instruction;

          length = controller_buffer_read();
          checksum += length;

        }
      }
    }
    // Reading the packet following the header.
    else
    {
      if(index == length)
      {
        if(255 - ((checksum) % 256) == controller_buffer_read())
        {
          if(instruction == INST_WRITE)
            controller_param_set(id, length, controller.temp_params);

          else if(instruction == INST_READ)
            controller_param_get(id, length, controller.temp_params);
          
          else if(instruction == INST_RESET)
            controller_param_init();

          index = -1;
        }
        else
          index = -1;
      }
      else
      {        
        controller.temp_params[index] = (unsigned char) controller_buffer_read();
        
        if(controller.temp_params[index] != 0xff){
          checksum += (int) controller.temp_params[index];
          index++;
        }
        else
          index = -1;
      }
    }
  }

  controller_buffer_flush();
}

ISR(USART1_RX_vect)
{
  char received_char = UDR1;
  controller_buffer_write(received_char);
}

/* ------------------------------------------------------------------------------------------ */
/* Leg                                                                                        */
/* ------------------------------------------------------------------------------------------ */

leg legs[NUM_LEGS];

// Calls all leg initilization functions.
void leg_init(leg *legs)
{
  legs[LEFT_FRONT].foot_neutral.x     = -ENDPOINT_X;
  legs[LEFT_FRONT].foot_neutral.y     =  ENDPOINT_Y;
  legs[LEFT_FRONT].foot_neutral.z     = -ENDPOINT_Z;

  legs[RIGHT_FRONT].foot_neutral.x    =  ENDPOINT_X;
  legs[RIGHT_FRONT].foot_neutral.y    =  ENDPOINT_Y;
  legs[RIGHT_FRONT].foot_neutral.z    = -ENDPOINT_Z;

  legs[RIGHT_REAR].foot_neutral.x     =  ENDPOINT_X;
  legs[RIGHT_REAR].foot_neutral.y     = -ENDPOINT_Y;
  legs[RIGHT_REAR].foot_neutral.z     = -ENDPOINT_Z;

  legs[LEFT_REAR].foot_neutral.x      = -ENDPOINT_X;
  legs[LEFT_REAR].foot_neutral.y      = -ENDPOINT_Y;
  legs[LEFT_REAR].foot_neutral.z      = -ENDPOINT_Z;

  legs[LEFT_FRONT].body_neutral.x     = -X_OFFSET;
  legs[LEFT_FRONT].body_neutral.y     =  Y_OFFSET;
  legs[LEFT_FRONT].body_neutral.z     = -Z_OFFSET;

  legs[RIGHT_FRONT].body_neutral.x    =  X_OFFSET;
  legs[RIGHT_FRONT].body_neutral.y    =  Y_OFFSET;
  legs[RIGHT_FRONT].body_neutral.z    = -Z_OFFSET;

  legs[RIGHT_REAR].body_neutral.x     =  X_OFFSET;
  legs[RIGHT_REAR].body_neutral.y     = -Y_OFFSET;
  legs[RIGHT_REAR].body_neutral.z     = -Z_OFFSET;

  legs[LEFT_REAR].body_neutral.x      = -X_OFFSET;
  legs[LEFT_REAR].body_neutral.y      = -Y_OFFSET;
  legs[LEFT_REAR].body_neutral.z      = -Z_OFFSET;

  legs[LEFT_FRONT].quadrant.x         = -1;
  legs[LEFT_FRONT].quadrant.y         =  1;

  legs[RIGHT_FRONT].quadrant.x        =  1;
  legs[RIGHT_FRONT].quadrant.y        =  1;

  legs[RIGHT_REAR].quadrant.x         =  1;
  legs[RIGHT_REAR].quadrant.y         = -1;

  legs[LEFT_REAR].quadrant.x          = -1;
  legs[LEFT_REAR].quadrant.y          = -1;

  legs[LEFT_FRONT].id.coxa            = LF_COXA_ID;
  legs[LEFT_FRONT].id.femur           = LF_FEMUR_ID;
  legs[LEFT_FRONT].id.tibia           = LF_TIBIA_ID;

  legs[RIGHT_FRONT].id.coxa           = RF_COXA_ID;
  legs[RIGHT_FRONT].id.femur          = RF_FEMUR_ID;
  legs[RIGHT_FRONT].id.tibia          = RF_TIBIA_ID;

  legs[RIGHT_REAR].id.coxa            = RR_COXA_ID;
  legs[RIGHT_REAR].id.femur           = RR_FEMUR_ID;
  legs[RIGHT_REAR].id.tibia           = RR_TIBIA_ID;

  legs[LEFT_REAR].id.coxa             = LR_COXA_ID;
  legs[LEFT_REAR].id.femur            = LR_FEMUR_ID;
  legs[LEFT_REAR].id.tibia            = LR_TIBIA_ID;

  legs[LEFT_FRONT].center.coxa        = 195;
  legs[LEFT_FRONT].center.femur       =  60;
  legs[LEFT_FRONT].center.tibia       = 240;

  legs[RIGHT_FRONT].center.coxa       = 105;
  legs[RIGHT_FRONT].center.femur      = 240;
  legs[RIGHT_FRONT].center.tibia      =  60;

  legs[RIGHT_REAR].center.coxa        = 195;
  legs[RIGHT_REAR].center.femur       =  60;
  legs[RIGHT_REAR].center.tibia       = 240;

  legs[LEFT_REAR].center.coxa         = 105;
  legs[LEFT_REAR].center.femur        = 240;
  legs[LEFT_REAR].center.tibia        =  60;

  legs[LEFT_FRONT].direction.coxa     = -1;
  legs[LEFT_FRONT].direction.femur    =  1;
  legs[LEFT_FRONT].direction.tibia    = -1;

  legs[RIGHT_FRONT].direction.coxa    =  1;
  legs[RIGHT_FRONT].direction.femur   = -1;
  legs[RIGHT_FRONT].direction.tibia   =  1;

  legs[RIGHT_REAR].direction.coxa     = -1;
  legs[RIGHT_REAR].direction.femur    =  1;
  legs[RIGHT_REAR].direction.tibia    = -1;

  legs[LEFT_REAR].direction.coxa      =  1;
  legs[LEFT_REAR].direction.femur     = -1;
  legs[LEFT_REAR].direction.tibia     =  1;
}

/* ------------------------------------------------------------------------------------------ */
/* Timmer                                                                                     */
/* ------------------------------------------------------------------------------------------ */

// Functions to use a timmer as a counter that keeps track of elapsed program run time in milliseconds.

volatile uint32_t milliseconds_count = 0;

void timmer_init(void)
{
  // Waveform Generation Mode - Fast PWM
  TCCR0A |= (1 << WGM01);
  TCCR0A |= (1 << WGM00);
  
  // Clock Select - Clock/64
  TCCR0B |= (1 << CS01);
  TCCR0B |= (1 << CS00);

  // Overflow Interrupt enable
  TIMSK0 |= (1 << TOIE0);
}

uint32_t millis(void)
{
  return milliseconds_count;
}

ISR(TIMER0_OVF_vect)
{
  milliseconds_count += 1;
}

/* ------------------------------------------------------------------------------------------ */
/* Rotation Matrix                                                                            */
/* ------------------------------------------------------------------------------------------ */

// Functions to calculate the position of a point on a rigid body that rotates around the axis 
// of a global coordinate frame.

// Rotation about the X axis.
pos x_rotation(float phi, float x, float y, float z)
{
  float s = sin(phi);
  float c  = cos(phi);

  pos result;
  result.x = x;
  result.y = (c * y) + (-s * z);
  result.z = (s * y) + (c * z);
  return result;
}

// Rotation about the Y axis.
pos y_rotation(float phi, float x, float y, float z)
{
  float s = sin(phi);
  float c  = cos(phi);

  pos result;
  result.x = (c * x) + (s * z);
  result.y = y;
  result.z = (-s * x) + (c * z);
  return result;
}

// Rotation about the Y axis.
pos z_rotation(float phi, float x, float y, float z)
{
  float s = sin(phi);
  float c  = cos(phi);

  pos result;
  result.x = (c * x) + (-s * y);
  result.y = (s * x) + (c * y);
  result.z = z;
  return result;
}

/* ------------------------------------------------------------------------------------------ */
/* Body IK                                                                                    */
/* ------------------------------------------------------------------------------------------ */

// Body IK - From origin of the robot find the position of the legs (coxa serov horn).
pos body_ik(float body_x, float body_y, float body_z, float roll, float pitch, float yaw)
{
  pos r = y_rotation(roll, body_x, body_y, body_z);
  pos p = x_rotation(pitch, r.x, r.y, r.z);
  pos y = z_rotation(yaw, p.x, p.y, p.z);  
  return y;
}

/* ------------------------------------------------------------------------------------------ */
/* 3DOF IK                                                                                    */
/* ------------------------------------------------------------------------------------------ */

// Leg IK - Calculate the leg servo joints.
angle leg_ik(float x, float y, float z)
{
  angle result;

  int leg_length = sqrt(pow(x, 2) + pow(y, 2));

  int side_a = sqrt(pow((leg_length - COXA_LENGTH), 2) + pow(z, 2));
  int side_b = TIBIA_LENGTH;
  int side_c = FEMUR_LENGTH;

  int angle_a = acos((-pow(side_a, 2) + pow(side_b, 2) + pow(side_c, 2)) / (2 * side_b * side_c)) * 57.295;
  int angle_b = acos(( pow(side_a, 2) - pow(side_b, 2) + pow(side_c, 2)) / (2 * side_a * side_c)) * 57.295;
  // float angle_c = acos(( pow(side_a, 2) + pow(side_b, 2) - pow(side_c, 2)) / (2 * side_a * side_b)) * 57.295;

  result.coxa = atan(y / x) * 57.295;

  result.femur = (angle_b - fabs(atan(z / (leg_length - COXA_LENGTH)) * 57.295));

  if (angle_a > 90)
    result.tibia = angle_a - 90;
  else if (angle_a < 90)
    result.tibia = -1 * (90 - angle_a);
  else
    result.tibia = 0;

  return result;
}

/* ------------------------------------------------------------------------------------------ */
/* Turret IK                                                                                  */
/* ------------------------------------------------------------------------------------------ */

int turret_position[2];

void turret_ik(int mode, int pan, int tilt){

  if(mode == 0)
  {
    turret_position[0] = (int) (pan);
    turret_position[1] = (int) (tilt);
  }
  else if(mode == 1)
  {
    turret_position[0] = 0;
    turret_position[1] = 0;
  }
  else if(mode == 2)
  {
    turret_position[0] = 0;
    turret_position[1] = 0;
  }

  if(turret_position[0] > PAN_MAX) turret_position[0] = PAN_MAX;
  else if(turret_position[0] < PAN_MIN) turret_position[0] = PAN_MIN;
  else if(turret_position[0] > TILT_MAX) turret_position[0] = TILT_MAX;
  else if(turret_position[0] < TILT_MIN) turret_position[0] = TILT_MIN;
  
}

/* ------------------------------------------------------------------------------------------ */
/* Gait                                                                                       */
/* ------------------------------------------------------------------------------------------ */

gaits g;

void gait_mode_select(int gaitmode)
{  
  if(gaitmode == 0)
  {
    g.leg_order[LEFT_FRONT]   = 0;
    g.leg_order[RIGHT_FRONT]  = 2;
    g.leg_order[LEFT_REAR]    = 2;
    g.leg_order[RIGHT_REAR]   = 0;
    g.push_steps              = 2;
    g.steps                   = 4;
    g.lift_height             = 25;
  }
  else if(gaitmode == 1)
  {
    g.leg_order[LEFT_FRONT]   = 0;
    g.leg_order[RIGHT_FRONT]  = 2;
    g.leg_order[LEFT_REAR]    = 4;
    g.leg_order[RIGHT_REAR]   = 6;
    g.push_steps              = 6;
    g.steps                   = 8;
    g.lift_height             = 25;
  }
  g.step = 0;
}

pos gait(pos gait, int moving, int leg, float travel_x, float travel_y)
{
  // If we should be moving...
  //if((fabs(travel_x) > 5)  || (fabs(travel_y)) > 5){
  if(moving > 0)
  {
    // Leg up and and neutral x/y.
    if(g.step == g.leg_order[leg])
    {
      gait.x = 0;
      gait.y = 0;
      gait.z = g.lift_height;
    }
    // Leg full forward and down.
    else if(g.step == g.leg_order[leg] + 1)
    {
      gait.x = travel_x;
      gait.y = travel_y;
      gait.z = 0;
    }
    // Leg moves body forward.
    else
    {
      gait.x = gait.x - (travel_x / (g.push_steps / 2));
      gait.y = gait.y - (travel_y / (g.push_steps / 2));
      gait.z = 0;
    }

    // Increment gait step if last leg.
    if(leg == 3)
      g.step = (g.step + 1) % g.steps;

  }
  // Not moving. Leg down, centered, reset step count.
  else
  {
    gait.x = 0;
    gait.y = 0;
    gait.z = 0;
    g.step = 0;
  }

  return gait;
}

/* ------------------------------------------------------------------------------------------ */
/* Gun                                                                                        */
/* ------------------------------------------------------------------------------------------ */

void gun_init(void)
{
  // Set pin as output.
  DDRD |= (1 << PD4);
  DDRD |= (1 << PD5);

  // Setting the compaire output mode.
  TCCR1A |=  (1 << COM1A1);
  TCCR1A &= ~(1 << COM1A0);
  TCCR1A |=  (1 << COM1B1);
  TCCR1A &= ~(1 << COM1B0);

  // Setting the waveform generation mode.
  TCCR1A &= ~(1 << WGM10);
  TCCR1A &= ~(1 << WGM11);
  TCCR1B &= ~(1 << WGM12);
  TCCR1B |=  (1 << WGM13);

  // Setting the clock prescaler.
  TCCR1B &= ~(1 << CS10);
  TCCR1B |=  (1 << CS11);
  TCCR1B &= ~(1 << CS12);

  // Setting the input capture register.
  ICR1H = 20000 >> 8;
  ICR1L = 20000;
}

/* ------------------------------------------------------------------------------------------ */
/* Interpolation                                                                              */
/* ------------------------------------------------------------------------------------------ */

// Interpolates the transition from one position to the next for smooth walking by dividing 
// one movement up into many smaller movements. Also makes all servos arrive at their final
// position at nearly the same time.

#define POSITION_SIZE (NUM_LEGS * NUM_DOF) + 2
#define INTERPOLATION_STEPS 30

int position[POSITION_SIZE];      // Current position of each servo.
int next_position[POSITION_SIZE];  // Next desired position of each servo.
int step[POSITION_SIZE];          // Size of each servos interpolation step.
int interpolating = 0;            // Flag to indicating interpolation completion.
unsigned long last_time = 0;
int pause = 0;

void interpolation_init(int time, int pose[], int npose[])
{
  pause = (time / INTERPOLATION_STEPS) + 1;
  last_time = millis();

  for(int i = 0; i < POSITION_SIZE; i++)
  {
    if(npose[i] > pose[i])
      step[i] = (int) (npose[i] - pose[i]) / INTERPOLATION_STEPS + 1;
    else
      step[i] = (int) (pose[i] - npose[i]) / INTERPOLATION_STEPS + 1;
  }
  interpolating = 1;
}

void interpolation_step(int pose[], int npose[])
{
  while(millis() - last_time < pause);
  last_time = millis();

  int completion = POSITION_SIZE;
  for(int i = 0; i < POSITION_SIZE; i++)
  {
    int diff = npose[i] - pose[i];
    if(diff == 0)
      completion--;
    else
    {
      if(diff > 0)
      {
        if(diff < step[i])
        {
          pose[i] = npose[i];
          completion--;
        }
        else
          pose[i] += step[i];
      }
      else
      {
        if(-diff < step[i])
        {
          pose[i] = npose[i];
          completion--;
        }
        else
          pose[i] -= step[i];
      }
    }
  }
  if(completion <= 0)
    interpolating = 0;
}

/* ------------------------------------------------------------------------------------------ */
/* Main                                                                                       */
/* ------------------------------------------------------------------------------------------ */

void do_pose(int pose, int position[])
{  
  // sitting
  if(pose == 1)
  {
    for (int i = 0; i < NUM_LEGS; i++)
    {
      legs[i].foot_endpoint.x = legs[i].quadrant.x * legs[i].foot_neutral.x;
      legs[i].foot_endpoint.y = legs[i].quadrant.y * legs[i].foot_neutral.y;
      legs[i].foot_endpoint.z = 50;
      legs[i].ik = leg_ik(legs[i].foot_endpoint.x, legs[i].foot_endpoint.y, legs[i].foot_endpoint.z);
      position[legs[i].id.coxa  - 1] = (int) ((legs[i].center.coxa  + (legs[i].direction.coxa  * legs[i].ik.coxa))  * 3.40);
      position[legs[i].id.femur - 1] = (int) ((legs[i].center.femur + (legs[i].direction.femur * legs[i].ik.femur)) * 3.40);
      position[legs[i].id.tibia - 1] = (int) ((legs[i].center.tibia + (legs[i].direction.tibia * legs[i].ik.tibia)) * 3.40);
    }
  }
}

void do_gait(void)
{
  pos travel[4];
  int moving = 0;
  int magnitude = 0;

  for (int leg = 0; leg < NUM_LEGS; leg++)
  {
    pos rot_z = z_rotation((((controller.paramaters[PARAM_TRAVEL_Z] - 127) / 127.0) * (10 * 3.14 / 180)), legs[leg].foot_neutral.x, legs[leg].foot_neutral.y, 0);

    travel[leg].x = (((controller.paramaters[PARAM_TRAVEL_X] - 127) / 127.0) * 35.0) + (legs[leg].foot_neutral.x - rot_z.x);
    travel[leg].y = (((controller.paramaters[PARAM_TRAVEL_Y] - 127) / 127.0) * 35.0) + (legs[leg].foot_neutral.y - rot_z.y);

    magnitude = sqrt(pow(travel[leg].x, 2) + pow(travel[leg].y, 2));

    if(magnitude > 35 )
    {
      travel[leg].x = travel[leg].x * (35.0 / magnitude);
      travel[leg].y = travel[leg].y * (35.0 / magnitude);
    }

    if((fabs(travel[leg].x) > 5)  || (fabs(travel[leg].y)) > 5)
      moving += 1;
  }

  for (int leg = 0; leg < NUM_LEGS; leg++)
    legs[leg].foot_gait = gait(legs[leg].foot_gait, moving, leg, travel[leg].x, travel[leg].y);

}

void do_ik(int position[])
{
  for (int i = 0; i < NUM_LEGS; i++)
  {
    // Body IK
    legs[i].body_endpoint.x = legs[i].body_neutral.x + legs[i].body_gait.x;
    legs[i].body_endpoint.y = legs[i].body_neutral.y + legs[i].body_gait.y;
    legs[i].body_endpoint.z = legs[i].body_neutral.z + legs[i].body_gait.z;

    legs[i].body_rotation.x = legs[i].body_gait_rot.x;
    legs[i].body_rotation.y = legs[i].body_gait_rot.y;
    legs[i].body_rotation.z = legs[i].body_gait_rot.z;

    legs[i].body_endpoint = body_ik(legs[i].body_endpoint.x, legs[i].body_endpoint.y, legs[i].body_endpoint.z, legs[i].body_rotation.x, legs[i].body_rotation.y, legs[i].body_rotation.z);

    // Leg IK
    legs[i].foot_endpoint.x = legs[i].quadrant.x * (legs[i].foot_neutral.x + legs[i].foot_gait.x - legs[i].body_endpoint.x);
    legs[i].foot_endpoint.y = legs[i].quadrant.y * (legs[i].foot_neutral.y + legs[i].foot_gait.y - legs[i].body_endpoint.y);
    legs[i].foot_endpoint.z =                       legs[i].foot_neutral.z + legs[i].foot_gait.z + legs[i].body_endpoint.z;

    legs[i].ik = leg_ik(legs[i].foot_endpoint.x, legs[i].foot_endpoint.y, legs[i].foot_endpoint.z);

    // Joint Conversions from deg to ax-12 ticks
    position[legs[i].id.coxa  - 1] = (int) ((legs[i].center.coxa  + (legs[i].direction.coxa  * legs[i].ik.coxa))  * 3.40);
    position[legs[i].id.femur - 1] = (int) ((legs[i].center.femur + (legs[i].direction.femur * legs[i].ik.femur)) * 3.40);
    position[legs[i].id.tibia - 1] = (int) ((legs[i].center.tibia + (legs[i].direction.tibia * legs[i].ik.tibia)) * 3.40);
  }
}

void do_turret(int position[])
{
  turret_ik(controller.paramaters[PARAM_TURRET_MODE], ((controller.paramaters[PARAM_TURRET_PAN] - 127) / 127.0), ((controller.paramaters[PARAM_TURRET_TILT] - 127) / 127.0));
  position[PAN_ID  - 1] = (int) ((150 + turret_position[0]) * 3.40);
  position[TILT_ID - 1] = (int) ((150 + turret_position[1]) * 3.40);
}

void do_guns(void)
{
  if((controller.paramaters[PARAM_GUN_STATUS] & 1) == 1)
  {
    OCR1AH = 2000 >> 8;
    OCR1AL = 2000;
  }
  else
  {
    OCR1AH = 1000 >> 8;
    OCR1AL = 1000;
  }

  if((controller.paramaters[PARAM_GUN_STATUS] & 2) == 2){
    OCR1BH = 2000 >> 8;
    OCR1BL = 2000;
  }else{
    OCR1BH = 1000 >> 8;
    OCR1BL = 1000;
  }
}

int main(void)
{
  timmer_init();
  ax12_init(1000000);
  controller_init(115200);
  leg_init(legs);
  gun_init();

  // Turn servo packet return off and set goal speed to max.
  ax12_write(AX_BROADCAST_ID, AX_STATUS_RETURN_LEVEL, AX_RETURN_NONE);
  ax12_write2(AX_BROADCAST_ID, AX_GOAL_SPEED_L, 0x03FF);

  // Set the gait
  gait_mode_select(0);

  // Starting position
  do_gait();
  do_ik(position);
  do_turret(position);
  ax12_sync_write(AX_GOAL_POSITION_L, 2, 14, position);

  _delay_ms(800);
  
  // Enable interupts.
  sei();

  while(1)
  {
    if(interpolating == 0)
    {
      controller_read();
      do_gait();
      do_ik(next_position);
      do_turret(next_position);
      do_guns();
      //interpolation_init(controller.paramaters[PARAM_INTERPOLATION_TIME], position, next_position);
      interpolation_init(150, position, next_position);
    }
    else
    {
      interpolation_step(position, next_position);
      ax12_sync_write(AX_GOAL_POSITION_L, 2, 14, position);
    }
  }
  
  return(0);
}
