/*robot world setup
ensure imu module is added to turretslot
rotation  0-1-0-1.5708
added solid node to turret slot with recognition
*/

/*added camera2 in groundsensor slot 
translation -0.08 1.59268e-05 0.05
rotation 0 0 1 1.57
*/



/*distance sensor 
<80 => free space ahead
100+ => very close to wall
1000+ =>collided */

/*todo
add PID control to align_robot_to
*/ 


/*led 0 always on */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <webots/device.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/nodes.h>
#include <webots/robot.h>
#include <webots/inertial_unit.h>
#include <webots/camera.h>
#include <webots/keyboard.h>
#include <webots/camera_recognition_object.h>

/* Device stuff */
#define DISTANCE_SENSORS_NUMBER 8
static WbDeviceTag distance_sensors[DISTANCE_SENSORS_NUMBER];
static double distance_sensors_values[DISTANCE_SENSORS_NUMBER];
static const char *distance_sensors_names[DISTANCE_SENSORS_NUMBER] = {"ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"};

#define GROUND_SENSORS_NUMBER 3
static WbDeviceTag ground_sensors[GROUND_SENSORS_NUMBER];
static double ground_sensors_values[GROUND_SENSORS_NUMBER] = {0.0, 0.0, 0.0};
static const char *ground_sensors_names[GROUND_SENSORS_NUMBER] = {"gs0", "gs1", "gs2"};

#define LEDS_NUMBER 10
static WbDeviceTag leds[LEDS_NUMBER];
static bool leds_values[LEDS_NUMBER];
static const char *leds_names[LEDS_NUMBER] = {"led0", "led1", "led2", "led3", "led4", "led5", "led6", "led7", "led8", "led9"};

static WbDeviceTag imu;
static double rpy[3];

static WbDeviceTag camera;
 
static WbDeviceTag left_motor, right_motor;

static void static_search_for(char *model);
static void align_bot_to(char *model);
#define LEFT 0
#define RIGHT 1
#define MAX_SPEED 6.28
static double speeds[2];

/* Breitenberg stuff */
static double weights[DISTANCE_SENSORS_NUMBER][2] = {{-1.3, -1.0}, {-1.3, -1.0}, {-0.5, 0.5}, {0.0, 0.0},
                                                     {0.0, 0.0},   {0.05, -0.5}, {-0.75, 0},  {-0.75, 0}};
static double offsets[2] = {0.5 * MAX_SPEED, 0.5 * MAX_SPEED};

static int get_time_step() {
  static int time_step = -1;
  if (time_step == -1)
    time_step = (int)wb_robot_get_basic_time_step();
  return time_step;
}

static void step() {
  if (wb_robot_step(get_time_step()) == -1) {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}

static void passive_wait(double sec) {
  double start_time = wb_robot_get_time();
  do {
    step();
  } while (start_time + sec > wb_robot_get_time());
}

static void init_devices() {
  int i;
  for (i = 0; i < DISTANCE_SENSORS_NUMBER; i++) {
    distance_sensors[i] = wb_robot_get_device(distance_sensors_names[i]);
    wb_distance_sensor_enable(distance_sensors[i], get_time_step());
  }
  for (i = 0; i < LEDS_NUMBER; i++)
    leds[i] = wb_robot_get_device(leds_names[i]);
    
  imu = wb_robot_get_device("inertial unit");
  wb_inertial_unit_enable(imu,get_time_step());

  camera = wb_robot_get_device("camera2");
  wb_camera_enable(camera,get_time_step());
  wb_camera_recognition_enable(camera,get_time_step());
  wb_camera_set_fov(camera,1.5);
  // silently initialize the ground sensors if they exists
  for (i = 0; i < GROUND_SENSORS_NUMBER; i++)
    ground_sensors[i] = (WbDeviceTag)0;
  int ndevices = wb_robot_get_number_of_devices();
  for (i = 0; i < ndevices; i++) {
    WbDeviceTag dtag = wb_robot_get_device_by_index(i);
    const char *dname = wb_device_get_name(dtag);
    WbNodeType dtype = wb_device_get_node_type(dtag);
    if (dtype == WB_NODE_DISTANCE_SENSOR && strlen(dname) == 3 && dname[0] == 'g' && dname[1] == 's') {
      int id = dname[2] - '0';
      if (id >= 0 && id < GROUND_SENSORS_NUMBER) {
        ground_sensors[id] = wb_robot_get_device(ground_sensors_names[id]);
        wb_distance_sensor_enable(ground_sensors[id], get_time_step());
      }
    }
  }

  // get a handler to the motors and set target position to infinity (speed control).
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  step();
}

static void reset_actuator_values() {
  int i;
  for (i = 0; i < 2; i++)
    speeds[i] = 0.0;
  for (i = 0; i < LEDS_NUMBER; i++)
    leds_values[i] = false;
}

static void get_sensor_input() {
  int i;
  for (i = 0; i < DISTANCE_SENSORS_NUMBER; i++) {
    distance_sensors_values[i] = wb_distance_sensor_get_value(distance_sensors[i]);

    // scale the data in order to have a value between 0.0 and 1.0
    // 1.0 representing something to avoid, 0.0 representing nothing to avoid
    distance_sensors_values[i] /= 4096;
  }

  for (i = 0; i < GROUND_SENSORS_NUMBER; i++) {
    if (ground_sensors[i])
      ground_sensors_values[i] = wb_distance_sensor_get_value(ground_sensors[i]);
  }
  
  
}

static double get_yaw(){
  for (int i=0;i<3;i++)
  rpy[i] = wb_inertial_unit_get_roll_pitch_yaw(imu)[i]; //rpy is a double pointer 
  return(rpy[2]);
}


static bool cliff_detected() {
  int i;
  for (i = 0; i < GROUND_SENSORS_NUMBER; i++) {
    if (!ground_sensors[i])
      return false;
    if (ground_sensors_values[i] < 500.0)
      return true;
  }
  return false;
}

static void set_actuators() {
  int i;
  for (i = 1; i < LEDS_NUMBER; i++)//led 0 always on
    wb_led_set(leds[i], leds_values[i]);
  wb_motor_set_velocity(left_motor, speeds[LEFT]);
  wb_motor_set_velocity(right_motor, speeds[RIGHT]);
}

static void blink_leds() {
  static int counter = 0;
  counter++;
  leds_values[(counter / 10) % LEDS_NUMBER] = true;
}

static void steady_leds(int x) {
  if(x==1)
  {
   for(int i=1;i<LEDS_NUMBER;i++)
   {
    leds_values[i] = true;
    wb_led_set(leds[i], leds_values[i]);
   }
  }
  else
  {
   for(int i=1;i<LEDS_NUMBER;i++)
   {
    leds_values[i] = false;
    wb_led_set(leds[i], leds_values[i]);
   }
  
  }
}

static void run_braitenberg() {
  int i, j;
  for (i = 0; i < 2; i++) {
    speeds[i] = 0.0;
    for (j = 0; j < DISTANCE_SENSORS_NUMBER; j++)
      speeds[i] += distance_sensors_values[j] * weights[j][i];

    speeds[i] = offsets[i] + speeds[i] * MAX_SPEED;
    if (speeds[i] > MAX_SPEED)
      speeds[i] = MAX_SPEED;
    else if (speeds[i] < -MAX_SPEED)
      speeds[i] = -MAX_SPEED;
  }
}

static void go_forward(int sf) {
  wb_motor_set_velocity(left_motor, MAX_SPEED/sf);
  wb_motor_set_velocity(right_motor, MAX_SPEED/sf);
  passive_wait(0.2);
}
static void go_backwards(int sf) {
  wb_motor_set_velocity(left_motor, -MAX_SPEED/sf);
  wb_motor_set_velocity(right_motor, -MAX_SPEED/sf);
  passive_wait(0.2);
}

static void turn_left(int sf) {
  wb_motor_set_velocity(left_motor, -MAX_SPEED/sf);
  wb_motor_set_velocity(right_motor, MAX_SPEED/sf);
  passive_wait(0.2);
}

static void turn_right(int sf) {
  wb_motor_set_velocity(left_motor, MAX_SPEED/sf);
  wb_motor_set_velocity(right_motor, -MAX_SPEED/sf);
  passive_wait(0.2);
}

static void stop() {
  wb_motor_set_velocity(left_motor, 0);
  wb_motor_set_velocity(right_motor, 0);
  passive_wait(0.2);
}

static void rotate_cw(double  t){
if(t>M_PI ||t<0)
{
printf("ERROR::input rotate angles in range 0 to pi\n");
return;
}
//input is in radians
double current_yaw = get_yaw();
double target_yaw = current_yaw - t;
while(target_yaw>M_PI)
target_yaw = target_yaw - 2*M_PI;
while (target_yaw<-M_PI)
target_yaw = target_yaw + 2*M_PI;

while(fabs(current_yaw - target_yaw)>0.0349066)//error 2 deg
{
turn_right(5);//turns at 1/5 speed to prevent infinte rotation
current_yaw = get_yaw();
steady_leds(1);
step();
}
 steady_leds(0);
}

static void rotate_ccw(double  t){
//input is in radians
if(t>M_PI ||t<0)
{printf("ERROR::input rotate angles in range 0 to pi");
return;
}
double current_yaw = get_yaw();
double target_yaw = current_yaw + t;
while(target_yaw>M_PI)
target_yaw = target_yaw - 2*M_PI;
while (target_yaw<-M_PI)
target_yaw = target_yaw + 2*M_PI;

while(fabs(current_yaw - target_yaw)>0.0349066)//error 2 deg
{turn_left(5);
current_yaw = get_yaw();
 steady_leds(1);
step();
}
 steady_leds(0);
}

static void get_distance_sensor_input() {
  int i;
  for (i = 0; i < DISTANCE_SENSORS_NUMBER; i++) {
    distance_sensors_values[i] = wb_distance_sensor_get_value(distance_sensors[i]);
  }
    
}

static bool free_space_ahead(){
 get_distance_sensor_input();
 if(distance_sensors_values[0] <80 || distance_sensors_values[7] < 80)
   return true;
 else
  return false;
}

static bool free_space_left(){
 get_distance_sensor_input();
 if(distance_sensors_values[6] <80|| distance_sensors_values[5] < 80)
   return true;
 else
  return false;
}

/*static void follow_wall_left(){ 
  
  //while checking for collision (todo)
  if(!free_space_ahead)
  {
    printf("wall infront! turning right\n");
    turn_right(5);
  }
  
  if(!free_space_left())
  {
    printf("wall left! moving forward\n");
    go_forward(5);
  }
  else
  {
    printf("turning left\n");
    turn_left(5);
  }
  
}*/
static double *get_cameraRecognitionObjectPosition(char *model)
{
const WbCameraRecognitionObject *objects = wb_camera_recognition_get_objects(camera);
 
      int i = wb_camera_recognition_get_number_of_objects(camera);
      for(int j=0;j<= i-1; j++)
      {
      if(strcmp(model,objects[j].model)==0)
        i = j; //object to be pushed found
      }
      //printf("Model %d: %s has been found returning position values..\n", i, objects[i].model);
      //printf("Relative position of object %d: %lf %lf %lf\n", i, objects[i].position[0], objects[i].position[1],
        //     objects[i].position[2]);
             
      return (objects[i].position);
}

static void approach(char *model)
{
printf("approaching ...\n");
double* position = get_cameraRecognitionObjectPosition("object");
while(position[2]<=-0.35)//move till safe distance
{
  align_bot_to(model);
  go_forward(7);  
  position = get_cameraRecognitionObjectPosition("object");
}

printf("approached! now in position!\n");
}
// add function to get closer -0.157

/*
demonstrates how to use camera recognition
static void use_camera(void)
{
const WbCameraRecognitionObject *objects = wb_camera_recognition_get_objects(camera);
 
      int i = wb_camera_recognition_get_number_of_objects(camera);
      for(int j=0;j<= i-1; j++)
      {
      if(strcmp("object",objects[j].model)==0)
        i = j; //object to be pushed found
      }
      printf("Model of object %d: %s\n", i, objects[i].model);
      printf("Relative position of object %d: %lf %lf %lf\n", i, objects[i].position[0], objects[i].position[1],
             objects[i].position[2]);
      
}
*/

static void align_bot_to(char *model)
{
  double *position = get_cameraRecognitionObjectPosition("object");
  if (position[2]<=-1.35) //very far so roughly align and move
{   printf("aligning from far away\n"); 
  while(position[0]>=0.1||position[0]<=-0.1) 
    {
      
      if (position[0]>0)
       { turn_right(20);
        position = get_cameraRecognitionObjectPosition("object");
        }
      else
       {turn_left(20);  
        position = get_cameraRecognitionObjectPosition("object");  //maybe if arena is too big try to align for 4-5 times and giveup , wander
      }  //or maybe only consider aligning if size of object in frame is big enough
    }
 }
else
{
  while(position[0]>=0.05||position[0]<=-0.05) 
    {printf("aligning from near by\n"); 
      if (position[0]>0)
        {turn_right(10);
        position = get_cameraRecognitionObjectPosition("object");
        }
      else
       {turn_left(10); 
        position = get_cameraRecognitionObjectPosition("object"); 
        }  
    }  
}


printf("aligned\n");   



}



static void static_search_for(char *model)
{

int i = wb_camera_recognition_get_number_of_objects(camera);
//if no objects in fov then rotate and search again
if (i==0)
  {
    printf("nothing found\n");
    turn_left(7);
    static_search_for(model);
  }

//if one or more ojects in view
else 
  { printf("something found\n");
    const WbCameraRecognitionObject *objects = wb_camera_recognition_get_objects(camera);
    
    
      for(int j=0;j<= i-1; j++)
      {
      if(strcmp(model,objects[j].model)==0)//is the detected object the correct model?
        {printf("object found\n");
          align_bot_to(model);
          break;//object found
        } 
        else if (j==i-1)//if loop is complete and object not found
          { printf("neglecting unidentified object\n");
            turn_left(7);
            static_search_for(model);
          }
               
      }
      
  }


}



int main(int argc, char **argv) {
  wb_robot_init();

  printf("Allwin's controller of the e-puck robot started...\n");


  init_devices();
  wb_led_set(leds[0], true);
  wb_keyboard_enable(get_time_step());
  reset_actuator_values();
 
  while (true) 
  {
  
    int key = wb_keyboard_get_key();
    if(key== 315)
    {
     go_forward(1);

    }
    else if(key== 317)
    {
       go_backwards(1);
    }
    else if(key== 314)
    {
      //rotate_ccw(M_PI);
     turn_left(5);
    }
    else if(key== 316)
    {
    //rotate_cw(M_PI);
    turn_right(5);
    }
    else if(key == 65)
    {//A
    rotate_ccw(M_PI/2);
    }
     else if(key == 68)
    {//D
    rotate_cw(M_PI/2);
    }
       else if (key == 87)
    {//W
    //use_camera();
    double *position = get_cameraRecognitionObjectPosition("object");
    printf("position is : %lf %lf %lf\n", position[0], position[1], position[2]);
    }
        else if (key == 81)
    {//Q
    static_search_for("object");
    }
    else if (key == 69)
    {//E
    approach("object");
    }
    else  
    {
      stop();
    }
    step();
  
 }
 

/*  //wandering
    while (true) {
    reset_actuator_values();
    get_sensor_input();
    blink_leds();
    if (cliff_detected()) {
      go_backwards(1);
      turn_left(1);
    } else {
      run_braitenberg();
    }
    set_actuators();
    step();
  };
*/

  return EXIT_SUCCESS;
 }
