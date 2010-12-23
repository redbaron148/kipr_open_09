#ifndef THIS_DEFINES_H
#define THIS_DEFINES_H   

// Create Locations
#define HOME                0.379449307919,0.611061275005
#define FIRST_FIELD         0.750000000000,0.824456801414
#define SECOND_FIELD        0.750000000000,1.162345061300
#define MIDDLE              0.750000000000,1.551653289790
#define POSTS               1.35,1.43
#define SCAN                1.1,1.63
#define SAFE_SCAN           0.8,1.63

// Servo Indicies
#define BASE                0
#define SHOULDER            1
#define ELBOW               4
#define WRIST_FLEX          3
#define WRIST_TWIST         5
#define CLAW                2

#define OPEN                100
#define CLOSE               0

#define START_POSITION      60,10,0,0,10,0
#define REST_POSITION       0,10,gsp(CLAW),0,10,0
#define END_POSITION        0,10,gsp(CLAW),0,10,0
#define SCORE_POSITION_LEFT 90,110,gsp(CLAW),60,90,0
#define UP_POSITION         65,gsp(CLAW),40,100,0
#define OUT_POSITION        120,gsp(CLAW),60,110,0
#define SWIPE_MIDDLE        15,90,0,80,80,0
#define SWIPE_DOWN          30,120,100,125,45,0
#define SWIPE_START_POSITION -70,5,0,0,5,0

#define HAS_SOMETHING_ANGLE 140
#define TWIST_HAS_SOMETHING_ANGLE 45

#define SERVO_SPEED         0.20  //speed of slowest servo on arm (per 60 degrees)

#define START_TIME_OFFSET   1.0
#define PHASE_ONE_TIME      75.0
#define PHASE_TWO_TIME      140.0
#define PHASE_THREE_TIME    140.0
#define GAME_DURATION       180.0

// Info About the camera frames
#define IMAGE_WIDTH         320
#define IMAGE_HEIGHT        240
#define IMAGE_FOV           46    //camera field of view (degrees)
  
#define CENTER_X            IMAGE_WIDTH/2
#define CENTER_Y            IMAGE_HEIGHT/2

#define CENTER_X_OFFSET     20
#define CENTER_Y_OFFSET     20

#define MAX_POM_SIZE        1100
#define MAX_POM_WIDTH       40

#define ARM_CENTER_X        190
#define ARM_CENTER_Y        CENTER_Y

#define MAX_ARM_REACH       22.5
#define MIN_ARM_REACH       10.

#ifndef PIE
#define PIE 3.14159
#endif

float ORIENTATION = 1.57;

bool ROBOT_HAS_PATH = false;

float LIGHT_THRESH =        90.;

// PID Control Variables
float CAMERA_P = 1.5;
float CAMERA_I = 1;
float CAMERA_D = 1;

// Color channels
enum  CHANNEL{ 
  orange=0, 
  botguy, 
  yellow, 
  blue, 
  greenFoam, 
  greenPom,
  orangePom,
  orangeFoam
};

enum DIRECTION{
  left=0,
  right,
  forward,
  backward
};

// Strings for nice printing to screen, makes debuging flow easier 
std::string channel[8]  = {"orange",
                           "botguy",
                           "yellow pom",
                           "blue foam ball",
                           "green foam ball",
                           "green pom",
                           "orange foam ball",
                           "orange pom"};
                 
std::string servo[6]    = {"base",
                           "shoulder",
                           "claw",
                           "wrist flex",
                           "wrist twist"};

#endif
