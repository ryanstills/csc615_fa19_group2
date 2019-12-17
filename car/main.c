//taken from the raspberry Pi tutorial
#include <wiringPi.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/time.h>
#include <pthread.h>
#include <stdlib.h>
#include <softPwm.h>

//contains defintion of wiringPi pin refs
#include "pins.h"

#define MAX_DISTANCE 220// define the maximum measured distance
#define timeOut MAX_DISTANCE*60 // calculate timeout according to the maximum measured distance
#define NUM_THREADS 7

#define ARR_SIZE 5

#define BASE_SPEED 110
#define TURN_SPEED 5
#define SAFE_DISTANCE 30
#define LEFT_MOTOR 50
#define RIGHT_MOTOR 51

#define TEST_TIME 3000

/* Car Modes:
    1 - Driving
    2 - Waiting
    3 - Obstacle Avoidance 
 */

typedef struct CarInfo {
	float speed;
	int mode;
	
	//sensor readouts
	volatile int ir_readout;
	volatile float ultrasonic_readout;
	volatile int left_line_readout;
	volatile int right_line_readout;
	volatile int tilt_readout;
	// more info to come
	
	//motor speed
	float left_motor_speed;
	float right_motor_speed;
	
	//sensor threads
	pthread_t threads[NUM_THREADS];
	
} CarInfo;


//function pulseIn: obtain pulse time of a pin
int pulseIn(int pin, int level, int timeout);
void startCar(struct CarInfo *);
float avg_distance(int, float *);
void motorStop();
void motorTest();

void * initMotor(void * carInfo) {
   printf("initMotor()\n");
    pinMode(left_motor_f, OUTPUT);
    pinMode(left_motor_r, OUTPUT);
    pinMode(left_motor_pwm, PWM_OUTPUT);  
   
    pinMode(right_motor_f, OUTPUT);
    pinMode(right_motor_r, OUTPUT);
    pinMode(right_motor_pwm, PWM_OUTPUT);
    
   softPwmCreate(left_motor_pwm, 0, 400);
   softPwmCreate(right_motor_pwm, 0, 400);
}

int setMotorSpeed(int motor, int speed, int forward, int reverse){
    
    //~ printf("setMotorSpeed()\n");
    if(speed > 400) speed = 400;
    
    if(motor == LEFT_MOTOR){
	//~ printf("LEFT MOTOR\nspeed: %d\n", speed);
	digitalWrite(left_motor_f, forward);
	digitalWrite(left_motor_r, reverse);
	softPwmWrite(left_motor_pwm, speed);
    }
    if(motor == RIGHT_MOTOR){
	//~ printf("RIGHT MOTOR\nspeed: %d\n", speed);
	digitalWrite(right_motor_f, forward);
	digitalWrite(right_motor_r, reverse);
	softPwmWrite(right_motor_pwm, speed);
    }
    
    return speed;
}

void * getMotorSpeed(void * carInfo){
    printf("getMotorSpeed()\n");
    struct CarInfo * car;
    car = (struct CarInfo *) carInfo;
    
    pinMode(leftmotor_d0, INPUT);
    pinMode(rightmotor_d0, INPUT);
    
    while(car->mode != 0){
	car->left_motor_speed = digitalRead(leftmotor_d0);
	car->right_motor_speed = digitalRead(rightmotor_d0);
    }
} 

void * getSonar(void * carInfo){ // get the measurement results of ultrasonic module, with unit: cm
    printf("getSonar()\n");
    struct CarInfo * car;
    car = (struct CarInfo *) carInfo;
	
    pinMode(trigPin,OUTPUT);
    pinMode(echoPin,INPUT);
	
    while(car->mode != 0){
	    long pingTime;
	    float readout;
	    digitalWrite(trigPin,HIGH); //trigPin send 10us high level
	    delayMicroseconds(2500);
	    digitalWrite(trigPin,LOW);
	    pingTime = pulseIn(echoPin,HIGH,timeOut); //read plus time of echoPin
	    readout = (float)pingTime * 340.0 / 2.0 / 10000.0;
	
	    car->ultrasonic_readout = (readout < 1.0) ? MAX_DISTANCE : readout; // the sound speed is 340m/s, and calculate distance
    }
}

void * getIR(void * carInfo){
	printf("getIR()\n");
	struct CarInfo * car;
	car = (struct CarInfo *) carInfo;

	pinMode(irPin,INPUT);
    
	while(car->mode != 0){
		car->ir_readout = !digitalRead(irPin);
	}
}

void * getLineSensor(void * carInfo){
	printf("getLineReader()\n");
	struct CarInfo * car;
	car = (struct CarInfo *) carInfo;
    
	pinMode(leftLinePin, INPUT);
	pinMode(rightLinePin, INPUT);
	
	while(car->mode != 0){
		car->left_line_readout = !digitalRead(leftLinePin);
		car->right_line_readout = !digitalRead(rightLinePin);
	}
	
}

void * getTiltSensor( void * carInfo){
	printf("getLineReader()\n");
	struct CarInfo * car;
	car = (struct CarInfo *) carInfo;
	
	pinMode(tiltPin, INPUT);
	
	while(car->mode != 0){
	    car->tilt_readout = !digitalRead(tiltPin);
	}
}
   
void startCar(struct CarInfo * carInfo){
	
	initMotor(carInfo);
	pthread_create(&carInfo->threads[0], NULL, getSonar, (void *) carInfo);
	pthread_create(&carInfo->threads[1], NULL, getIR, (void *) carInfo);
	pthread_create(&carInfo->threads[2], NULL, getLineSensor, (void *) carInfo);
	pthread_create(&carInfo->threads[3], NULL, getTiltSensor, (void *) carInfo);
	pthread_create(&carInfo->threads[4], NULL, getMotorSpeed, (void *) carInfo);
}

int main(){
  
    printf("Program is starting ... \n");
    if(wiringPiSetup() == -1){ //when initialize wiring failed, print message to screen
        printf("setup wiringPi failed !");
        return 1;
    }
    CarInfo * carInfo = malloc(sizeof(CarInfo));
    carInfo->mode = 1;
    
    startCar(carInfo);
     
    int count = 0;
    int distance_flag = 0;
    int arr_pos = 0;
    //adjusting left motor speed by 24 to compensate for speed mismatch
    int left_motor_speed = BASE_SPEED + 24;
    int right_motor_speed = BASE_SPEED;
    int current_speed = BASE_SPEED;
    
    float distance_arr[ARR_SIZE];
    //~ motorStop();
    //~ return 0;
    printf("Car not started\n");
    while(carInfo->ir_readout != 1) {}
    printf("Car started\n");
    sleep(2);
    
   
    
    while(carInfo->mode != 0)
    {
	distance_arr[arr_pos] = carInfo->ultrasonic_readout;
	arr_pos++;
	if(arr_pos > 4){
	    arr_pos = 0;
	}
	if(carInfo->mode == 1){
	    
	    if(avg_distance(ARR_SIZE, distance_arr) <= SAFE_DISTANCE){	   
		printf("Car stopped due to unsafe distance of: %2f cm\n", carInfo->ultrasonic_readout);
		motorStop();
		carInfo->mode = 2;
		printf("Changing to car mode %d\n", carInfo->mode);
	    }
	
	    while(carInfo->left_line_readout == 1)
	    {		
		left_motor_speed = setMotorSpeed(LEFT_MOTOR, TURN_SPEED, HIGH, LOW);
		right_motor_speed = setMotorSpeed(RIGHT_MOTOR, right_motor_speed,HIGH, LOW);
		usleep(2500);
	    }
	    left_motor_speed = setMotorSpeed(LEFT_MOTOR, BASE_SPEED + 24, HIGH, LOW);
	    right_motor_speed = setMotorSpeed(RIGHT_MOTOR, BASE_SPEED , HIGH, LOW);
	    while(carInfo->right_line_readout == 1)
	    {
		left_motor_speed = setMotorSpeed(LEFT_MOTOR, left_motor_speed, HIGH, LOW);
		right_motor_speed = setMotorSpeed(RIGHT_MOTOR, TURN_SPEED, HIGH, LOW);
		usleep(2500);
	    }
	    
	    left_motor_speed = setMotorSpeed(LEFT_MOTOR, BASE_SPEED + 24, HIGH, LOW);
	    right_motor_speed = setMotorSpeed(RIGHT_MOTOR, BASE_SPEED , HIGH, LOW);

	}
	else if(carInfo->mode == 2){
	    printf("mode 2\n");
	    carInfo->mode = 0;
	    motorStop();
	}
	if(count > TEST_TIME){
	    carInfo->mode = 0;
	    printf("+++++++++++++++++++++++++++++++++++++++++++++\n");
	    motorStop();
	}
	usleep(10000);
	count++;
    }

    for(int i = 0; i < 5; i++){
	printf("join thread %d\n", i);
	pthread_join(carInfo->threads[i], NULL);
    }
    free(carInfo);
    return 1;
}
void motorStop()
{
    printf("motorStop()\n");
    setMotorSpeed(LEFT_MOTOR,0,HIGH,LOW);
    setMotorSpeed(RIGHT_MOTOR,0,HIGH,LOW);
    for(int i = 1; i <= 3; i++){
	printf("waiting %d..\r", i);
	sleep(1);
    }
}
int pulseIn(int pin, int level, int timeout)
{
   struct timeval tn, t0, t1;

   long micros;

   gettimeofday(&t0, NULL);

   micros = 0;

   while (digitalRead(pin) != level)
   {
      gettimeofday(&tn, NULL);

      if (tn.tv_sec > t0.tv_sec) micros = 1000000L; else micros = 0;
      micros += (tn.tv_usec - t0.tv_usec);

      if (micros > timeout) return 0;
   }

   gettimeofday(&t1, NULL);

   while (digitalRead(pin) == level)
   {
      gettimeofday(&tn, NULL);

      if (tn.tv_sec > t0.tv_sec) micros = 1000000L; else micros = 0;
      micros = micros + (tn.tv_usec - t0.tv_usec);

      if (micros > timeout) return 0;
   }

   if (tn.tv_sec > t1.tv_sec) micros = 1000000L; else micros = 0;
   if (tn.tv_sec > t1.tv_sec) micros = 1000000L; else micros = 0;
   micros = micros + (tn.tv_usec - t1.tv_usec);

   return micros;
}

float avg_distance(int max_size, float * distance_arr){
    float sum = 0.0;
    for(int i = 0; i < max_size; i++){
	sum += distance_arr[i];
    }
    return sum / max_size;
}
void motorTest(){
    
    printf("start\n");
    for(int i = 25; i <= 40; i++){
	printf("Forward %d\n", i);
	setMotorSpeed(LEFT_MOTOR, i, HIGH, LOW);
	setMotorSpeed(RIGHT_MOTOR, i, HIGH, LOW);
	sleep(1);
    }
    motorStop();
    for(int i = 25; i <= 40; i++){
	printf("Reverse %d\n", i);
	setMotorSpeed(LEFT_MOTOR, i, LOW, HIGH);
	setMotorSpeed(RIGHT_MOTOR, i, LOW, HIGH);
	sleep(1);
    } 
    motorStop();
    printf("finish\n");
}
