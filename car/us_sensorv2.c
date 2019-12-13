//taken from the raspberry Pi tutorial
#include <wiringPi.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/time.h>
#include <pthread.h>
#include <stdlib.h>
#include <softPwm.h>

//ultrasonic sensor pins
#define trigPin 0
#define echoPin 2

#define irPin 10

#define leftLinePin 16
#define rightLinePin 15

#define tiltPin 3

//left motor data pins
#define leftmotor_a0 8
#define leftmotor_d0 9

//Left motor control pins
#define left_motor_pwm 12
#define left_motor_f 13
#define left_motor_r 14

//Right motor control pins
#define right_motor_pwm 6
#define right_motor_f 4
#define right_motor_r 5

#define MAX_DISTANCE 220 // define the maximum measured distance
#define timeOut MAX_DISTANCE*60 // calculate timeout according to the maximum measured distance
#define NUM_THREADS 7

#define BASE_SPEED 45
#define SAFE_DISTANCE 45.0
#define LEFT_MOTOR 50
#define RIGHT_MOTOR 51

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
	
	//left motor
	float leftmotor_speed_ao;
	float leftmotor_speed_do;
	
	//sensor threads
	pthread_t threads[NUM_THREADS];
	
} CarInfo;


//function pulseIn: obtain pulse time of a pin
int pulseIn(int pin, int level, int timeout);
void startCar(struct CarInfo *);
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
    
   softPwmCreate(left_motor_pwm, 0, 100);
   softPwmCreate(right_motor_pwm, 0, 100);
}

int setMotorSpeed(int motor, int speed, int forward, int reverse){
    
    printf("setMotorSpeed()\n");
    if(speed > 100) speed = 100;
    
    if(motor == LEFT_MOTOR){
	printf("LEFT MOTOR\nspeed: %d\nforward: %d\nreverse: %d\n", speed, forward, reverse);
	digitalWrite(left_motor_f, forward);
	digitalWrite(left_motor_r, reverse);
	softPwmWrite(left_motor_pwm, speed);
    }
    if(motor == RIGHT_MOTOR){
	printf("RIGHT MOTOR\nspeed: %d\nforward: %d\nreverse: %d\n", speed, forward, reverse);
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
    
    pinMode(leftmotor_a0, INPUT);
    pinMode(leftmotor_d0, INPUT);
    
    while(car->mode != 0){
	car->leftmotor_speed_ao = digitalRead(leftmotor_a0);
	car->leftmotor_speed_do = digitalRead(leftmotor_d0);
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
	    digitalWrite(trigPin,HIGH); //trigPin send 10us high level
	    delayMicroseconds(10);
	    digitalWrite(trigPin,LOW);
	    pingTime = pulseIn(echoPin,HIGH,timeOut); //read plus time of echoPin
	    car->ultrasonic_readout = (float)pingTime * 340.0 / 2.0 / 10000.0; // the sound speed is 340m/s, and calculate distance
	    if(car->ultrasonic_readout < 0.1){
		car->ultrasonic_readout = MAX_DISTANCE;
	    }
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
    int left_motor_speed = BASE_SPEED;
    int right_motor_speed = BASE_SPEED;
    int current_speed = BASE_SPEED;
    
    printf("Car not started\n");
    while(carInfo->ir_readout != 1) {
	
    }
    printf("Car started\n");
    
    while(carInfo->mode != 0)
    {
	printf("Ultrasonic Readout: %2f\n", carInfo->ultrasonic_readout);
	printf("IR Readout: %d\n", carInfo->ir_readout);
	printf("Tilt Readout: %d\n", carInfo->tilt_readout);
	printf("Left Line Sensor Readout: %d\n", carInfo->left_line_readout);
	printf("Right Line Sensor Readout: %d\n", carInfo->right_line_readout);
	if(carInfo->ultrasonic_readout < SAFE_DISTANCE){
	    motorStop();
	}
	else if(carInfo->left_line_readout == 0){
	    left_motor_speed = setMotorSpeed(LEFT_MOTOR, current_speed , HIGH, LOW);
	    right_motor_speed = setMotorSpeed(RIGHT_MOTOR, current_speed - 12 , HIGH, LOW);
	}
	else if(carInfo->right_line_readout == 0){
	    left_motor_speed = setMotorSpeed(LEFT_MOTOR, current_speed - 12, HIGH, LOW);
	    right_motor_speed = setMotorSpeed(RIGHT_MOTOR, current_speed , HIGH, LOW);
	}
	else if(carInfo->right_line_readout == 1 && (carInfo->left_line_readout == 1)){	    
	    left_motor_speed = setMotorSpeed(LEFT_MOTOR, current_speed , HIGH, LOW);
	    right_motor_speed = setMotorSpeed(RIGHT_MOTOR, current_speed , HIGH, LOW);
	}
	//~ else{	    
	    //~ left_motor_speed = setMotorSpeed(LEFT_MOTOR, current_speed , HIGH, LOW);
	    //~ right_motor_speed = setMotorSpeed(RIGHT_MOTOR, current_speed , HIGH, LOW);
	//~ }
	//~ sleep(1);
	usleep(20000);
	//~ delay(10);
	if(count > 2000){
	    carInfo->mode = 0;
	    motorStop();
	}
	count++;
    }

    for(int i = 0; i < 4; i++){
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
   micros = micros + (tn.tv_usec - t1.tv_usec);

   return micros;
}

void motorTest(){
    
    printf("start\n");
    for(int i = 25; i <= 50; i++){
	printf("Forward %d\n", i);
	setMotorSpeed(LEFT_MOTOR, i, HIGH, LOW);
	setMotorSpeed(RIGHT_MOTOR, i, HIGH, LOW);
	sleep(1);
    }
    motorStop();
    for(int i = 25; i <= 50; i++){
	printf("Reverse %d\n", i);
	setMotorSpeed(LEFT_MOTOR, i, HIGH, LOW);
	setMotorSpeed(RIGHT_MOTOR, i, HIGH, LOW);
	sleep(1);
    } 
    motorStop();
    printf("finish\n");
}
