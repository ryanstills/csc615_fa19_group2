//taken from the raspberry Pi tutorial
#include <wiringPi.h>
#include <stdio.h>
#include <sys/time.h>
#include <pthread.h>
#include <stdlib.h>

#define trigPin 7
#define echoPin 0
#define irPin 1
#define linePin 3

#define MAX_DISTANCE 220 // define the maximum measured distance
#define timeOut MAX_DISTANCE*60 // calculate timeout according to the maximum measured distance
#define NUM_THREADS 7

/* Car Modes:
    1 - Driving
	2 - Waiting
	3 - Obstacle Avoidance 
 */

typedef struct CarInfo {
	float speed;
	int mode;
	
	//sensor readouts
	int ir_readout;
	float us_distance;
	int line_readout;
	// more info to come	
	
	//sensor threads
	pthread_t threads[NUM_THREADS];
	
} CarInfo;


//function pulseIn: obtain pulse time of a pin
int pulseIn(int pin, int level, int timeout);
int startCar(struct CarInfo *);



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
		car->us_distance = (float)pingTime * 340.0 / 2.0 / 10000.0; // the sound speed is 340m/s, and calculate distance
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

void * getLineReader(void * carInfo){
	printf("getLineReader()\n");
	struct CarInfo * car;
    car = (struct CarInfo *) carInfo;
    
	pinMode(linePin, INPUT);
	
	while(car->mode != 0){
		car->line_readout = !digitalRead(linePin);
	}
	
}

int startCar(struct CarInfo * carInfo){
	
	pthread_create(&carInfo->threads[0], NULL, getSonar, (void *) carInfo);
	pthread_create(&carInfo->threads[1], NULL, getIR, (void *) carInfo);
	pthread_create(&carInfo->threads[2], NULL, getLineReader, (void *) carInfo);
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
	
    while(carInfo->mode = 1){
        printf("The distance is : %.2f cm\n", carInfo->us_distance);
        printf("The readout of the IR sensor is: %d\n", carInfo->ir_readout);
        printf("The readout of the Line sesnor is: %d\n", carInfo->line_readout);
        delay(1000);
    }
    
    free(carInfo);
    return 1;
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
