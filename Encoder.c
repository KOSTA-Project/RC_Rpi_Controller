#include <stdio.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <stdlib.h>
#include <math.h>
#include <pthread.h>

#define left 0
#define right 1

pthread_t thr_input;
char device[] = "/dev/ttyACM0";
int fd;
unsigned long baud = 9600;

void *keyinput();

int main(){
	
	if(wiringPiSetup()==-1) exit(1);
    if((fd=serialOpen(device, baud))<0) exit(1);
    
    pinMode(left,INPUT);
    pinMode(right,INPUT);
    float dist = 21.2;
    
    float cntl = 0.;
    float cntr = 0.;
    int prevl = -1;
    int prevr = -1;
    
    pthread_create(&thr_input, NULL, keyinput, NULL);
	
    while(1){
	int ll = digitalRead(left);
	int rr = digitalRead(right);
		
        if(ll!=prevl){
	    cntl+=1.0;
	    prevl=ll;
	}
	if(rr!=prevr){
	    cntr+=1.0;
	    prevr=rr;
	}
	
	//printf("left : %.f\t right : %.2f\n", cntl, cntr);
	
        printf("left(%.2f) : %.2f\t right(%.2f) : %.2f\n", cntl,(cntl-1.0)/40.0*dist,cntr,(cntr-1.0)/40.0*dist);
        
        
        delay(10);
    }
    
    pthread_join(thr_input,NULL);
	
    return 0;
}
void *keyinput(){
	
    char op= '\0';
    while(1){
        op = getchar();
        getchar();
        
        serialPutchar(fd, op);
        delay(100);
    }
}
