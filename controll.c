//
//  controll.c
//  ultrasonic
//
//  Created by ParkZoo on 2021/06/10.
//

#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <stdlib.h>
#include <wiringSerial.h>
#include <pthread.h>
#include <string.h>
#include <math.h>

#include <fcntl.h>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>

char *IP = "192.168.0.136";
int PORT = 9001;
int sock_serv, sock_cli;
struct sockaddr_in sockinfo_serv, sockinfo_cli;

#define t1 0
#define e1 1

#define t2 2
#define e2 3

#define t3 4
#define e3 5

#define t4 21
#define e4 22

#define t5 23
#define e5 24

// Ultra sonic sensor variable

struct Us_total{
    int trig[5];
    int echo[5];
    double dt[5];
};

pthread_t thr_sonic;

// Key input thread
pthread_t thr_input;

// Serial Comm variable
char device[] = "/dev/ttyACM0";
int fd;
unsigned long baud = 9600;

// MPU6050 sensor variable
const float pi = 3.141592;
float AcX, AcY, AcZ, GyX, GyY, GyZ;
float baseAcX, baseAcY, baseAcZ, baseGyX, baseGyY, baseGyZ;
const int mpuAddr = 0x68, buffAddr=0x3b, pwrAddr=0x6b;
float RadToDeg = 180/pi;
const float GyrToDegPerSec = 131;
const float alpha = 0.96;
float gyro_x, gyro_y, gyro_z;
int hndl;
long t_prev, t_now;
float dt;
float accel_angle_x, accel_angle_y, accel_angle_z;
float gyro_angle_x, gyro_angle_y, gyro_angle_z;
float fil_angle_x, fil_angle_y, fil_angle_z;

pthread_t thr_mpu;

void *getangle();
void loop(void);
void readAccelGyro(void);
void calibAccelGyro(void);
void initDt(void);
void calcDt(void);
void calcAccelYPR(void);
void calcGyroYPR(void);
void calcFilteredYPR(void);

void *dist(void* data);
void setupSensor(void);

void *keyinput();
int isRange(double val);
double getdistance(int wTRIG, int wECHO);
void *dist_t(void *data);

const double limit_dist = 25.;
int isKeyIn = 1;

int main(){
    fflush(stdout);
    

    // Setup wiringPi
    if(wiringPiSetup()==-1) exit(1);
    if((fd=serialOpen(device, baud))<0) exit(1);
    if((hndl = wiringPiI2CSetup(mpuAddr))==-1) exit(1);
    if(wiringPiI2CWriteReg8(hndl, pwrAddr,0)==-1) exit(1);
    
    pinMode(t1,OUTPUT); pinMode(t2,OUTPUT); pinMode(t3,OUTPUT);
    pinMode(e1,INPUT); pinMode(e2,INPUT); pinMode(e3,INPUT);
    
    pinMode(t4,OUTPUT); pinMode(t5,OUTPUT);
    pinMode(e4,INPUT); pinMode(e5,INPUT);
    
    struct Us_total *ut = malloc(sizeof(struct Us_total));
    int t[5] ={0,2,4,21,23};
    int e[5] = {1,3,5,22,24};
    
    for(int i=0;i<5;i++){
        *(ut->trig+i) = t[i];
        *(ut->echo+i) = e[i];
    }
    

    calibAccelGyro();
    initDt();
    
    //thread create
    pthread_create(&thr_sonic,NULL, dist_t, (void*)ut);
    pthread_create(&thr_mpu, NULL, getangle, NULL);
    pthread_create(&thr_input, NULL, keyinput, NULL);
    
    char op='w';
    char prev='i';
    
    float rotAng =0.;
    int cn=0;
    
    int dcnt=0;
    double dprev[3];
    
    int init = 0;
    serialPutchar(fd,op);
    while(1){
        // us1: front_center, us2: front_left, us3: front_right
        // us4: left, us5: right
        
        if(!init&&(!isRange(ut->dt[0])||!isRange(ut->dt[1])||!isRange(ut->dt[2])||!isRange(ut->dt[3])||!isRange(ut->dt[4]))){
            //printf("init return\n");
            continue;
        }
        
        if(!init) init=1;
        
        if(isKeyIn==-1) break;
        if(isKeyIn){
            //isKeyIn=0;
            continue;
            //serialPutchar(fd,'x');
            //break;
        }
        
        // normal state 
        if(isRange(ut->dt[0]) && isRange(ut->dt[1]) && isRange(ut->dt[2])){
            // dir change
            if(ut->dt[0] <= limit_dist || ut->dt[1]<=limit_dist || ut->dt[2]<=limit_dist){            
                printf("turn: %d\n",cn++);
                if(prev!='a') op ='d';
                // go left 
                if(ut->dt[4] <= limit_dist) op = 'a';
            }
            else op = 'w';
            dcnt = 0;
        }
        
        // abnormal
        else{
            dcnt++;
            if(dcnt>20) {
                printf("back\n");
                op = 's';
                //serialPutchar(fd,op);
                //continue;
            }
            // 1ㅇㅣㄴㅐ
        }
        
        
        
        printf("--> %.2f %.2f %.2f\n", ut->dt[0],ut->dt[1],ut->dt[2]);
        if(prev!=op) serialPutchar(fd,op);
        prev = op;
        
        
        
        /*
        if(ut->dt[0]>500.||ut->dt[1]>500.||ut->dt[2]>500.){
            dcnt++;
        }
        else dcnt=0;
        printf("dnct: %d --> %.2f %.2f %.2f\n",dcnt, ut->dt[0],ut->dt[1],ut->dt[2]);
        if(dcnt>30) {
            printf("back\n");
            while(1){
                if(isRange(ut->dt[0])&&isRange(ut->dt[1])&&isRange(ut->dt[2])) break;
                else serialPutchar(fd,'s');
            }
            serialPutchar(fd,'w');
            //continue;
        }
    
        //printf("%.2f %.2f %.2f %.2f %.2f\n", ut->dt[0],ut->dt[1],ut->dt[2],ut->dt[3],ut->dt[4]);
        if(ut->dt[0] <= limit_dist || ut->dt[1]<=limit_dist || ut->dt[2]<=limit_dist){

            printf("turn: %d\n",cn++);
            op ='d';
            // go left 
            if(ut->dt[4] <= limit_dist) op = 'a';
            if(prev!=op){
                //printf("send: %d", op);
                serialPutchar(fd,op);
            }
            
            while(1){
                if(ut->dt[0] > limit_dist && ut->dt[1]>limit_dist && ut->dt[2]>limit_dist) break;
            }
            // rotAng = fil_angle_z;
        }
        */
        dprev[0]= ut->dt[0];
        dprev[1]= ut->dt[1];
        dprev[2]= ut->dt[2];
        
        //prev = op;
        
        
        delay(50);
    }
    
    serialPutchar(fd,'x');
    
    pthread_join(thr_mpu,NULL);
    pthread_join(thr_input,NULL);
    pthread_join(thr_sonic,NULL);
    
    free(ut);
    exit(1);
    return 0;
}

int isRange(double val){
    return(val>=2&&val<=500);
}

void *getangle(){
    double ini_angle =0;
    while(1){
        loop();
        static int cnt;
        cnt++;
        if(cnt%2==0){
            //printf("%.2f\n",fil_angle_z);
        }
        delay(50);
    }
}


void loop(){
    readAccelGyro();
    calcDt();
    calcAccelYPR();
    calcGyroYPR();
    calcFilteredYPR();
}
short i2cInt16(int handle,int addr){
    short d1 = wiringPiI2CReadReg8(handle,addr);
    short d2 = wiringPiI2CReadReg8(handle, addr+1);
    short d3 = (d1<<8) | d2;
    return d3;
}

void readAccelGyro(){
    AcX=i2cInt16(hndl, buffAddr);
    AcY=i2cInt16(hndl, buffAddr+2);
    AcZ=i2cInt16(hndl, buffAddr+4);

    GyX=i2cInt16(hndl, buffAddr+8);
    GyY=i2cInt16(hndl, buffAddr+10);
    GyZ=i2cInt16(hndl, buffAddr+12);
}
void calibAccelGyro(){
    float sumAcX =0, sumAcY=0, sumAcZ=0;
    float sumGyX =0, sumGyY=0, sumGyZ=0;
    
    readAccelGyro();
    for(int i=0;i<10;i++){
        readAccelGyro();
        sumAcX += AcX; sumAcY += AcY; sumAcZ += AcZ;
        sumGyX += GyX; sumGyY += GyY; sumGyZ += GyZ;
        delay(10);
    }
    baseAcX = sumAcX/10;
    baseAcY = sumAcY/10;
    baseAcZ = sumAcZ/10;
    
    baseGyX = sumGyX/10;
    baseGyY = sumGyY/10;
    baseGyZ = sumGyZ/10;
    
}
void initDt(){
    t_prev = millis();
}
void calcDt(){
    t_now = millis();
    dt = (t_now-t_prev)/1000.0;
    t_prev =t_now;
}
void calcAccelYPR(){
    float accel_x, accel_y, accel_z;
    float accel_xz, accel_yz;
    accel_x = AcX-baseAcX;
    accel_y = AcY-baseAcY;
    accel_z = AcZ+(16384-baseAcZ);
    
    accel_yz = sqrt(pow(accel_y,2)+pow(accel_z,2));
    accel_angle_y = atan(-accel_x/accel_yz)*RadToDeg;
    
    accel_xz = sqrt(pow(accel_x,2)+pow(accel_z,2));
    accel_angle_x = atan(accel_y/accel_xz)*RadToDeg;
    
    accel_angle_z=0;
}
void calcGyroYPR(){
    gyro_x = (GyX-baseGyX)/GyrToDegPerSec;
    gyro_y = (GyY-baseGyY)/GyrToDegPerSec;
    gyro_z = (GyZ-baseGyZ)/GyrToDegPerSec;
    
    gyro_angle_x += gyro_x*dt;
    gyro_angle_y += gyro_y*dt;
    gyro_angle_z += gyro_z*dt;
    
}
void calcFilteredYPR(){
    float tmp_angle_x = fil_angle_x+gyro_x*dt;
    float tmp_angle_y = fil_angle_y+gyro_y*dt;
    float tmp_angle_z = fil_angle_z+gyro_z*dt;
    
    fil_angle_x = alpha*tmp_angle_x + (1.0-alpha)*accel_angle_x;
    fil_angle_y = alpha*tmp_angle_y + (1.0-alpha)*accel_angle_y;
    fil_angle_z = tmp_angle_z;
}

void *keyinput(){
    char buf[1024];

	sock_serv = socket(AF_INET, SOCK_STREAM, 0);

	sockinfo_serv.sin_family = AF_INET;
	sockinfo_serv.sin_addr.s_addr = htonl(INADDR_ANY);
	sockinfo_serv.sin_port = htons(PORT);

	bind(sock_serv, (struct sockaddr*)&sockinfo_serv, sizeof(sockinfo_serv));

	listen(sock_serv, 100);

	int n = sizeof(sockinfo_cli);
    while(1)
	{
		sock_cli = accept(sock_serv, (struct sockaddr*)&sockinfo_cli, &n);
		int i=recv(sock_cli,buf,1024,0);	
		if(i>0)	buf[i]=0;
        printf("%s\n", buf);
        serialPutchar(fd,buf[0]);
		//if(buf[0]=='q') break;
        delay(50);
		
    }
	close(sock_cli);
	close(sock_serv);
    /*
    char op= '\0';
    while(1){
        op = getchar();
        getchar();
        isKeyIn=1;
        serialPutchar(fd, op);
        if(op=='x') isKeyIn=-1;
        delay(100);
    }
    */
}

double getdistance(int wTRIG, int wECHO){
    double result, start_time, end_time;
    digitalWrite(wTRIG,LOW);
    delay(100);
    digitalWrite(wTRIG,HIGH);
    delayMicroseconds(10);
    digitalWrite(wTRIG,LOW);
        
    while(digitalRead(wECHO)==0);
    start_time=micros();
        
    while(digitalRead(wECHO)==1);
    end_time=micros();
        
    result = (end_time-start_time)/29./2.;
    return result;
}

void *dist_t(void *data){
    struct Us_total *data_ = (struct Us_total*)data;
    int *tg = data_->trig;
    int *ec = data_->echo;
    double ds[5];

    
    while(1){
        for(int i=0;i<5;i++){
            ds[i]=getdistance(*(tg+i), *(ec+i));
            //printf("%d : %.2f\t",i, ds[i]);
            *(data_->dt+i) = ds[i];
        }
        
        //printf(">> %.2f %.2f %.2f\n", ds[0],ds[1],ds[2]);
        delay(10);
        //printf("\n");
    }
    
}

