/*
Project: Real-Time Autonomous Vehicle
Author: Kishore Kanna P

Description:
Multi-threaded embedded system using QNX RTOS for autonomous navigation.
Implements sensor fusion using ultrasonic sensors, IMU, and encoders.
Includes obstacle detection, collision avoidance, and real-time motor control.

Note:
Hardware-specific configurations (GPIO base addresses, pin mappings) 
may vary depending on platform and are simplified for demonstration.
*/
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <time.h>
#include <math.h>
#include <fcntl.h>
#include <devctl.h>
#include <hw/i2c.h>
#include <sys/neutrino.h>
#include <sys/mman.h>
#include <pthread.h>

#define GPIO_BASE 0xFE200000
#define GPFSEL0 0
#define GPSET0  7
#define GPCLR0  10
#define GPLEV0  13

#define BUTTON_PIN 7

volatile uint32_t *gpio;
volatile int system_running = 0;

char direction = 'S'; // F, L, R, B, S

/*
⚠️ Note:
This implementation is a simplified representation of the full system.
Certain hardware calibration, tuning parameters, and advanced optimizations
are not included in this repository.
*/

// ----------- GPIO -----------
void set_output(int pin){
    int r=pin/10, s=(pin%10)*3;
    gpio[r]&=~(7<<s);
    gpio[r]|=(1<<s);
}
void set_input(int pin){
    int r=pin/10, s=(pin%10)*3;
    gpio[r]&=~(7<<s);
}
void write_pin(int pin,int v){
    if(v) gpio[GPSET0]=(1<<pin);
    else  gpio[GPCLR0]=(1<<pin);
}
int read_pin(int pin){
    return (gpio[GPLEV0]&(1<<pin))?1:0;
}
uint64_t now_us(){
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC,&ts);
    return ts.tv_sec*1000000ULL + ts.tv_nsec/1000;
}

// ----------- BUTTON -----------
void *button_thread(void *arg){
    int last=1;
    while(1){
        int cur=read_pin(BUTTON_PIN);
        if(last==1 && cur==0){
            system_running=!system_running;
            printf("\nSYSTEM %s\n", system_running?"STARTED":"STOPPED");
            usleep(300000);
        }
        last=cur;
        usleep(10000);
    }
}

// ----------- ULTRASONIC -----------
int trig[6]={17,27,22,23,24,25};
int echo[6]={4,18,21,20,10,9};

double us[6], us_prev[6];
int us_status[6];

// NEW: separate perception
int obstacle_detected[6];   // 1 if 2–10 cm
int collision_detected[6];  // 1 if <2 cm

double measure(int t,int e){
    write_pin(t,0); usleep(2);
    write_pin(t,1); usleep(10);
    write_pin(t,0);

    uint64_t t0=now_us();
    while(!read_pin(e)){
        if(now_us()-t0>30000) return -1;
    }
    uint64_t s=now_us();
    while(read_pin(e)){
        if(now_us()-s>30000) break;
    }
    return (now_us()-s)/58.0;
}

void validate_ultra(){
    for(int i=0;i<6;i++){
        us_status[i]=0;
        obstacle_detected[i]=0;
        collision_detected[i]=0;

        // --- SENSOR VALIDATION (UNCHANGED) ---
        if(us[i]<ULTRA_MIN || us[i]>ULTRA_MAX)
            us_status[i]=2;
        else if(fabs(us[i]-us_prev[i])>ULTRA_JUMP)
            us_status[i]=1;

        if(i<5 && fabs(us[i]-us[i+1])>100)
            us_status[i]=1;

        // --- PERCEPTION (SEPARATE) ---
        if(us[i] >= 0 && us[i] < COLLISION_DIST)
            collision_detected[i]=1;
        else if(us[i] >= COLLISION_DIST && us[i] <= OBSTACLE_DIST)
            obstacle_detected[i]=1;

        us_prev[i]=us[i];
    }
}

void *ultra_thread(void *arg){
    while(1){
        if(system_running){
            for(int i=0;i<6;i++){
                us[i]=measure(trig[i],echo[i]);
                usleep(60000);
            }
            validate_ultra();
        }
        usleep(10000);
    }
}

// ----------- ENCODER -----------
int ENC_L=11, ENC_R=8;
volatile uint32_t countL=0,countR=0;
double speedL=0,speedR=0, prev_speedL=0, prev_speedR=0;
int enc_status=0;

#define PPR 20.0
#define RADIUS 0.03

void *encoder_thread(void *arg){
    int l_prev=0,r_prev=0;
    while(1){
        if(system_running){
            int l=read_pin(ENC_L);
            int r=read_pin(ENC_R);
            if(l && !l_prev) countL++;
            if(r && !r_prev) countR++;
            l_prev=l; r_prev=r;
        }
        usleep(500);
    }
}

void validate_encoder(){
    enc_status=0;

    if(speedL>SPEED_MAX || speedR>SPEED_MAX)
        enc_status=2;

    if(fabs(speedL-prev_speedL)>SPEED_JUMP)
        enc_status=1;

    if(fabs(speedL-speedR)>1.0)
        enc_status=1;

    prev_speedL=speedL;
    prev_speedR=speedR;
}

void *speed_thread(void *arg){
    uint32_t pL=0,pR=0;
    uint64_t t0=now_us();

    while(1){
        if(system_running){
            usleep(200000);
            uint64_t t1=now_us();
            double dt=(t1-t0)/1e6;

            uint32_t cL=countL,cR=countR;

            speedL=((cL-pL)/PPR)/dt * 2*M_PI*RADIUS;
            speedR=((cR-pR)/PPR)/dt * 2*M_PI*RADIUS;

            validate_encoder();

            pL=cL; pR=cR; t0=t1;
        }
    }
}

// ----------- IMU -----------
#define MPU_ADDR 0x68
#define ACCEL_XOUT_H 0x3B
#define PWR_MGMT_1 0x6B

typedef struct{
    i2c_sendrecv_t hdr;
    uint8_t buf[15];
}pkt;

int i2c_fd;
double ax,ay,az,prev_ax=0,prev_ay=0,prev_az=0;
int imu_status=0;

void mpu_init(){
    uint8_t d[2]={PWR_MGMT_1,0};
    write(i2c_fd,d,2);
    sleep(1);
}

void validate_imu(){
    imu_status=0;

    double mag=sqrt(ax*ax+ay*ay+az*az);

    if(mag>ACC_LIMIT)
        imu_status=2;

    if(fabs(ax-prev_ax)>ACC_JUMP)
        imu_status=1;

    prev_ax=ax; prev_ay=ay; prev_az=az;
}

void *imu_thread(void *arg){
    pkt p;
    p.hdr.slave.addr=MPU_ADDR;
    p.hdr.slave.fmt=I2C_ADDRFMT_7BIT;
    p.hdr.send_len=1;
    p.hdr.recv_len=14;

    while(1){
        if(system_running){
            p.buf[0]=ACCEL_XOUT_H;

            if(!devctl(i2c_fd,DCMD_I2C_SENDRECV,&p,sizeof(p),NULL)){
                int16_t x=(p.buf[1]<<8)|p.buf[2];
                int16_t y=(p.buf[3]<<8)|p.buf[4];
                int16_t z=(p.buf[5]<<8)|p.buf[6];

                ax=(x/16384.0)*9.81;
                ay=(y/16384.0)*9.81;
                az=(z/16384.0)*9.81;

                validate_imu();
            }
        }
        usleep(100000);
    }
}

// ----------- FUSION (FAULTS ONLY) -----------
int fusion_status=0;

void fusion_check(){
    fusion_status=0;

    for(int i=0;i<6;i++){
        if(us_status[i]==2)
            fusion_status=1;
    }

    if(speedL<0.05 && (fabs(ax)>2 || fabs(ay)>2))
        fusion_status=1;
}

// ----------- MOTOR -----------
int PWMA=12,PWMB=13,AIN1=5,AIN2=6,BIN1=19,BIN2=16,STBY=26;

void forward(){
    write_pin(STBY,1);
    write_pin(AIN1,1); write_pin(AIN2,0);
    write_pin(BIN1,1); write_pin(BIN2,0);
}
void stop_motor(){
    write_pin(STBY,0);
}

void *motor_thread(void *arg){
    while(1){
        if(system_running){

            // ---- COLLISION PRIORITY ----
            for(int i=0;i<6;i++){
                if(collision_detected[i]){
                    printf("COLLISION at S%d → STOP\n", i+1);
                    stop_motor();
                    usleep(200000);
                    goto end_cycle;
                }
            }

            // ---- DIRECTION MAPPING ----
            int front = obstacle_detected[1] + obstacle_detected[0] + obstacle_detected[2];
            int left  = obstacle_detected[2] + obstacle_detected[3];
            int right = obstacle_detected[0] + obstacle_detected[5];

            if(front){
                printf("FRONT OBSTACLE → STOP\n");
                stop_motor();
            }
            else if(left){
                printf("LEFT OBSTACLE → TURN RIGHT\n");
                write_pin(AIN1,1); write_pin(AIN2,0);
                write_pin(BIN1,0); write_pin(BIN2,1);
            }
            else if(right){
                printf("RIGHT OBSTACLE → TURN LEFT\n");
                write_pin(AIN1,0); write_pin(AIN2,1);
                write_pin(BIN1,1); write_pin(BIN2,0);
            }
            else{
                forward();
                write_pin(PWMA,1); write_pin(PWMB,1);
            }

            usleep(150000);

        } else {
            stop_motor();
            usleep(100000);
        }
end_cycle:;
    }
}

// ----------- MAIN -----------
int main(){
    ThreadCtl(_NTO_TCTL_IO,0);
    gpio=(volatile uint32_t*)mmap_device_io(0x1000,GPIO_BASE);

    for(int i=0;i<6;i++){
        set_output(trig[i]);
        set_input(echo[i]);
    }

    set_input(ENC_L);
    set_input(ENC_R);

    set_output(PWMA); set_output(PWMB);
    set_output(AIN1); set_output(AIN2);
    set_output(BIN1); set_output(BIN2);
    set_output(STBY);

    set_input(BUTTON_PIN);

    i2c_fd=open("/dev/i2c1",O_RDWR);
    mpu_init();

    pthread_t b,u,e,s,i,m;
    pthread_create(&b,NULL,button_thread,NULL);
    pthread_create(&u,NULL,ultra_thread,NULL);
    pthread_create(&e,NULL,encoder_thread,NULL);
    pthread_create(&s,NULL,speed_thread,NULL);
    pthread_create(&i,NULL,imu_thread,NULL);
    pthread_create(&m,NULL,motor_thread,NULL);

    printf("Press button to START/STOP system\n");

    while(1){
        if(system_running){
            fusion_check();

            // -------- DISPLAY FORMAT --------
            char dir = 'F'; // default

            // Determine direction (based on your motor logic)
            int front = obstacle_detected[1] + obstacle_detected[0] + obstacle_detected[2];
            int left  = obstacle_detected[2] + obstacle_detected[3];
            int right = obstacle_detected[0] + obstacle_detected[5];

            if(front) dir = 'B';       // stopping due to front
            else if(left) dir = 'R';
            else if(right) dir = 'L';
            else dir = 'F';

            // Faulty sensors
            char faulty[50] = "";
            int first = 1;
            for(int i=0;i<6;i++){
                if(us_status[i]==2){
                    char temp[10];
                    sprintf(temp,"%d",i+1);
                    if(!first) strcat(faulty,"|");
                    strcat(faulty,temp);
                    first=0;
                }
            }
            if(first) strcpy(faulty,"None");

            // Obstacle sensors
            char obs[50] = "";
            first = 1;
            for(int i=0;i<6;i++){
                if(obstacle_detected[i]){
                    char temp[10];
                    sprintf(temp,"%d",i+1);
                    if(!first) strcat(obs,"|");
                    strcat(obs,temp);
                    first=0;
                }
            }
            if(first) strcpy(obs,"None");

            // -------- FINAL OUTPUT --------
            printf("\n%-15s | L=%.2f R=%.2f | %c | %s\n",
                   faulty,
                   speedL, speedR,
                   dir,
                   obs);
        }
        usleep(300000);
    }
}
