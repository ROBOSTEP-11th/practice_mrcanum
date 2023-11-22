#include "mbed.h"
#include <algorithm>
#include <cstdio>
#define M_PI 3.141592653590
#define WAIT_TIME_MS 500
#define MAX_SPEED 50
#define MAX_ACCELERATION 1.0
#define R_WHEEL 65 //車輪の半径[mm]
#define R 300 //機体の半径[mm]
#define CAN_ID 0x01
#define REV 0.04

CAN can(D10, D2);
CANMessage msg;

union Target {
    struct {
        short vx;
        short vy;
        short vr;
    } data;
    uint8_t raw[6];
};

Target target;
float Vx =0;
float Vy =0;
float Vr =0;

float Current_duty1; // 現在のduty
float Current_duty2; // 現在のduty
float Target_duty1; // 目標のduty
float Target_duty2; // 目標のduty

DigitalIn in(D3,PullUp); // 一つのコードで二種類のタイヤのコードを書くため
PwmOut pwm1_1(PA_1);  // モーター１の正転pwmを設定
PwmOut pwm1_2(PA_3); // モーター1の逆転pwmを設定
PwmOut pwm2_1(PA_6_ALT0); // モーター2の正転pwmを設定
PwmOut pwm2_2(PA_7_ALT1); // モーター2の逆転pwmを設定
float matrix[2][3];
Ticker ticker;

void motor(PwmOut *pwm1, PwmOut *pwm2, float speed);

void calc1(); // 機体の目標速度、角速度からdutyを計算する関数

void update(); // 目標速度に対して滑らかに台形加速をさせる関数

int main(){
    printf("START\r\n");
    can.frequency(1000'000);
    printf("frequency: 1Mbps\r\n");
    pwm1_1.period_us(50);
    pwm1_2.period_us(50);
    pwm2_1.period_us(50);
    pwm2_2.period_us(50);
    
   
    if (in == 1) {
        matrix[0][0] = -1;
        matrix[0][1] = 1;
        matrix[0][2] = 2.828427;
        matrix[1][0] = 1;
        matrix[1][1] = 1;
        matrix[1][2] = -2.828427;
    } else {
        matrix[0][0] = -1;
        matrix[0][1] = 1;
        matrix[0][2] = -2.828427;
        matrix[1][0] = 1;
        matrix[1][1] = 1;
        matrix[1][2] = 2.828427;
    }

    ticker.attach(update, 50ms);

    while(true){
        if (can.read(msg)) {
            // printf("id: %d, len: %d", msg.id, msg.len);
            // for (int i = 0; i < msg.len; i++) {
            //     printf(" %x", msg.data[i]);
            // }
            // printf("\r\n");
            if (msg.id != CAN_ID) continue;
            if (msg.len != 6) continue;
            for (int i = 0; i < msg.len; i++) {
                target.raw[i] = msg.data[i];
            }
            Vx = target.data.vx;
            Vy = target.data.vy;
            Vr = target.data.vr * M_PI / 180;
            calc1();
            // printf("vx: %d, vy: %d, vr: %d, ", target.data.vx, target.data.vy, target.data.vr);
            // printf("Vx: %f, Vy: %f, Vr: %f\r\n", Vx, Vy, Vr);
        }
    }
}

PwmOut *pwm1;
PwmOut *pwm2;


void calc1() {
    // 0.7071 * vx + 0.7071*vy + R*Vr*0.04 /r
    Target_duty1 = (matrix[0][0]*Vx + matrix[0][1]*Vy + matrix[0][2]*R*Vr)*REV / (4*M_PI*R_WHEEL);
    //cos(60°)=0.5,sin(60°)=0.86602....
    Target_duty2 = (matrix[1][0]*Vx + matrix[1][1]*Vy + matrix[1][2]*R*Vr)*REV / (4*M_PI*R_WHEEL);
    // printf("Vx: %f, Vy: %f, duty1: %f, duty2: %f\r\n", Vx, Vy, Target_duty1, Target_duty2);
}

inline float min(float a, float b) {
    if (a < b) {
        return a;
    } else {
        return b;
    }
}

inline float max(float a, float b) {
    if (a < b) {
        return b;
    } else {
        return a;
    }
}

void update(){
    if(Target_duty1 - Current_duty1 > 0){
        Current_duty1 = min(Target_duty1, Current_duty1+MAX_ACCELERATION);
        if(Current_duty1>0){
            pwm1_1.write(Current_duty1);
            pwm1_2.write(0);
        }else{
            pwm1_1.write(0);
            pwm1_2.write(-Current_duty1);  
        }
    } else {
        Current_duty1 = max(Target_duty1, Current_duty1-MAX_ACCELERATION);
        if(Current_duty1>0){
            pwm1_1.write(Current_duty1);
            pwm1_2.write(0);
        }else{
            pwm1_1.write(0);
            pwm1_2.write(-Current_duty1);  
        }
    }
    if(Target_duty2 - Current_duty2 > 0){
        Current_duty2 = min(Target_duty2, Current_duty2+MAX_ACCELERATION);
        if(Current_duty2>0){
            pwm2_1.write(Current_duty2);
            pwm2_2.write(0);
        }else{
            pwm2_1.write(0);
            pwm2_2.write(-Current_duty2);  
        }
    } else {
        Current_duty2 = max(Target_duty2, Current_duty2-MAX_ACCELERATION);
        if(Current_duty2>0){
            pwm2_1.write(Current_duty2);
            pwm2_2.write(0);
        }else{
            pwm2_1.write(0);
            pwm2_2.write(-Current_duty2);  
        }
    }
}