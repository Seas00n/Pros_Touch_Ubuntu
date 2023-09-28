#include <Arduino.h>
#include <HardwareSerial.h>
#include "BluetoothSerial.h"

#define LED_PIN 2
#define RX_2 16
#define TX_2 17
BluetoothSerial SerialBT;


void setup(){
    pinMode(LED_PIN, OUTPUT);
    Serial.begin(115200);
    Serial2.begin(115200,SERIAL_8N1, RX_2, TX_2);
    SerialBT.begin("ESP32test");
}



#define Buf_Max 80
#define samplingtime 50
char Rec_Buf[Buf_Max] = {0};
char c;
int rx_count = 0;
static unsigned long samplingTime = millis();

float p_pressure[18] = {0};
float init_p_pressure[18] = {0};
int init_count = 0;
const float p_pos[18][2] = {
    {-4.2,18.5},{-4.6,22.1},{-1.3,3.1},{-1.3,6.7},{-2,10.6},{-2.8,14.6},
    {-2.2,18.5},{-3,22.9},{1.1,3.1},{1.2,6.7},{-0.2,10.6},{2,14.6},
    {-0.3,18.5},{-1.5,22.9},{1.5,10.6},{2,14.6},{1.4,18.5},{0.2,21.7}
};
const uint8_t area1_idx[4] = {2,3,8,9};
const uint8_t area2_idx[6]= {4,5,10,11,14,15};
const uint8_t area3_idx[8] = {0,1,6,7,12,13,16,17}; 
float force_data[9] = {0};
uint16_t force_data_int[9] = {0}; 
uint8_t force_msg[19] = {0};

int k_float2int = 10;
int b_float2int = 30000;
bool is_init_over = false;


void Receive_handler(void);
void Calc_COM(void);
void Pack_Msg(void);
void Clear_Buffer(void);



void loop(){
    digitalWrite(LED_PIN, LOW);
    while(Serial2.available()){
        c = Serial2.read();
        Rec_Buf[rx_count]=c;
        rx_count++; 
    }
    if(millis()-samplingTime>samplingtime){
        Receive_handler();
        Pack_Msg();
        Clear_Buffer();
        // for(int i=0;i<18;i++){
        //     Serial.printf("[%d]:%.2f",i,p_pressure[i]);
        // }
        // for(int i=0;i<9;i++){
        //     Serial.printf("force_data[%d]:%d",i,force_data[i]);
        // }
        // Serial.println();
        
        if(is_init_over)
            Serial.write(force_msg, sizeof(force_msg));
        samplingTime = millis();
    }
    // if(SerialBT.available()){
    //     SerialBT.write(force_msg,sizeof(force_msg));
    // }else{
    //     Serial.println("BT Not available");
    // }
    delay(50);
    digitalWrite(LED_PIN, HIGH);
}
void Receive_handler(){
    int i = 0;
    if(Rec_Buf[0]==0xAA && Rec_Buf[1]==0x02){
        p_pressure[0]=Rec_Buf[2]*256+Rec_Buf[3];
        p_pressure[1]=Rec_Buf[4]*256+Rec_Buf[5];
        p_pressure[2]=Rec_Buf[6]*256+Rec_Buf[7];
        p_pressure[3]=Rec_Buf[8]*256+Rec_Buf[9];
        p_pressure[4]=Rec_Buf[10]*256+Rec_Buf[11];
        p_pressure[5]=Rec_Buf[12]*256+Rec_Buf[13];
        p_pressure[6]=Rec_Buf[14]*256+Rec_Buf[15];
        p_pressure[7]=Rec_Buf[16]*256+Rec_Buf[17];
        p_pressure[8]=Rec_Buf[18]*256+Rec_Buf[19]; 
        p_pressure[9]=Rec_Buf[20]*256+Rec_Buf[21];
        p_pressure[10]=Rec_Buf[22]*256+Rec_Buf[23];
        p_pressure[11]=Rec_Buf[24]*256+Rec_Buf[25];
        p_pressure[12]=Rec_Buf[26]*256+Rec_Buf[27];
        p_pressure[13]=Rec_Buf[28]*256+Rec_Buf[29];
        p_pressure[14]=Rec_Buf[30]*256+Rec_Buf[31];
        p_pressure[15]=Rec_Buf[32]*256+Rec_Buf[33];   
        p_pressure[16]=Rec_Buf[34]*256+Rec_Buf[35];
        p_pressure[17]=Rec_Buf[36]*256+Rec_Buf[37]; 
    }else{}

    if(init_count<30){
        for(i=0;i<18;i++){
            init_p_pressure[i] += p_pressure[i];
        }
        init_count+=1;
    }else if(init_count==30){
        for(i=0;i<18;i++){
            init_p_pressure[i] = init_p_pressure[i]/30;
            init_count++;
        }
        is_init_over = true;
        Serial.printf("init_p_pressure_over:%.2f",init_p_pressure[0]);
    }else{
        for(i=0;i<18;i++){
            p_pressure[i]-=init_p_pressure[i];
        }
        Calc_COM();
    }
}

void Calc_COM(){
    int i=0;
    int j=0;
    float temp_f = 0;
    float temp_px = 0;
    float temp_py = 0;
    float center_x_temp = 0;
    float center_y_temp = 0;
    for(i=0;i<4;i++){
        j = area1_idx[i];
        temp_f+= p_pressure[j];
        temp_px+=p_pressure[j]*p_pos[j][0];
        temp_py+=p_pressure[j]*p_pos[j][1];
        center_x_temp += p_pos[j][0];
        center_y_temp += p_pos[j][1];
    }
    force_data[0] = temp_f/100.0;
    if(temp_f<0.01){
        force_data[1] = center_x_temp/4;
        force_data[2] = center_y_temp/4;
    }
    else{
        force_data[1] = temp_px/temp_f;
        force_data[2] = temp_py/temp_f;
    }
    // Serial.printf("temp_f:%.3f,temp_px:%.3f,temp_py:%.3f",force_data[0],force_data[1],force_data[2]);
    
    
    temp_f = 0;
    temp_px = 0;
    temp_py = 0;
    center_x_temp = 0;
    center_y_temp = 0;
    for(i=0;i<6;i++){
        j = area2_idx[i];
        temp_f+= p_pressure[j];
        temp_px+=p_pressure[j]*p_pos[j][0];
        temp_py+=p_pressure[j]*p_pos[j][1];
        center_x_temp += p_pos[j][0];
        center_y_temp += p_pos[j][1];
    }
    force_data[3] = temp_f/100.0;
    if(temp_f<0.01){
        force_data[4] = center_x_temp/6;
        force_data[5] = center_y_temp/6;
    }
    else{
        force_data[4] = temp_px/temp_f;
        force_data[5] = temp_py/temp_f;
    }


    temp_f = 0;
    temp_px = 0;
    temp_py = 0;
    center_x_temp = 0;
    center_y_temp = 0;
    for(i=0;i<8;i++){
        j = area3_idx[i];
        temp_f+= p_pressure[j];
        temp_px+=p_pressure[j]*p_pos[j][0];
        temp_py+=p_pressure[j]*p_pos[j][1];
        center_x_temp += p_pos[j][0];
        center_y_temp += p_pos[j][1];
    }
    
    force_data[6] = temp_f/100.0;
    if(temp_f<0.01){
        force_data[7] = center_x_temp/8;
        force_data[8] = center_y_temp/8;
    }
    else{
        force_data[7] = temp_px/temp_f;
        force_data[8] = temp_py/temp_f;
    }
    
}
void Pack_Msg(){
    for (int i=0;i<9;i++){
        force_data_int[i] = (uint16_t)(k_float2int*force_data[i]+b_float2int);
    }
    force_msg[0] = 0xAA;
    memcpy(force_msg+1, force_data_int, sizeof(force_msg)-1);
}

void Clear_Buffer(){
  for(int i=0;i<sizeof(Rec_Buf);i++){
        Rec_Buf[i] = 0;
  }
  rx_count = 0;
}