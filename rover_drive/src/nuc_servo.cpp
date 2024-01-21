#include <string>
#include <iostream>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#define MAX_BUFF_LEN   6
#define SERVOMIN  115 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  485 // This is the 'maximum' pulse length count (out of 4096)
#define SERVOC_1  3072 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOAC_1  1024 // This is the 'maximum' pulse length count (out of 4096)
#define SERVOSTOP_1  2048 
#define USMIN  500 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 15018
#define USMAX  2500 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
char c;
uint8_t servonum = 0;
int pulselen_0=0;
int pulselen_1=0;
int main_command=0;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
char str[MAX_BUFF_LEN];
uint8_t idx=0;
//#define TXD_PIN (GPIO_NUM_17)
//#define RXD_PIN (GPIO_NUM_16)

//#define UART UART_NUM_2
//static const int RX_BUF_SIZE = 1024;
//int num = 0;
const int gripper_Pin = 32;
const int gripper_rotation_Pin=33;  // Define the PWM output pin
int dutycycle_gripper=11.5; 
int dutycycle_gripper_rotation=7;    // Variable to store the received duty cycle value

void setup() {
  Serial.begin(115200);


  pwm.begin();
  /*
   * In theory the internal oscillator (clock) is 25MHz but it really isn't
   * that precise. You can 'calibrate' this by tweaking this number until
   * you get the PWM update frequency you're expecting!
   * The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
   * is used for calculating things like writeMicroseconds()
   * Analog servos run at ~50 Hz updates, It is importaint to use an
   * oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
   * 1) Attach the oscilloscope to one of the PWM signal pins and ground on
   *    the I2C PCA9685 chip you are setting the value for.
   * 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
   *    expected value (50Hz for most ESCs)
   * Setting the value here is specific to each individual I2C PCA9685 chip and
   * affects the calculations for the PWM update frequency. 
   * Failure to correctly set the int.osc value will cause unexpected PWM results
   */
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(10); 
  pwm.setPWM(0, 0, SERVOMIN);
  pwm.setPWM(1, 0, 2048);// Start serial communication
  
//
//  const uart_config_t uart_config = {
//        .baud_rate = 115200,
//        .data_bits = UART_DATA_8_BITS,
//        .parity = UART_PARITY_DISABLE,
//        .stop_bits = UART_STOP_BITS_1,
//        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
//        .source_clk = UART_SCLK_APB,
//};
//
//uart_driver_install(UART, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
//    uart_param_config(UART, &uart_config);
//    uart_set_pin(UART, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

}


void loop() {
  if (Serial.available()>0){ 
    c=Serial.read() ;    //Read one byte
    if(c!='\n'){ //Still Reading
      str[idx++]=c;
    }
    else{
      str[idx]='\0';//Convert into a string
      idx=0;
    }
    int final_command = atoi(str);
    main_command=final_command;
//    if ((main_command==22) && (pulselen<=SERVOMAX)){
//    pulselen=pulselen+5;
//    }else if((main_command==1) && (pulselen>=SERVOMIN)){
//      pulselen=pulselen-5;
//    }

    switch(main_command){
        case 22:
            if(pulselen_0<=SERVOMAX){
                pulselen_0=pulselen_0+5;}
            pulselen_1=SERVOC_1;
        break;
        case 21:
            if(pulselen_0<=SERVOMAX){
                pulselen_0=pulselen_0+5;}
            pulselen_1=SERVOAC_1;
        break;
        case 20:
            if(pulselen_0<=SERVOMAX){
                pulselen_0=pulselen_0+5;}
            pulselen_1=SERVOSTOP_1;
        break;
        case 12:
            if(pulselen_0>=SERVOMIN){
                pulselen_0=pulselen_0-5;}
            pulselen_1=SERVOC_1;
        break;
        case 11:
            if(pulselen_0>=SERVOMIN){
                pulselen_0=pulselen_0-5;}
            pulselen_1=SERVOAC_1;
        break;
        case 10:
            if(pulselen_0>=SERVOMIN){
                pulselen_0=pulselen_0-5;}
            pulselen_1=SERVOSTOP_1;
        break;
        case 2:
            
            pulselen_1=SERVOC_1;
              pulselen_1=pulselen_1+5;
        break;
        case 1:
            
            pulselen_1=SERVOAC_1;
              pulselen_1=pulselen_1-5;
        break;
        case 0:
           
            pulselen_1=SERVOSTOP_1;
        break;


      
    
  
    
    pwm.setPWM(0, 0, main_command);
    pwm.setPWM(1, 0, pulselen_1);
    
  }
//  pwm.setPWM(0, 0, main_command);
    
  }