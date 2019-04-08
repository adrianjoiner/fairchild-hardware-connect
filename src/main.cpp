#include <Arduino.h>
#include <Wire.h>

void reading_receiver_signals();
void timer_setup();
void handler_channel_1();

//Let's declare some variables so we can use them in the complete program.
uint32_t loop_timer;
int16_t loop_counter;
uint8_t data, start, warning;
int16_t acc_axis[4], gyro_axis[4], temperature;
int32_t gyro_axis_cal[4], acc_axis_cal[4];
int32_t cal_int;
int32_t channel_1_start, channel_1;
int32_t channel_2_start, channel_2;
int32_t channel_3_start, channel_3;
int32_t channel_4_start, channel_4;
int32_t channel_5_start, channel_5;
int32_t channel_6_start, channel_6;
int32_t measured_time, measured_time_start;
uint8_t channel_select_counter;


void setup() {
Serial.begin(57600);              //Set the serial output to 57600 kbps.
  delay(100);                     //Give the serial port some time to start to prevent data loss.
  timer_setup();                  //Setup the timers for the receiver inputs and ESC's output.
  delay(50); 

  // Read and print what'e being recieved from the RX
  reading_receiver_signals();
}

void loop() {
  // put your main code here, to run repeatedly:
}

void reading_receiver_signals(void) {
  Serial.println("Stating to look for rx signal");
  while (data != 'q') {                                                                   //Stay in this loop until the data variable data holds a q.
    delay(250);                                                                           //Print the receiver values on the screen every 250ms
    if (Serial.available() > 0) {                                                         //If serial data is available
      data = Serial.read();                                                               //Read the incomming byte
      delay(100);                                                                         //Wait for any other bytes to come in
      while (Serial.available() > 0)loop_counter = Serial.read();                         //Empty the Serial buffer
    }
    //For starting the motors: throttle low and yaw left (step 1).
    if (channel_3 < 1100 && channel_4 < 1100)start = 1;
    //When yaw stick is back in the center position start the motors (step 2).
    if (start == 1 && channel_3 < 1100 && channel_4 > 1450)start = 2;
    //Stopping the motors: throttle low and yaw right.
    if (start == 2 && channel_3 < 1100 && channel_4 > 1900)start = 0;

    Serial.print("Start:");
    Serial.print(start);

    Serial.print("  Roll:");
    if (channel_1 - 1480 < 0)Serial.print("<<<");
    else if (channel_1 - 1520 > 0)Serial.print(">>>");
    else Serial.print("-+-");
    Serial.print(channel_1);

    Serial.print("  Pitch:");
    if (channel_2 - 1480 < 0)Serial.print("^^^");
    else if (channel_2 - 1520 > 0)Serial.print("vvv");
    else Serial.print("-+-");
    Serial.print(channel_2);

    Serial.print("  Throttle:");
    if (channel_3 - 1480 < 0)Serial.print("vvv");
    else if (channel_3 - 1520 > 0)Serial.print("^^^");
    else Serial.print("-+-");
    Serial.print(channel_3);

    Serial.print("  Yaw:");
    if (channel_4 - 1480 < 0)Serial.print("<<<");
    else if (channel_4 - 1520 > 0)Serial.print(">>>");
    else Serial.print("-+-");
    Serial.print(channel_4);

    Serial.print("  CH5:");
    Serial.print(channel_5);

    Serial.print("  CH6:");
    Serial.println(channel_6);
  }
}


void  handler_channel_1(void) {
    measured_time = TIMER2_BASE->CCR1 - measured_time_start;
    if (measured_time < 0)measured_time += 0xFFFF;
    measured_time_start = TIMER2_BASE->CCR1;
    if (measured_time > 3000)channel_select_counter = 0;
    else channel_select_counter++;

    if (channel_select_counter == 1)channel_1 = measured_time;
    if (channel_select_counter == 2)channel_2 = measured_time;
    if (channel_select_counter == 3)channel_3 = measured_time;
    if (channel_select_counter == 4)channel_4 = measured_time;
    if (channel_select_counter == 5)channel_5 = measured_time;
    if (channel_select_counter == 6)channel_6 = measured_time;
  }

