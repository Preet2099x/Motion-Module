#include <var.h>
#include <Arduino.h>

void updateEncoder_L() {
  int MSB = digitalRead(encoderPin_1_L);
  int LSB = digitalRead(encoderPin_2_L);

  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded_L << 2) | encoded;

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
    encoderValue_L++;

  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
    encoderValue_L--;

  lastEncoded_L = encoded;
}

void updateEncoder_R() {
  int MSB = digitalRead(encoderPin_1_R);
  int LSB = digitalRead(encoderPin_2_R);

  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded_R << 2) | encoded;

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
    encoderValue_R++;

  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
    encoderValue_R--;

  lastEncoded_R = encoded;
}