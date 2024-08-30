/*
 * motor.h
 *
 * Created on: 2024
 * Description:
 * 振动马达相关的函数定义
 * Copyright (c) 2024 AgileX Robotics
 */
 
#define motor_pin   44

//初始化震动马达
void init_motor()
{
  pinMode(motor_pin,OUTPUT);
}

//让马达短时间轻微震动
void motor_start_short()
{
  digitalWrite(motor_pin, HIGH);
  delay(60);
  digitalWrite(motor_pin, LOW);
}

void motor_start_short_twice()
{
  digitalWrite(motor_pin, HIGH);
  delay(50);
  digitalWrite(motor_pin, LOW);
  delay(80);
  digitalWrite(motor_pin, HIGH);
  delay(50);
  digitalWrite(motor_pin, LOW);
}

//马达长时间震动
void motor_start_long()
{
  digitalWrite(motor_pin, HIGH);
  delay(120);
  digitalWrite(motor_pin, LOW);
}