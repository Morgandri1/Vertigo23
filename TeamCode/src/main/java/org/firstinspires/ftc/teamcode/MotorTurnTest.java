package org.firstinspires.ftc.teamcode;

public class MotorTurnTest {
    public static void main(String[] args){
    MotorMethods MethodObj = new MotorMethods();
    MethodObj.Turn(1.0);
    double LB = MethodObj.ReturnLB();
    System.out.println(LB);
}}
