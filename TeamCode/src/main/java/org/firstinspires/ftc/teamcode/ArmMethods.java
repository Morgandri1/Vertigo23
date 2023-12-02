package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class ArmMethods {
    private DcMotor armMotor;
    private Servo angleIntake;
    private Servo wheelIntake;
    private int fullRatio=5;

    int offset =-250;

    public ArmMethods(DcMotor mainArmMotor, Servo angle, Servo wheel){
        armMotor=mainArmMotor;
        armMotor.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        angleIntake=angle;
        wheelIntake=wheel;
    }
    public double armMotorMove(float leftStickY, int speedDecrease){
        double power = 0.0;
        int motorSpeed = speedDecrease;
        if ((double)leftStickY > 1.0){
            power = 1.0/motorSpeed;
        }
        else if ((double)leftStickY < -1.0){
            power = -1.0/motorSpeed;
        }
        else{
            power = (double)leftStickY/motorSpeed;
        }
        return power;
    }
    public void setArmDegree(int degree){
        armMotor.setTargetPosition((degree+offset)*fullRatio);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.1);
    }
    public int getArmDegree(){
        return (armMotor.getCurrentPosition()/fullRatio)-offset;
    }

}
