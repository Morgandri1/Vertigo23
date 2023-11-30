package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class ArmMethods {
    private DcMotor armMotor;

    private int fullRatio;
    private int offset;
    public ArmMethods(DcMotor mainArmMotor){
        armMotor=mainArmMotor;
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        armMotor.setTargetPosition((degree)*fullRatio);
    }
    public int getArmDegree(){
        return (armMotor.getTargetPosition()/fullRatio);
    }

}
