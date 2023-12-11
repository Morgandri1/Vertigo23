package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
public class ArmMethods extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor armMotor;
    private Servo angleIntake;
    private Servo wheelIntake;
    private int fullRatio=5;

    int offset =-252;

    @Override
    public void runOpMode(){}
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

    //This method is used to control the arm and intake system's position:
    public void intakeAuto(int position, int timeToMove) {
        //This method has not been tested, please correct the method if needed.

        //Full forward (Position to intake pixels from ground):
        if (position == 1){
            for(double time = runtime.milliseconds(); runtime.milliseconds()-time<timeToMove;) {
                setArmDegree(15);
                angleIntake.setPosition(0.8);
                sleep(2);
            }
        }
        //Full back (Standard/default position):
        else if (position == 0){
            for(double time = runtime.milliseconds(); runtime.milliseconds()-time<timeToMove;) {
                setArmDegree(252);
                angleIntake.setPosition(0.03);
                sleep(2);
            }
        }
        //Upright Position:
        else if (position == 2 || position == 100){
            for(double time = runtime.milliseconds(); runtime.milliseconds()-time<timeToMove;) {
                setArmDegree(100);
                angleIntake.setPosition(100-Math.round(getArmDegree()));
                sleep(2);
            }
        }
    }
}
