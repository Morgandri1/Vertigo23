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
    private DcMotor linearMotor;
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
    //Moves the linear slide to different positions depending on the state specified:
    public void linearMove(int positionState){
        if (positionState == 0){
            for (double time = runtime.milliseconds();runtime.milliseconds()-time<5000;){
                linearMotor.setPower(-0.5);
            }
            linearMotor.setPower(0);
        }
        else if (positionState == 1){
            for (double time = runtime.milliseconds();runtime.milliseconds()-time<5000;){
                linearMotor.setPower(0.5);
            }
            linearMotor.setPower(0);
        }
        else if (positionState == 2){
            //Modify time and power to move to correct distance for autonomous:
            for (double time = runtime.milliseconds();runtime.milliseconds()-time<5000;){
                //Add code here
            }
            linearMotor.setPower(0);
        }
    }
    //Brakes the arm-motors whenever they have 0 power:
    public void setZeroBehaviorAll(){
        DcMotor.ZeroPowerBehavior brake = DcMotor.ZeroPowerBehavior.BRAKE;
        armMotor.setZeroPowerBehavior(brake);
        linearMotor.setZeroPowerBehavior(brake);
    }
}
