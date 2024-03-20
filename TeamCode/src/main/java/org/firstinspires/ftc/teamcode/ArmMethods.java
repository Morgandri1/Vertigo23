package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
public class ArmMethods extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor armMotor;
    private Servo angleIntake;
    private Servo wheelIntake;
    private DcMotor linearMotor;
    private int fullRatio=5;

    int offset = 0;

    @Override
    public void runOpMode(){}
    public ArmMethods(HardwareMap hardwareMap){
        armMotor = hardwareMap.get(DcMotor.class,"am1");
        armMotor.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelIntake = hardwareMap.get(Servo.class,"servowheel");
        angleIntake = hardwareMap.get(Servo.class,"servoangle");
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
        //Position 0 is starting position, Position 1 is to intake pixels, Position 2 is to go to the backboard, and position 3 is to put a pixel on the stripe:
        double[][] armPositions = {{0,0.98},{0,0.4},{170,0.95},{20,0.32}};
        for (time = runtime.milliseconds(); runtime.milliseconds()-time<timeToMove;){
            angleIntake.setPosition(armPositions[position][1]);
            setArmDegree((int)armPositions[position][0]);
        }
        armMotor.setPower(0);
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
