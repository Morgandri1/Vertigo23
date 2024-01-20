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
    private int fullRatio=5;
    private HardwareMap hardwareMap;
    int offset = 0;

    @Override
    public void runOpMode(){}
    public ArmMethods(HardwareMap hardwareMapObj){
        hardwareMap = hardwareMapObj;
        armMotor = hardwareMap.get(DcMotor.class,"am1");
        armMotor.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        angleIntake = hardwareMap.get(Servo.class,"servoangle");
        wheelIntake = hardwareMap.get(Servo.class,"servowheel");
    }
    public void setArmDegree(int degree){
        armMotor.setTargetPosition((degree+offset)*fullRatio);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.1);
    }
    public int getArmDegree(){
        return (armMotor.getCurrentPosition()/fullRatio)-offset;
    }

    //This method is used to control the arm and intake system's position in the autonomous code(s) only:
    public void intakeAuto(int position, int timeToMove) {
        //Position 0 is starting position, Position 1 is to intake pixels, Position 2 is to go to the backboard, and position 3 is to put a pixel on the stripe:
        double[][] armPositions = {{0,0.98},{0,0.4},{170,0.95},{20,0.32}};
        //Other functions will cease during the usage of this loop/task:
        for (time = runtime.milliseconds(); runtime.milliseconds()-time<timeToMove;){
            angleIntake.setPosition(armPositions[position][1]);
            setArmDegree((int)armPositions[position][0]);
        }
        armMotor.setPower(0);
    }
}
