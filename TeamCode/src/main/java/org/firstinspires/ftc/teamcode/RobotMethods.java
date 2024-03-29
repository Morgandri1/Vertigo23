//Allows this file to be used by other files, and to use other files:
package org.firstinspires.ftc.teamcode;
//Importing necessary files:
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//This class contains all the methods and hardware variables you will need to use the robot, and more of each can be added as necessary:
public class RobotMethods extends LinearOpMode {
    //Global variable declaration:
    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor leftFrontDrive;
    public DcMotor leftBackDrive;
    public DcMotor rightFrontDrive;
    public DcMotor rightBackDrive;
    public DistanceSensor distanceSensor;
    public DcMotor armMotor;
    public Servo angleIntake;
    public Servo wheelIntake;
    private static final int fullRatio=5;
    static int offset = 0;
    //Required to have this method when extending LinearOpMode:
    @Override
    public void runOpMode(){}
    //Declares all of the motors and servos, and can add more if needed:
    public RobotMethods(HardwareMap hardwareMap){
        leftFrontDrive = hardwareMap.get(DcMotor.class,"FL");
        leftBackDrive = hardwareMap.get(DcMotor.class,"BL");
        rightFrontDrive = hardwareMap.get(DcMotor.class,"FR");
        rightBackDrive = hardwareMap.get(DcMotor.class,"BR");
        distanceSensor = hardwareMap.get(DistanceSensor.class,"DS");
        armMotor = hardwareMap.get(DcMotor.class,"am1");
        armMotor.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelIntake = hardwareMap.get(Servo.class,"servowheel");
        angleIntake = hardwareMap.get(Servo.class,"servoangle");
    }
    //Moves the robot according to given parameters: (Note: the method runs once and sets the power, it does not stop the robot automatically, the user must do that manually by using move(0,0,0) )
    public void move(double axial, double lateral, double yaw){
        double max;
        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;
        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));
        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }
        leftFrontDrive.setPower(leftFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightFrontDrive.setPower(rightFrontPower);
        rightBackDrive.setPower(rightBackPower);
    }
    public double ReturnLF(){return leftFrontDrive.getPower();}
    public double ReturnLB(){return leftBackDrive.getPower();}
    public double ReturnRF(){return rightFrontDrive.getPower();}
    public double ReturnRB(){return rightBackDrive.getPower();}
    //Sets the direction of the 4 wheels to be 'forward':
    public void SetDirectionForward() {
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
    }
    //Sets the direction of the 4 wheels to be 'backwards':
    public void SetDirectionBackwards(){
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
    }
    //Sets the motors to brake when their power is set to zero:
    public void setZeroBehaviorAll() {
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    //This method moves the robot for a set amount of time depending on the call's arguments (Only use for autonomous):
    public void timedMotorMove(double time, double axial, double lateral, double yaw, boolean sensor) {
        double pauseTime = 0;
        for (double startTime = runtime.milliseconds(); runtime.milliseconds() - startTime - pauseTime < time; ) {
            move(axial, lateral, yaw);
            if (sensor) {
                if (distanceSensor.getDistance(DistanceUnit.CM) < 10) {
                    double startPause = runtime.milliseconds();
                    while (distanceSensor.getDistance(DistanceUnit.CM) < 10) {
                        move(0, 0, 0);
                    }
                    pauseTime += runtime.milliseconds() - startPause;
                }
            }
        }
        move(0, 0, 0);
    }
    //Set the position of the arm motor:
    public void setArmDegree(int degree){
        armMotor.setTargetPosition((degree+offset)*fullRatio);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.1);
    }
    //Returns the arm motor's degree with the offset and 'fullRatio' taken into account.
    public int getArmDegree(){
        return (armMotor.getCurrentPosition()/fullRatio)-offset;
    }
    //This method is used to set the arm and intake system's position:
    public void intakeAuto(int position, int timeToMove) {
        //Position 0 is starting position, Position 1 is to intake pixels, Position 2 is to go to the backboard, and position 3 is to put a pixel on the stripe:
        double[][] armPositions = {{0,0.98},{0,0.4},{170,0.95},{20,0.32}};
        for (double time = runtime.milliseconds(); runtime.milliseconds()-time<timeToMove;){
            angleIntake.setPosition(armPositions[position][1]);
            setArmDegree((int)armPositions[position][0]);
        }
        armMotor.setPower(0);
    }
    //Moves the robot for a set amount of time while also having the arm at a set position for the duration of that period (Sensor not yet implemented):
    public void TMM_With_Arm(double time, double axial, double lateral, double yaw, boolean sensor, int position){
        double[][] armPositions = {{0,0.9},{0,0.4},{170,0.95},{20,0.32}};
        for (double startTime = runtime.milliseconds(); runtime.milliseconds()-startTime<time && opModeIsActive();){
            move(axial,lateral,yaw);
            if (position != 1){
                angleIntake.setPosition(armPositions[position][1]);
                setArmDegree((int) armPositions[position][0]);
            }
        }
        move(0,0,0);
    }
}
