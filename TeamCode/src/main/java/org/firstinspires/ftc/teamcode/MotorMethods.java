package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.HardwareMap;
class MotorMethods{
    private ElapsedTime runtime = new ElapsedTime();
    //Instance variables that will be defined once another file calls this file:
    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;
    private DistanceSensor distanceSensor;
    private HardwareMap hardwareMap;
    //Constructor to set the value of the instance variables when an object is made:
    public MotorMethods(HardwareMap hardwareMapObj){
        hardwareMap = hardwareMapObj;
        leftFrontDrive = hardwareMap.get(DcMotor.class,"FL");
        leftBackDrive = hardwareMap.get(DcMotor.class,"BL");
        rightFrontDrive = hardwareMap.get(DcMotor.class,"FR");
        rightBackDrive = hardwareMap.get(DcMotor.class,"BR");
        distanceSensor = hardwareMap.get(DistanceSensor.class,"DS");
    }
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
    public void SetDirectionForward(){
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
    }
    public void SetDirectionBackwards(){
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
    }
    public void setZeroBehaviorAll(DcMotor.ZeroPowerBehavior thing) {
        leftFrontDrive.setZeroPowerBehavior(thing);
        leftBackDrive.setZeroPowerBehavior(thing);
        rightFrontDrive.setZeroPowerBehavior(thing);
        rightBackDrive.setZeroPowerBehavior(thing);
    }

    //This method moves the robot for a set amount of time depending on the calls' arguments:
    public void timedMotorMove(double time, double axial, double lateral, double yaw, boolean sensor) {
        double pauseTime = 0;
        for (double startTime = runtime.milliseconds(); runtime.milliseconds() - startTime-pauseTime < time; ) {
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
        move(0,0,0);
    }
    //timedMotorMove(1000,1.0,0,0);
}

