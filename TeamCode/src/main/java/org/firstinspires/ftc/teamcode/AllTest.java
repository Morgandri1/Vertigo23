package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp(name="Testing Code", group="Linear OpMode")
public class AllTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        //Variable declaration, initialization, and instantiation:
        DcMotor armMotor;
        Servo wheelIntake;
        Servo angleIntake;
        DistanceSensor distanceSensor;
        /* Not necessary because the MotorMethods file does/can do all of the motor-related tasks:
        DcMotor leftFrontDrive;
        DcMotor leftBackDrive;
        DcMotor rightFrontDrive;
        DcMotor rightBackDrive;
        leftFrontDrive = hardwareMap.get(DcMotor.class,"FL");
        leftBackDrive = hardwareMap.get(DcMotor.class,"BL");
        rightFrontDrive = hardwareMap.get(DcMotor.class,"FR");
        rightBackDrive = hardwareMap.get(DcMotor.class,"BR");
         */
        //Also not needed due to the ArmMethods file doing any arm motor related tasks:
        //armMotor = hardwareMap.get(DcMotor.class,"am1");
        wheelIntake = hardwareMap.get(Servo.class,"servowheel");
        angleIntake = hardwareMap.get(Servo.class,"servoangle");
        distanceSensor = hardwareMap.get(DistanceSensor.class,"DS");
        ElapsedTime runtime = new ElapsedTime();
        ArmMethods AMO = new ArmMethods(hardwareMap);
        MotorMethods MMO = new MotorMethods(hardwareMap);
        MMO.SetDirectionBackwards();
        MMO.setZeroBehaviorAll(DcMotor.ZeroPowerBehavior.BRAKE);
        double axial = 0.0;
        double lateral = 0.0;
        double yaw = 0.0;
        double driveMultiplier = 0.5;
        int changePos = -1;
        double stickOffset = 0.0;
        double continuousOffset = 0.0;
        double currentPos = 0.0;
        double motorOffset = 0.0;
        final double startingDegree = 0.8;
        //Array for armmotor and angleIntake for positions 0-3:
        double[][] armPositions = {{0,startingDegree},{0,0.4},{170,0.96},{25,0.15}};
        final int SET_ARM_MOTOR = 0;
        final int SET_ARM_SERVO = 1;
        waitForStart();
        runtime.reset();
        //Repeatedly runs after the play button is pressed:
        while(opModeIsActive()) {
            //Pressing start on either of the controllers will stop the code:
            if (gamepad1.back || gamepad2.back){terminateOpModeNow();}
            //Pressing start will stop the robot and cease any other functions:
            while(gamepad1.start || gamepad2.start) {
                MMO.move(0,0,0);
            }
            if (gamepad2.dpad_right){
                motorOffset = 0;
                continuousOffset = 0;
            }
            if (gamepad2.dpad_left){changePos = -1;}
            //Full Back (To starting position):
            else if (gamepad2.a) {
                changePos = 0;
            }
            //Full forward to intake pixels:
            else if (gamepad2.b || gamepad2.dpad_down) {
                changePos = 1;
            }
            //Deposit pixels on Backboard:
            else if (gamepad2.x || gamepad2.dpad_up) {
                changePos = 2;
            }
            //Deposit pixels on stripe:
            else if (gamepad2.y) {
                changePos = 3;
            }
            if (gamepad2.right_bumper){wheelIntake.setPosition(0.9);}
            else if (gamepad2.left_bumper){wheelIntake.setPosition(0.1);}
            else{wheelIntake.setPosition(0.5);}
            if (gamepad2.right_trigger > 0.2f){motorOffset -= 0.5;}
            if (gamepad2.left_trigger > 0.2f){motorOffset += 0.5;}
            axial = driveMultiplier * (-gamepad1.left_stick_y);  // Note: pushing stick forward gives negative value
            lateral = driveMultiplier * (gamepad1.left_stick_x);
            yaw = driveMultiplier * (-gamepad1.right_stick_x);
            if (gamepad2.left_stick_y > 0.2f){stickOffset = 0.1;}
            else if (gamepad2.left_stick_y < -0.2f){stickOffset = -0.1;}
            else{stickOffset = 0.0;}
            if(gamepad2.right_stick_y < -0.2f){continuousOffset -= 0.003;}
            else if (gamepad2.right_stick_y > 0.2f){continuousOffset += 0.003;}
            //SET_ARM_MOTOR and SET_ARM_SERVO are constant variables set at the beginning of runOpMode()
            //Position -1 will make the arm go limp
            if (changePos != -1) {
                angleIntake.setPosition(armPositions[changePos][SET_ARM_SERVO] + stickOffset + continuousOffset);
                AMO.setArmDegree((int) armPositions[changePos][SET_ARM_MOTOR] + (int) Math.floor(motorOffset));
                currentPos = armPositions[changePos][SET_ARM_SERVO] + stickOffset + continuousOffset;
            }
            if (gamepad1.left_trigger > 0.1f){driveMultiplier = 0.25;}
            else if (gamepad1.right_trigger > 0.1f){driveMultiplier = 1;}
            else{driveMultiplier = 0.5;}
            MMO.move(axial,lateral,yaw);
            telemetry.addData("Current arm motor position: ", AMO.getArmDegree());
            telemetry.addData("Current arm servo position: ", angleIntake.getPosition());
            telemetry.addData("Current arm servo position with var: ", currentPos);
            telemetry.addData("Current wheel position: ", wheelIntake.getPosition());
            telemetry.addData("Current centimeters from distance sensor: ", distanceSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("Current continuous offset: ", continuousOffset);
            telemetry.addData("Current stick offset", stickOffset);
            telemetry.addData("Current moto offset", motorOffset);
            telemetry.update();
        }
    }
}
