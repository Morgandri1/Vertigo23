package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp(name="Testing Code", group="Linear OpMode")
public class allTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        //Variable declaration, initialization, and instantiation:
        DcMotor leftFrontDrive;
        DcMotor leftBackDrive;
        DcMotor rightFrontDrive;
        DcMotor rightBackDrive;
        DcMotor armMotor;
        Servo wheelIntake;
        Servo angleIntake;
        DistanceSensor distanceSensor;
        leftFrontDrive = hardwareMap.get(DcMotor.class,"FL");
        leftBackDrive = hardwareMap.get(DcMotor.class,"BL");
        rightFrontDrive = hardwareMap.get(DcMotor.class,"FR");
        rightBackDrive = hardwareMap.get(DcMotor.class,"BR");
        armMotor = hardwareMap.get(DcMotor.class,"am1");
        wheelIntake = hardwareMap.get(Servo.class,"servowheel");
        angleIntake = hardwareMap.get(Servo.class,"servoangle");
        distanceSensor = hardwareMap.get(DistanceSensor.class,"DS");
        ElapsedTime runtime = new ElapsedTime();
        ArmMethods AMO = new ArmMethods(armMotor,angleIntake,wheelIntake);
        MotorMethods MMO = new MotorMethods(leftFrontDrive,rightFrontDrive,leftBackDrive,rightBackDrive,distanceSensor);
        MMO.SetDirectionBackwards();
        MMO.setZeroBehaviorAll(DcMotor.ZeroPowerBehavior.BRAKE);
        double axial = 0.0;
        double lateral = 0.0;
        double yaw = 0.0;
        double driveMultiplier = 0.5;
        boolean active = true;
        int changePos = -1;
        double stickOffset = 0.0;
        double continuousOffset = 0.0;
        double currentPos = 0.0;
        double motorOffset = 0.0;
        //Repeatedly runs after the play button is pressed:
        waitForStart();
        runtime.reset();
        while(opModeIsActive()) {
            //Pressing start on either of the controllers will stop the code:
            if (gamepad1.back || gamepad2.back){terminateOpModeNow();}
            //Pressing start will stop the robot and cease any other functions:
            while(gamepad1.start || gamepad2.start) {
                active = false;
                MMO.move(0,0,0);
            }
            if (gamepad2.dpad_left){changePos = -1;}
            //Full Back
            if (gamepad2.a) {
                changePos = 0;
            }
            //Full forward to intake pixels
            if (gamepad2.b) {
                changePos = 1;
            }
            //Deposit pixels on Backboard
            if (gamepad2.x || gamepad2.dpad_up) {
                changePos = 2;
            }
            //
            if (gamepad2.y) {
                changePos = 3;
            }
            if (gamepad1.a) {
                MMO.move(0,0,0);
                leftFrontDrive.setPower(0.5);
            }
            if (gamepad1.b) {
                MMO.move(0,0,0);
                leftBackDrive.setPower(0.5);
            }
            if (gamepad1.x) {
                MMO.move(0,0,0);
                rightFrontDrive.setPower(0.5);
            }
            if (gamepad1.y) {
                MMO.move(0,0,0);
                rightBackDrive.setPower(0.5);
            }
            if (gamepad1.dpad_down){MMO.timedMotorMove(1000,1,0,0,false);}
            if (gamepad1.dpad_up){MMO.timedMotorMove(1000,1,0,0,true);}
            if (gamepad2.right_bumper){
                for(double time = runtime.milliseconds();runtime.milliseconds()-time<2000 && !(gamepad1.start || gamepad2.start);){wheelIntake.setPosition(0.9);}
            }
            else if (gamepad2.left_bumper){
                for(double time = runtime.milliseconds();runtime.milliseconds()-time<2000 && !(gamepad1.start || gamepad2.start);){wheelIntake.setPosition(0.1);}
            }
            if (gamepad2.right_trigger > 0.2f){motorOffset -= 0.1;}
            if (gamepad2.left_trigger > 0.2f){motorOffset += 0.1;}
            axial = driveMultiplier * (-gamepad1.left_stick_y);  // Note: pushing stick forward gives negative value
            lateral = driveMultiplier * (gamepad1.left_stick_x);
            yaw = driveMultiplier * (-gamepad1.right_stick_x);
            if(!(gamepad1.a||gamepad1.b||gamepad1.x||gamepad1.y)){MMO.move(axial,lateral,yaw);}
            if (gamepad2.left_stick_y > 0.2f){stickOffset = 0.1;}
            else if (gamepad2.left_stick_y < -0.2f){stickOffset = -0.1;}
            else{stickOffset = 0.0;}
            if(gamepad2.right_stick_y < -0.2f){continuousOffset -= 0.003;}
            else if (gamepad2.right_stick_y > 0.2f){continuousOffset += 0.003;}
            if(gamepad2.left_bumper){
                if(changePos <= 2){changePos++;}
                else{changePos = 0;}
            }
            if (changePos == 0){
                AMO.setArmDegree(252 + (int)Math.floor(motorOffset));
                currentPos = 0.03+stickOffset+continuousOffset;
                angleIntake.setPosition(0.03+stickOffset + continuousOffset);
            }
            else if (changePos == 1){
                AMO.setArmDegree(0 + (int)Math.floor(motorOffset));
                currentPos = 0.1+stickOffset+continuousOffset;
                angleIntake.setPosition(0.1+stickOffset + continuousOffset);
            }
            else if (changePos == 2){
                AMO.setArmDegree(170 + (int)Math.floor(motorOffset));
                currentPos = 0.15+stickOffset+continuousOffset;
                angleIntake.setPosition(0.15+stickOffset + continuousOffset);
            }
            else if (changePos == 3){
                AMO.setArmDegree(15 + (int)Math.floor(motorOffset));
                currentPos = 0.15+stickOffset+continuousOffset;
                angleIntake.setPosition(0.15+stickOffset + continuousOffset);
            }
            driveMultiplier = 0.5;
            if (gamepad1.left_trigger > 0.1f){driveMultiplier = driveMultiplier - 0.25;}
            if (gamepad1.right_trigger > 0.1f){driveMultiplier = driveMultiplier + 0.25;}
            telemetry.addData("Current arm motor position: ", AMO.getArmDegree());
            telemetry.addData("Current arm servo position: ", angleIntake.getPosition());
            telemetry.addData("Current arm servo position with var: ", currentPos);
            telemetry.addData("Current wheel position: ", wheelIntake.getPosition());
            telemetry.addData("Current centimeters from distance sensor: ", distanceSensor.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
}
