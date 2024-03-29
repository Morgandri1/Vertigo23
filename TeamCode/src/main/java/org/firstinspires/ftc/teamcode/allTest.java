//Allows this file to use other files:
package org.firstinspires.ftc.teamcode;
//Imports necessary files:
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//This is currently the main TeleOp code for Vertigo Robotics Team:
@TeleOp(name="Testing Code", group="Linear OpMode")
public class allTest extends LinearOpMode {
    //Code that will run when the user presses 'INIT':
    @Override
    public void runOpMode() {
        //Variable declaration, initialization, and instantiation:
        ElapsedTime runtime = new ElapsedTime();
        RobotMethods RMO = new RobotMethods(hardwareMap);
        //Sets the robot's wheels to go 'backwards', and for the robot's motors to brake when at 0 power:
        RMO.SetDirectionBackwards();
        RMO.setZeroBehaviorAll();
        //Variables to move the robot:
        double axial = 0.0;
        double lateral = 0.0;
        double yaw = 0.0;
        //Variable to adjust the speed of the robot:
        double driveMultiplier = 0.5;
        //Arm position:
        int changePos = -1;
        //Manual arm motor and intake system offset:
        double stickOffset = 0.0;
        double continuousOffset = 0.0;
        double motorOffset = 0.0;
        //Current position of the intake system:
        double currentPos = 0.0;
        //Starting degree of the intake system (not really needed tbh):
        final double startingDegree = 0.8;
        //Array to keep track of degrees for armMotor and angleIntake positions 0-3:
        double[][] armPositions = {{0,startingDegree},{0,0.4},{170,0.96},{25,0.15}};
        //Variables to keep track of which position in the armPositions array correspond to the arm motor or intake system:
        final int SET_ARM_MOTOR = 0;
        final int SET_ARM_SERVO = 1;
        //Waits for the play button to be pressed:
        waitForStart();
        runtime.reset();
        //Repeatedly runs after the play button is pressed:
        while(opModeIsActive()) {
            //Pressing start on either of the controllers will stop the code:
            if (gamepad1.back || gamepad2.back){terminateOpModeNow();}
            //Pressing start will stop the robot and cease any other functions:
            while(gamepad1.start || gamepad2.start) {
                RMO.move(0,0,0);
            }
            //Resets any manual change to the arm position:
            if (gamepad2.dpad_right){
                motorOffset = 0;
                continuousOffset = 0;
            }
            //Makes the arm go limp:
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
            //Intake system's intake and outtake:
            if (gamepad2.right_bumper){RMO.wheelIntake.setPosition(0.9);}
            else if (gamepad2.left_bumper){RMO.wheelIntake.setPosition(0.1);}
            else{RMO.wheelIntake.setPosition(0.5);}
            //Manually changes the position of the arm motor:
            if (gamepad2.right_trigger > 0.2f){motorOffset -= 0.5;}
            if (gamepad2.left_trigger > 0.2f){motorOffset += 0.5;}
            //Changes where the robot will go according to the stick directions and the speed setting:
            axial = driveMultiplier * (-gamepad1.left_stick_y);  // Note: pushing stick forward gives negative value
            lateral = driveMultiplier * (gamepad1.left_stick_x);
            yaw = driveMultiplier * (-gamepad1.right_stick_x);
            //Moves intake system up/down by 1/10 of a rotation depending on stick direction, and will go back to the specified position once the stick is back to it's default position:
            if (gamepad2.left_stick_y > 0.2f){stickOffset = 0.1;}
            else if (gamepad2.left_stick_y < -0.2f){stickOffset = -0.1;}
            else{stickOffset = 0.0;}
            //Moves the intake system up/down, and will not go back to the specified position once the stick has returned to it's default position
            if(gamepad2.right_stick_y < -0.2f){continuousOffset -= 0.003;}
            else if (gamepad2.right_stick_y > 0.2f){continuousOffset += 0.003;}
            //SET_ARM_MOTOR and SET_ARM_SERVO are constant variables set at the beginning of runOpMode()
            //Implements changes to the arm from the other parts of the code:
            if (changePos != -1) {
                RMO.angleIntake.setPosition(armPositions[changePos][SET_ARM_SERVO] + stickOffset + continuousOffset);
                RMO.setArmDegree((int) armPositions[changePos][SET_ARM_MOTOR] + (int) Math.floor(motorOffset));
                currentPos = armPositions[changePos][SET_ARM_SERVO] + stickOffset + continuousOffset;
            }
            //Determines speed setting:
            if (gamepad1.left_trigger > 0.1f){driveMultiplier = 0.25;}
            else if (gamepad1.right_trigger > 0.1f){driveMultiplier = 1;}
            else{driveMultiplier = 0.5;}
            //Implements the changes to the robot's position from other parts of the code:
            RMO.move(axial,lateral,yaw);
            //Sends data back to the driver's station:
            telemetry.addData("Current arm motor position: ", RMO.getArmDegree());
            telemetry.addData("Current arm servo position: ", RMO.angleIntake.getPosition());
            telemetry.addData("Current arm servo position with var: ", currentPos);
            telemetry.addData("Current wheel position: ", RMO.wheelIntake.getPosition());
            telemetry.addData("Current centimeters from distance sensor: ", RMO.distanceSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("Current continuous offset: ", continuousOffset);
            telemetry.addData("Current stick offset", stickOffset);
            telemetry.addData("Current moto offset", motorOffset);
            telemetry.update();
        }
    }
}