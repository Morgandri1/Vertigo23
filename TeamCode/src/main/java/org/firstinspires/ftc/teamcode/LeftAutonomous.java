/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Left Side Autonomous", group="Robot")
//This code should be run when the robot starts on the left side of the truss:
public class LeftAutonomous extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor leftFrontDrive = null;
    DcMotor leftBackDrive = null;
    DcMotor rightFrontDrive = null;
    DcMotor rightBackDrive = null;
    DcMotor armMotor = null;
    Servo angleIntake;
    Servo wheelIntake;
    DistanceSensor distanceSensor;

    @Override
    public void runOpMode() {
        /*
        Please input important information here (such as how to do 90 degree turns, etc.):

         */

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "FL");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "BL");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "FR");
        rightBackDrive = hardwareMap.get(DcMotor.class, "BR");
        armMotor = hardwareMap.get(DcMotor.class, "am1");
        angleIntake = hardwareMap.get(Servo.class, "servoangle");
        wheelIntake = hardwareMap.get(Servo.class, "servowheel");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "DS");
        MotorMethods MethodObj = new MotorMethods(leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive);
        ArmMethods armMethodObj = new ArmMethods(armMotor, angleIntake, wheelIntake);
        MethodObj.SetDirectionForward();
        MethodObj.setZeroBehaviorAll(DcMotor.ZeroPowerBehavior.BRAKE);
        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        String marker = "None";
        boolean found = false;
        double timeTurned = 0.0;
        double distance = distanceSensor.getDistance(DistanceUnit.CM);
            if (distanceSensor.getDistance(DistanceUnit.CM) < 90) {
                marker = "Center";
                found = true;
                //turn slightly right:
                MethodObj.timedMotorMove(80,0,0,0.3);
                sleep(400);
                //Move forward to stripe:
                MethodObj.timedMotorMove(2000,-0.2,0,0.0);
                //Deposits Pixel on stripe (Intake System):
                armMethodObj.intakeAuto(1);
                for(double time = runtime.milliseconds(); runtime.milliseconds()-time<2000;){wheelIntake.setPosition(0.1);}
                armMethodObj.intakeAuto(0);
                //Moves back to the start position:
                MethodObj.timedMotorMove(2000,0.2,0,0.0);
                sleep(400);
                //Turns the robot back to the starting direction:
                MethodObj.timedMotorMove(80,0.0,0,-0.3);
            }
            if (!found) {
                //Turns left until it finds the object or has completed the search (Scanning period):
                double firstTurned = runtime.milliseconds();
                for (double time = firstTurned; runtime.milliseconds()-time<1200; ) {
                    MethodObj.move(0, 0, -0.2);
                    telemetry.addData("Distance: ", distanceSensor.getDistance(DistanceUnit.CM));
                    telemetry.update();
                    if (distanceSensor.getDistance(DistanceUnit.CM) <= 100) {
                        marker = "left";
                        found = true;
                        MethodObj.move(0, 0, 0.0);
                        timeTurned = runtime.milliseconds()-firstTurned;
                        sleep(400);
                        break;
                    }
                }
                //Moves to the left stripe and places down the pixel (Only runs if the object was found during the scanning period):
                if (marker == "left"){
                    //Moves the robot to the stripe:
                    MethodObj.timedMotorMove(1000,-0.3,0,0.0);
                    sleep(400);
                    //Deposits pixel on stripe (Intake System):
                    armMethodObj.intakeAuto(1);
                    for(double time = runtime.milliseconds(); runtime.milliseconds()-time<2000;){wheelIntake.setPosition(0.1);}
                    armMethodObj.intakeAuto(0);
                    //Moves the robot back to the starting position:
                    MethodObj.timedMotorMove(1000,0.3,0,0.0);
                    sleep(1000);
                }
                //Turns the robot back to the starting direction:
                MethodObj.timedMotorMove((int)timeTurned,0.0,0,0.2);
                sleep(400);
            }
            //Runs if the object has not been found on both the center and left lines:
            if (!found) {
                marker = "right";
                //Turns the robot to the right:
                MethodObj.timedMotorMove(300,0.0,0,0.2);
                sleep(300);
                //Moves the robot to the stripe:
                MethodObj.timedMotorMove(900,0.3,0,0.0);
                sleep(300);
                //Deposits pixel on stripe (Intake System):
                armMethodObj.intakeAuto(1);
                for(double time = runtime.milliseconds(); runtime.milliseconds()-time<2000;){wheelIntake.setPosition(0.1);}
                armMethodObj.intakeAuto(0);
                //Moves the robot back to the starting position:
                MethodObj.timedMotorMove(900,-0.3,0,0.0);
                sleep(300);
                //Moves the robot back to the starting direction:
                MethodObj.timedMotorMove(300,0.0,0,-0.2);
                sleep(300);
            }
            //(Left side edition) Goes from starting position to the board, puts the pixel on the board, and then parks in the parking area:
            /*
            MethodObj.timedMotorMove(70,0.3,0,0);
            sleep(200);
            MethodObj.timedMotorMove(300,0,0,0.3);
            sleep(200);
            MethodObj.timedMotorMove(6000,0.3,0,0);
            sleep(200);
            MethodObj.timedMotorMove(300,0,0,-0.3);
            sleep(200);
            MethodObj.timedMotorMove(400,0.3,0,0);
            sleep(200);
            MethodObj.timedMotorMove(300,0,0,0.3);
            sleep(200);
            armMethodObj.intakeAuto(2);
            wheelIntake.setPosition(0.9);
            sleep(200);
            wheelIntake.setPosition(0.5);
            sleep(200);
            armMethodObj.intakeAuto(0);
            MethodObj.timedMotorMove(300,0,0,0.3);
            sleep(200);
            MethodObj.timedMotorMove(500,0.3,0,0);
            sleep(200);
            MethodObj.timedMotorMove(300,0,0,-0.3);
            sleep(200);
            MethodObj.timedMotorMove(200,0.3,0,0);
            sleep(200);
             */
            //Outputs information until the end of the Autonomous Period:
            while (opModeIsActive()) {
                telemetry.addData("First Distance: ", distance);
                telemetry.addData("Distance: ", distanceSensor.getDistance(DistanceUnit.CM));
                telemetry.addData("Object at side: ", marker);
                telemetry.update();
            }
        //Code that was used previously, but has either been replaced or turned into a method:
        /*
        MethodObj.setArmDegree(0);
        angleIntake.setPosition(0.9);
        sleep(2);
        telemetry.addData("a","aaaaaaa");
        telemetry.update();
        */

        /*
        Turns the robot back to the center:
        for(double turnback = runtime.milliseconds(); (runtime.milliseconds()+1000)-turnback<turnback;){
            MethodObj.move(0, 0, 0.2);
            telemetry.addData("Distance: ", distanceSensor.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
        MethodObj.move(0,0,0);
         */
    }
}