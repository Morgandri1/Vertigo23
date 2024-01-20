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

@Autonomous(name="Left Autonomous", group="Robot")
//This code should be run when the robot starts on the left side of the truss:
public class LeftAutonomous extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        /* Not necessary because the MotorMethods file does/can do all of the motor-related tasks:
        DcMotor leftFrontDrive = hardwareMap.get(DcMotor.class, "FL");
        DcMotor leftBackDrive = hardwareMap.get(DcMotor.class, "BL");
        DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "FR");
        DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "BR");
        DcMotor armMotor = hardwareMap.get(DcMotor.class, "am1");
        */
        Servo angleIntake = hardwareMap.get(Servo.class, "servoangle");
        Servo wheelIntake = hardwareMap.get(Servo.class, "servowheel");
        ElapsedTime runtime = new ElapsedTime();
        DistanceSensor distanceSensor = hardwareMap.get(DistanceSensor.class, "DS");
        MotorMethods MMO = new MotorMethods(hardwareMap);
        ArmMethods AMO = new ArmMethods(hardwareMap);
        MMO.SetDirectionForward();
        MMO.setZeroBehaviorAll(DcMotor.ZeroPowerBehavior.BRAKE);
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();
        String marker = "None";
        boolean found = false;
        double timeTurned = 0.0;
        double[][] armPositions = {{0, 0.90}, {0, 0.4}, {170, 0.95}, {25, 0.20}};
        MMO.timedMotorMove(70, -0.3, 0, 0, false);
        MMO.timedMotorMove(20, 0, 0, 0.2, false);
        sleep(100);
        double distance = distanceSensor.getDistance(DistanceUnit.CM);
        if (distanceSensor.getDistance(DistanceUnit.CM) < 90) {
            marker = "Center";
            found = true;
            //turn slightly right:
            MMO.TMM_With_Arm(100, 0, 0, 0.3, 3, MMO, AMO);
            sleep(400);
            //Move forward to stripe:
            MMO.TMM_With_Arm(2000, -0.2, 0, 0.0,3, MMO, AMO);
            sleep(400);
            //Deposits Pixel on stripe (Intake System):
            for (double time = runtime.milliseconds(); runtime.milliseconds() - time < 2000; ) {
                angleIntake.setPosition(armPositions[3][1]);
                AMO.setArmDegree((int) armPositions[3][0]);
                wheelIntake.setPosition(0.1);
                telemetry.addData("Current angle position: ", angleIntake.getPosition());
                telemetry.update();
            }
            wheelIntake.setPosition(0.5);
            for (double time = runtime.milliseconds(); runtime.milliseconds() - time < 2000; ) {
                angleIntake.setPosition(armPositions[0][1]);
                telemetry.addData("Current angle position: ", angleIntake.getPosition());
                telemetry.update();
            }
            sleep(200);
            for (double time = runtime.milliseconds(); runtime.milliseconds() - time < 3000; ) {
                AMO.setArmDegree((int) armPositions[0][0]);
            }
            sleep(200);
            //Moves back to the start position:
            MMO.TMM_With_Arm(2000, 0.2, 0, 0.0,  0, MMO, AMO);
            sleep(400);
            //Turns the robot back to the starting direction:
            MMO.TMM_With_Arm(100, 0.0, 0, -0.3,  0, MMO, AMO);
            sleep(200);
        }
        if (!found) {
            //Turns left until it finds the object or has completed the search (Scanning period):
            double firstTurned = runtime.milliseconds();
            for (double time = firstTurned; runtime.milliseconds() - time < 800; ) {
                MMO.move(0, 0, -0.2);
                angleIntake.setPosition(armPositions[3][1]);
                AMO.setArmDegree((int) armPositions[3][0]);
                telemetry.addData("Distance: ", distanceSensor.getDistance(DistanceUnit.CM));
                telemetry.update();
                timeTurned = runtime.milliseconds() - firstTurned;
                if (distanceSensor.getDistance(DistanceUnit.CM) <= 85) {
                    timeTurned = runtime.milliseconds() - firstTurned;
                    marker = "left";
                    found = true;
                    MMO.move(0, 0, 0.0);
                    sleep(400);
                    break;
                }
            }
            //Moves to the left stripe and places down the pixel (Only runs if the object was found during the scanning period):
            if (marker == "left") {
                MMO.TMM_With_Arm(200, 0, 0, -0.2, 3, MMO, AMO);
                //Moves the robot to the stripe:
                MMO.TMM_With_Arm(900, -0.3, 0, 0.0, 3, MMO, AMO);
                sleep(400);
                //Deposits pixel on stripe (Intake System):
                for (double time = runtime.milliseconds(); runtime.milliseconds() - time < 2000; ) {
                    angleIntake.setPosition(armPositions[3][1]);
                    AMO.setArmDegree((int) armPositions[3][0]);
                    wheelIntake.setPosition(0.1);
                    telemetry.addData("Current angle position: ", angleIntake.getPosition());
                    telemetry.update();
                }
                wheelIntake.setPosition(0.5);
                for (double time = runtime.milliseconds(); runtime.milliseconds() - time < 2000; ) {
                    angleIntake.setPosition(armPositions[0][1]);
                    telemetry.addData("Current angle position: ", angleIntake.getPosition());
                    telemetry.update();
                }
                sleep(200);
                //for(double time = runtime.milliseconds(); runtime.milliseconds()-time<2000;){wheelIntake.setPosition(0.1);}
                sleep(200);
                //Moves the robot back to the starting position:
                MMO.TMM_With_Arm(900, 0.3, 0, 0.0, 0, MMO, AMO);
                sleep(200);
                MMO.TMM_With_Arm(200, 0, 0, 0.2, 0, MMO, AMO);
                sleep(300);
            }
            //Turns the robot back to the starting direction:
            MMO.timedMotorMove(timeTurned, 0.0, 0, 0.2, false);
            sleep(400);
        }
        //Runs if the object has not been found on both the center and left lines:
        if (!found) {
            marker = "right";
            //Turns the robot to the right:
            MMO.TMM_With_Arm(500, 0.0, 0, 0.2, 3, MMO, AMO);
            sleep(300);
            //Moves the robot to the stripe:
            MMO.TMM_With_Arm(900, -0.3, 0, 0.0, 3, MMO, AMO);
            sleep(300);
            //Deposits pixel on stripe (Intake System):
            for (double time = runtime.milliseconds(); runtime.milliseconds() - time < 2000; ) {
                angleIntake.setPosition(armPositions[3][1]);
                AMO.setArmDegree((int) armPositions[3][0]);
                wheelIntake.setPosition(0.1);
                telemetry.addData("Current angle position: ", angleIntake.getPosition());
                telemetry.update();
            }
            wheelIntake.setPosition(0.5);
            for (double time = runtime.milliseconds(); runtime.milliseconds() - time < 2000; ) {
                angleIntake.setPosition(armPositions[0][1]);
                telemetry.addData("Current angle position: ", angleIntake.getPosition());
                telemetry.update();
            }
            sleep(200);
            //for(double time = runtime.milliseconds(); runtime.milliseconds()-time<2000;){wheelIntake.setPosition(0.1);}
            sleep(200);
            //Moves the robot back to the starting position:
            MMO.TMM_With_Arm(900, 0.3, 0, 0.0, 0, MMO, AMO);
            sleep(200);
            //Moves the robot back to the starting direction:
            MMO.TMM_With_Arm(500, 0.0, 0, -0.2, 0, MMO, AMO);
            sleep(200);
        }
        //Outputs information until the end of the Autonomous Period:
        while (opModeIsActive()) {
            telemetry.addData("First Distance: ", distance);
            telemetry.addData("Distance: ", distanceSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("Object at side: ", marker);
            telemetry.update();
        }
    }
}