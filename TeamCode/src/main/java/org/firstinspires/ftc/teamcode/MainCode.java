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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


/* Copyright (c) 2021 FIRST. All rights reserved.
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
import com.qualcomm.robotcore.util.ElapsedTime;

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

@TeleOp(name="Main code to run", group="Linear OpMode")
public class MainCode extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor leftFrontDrive = null;
    DcMotor leftBackDrive = null;
    DcMotor rightFrontDrive = null;
    DcMotor rightBackDrive = null;
    DcMotor armMotor = null;

    Servo angleIntake = null;
    Servo wheelIntake=null;

    int armStage=2;
    int none=0;
    int active=1;
    int idle=2;
    boolean gameToggle=true;
    int defaultDegreesFromStart=250;
    int armMovementArea=100;
    int servoTiltFactor=-50;
    int wheelIntakeDirection=1;
    double axial=0;
    double lateral=0;
    double yaw=0;
    double driveMultiplier=0.5;
    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "FL");
        leftBackDrive = hardwareMap.get(DcMotor.class, "BL");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "FR");
        rightBackDrive = hardwareMap.get(DcMotor.class, "BR");
        armMotor = hardwareMap.get(DcMotor.class, "am1");
        angleIntake = hardwareMap.get(Servo.class,"servoangle");
        wheelIntake = hardwareMap.get(Servo.class,"servowheel");
        MotorMethods MotorMethodObj = new MotorMethods(leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive);
        ArmMethods armMethodObj = new ArmMethods(armMotor,angleIntake,wheelIntake);
        MotorMethodObj.SetDirectionBackwards();

        MotorMethodObj.setZeroBehaviorAll(DcMotor.ZeroPowerBehavior.BRAKE);
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
        //MethodObj.SetDirectionForward();

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Arm Motor Position: ", armMotor.getCurrentPosition());
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.

            if((gamepad1.left_trigger<0.5&&gamepad1.right_trigger<0.5)||(gamepad1.left_trigger>0.5&&gamepad1.right_trigger>0.5)) {
                driveMultiplier=0.5;
            } else if (gamepad1.left_trigger>0.5&&gamepad1.right_trigger<0.5) {
                driveMultiplier=0.25;
            } else if (gamepad1.left_trigger<0.5&&gamepad1.right_trigger>0.5) {
                driveMultiplier=1;
            }

            telemetry.addData("drive mult",driveMultiplier);
            axial =driveMultiplier*(-gamepad1.left_stick_y);  // Note: pushing stick forward gives negative value
            lateral = driveMultiplier*(gamepad1.left_stick_x);
            yaw = driveMultiplier*(-gamepad1.right_stick_x);
            MotorMethodObj.move(axial, lateral, yaw);

            //arm movement and initialization
            if(!gamepad2.left_bumper){gameToggle=true;}

            //arm toggle
            if (gamepad2.left_bumper&&!(armStage==none)&&gameToggle) {
                telemetry.addData("Arm toggle ", armStage);
                if(armStage==active){armStage=idle;}else{armStage=active;}
                gameToggle=false;
            }
            //determines the angle for the arm based on the controller
            int gamepadArmInput = (Math.round(armMovementArea*(-gamepad2.right_stick_y)));
            double servoPosition=100-Math.round(armMethodObj.getArmDegree());
            telemetry.addData("input",gamepadArmInput);
            //moves the arm and tilts the intake to the correct position
            if (armStage==active) {

                if (gamepadArmInput >= 0) {
                     armMethodObj.setArmDegree(gamepadArmInput);angleIntake.setPosition(servoPosition/100);
                }else{armMethodObj.setArmDegree(0);angleIntake.setPosition(0);}

            } else if (armStage==idle) {

                    armMethodObj.setArmDegree(defaultDegreesFromStart);
                    angleIntake.setPosition(servoPosition/100);
            }else {
                armMethodObj.setArmDegree(0);
                telemetry.addData("Degree 0 ", armMotor.getCurrentPosition());
                angleIntake.setPosition(0.03);
            }

            //spins the wheels on the intake
            if(armStage==active){
                if(gamepad2.right_bumper) {
                    wheelIntake.setPosition(0.10);
                    telemetry.addData("servocont",wheelIntake.getPosition());
                } else if (gamepad2.right_trigger>0.5) {
                    wheelIntake.setPosition(0.90);
                    telemetry.addData("servocont",wheelIntake.getPosition());

                }else{
                    wheelIntake.setPosition(0.50);
                }
            }

            manageTelemetry();
            telemetry.addData("Armstage", armStage);
            telemetry.addData("arm position",armMethodObj.getArmDegree());
            telemetry.addData("servo position",angleIntake.getPosition());
            telemetry.update();
        }

    }
        public void manageTelemetry(){
            if(gamepad2.a){telemetry.addData("Arm Motor Position: ", armMotor.getTargetPosition());}
            if(gamepad1.guide||gamepad2.guide){telemetry.addData("Status", "Run Time: " + runtime.toString());}
            if(gamepad1.a){telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontDrive.getPower(), rightFrontDrive.getPower());}
            if(gamepad1.b){telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackDrive.getPower(), rightBackDrive.getPower());}
        }




}


