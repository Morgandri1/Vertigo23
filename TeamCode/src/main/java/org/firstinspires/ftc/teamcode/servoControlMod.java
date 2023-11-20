package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;

/*
 * This OpMode scans a single servo back and forward until Stop is pressed.
 * The code is structured as a LinearOpMode
 * INCREMENT sets how much to increase/decrease the servo position each cycle
 * CYCLE_MS sets the update period.
 *
 * This code assumes a Servo configured with the name "left_hand" as is found on a Robot.
 *
 * NOTE: When any servo position is set, ALL attached servos are activated, so ensure that any other
 * connected servos are able to move freely before running this test.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@TeleOp(name = "Servo Control Mod", group = "Concept")
public class servoControlMod extends LinearOpMode {

    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    // Define class members
    Servo   servoIntake;
    Servo   servoArmFirst;
    Servo   servoArmSecond;
    Servo   servoArmThird;
    Servo   servoArmFourth;
    double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    // We might want to change the starting position depending on where the arm needs to start.


    @Override
    public void runOpMode() {

        // Connect to servo (Assume Robot Left Hand)
        // Change the text in quotes to match any servo name on your robot.
        servoIntake = hardwareMap.get(Servo.class, "servoIN");
        servoArmFirst = hardwareMap.get(Servo.class, "servoArm1");
        servoArmSecond = hardwareMap.get(Servo.class, "servoArm2");
        servoArmThird = hardwareMap.get(Servo.class, "servoArm3");
        servoArmFourth = hardwareMap.get(Servo.class, "servoArm4");
        // Wait for the start button
        telemetry.addData(">", "Press Start to scan Servo." );
        telemetry.update();
        waitForStart();


        // Scan servo till stop pressed.
        while(opModeIsActive()){
            position = servoIntake.getPosition();
            if (gamepad1.left_bumper){
                position -= 0.1;
                if (position > 1.0){position = 1.0;}
            }
            if (gamepad1.right_bumper){
                position += 0.1;
                if (position < 0.0){position = 0.0;}
            }
            servoIntake.setPosition(position);

            position = servoArmFirst.getPosition();
            if (gamepad1.left_trigger > 0f){
                position -= gamepad1.left_trigger;
                if (position < 0.0){position = 0.0;}
            }
            if (gamepad1.right_trigger > 0f){
                position += gamepad1.right_trigger;
                if (position > 0.5){position = 0.5;}
            }
            servoArmFirst.setPosition(position);

            position = servoArmSecond.getPosition();
            if (gamepad1.dpad_up){
                position += 0.1;
                if (position > 0.5){position = 0.5;}
            }
            else if (gamepad1.dpad_down){
                position -= 0.1;
                if (position < 0.0){position = 0.0;}
            }
            servoArmSecond.setPosition(position);

            servoArmThird.getPosition();
            if (gamepad1.dpad_right){
                position += 0.1;
                if (position > 0.5){position = 0.5;}
            }
            if (gamepad1.dpad_left){
                position -= 0.1;
                if (position < 0.0){position = 0.0;}
            }
            servoArmThird.setPosition(position);

            // test code
            servoArmFourth.setPosition(0.0);
            sleep(3000);
            servoArmFourth.setPosition(1.0);
            sleep(2000);
            servoArmFourth.setPosition(0.5);

            // Another possible trigger statment could be:
            /*
            position = servoArmSecond.getPosition();
            position += gamepad1.right_trigger;
            if (position > MAX_POS){position = MAX_POS;}
            servoArmSecond.setPosition(position);

            And reversed for the left trigger
            */
            // Display the current value
            telemetry.addData("Servo Position", "%5.2f", position);
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();

            // Set the servo to the new position and pause;

            sleep(CYCLE_MS);
            idle();
        }

        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();
    }
}
