package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
public class ServoMethod extends LinearOpMode{
    Servo servoIntake;
    private double intakeMax = 1.0;
    private double intakeMin = 0.0;
    public ServoMethod(){
        servoIntake = hardwareMap.get(Servo.class, "servoIN");
    }
    public void intakeToPosition(double position){
        if (position <= intakeMax && position >= intakeMin) {
            servoIntake.setPosition(position);
        }
    }
    public void intakeSetLimits(double min, double max){
        // limits will most likely be set beforehand
        if (max > 1.0){max = 1.0;}
        else if (max < 0.0){max = 0.0;}
        if (min > max){min = max;}
        else if (min < 0.0){min = 0.0;}
        intakeMax = max;
        intakeMin = min;
    }
    @Override
    public void runOpMode(){
    }
}
