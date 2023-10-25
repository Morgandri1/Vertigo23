package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;


public class MotorMethods {

    public static void main(String[] args) {
        OmniDriveMod obj = new OmniDriveMod();
        double yaw  =  gamepad1.right_stick_x;
        obj.setAllSpeed(0.0,1.0,0.6,0.3);
        obj.setAllDirec(0.0,0.0,gamepad1.right_stick_x);
}
}
