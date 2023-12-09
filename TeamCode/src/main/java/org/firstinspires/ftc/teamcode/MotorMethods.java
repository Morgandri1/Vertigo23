package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

class MotorMethods{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;
    public MotorMethods(DcMotor leftFront, DcMotor rightFront,DcMotor leftBack,DcMotor rightBack){
        leftFrontDrive = leftFront;
        leftBackDrive = leftBack;
        rightFrontDrive = rightFront;
        rightBackDrive = rightBack;

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
    public double ReturnLF(){return leftFrontDrive.getPower();}
    public double ReturnLB(){return leftBackDrive.getPower();}
    public double ReturnRF(){return rightFrontDrive.getPower();}
    public double ReturnRB(){
        return rightBackDrive.getPower();
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
    public void timedMotorMove(int time, double axial, double lateral, double yaw) {
        for (double startTime = runtime.milliseconds(); runtime.milliseconds() - startTime < time; ) {move(axial, lateral, yaw);}
        move(0,0,0);
    }
}
