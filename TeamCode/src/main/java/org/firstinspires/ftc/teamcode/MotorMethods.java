package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

class MotorMethods {
    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;
    private double axial;
    private double lateral;
    private double yaw;
    private double RFPower;
    private double RBPower;
    private double LFPower;
    private double LBPower;
    public MotorMethods(DcMotor leftFront, DcMotor rightFront,DcMotor leftBack,DcMotor rightBack){
leftFrontDrive=leftFront;
rightFrontDrive=rightFront;
leftBackDrive=leftBack;
rightBackDrive=rightBack;

    }

    
    /*
    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "FL");
        leftBackDrive = hardwareMap.get(DcMotor.class, "BL");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "FR");
        rightBackDrive = hardwareMap.get(DcMotor.class, "BR");

    }
     */
    public void move(double axial, double lateral, double yaw){
        leftFrontDrive.setPower(axial + lateral + yaw);
        leftBackDrive.setPower(axial - lateral + yaw);
        rightFrontDrive.setPower(axial - lateral - yaw);
        rightBackDrive.setPower(axial + lateral - yaw);
    }
    public void setAllSpeed(double lf, double lb, double rf, double rb){
        leftFrontDrive.setPower(lf);
        rightFrontDrive.setPower(rf);
        leftBackDrive.setPower(lb);
        rightBackDrive.setPower(rb);
    }
    public void MotorStop(){
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
    public void MotorStall(){
        RFPower = rightFrontDrive.getPower();
        RBPower = rightBackDrive.getPower();
        LFPower = leftFrontDrive.getPower();
        LBPower = leftBackDrive.getPower();
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
    public void MotorResume(){
        leftFrontDrive.setPower(LFPower);
        rightFrontDrive.setPower(RFPower);
        leftBackDrive.setPower(LBPower);
        rightBackDrive.setPower(RBPower);
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
}
