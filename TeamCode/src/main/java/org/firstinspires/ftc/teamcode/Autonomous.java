package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.Timer;

/**
 * Created by Francis on 11/21/2016.
 */

public class Autonomous extends LinearOpMode {

    DcMotor leftfrontMotor;
    DcMotor leftbackMotor;
    DcMotor rightfrontMotor;
    DcMotor rightbackMotor;
    DcMotor tumbler;


    @Override
    public void runOpMode() throws InterruptedException {


        leftfrontMotor = hardwareMap.dcMotor.get("leftfront_motor");     //grab the configure file on the phone
        leftbackMotor = hardwareMap.dcMotor.get("leftback_motor");       //and compare it to the motors/sensors
        rightfrontMotor = hardwareMap.dcMotor.get("rightfront_motor");  //in the code
        rightbackMotor = hardwareMap.dcMotor.get("rightback_motor");
        tumbler = hardwareMap.dcMotor.get("tublr");

        leftbackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightbackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftfrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightfrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        PDTarget(1);
        encoderDrive(1,1, 1);

    }




    private void tankdrive(double leftY, double rightY, long sleepAmount) throws InterruptedException{
        rightY = -rightY;

        leftfrontMotor.setPower(leftY); //set the according power to each motor
        leftbackMotor.setPower(leftY);
        rightfrontMotor.setPower(rightY);
        rightbackMotor.setPower(rightY);

        sleep(sleepAmount);             //this is the amount the robot will run in milliseconds

        leftfrontMotor.setPower(0);     //make sure that the all motors are set to zero afterward
        leftbackMotor.setPower(0);
        rightfrontMotor.setPower(0);
        rightbackMotor.setPower(0);

    }



    private void tumblerDrive(double power, long sleepAmount) throws InterruptedException {
        tumbler.setPower(power);
        sleep(sleepAmount);
        tumbler.setPower(power);

    }

    private void encoderTankDrive(double leftY, double rightY) throws InterruptedException {
        rightY = -rightY;

        leftfrontMotor.setPower(leftY); //set the according power to each motor
        leftbackMotor.setPower(leftY);
        rightfrontMotor.setPower(rightY);
        rightbackMotor.setPower(rightY);

    }

    private void encoderDrive(double leftY, double rightY, int rotationNumber) throws InterruptedException {
        rotationNumber = rotationNumber*1440;

        int leftTarget = leftbackMotor.getCurrentPosition() + rotationNumber;
        int rightTarget = rightbackMotor.getCurrentPosition() - rotationNumber;

        encoderTankDrive(leftY, rightY);

        while (leftbackMotor.getCurrentPosition() < leftTarget && rightbackMotor.getCurrentPosition() > rightTarget) {}
        encoderTankDrive(0, 0);
        //Math.abs(e) > 20 do this
    }

    private final float P = 0.0001f;
    private final float D = 0.0001f;
    private void PDTarget(int leftTarget, int rightTarget) {
        PDTarget(leftTarget, rightTarget, 500, 20);
    }

    private void PDTarget(int leftTarget, int rightTarget, long timeoutMs, int thresh) {
        long startMs = System.currentTimeMillis();
        double leftPower = 0;
        double rightPower = 0;
        int rightError = 0;
        int rightDer = 0;
        int leftError = 0;
        int leftDer = 0;
        while (System.currentTimeMillis() - startMs > timeoutMs) {
            rightDer = 
            rightError = rightbackMotor.getCurrentPosition() - rightTarget;
            if (rightDer != 0 || leftDer != 0) {
                startMs = System.currentTimeMillis();
            }

        }
    }

}