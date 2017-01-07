package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.Timer;

/**
 * Created by Francis on 11/21/2016.
 */

public class AutonomousEncoderTest extends LinearOpMode {

    DcMotor leftfrontMotor;
    DcMotor leftbackMotor;
    DcMotor rightfrontMotor;
    DcMotor rightbackMotor;
    DcMotor tumbler;

    int lastRightError = 0;
    int lastLeftError = 0;

    private final float P = 0.0001f; // Proportional term constant
    private final float D = 0.0001f; // Derivative term constant


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

        //Math.abs(e) > 20 do this

        encoderDrive(1,1, 1);
        //PDTarget(1440, 1440);

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
        telemetry.addData("encoderdrive called", leftY);
        rotationNumber = rotationNumber*1440;

        int leftTarget = leftbackMotor.getCurrentPosition() + rotationNumber;
        int rightTarget = rightbackMotor.getCurrentPosition() - rotationNumber;

        encoderTankDrive(leftY, rightY);
        long time = System.currentTimeMillis();
        while(System.currentTimeMillis() < time+500) {}
        //while (leftbackMotor.getCurrentPosition() < leftTarget && rightbackMotor.getCurrentPosition() > rightTarget) {}
        encoderTankDrive(0, 0);

    }


    private void PDTarget(int leftTarget, int rightTarget) {
        PDTarget(leftTarget, rightTarget, 500, 20);
    }

    private void PDTarget(int leftTarget, int rightTarget, long timeoutMs, int thresh) {
        long startMs = System.currentTimeMillis();
        while(true) {
            if (System.currentTimeMillis() - startMs > 100) {
                double leftPower = updateLeft(leftTarget, leftbackMotor.getCurrentPosition(), System.currentTimeMillis() - startMs);
                double rightPower = updateRight(rightTarget, rightbackMotor.getCurrentPosition(), System.currentTimeMillis() - startMs);
                rightfrontMotor.setPower(rightPower);
                rightbackMotor.setPower(rightPower);
                leftfrontMotor.setPower(leftPower);
                leftbackMotor.setPower(leftPower);
                if (Math.abs(leftPower) < 0.1) {
                    break;
                }
                startMs = System.currentTimeMillis();
            }
        }
    }

    private double updateRight(int target, int current, long elapsedMs) {
        int rightError = target - current;
        double pdTerm = P*rightError + D*(rightError - lastRightError);
        lastRightError = rightError;
        return constrain(pdTerm, -1.0, 1.0);
    }

    private double updateLeft(int target, int current, long elapsedMs) {
        int leftError = target - current;
        double pdTerm = P*leftError + D*(leftError - lastLeftError);
        lastLeftError = leftError;
        return constrain(pdTerm, -1.0, 1.0);
    }

    /**
     * Returns v constrained between min and max
     * @param v Value to be constained
     * @param max
     * @param min
     * @return
     */
    private double constrain(double v, double min, double max) {
        if (v > max) {
            return max;
        } else if (v < min) {
            return min;
        } else {
            return v;
        }
    }
}