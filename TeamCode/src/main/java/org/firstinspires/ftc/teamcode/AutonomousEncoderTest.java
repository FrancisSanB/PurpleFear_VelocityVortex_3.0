package org.firstinspires.ftc.teamcode;

import android.test.InstrumentationTestRunner;

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

        //Math.abs(e) > 20 do this

        waitForStart();

        encoderDrive(-1, -1, -5);
        //sleep(5000);
        //encoderDrivePD(1440, 1440);


    }

    private void encoderTankDrive(double leftY, double rightY) throws InterruptedException {
        rightY = -rightY;

        leftfrontMotor.setPower(leftY); //set the according power to each motor
        leftbackMotor.setPower(leftY);
        rightfrontMotor.setPower(rightY);
        rightbackMotor.setPower(rightY);

    }

    private void encoderDrive(double leftY, double rightY, int distance) throws InterruptedException {
        distance = distance*1440;
        //telemetry.addData(">", "method called");

        leftbackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightbackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //reset the encoders
        leftbackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightbackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //tell it to start counting the position
        /*leftbackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightbackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/

        //set the distance

        //tell it to start counting the position
        leftbackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightbackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftbackMotor.setTargetPosition(distance);
        rightbackMotor.setTargetPosition(distance);

        //set the power
        encoderTankDrive(leftY, rightY);

        while (opModeIsActive() && leftbackMotor.getCurrentPosition() < (distance - 180) && rightbackMotor.getCurrentPosition() < (distance - 180)) {
            //wait until the position is reached
        }

        //telemetry.addData(">", "while loop called");
        //telemetry.addData("number of ticks right", rightbackMotor.getCurrentPosition());

        rightfrontMotor.setPower(0);
        leftfrontMotor.setPower(0);
    }


    private void encoderDrive2(double leftY, double rightY, int rotationNumber) throws InterruptedException {
        telemetry.addData("encoderdrive called", leftY);
        rotationNumber = rotationNumber*1440;

        leftbackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightbackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int leftTarget = leftbackMotor.getCurrentPosition() + rotationNumber;
        int rightTarget = rightbackMotor.getCurrentPosition() - rotationNumber;

        leftbackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightbackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        encoderTankDrive(leftY, rightY);
        long time = System.currentTimeMillis();
        boolean atLeftTarget, atRightTarget;
        boolean isLeftReversed = leftY < 0;
        boolean isRightReversed = rightY < 0;
        while(true) {
            atLeftTarget = leftbackMotor.getCurrentPosition() >= leftTarget;
            atRightTarget = rightbackMotor.getCurrentPosition() <= rightTarget;
            if (atLeftTarget && atRightTarget) {
                break;
            } else if (atLeftTarget) {
                rightbackMotor.setPower(rightY);
                rightfrontMotor.setPower(rightY);
            } else if (atRightTarget) {
                leftbackMotor.setPower(leftY);
                leftfrontMotor.setPower(rightY);
            }else {
                encoderTankDrive(leftY, rightY);
            }
            sleep(50);
        }

        encoderTankDrive(0, 0);

        leftbackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightbackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }


    private void encoderDrivePD(int leftTarget, int rightTarget) {
        PDTarget(leftTarget, rightTarget, 500, 20);
    }

    private void PDTarget(int leftTarget, int rightTarget, long timeoutMs, int thresh) {
        leftbackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightbackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftbackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightbackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        long startMs = System.currentTimeMillis();
        while(true) {
            if (System.currentTimeMillis() - startMs > 100) {
                double leftPower = updateLeft(leftTarget, leftbackMotor.getCurrentPosition(), System.currentTimeMillis() - startMs);
                double rightPower = updateRight(rightTarget, rightbackMotor.getCurrentPosition(), System.currentTimeMillis() - startMs);
                rightfrontMotor.setPower(rightPower);
                rightbackMotor.setPower(rightPower);
                leftfrontMotor.setPower(leftPower);
                leftbackMotor.setPower(leftPower);
                if (Math.abs(leftPower) < 0.3) {
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