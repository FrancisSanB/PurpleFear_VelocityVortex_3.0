package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Francis on 1/6/2017.
 */
public class Autonomous2 extends LinearOpMode {
    DcMotor leftbackMotor;     //identify all of the motors
    DcMotor rightbackMotor;
    DcMotor leftfrontMotor;
    DcMotor rightfrontMotor;
    DcMotor shooterR;
    DcMotor shooterL;
    DcMotor elevator;
    DcMotor tumbler;
    Servo beaconServo;

    @Override
    public void runOpMode() throws InterruptedException {
        leftbackMotor = hardwareMap.dcMotor.get("leftback_motor");     //link each motor to each
        leftfrontMotor = hardwareMap.dcMotor.get("leftfront_motor");   //of the motors in the
        rightbackMotor = hardwareMap.dcMotor.get("rightback_motor");   //configure file on the
        rightfrontMotor = hardwareMap.dcMotor.get("rightfront_motor"); //phone
        shooterL = hardwareMap.dcMotor.get("shooterL");
        shooterR = hardwareMap.dcMotor.get("shooterR");
        elevator = hardwareMap.dcMotor.get("elevator");
        tumbler = hardwareMap.dcMotor.get("tublr");
        beaconServo = hardwareMap.servo.get("Bacon");

        waitForStart();



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
}
