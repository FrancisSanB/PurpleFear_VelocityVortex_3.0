package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Vikash on 12/28/2016.
 */
public class NeverestMotorCheck extends LinearOpMode {
    DcMotor shooterL;
    DcMotor shooterR;
    DcMotor leftfrontMotor;
    DcMotor leftbackMotor;
    DcMotor rightfrontMotor;
    DcMotor rightbackMotor;
    DcMotor tumbler;                      //you might have to keep this on PLEASE ASK
    DcMotor elevator;

    @Override
    public void runOpMode() throws InterruptedException {
        leftfrontMotor = hardwareMap.dcMotor.get("leftfront_motor");
        leftbackMotor = hardwareMap.dcMotor.get("leftback_motor");
        rightfrontMotor = hardwareMap.dcMotor.get("rightfront_motor");
        rightbackMotor = hardwareMap.dcMotor.get("rightback_motor");
        tumbler = hardwareMap.dcMotor.get("tublr");
        shooterL = hardwareMap.dcMotor.get("shooterL");
        shooterR = hardwareMap.dcMotor.get("shooterR");
        elevator = hardwareMap.dcMotor.get("elevator");

        waitForStart();

        tumblerDrive(1, 1000);
        elevatorDrive(1,1000);
        shooterDrive(-1, 1, 1000);
        /*elevator.setPower(1);
        sleep(1000);
        elevator.setPower(0);*/

    }

    private void tumblerDrive(double power, long sleepAmount) throws InterruptedException {
        tumbler.setPower(power);
        sleep(sleepAmount);
        tumbler.setPower(0);
    }

    private void elevatorDrive(double power, long sleepAmount) throws InterruptedException {
        elevator.setPower(power);
        sleep(sleepAmount);
        elevator.setPower(0);
    }

    private void shooterDrive(double leftpower, double rightpower, long sleepAmount) throws InterruptedException {
        shooterL.setPower(leftpower);
        shooterR.setPower(rightpower);

        sleep(sleepAmount);

        shooterL.setPower(0);
        shooterR.setPower(0);
    }

    private void tankDrive(double leftY, double rightY, long sleepAmount) throws InterruptedException {
        rightY = -rightY;


        leftfrontMotor.setPower(leftY);
        leftbackMotor.setPower(leftY);
        rightfrontMotor.setPower(rightY);
        rightbackMotor.setPower(rightY);

        sleep(sleepAmount);

        leftfrontMotor.setPower(0);
        leftbackMotor.setPower(0);
        rightfrontMotor.setPower(0);
        rightbackMotor.setPower(0);


    }
}
