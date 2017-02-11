package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Edmund on 1/5/2017.
 * Stolen by paco on 1/5/2017.
 */
public class Autonomous4 extends LinearOpMode {

    DcMotor leftfrontMotor;
    DcMotor leftbackMotor;
    DcMotor rightfrontMotor;
    DcMotor rightbackMotor;
    DcMotor shooterL;
    DcMotor shooterR;
    DcMotor tumbler;
    //DcMotor elevator;

    @Override
    public void runOpMode() throws InterruptedException {
        leftfrontMotor = hardwareMap.dcMotor.get("leftfront_motor");     //grab the configure file on the phone
        leftbackMotor = hardwareMap.dcMotor.get("leftback_motor");       //and compare it to the motors/sensors
        rightfrontMotor = hardwareMap.dcMotor.get("rightfront_motor");  //in the code
        rightbackMotor = hardwareMap.dcMotor.get("rightback_motor");
        tumbler = hardwareMap.dcMotor.get("tublr");
        shooterL = hardwareMap.dcMotor.get("shooterL");
        shooterR = hardwareMap.dcMotor.get("shooterR");
        //elevator = hardwareMap.dcMotor.get("elevator");

        waitForStart();

        tankDrive(-0.3, -0.3, 1900);
        shooterDrive(1, -1);
        sleep(5000);
        tumblerDrive(1);
        sleep(2000);
        tumblerDrive(0);
        shooterDrive(0, 0);
        tankDrive(-0.35, 0.35, 300);
        tankDrive(-0.3, -0.3, 1000);

    }


    private void tankDrive(double leftY, double rightY, long sleepAmount) throws InterruptedException {

        rightY = -rightY;               //flip the power of the right side

        leftfrontMotor.setPower(leftY); //set the according power to each motor
        leftbackMotor.setPower(leftY);
        rightfrontMotor.setPower(rightY);
        rightbackMotor.setPower(rightY);

        sleep(sleepAmount);

        leftfrontMotor.setPower(0); //set the according power to each motor
        leftbackMotor.setPower(0);
        rightfrontMotor.setPower(0);
        rightbackMotor.setPower(0);
    }

    private void tumblerDrive(double power) throws InterruptedException {
        tumbler.setPower(power);
    }
    // private void elevator(double power) throws InterruptedException {
    //elevator.setPower(power);


//    }

    private void shooterDrive(double leftpower, double rightpower) throws InterruptedException {
        shooterL.setPower(-leftpower);
        shooterR.setPower(rightpower);


    }
}
