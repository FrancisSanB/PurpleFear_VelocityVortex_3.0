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


        tankDrive(-0.3,-0.3,800);
        tumbler(1);
        shooter(1, 1);
        sleep(2000);
        //elevator(0.3);
        sleep(3000);
        shooterL.setPower(0);
        shooterR.setPower(0);
       // elevator.setPower(0);
        tumbler.setPower(0);
        tankDrive(-0.3,-0.3,1500);
        tankDrive(-0.3,0.3,1500);
        tankDrive(-0.3,-0.3,1100);
       /*
      tankDrive(0.3,0.3,1400);
      tankDrive(-0.3,0.3,300);
      tankDrive(0.3,0.3,700);
      */


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

    private void tumbler(double power) throws InterruptedException {
        tumbler.setPower(power);
    }

   // private void elevator(double power) throws InterruptedException {
        //elevator.setPower(power);



//    }

    private void shooter(double leftpower, double rightpower) throws InterruptedException {
        shooterL.setPower(-leftpower);
        shooterR.setPower(rightpower);




    }


}