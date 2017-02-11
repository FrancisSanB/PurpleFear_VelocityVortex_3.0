package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import com.qualcomm.robotcore.hardware.ColorSensor;


/*
* Created by Aakarsh on 1/15/2017.
*/


public class BeaconAutonomous extends LinearOpMode {

    DcMotor leftbackmotor;     //identify all of the tacos
    DcMotor rightbackmotor;
    DcMotor leftfrontmotor;
    DcMotor rightfrontmotor;
    DcMotor tumbler;
    DcMotor shooterRight;
    DcMotor shooterLeft;
    //DcMotor elevator;
    Servo beaconright;
    Servo beaconleft;
    ColorSensor ColorSensor;


    @Override
    public void runOpMode() throws InterruptedException {

        leftfrontmotor = hardwareMap.dcMotor.get("leftfront_motor");
        leftbackmotor = hardwareMap.dcMotor.get("leftback_motor");
        rightfrontmotor = hardwareMap.dcMotor.get("rightfront_motor");
        rightbackmotor = hardwareMap.dcMotor.get("rightback_motor");
        tumbler = hardwareMap.dcMotor.get("tublr");
        shooterLeft = hardwareMap.dcMotor.get("shooterL");
        shooterRight = hardwareMap.dcMotor.get("shooterR");
        //elevator = hardwareMap.dcMotor.get("elevator");
        beaconright = hardwareMap.servo.get("bacon");
        beaconleft = hardwareMap.servo.get("bacon2");

        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.RelativeLayout);


        // bPrevState and bCurrState represent the previous and current state of the button.
        boolean bPrevState = false;
        boolean bCurrState = false;

        // bLedOn represents the state of the LED.
        boolean bLedOn = true;

        // get a reference to our ColorSensor object.

        // Set the LED in the beginning
        ColorSensor.enableLed(bLedOn);


        waitForStart();

        tankdrive(-0.3, -0.3, 650);
        shooterDrive(-1, 1);
        sleep(1500);
        tumblerDrive(1);
        //elevatorDrive(1, 2000);
        sleep(2000);
        tumbler.setPower(0);
        tumbler.setPower(0);
        shooterLeft.setPower(0);
        shooterRight.setPower(0);
        tankdrive(-0.3,0.3,500);
        vuforia(3);
    }


    private void tankdrive(double leftY, double rightY, long sleepAmount) throws InterruptedException {

        rightY = -rightY;               //flip the power of the right side

        leftfrontmotor.setPower(leftY); //set the according power to each motor
        leftbackmotor.setPower(leftY);
        rightfrontmotor.setPower(rightY);
        rightbackmotor.setPower(rightY);

        sleep(sleepAmount);

        leftfrontmotor.setPower(0); //set the according power to each motor
        leftbackmotor.setPower(0);
        rightfrontmotor.setPower(0);
        rightbackmotor.setPower(0);
    }


    private void vuforia(int beaconnumber) throws InterruptedException {

        boolean beacon_aligned = false;

        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);     //what the camera sees will show up on screen
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;                                     //use back camera cuz its better
        params.vuforiaLicenseKey = "AVfIMZf/////AAAAGW5IK3Zcv0DuqiUmrZ3ShqEzEv3bTkXlTsuKRC+0ErP/Lubue95iAvq5BKhA279VyIwDrKUVBQ+5d/UGHn+wyr56yBnLJKx6ATkuaKA5dbnFBy9eC0J/HD0/ddGhCGM9zs2sxv70zd/mfYMN+7z60iVvuHNBGjJKRyrnstPGqA0QuLniNLIZLW71HPcZxBWoLnfTq6CEPIvM4lOqZylvjMWREAz5Y+Hb9EtrExmRLqPAcDKefyjZVZnfO+2u9rtBe7f6OcKQsQnrvodA1+gQ1C89gicAo3gB76b3llisd4QLk41AAjz5HJkuGo0WqpjMIQcRVKhHaeapQZ5D76YxyPb1hQrx08IQSv5Bkwv7yjJp";
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;              //axes of image show up
        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(params);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);                                       //to be able to track multiple images

        VuforiaTrackables beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Lego");
        beacons.get(3).setName("Gears");

        while (opModeIsActive() && !beacon_aligned) {
            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beacons.get(beaconnumber).getListener()).getPose();

            if (pose != null) {
                VectorF translation = pose.getTranslation();

                telemetry.addData("translation", translation);

                double degreesToTurn = Math.toDegrees(Math.atan2(translation.get(1), translation.get(2)));
                telemetry.addData("degrees", degreesToTurn);

                if (degreesToTurn >= -3) {

                    unlimitedDrive(0.3, -0.3);

                } else if (degreesToTurn <= 3) {

                    unlimitedDrive(-0.3, 0.3);

                } else {
                    beacon_aligned = true;

                    tankdrive(-0.3, -0.3, 3000);
                    tankdrive(-0.3, 0.3, 500);


                    telemetry.addData("YOU'VE ARRIVED AT SOOUBWAY EXTENSIONS", beacon_aligned);
                }
            }
            telemetry.update();

            if ((ColorSensor.red() == 360) && (ColorSensor.green() == 0) && (ColorSensor.blue() == 0)) {   //beacon detect
                Servo(1, 0.9);
                tankdrive(-0.3, -0.3, 500);
                wait(1500);
                tankdrive(0.3, 0.3, 500);
                stopMotors();

            }
            if ((ColorSensor.red() == 0) && (ColorSensor.green() == 0) && (ColorSensor.blue() == 360)) {
                Servo(2, 0.9);
                tankdrive(-0.3, -0.3, 1000);
                wait(1500);
                tankdrive(0.3, 0.3, 1000);
                stopMotors();

            }
        }
    }

    private void unlimitedDrive(double leftY, double rightY) throws InterruptedException {

        rightY = -rightY;

        leftbackmotor.setPower(leftY);
        leftfrontmotor.setPower(leftY);
        rightbackmotor.setPower(rightY);
        rightfrontmotor.setPower(rightY);

    }

    private void Servo(double servoNumber, double servoPosition) throws InterruptedException {
        if (servoNumber == 1) {
            beaconright.setPosition(servoPosition);
        }
        if (servoNumber == 2) {
            beaconleft.setPosition(servoPosition);
        }


    }

    private void tumblerDrive(double power) throws InterruptedException {
        tumbler.setPower(power);
    }

    //private void elevatorDrive(double power, long sleepAmount) throws InterruptedException {
        //elevator.setPower(power);
      //  sleep(sleepAmount);
        //elevator.setPower(0);

//    }

    private void shooterDrive(double leftpower, double rightpower) throws InterruptedException {
        shooterLeft.setPower(leftpower);
        shooterRight.setPower(rightpower);


    }

    private void stopMotors() throws InterruptedException {
        shooterLeft.setPower(0);
        shooterRight.setPower(0);
        //elevator.setPower(0);
        tumbler.setPower(0);
        leftfrontmotor.setPower(0);
        leftbackmotor.setPower(0);
        rightfrontmotor.setPower(0);
        rightbackmotor.setPower(0);
        beaconleft.setPosition(0);
        beaconright.setPosition(0);

    }
}


