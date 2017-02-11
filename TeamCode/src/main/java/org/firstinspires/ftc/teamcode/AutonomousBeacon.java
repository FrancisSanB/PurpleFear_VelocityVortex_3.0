package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;



public class AutonomousBeacon extends LinearOpMode {
    DcMotor leftfrontMotor;
    DcMotor leftbackMotor;
    DcMotor rightfrontMotor;
    DcMotor rightbackMotor;
    DcMotor shooterL;
    DcMotor shooterR;
    DcMotor tumbler;
    DcMotor elevator;
    ColorSensor colorSensor;
    Servo servo1;
    Servo servo2;


    boolean beacon_aligned = false;

    @Override
    public void runOpMode() throws InterruptedException {
        leftfrontMotor = hardwareMap.dcMotor.get("leftfront_motor");     //grab the configure file on the phone
        leftbackMotor = hardwareMap.dcMotor.get("leftback_motor");       //and compare it to the motors/sensors
        rightfrontMotor = hardwareMap.dcMotor.get("rightfront_motor");  //in the code
        rightbackMotor = hardwareMap.dcMotor.get("rightback_motor");
        tumbler = hardwareMap.dcMotor.get("tublr");
        shooterL = hardwareMap.dcMotor.get("shooterL");
        shooterR = hardwareMap.dcMotor.get("shooterR");
        elevator = hardwareMap.dcMotor.get("elevator");
        servo1 = hardwareMap.servo.get("bacon");
        servo2 = hardwareMap.servo.get("bacon2");


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
        colorSensor.enableLed(bLedOn);


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


        VuforiaLocalizer.Parameters params2 = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params2.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params2.vuforiaLicenseKey = "AVfIMZf/////AAAAGW5IK3Zcv0DuqiUmrZ3ShqEzEv3bTkXlTsuKRC+0ErP/Lubue95iAvq5BKhA279VyIwDrKUVBQ+5d/UGHn+wyr56yBnLJKx6ATkuaKA5dbnFBy9eC0J/HD0/ddGhCGM9zs2sxv70zd/mfYMN+7z60iVvuHNBGjJKRyrnstPGqA0QuLniNLIZLW71HPcZxBWoLnfTq6CEPIvM4lOqZylvjMWREAz5Y+Hb9EtrExmRLqPAcDKefyjZVZnfO+2u9rtBe7f6OcKQsQnrvodA1+gQ1C89gicAo3gB76b3llisd4QLk41AAjz5HJkuGo0WqpjMIQcRVKhHaeapQZ5D76YxyPb1hQrx08IQSv5Bkwv7yjJp";
        params2.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        VuforiaLocalizer vuforia2 = ClassFactory.createVuforiaLocalizer(params2);

        VuforiaTrackables beacons2 = vuforia2.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Lego");
        beacons.get(3).setName("Gears");


        waitForStart();
        beacons.activate();

        while (opModeIsActive()) {


            // update previous state variable.
            bPrevState = bCurrState;

            // convert the RGB values to HSV values.
            Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

            // send the info back to driver station using telemetry function.
            telemetry.addData("LED", bLedOn ? "On" : "Off");
            telemetry.addData("Clear", colorSensor.alpha());
            telemetry.addData("Red  ", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue ", colorSensor.blue());
            telemetry.addData("Hue", hsvValues[0]);

            // change the background color to match the color detected by the RGB sensor.
            // pass a reference to the hue, saturation, and value array as an argument
            // to the HSVToColor method.
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                }
            });

            telemetry.update();


        }

        tankDrive(-0.3, -0.3, 400);
        shooter(0.3, 0.3);

        sleep(2000);
        stopMotors();
        sleep(3000);
        stopMotors();
        tankDrive(-0.3, -0.3, 1500);
        tankDrive(-0.3, 0.3, 700);

        while (opModeIsActive() && !beacon_aligned) {
            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beacons.get(0).getListener()).getPose();

            if (pose != null) {
                VectorF translation = pose.getTranslation();

                telemetry.addData("Wheels translation", translation);
                telemetry.addData("Beacon Aligned: ", beacon_aligned);

                double degreesToTurn = Math.toDegrees(Math.atan2(translation.get(1), translation.get(2)));
                telemetry.addData("Wheels degrees", degreesToTurn);

                if (degreesToTurn >= -3) {

                    unlimitedDrive(0.3, -0.3);
                } else if (degreesToTurn <= 3) {
                    unlimitedDrive(-0.3, 0.3);
                } else {
                    beacon_aligned = true;
                    stopMotors();
                    tankDrive(0.3, 0.3, 1200);
                }
            }
            telemetry.update();

        }

        if ((colorSensor.red() == 360) && (colorSensor.green() == 0) && (colorSensor.blue() == 0)) {   //beacon detect
            servo(1, 0.9);
            tankDrive(-0.3, -0.3, 1000);
            wait(1500);
            tankDrive(0.3, 0.3, 1000);
            stopMotors();
        }
        if ((colorSensor.red() == 0) && (colorSensor.green() == 0) && (colorSensor.blue() == 360)) {
            servo(2, 0.9);
            tankDrive(-0.3, -0.3, 1000);
            wait(1500);
            tankDrive(0.3, 0.3, 1000);
            stopMotors();

        }
        tankDrive(-0.3,-0.3,1600);
    }


    private void servo(double servoNumber, double servoPosition) throws InterruptedException {
        if (servoNumber == 1) {
            servo1.setPosition(servoPosition);
        }
        if (servoNumber == 2) {
            servo2.setPosition(servoPosition);
        }
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

    private void elevator(double power) throws InterruptedException {
        elevator.setPower(power);


    }

    private void shooter(double leftpower, double rightpower) throws InterruptedException {
        shooterL.setPower(leftpower);
        shooterR.setPower(rightpower);


    }

    private void stopMotors() throws InterruptedException {
        shooterL.setPower(0);
        shooterR.setPower(0);
        elevator.setPower(0);
        tumbler.setPower(0);
        leftfrontMotor.setPower(0);
        leftbackMotor.setPower(0);
        rightfrontMotor.setPower(0);
        rightbackMotor.setPower(0);
        servo1.setPosition(0);
        servo2.setPosition(0);

    }

    private void unlimitedDrive(double l, double r) throws InterruptedException {
        leftfrontMotor.setPower(l); //set the according power to each motor
        leftbackMotor.setPower(l);
        rightfrontMotor.setPower(r);
        rightbackMotor.setPower(r);
    }


}
