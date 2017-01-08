package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

/**
 * Created by pfernand on 2/17/2016.
 */
public class AutonomousColourSensor extends LinearOpMode{
    DcMotor leftfrontMotor;     //identify the motors and sensors
    DcMotor leftbackMotor;
    DcMotor rightfrontMotor;
    DcMotor rightbackMotor;

    ColorSensor sensorRGB;

    @Override
    public void runOpMode() throws InterruptedException {
        leftfrontMotor = hardwareMap.dcMotor.get("leftfront_motor");     //grab the configure file on the phone
        leftbackMotor = hardwareMap.dcMotor.get("leftback_motor");       //and compare it to the motors/sensors
        rightfrontMotor = hardwareMap.dcMotor.get("rightfront_motor");  //in the code
        rightbackMotor = hardwareMap.dcMotor.get("rightback_motor");

        // get a reference to our ColorSensor object.
        sensorRGB = hardwareMap.colorSensor.get("sensorColor");

        // turn the LED on in the beginning, just so user will know that the sensor is active.
        sensorRGB.enableLed(true);

        // wait one cycle.
        waitOneFullHardwareCycle();

        waitForStart();

        moveDistance(0.3, 0.3, 30);

        //moveDistanceColor(0.3, 0.3, 30);

        movetoBlue(0.3,0.3);

    }

    private void movetoBlue(double leftY, double rightY) throws InterruptedException {

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F,0F,0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;


        leftfrontMotor.setPower(leftY);
        leftbackMotor.setPower(leftY);
        rightfrontMotor.setPower(rightY);
        rightbackMotor.setPower(rightY);

        while(sensorRGB.blue() < 200) {
            //waiting
            telemetry.addData("right back motor",rightbackMotor.getCurrentPosition());
            telemetry.addData("left back motor",leftbackMotor.getCurrentPosition());

            // convert the RGB values to HSV values.
            //Color.RGBToHSV((sensorRGB.red() * 8), (sensorRGB.green() * 8), (sensorRGB.blue() * 8), hsvValues);
            Color.RGBToHSV(sensorRGB.red() * 8, sensorRGB.green() * 8, sensorRGB.blue() * 8, hsvValues);

            // send the info back to driver station using telemetry function.
            telemetry.addData("Clear", sensorRGB.alpha());
            telemetry.addData("Red  ", sensorRGB.red());
            telemetry.addData("Green", sensorRGB.green());
            telemetry.addData("Blue ", sensorRGB.blue());
            telemetry.addData("Hue", hsvValues[0]);

            // wait a hardware cycle before iterating.
            waitOneFullHardwareCycle();

        }
        leftfrontMotor.setPower(0.0);
        leftbackMotor.setPower(0.0);
        rightfrontMotor.setPower(0.0);
        rightbackMotor.setPower(0.0);
    }


    private void moveDistanceColor(double leftY, double rightY, int distance) throws InterruptedException {
        //distance is in inches

        final int ENCODER_CPR = 1440;
        final double GEAR_RATIO = 1;
        final int WHEEL_DIAMETER = 4;

        final double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
        final double ROTATIONS = distance/CIRCUMFERENCE;
        final double COUNTS = ENCODER_CPR*ROTATIONS*GEAR_RATIO;

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F,0F,0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(R.id.RelativeLayout);

        leftbackMotor.setTargetPosition((int) COUNTS);
        rightbackMotor.setTargetPosition((int) COUNTS);
        //one rotation is 1440

        leftfrontMotor.setPower(leftY);
        leftbackMotor.setPower(leftY);
        rightfrontMotor.setPower(rightY);
        rightbackMotor.setPower(rightY);

        while(rightbackMotor.getCurrentPosition() < COUNTS && leftbackMotor.getCurrentPosition() < COUNTS) {
            //waiting
            telemetry.addData("right back motor",rightbackMotor.getCurrentPosition());
            telemetry.addData("left back motor",leftbackMotor.getCurrentPosition());

            // convert the RGB values to HSV values.
            //Color.RGBToHSV((sensorRGB.red() * 8), (sensorRGB.green() * 8), (sensorRGB.blue() * 8), hsvValues);
            Color.RGBToHSV(sensorRGB.red() * 8, sensorRGB.green() * 8, sensorRGB.blue() * 8, hsvValues);

            // send the info back to driver station using telemetry function.
            telemetry.addData("Clear", sensorRGB.alpha());
            telemetry.addData("Red  ", sensorRGB.red());
            telemetry.addData("Green", sensorRGB.green());
            telemetry.addData("Blue ", sensorRGB.blue());
            telemetry.addData("Hue", hsvValues[0]);

            // change the background color to match the color detected by the RGB sensor.
            // pass a reference to the hue, saturation, and value array as an argument
            // to the HSVToColor method.
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                }
            });

            // wait a hardware cycle before iterating.
            waitOneFullHardwareCycle();


        }
        rightfrontMotor.setPower(0);
        leftfrontMotor.setPower(0);
    }

    private void moveDistance(double leftY, double rightY, int distance) throws InterruptedException {
        //distance is in inches

        final int ENCODER_CPR = 1440;
        final double GEAR_RATIO = 1;
        final int WHEEL_DIAMETER = 4;

        final double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
        final double ROTATIONS = distance/CIRCUMFERENCE;
        final double COUNTS = ENCODER_CPR*ROTATIONS*GEAR_RATIO;


        leftbackMotor.setTargetPosition((int) COUNTS);
        rightbackMotor.setTargetPosition((int) COUNTS);
        //one rotation is 1440

        leftfrontMotor.setPower(leftY);
        leftbackMotor.setPower(leftY);
        rightfrontMotor.setPower(rightY);
        rightbackMotor.setPower(rightY);

        while(rightbackMotor.getCurrentPosition() < COUNTS && leftbackMotor.getCurrentPosition() < COUNTS) {
            //waiting
            telemetry.addData("right back motor",rightbackMotor.getCurrentPosition());
            telemetry.addData("left back motor",leftbackMotor.getCurrentPosition());

        }
        rightfrontMotor.setPower(0);
        leftfrontMotor.setPower(0);
    }

    private void tankDrive(double leftY, double rightY) throws InterruptedException {
        rightY = -rightY;               //flip the power of the right side

        leftfrontMotor.setPower(leftY); //set the according power to each motor
        leftbackMotor.setPower(leftY);
        rightfrontMotor.setPower(rightY);
        rightbackMotor.setPower(rightY);

    }

    private void tankDrive(double leftY, double rightY, long sleepAmount) throws InterruptedException {
        tankDrive(leftY, rightY); //use the tankDrive function to add power

        sleep(sleepAmount);       //sleep for a certain amount of milliseconds

        tankDrive(0.0, 0.0);      //stop the motors

    }

}


