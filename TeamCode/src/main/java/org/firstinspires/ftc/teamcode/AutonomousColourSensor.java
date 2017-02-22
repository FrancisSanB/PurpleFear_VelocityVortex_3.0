package org.firstinspires.ftc.teamcode;


import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;


/**
 * Created by Aakarsh on 1/29/2017.
 */
public class AutonomousColourSensor extends LinearOpMode {

    DcMotor leftbackmotor;     //identify all of the tacos
    DcMotor rightbackmotor;
    DcMotor leftfrontmotor;
    DcMotor rightfrontmotor;
    DcMotor tumbler;
    DcMotor shooterRight;
    DcMotor shooterLeft;
   // DcMotor elevator;
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

        if (ColorSensor.red() == 300 && ColorSensor.blue() == 0 && ColorSensor.green() == 0) {//beacon detect
            telemetry.addLine("red");
        }
        if (ColorSensor.blue() == 300 && ColorSensor.blue() == 0 && ColorSensor.green() == 0) {
            telemetry.addLine("blue");
        }

    }



    }
