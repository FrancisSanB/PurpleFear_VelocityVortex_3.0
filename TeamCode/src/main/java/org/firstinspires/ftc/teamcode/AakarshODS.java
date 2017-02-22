/*
Modern Robotics ODS Encoder Example1
Created 9/20/2016 by Colton Mehlhoff of Modern Robotics using FTC SDK 2.2 Beta
Reuse permitted with credit where credit is due

Configuration:
Optical Distance Sensor named "ods1"

This program can be run without a battery and Power Destitution Module.

View the video about this at https://youtu.be/EuDYJPGOOPI.

For more information, visit modernroboticsedu.com.
Support is available by emailing support@modernroboticsinc.com.
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;


public class AakarshODS extends LinearOpMode {

    OpticalDistanceSensor ods1;
    DeviceInterfaceModule CDI;

    //sensor value between 0 and 1023
    int raw1;
   // int state = 0;
    //int count = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        ods1 = hardwareMap.opticalDistanceSensor.get("ods");
        CDI = hardwareMap.deviceInterfaceModule.get("Device Interface Module 1");


        waitForStart();
        while (opModeIsActive())

            raw1 = (int) (ods1.getLightDetected() * 1023);
            telemetry.addData("ODS", raw1);

       /* if(raw1 > 400){ //Adjust this test value to be half way between the white and black value for this sensor.
            CDI.setLED(0, true);
            if(state == 1)
                count++;
            state = 0;
        }else{
            CDI.setLED(0, false);
            if(state == 0)
                count++;
            state = 1;

        }*/


       // telemetry.addData("State", state);
       // telemetry.addData("Count", count);

        telemetry.update();


    }
}



