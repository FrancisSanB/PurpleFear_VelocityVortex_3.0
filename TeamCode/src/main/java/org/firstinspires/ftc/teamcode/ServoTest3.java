package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Aakarsh on 1/21/2017.
 */
public class ServoTest3 extends LinearOpMode{

    Servo beaconRight;
    Servo beaconLeft;

    @Override
    public void runOpMode() throws InterruptedException {

        beaconRight = hardwareMap.servo.get("bacon");
        beaconLeft = hardwareMap.servo.get("bacon2");

        waitForStart();
        beaconRight.setPosition(0.2);
        beaconRight.setPosition(0.9);
        beaconLeft.setPosition(0.9);
        beaconLeft.setPosition(0.2);


    }
}
