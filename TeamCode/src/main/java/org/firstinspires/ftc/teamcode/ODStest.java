package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by Aakarsh on 1/8/2017.
 */
public class ODStest extends LinearOpMode{
    OpticalDistanceSensor odsSensor;  // Hardware Device Object

    @Override
    public void runOpMode() {

        // get a reference to our Light Sensor object.
        odsSensor = hardwareMap.opticalDistanceSensor.get("sensor_ods");

        // wait for the start button to be pressed.
        waitForStart();

        // while the op mode is active, loop and read the light levels.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {

            // send the info back to driver station using telemetry function.
            telemetry.addData("Raw",    odsSensor.getRawLightDetected()); //should be a # between 0 and 25
            telemetry.addData("Normal", odsSensor.getLightDetected());

            telemetry.update();
        }
    }
}