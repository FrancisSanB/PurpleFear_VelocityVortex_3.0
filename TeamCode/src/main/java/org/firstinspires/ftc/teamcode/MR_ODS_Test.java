package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

@TeleOp(name = "Wall Follow", group = "MRI")
//@Disabled
public class MR_ODS_Test extends LinearOpMode {

    //Instance of OpticalDistanceSensor
    OpticalDistanceSensor ODS;

    //Raw value is between 0 and 1
    double odsReadingRaw;

    // odsReadingRaw to the power of (-0.5)
    static double odsReadingLinear;

    //distance from bacon
    double distance;
    @Override
    public void runOpMode() throws InterruptedException {

        //identify the port of the ODS and motors in the configuration file
        ODS = hardwareMap.opticalDistanceSensor.get("ods");

        ODS.enableLed(true);

        waitForStart();

        while (opModeIsActive()) {

            odsReadingRaw = ODS.getRawLightDetected() / 25;                   //update raw value (This function now returns a value between 0 and 5 instead of 0 and 1 as seen in the video)
            odsReadingLinear = Math.pow(odsReadingRaw, -0.5);                //calculate linear value
            distance = odsReadingLinear*1.755568 - 0.7563;                      //calculate distance

            //The below two equations operate the motors such that both motors have the same speed when the robot is the right distance from the wall
            //As the robot gets closer to the wall, the left motor received more power and the right motor received less power
            //The opposite happens as the robot moves further from the wall. This makes a proportional and elegant wall following robot.
            //See the video explanation on the Modern Robotics YouTube channel, the ODS product page, or modernroboticsedu.com.
            //mLeft.setPower(odsReadingLinear * 2);
            //mRight.setPower(0.5 - (odsReadingLinear * 2));

            telemetry.addData("0 ODS Raw", odsReadingRaw);
            telemetry.addData("1 ODS linear", odsReadingLinear);
            telemetry.addData("2 ODS Distance", distance);
            //telemetry.addData("2 Motor Left", mLeft.getPower());
            //telemetry.addData("3 Motor Right", mRight.getPower());
            telemetry.update();
        }
    }
}
