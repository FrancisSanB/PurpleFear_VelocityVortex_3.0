package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Aakarsh on 1/22/2017.
 */
public class TeleOp extends MVMSTeleOpTelemetry {
    DcMotor leftfrontMotor;
    DcMotor leftbackMotor;
    DcMotor rightbackMotor;
    DcMotor rightfrontMotor;
    DcMotor tumbler;
    DcMotor shooterR;
    DcMotor shooterL;
    Servo beaconServo;

    @Override
    public void init() {
        leftbackMotor = hardwareMap.dcMotor.get("leftback_motor");
        leftfrontMotor = hardwareMap.dcMotor.get("leftfront_motor");
        rightbackMotor = hardwareMap.dcMotor.get("rightback_motor");
        rightfrontMotor = hardwareMap.dcMotor.get("rightfront_motor");
        shooterL = hardwareMap.dcMotor.get("shooterL");
        shooterR = hardwareMap.dcMotor.get("shooterR");
        tumbler = hardwareMap.dcMotor.get("tublr");
        beaconServo = hardwareMap.servo.get("Bacon");

    }

    @Override
    public void loop() {
        float rightY = gamepad1.right_stick_y;  //create a float based off of the y axis of the left
        float leftY = -gamepad1.left_stick_y;   //and right joysticks
        float elevatorUp = gamepad1.right_trigger;
        float elevatorDown = gamepad1.left_trigger;
        boolean in = gamepad1.left_bumper;
        boolean out = gamepad1.right_bumper;
        boolean shooter = gamepad1.a;
        boolean beacon = gamepad1.b;

        telemetry.addData("RightY", rightY);        //print out the current y axis of both joysticks
        telemetry.addData("LeftY", leftY);
        telemetry.addData("out", out);
        telemetry.addData("in", in);
        telemetry.addData("shooter", shooter);
        telemetry.addData("elevatorUp", elevatorUp);
        telemetry.addData("elevatorDown", elevatorDown);
        telemetry.addData("beacon", beacon);
        telemetry.addData("servo power", beaconServo.getPosition());

        leftY = (float) scaleInput(leftY);      //use the scaleInput function on the power to scale
        rightY = (float) scaleInput(rightY);    //it
        leftbackMotor.setPower(leftY);         //set the power to each corresponding motor
        rightbackMotor.setPower(rightY);
        leftfrontMotor.setPower(leftY);
        rightfrontMotor.setPower(rightY);



        if(gamepad1.x) {
            beaconServo.setPosition(0.8);
        }
        if (gamepad1.y) {
            beaconServo.setPosition(0.2);
        }

    }

    double scaleInput(double dVal) {
        double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};


        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);


        // index should be positive.
        if (index < 0) {
            index = -index;
        }


        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }


        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }


        // return scaled value.
        return dScale;
    }
}
