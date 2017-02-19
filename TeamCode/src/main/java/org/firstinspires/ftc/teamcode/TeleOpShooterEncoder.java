package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.Timer;
import java.util.TimerTask;

/**
 * Created by Juan Pablo Martinez on 11/29/2015.
 */
//*******************************************************************************
//*******************************************************************************
//SHOOTER IS NOT WORKING + BOTH SERVO ARMS ARE NOT RETRACTING      -Edward  *****
//*******************************************************************************
//*******************************************************************************
public class TeleOpShooterEncoder extends MVMSTeleOpTelemetry {
    DcMotor leftback_motor;     //identify all of the motors
    DcMotor rightback_motor;
    DcMotor leftfront_motor;
    DcMotor rightfront_motor;
    DcMotor shooterR;
    DcMotor shooterL;
    Servo beaconServo1;
    Servo beaconServo2;
    DcMotor tumbler;

    int a = 1;
    boolean shooterDown = false;
    float tStart = System.currentTimeMillis();

    @Override
    public void init() {
        leftback_motor = hardwareMap.dcMotor.get("leftback_motor");     //link each motor to each
        leftfront_motor = hardwareMap.dcMotor.get("leftfront_motor");   //of the motors in the
        rightback_motor = hardwareMap.dcMotor.get("rightback_motor");   //configure file on the
        rightfront_motor = hardwareMap.dcMotor.get("rightfront_motor"); //phone
        shooterL = hardwareMap.dcMotor.get("shooterL");
        shooterR = hardwareMap.dcMotor.get("shooterR");
        tumbler = hardwareMap.dcMotor.get("tublr");
        beaconServo1 = hardwareMap.servo.get("Bacon");
        beaconServo2 = hardwareMap.servo.get("Bacon2");
        //elevator = hardwareMap.dcMotor.get("elevator");

    }

    @Override
    public void loop() {                        //create a loop where the code goes
        float rightY = gamepad1.right_stick_y;  //create a float based off of the y axis of the left
        float leftY = -gamepad1.left_stick_y;   //and right joysticks
        boolean in = gamepad1.left_bumper;
        boolean out = gamepad1.right_bumper;
        boolean shooter = gamepad1.a;
        boolean shooterBack = gamepad1.dpad_up;
        float tEnd = System.currentTimeMillis();
        float tDelta = tEnd - tStart;
        //float elapsedSeconds = tDelta / 1000.0;

        shooterL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        float rightEncoder = shooterR.getCurrentPosition();
        float leftEncoder = shooterL.getCurrentPosition();
        //float rightRatio = rightEncoder/elapsedSeconds;
        //float leftRatio = leftEncoder/elapsedSeconds;

        telemetry.addData("RightY", rightY);        //print out the current y axis of both joysticks
        telemetry.addData("LeftY", leftY);
        telemetry.addData("a =", a);
        telemetry.addData("tumbler out", out);
        telemetry.addData("tumbler in", in);
        telemetry.addData("tumbler", tumbler.getCurrentPosition());
        telemetry.addData("shooter", shooter);
        telemetry.addData("servo power", beaconServo1.getPosition());
        //telemetry.addData("shooter encoder values left", leftRatio);
        //telemetry.addData("shooter encoder values right", rightRatio);
        telemetry.addData("time", time);

        leftY = (float) scaleInput(leftY);      //use the scaleInput function on the power to scale
        rightY = (float) scaleInput(rightY);    //it

        leftback_motor.setPower(leftY);         //set the power to each corresponding motor
        rightback_motor.setPower(rightY);
        leftfront_motor.setPower(leftY);
        rightfront_motor.setPower(rightY);

        if (shooter) {
            if (!shooterDown) {
                a = a + 1;

            }
        }

        shooterDown = shooter;

        if(a % 2 == 0) {
            // even
            shooterL.setPower(1);
            shooterR.setPower(-1);

        } else {
            // odd
            shooterL.setPower(0);
            shooterR.setPower(0);

        }

        if (shooterBack) {
            shooterL.setPower(-1);
            shooterR.setPower(1);
        }

        if (!out && !in) {
            tumbler.setPower(0);
        }
        if(out) {
            tumbler.setPower(1);

        }
        if(in) {
            tumbler.setPower(-1);
        }

        if(in) {
            tumbler.setPower(-1);
        }

        if (gamepad1.b) {
            beaconServo2.setPosition(0.1);
        }
        if (gamepad1.x) {
            beaconServo1.setPosition(0.9);
        }
        if (gamepad1.y) {
            beaconServo1.setPosition(0.2);
            beaconServo2.setPosition(0.9);
        }

    }

    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };


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




