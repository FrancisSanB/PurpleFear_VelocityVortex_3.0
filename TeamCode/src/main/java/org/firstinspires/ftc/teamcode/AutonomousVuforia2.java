package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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


/*
* Created by paco on 1/2/2017.
* Not sponsored in any way by pacogames.com
*
*
* Starts on red side
*/

public class AutonomousVuforia2 extends LinearOpMode {

    DcMotor leftback_motor;     //identify all of the tacos
    DcMotor rightback_motor;
    DcMotor leftfront_motor;
    DcMotor rightfront_motor;
    DcMotor tumbler;
    DcMotor shooterright;
    DcMotor shooterleft;
    DcMotor elevator;
    Servo beaconright;
    Servo beaconleft;

    boolean beacon_aligned = false;

    @Override
    public void runOpMode() throws InterruptedException {
        leftfront_motor = hardwareMap.dcMotor.get("leftfront_motor");
        leftback_motor = hardwareMap.dcMotor.get("leftback_motor");
        rightfront_motor = hardwareMap.dcMotor.get("rightfront_motor");
        rightback_motor = hardwareMap.dcMotor.get("rightback_motor");
        tumbler = hardwareMap.dcMotor.get("tublr");
        shooterleft = hardwareMap.dcMotor.get("shooterL");
        shooterright = hardwareMap.dcMotor.get("shooterR");
        elevator = hardwareMap.dcMotor.get("elevator");
        beaconright = hardwareMap.servo.get("bacon2");
        beaconleft = hardwareMap.servo.get("bacon");

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

        //vuforia(0);
        tankdrive(-0.3,-0.3, 400);
        tankdrive(0.3,-0.3, 90000000);
    }


    private void tankdrive(double leftY, double rightY, long sleepAmount) throws InterruptedException {
        rightY = -rightY;               //flip the power of the right side

        leftfront_motor.setPower(leftY); //set the according power to each motor
        leftback_motor.setPower(leftY);
        rightfront_motor.setPower(rightY);
        rightback_motor.setPower(rightY);

        sleep(sleepAmount);

        leftfront_motor.setPower(0); //set the according power to each motor
        leftback_motor.setPower(0);
        rightfront_motor.setPower(0);
        rightback_motor.setPower(0);

    }

    /*private void vuforia(int beaconNumber) throws InterruptedException {
        while (opModeIsActive() && !beacon_aligned) {
            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beacons.get(beaconNumber).getListener()).getPose();

            if (pose != null) {
                VectorF translation = pose.getTranslation();

                telemetry.addData("Gears translation", translation);
                telemetry.addData("YOU AINT GETTEN TO MCD'S EVER!", beacon_aligned);

                double degreesToTurn = Math.toDegrees(Math.atan2(translation.get(1), translation.get(2)));
                telemetry.addData("Gears degrees", degreesToTurn);

                if (degreesToTurn >= -3) {
                    leftback_motor.setPower(0.3);
                    leftfront_motor.setPower(0.3);
                    rightback_motor.setPower(-0.3);
                    rightfront_motor.setPower(-0.3);

                } else if (degreesToTurn <= 3) {
                    leftback_motor.setPower(-0.3);
                    leftfront_motor.setPower(-0.3);
                    rightback_motor.setPower(0.3);
                    rightfront_motor.setPower(0.3);

                } else {
                    beacon_aligned = true;
                    leftback_motor.setPower(0);
                    leftfront_motor.setPower(0);
                    rightback_motor.setPower(0);
                    rightfront_motor.setPower(0);
                    telemetry.addData("YOU'VE ARRIVED AT MCDONALDS", beacon_aligned);

                }
            }
            telemetry.update();
            telemetry.addData("It worked", beacon_aligned);
        }
    }*/
}

