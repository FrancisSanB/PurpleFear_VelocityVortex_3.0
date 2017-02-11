package org.firstinspires.ftc.teamcode;

/**
 * Created by andre on 2/4/2017.
 */

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

import com.qualcomm.robotcore.util.ElapsedTime;
/*
 * Created by paco on 1/2/2017.
 * Not sponsored in any way by pacogames.com
 */

public class PacoVisionAutonomouse extends LinearOpMode {

    DcMotor leftback_motor;     //identify all of the tacos
    DcMotor rightback_motor;
    DcMotor leftfront_motor;
    DcMotor rightfront_motor;
    DcMotor tumbler;
    /*DcMotor shooterright;
    DcMotor shooterleft;
    DcMotor elevator;
    Servo beaconright;
    Servo beaconleft;*/
    ElapsedTime elapsedtime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException{

        leftfront_motor = hardwareMap.dcMotor.get("leftfront_motor");
        leftback_motor = hardwareMap.dcMotor.get("leftback_motor");
        rightfront_motor = hardwareMap.dcMotor.get("rightfront_motor");
        rightback_motor = hardwareMap.dcMotor.get("rightback_motor");
        tumbler = hardwareMap.dcMotor.get("tublr");
        /*shooterleft = hardwareMap.dcMotor.get("shooterL");
        shooterright = hardwareMap.dcMotor.get("shooterR");
        elevator = hardwareMap.dcMotor.get("elevator");
        beaconright = hardwareMap.servo.get("bacon2");
        beaconleft = hardwareMap.servo.get("bacon");*/

        waitForStart();
        tankdrive(-0.3,-0.3,2091);
        beaconpaco(3);
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

    private void drive(double leftY, double rightY) throws InterruptedException {
        rightY = -rightY;

        leftfront_motor.setPower(leftY); //set the according power to each motor
        leftback_motor.setPower(leftY);
        rightfront_motor.setPower(rightY);
        rightback_motor.setPower(rightY);

    }


    private void beaconpaco(int beaconnumber) throws InterruptedException {

        telemetry.addLine("Ok Siri, directions to McDonalds");

        boolean beacon_aligned = false;

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

        beacons.activate();

        telemetry.addLine("Starting google maps");
        elapsedtime.reset();
        while (opModeIsActive() && !beacon_aligned) {
            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beacons.get(beaconnumber).getListener()).getPose();
            if (pose != null) {
                VectorF translation = pose.getTranslation();

                telemetry.addData("translation", translation);
                telemetry.addData("atmcdonalds", beacon_aligned);
                telemetry.addData("time elapsed ms", elapsedtime.milliseconds());


                double degreesToTurn = Math.toDegrees(Math.atan2(translation.get(0), translation.get(2)));
                telemetry.addData("degrees", degreesToTurn);

                if (degreesToTurn <= 170 && degreesToTurn > 0) {
                    drive(0.3, -0.3);
                    telemetry.addLine("turning right");
                } else if (degreesToTurn >= -170 && degreesToTurn < 0) {
                    drive(-0.3,0.3);
                    telemetry.addLine("turning left");
                } else if (degreesToTurn < -170 || degreesToTurn > 170){
                    beacon_aligned = true;
                    drive(0,0);
                    telemetry.addLine("YOU'VE ARRIVED AT MCDONALDS");
                }
            }else {
                drive(-0.3,0.3);
            }
            telemetry.update();
            elapsedtime.reset();
        }
    }
}
