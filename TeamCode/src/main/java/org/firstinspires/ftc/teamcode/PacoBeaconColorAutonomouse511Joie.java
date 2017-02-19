package org.firstinspires.ftc.teamcode;

/**
 * Created by andre on 2/4/2017.
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
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
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.util.ElapsedTime;
/*
* Created by paco on 1/2/2017.
* Not sponsored in any way by pacogames.com
* Stolen By Edmund on 2/11/2017
*/


public class PacoBeaconColorAutonomouse511Joie extends LinearOpMode {

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
    byte[] colorCcache;

    //ColorSensor colorSensor; //don't pu this in

    I2cDevice colorC;
    I2cDeviceSynch colorCreader;
    boolean LEDState = true;     //Tracks the mode of the color sensor; Active = true, Passive = false

    ///////////////8----
    double PosMaxDeg = 175;
    double NegMaxDeg = -175;

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
        colorC = hardwareMap.i2cDevice.get("cc");
        //colorSensor = hardwareMap.colorSensor.get("cc");
        colorCreader = new I2cDeviceSynchImpl(colorC, I2cAddr.create8bit(0x3c), false);

        colorCreader.engage();

        colorCreader.write8(3, 0);
        waitForStart();
        unlimitedDrive(-0.3, -0.3);
        while (opModeIsActive()) {
            colorCcache = colorCreader.read(0x04, 1);
            int colornumber = colorCcache[0] & 0xFF;

            //display values
            telemetry.addData("2 #C", colorCcache[0] & 0xFF);  //colorCcashe[0] is the color number

            telemetry.addData("4 A", colorCreader.getI2cAddress().get8Bit());

            telemetry.update();

            if (colornumber >= 14 && colornumber <= 16) {
                telemetry.update();
                stopMotors();
                telemetry.addLine("at white");
                break;
            }
        }
        tankdrive(-0.3, 0.3, 1700);
        tankdrive(0.3, 0.3, 500);
        beaconpaco(3);
        sleep(911);
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

    private void unlimitedDrive(double L, double R){
        R = -R;
        leftfront_motor.setPower(L); //set the according power to each motor
        leftback_motor.setPower(L);
        rightfront_motor.setPower(R);
        rightback_motor.setPower(R);

    }
    private void stopMotors() throws InterruptedException{
        leftfront_motor.setPower(0); //set the according power to each motor
        leftback_motor.setPower(0);
        rightfront_motor.setPower(0);
        rightback_motor.setPower(0);
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

                if (degreesToTurn <= PosMaxDeg && degreesToTurn > 0) {
                    unlimitedDrive(0.3, -0.3);
                    telemetry.addLine("turning right");
                } else if (degreesToTurn >= NegMaxDeg && degreesToTurn < 0) {
                    unlimitedDrive(-0.3,0.3);
                    telemetry.addLine("turning left");
                } else if (degreesToTurn < NegMaxDeg || degreesToTurn > PosMaxDeg){
                    beacon_aligned = true;
                    stopMotors();
                    telemetry.addLine("YOU'VE ARRIVED AT MCDONALDS");
                }
            }else {
                unlimitedDrive(-0.3,0.3);
                telemetry.addLine("Searching...");
            }
            telemetry.update();
            elapsedtime.reset();
        }
    }

}



