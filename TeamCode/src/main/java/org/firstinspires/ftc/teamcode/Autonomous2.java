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

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;


import com.qualcomm.robotcore.util.ElapsedTime;
/*
* Created by paco on 1/2/2017.
* Not sponsored in any way by pacogames.com
* Stolen By Edmund on 2/11/2017
*/

public class Autonomous2 extends LinearOpMode {
    DcMotor leftback_motor;     //identify all of the tacos
    DcMotor rightback_motor;
    DcMotor leftfront_motor;
    DcMotor rightfront_motor;
    DcMotor tumbler;

    DcMotor shooterRight;
    DcMotor shooterLeft;
    Servo beaconright;
    Servo beaconleft;
    ElapsedTime elapsedtime = new ElapsedTime();
    byte[] colorCcache;
    byte[] colorAcache;
    ModernRoboticsI2cGyro gyro;
    OpticalDistanceSensor ods1;

    I2cDevice colorC;
    I2cDevice colorA;
    I2cDeviceSynch colorAreader;
    I2cDeviceSynch colorCreader;
    int raw1;



    boolean LEDState = true;     //Tracks the mod``.3
    // e of the color sensor; Active = true, Passive = false
    boolean Detect = false;
    int x = 0;

    @Override

    public void runOpMode() throws InterruptedException {
        leftfront_motor = hardwareMap.dcMotor.get("leftfront_motor");
        leftback_motor = hardwareMap.dcMotor.get("leftback_motor");
        rightfront_motor = hardwareMap.dcMotor.get("rightfront_motor");
        rightback_motor = hardwareMap.dcMotor.get("rightback_motor");
        tumbler = hardwareMap.dcMotor.get("tublr");
        shooterLeft = hardwareMap.dcMotor.get("shooterL");
        shooterRight = hardwareMap.dcMotor.get("shooterR");
        beaconright = hardwareMap.servo.get("Bacon2");
        beaconleft = hardwareMap.servo.get("Bacon");
        colorA = hardwareMap.i2cDevice.get("ca");
        colorC = hardwareMap.i2cDevice.get("cc");
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        ods1 = hardwareMap.opticalDistanceSensor.get("ods");

        colorAreader = new I2cDeviceSynchImpl(colorA, I2cAddr.create8bit(0x3a), false);
        colorCreader = new I2cDeviceSynchImpl(colorC, I2cAddr.create8bit(0x3c), false);

        colorAreader.engage();
        colorCreader.engage();

        colorCreader.write8(3, 0);
        colorAreader.write8(3, 0);

        gyro.calibrate();
        while (!isStopRequested() && gyro.isCalibrating()) {
            sleep(50);
            idle();
        }

        waitForStart();

        encoderDrive(-0.3, -0.3, -2);
        shooterDrive(1, 1);
        sleep(2000);
        tumblerDrive(1);
        sleep(3000);
        stopDCMotors();
        gyroTurnBlue(5);
        unlimitedDrive(-0.3, -0.3);
        borderColorBlind(13, 16, 69, 69, 2, 0);   //colorsensor 1 = A colorsensor 2 = C mode 0 = Active mode 1 = passive
        stopDCMotors();
        gyroTurnBlue(10);
        encoderDrive(0.3, 0.3, 1);
        beaconpaco(0);
        unlimitedDrive(-0.3, -0.3);

        while (opModeIsActive()) {
            int raw1;
            //int count;
            //count = 0;

            //display values
            //telemetry.addData("2 #C", colorCcache[0] & 0xFF);  //colorCcashe[0] is the color number
            //telemetry.addData("1 #A", colorAcache[0] & 0xFF);

            //telemetry.addData("4 C", colorCreader.getI2cAddress().get8Bit());
            //telemetry.addData("3 A", colorAreader.getI2cAddress().get8Bit());
            telemetry.addLine("stutter 1");
            telemetry.update();

            raw1 = (int) (ods1.getLightDetected() * 1023);
            unlimitedDrive(-0.3, -0.3);
            //telemetry.addData("ODS", raw1);
            if (raw1 >= 5) {
                stopDCMotors();
                break;
            }
        }

        borderColorBlind(9, 11, 2, 4, 1, 1);
        if (x == 1) {
            encoderDrive(0.3, 0.3, 1/2);
            beaconleft.setPosition(0.01);
            sleep(200);
            encoderDrive(-0.3, -0.3, -11 / 20);
        }
        if (x == 2) {
            encoderDrive(0.3, 0.3, 1 / 2);
            beaconright.setPosition(0.9);
            telemetry.addData("beaconright", beaconright.getPosition());
            sleep(200);
            encoderDrive(-0.3, -0.3, -11 / 20);
        }

        /*telemetry.addLine("stutter 2");
        telemetry.update();

        colorCcache = colorCreader.read(0x04, 1);
        colorAcache = colorAreader.read(0x04, 1);


        int colornumberA = colorAcache[0] & 0xFF;
        int colornumberC = colorCcache[0] & 0xFF;

        if ((colornumberA > 4 && colornumberA <11) || (colornumberA < 2) || (colornumberA > 11)) {
            telemetry.addLine("it can't see it");
            telemetry.update();
            sleep(5000);
        }

        if (colornumberA >= 9 && colornumberA <= 11 ) {
            telemetry.addLine("stutter 3");
            telemetry.update();
            encoderDrive(0.3, 0.3, 1/2);
            beaconleft.setPosition(0.01);
            sleep(200);
            encoderDrive(-0.3, -0.3, -11 / 20);

        }
        if (colornumberA >= 2 && colornumberA <= 4 ) {
            //  telemetry.addData("color Value", colornumberA);
            telemetry.addLine("stutter4");
            encoderDrive(0.3, 0.3, 1 / 2);
            beaconright.setPosition(0.9);
            telemetry.addData("beaconright", beaconright.getPosition());
            sleep(200);
            encoderDrive(-0.3, -0.3, -11 / 20);
            telemetry.update();

        }*/

        encoderDrive(0.3, 0.3, 1);
        gyroTurnBlue(160);
        encoderDrive(-0.3, -0.3, -4);


    }

    private void servo(int servoNumber) throws InterruptedException {
        if (servoNumber == 1) {
            beaconleft.setPosition(0.9);
        }

        if (servoNumber == 2) {
            beaconright.setPosition(0.1);
        }
    }
    private void encoderDrive(double leftY, double rightY, int distance) throws InterruptedException {
        distance = distance*1440;
        //telemetry.addData(">", "method called");

        leftback_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightback_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //reset the encoders
        leftback_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightback_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //tell it to start counting the position
        /*leftbackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightbackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/

        //set the distance
        leftback_motor.setTargetPosition(distance);
        rightback_motor.setTargetPosition(distance);

        //telemetry.addData(">","encoder values set");

        //set the power
        unlimitedDrive(leftY, rightY);

        while (opModeIsActive() && Math.abs(leftback_motor.getCurrentPosition()) < (Math.abs(distance) - 180) ) {
            telemetry.addData("right", rightback_motor.getCurrentPosition());
            telemetry.addData("left", leftback_motor.getCurrentPosition());
            telemetry.addData("distance", distance);
            telemetry.update();
            //wait until the position is reached
        }

        //telemetry.addData(">", "while loop called");
        //telemetry.addData("number of ticks right", rightbackMotor.getCurrentPosition());

        rightfront_motor.setPower(0);
        leftfront_motor.setPower(0);
        rightback_motor.setPower(0);
        leftback_motor.setPower(0);
    }

    private void tumblerDrive(double power) throws InterruptedException {
        tumbler.setPower(power);
    }

    private void shooterDrive(double leftpower, double rightpower) throws InterruptedException {
        shooterLeft.setPower(leftpower);
        shooterRight.setPower(-rightpower);

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

    private void gyroTurnBlue(double angleTurn) throws InterruptedException {
        int xVal, yVal, zVal = 0;
        int heading = 0;
        int angleZ = 0;
        int initial_angle = 0;
        int angle_difference = 0;
        boolean lastResetState = false;
        boolean curResetState = false;

        while (opModeIsActive()) {

            while (!isStarted()) {
                telemetry.addData(">", "Robot angle = %d", gyro.getIntegratedZValue());
                telemetry.update();
                idle();
            }
            gyro.resetZAxisIntegrator();

            initial_angle = gyro.getIntegratedZValue();
            telemetry.addData("1", "Int. Ang. %03d", angleZ);
            telemetry.update();

            while (angle_difference < angleTurn && angle_difference > -angleTurn) { //change angle to what’s needed
                unlimitedDrive(-0.3, 0.3);
                angleZ = gyro.getIntegratedZValue();
                telemetry.addData("1", "Int. Ang. %03d", angleZ);
                telemetry.update();
                angle_difference = angleZ - initial_angle;
            }
            stopDCMotors();
            break;
        }
    }


    private void unlimitedDrive(double L, double R) {
        leftfront_motor.setPower(L); //set the according power to each motor
        leftback_motor.setPower(L);
        rightfront_motor.setPower(-R);
        rightback_motor.setPower(-R);

    }

    private void stopServo() throws InterruptedException {
        beaconleft.setPosition(0.9);
        beaconright.setPosition(0);

    }

    private void stopDCMotors() throws InterruptedException {
        leftfront_motor.setPower(0); //set the according power to each motor
        leftback_motor.setPower(0);
        rightfront_motor.setPower(0);
        rightback_motor.setPower(0);

        tumbler.setPower(0);
        shooterLeft.setPower(0);
        shooterRight.setPower(0);
    }

    private void borderColorBlind(int ColorMin, int ColorMax, int mintwo,int maxtwo, int ColorSensorNumber, int mode) throws InterruptedException {
        int colornumber = 0;
        if (mode == 0) {
            colorCreader.write8(3, 0);
            colorAreader.write8(3, 0);
        } else if (mode == 1) {
            colorCreader.write8(3, 1);
            colorAreader.write8(3, 1);
        } else {
            telemetry.addLine("The mode you imputed in not valid!");
        }
        while (opModeIsActive() && Detect == false) {
            if (ColorSensorNumber == 2) {
                telemetry.addLine("Color Sensor C Initialized");
                colorCcache = colorCreader.read(0x04, 1);
                colornumber = colorCcache[0] & 0xFF;
                telemetry.addData("2 #C", colorCcache[0] & 0xFF);  //colorCcashe[0] is the color number
                telemetry.addData("4 A", colorCreader.getI2cAddress().get8Bit());
                telemetry.update();
            }else if (ColorSensorNumber == 1) {
                telemetry.addLine("Color Sensor A Initialized");
                colorAcache = colorAreader.read(0x04, 1);
                colornumber = colorAcache[0] & 0xFF;
                telemetry.addData("2 #C", colorAcache[0] & 0xFF);  //colorCcashe[0] is the color number
                telemetry.addData("4 A", colorAreader.getI2cAddress().get8Bit());
                telemetry.update();
            } else {
                telemetry.addLine("There was an Unexpected error when initializing your color sensor.  Please double check your code and make sure your color sensor number in your program is either 1 OR 2!");
                telemetry.update();
            }
            if(mintwo == 69 && maxtwo == 69){
                if (colornumber >= ColorMin && colornumber <= ColorMax) {
                    Detect = true;
                    telemetry.addLine("Warning! The Canadian Border Wall has been detected!!");
                    telemetry.update();

                    break;
                }
            }else if(mintwo != 69 && maxtwo != 69) {
                if (colornumber >= ColorMin && colornumber <= ColorMax) {
                    Detect = true;
                    telemetry.addLine("Warning! The Canadian Border Wall has been detected!!");
                    telemetry.update();
                    x = 1;
                    break;
                }
                if (colornumber >= mintwo && colornumber <= maxtwo) {
                    Detect = true;
                    telemetry.addLine("Warning! The Mexican Border Wall has been detected!!");
                    telemetry.update();
                    x = 2;
                    break;
                }
                telemetry.update();
            }
        }
    }
    private void resetcolor()throws InterruptedException{
        x = 0;
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

                int turnThreshhold = 175;
                telemetry.addData("translation", translation);
                telemetry.addData("atmcdonalds", beacon_aligned);
                telemetry.addData("time elapsed ms", elapsedtime.milliseconds());


                double degreesToTurn = Math.toDegrees(Math.atan2(translation.get(0), translation.get(2)));
                telemetry.addData("degrees", degreesToTurn);


                if (degreesToTurn <= turnThreshhold && degreesToTurn > 0) {
                    unlimitedDrive(0.3, -0.3);
                    telemetry.addLine("turning right");
                } else if (degreesToTurn >= -turnThreshhold && degreesToTurn < 0) {
                    unlimitedDrive(-0.3, 0.3);
                    telemetry.addLine("turning left");
                } else if (degreesToTurn < -turnThreshhold || degreesToTurn > turnThreshhold) {

                    beacon_aligned = true;
                    stopDCMotors();
                    telemetry.addLine("YOU'VE ARRIVED AT the Beacon");
                    telemetry.update();
                    sleep(1000);
                }
            } else {
                unlimitedDrive(-0.3, 0.3);
            }
            telemetry.update();
            elapsedtime.reset();
        }
    }
    private void ODS(int distance) throws InterruptedException{
        boolean stop = false;
        while(opModeIsActive() && stop == true) {
            raw1 = (int) (ods1.getLightDetected() * 1023);
            if (raw1 >= distance) {
                stop = true;
                break;

            }
        }
    }


}


