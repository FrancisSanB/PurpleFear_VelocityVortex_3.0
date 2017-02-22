package org.firstinspires.ftc.teamcode;

/**
 * Created by andre on 2/4/2017.
 * * Created by paco on 1/2/2017.
 * Not sponsored in any way by pacogames.com
 * Stolen By Edmund on 2/11/2017
 * Stolen by Francis on 2/20/2017
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


public class AutonomousVuforia extends LinearOpMode {

    DcMotor leftback_motor;     //identify all of the tacos
    DcMotor rightback_motor;
    DcMotor leftfront_motor;
    DcMotor rightfront_motor;
    DcMotor tumbler;
    DcMotor shooterRight;
    DcMotor shooterLeft;
    Servo beaconright;
    Servo beaconleft;
    OpticalDistanceSensor ODS;
    ModernRoboticsI2cGyro gyro;
    ElapsedTime elapsedtime = new ElapsedTime();
    byte[] colorCcache;
    byte[] colorAcache;


    //Raw value is between 0 and 1
    double odsReadingRaw;

    // odsReadingRaw to the power of (-0.5)
    static double odsReadingLinear;

    //distance from bacon
    double distance;
    I2cDevice colorC;
    I2cDevice colorA;
    I2cDeviceSynch colorAreader;
    I2cDeviceSynch colorCreader;


    boolean LEDState = true;     //Tracks the mode of the color sensor; Active = true, Passive = false


    @Override
    public void runOpMode() throws InterruptedException {
        leftfront_motor = hardwareMap.dcMotor.get("leftfront_motor");
        leftback_motor = hardwareMap.dcMotor.get("leftback_motor");
        rightfront_motor = hardwareMap.dcMotor.get("rightfront_motor");
        rightback_motor = hardwareMap.dcMotor.get("rightback_motor");
        tumbler = hardwareMap.dcMotor.get("tublr");
        shooterLeft = hardwareMap.dcMotor.get("shooterL");
        shooterRight = hardwareMap.dcMotor.get("shooterR");
        beaconright = hardwareMap.servo.get("bacon2");
        beaconleft = hardwareMap.servo.get("bacon");
        colorC = hardwareMap.i2cDevice.get("cc");
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");


        colorCreader = new I2cDeviceSynchImpl(colorC, I2cAddr.create8bit(0x3c), false);

        colorCreader.engage();

        colorCreader.write8(3, 0);

        gyro.calibrate();
        while (!isStopRequested() && gyro.isCalibrating()) {
            sleep(50);
            idle();
        }


        waitForStart();

        while (opModeIsActive()) {

            colorCcache = colorCreader.read(0x04, 1);
            int colornumber = colorCcache[0] & 0xFF;

            //display values
            telemetry.addData("2 #C", colorCcache[0] & 0xFF);  //colorCcashe[0] is the color number

            telemetry.addData("4 A", colorCreader.getI2cAddress().get8Bit());

            telemetry.update();

            unlimitedDrive(-0.3, -0.3);
            if (colornumber >= 13 && colornumber <= 16) {
                stopMotors();
                beaconpaco(3);
            }
            gyroTurn(120);
            encoderDrive(0.3, 0.3, 3);
            beaconpaco(3);
            sleep(911);
            while (opModeIsActive()) {
                colorCcache = colorCreader.read(0x04, 1);
                colorAcache = colorAreader.read(0x04, 1);


                int colornumberA = colorAcache[0] & 0xFF;
                int colornumberC = colorCcache[0] & 0xFF;
                //display values
                telemetry.addData("2 #C", colorCcache[0] & 0xFF);  //colorCcashe[0] is the color number
                telemetry.addData("1 #A", colorAcache[0] & 0xFF);

                telemetry.addData("4 C", colorCreader.getI2cAddress().get8Bit());
                telemetry.addData("3 A", colorAreader.getI2cAddress().get8Bit());

                //  unlimitedDrive(-0.3, -0.3);
                if (colornumberA >= 9 && colornumberA <= 11) {
                    unlimitedDrive(0.3, 0.3);
                    ODS(8);
                    stopMotors();
                    beaconleft.setPosition(0.01);
                    unlimitedDrive(-0.3, -0.3);
                    ODS(3.8);
                    stopMotors();
                    encoderDrive(0.3, 0.3, 8);
                }
                if (colornumberA >= 2 && colornumberA <= 4) {
                    unlimitedDrive(0.3, 0.3);
                    ODS(8);
                    stopMotors();
                    beaconright.setPosition(0.99);
                    unlimitedDrive(-0.3, -0.3);
                    ODS(3.8);
                    stopMotors();
                    encoderDrive(0.3, 0.3, 8);
                }
                gyroTurn(-40);
                encoderDrive(0.3,0.3,18);
                shooterDrive(1,1);
                sleep(3000);
                tumblerDrive(1);
                sleep(4000);
                stopMotors();
            }


        }
    }

    private void servoDrive (int servoNumber) throws InterruptedException {
        if (servoNumber == 1) {
            beaconleft.setPosition(0.9);
        }

        if (servoNumber == 2) {
            beaconright.setPosition(0.1);
        }

    }

    private void encoderTankDrive(double leftY, double rightY) throws InterruptedException {
        rightY = -rightY;

        leftfront_motor.setPower(leftY); //set the according power to each motor
        leftfront_motor.setPower(leftY);
        rightfront_motor.setPower(rightY);
        rightback_motor.setPower(rightY);

    }

    private void encoderDrive(double leftY, double rightY, int distance) throws InterruptedException {
        distance = distance*1440;
        //telemetry.addData(">", "method called");

        leftfront_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightback_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //reset the encoders
        leftfront_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightback_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //tell it to start counting the position
       /*leftfront_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       rightback_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/

        //set the distance
        leftfront_motor.setTargetPosition(distance);
        rightback_motor.setTargetPosition(distance);

        //telemetry.addData(">","encoder values set");

        //set the power
        encoderTankDrive(leftY, rightY);

        while (opModeIsActive() && leftfront_motor.getCurrentPosition() < (distance - 180) && rightback_motor.getCurrentPosition() < (distance - 180)) {
           /*telemetry.addData("number of ticks right", rightback_motor.getCurrentPosition());
           telemetry.addData("number of ticks left", leftfront_motor.getCurrentPosition());
           telemetry.update();*/
            //wait until the position is reached
        }

        //telemetry.addData(">", "while loop called");
        //telemetry.addData("number of ticks right", rightback_motor.getCurrentPosition());

        rightfront_motor.setPower(0);
        leftfront_motor.setPower(0);

    }

    private void tumblerDrive(double power) throws InterruptedException {
        tumbler.setPower(power);
    }
    private void ODS(double dis) throws InterruptedException {
        while(opModeIsActive()) {
            odsReadingRaw = ODS.getRawLightDetected() / 25;                   //update raw value (This function now returns a value between 0 and 5 instead of 0 and 1 as seen in the video)
            odsReadingLinear = Math.pow(odsReadingRaw, -0.5);                //calculate linear value
            distance = odsReadingLinear * 1.755568 - 0.7563;                      //calculate distance

            //The below two equations operate the motors such that both motors have the same speed when the robot is the right distance from the wall
            //As the robot gets closer to the wall, the left motor received more power and the right motor received less power
            //The opposite happens as the robot moves further from the wall. This makes a proportional and elegant wall following robot.
            //See the video explanation on the Modern Robotics YouTube channel, the ODS product page, or modernroboticsedu.com.
            //mLeft.setPower(odsReadingLinear * 2);
            //mRight.setPower(0.5 - (odsReadingLinear * 2));

            telemetry.addData("0 ODS Raw", odsReadingRaw);
            telemetry.addData("1 ODS linear", odsReadingLinear);
            telemetry.addData("2 ODS Distance", distance);
            telemetry.update();
            //telemetry.addData("2 Motor Left", mLeft.getPower());
            //telemetry.addData("3 Motor Right", mRight.getPower());
            while(distance != dis){
            }
        }
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
    private void gyroTurn (double angleTurn) throws InterruptedException {
        int xVal, yVal, zVal = 0;
        int heading = 0;
        int angleZ = 0;
        int initial_angle = 0;
        int angle_difference = 0;
        boolean lastResetState = false;
        boolean curResetState = false;
        gyro.calibrate();
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

            while (angle_difference < angleTurn && -angle_difference > angleTurn) { //change angle to what’s needed
                unlimitedDrive(-0.3, 0.3);
                angleZ = gyro.getIntegratedZValue();
                telemetry.addData("1", "Int. Ang. %03d", angleZ);
                telemetry.update();
                angle_difference = angleZ - initial_angle;
            }
            stopMotors();
        }
    }

    private void unlimitedDrive(double L, double R){
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

                if (degreesToTurn <= 170 && degreesToTurn > 0) {
                    unlimitedDrive(0.3, -0.3);
                    telemetry.addLine("turning right");
                } else if (degreesToTurn >= -170 && degreesToTurn < 0) {
                    unlimitedDrive(-0.3,0.3);
                    telemetry.addLine("turning left");
                } else if (degreesToTurn < -170 || degreesToTurn > 170){
                    beacon_aligned = true;
                    stopMotors();
                    wait(1500);
                    tankdrive(-0.3,-0.3,1000);
                    telemetry.addLine("YOU'VE ARRIVED AT the Beacon");

                }
            }else {
                unlimitedDrive(-0.3,0.3);
            }
            telemetry.update();
            elapsedtime.reset();
        }
    }

}



