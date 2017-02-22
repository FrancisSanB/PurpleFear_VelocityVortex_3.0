package org.firstinspires.ftc.teamcode;

/**
 * Created by andre on 2/4/2017.
 * Created again by paco on 1/2/2017.
 * Not sponsored in any way by pacogames.com
 * Stolen By Edmund on 2/11/2017
 * stolen again by Aakarsh on 2/11/17
 * stolen one last time by Francis on 2/20/17
 * hahahaha it's all mine now!!!
 */

<<<<<<< HEAD
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
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
* stolen again by Aakarsh on 2/11/17
*/
=======
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

import com.qualcomm.robotcore.util.ElapsedTime;
>>>>>>> 9a9097bbdafd616d46ec5a8a1f761f49eaf9aec7

public class autonomousA extends LinearOpMode {

    DcMotor leftback_motor;     //identify all of the tacos
    DcMotor rightback_motor;
    DcMotor leftfront_motor;
    DcMotor rightfront_motor;
    DcMotor tumbler;
    //DcMotor shooterright;
    //DcMotor shooterleft;
    //DcMotor elevator;
    Servo beaconright;
    Servo beaconleft;
    ElapsedTime elapsedtime = new ElapsedTime();
    byte[] colorCcache;
    byte[] colorAcache;

    I2cDevice colorC;
    I2cDevice colorA;
    I2cDeviceSynch colorAreader;
    I2cDeviceSynch colorCreader;
    OpticalDistanceSensor ods1;

    boolean LEDState = true; //Tracks the mode of the color sensor; Active = true, Passive = false


    @Override
    public void runOpMode() throws InterruptedException {
        leftfront_motor = hardwareMap.dcMotor.get("leftfront_motor");
        leftback_motor = hardwareMap.dcMotor.get("leftback_motor");
        rightfront_motor = hardwareMap.dcMotor.get("rightfront_motor");
        rightback_motor = hardwareMap.dcMotor.get("rightback_motor");
        tumbler = hardwareMap.dcMotor.get("tublr");
        //shooterleft = hardwareMap.dcMotor.get("shooterL");
        //hooterright = hardwareMap.dcMotor.get("shooterR");
        // elevator = hardwareMap.dcMotor.get("elevator");
        beaconright = hardwareMap.servo.get("Bacon");
        beaconleft = hardwareMap.servo.get("Bacon2");
        colorC = hardwareMap.i2cDevice.get("cc");
        colorA = hardwareMap.i2cDevice.get("ca");
        ods1 = hardwareMap.opticalDistanceSensor.get("ods");

        colorCreader = new I2cDeviceSynchImpl(colorC, I2cAddr.create8bit(0x3c), false);
        colorAreader = new I2cDeviceSynchImpl(colorA, I2cAddr.create8bit(0x3a), false);


        colorCreader.engage();
        colorAreader.engage();

        colorCreader.write8(3, 1);
        colorAreader.write8(3, 1);


        elapsedtime.reset();

        waitForStart();
        while (opModeIsActive()) {
            colorCcache = colorCreader.read(0x04, 1);
            colorAcache = colorAreader.read(0x04, 1);


            int colornumberA = colorAcache[0] & 0xFF;
            int colornumberC = colorCcache[0] & 0xFF;
            int raw1;
            int count = 0;

            //display values
            telemetry.addData("2 #C", colorCcache[0] & 0xFF);  //colorCcashe[0] is the color number
            telemetry.addData("1 #A", colorAcache[0] & 0xFF);

            telemetry.addData("4 C", colorCreader.getI2cAddress().get8Bit());
            telemetry.addData("3 A", colorAreader.getI2cAddress().get8Bit());

<<<<<<< HEAD
            raw1 = (int) (ods1.getLightDetected() * 1023);
            telemetry.addData("ODS", raw1);
            unlimitedDrive(-0.3, -0.3);
            if (raw1 >= 5) {
                stopMotors();
                count = 1;
            }

            if (colornumberA >= 9 && colornumberA <= 11 && count == 1) {
                encoderDrive(0.3, 0.3, 1/2);
                beaconleft.setPosition(0.01);
                encoderDrive(-0.3, -0.3, -11/20);
                break;
            }
            if (colornumberA >= 2 && colornumberA <= 4 && count == 1) {
                encoderDrive(0.3, 0.3, 1/2);
                beaconright.setPosition(0.99);
                encoderDrive(-0.3, -0.3, -11/20);
                break;

            }
            telemetry.addData("color Value", colornumberA);
            telemetry.update();

        }
    }
=======
            //  unlimitedDrive(-0.3, -0.3);
            if (colornumberA >= 9 && colornumberA <= 11) {
                tankdrive(0.3, 0.3, 1000);
                beaconleft.setPosition(0.01);
                tankdrive(-0.3, -0.3, 1000);
                break;
            }
            if (colornumberA >= 2 && colornumberA <= 4) {
                tankdrive(0.3, 0.3, 1000);
                beaconright.setPosition(0.99);
                tankdrive(-0.3, -0.3, 1000);
                break;
            }
        }
    }




>>>>>>> 9a9097bbdafd616d46ec5a8a1f761f49eaf9aec7


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

    private void unlimitedDrive(double L, double R) {
        R = -R;

        leftfront_motor.setPower(L); //set the according power to each motor
        leftback_motor.setPower(L);
        rightfront_motor.setPower(R);
        rightback_motor.setPower(R);

    }

    private void stopMotors() throws InterruptedException {
        leftfront_motor.setPower(0); //set the according power to each motor
        leftback_motor.setPower(0);
        rightfront_motor.setPower(0);
        rightback_motor.setPower(0);
    }



    private void encoderDrive(double leftY, double rightY, int distance) throws InterruptedException {
        distance = distance * 1440;

        //reset the encoders
        leftback_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightback_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set the distance

        //tell it to start counting the position
        leftback_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightback_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftback_motor.setTargetPosition(distance);
        rightback_motor.setTargetPosition(distance);

        //set the power
        unlimitedDrive(leftY, rightY);

        while (opModeIsActive() && leftback_motor.getCurrentPosition() < distance - 180 && rightback_motor.getCurrentPosition() < distance - 180) {
            //wait until the position is reached

        }

    }
}
<<<<<<< HEAD





=======
>>>>>>> 9a9097bbdafd616d46ec5a8a1f761f49eaf9aec7



