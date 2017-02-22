/*
Modern Robotics ODS Encoder Example1
        Created 9/20/2016 by Colton Mehlhoff of Modern Robotics using FTC SDK 2.2 Beta
        Reuse permitted with credit where credit is due

        Configuration:
        Optical Distance Sensor named "ods1"

        This program can be run without a battery and Power Destitution Module.

        View the video about this at https://youtu.be/EuDYJPGOOPI.

        For more information, visit modernroboticsedu.com.
        Support is available by emailing support@modernroboticsinc.com.
*/

        package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
        import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;


public class AakarshODS extends LinearOpMode {

    OpticalDistanceSensor ods1;
    //DeviceInterfaceModule CDI;
    DcMotor leftfrontm;
    DcMotor leftbackm;
    DcMotor rightfrontm;
    DcMotor rightbackm;


    //sensor value between 0 and 1023
    int raw1;
    //int state = 0;
    int count = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        ods1 = hardwareMap.opticalDistanceSensor.get("ods");
        //CDI = hardwareMap.deviceInterfaceModule.get("Device Interface Module 1");
        leftfrontm = hardwareMap.dcMotor.get("leftfront_motor");
        rightfrontm = hardwareMap.dcMotor.get("rightfront_motor");
        leftbackm = hardwareMap.dcMotor.get("leftback_motor");
        rightbackm = hardwareMap.dcMotor.get("rightback_motor");

        waitForStart();
        while (opModeIsActive() && count == 0) {
            raw1 = (int) (ods1.getLightDetected() * 1023);
           // telemetry.addData("ODS", raw1);
              unlimitedDrive(-0.3, -0.3);
            if (raw1 >= 5) {
                break;
            }




           // telemetry.update();

        }
    }





           /* CDI.setLED(0, true);
            if (state == 1)
                count++;
            state = 0;

            CDI.setLED(0, false);
            if(state == 0)
                count++;
            state = 1;*/


        // telemetry.addData("State", state);
        // telemetry.addData("Count", count);

    private void tankdrive(double l, double r, long sleepAmount) throws InterruptedException {

        r = -r;

        leftbackm.setPower(l);
        leftfrontm.setPower(l);
        rightfrontm.setPower(r);
        rightbackm.setPower(r);

        sleep(sleepAmount);

        leftbackm.setPower(0);
        leftfrontm.setPower(0);
        rightfrontm.setPower(0);
        rightbackm.setPower(0);
    }

    private void unlimitedDrive(double L, double R) throws InterruptedException {

        R = -R;

        leftfrontm.setPower(L); //set the according power to each motor
        leftbackm.setPower(L);
        rightfrontm.setPower(R);
        rightbackm.setPower(R);
    }
        private void stopMotors() throws InterruptedException {
            leftfrontm.setPower(0); //set the according power to each motor
            leftbackm.setPower(0);
            rightfrontm.setPower(0);
            rightbackm.setPower(0);
        }





    }




