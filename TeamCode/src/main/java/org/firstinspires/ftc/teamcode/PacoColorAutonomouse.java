package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by andre on 2/9/2017.
 */

public class PacoColorAutonomouse extends OpMode{
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    byte[] colorCcache;

    I2cDevice colorC;
    I2cDeviceSynch colorCreader;

    DcMotor leftfrontMotor;     //identify the motors and sensors
    DcMotor leftbackMotor;
    DcMotor rightfrontMotor;
    DcMotor rightbackMotor;



    boolean LEDState = true;     //Tracks the mode of the color sensor; Active = true, Passive = false

    @Override
    public void init() {
        runtime.reset();

        leftfrontMotor = hardwareMap.dcMotor.get("leftfront_motor");     //grab the configure file on the phone
        leftbackMotor = hardwareMap.dcMotor.get("leftback_motor");       //and compare it to the motors/sensors
        rightfrontMotor = hardwareMap.dcMotor.get("rightfront_motor");  //in the code
        rightbackMotor = hardwareMap.dcMotor.get("rightback_motor");

        telemetry.addData("Status", "Initialized");

        //the below lines set up the configuration file
        colorC = hardwareMap.i2cDevice.get("cc");

        colorCreader = new I2cDeviceSynchImpl(colorC, I2cAddr.create8bit(0x3c), false);

        colorCreader.engage();

        colorCreader.write8(3, 0);


    }
    public void start() {

    }

    private void colorSensor(int pacoMin, int pacoMax){
        
    }
    @Override
    public void loop() {


        telemetry.addData("Status", "Running: " + runtime.toString());
        //The below two if() statements ensure that the mode of the color sensor is changed only once each time the touch sensor is pressed.
        //The mode of the color sensor is saved to the sensor's long term memory. Just like flash drives, the long term memory has a life time in the 10s or 100s of thousands of cycles.
        //This seems like a lot but if your program wrote to the long term memory every time though the main loop, it would shorten the life of your sensor.


        colorCcache = colorCreader.read(0x04, 1);
        int colornumber = colorCcache[0] & 0xFF;
        if (colornumber >= 13){

        }
        //display values
        telemetry.addData("2 #C", colorCcache[0] & 0xFF);  //colorCcashe[0] is the color number

        telemetry.addData("4 A", colorCreader.getI2cAddress().get8Bit());

        telemetry.update();
    }

    @Override
    public void stop() {
    }


}
