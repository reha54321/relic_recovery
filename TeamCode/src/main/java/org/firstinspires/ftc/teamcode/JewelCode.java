package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import java.lang.Runtime;
import 	java.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;


import com.qualcomm.robotcore.util.ElapsedTime;

import android.graphics.Color;
import com.qualcomm.robotcore.hardware.ColorSensor;
import android.app.Activity;
import android.view.View;
//package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.lang.Object;




@Autonomous(name="ColorSensorTest", group="Autonomous")

public class JewelCode extends OpMode {
    final boolean DEBUG = true;
    DcMotor leftBack;
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor rightBack;
    ColorSensor testSense;
    Servo continuousServo;
    String whichColor;
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void init(){
        continuousServo.setPosition(0.0);
    }
   
    

    @Override
    public void loop(){



        // bLedOn represents the state of the LED.
        boolean bLedOn = true;

        // turn the LED on in the beginning, just so user will know that the sensor is active.
        testSense.enableLed(bLedOn);

        testSense = hardwareMap.colorSensor.get("sensor_color");

        // convert the RGB values to HSV values.
        Color.RGBToHSV(testSense.red(), testSense.green(), testSense.blue(), hsvValues);

        // send the info back to driver station using telemetry function.
        telemetry.addData("LED", bLedOn ? "On" : "Off");
        telemetry.addData("Clear", testSense.alpha());
        telemetry.addData("Red ", testSense.red());
        telemetry.addData("Green", testSense.green());
        telemetry.addData("Blue ", testSense.blue());
        telemetry.addData("Hue", hsvValues[0]);
        // telemetry.addData("Is Red: ", (colorSensor.red()>200) && (colorSensor.blue()<100));
        // telemetry.addData("Is Blue: ", (colorSensor.blue()>200) && (colorSensor.red()<100));

        // change the background color to match the color detected by the RGB sensor.
        // pass a reference to the hue, saturation, and value array as an argument
        // to the HSVToColor method.
        //pass HSV values to Driver Station phones, thereby changing phone backgrounds to RGB color values

        telemetry.update();

        

        //assuming red team
        
       /* if(DEBUG){
            telemetry.addData("Text:", "isColor Data(4): " + isColor(colorSensor));
            //telemetry.addData("Text:", "RGB Colors: " + colorSensor.red() + ", " + colorSensor.green() + ", " + colorSensor.blue());
            telemetry.addData("Colors (R, B, G)", hsvValues[0]);
            
        }*/
    }





        

    public void checkColor(){
        if(testSense.red()>testSense.blue()){
            whichColor="red";

        }
        else{
            whichColor="blue";
        }

    }

    public void driveForward(int pos1, int pos2, int pos3, int pos4, double newTime) {
        if(newTime == 0) { //in loops like do whiles
            leftFront.setPower(pos1 / 10);
            leftBack.setPower(pos2 / 10);
            rightFront.setPower(pos3 / 10);
            rightBack.setPower(pos4 / 10);
        }

        else { //extra precaution
            double currentTime = runtime.milliseconds();
            while (newTime > 0 && runtime.milliseconds() <= (currentTime + (newTime * 100))) { //1s = 1000ms
                leftFront.setPower(pos1 / 10);
                leftBack.setPower(pos2 / 10);
                rightFront.setPower(pos3 / 10);
                rightBack.setPower(pos4 / 10);
                wait(5);

                //runtime.reset();
            }
        }
        }



        @Override
        public void start() {
               // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F,0F,0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.RelativeLayout);

        testSense.enableLed(false);

        //waitForStart();

        Color.RGBToHSV(testSense.red() * 8, testSense.green() * 8, testSense.blue() * 8, hsvValues);

        // hsvValues is an array that will hold the hue, saturation, and value information.


        // values is a reference to the hsvValues array.
            checkColor();
            leftFront = hardwareMap.dcMotor.get("leftFront");
            leftBack = hardwareMap.dcMotor.get("leftBack");
            rightFront = hardwareMap.dcMotor.get("rightFront");
            rightBack = hardwareMap.dcMotor.get("rightBack");

            testSense = hardwareMap.colorSensor.get("colorSensor");

            continuousServo = hardwareMap.servo.get("servo");

            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            leftFront.setDirection(DcMotor.Direction.REVERSE);
            leftBack.setDirection(DcMotor.Direction.REVERSE);

            continuousServo.setPosition(.5);
            driveForward(8,8,8,8,2);

            while(whichColor!="red"){
                //side to side until it finds red
                for(int i; i<2;i++){
                driveForward(8,-8,-8,8,2+i);
                driveForward(-16,16,16,-16,2+i);
                }
                driveForward(0,0,0,0,1);
            }
            if (whichColor=="red"){
                continuousServo.setPosition(1.0);
            }


        }
    public void wait(int time){
        try{
            Thread.sleep(time * 1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}

