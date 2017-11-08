package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;


import com.qualcomm.robotcore.util.ElapsedTime;

import android.graphics.Color;
import com.qualcomm.robotcore.hardware.ColorSensor;
import android.app.Activity;
import android.view.View;


@Autonomous(name="ColorSensorTest", group="Autonomous")

public class JustColorSensor extends OpMode {
    ColorSensor testSense;
    @Override
   // @Override
    public void init(){
        //continuousServo.setPosition(0.0);
        testSense = hardwareMap.colorSensor.get("ballSensor");

    }

    public void loop(){
        boolean bLedOn = true;

        // turn the LED on in the beginning, just so user will know that the sensor is active.
        testSense.enableLed(bLedOn);

        telemetry.addData("LED", bLedOn ? "On" : "Off");
        telemetry.addData("Clear", testSense.alpha());
        telemetry.addData("Red ", testSense.red());
        telemetry.addData("Green", testSense.green());
        telemetry.addData("Blue ", testSense.blue());
        //telemetry.addData("Hue", hsvValues[0]);
        // telemetry.addData("Is Red: ", (colorSensor.red()>200) && (colorSensor.blue()<100));
        // telemetry.addData("Is Blue: ", (colorSensor.blue()>200) && (colorSensor.red()<100));

        // change the background color to match the color detected by the RGB sensor.
        // pass a reference to the hue, saturation, and value array as an argument
        // to the HSVToColor method.
        //pass HSV values to Driver Station phones, thereby changing phone backgrounds to RGB color values

        telemetry.update();  //hi reha

    }

    @Override
    public void start(){
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



        // bLedOn represents the state of the LED.



        // convert the RGB values to HSV values.
        Color.RGBToHSV(testSense.red(), testSense.green(), testSense.blue(), hsvValues);

        // send the info back to driver station using telemetry function.




        //assuming red team

       /* if(DEBUG){
            telemetry.addData("Text:", "isColor Data(4): " + isColor(colorSensor));
            //telemetry.addData("Text:", "RGB Colors: " + colorSensor.red() + ", " + colorSensor.green() + ", " + colorSensor.blue());
            telemetry.addData("Colors (R, B, G)", hsvValues[0]);

        }*/
    }
}

