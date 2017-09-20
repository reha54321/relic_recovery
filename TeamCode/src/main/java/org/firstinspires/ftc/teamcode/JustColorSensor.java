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

public class ColorSensorTest extends OpMode {
    ColorSensor testSense;
    @Override
    public void loop{
        if(DEBUG){
            telemetry.addData("Text:", "isColor Data(4): " + isColor(colorSensor));
            //telemetry.addData("Text:", "RGB Colors: " + colorSensor.red() + ", " + colorSensor.green() + ", " + colorSensor.blue());
            telemetry.addData("Colors (R, B, G)", hsvValues[0]);
        }
    }

    @Override
    public void runOpMode{
        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F,0F,0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.RelativeLayout);

        colorSensor.enableLed(false);

        waitForStart();

        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

        telemetry.update();

    }
    
    
       //SENSOR CODE
   public void sense(){
       // hsvValues is an array that will hold the hue, saturation, and value information.
       float hsvValues[] = {0F,0F,0F};

       // values is a reference to the hsvValues array.
       final float values[] = hsvValues;

       // get a reference to the RelativeLayout so we can change the background
       // color of the Robot Controller app to match the hue detected by the RGB sensor.
       final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(R.id.RelativeLayout);
    

       // bLedOn represents the state of the LED.
       boolean bLedOn = true;

       // turn the LED on in the beginning, just so user will know that the sensor is active.
       colorSensor.enableLed(bLedOn);

       colorSensor = hardwareMap.colorSensor.get("sensor_color");

       // convert the RGB values to HSV values.
       Color.RGBToHSV(colorSensor.red(), colorSensor.green(), colorSensor.blue(), hsvValues);

       // send the info back to driver station using telemetry function.
       telemetry.addData("LED", bLedOn ? "On" : "Off");
       telemetry.addData("Clear", colorSensor.alpha());
       telemetry.addData("Red ", colorSensor.red());
       telemetry.addData("Green", colorSensor.green());
       telemetry.addData("Blue ", colorSensor.blue());
       telemetry.addData("Hue", hsvValues[0]);
      // telemetry.addData("Is Red: ", (colorSensor.red()>200) && (colorSensor.blue()<100));
      // telemetry.addData("Is Blue: ", (colorSensor.blue()>200) && (colorSensor.red()<100));
    
       // change the background color to match the color detected by the RGB sensor.
       // pass a reference to the hue, saturation, and value array as an argument
       // to the HSVToColor method.
       //pass HSV values to Driver Station phones, thereby changing phone backgrounds to RGB color values
   }


   public void wait(int time){
       try{
           Thread.sleep(time * 1000);
       } catch (InterruptedException e) {
           e.printStackTrace();
       }
   }
}

