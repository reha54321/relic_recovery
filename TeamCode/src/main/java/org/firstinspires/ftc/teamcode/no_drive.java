package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOp Only Lift", group = "Tele Op")
public class no_drive extends OpMode {

    DcMotor lift;

    Servo rightArm;
    Servo leftArm;

    @Override
    public void init() {

        lift = hardwareMap.dcMotor.get("lift");

        rightArm = hardwareMap.servo.get("right arm");
        leftArm = hardwareMap.servo.get("left arm");

    }

    public float rightPos = 1;
    public float leftPos = 0;

    public float rightMax;
    public float leftMax;

    @Override
    public void loop() {


        float gamepad2LeftY = gamepad2.left_stick_y;
        float gamepad2RightY = gamepad2.right_stick_y;

        lift.setPower(-gamepad2LeftY * .9);

        if((rightPos+.025 <= 1) && (leftPos-.025 >= 0)) {
            if(gamepad2RightY > 0) // open
            {
                rightPos += 0.025;
                leftPos -= 0.025;
            }
        }
        if((rightPos-.025 >= 0) && (leftPos+.025) <=1) {
            if(gamepad2RightY < 0) // close
            {
                rightPos -= 0.025;
                leftPos += 0.025;
            }
        }
        else if(gamepad2.x) {
            rightPos = 0;
            leftPos = 0;
        }

        rightArm.setPosition(rightPos);
        leftArm.setPosition(leftPos);
        /*
        if(pressLeft)
        {
            leftArm.setPosition(0.00);
            rightArm.setPosition(1.00);
        }
        else if(pressRight)
        {
            leftArm.setPosition(1.00);
            rightArm.setPosition(0.00);
        }
         */


    }
}