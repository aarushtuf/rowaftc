package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

public class RoWa_Hardware {
    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    private DcMotor leftFrontMotor = null; //0
    private DcMotor rightFrontMotor = null; //1
    private DcMotor leftBackMotor = null; //2
    private DcMotor rightBackMotor = null; //3

    NormalizedColorSensor colorSensor;

    TouchSensor touchSensor;  // Touch sensor Object

    View relativeLayout;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RoWa_Hardware (LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init()    {
        // setup motors
        leftFrontMotor  = myOpMode.hardwareMap.get(DcMotor.class, "lfm");
        rightFrontMotor  = myOpMode.hardwareMap.get(DcMotor.class, "rfm");
        rightBackMotor  = myOpMode.hardwareMap.get(DcMotor.class, "rbm");
        leftBackMotor  = myOpMode.hardwareMap.get(DcMotor.class, "lbm");
        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);

        // setup color sensor
        colorSensor = myOpMode.hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        //int relativeLayoutId = myOpMode.hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", myOpMode.hardwareMap.appContext.getPackageName());
        //relativeLayout = ((Activity) myOpMode.hardwareMap.appContext).findViewById(relativeLayoutId);

        //setup touch sensor
        touchSensor = myOpMode.hardwareMap.get(TouchSensor.class, "sensor_touch");

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    public NormalizedRGBA getColor(float gain, boolean lightOn){

        if (colorSensor instanceof SwitchableLight) {
            SwitchableLight light = (SwitchableLight)colorSensor;
            light.enableLight(lightOn);
        }

        // Tell the sensor our desired gain value (normally you would do this during initialization,
        // not during the loop)
        colorSensor.setGain(gain);

        // Get the normalized colors from the sensor
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        // Update the hsvValues array by passing it to Color.colorToHSV()

        myOpMode.telemetry.addData("Gain", gain);

        myOpMode.telemetry.addLine()
                .addData("Red", "%.3f", colors.red)
                .addData("Green", "%.3f", colors.green)
                .addData("Blue", "%.3f", colors.blue);

        myOpMode.telemetry.addData("Alpha", "%.3f", colors.alpha);
        return  colors;
    }

    public void driveRobot(double axial, double lateral, double yaw) {
        double max;
        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }
        setDrivePower(leftFrontPower,rightFrontPower,leftBackPower,rightBackPower);
    }

    public void setBackgroundColor(NormalizedRGBA bgColor){
        final float[] hsvValues = new float[3];
        Color.colorToHSV(bgColor.toColor(), hsvValues);
        myOpMode.telemetry.addLine()
                .addData("Hue", "%.3f", hsvValues[0])
                .addData("Saturation", "%.3f", hsvValues[1])
                .addData("Value", "%.3f", hsvValues[2]);
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.HSVToColor(hsvValues));
            }
        });
    }

    public boolean isTouchSenorPressed(){
        myOpMode.telemetry.addData("Touch Sensor Pressed:", touchSensor.isPressed());
        return touchSensor.isPressed();
    }

    public void setDrivePower(double leftFrontPower, double rightFrontPower, double leftBackPower, double rightBackPower) {
        // Send calculated power to wheels
        leftFrontMotor.setPower(leftFrontPower);
        leftBackMotor.setPower(leftBackPower);
        rightFrontMotor.setPower(rightFrontPower);
        rightBackMotor.setPower(rightBackPower);
        myOpMode.telemetry.addData("Motors", "leftFront (%.2f), rightFront (%.2f), leftBack (%.2f) rightBack (%.2f)", leftFrontPower, rightFrontPower, leftBackPower,rightBackPower);
    }


}