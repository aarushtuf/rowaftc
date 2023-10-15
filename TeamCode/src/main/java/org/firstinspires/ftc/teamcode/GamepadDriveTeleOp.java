package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class GamepadDriveTeleOp extends OpMode {

    DcMotor motorLeft = null;
    DcMotor motorRight = null;

    //Servo armServo = null;

    double DRIVE_POWER = 1.0;

    double DRIVE_REV_POWER = -1.0;

    double leftWheelPower;
    double rightWheelPower;

    /**
     *
     */
    @Override
    public void init() {
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorRight = hardwareMap.dcMotor.get("motorRight");
        motorLeft.setDirection(DcMotor.Direction.REVERSE);
    }

    /**
     *
     */
    @Override
    public void loop() {
        try {
            leftWheelPower = gamepad1.left_stick_y;
            rightWheelPower = gamepad1.right_stick_y;

            driveForward(DRIVE_POWER, 4000);
            turnLeft(DRIVE_POWER, 500);
            driveForward(DRIVE_POWER, 4000);
            turnRight(DRIVE_POWER, 500);
            driveForward(DRIVE_POWER, 4000);
            stopDriving();
            //lowerArm();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    public void driveForward(double power, long time) throws InterruptedException {
        motorLeft.setPower(power);
        motorRight.setPower(power);
        Thread.sleep(time);
    }

    public void stopDriving() throws InterruptedException {
        driveForward(0, 4000);
    }

    public void turnLeft(double power, long time) throws InterruptedException {
        motorLeft.setPower(-power);
        motorRight.setPower(power);
        Thread.sleep(time);
    }

    public void turnRight(double power, long time) throws InterruptedException {
        turnLeft(-power, 500);
        Thread.sleep(time);
    }
}
