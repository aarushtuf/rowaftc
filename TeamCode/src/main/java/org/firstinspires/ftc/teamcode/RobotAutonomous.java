package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * A skeletal example of a do-nothing first OpMode. Go ahead and change this code
 * to suit your needs, or create sibling OpModes adjacent to this one in the same
 * Java package.
 */
@Autonomous(name="Robot Autonomous Program")
public class RobotAutonomous extends OpMode {
    /* Declare here any fields you might find useful. */
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

    /*public void raiseArm() {
        armServo.setPosition(0.8);
    }

    public void lowerArm() {
        armServo.setPosition(0.2);
    }*/

}
