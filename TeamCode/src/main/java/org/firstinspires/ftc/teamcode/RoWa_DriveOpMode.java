package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Basic: RoWa Drive OpMode", group="Linear Opmode")
//@Disabled
public class RoWa_DriveOpMode extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    RoWa_Hardware robot = new RoWa_Hardware(this);

    float gain = 2;

    @Override
    public void runOpMode() {

        robot.init();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            // slow mode
            boolean slowDown = gamepad1.left_bumper;
            if(slowDown){
                axial = axial * 0.1;
                lateral = lateral * 0.1;
                yaw = yaw * 0.1;
            }

            //Detect color
            if (gamepad1.a) {
                // Only increase the gain by a small amount, since this loop will occur multiple times per second.
                gain += 0.005;
            } else if (gamepad1.b && gain > 1) { // A gain of less than 1 will make the values smaller, which is not helpful.
                gain -= 0.005;
            }
            boolean lightOn = gamepad1.x;
            NormalizedRGBA color = robot.getColor(gain,lightOn);
            robot.setBackgroundColor(color);
            if(color.red == 1) {
                //stop
                axial = 0;
                lateral = 0;
                yaw = 0;
            }
            //Detect if touch sensor pressed
            boolean isSensorPressed = robot.isTouchSenorPressed();
            if(isSensorPressed){
                // if sensor is pressed we have hit something so stop
                axial = 0;
                lateral = 0;
                yaw = 0;
            }

            robot.driveRobot(axial,lateral,yaw);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}