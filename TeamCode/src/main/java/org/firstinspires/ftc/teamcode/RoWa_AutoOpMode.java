package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@Autonomous(name="Robot: Auto Drive To Line", group="Linear Opmode")
public class RoWa_AutoOpMode extends LinearOpMode {

        private RoWa_Hardware robot = new RoWa_Hardware(this);

        static final double     WHITE_THRESHOLD = 0.5;  // spans between 0.0 - 1.0 from dark to light
        static final double     APPROACH_SPEED  = 0.25;

        @Override
        public void runOpMode() {

            // Wait for driver to press PLAY)
            // Abort this loop is started or stopped.
            while (opModeInInit()) {

                // Send telemetry message to signify robot waiting;
                telemetry.addData("Status", "Ready to drive to white line.");    //

                // Display the light level while we are waiting to start
                getBrightness();
            }

            // Start the robot moving forward, and then begin looking for a white line.
            robot.driveRobot(APPROACH_SPEED,0,0);

            // run until the white line is seen OR the driver presses STOP;
            while (opModeIsActive() && (getBrightness() < WHITE_THRESHOLD)) {
                sleep(5);
            }

            // Stop all motors
            robot.stopRobot();
        }

        // to obtain reflected light, read the normalized values from the color sensor.  Return the Alpha channel.
        double getBrightness() {
            // Some sensors allow you to set your light sensor gain for optimal sensitivity...
            // See the SensorColor sample in this folder for how to determine the optimal gain.
            // A gain of 15 causes a Rev Color Sensor V2 to produce an Alpha value of 1.0 at about 1.5" above the floor.
            NormalizedRGBA colors = robot.getColor(15);
            telemetry.addData("Light Level (0 to 1)",  "%4.2f", colors.alpha);
            telemetry.update();

            return colors.alpha;
        }

}
