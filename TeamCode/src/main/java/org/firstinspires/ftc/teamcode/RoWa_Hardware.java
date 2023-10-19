package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class RoWa_Hardware {
    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    private DcMotor leftFrontMotor = null; //0
    private DcMotor rightFrontMotor = null; //1
    private DcMotor leftBackMotor = null; //2
    private DcMotor rightBackMotor = null; //3

    private NormalizedColorSensor colorSensor;

    private TouchSensor touchSensor;  // Touch sensor Object

    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.

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

        //Setup webcam for April tag detection
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        visionPortal = new VisionPortal.Builder()
                .setCamera(myOpMode.hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }


    public NormalizedRGBA getColor(float gain){

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

    public void stopRobot() {
        setDrivePower(0,0,0,0);
    }

    public AprilTagDetection getAprilTag(int tagId){
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        if(currentDetections.isEmpty()){
            myOpMode.telemetry.addData("NoTags", "No tags detected");
        }
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((tagId < 0) || (detection.id == tagId)) {
                    // Yes, we want to use this tag.
                    myOpMode.telemetry.addData("Tag Found", "ID %d (%s)", detection.id, detection.metadata.name);
                    myOpMode.telemetry.addData("Range",  "%5.1f inches", detection.ftcPose.range);
                    myOpMode.telemetry.addData("Bearing","%3.0f degrees", detection.ftcPose.bearing);
                    myOpMode.telemetry.addData("Yaw","%3.0f degrees", detection.ftcPose.yaw);
                    //myOpMode.telemetry.update();
                    return detection;  // don't look any further.
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                    myOpMode.telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                myOpMode.telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }
        //myOpMode.telemetry.update();
        return  null;
    }

    public void  setCameraExposureAndGain(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            myOpMode.telemetry.addData("Camera", "Waiting");
            myOpMode.telemetry.update();
            while (!myOpMode.isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                myOpMode.sleep(20);
            }
            myOpMode.telemetry.addData("Camera", "Ready");
            myOpMode.telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!myOpMode.isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                myOpMode.sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            myOpMode.sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            myOpMode.sleep(20);
        }
    }

}