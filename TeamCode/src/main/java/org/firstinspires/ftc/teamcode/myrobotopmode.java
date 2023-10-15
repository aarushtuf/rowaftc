package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name = "myrobotopmode (Blocks to Java)")
public class myrobotopmode extends LinearOpMode {

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    AprilTagLibrary.Builder myAprilTagLibraryBuilder;
    AprilTagProcessor.Builder myAprilTagProcessorBuilder;
    AprilTagLibrary myAprilTagLibrary;
    AprilTagProcessor myAprilTagProcessor;
    VisionPortal myVisionPortal;
    AprilTagDetection myAprilTagDetection;
    List<AprilTagDetection> myAprilTagDetections;

    // Put initialization blocks here.
    // Create a new AprilTagLibrary.Builder objectand assigns it to a variable.
    myAprilTagLibraryBuilder = new AprilTagLibrary.Builder();
    // Add a tag, without pose information, to the AprilTagLibrary.Builder.
    myAprilTagLibraryBuilder.addTag(17, "myrobottag", 6.5, DistanceUnit.INCH);
    // Build the AprilTag library and assign it to a variable.
    myAprilTagLibrary = myAprilTagLibraryBuilder.build();
    // Create a new AprilTagProcessor.Builder object and assign it to a variable.
    myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
    // Set the tag library.
    myAprilTagProcessorBuilder.setTagLibrary(myAprilTagLibrary);
    // Build the AprilTag processor and assign it to a variable.
    myAprilTagProcessor = myAprilTagProcessorBuilder.build();
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      // Create a VisionPortal, with the specified webcam name
      // and AprilTag processor, and assign it to a variable.
      myVisionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), myAprilTagProcessor);
      while (opModeIsActive()) {
        // Put loop blocks here.
        // Get a list containing the latest detections, which may be stale.
        myAprilTagDetections = myAprilTagProcessor.getDetections();
        // TODO: Enter the type for variable named myAprilTagDetection
        for (UNKNOWN_TYPE myAprilTagDetection_item : myAprilTagDetection) {
          myAprilTagDetection = myAprilTagDetection_item;
          telemetry.addData("tag", myAprilTagDetection.id);
          telemetry.addData("x", myAprilTagDetection.ftcPose.x);
          telemetry.addData("y", myAprilTagDetection.ftcPose.y);
          telemetry.addData("z", myAprilTagDetection.ftcPose.z);
        }
        telemetry.update();
      }
    }
  }
}