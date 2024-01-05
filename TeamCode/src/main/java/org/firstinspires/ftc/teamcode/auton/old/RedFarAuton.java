package org.firstinspires.ftc.teamcode.auton.old;

import android.util.Size;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.vision.Position;
import org.firstinspires.ftc.teamcode.vision.RedPropThreshold;
import org.firstinspires.ftc.vision.VisionPortal;

@Disabled
@Config
@Autonomous(name = "Red Far")
public class RedFarAuton extends LinearOpMode {

  public static int dfDist = 700;
  public static int lAng = 90;
  public static int rAng = -90;

  Robot robot;
  RedPropThreshold processor;

  @Override
  public void runOpMode() throws InterruptedException {
    processor = new RedPropThreshold();
    robot = new Robot(this);

    VisionPortal portal = new VisionPortal.Builder()
        .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
        .setCameraResolution(new Size(640, 480))
        .addProcessor(processor)
        .build();

    while (opModeInInit()) {
      telemetry.addData("Location", processor.getElePos());
      telemetry.addData("Left", processor.averagedLeftBox);
      telemetry.addData("Right", processor.averagedRightBox);
      telemetry.addData("Thresh", RedPropThreshold.redThreshold);
      telemetry.update();
    }

    Position x = processor.getElePos();

    switch (x) {
      case LEFT:
        robot.encodeDriveForward(dfDist - 10, .3);
        robot.turnByGyro(lAng);
        sleep(300);
        robot.encodeDriveForward(200, .3);
        sleep(300);
        robot.encodeDriveForward(-140, .3);
        break;

      case CENTER:
        robot.encodeDriveForward(dfDist + 200, .3);
        sleep(300);
        robot.encodeDriveForward(-200, .3);
        break;

      case RIGHT:
        robot.encodeDriveForward(dfDist, .3);
        robot.turnByGyro(rAng);
        robot.encodeDriveForward(200, .3);
        sleep(300);
        robot.encodeDriveForward(-200, .3);
        break;
    }
    robot.spitPixel();
    sleep(2000);
    robot.encodeDriveForward(-100, .3);
  }
}