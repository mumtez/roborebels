package org.firstinspires.ftc.teamcode;

import android.util.Size;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.BluePropThreshold;
import org.firstinspires.ftc.teamcode.vision.Position;
import org.firstinspires.ftc.teamcode.vision.RedPropThreshold;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Autonomous(name = "Red Close")
public class RedCloseAuto extends LinearOpMode {

  public static int dfDist = 700;

  public static int leftDist = 500;
  public static int centerDist = 600;
  public static int rightDist = 620;

  public static int leftAng = -90;
  public static int centerAng = -80;


  public static int dsDist = 800;
  public static int lAng = 90;
  public static int rAng = -90;

  public static int place = -600;

  Robot robot;
  private VisionPortal portal;
  RedPropThreshold processor;

  @Override
  public void runOpMode() throws InterruptedException {
    processor = new RedPropThreshold();
    robot = new Robot(this);
    int ang = 0;

    portal = new VisionPortal.Builder()
        .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
        .setCameraResolution(new Size(640, 480))
        .addProcessor(processor)
        .build();

    while (opModeInInit()) {
      telemetry.addData("Location", processor.getElePos());
      telemetry.update();

    }
    Position x = processor.getElePos();
    switch (x) {
      case LEFT:
        ang = lAng;
        break;
      case RIGHT:
        ang = rAng;
        break;
    }

    //Start Movement

    robot.encodeDriveForward(dfDist, .3);
    if (x != Position.CENTER) {
      robot.turnByGyro(ang, .6);
    }
    robot.encodeDriveForward(20, .3);
    robot.spitPixel();
    sleep(2000);
    robot.encodeDriveForward(-20, .3);

    switch (x) {
      case LEFT:
        robot.encodeDriveStrafe(-dfDist, .3);
        robot.encodeDriveForward(dfDist-100, .3);
        robot.encodeDriveStrafe(200, .3);

        robot.turnByGyro(leftAng, .6);

        robot.encodeDriveStrafe(-leftDist+200, .3);
        robot.encodeDriveForward(-200, .3);


        robot.setSlidePos(2650, 1);
        robot.setSlidePos(0, 1);
        break;
      case RIGHT:
        robot.encodeDriveForward(-dfDist-175, .3);

        robot.setSlidePos(2650, 1);
        robot.setSlidePos(0, 1);
        break;

      case CENTER:

        robot.encodeDriveStrafe(-dfDist, .3);
        robot.turnByGyro(centerAng, .6);
        robot.encodeDriveForward(-200, 0.3);

        robot.setSlidePos(2350, 1);
        robot.setSlidePos(0, 1);
        break;
    }

  }
}