package org.firstinspires.ftc.teamcode;

import android.util.Size;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.BluePropThreshold;
import org.firstinspires.ftc.teamcode.vision.Position;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Autonomous(name = "Blue Far")
public class BlueFarAuton extends LinearOpMode {

  public static int dfDist = 700;
  public static int lAng = 90;
  public static int rAng = -90;

  Robot robot;
  BluePropThreshold processor;

  @Override
  public void runOpMode() throws InterruptedException {
    processor = new BluePropThreshold();
    robot = new Robot(this);
    int ang = 0;

    VisionPortal portal = new VisionPortal.Builder()
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

    if(x != Position.LEFT){
      robot.encodeDriveForward(dfDist, .3);
      if (x != Position.CENTER) {
        robot.turnByGyro(ang);
      }
      robot.encodeDriveForward(200, .3);
      sleep(300);
      robot.encodeDriveForward(-200, .3);
    } else {
      robot.encodeDriveForward(dfDist - 10, .3);
      robot.turnByGyro(ang);
      sleep(300);
      robot.encodeDriveForward(200, .3);
      sleep(300);
      robot.encodeDriveForward(-140, .3);
    }
    robot.spitPixel();
    sleep(2000);
    robot.encodeDriveForward(-100, .3);
  }
}