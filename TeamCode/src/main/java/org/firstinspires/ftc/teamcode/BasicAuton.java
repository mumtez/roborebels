package org.firstinspires.ftc.teamcode;

import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.RedPropThreshold;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "Vision Test")
public class BasicAuton extends LinearOpMode {

  private VisionPortal portal;
  private RedPropThreshold redPropThreshold;


  @Override
  public void runOpMode() throws InterruptedException {
    redPropThreshold = new RedPropThreshold();
    portal = new VisionPortal.Builder()
        .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
        .setCameraResolution(new Size(640, 480))
        .addProcessor(redPropThreshold)
        .build();

    waitForStart();
    while (opModeIsActive()) {
      telemetry.addData("Prop Position", redPropThreshold.getElePos());
      telemetry.update();
    }
    //Will output prop position on Driver Station Console
  }
}