package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "Vision Test", group = "TESTING")
public class VisionTest extends LinearOpMode {

  private VisionPortal portal;

  @Override
  public void runOpMode() throws InterruptedException {
    RedPropThreshold redPropThreshold = new RedPropThreshold();
    portal = new VisionPortal.Builder()
        .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
        .setCameraResolution(new Size(640, 480))
        .addProcessor(redPropThreshold)
        .build();

    waitForStart();
    while (opModeIsActive()) {
      telemetry.addData("Prop Position", redPropThreshold.getElePos());
      telemetry.addData("", redPropThreshold.averagedLeftBox);
      telemetry.addData("", redPropThreshold.averagedRightBox);
      telemetry.update();
    }
    //Will output prop position on Driver Station Console
  }
}