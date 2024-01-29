package org.firstinspires.ftc.teamcode.Testing;

import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.BluePropThreshold;
import org.firstinspires.ftc.teamcode.vision.RedPropThreshold;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "Vision Test", group = "TESTING")
public class VisionTest extends LinearOpMode {

  @Override
  public void runOpMode() throws InterruptedException {
    RedPropThreshold redPropThreshold = new RedPropThreshold();
    BluePropThreshold bluePropThreshold = new BluePropThreshold();
    VisionPortal portal = new VisionPortal.Builder()
        .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
        .setCameraResolution(new Size(640, 480))
        .addProcessor(redPropThreshold)
        .addProcessor(bluePropThreshold)
        .build();

    portal.setProcessorEnabled(redPropThreshold, true);
    portal.setProcessorEnabled(bluePropThreshold, false);
    waitForStart();
    while (opModeIsActive()) {
      telemetry.addLine("PRESS A / B TO SWITCH PROCESSORS");
      if (gamepad1.a) {
        portal.setProcessorEnabled(redPropThreshold, false);
        portal.setProcessorEnabled(bluePropThreshold, true);
      }

      if (gamepad1.b) {
        portal.setProcessorEnabled(redPropThreshold, true);
        portal.setProcessorEnabled(bluePropThreshold, false);
      }

      if (portal.getProcessorEnabled(redPropThreshold)) {
        telemetry.addData("RED Prop Position", redPropThreshold.getElePos());
        telemetry.addData("RED left box avg", redPropThreshold.averagedLeftBox);
        telemetry.addData("RED right box avg", redPropThreshold.averagedRightBox);
      }
      if (portal.getProcessorEnabled(bluePropThreshold)) {
        telemetry.addData("BLUE Prop Position", bluePropThreshold.getElePos());
        telemetry.addData("BLUE left box avg", bluePropThreshold.averagedLeftBox);
        telemetry.addData("BLUE right box avg", bluePropThreshold.averagedRightBox);
      }
      telemetry.update();
    }
    //Will output prop position on Driver Station Console
  }
}