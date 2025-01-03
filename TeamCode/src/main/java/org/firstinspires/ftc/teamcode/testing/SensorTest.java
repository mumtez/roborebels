package org.firstinspires.ftc.teamcode.testing;

import android.graphics.Color;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import java.util.Locale;
import org.firstinspires.ftc.teamcode.Robot;

@Config
@TeleOp(name = "Sensor Test", group = "TESTING")
public class SensorTest extends LinearOpMode {

  Robot robot;

  @Override
  public void runOpMode() throws InterruptedException {
    /*
    robot = new Robot(this);

    waitForStart();

    while (opModeIsActive()) {
      robot.intakeColor.setGain(Robot.INTAKE_COLOR_GAIN);

      updateTelemetry();
    }

     */
  }
/*
  void updateTelemetry() {
    telemetry.addLine("INTAKE CS ----------");
    NormalizedRGBA rgba = robot.intakeColor.getNormalizedColors();
    float[] hsv = new float[3];
    Color.colorToHSV(rgba.toColor(), hsv);
    telemetry.addData("GAIN", robot.intakeColor.getGain());
    telemetry.addLine(
        String.format(Locale.US, "R %3f, G %3f, B %3f, A %3f", rgba.red, rgba.green, rgba.blue,
            rgba.alpha));
    telemetry.addLine(
        String.format(Locale.US, "HUE %3f, SAT %3f, VAL %3f", hsv[0], hsv[1], hsv[2]));

    telemetry.update();
  }

 */
}
