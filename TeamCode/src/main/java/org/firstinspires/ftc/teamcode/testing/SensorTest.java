package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import org.firstinspires.ftc.teamcode.NewRobot;
import org.firstinspires.ftc.teamcode.NewRobot.AllianceColor;

@TeleOp(name = "SENSOR TEST", group = "TESTING")
public class SensorTest extends LinearOpMode {

  @Override
  public void runOpMode() throws InterruptedException {
    telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    NewRobot robot = new NewRobot(this, AllianceColor.RED);
    robot.follower.setStartingPose(new Pose(0, 0, 0));

    waitForStart();

    robot.horSlide.hSlide.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
    robot.slides.slideLeft.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
    robot.slides.slideRight.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);

    while (opModeIsActive()) {

      telemetry.addLine("=== ENCODERS ===");
      telemetry.addData("H SLIDE", robot.horSlide.hSlide.getCurrentPosition());
      telemetry.addData("V SLIDE LEFT", robot.slides.slideLeft.getCurrentPosition());
      telemetry.addData("V SLIDE RIGHT", robot.slides.slideRight.getCurrentPosition());

      telemetry.addLine();
      telemetry.addLine("=== SENSORS ===");
      telemetry.addData("V SLIDE MAG LIM", robot.slides.magLim.isPressed());
      double intakePow = gamepad1.right_trigger - gamepad1.left_trigger;
      robot.intake.senseDistance();
      robot.intake.senseColor();
      robot.intake.updateSampleColor();
      robot.intake.setPower(intakePow);
      if (Math.abs(intakePow) > 0.1) {
        robot.intake.rotateDown();
      } else {
        robot.intake.rotateFlat();
      }
      telemetry.addData("INTAKE COLOR RED", robot.intake.getColors().red);
      telemetry.addData("INTAKE COLOR GREEN", robot.intake.getColors().green);
      telemetry.addData("INTAKE COLOR BLUE", robot.intake.getColors().blue);
      telemetry.addData("INTAKE COLOR ALPHA", robot.intake.getColors().alpha);
      telemetry.addData("INTAKE COLOR SENSOR DIST", robot.intake.getDist());
      telemetry.addData("DETECT", robot.intake.getSampleColor());

      telemetry.addLine();
      telemetry.addLine("=== LOCALIZER ===");
      robot.follower.updatePose();
      Pose pose = robot.follower.getPose();
      telemetry.addData("X", pose.getX());
      telemetry.addData("Y", pose.getY());
      telemetry.addData("Heading", pose.getHeading());

      telemetry.update();
    }
  }
}
