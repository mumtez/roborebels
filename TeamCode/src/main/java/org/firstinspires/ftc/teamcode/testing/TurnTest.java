package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.NewRobot;
import org.firstinspires.ftc.teamcode.NewRobot.AllianceColor;

@Config
@Autonomous(name = "Turn Test", group = "TESTING")
public class TurnTest extends LinearOpMode {

  public static int HEADING_DEG = 180;
  public static boolean HOLD_END = true;

  @Override
  public void runOpMode() throws InterruptedException {
    telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    NewRobot robot = new NewRobot(this, AllianceColor.RED, true);

    Point fixedPoint = new Point(0, 0);

    waitForStart();
    robot.follower.setStartingPose(new Pose(0, 0, 0));

    while (opModeIsActive()) {
      if (gamepad1.cross) {
        PathChain path = robot.follower.pathBuilder()
            .addBezierLine(
                fixedPoint,
                fixedPoint
            )
            .setConstantHeadingInterpolation(Math.toRadians(HEADING_DEG))
            .setPathEndTimeoutConstraint(3000)
            .build();

        robot.follower.followPath(path, HOLD_END);
        int x = 0;
        while (opModeIsActive() && robot.follower.isBusy()) {
          robot.follower.update();
          telemetry.addData("x", robot.follower.getPose().getX());
          telemetry.addData("y", robot.follower.getPose().getY());
          telemetry.addData("heading", robot.follower.getPose().getHeading());
          telemetry.addData("heading offset", robot.follower.getHeadingOffset());
          telemetry.addData("TURNING ITERATIONS", x++);
          telemetry.update();
        }
      }
    }
  }
}
