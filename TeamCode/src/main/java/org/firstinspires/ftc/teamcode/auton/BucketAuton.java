package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Robot;

@Config
@Autonomous(name = "BUCKET", group = "PEDRO")
public class BucketAuton extends LinearOpMode {

  Robot robot;
  Follower follower;

  PathChain path1;
  private int pathState = 0;
  private Timer pathTimer;


  public void buildPaths() {
    path1 = follower.pathBuilder()
        .addPath(
            // Line 1
            new BezierLine(
                new Point(9.513, 109.492, Point.CARTESIAN),
                new Point(12.000, 130.000, Point.CARTESIAN)
            )
        )
        .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
        .addPath(
            // Line 2
            new BezierLine(
                new Point(12.000, 130.000, Point.CARTESIAN),
                new Point(22.000, 125.000, Point.CARTESIAN)
            )
        )
        .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(160))
        .addPath(
            // Line 3
            new BezierLine(
                new Point(22.000, 125.000, Point.CARTESIAN),
                new Point(12.000, 130.000, Point.CARTESIAN)
            )
        )
        .setLinearHeadingInterpolation(Math.toRadians(160), Math.toRadians(135))
        .addPath(
            // Line 4
            new BezierLine(
                new Point(12.000, 130.000, Point.CARTESIAN),
                new Point(22.000, 130.000, Point.CARTESIAN)
            )
        )
        .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
        .addPath(
            // Line 5
            new BezierLine(
                new Point(22.000, 130.000, Point.CARTESIAN),
                new Point(12.000, 130.000, Point.CARTESIAN)
            )
        )
        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
        .addPath(
            // Line 6
            new BezierLine(
                new Point(12.000, 130.000, Point.CARTESIAN),
                new Point(26.000, 131.000, Point.CARTESIAN)
            )
        )
        .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(215))
        .addPath(
            // Line 7
            new BezierLine(
                new Point(26.000, 131.000, Point.CARTESIAN),
                new Point(12.000, 130.000, Point.CARTESIAN)
            )
        )
        .setLinearHeadingInterpolation(Math.toRadians(215), Math.toRadians(135))
        .addPath(
            // Line 8
            new BezierCurve(
                new Point(12.000, 130.000, Point.CARTESIAN),
                new Point(48.000, 130.000, Point.CARTESIAN),
                new Point(65.000, 100.000, Point.CARTESIAN)
            )
        )
        .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(90))
        .addPath(
            // Line 9
            new BezierCurve(
                new Point(65.000, 100.000, Point.CARTESIAN),
                new Point(48.000, 130.000, Point.CARTESIAN),
                new Point(12.000, 130.000, Point.CARTESIAN)
            )
        )
        .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
        .build();

  }

  public void setPathState(int pState) {
    pathState = pState;
    pathTimer.resetTimer();
  }

  public void autonomousPathUpdate() {
    switch (pathState) {
      case 0:
        follower.followPath(path1);
        setPathState(1);
        break;
    }
  }


  @Override
  public void runOpMode() throws InterruptedException {
    robot = new Robot(this);
    follower = robot.follower;
    follower.setStartingPose(new Pose(9.513, 109.492, Math.toRadians(90)));

    pathTimer = new Timer();
    buildPaths();

    waitForStart();

    while (opModeIsActive()) {
      follower.update();
      autonomousPathUpdate();
      telemetry.addData("Path State", pathState);
      telemetry.addData("Position", follower.getPose().toString());
      telemetry.update();
    }
  }
}
