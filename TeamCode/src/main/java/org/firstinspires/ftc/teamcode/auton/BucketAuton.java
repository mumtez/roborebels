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

  PathChain path1, path2, path3, path4, path5, path6, path7, path8, path9;
  private int pathState = 0;
  private Timer pathTimer;


  public void buildPaths() {
    path1 = follower.pathBuilder()
        .addPath(
            // Line 1
            new BezierLine(
                new Point(9.000, 110.000, Point.CARTESIAN),
                new Point(12.000, 130.000, Point.CARTESIAN)
            )
        )
        .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(315))
        .build();
    path2 = follower.pathBuilder()
        .addPath(
            // Line 2
            new BezierLine(
                new Point(12.000, 130.000, Point.CARTESIAN),
                new Point(22.000, 125.000, Point.CARTESIAN)
            )
        )
        .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(340))
        .build();
    path3 = follower.pathBuilder()
        .addPath(
            // Line 3
            new BezierLine(
                new Point(22.000, 125.000, Point.CARTESIAN),
                new Point(12.000, 130.000, Point.CARTESIAN)
            )
        )
        .setLinearHeadingInterpolation(Math.toRadians(340), Math.toRadians(315))
        .build();
    path4 = follower.pathBuilder()
        .addPath(
            // Line 4
            new BezierLine(
                new Point(12.000, 130.000, Point.CARTESIAN),
                new Point(22.000, 130.000, Point.CARTESIAN)
            )
        )
        .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(0))
        .build();
    path5 = follower.pathBuilder()
        .addPath(
            // Line 5
            new BezierLine(
                new Point(22.000, 130.000, Point.CARTESIAN),
                new Point(12.000, 130.000, Point.CARTESIAN)
            )
        )
        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(315))
        .build();
    path6 = follower.pathBuilder()
        .addPath(
            // Line 6
            new BezierLine(
                new Point(12.000, 130.000, Point.CARTESIAN),
                new Point(26.000, 131.000, Point.CARTESIAN)
            )
        )
        .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(40))
        .build();
    path7 = follower.pathBuilder()
        .addPath(
            // Line 7
            new BezierLine(
                new Point(26.000, 131.000, Point.CARTESIAN),
                new Point(12.000, 130.000, Point.CARTESIAN)
            )
        )
        .setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(315))
        .build();
    path8 = follower.pathBuilder()
        .addPath(
            // Line 8
            new BezierCurve(
                new Point(12.000, 130.000, Point.CARTESIAN),
                new Point(48.000, 130.000, Point.CARTESIAN),
                new Point(65.000, 100.000, Point.CARTESIAN)
            )
        )
        .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(270))
        .build();
    path9 = follower.pathBuilder()
        .addPath(
            // Line 9
            new BezierCurve(
                new Point(65.000, 100.000, Point.CARTESIAN),
                new Point(48.000, 130.000, Point.CARTESIAN),
                new Point(12.000, 130.000, Point.CARTESIAN)
            )
        )
        .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(315))
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

      case 1:
        if (!follower.isBusy()) {
          follower.followPath(path2);
          setPathState(2);
        }
        break;
      case 2:
        if (!follower.isBusy()) {
          follower.followPath(path3);
          setPathState(3);
        }
        break;
      case 3:
        if (!follower.isBusy()) {
          follower.followPath(path4);
          setPathState(4);
        }
        break;
      case 4:
        if (!follower.isBusy()) {
          follower.followPath(path5);
          setPathState(5);
        }
        break;
      case 5:
        if (!follower.isBusy()) {
          follower.followPath(path6);
          setPathState(6);
        }
        break;
      case 6:
        if (!follower.isBusy()) {
          follower.followPath(path7);
          setPathState(7);
        }
        break;
      case 7:
        if (!follower.isBusy()) {
          follower.followPath(path8);
          setPathState(8);
        }
        break;
      case 8:
        if (!follower.isBusy()) {
          follower.followPath(path9);
          setPathState(9);
        }
        break;
    }

  }


  @Override
  public void runOpMode() throws InterruptedException {
    robot = new Robot(this);
    follower = robot.follower;
    follower.setStartingPose(new Pose(9, 110, Math.toRadians(270)));

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
