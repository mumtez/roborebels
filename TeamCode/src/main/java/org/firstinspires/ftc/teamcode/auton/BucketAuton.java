package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@Config
@Autonomous(name = "BUCKET", group = "PEDRO")
public class BucketAuton extends OpMode {

  private Robot robot;

  private Telemetry telemetryA;

  private Follower follower;

  // x values
  double[] xValues2 = {
          9.757, 24.163, 17.000, 57.000, 66.000
  };

  // y values
  double[] yValues2 = {
          84.983, 106.658, 127.000, 120.000, 100.000
  };

  // Rewriting points
  Point p1 = new Point(xValues2[0], yValues2[0], Point.CARTESIAN);
  Point p2 = new Point(xValues2[1], yValues2[1], Point.CARTESIAN);
  Point p3 = new Point(xValues2[2], yValues2[2], Point.CARTESIAN);
  Point p4 = new Point(xValues2[3], yValues2[3], Point.CARTESIAN);
  Point p5 = new Point(xValues2[4], yValues2[4], Point.CARTESIAN);





  private PathChain path1, path2, path3, path4, path5, path6, path7, path8, path9;

  private int pathState = 0;

  private Timer pathTimer;

  public static Point wallPickup = new Point(42, -63);

  public void buildPaths() {

    path1 = follower.pathBuilder()
            .addPath(new BezierCurve(p1, p2, p3))
            .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
            .build();

    path2 = follower.pathBuilder()
            .addPath(new BezierLine(p3, p3))
            .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(165))
            .build();

    path3 = follower.pathBuilder()
            .addPath(new BezierLine(p3, p3))
            .setLinearHeadingInterpolation(Math.toRadians(165), Math.toRadians(135))
            .build();

    path4 = follower.pathBuilder()
            .addPath(new BezierLine(p3, p3))
            .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(191))
            .build();

    path5 = follower.pathBuilder()
            .addPath(new BezierLine(p3, p3))
            .setLinearHeadingInterpolation(Math.toRadians(191), Math.toRadians(135))
            .build();

    path6 = follower.pathBuilder()
            .addPath(new BezierLine(p3, p3))
            .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(210))
            .build();

    path7 = follower.pathBuilder()
            .addPath(new BezierLine(p3, p3))
            .setLinearHeadingInterpolation(Math.toRadians(210), Math.toRadians(135))
            .build();

    path8 = follower.pathBuilder()
            .addPath(new BezierCurve(p3, p4, p5))
            .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(90))
            .build();

    path9 = follower.pathBuilder()
            .addPath(new BezierCurve(p5, p4, p3))
            .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
            .build();

  }

  public void autonomousPathUpdate() {
    switch (pathState) {
      case 0:
        follower.followPath(path1);

        setPathState(1);
        break;

      case 1:
        //robot.startSlideUpPos(Robot.VERTICAL_SLIDE_DEFAULT, 0.8);
        //robot.claw.setUnder();

        if (!follower.isBusy()) { //go to bar
          follower.followPath(path2, true);
          //place on bar
          //placeBar();
          //add a wait?

          setPathState(2);
        }
        break;

      case 2:
        if (!follower.isBusy()) { // set up for push
          follower.followPath(path3, true);
          setPathState(3);
        }
        break;

      case 3:
        if (!follower.isBusy()) { // push one
          follower.followPath(path4, true);
          setPathState(4);
        }
        break;

      case 4:
        if (!follower.isBusy()) { // set up for push 2
          follower.followPath(path5, true);
          setPathState(5);
        }
        break;

      case 5:
        if (!follower.isBusy()) { // push 2
          follower.followPath(path6, true);
          setPathState(6);
        }
        break;

      case 6:
        if (!follower.isBusy()) { // set up for push 3
          follower.followPath(path7, true);
          setPathState(7);
        }
        break;

      case 7:
        if (!follower.isBusy()) { // push 3 / pick up wall
          follower.followPath(path8, true);

          //getWall();
          //add wait?

          setPathState(8);
        }
        break;

      case 8:
        if (!follower.isBusy()) { // place bar
          follower.followPath(path9, true);

          //placeBar();
          //add a wait?
          setPathState(9);
        }
        break;

      case 11:
        if (!follower.isBusy()) {
          setPathState(-1); // End the autonomous routine
        }
        break;
    }
  }

  public void setPathState(int pState) {
    pathState = pState;
    pathTimer.resetTimer();
  }


@Override
public void init() {
  Constants.setConstants(FConstants.class, LConstants.class);
  pathTimer = new Timer();
  Constants.setConstants(FConstants.class, LConstants.class);
  follower = new Follower(hardwareMap);
  follower.setStartingPose(new Pose(10.000, 65.000));
  buildPaths();
}

/**
 * This runs the OpMode, updating the Follower as well as printing out the debug statements to the Telemetry, as well
 * as the FTC Dashboard.
 */


@Override
public void loop() {
  follower.update();
  autonomousPathUpdate();
  telemetry.addData("Path State", pathState);
  telemetry.addData("Position", follower.getPose().toString());
  telemetry.update();
}
}
