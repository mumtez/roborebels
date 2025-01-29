package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@Config
@Autonomous(name = "SPECIMEN", group = "PEDRO")
public class SamplingAuton extends OpMode {

  private Telemetry telemetryA;

  private Follower follower;

  private PathChain path1, path2, path3, path4, path5, path6, path7, path8, path9, path10, path11;

  private int pathState = 0;

  private Timer pathTimer;

  public void buildPaths() {

    path1 = follower.pathBuilder()
        .addPath(new BezierLine(new Point(10.000, 65.000, Point.CARTESIAN), new Point(37.000, 65.000, Point.CARTESIAN)))
        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
        .build();

    path2 = follower.pathBuilder()
        .addPath(new BezierCurve(new Point(37.000, 65.000, Point.CARTESIAN), new Point(19.290, 29.159, Point.CARTESIAN),
            new Point(65.495, 33.869, Point.CARTESIAN)))
        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
        .build();

    path3 = follower.pathBuilder()
        .addPath(new BezierCurve(new Point(65.495, 33.869, Point.CARTESIAN), new Point(47.776, 22.654, Point.CARTESIAN),
            new Point(19.000, 23.000, Point.CARTESIAN)))
        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
        .build();

    path4 = follower.pathBuilder()
        .addPath(new BezierLine(new Point(19.000, 23.000, Point.CARTESIAN), new Point(68.000, 23.000, Point.CARTESIAN)))
        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
        .build();

    path5 = follower.pathBuilder()
        .addPath(new BezierCurve(new Point(68.000, 23.000, Point.CARTESIAN), new Point(60.336, 15.252, Point.CARTESIAN),
            new Point(19.000, 19.065, Point.CARTESIAN)))
        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
        .build();

    path6 = follower.pathBuilder()
        .addPath(new BezierLine(new Point(19.000, 19.065, Point.CARTESIAN), new Point(40.000, 19.000, Point.CARTESIAN)))
        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
        .build();

    path7 = follower.pathBuilder()
        .addPath(new BezierLine(new Point(40.000, 19.000, Point.CARTESIAN), new Point(10.000, 19.000, Point.CARTESIAN)))
        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
        .build();

    path8 = follower.pathBuilder()
        .addPath(new BezierLine(new Point(10.000, 19.000, Point.CARTESIAN), new Point(37.000, 70.000, Point.CARTESIAN)))
        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
        .build();

    path9 = follower.pathBuilder()
        .addPath(new BezierLine(new Point(37.000, 70.000, Point.CARTESIAN), new Point(10.000, 19.000, Point.CARTESIAN)))
        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
        .build();

    path10 = follower.pathBuilder()
        .addPath(new BezierLine(new Point(10.000, 19.000, Point.CARTESIAN), new Point(37.000, 70.000, Point.CARTESIAN)))
        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
        .build();

    path11 = follower.pathBuilder()
        .addPath(new BezierLine(new Point(37.000, 70.000, Point.CARTESIAN), new Point(28.000, 70.000, Point.CARTESIAN)))
        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
        .build();
  }

  public void autonomousPathUpdate() {
    switch (pathState) {
      case 0: // Move from start to scoring position
        follower.followPath(path1);
        setPathState(1);
        break;

      case 1: // Wait until the robot is near the first sample pickup position
        if (!follower.isBusy()) {
          follower.followPath(path2, true);
          setPathState(2);
        }
        break;

      case 2: // Wait until the robot is near the first scoring position
        if (!follower.isBusy()) {
          follower.followPath(path3, true);
          setPathState(3);
        }
        break;

      case 3: // Wait until the robot is near the second sample pickup position
        if (!follower.isBusy()) {
          follower.followPath(path4, true);
          setPathState(4);
        }
        break;

      case 4: // Wait until the robot is near the second scoring position
        if (!follower.isBusy()) {
          follower.followPath(path5, true);
          setPathState(5);
        }
        break;

      case 5: // Wait until the robot is near the third sample pickup position
        if (!follower.isBusy()) {
          follower.followPath(path6, true);
          setPathState(6);
        }
        break;

      case 6: // Wait until the robot is near the third scoring position
        if (!follower.isBusy()) {
          follower.followPath(path7, true);
          setPathState(7);
        }
        break;

      case 7: // Wait until the robot is near the parking position
        if (!follower.isBusy()) {
          follower.followPath(path8, true);
          setPathState(8);
        }
        break;

      case 8: // Wait until the robot is near the parking position
        if (!follower.isBusy()) {
          follower.followPath(path9, true);
          setPathState(9);
        }
        break;

      case 9: // Wait until the robot is near the final scoring position
        if (!follower.isBusy()) {
          follower.followPath(path10, true);
          setPathState(10);
        }
        break;

      case 10: // Wait until the robot completes its final task
        if (!follower.isBusy()) {
          follower.followPath(path11, true);
          setPathState(11);
        }
        break;

      case 11: // End the autonomous routine
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