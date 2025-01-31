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
@Autonomous(name = "SPECIMEN", group = "PEDRO")
public class SamplingAuton extends LinearOpMode {

  Robot robot;
  Follower follower;

  // x values
  public static double[] xValues = {
      10.000, 37.000, 19.290, 65.495, 47.776, 19.000, 68.000, 60.336, 19.000, 40.000, 10.000, 37.000, 28.000
  };

  // y values
  public static double[] yValues = {
      65.000, 65.000, 29.159, 33.869, 22.654, 23.000, 23.000, 15.252, 19.065, 19.000, 19.000, 70.000, 70.000
  };


  Point p1 = new Point(xValues[0], yValues[0], Point.CARTESIAN);   //10.000, 65.000
  Point p2 = new Point(xValues[1], yValues[1], Point.CARTESIAN);   //37.000, 65.00
  Point p3 = new Point(xValues[2], yValues[2], Point.CARTESIAN);   //9.290, 29.159
  Point p4 = new Point(xValues[3], yValues[3], Point.CARTESIAN);   //65.495, 33.869
  Point p5 = new Point(xValues[4], yValues[4], Point.CARTESIAN);   //47.776, 22.654
  Point p6 = new Point(xValues[5], yValues[5], Point.CARTESIAN);   //19.000, 23.000
  Point p7 = new Point(xValues[6], yValues[6], Point.CARTESIAN);   //68.000, 23.000
  Point p8 = new Point(xValues[7], yValues[7], Point.CARTESIAN);   //60.336, 15.252
  Point p9 = new Point(xValues[8], yValues[8], Point.CARTESIAN);   //19.000, 19.065
  Point p10 = new Point(xValues[9], yValues[9], Point.CARTESIAN);   //40.000, 19.000
  Point p11 = new Point(xValues[10], yValues[10], Point.CARTESIAN); //10.000, 19.000
  Point p12 = new Point(xValues[11], yValues[11], Point.CARTESIAN); //37.000, 70.000
  Point p13 = new Point(xValues[12], yValues[12], Point.CARTESIAN); //28.000, 70.000

  public static Point wallPickup = new Point(42, -63);

  private PathChain path1, path2, path3, path4, path5, path6, path7, path8, path9, path10, path11, tempPath;
  private int pathState = 0;
  private Timer pathTimer;

  public void setPathState(int pState) {
    pathState = pState;
    pathTimer.resetTimer();
  }

  public void buildPaths() {

    path1 = follower.pathBuilder()//to bar
        .addPath(new BezierLine(p1, p2))
        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
        .build();

    path2 = follower.pathBuilder()//set up for push
        .addPath(new BezierCurve(p2, p3, p4))
        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
        .build();

    path3 = follower.pathBuilder()//push 1
        .addPath(new BezierCurve(p4, p5, p6))
        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
        .build();

    path4 = follower.pathBuilder()//set up push 2
        .addPath(new BezierLine(p6, p7))
        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
        .build();

    path5 = follower.pathBuilder()//push 2
        .addPath(new BezierCurve(p7, p8, p9))
        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
        .build();

    path6 = follower.pathBuilder()//set up push 3
        .addPath(new BezierLine(p9, p10))
        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
        .build();

    path7 = follower.pathBuilder()//push 3 / pick up wall
        .addPath(new BezierLine(p10, p11))
        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
        .build();

    path8 = follower.pathBuilder()//place bar
        .addPath(new BezierLine(p11, p12))
        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
        .build();

    path9 = follower.pathBuilder()//pick up wall
        .addPath(new BezierLine(p12, p11))
        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
        .build();

    path10 = follower.pathBuilder()//place bar
        .addPath(new BezierLine(p11, p12))
        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
        .build();

    path11 = follower.pathBuilder()
        .addPath(new BezierLine(p12, p13))
        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
        .build();
  }

  public void getWall() {
    robot.claw.clawOpen();
    robot.claw.setWall();

    tempPath = follower.pathBuilder()
        .addPath(new BezierLine(new Point(follower.getPose()),
            new Point(wallPickup.getX(), wallPickup.getY() + 18, Point.CARTESIAN)))
        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
        .build();
    follower.followPath(tempPath, true);
    robot.waitTime(100);

    tempPath = follower.pathBuilder()
        .addPath(new BezierLine(new Point(follower.getPose()), wallPickup))
        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
        .build();

    follower.followPath(tempPath, true);

/*
    Actions.runBlocking(
            drive.actionBuilder(drive.localizer.getPose())
                    .setTangent(Math.toRadians(270))
                    .splineToConstantHeading(new Point(wallPickup.x, wallPickup.y + 18), Math.toRadians(270))
                    .waitSeconds(0.1)
                    .splineToConstantHeading(wallPickup, Math.toRadians(270), SLOW, SLOW_ACCEL)
                    .build()
    );

 */

    robot.claw.clawClose();
    robot.waitTime(200);
    robot.claw.setUnder();
  }

  public void placeBarFast() {

    robot.claw.setPlace();

    tempPath = follower.pathBuilder()
        .addPath(new BezierLine(new Point(follower.getPose()),
            new Point(wallPickup.getX(), wallPickup.getY() + 9, Point.CARTESIAN)))
        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
        .build();
    follower.followPath(tempPath, true);
    robot.waitTime(70);
    robot.claw.clawOpen();

    tempPath = follower.pathBuilder()
        .addPath(new BezierLine(new Point(follower.getPose()),
            new Point(wallPickup.getX(), wallPickup.getY() + 18, Point.CARTESIAN)))
        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
        .build();
    follower.followPath(tempPath, true);


/*
    Actions.runBlocking(
            drive.actionBuilder(drive.localizer.getPose())
                    .afterTime(0, () -> {
                      robot.claw.setPlace();
                    })
                    .setTangent(Math.toRadians(270))
                    .splineToConstantHeading(new Vector2d(wallPickup.getX(), wallPickup.getY() + 18), Math.toRadians(270))
                    .afterTime(0.07, () -> {
                      robot.claw.clawOpen();
                    })
                    .build()
    );

 */
  }

  public void placeBar() {

    robot.claw.setPlace();

    tempPath = follower.pathBuilder()
        .addPath(new BezierLine(new Point(follower.getPose()),
            new Point(wallPickup.getX(), wallPickup.getY() + 9, Point.CARTESIAN)))
        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
        .build();
    follower.followPath(tempPath, true);
    robot.waitTime(200);
    robot.claw.clawOpen();

    tempPath = follower.pathBuilder()
        .addPath(new BezierLine(new Point(follower.getPose()),
            new Point(wallPickup.getX(), wallPickup.getY() + 18, Point.CARTESIAN)))
        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
        .build();
    follower.followPath(tempPath, true);

    /*
    Actions.runBlocking(
            drive.actionBuilder(drive.localizer.getPose())
                    .afterTime(0, () -> {
                      robot.claw.setPlace();
                    })
                    .setTangent(Math.toRadians(270))
                    .splineToConstantHeading(new Vector2d(wallPickup.getX()), wallPickup.getY() + 18), Math.toRadians(270))
            .afterTime(0, () -> {
                        robot.claw.clawOpen();
                      })
                              .build();
                    }

     */
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

      case 9:
        if (!follower.isBusy()) { // pick up wall
          follower.followPath(path10, true);

          //getWall();
          //add wait?
          setPathState(10);
        }
        break;

      case 10:
        if (!follower.isBusy()) { //place bar
          follower.followPath(path11, true);

          //placeBar();
          //add a wait?
          setPathState(11);
        }
        break;

      case 11:
        if (!follower.isBusy()) {
          setPathState(-1); // End the autonomous routine
        }
        break;
    }
  }

  @Override
  public void runOpMode() throws InterruptedException {
    robot = new Robot(this);
    follower = robot.follower;
    follower.setStartingPose(new Pose(10.000, 65.000));

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
