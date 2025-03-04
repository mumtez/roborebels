
package org.firstinspires.ftc.teamcode.auton.baseAutons;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.NewRobot;

@Config
public class BaseSpecAuton {

  public static double[] START = {0, 0, 0};


  private int pathState = 0;
  private Timer pathTimer;

  final NewRobot robot;
  final LinearOpMode opMode;
  final Telemetry telemetry;


  public BaseSpecAuton(LinearOpMode opMode, NewRobot robot) {
    this.opMode = opMode;
    this.telemetry = opMode.telemetry;
    this.robot = robot;
  }

  public Point pointFromArr(double[] arr) {
    return new Point(arr[0], arr[1]);
  }

  public Pose poseFromArr(double[] arr) {
    return new Pose(arr[0], arr[1], Math.toRadians(arr[2]));
  }

  public void buildPaths() {
    // TODO
  }

  public void setPathState(int pState) {
    pathState = pState;
    pathTimer.resetTimer();
  }

  public void autonomousPathUpdate() {
    switch (pathState) {
      case 1000:
        break;

      case 2000:
        break;

      // TODO
    }
  }

  public void run() {
    pathTimer = new Timer();
    buildPaths();
    robot.initAuton();

    // INIT LOOP
    while (this.opMode.opModeInInit()) {
      telemetry.addData("ALLIANCE", robot.getAllianceColor());
      telemetry.update();
    }

    // START
    robot.follower.setStartingPose(poseFromArr(START));

    while (this.opMode.opModeIsActive()) {
      robot.follower.update();
      robot.slides.updatePIDControl();
      robot.horSlide.updatePosition();
      robot.horSlide.updatePIDControl();

      autonomousPathUpdate();

      telemetry.addData("Path State", pathState);
      telemetry.addData("Position", robot.follower.getPose().toString());
      telemetry.update();
    }
  }
}