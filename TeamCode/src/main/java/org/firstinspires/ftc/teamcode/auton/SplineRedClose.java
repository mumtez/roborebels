package org.firstinspires.ftc.teamcode.auton;

import android.util.Size;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.odom.MecanumDrive;
import org.firstinspires.ftc.teamcode.vision.Position;
import org.firstinspires.ftc.teamcode.vision.RedPropThreshold;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Autonomous(name = "Red Close Spline Autonomous")
public class SplineRedClose extends LinearOpMode {

  public FtcDashboard dash = FtcDashboard.getInstance();
  Robot robot;
  private VisionPortal portal;
  RedPropThreshold processor;

  public static Pose2d BACKDROP_START = new Pose2d(10.5, -61, Math.toRadians(90));

  public static double BOARD_X = 44;
  public static Vector2d BOARD_LEFT = new Vector2d(BOARD_X, -29);
  public static Vector2d BOARD_CENTER = new Vector2d(BOARD_X, -36);
  public static Vector2d BOARD_RIGHT = new Vector2d(BOARD_X, -43);

  public static Vector2d SPIKE_LEFT = new Vector2d(10, -35);
  public static Vector2d SPIKE_CENTER = new Vector2d(18, -24);
  public static Vector2d SPIKE_RIGHT = new Vector2d(32, -30);

  public static Vector2d PARK_CORNER = new Vector2d(60, -60);
  public static Vector2d PARK_CENTER = new Vector2d(60, -10);

  public static Vector2d AUDIENCE_TRUSS = new Vector2d(-30, -58);
  public static Vector2d BACKDROP_TRUSS = new Vector2d(10.5, -58);
  public static Vector2d STACK = new Vector2d(-60, -35);

  //pose2dDual.position.x.value() > 45 ||
  public static VelConstraint BASE_VEL_CONSTRAINTS = (pose2dDual, posePath, v) -> {
    if (pose2dDual.position.y.value() < -48) {
      return 30.0;
    } else {
      return 100.0;
    }
  }; // TODO: doesnt work in meep meep but will work in real bot

  // CONFIGURATION

  public enum PARK_POSITION {
    CENTER, CORNER
  }

  public static PARK_POSITION parkPosition = PARK_POSITION.CENTER; // TODO: change to test all possibilities

  public enum PROP_POSITION {
    LEFT, CENTER, RIGHT, NONE;
  }

  public static PROP_POSITION propPosition = PROP_POSITION.RIGHT; // TODO: change to test all 3 possibilities

  public static double PLACE_TIME = 1.0;
  public static double PICKUP_TIME = 2.0;

  @Override
  public void runOpMode() throws InterruptedException {
    MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(13, -61, Math.toRadians(90)));
    processor = new RedPropThreshold();
    robot = new Robot(this);

    portal = new VisionPortal.Builder()
        .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
        .setCameraResolution(new Size(640, 480))
        .addProcessor(processor)
        .build();

    Position x = Position.NONE;
    while (opModeInInit() && !isStopRequested()) {
      x = processor.getElePos();
      telemetry.addLine("Case" + ":" + x.name());
      telemetry.addData("RED Prop Position", processor.getElePos());
      telemetry.addData("RED left box avg", processor.averagedLeftBox);
      telemetry.addData("RED right box avg", processor.averagedRightBox);
      telemetry.update();
    }

    Vector2d parkVec;
    switch (parkPosition) {
      case CENTER:
        parkVec = PARK_CENTER;
        break;
      default: // CORNER
        parkVec = PARK_CORNER;
        break;
    }

    Vector2d yellowPlacement;
    Vector2d spikePlacement;
    Vector2d whitePlacement;
    switch (x) {

      case CENTER:
        yellowPlacement = BOARD_CENTER;
        spikePlacement = SPIKE_CENTER;
        whitePlacement = BOARD_RIGHT;

        break;

      case LEFT:
        yellowPlacement = BOARD_LEFT;
        spikePlacement = SPIKE_LEFT;
        whitePlacement = BOARD_RIGHT;

        break;

      default:
        yellowPlacement = BOARD_RIGHT;
        spikePlacement = SPIKE_RIGHT;
        whitePlacement = BOARD_CENTER;
    }

//    //TODO remove this
//    Actions.runBlocking(
//        drive.actionBuilder(drive.pose)
//            .splineToLinearHeading(new Pose2d(-24, -34, Math.toRadians(0)), Math.toRadians(0))
//            .waitSeconds(999)
//            .build());
//    //

    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            // Board
            .splineToLinearHeading(new Pose2d(yellowPlacement, Math.toRadians(180)),
                Math.toRadians(180))
            .build());

    robot.setSlidePos(2400, 1);
    robot.waitTime(200);
    robot.setSlidePos(0, 0.6);

    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            // Spike
            .strafeToConstantHeading(spikePlacement)
            .waitSeconds(0.2)

            // By truss
            .setReversed(true)
            .splineToConstantHeading(BACKDROP_TRUSS, Math.toRadians(180))
            .setReversed(false)

            //through truss to corner
            .strafeToConstantHeading(AUDIENCE_TRUSS)
            .build());

    robot.flipperControl(false);
    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            //stack
            .splineToConstantHeading(STACK, Math.toRadians(90))
            .waitSeconds(PICKUP_TIME)
            .build());

    robot.flipperControl(true);
    robot.intake.setPower(0);
    robot.intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    robot.intake.setPower(1);

    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            //back
            .setReversed(true)
            .setTangent(Math.toRadians(270))
            .splineToConstantHeading(AUDIENCE_TRUSS, Math.toRadians(0))

            // Cross truss
            .strafeToConstantHeading(BACKDROP_TRUSS)

            //board
            .setReversed(true)
            .splineToConstantHeading(whitePlacement, Math.toRadians(0))
            .waitSeconds(PLACE_TIME)
            .setReversed(false)
            .build());

    robot.setSlidePos(2400, 1);
    robot.waitTime(200);
    robot.setSlidePos(1000, 0.6);
    robot.waitTime(500);
    robot.setSlidePos(2400, 1);
    robot.waitTime(200);
    robot.setSlidePos(0, 0.6);

    Actions.runBlocking(
        drive.actionBuilder(drive.pose)

            // park
            .splineToConstantHeading(parkVec, Math.toRadians(0))
            .build());

  }
}