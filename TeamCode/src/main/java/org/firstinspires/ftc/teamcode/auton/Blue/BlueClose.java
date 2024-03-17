package org.firstinspires.ftc.teamcode.auton.Blue;

import android.util.Size;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.Objects;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.auton.CycleDirection;
import org.firstinspires.ftc.teamcode.auton.ParkPosition;
import org.firstinspires.ftc.teamcode.odom.MecanumDrive;
import org.firstinspires.ftc.teamcode.vision.BluePropThreshold;
import org.firstinspires.ftc.teamcode.vision.Position;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
public class BlueClose {

  // FOR NO TEETH
  public static Pose2d BACKDROP_START = new Pose2d(13 - 1, 61, Math.toRadians(270));

  public static double BOARD_X = 49;
  public static Vector2d BOARD_LEFT = new Vector2d(BOARD_X, 29);
  public static Vector2d BOARD_CENTER = new Vector2d(BOARD_X, 36);
  public static Vector2d BOARD_RIGHT = new Vector2d(BOARD_X, 43);

  public static Vector2d BOARD_WHITE_LEFT = new Vector2d(BOARD_X - 2, 40);
  public static Vector2d BOARD_WHITE_CENTER = new Vector2d(BOARD_X - 2, 38);
  public static Vector2d BOARD_WHITE_RIGHT = new Vector2d(BOARD_X - 2, 35);

  public static Vector2d SPIKE_LEFT = new Vector2d(7, 35);
  public static Vector2d SPIKE_CENTER = new Vector2d(13, 33);
  public static Vector2d SPIKE_RIGHT = new Vector2d(15.5, 35);

  public static Vector2d PARK_CORNER = new Vector2d(60, 64);
  public static Vector2d PARK_CENTER = new Vector2d(55, 8);

  public static Vector2d AUDIENCE_TRUSS = new Vector2d(-38, 62);
  public static Vector2d BACKDROP_TRUSS = new Vector2d(10.5, 62);
  public static Vector2d NEAR_STACK = new Vector2d(-57, 36);

  public static double SPIKE_LEFT_HEADING = Math.toRadians(180);
  public static double SPIKE_RIGHT_HEADING = Math.toRadians(270);
  public static double SPIKE_CENTER_HEADING = Math.toRadians(225);

  //public static VelConstraint slowVel = new TranslationalVelConstraint(40);
  //public static AccelConstraint slowAccel = new ProfileAccelConstraint(-40, 40);

  private final LinearOpMode opMode;
  private final Telemetry telemetry;
  private final HardwareMap hardwareMap;

  private final double delaySeconds;
  private final CycleDirection cycle;
  private final ParkPosition parkPosition;
  private final Vector2d parkVec;
  private final Robot robot;
  private final BluePropThreshold processor;
  private final MecanumDrive drive;
  public FtcDashboard dash = FtcDashboard.getInstance();

  public BlueClose(LinearOpMode opMode, CycleDirection cycle, ParkPosition parkPosition,
      double delaySeconds) {
    this.opMode = opMode;
    this.telemetry = new MultipleTelemetry(dash.getTelemetry(), opMode.telemetry);
    this.hardwareMap = opMode.hardwareMap;
    this.cycle = cycle;
    this.delaySeconds = delaySeconds;

    this.parkPosition = parkPosition;
    if (Objects.requireNonNull(parkPosition) == ParkPosition.CENTER) {
      this.parkVec = PARK_CENTER;
    } else {
      this.parkVec = PARK_CORNER;
    }

    this.drive = new MecanumDrive(hardwareMap, BACKDROP_START);
    this.processor = new BluePropThreshold();
    this.robot = new Robot(this.opMode);
  }

  public void run() {
    VisionPortal portal = new VisionPortal.Builder()
        .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
        .setCameraResolution(new Size(640, 480))
        .addProcessor(processor)
        .build();
    // DETECT SPIKE
    Position spikeDetection = Position.NONE;
    while (opMode.opModeInInit() && !opMode.isStopRequested()) {
      spikeDetection = processor.getElePos();
      telemetry.addLine("Case" + ":" + spikeDetection.name());
      telemetry.addData("RED Prop Position", processor.getElePos());
      telemetry.addData("RED left box avg", processor.averagedLeftBox);
      telemetry.addData("RED right box avg", processor.averagedRightBox);
      telemetry.addLine();
      telemetry.addData("Cycle Direction", this.cycle);
      telemetry.addData("Park", this.parkPosition);
      telemetry.update();
    }

    Vector2d yellowPlacement;
    Vector2d spikePlacement;
    Vector2d whitePlacement;
    double spikeHeading;
    switch (spikeDetection) {
      case CENTER:
        yellowPlacement = BOARD_CENTER;
        spikePlacement = SPIKE_CENTER;
        whitePlacement = BOARD_WHITE_CENTER;
        spikeHeading = SPIKE_CENTER_HEADING;
        break;

      case RIGHT:
        yellowPlacement = BOARD_LEFT;
        spikePlacement = SPIKE_LEFT;
        whitePlacement = BOARD_WHITE_LEFT;
        spikeHeading = SPIKE_LEFT_HEADING;
        break;

      default:
        yellowPlacement = BOARD_RIGHT;
        spikePlacement = SPIKE_RIGHT;
        whitePlacement = BOARD_WHITE_RIGHT;
        spikeHeading = SPIKE_RIGHT_HEADING;
        break;
    }

    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            // Spike
            .splineTo(spikePlacement, spikeHeading)
            // Board
            .setTangent(Math.toRadians(90))
            .splineToConstantHeading(new Vector2d(spikePlacement.x + 6, spikePlacement.y + 6),
                Math.toRadians(0))
            .splineToSplineHeading(new Pose2d(yellowPlacement, Math.toRadians(180)),
                Math.toRadians(180))
            .build()
    );

    // TODO: check below for faster
//        drive.actionBuilder(drive.pose)
//            // Spike
//            .splineTo(spikePlacement, spikeHeading)
//            // Board
//            .setReversed(true)
//            .splineTo(yellowPlacement,
//                Math.toRadians(180))
//            .build());

    // Adjust Position
    robot.turnByGyro(270);

    // Place Pixel
    robot.setSlidePos(1500, 1);
    robot.waitTime(200);
    //robot.setSlidePos(0, 1);
    robot.slideL.setTargetPosition(0);
    robot.slideR.setTargetPosition(0);
    robot.slideL.setPower(1);
    robot.slideR.setPower(1);

    // TODO CONFIG DELAY IF NEEDED
    robot.waitTime(100);

    // CYCLE
    boolean safeToContinue = true;
    if (this.cycle != CycleDirection.NO_CYCLE) {
      switch (this.cycle) {
        case GATE:
          this.gateCycle(whitePlacement);
          break;
        case TRUSS:
          this.trussCycle(whitePlacement);
          break;
      }
      // Place collected pixels if safely crossed field
      if (drive.pose.position.x > 40) {
        robot.turnByGyro(270);
        robot.driveToBackdrop();
        robot.setSlidePos(2000, 1);
        robot.waitTime(200);
        robot.setSlidePos(0, 1);
      } else {
        safeToContinue = false;
      }
    }

    // PARK if safe
    if (safeToContinue) {
      Actions.runBlocking(
          drive.actionBuilder(drive.pose)
              .strafeToConstantHeading(new Vector2d(drive.pose.position.x - 1, parkVec.y))
              .build());
    }

    // STOP
    robot.setDriveTrainPower(0, 0, 0, 0);
    robot.setSlidePower(0);
    robot.intake.setPower(0);
  }

  private void trussCycle(Vector2d whitePlacement) {
    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            // By truss
            .splineToConstantHeading(new Vector2d(BACKDROP_TRUSS.x, 50), Math.toRadians(180))
            .build());
    // intentional break so slowdown

    // Align
    ElapsedTime sweepTime = new ElapsedTime();
    robot.setSweepOut(true);
    robot.turnByGyro(270);
    robot.driveToRightWall();
    drive.pose = new Pose2d(new Vector2d(BACKDROP_TRUSS.x, 56), Math.toRadians(180));
    Actions.runBlocking(drive.actionBuilder(drive.pose)
        .strafeToConstantHeading(BACKDROP_TRUSS)
        .build()
    );
    robot.flipperControl(true); // open
    robot.turnByGyro(270);
    drive.pose = new Pose2d(BACKDROP_TRUSS, Math.toRadians(180));
    while (opMode.opModeIsActive() && sweepTime.milliseconds() < 700) {
    }

    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            //through truss to corner
            .strafeToConstantHeading(AUDIENCE_TRUSS/*, slowVel, slowAccel*/)
            //near stack
            .splineToConstantHeading(NEAR_STACK, Math.toRadians(225))
            .build());

    robot.driveToStack();
    robot.turnByGyro(65);
    robot.setSweepOut(false);
    robot.waitTime(500);
    robot.turnByGyro(270);
    robot.intake.setPower(1);
    robot.setGate(false);

    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            .lineToX(-65)
            .waitSeconds(.5)
            .strafeToConstantHeading(new Vector2d(-62, 34))
            .lineToX(-65)
            .lineToX(-60)
            .build());

    robot.intake.setPower(-.8);
    robot.waitTime(500);

    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            //back
            .setReversed(true)
            .setTangent(Math.toRadians(90))
            .splineToConstantHeading(new Vector2d(AUDIENCE_TRUSS.x, 56), Math.toRadians(0))
            .build());
    // Intentional stop

    robot.turnByGyro(270);
    robot.driveToRightWall();
    robot.turnByGyro(270);
    drive.pose = new Pose2d(new Vector2d(AUDIENCE_TRUSS.x, 56), Math.toRadians(180));

    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            .strafeToConstantHeading(AUDIENCE_TRUSS)
            // Cross truss
            .setReversed(true)
            .strafeToConstantHeading(BACKDROP_TRUSS)
            //board
            .splineToConstantHeading(whitePlacement, Math.toRadians(0))
            .build());

    robot.setGate(true);
  }

  private void gateCycle(Vector2d whitePlacement) {
    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            .setTangent(180)
            .splineToConstantHeading(new Vector2d(24, 4), Math.toDegrees(180))
            .splineToConstantHeading(new Vector2d(-57, 8), Math.toDegrees(180))
            .build());
    robot.setSweepOut(true);
    robot.waitTime(2000);
    robot.driveToStack();
    robot.turnByGyro(65);
    robot.setSweepOut(false);
    robot.waitTime(500);
    robot.turnByGyro(270);
    robot.intake.setPower(1);
    robot.setGate(false);
    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            .lineToX(-65)
            .waitSeconds(.5)
            .strafeToConstantHeading(new Vector2d(-62, 10))
            .lineToX(-65)
            .lineToX(-60)
            .build());

    robot.intake.setPower(-.8);
    robot.waitTime(500);

    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            .setTangent(Math.toDegrees(270))
            .splineToConstantHeading(new Vector2d(24, 0), 0)
            .splineToConstantHeading(whitePlacement, Math.toDegrees(0))
            .build());
  }

  private void grabFromStack(double stackX, double stackY) {

  }
}