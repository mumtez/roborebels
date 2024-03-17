package org.firstinspires.ftc.teamcode.auton.Red;

import android.util.Size;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.Objects;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.auton.CycleDirection;
import org.firstinspires.ftc.teamcode.auton.ParkPosition;
import org.firstinspires.ftc.teamcode.odom.MecanumDrive;
import org.firstinspires.ftc.teamcode.vision.Position;
import org.firstinspires.ftc.teamcode.vision.RedPropThreshold;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
public class RedFar {

  // FOR FULL TOOTH
  public static Pose2d FAR_START = new Pose2d(-13, -61, Math.toRadians(90));

  public static double BOARD_X = 49;
  public static Vector2d BOARD_LEFT = new Vector2d(BOARD_X, -29);
  public static Vector2d BOARD_CENTER = new Vector2d(BOARD_X, -36);
  public static Vector2d BOARD_RIGHT = new Vector2d(BOARD_X, -43);

  public static Vector2d BOARD_WHITE_LEFT = new Vector2d(BOARD_X - 2, -40);
  public static Vector2d BOARD_WHITE_CENTER = new Vector2d(BOARD_X - 2, -38);
  public static Vector2d BOARD_WHITE_RIGHT = new Vector2d(BOARD_X - 2, -35);

  public static Vector2d SPIKE_LEFT = new Vector2d(-46, -38);
  public static Vector2d SPIKE_CENTER = new Vector2d(-38, -33);
  public static Vector2d SPIKE_RIGHT = new Vector2d(-30, -35);

  public static Vector2d PARK_CORNER = new Vector2d(60, -64);
  public static Vector2d PARK_CENTER = new Vector2d(55, -8);

  public static Vector2d AUDIENCE_GATE = new Vector2d(-12, -6);
  public static Vector2d BACKDROP_GATE = new Vector2d(12, -6);
  public static Vector2d NEAR_STACK = new Vector2d(-60, -12);

  public static double SPIKE_LEFT_HEADING = Math.toRadians(180);
  public static double SPIKE_RIGHT_HEADING = Math.toRadians(90);
  public static double SPIKE_CENTER_HEADING = Math.toRadians(0);

  private final LinearOpMode opMode;
  private final Telemetry telemetry;
  private final HardwareMap hardwareMap;

  private final double delaySeconds;
  private final CycleDirection cycle;
  private final ParkPosition parkPosition;
  private final Vector2d parkVec;
  private final Robot robot;
  private final RedPropThreshold processor;
  private final MecanumDrive drive;
  public FtcDashboard dash = FtcDashboard.getInstance();

  public RedFar(LinearOpMode opMode, CycleDirection cycle, ParkPosition parkPosition,
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

    this.drive = new MecanumDrive(hardwareMap, FAR_START);
    this.processor = new RedPropThreshold();
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
    Vector2d whitePlacement;
    switch (spikeDetection) {
      case CENTER:
        yellowPlacement = BOARD_CENTER;
        whitePlacement = BOARD_WHITE_CENTER;
        Actions.runBlocking(drive.actionBuilder(drive.pose)
            .splineToConstantHeading(SPIKE_CENTER, SPIKE_CENTER_HEADING)
            .strafeToConstantHeading(new Vector2d(FAR_START.position.x, FAR_START.position.y + 5))
            .strafeToLinearHeading(NEAR_STACK, Math.toRadians(180))
            .build());
        break;

      case LEFT:
        yellowPlacement = BOARD_LEFT;
        whitePlacement = BOARD_WHITE_LEFT;
        Actions.runBlocking(drive.actionBuilder(drive.pose)
            .splineTo(SPIKE_LEFT, SPIKE_LEFT_HEADING)
            .setTangent(0)
            .splineToConstantHeading(new Vector2d(SPIKE_LEFT.x + 10, NEAR_STACK.y), Math.toRadians(180))
            .strafeToConstantHeading(NEAR_STACK)
            .build());
        break;

      default:
        yellowPlacement = BOARD_RIGHT;
        whitePlacement = BOARD_WHITE_RIGHT;
        Actions.runBlocking(drive.actionBuilder(drive.pose)
            .splineTo(SPIKE_RIGHT, SPIKE_RIGHT_HEADING)
            .strafeToConstantHeading(NEAR_STACK)
            .turnTo(Math.toRadians(180))
            .build());
        break;
    }

    // Get 1 from stack
    robot.setSweepOut(true);
    robot.waitTime(1000);
    robot.driveToStack();
    robot.turnByGyro(115);
    robot.setSweepOut(false);
    robot.waitTime(500);
    robot.turnByGyro(90);
    robot.intake.setPower(1);
    robot.setGate(false);

    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            .lineToX(-65)
            .waitSeconds(.5)
            .strafeToConstantHeading(new Vector2d(-61, -15))
            .lineToX(-65)
            .lineToX(-60)
            .build());

    robot.intake.setPower(-.8);
    robot.waitTime(500);

    // TODO CONFIG DELAY IF NEEDED
    robot.waitTime(100);

    // Go to board through gate
    Actions.runBlocking(drive.actionBuilder(drive.pose)
        .splineToConstantHeading(AUDIENCE_GATE, Math.toRadians(0))
        .splineToConstantHeading(BACKDROP_GATE, Math.toRadians(0))
        .splineToConstantHeading(yellowPlacement, Math.toRadians(270))
        .build());

    // Place 2
    robot.turnByGyro(90);
    robot.driveToBackdrop();
    robot.setSlidePos(2000, 1);
    robot.waitTime(200);
    robot.setSlidePos(0, 1);

    // CYCLE
    boolean safeToContinue = true;
    if (this.cycle != CycleDirection.NO_CYCLE) {
      this.gateCycle(whitePlacement);
    }

    // Place collected pixels if safely crossed field
    if (drive.pose.position.x > 40) {
      robot.turnByGyro(90);
      robot.driveToBackdrop();
      robot.setSlidePos(2000, 1);
      robot.waitTime(200);
      robot.setSlidePos(0, 1);
    } else {
      safeToContinue = false;
    }

    // PARK if safe
    if (safeToContinue) {
      Actions.runBlocking(
          drive.actionBuilder(drive.pose)
              .strafeToConstantHeading(new Vector2d(drive.pose.position.x, parkVec.y))
              .build());
    }

    // STOP
    robot.setDriveTrainPower(0, 0, 0, 0);
    robot.setSlidePower(0);
    robot.intake.setPower(0);
  }

  private void gateCycle(Vector2d whitePlacement) {
    Actions.runBlocking(drive.actionBuilder(drive.pose)
        .setReversed(true)
        .splineToConstantHeading(BACKDROP_GATE, Math.toRadians(180))
        .splineToConstantHeading(AUDIENCE_GATE, Math.toRadians(180))
        .splineToConstantHeading(NEAR_STACK, Math.toRadians(270))
        .build());

    // Get 2 more from stack
    robot.driveToStack();
    robot.turnByGyro(90);
    robot.intake.setPower(1);
    robot.setGate(false);

    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            .lineToX(-65)
            .waitSeconds(.5)
            .strafeToConstantHeading(new Vector2d(-61, -15))
            .lineToX(-65)
            .lineToX(-60)
            .build());

    robot.intake.setPower(-.8);
    robot.waitTime(500);

    // return to board
    Actions.runBlocking(drive.actionBuilder(drive.pose)
        .splineToConstantHeading(AUDIENCE_GATE, Math.toRadians(0))
        .splineToConstantHeading(BACKDROP_GATE, Math.toRadians(0))
        .splineToConstantHeading(whitePlacement, Math.toRadians(270))
        .build());
  }


  private void grabFromStack(double stackX, double stackY) {

  }
}