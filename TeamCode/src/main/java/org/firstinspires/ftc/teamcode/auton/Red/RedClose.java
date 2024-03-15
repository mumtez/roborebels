package org.firstinspires.ftc.teamcode.auton.Red;

import android.util.Size;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
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
public class RedClose {

  public static Pose2d BACKDROP_START = new Pose2d(13, -61, Math.toRadians(90));

  public static double BOARD_X = 50;
  public static Vector2d BOARD_LEFT = new Vector2d(BOARD_X, -29);
  public static Vector2d BOARD_CENTER = new Vector2d(BOARD_X, -36);
  public static Vector2d BOARD_RIGHT = new Vector2d(BOARD_X, -43);

  public static Vector2d BOARD_WHITE_LEFT = new Vector2d(BOARD_X - 2, -40);
  public static Vector2d BOARD_WHITE_CENTER = new Vector2d(BOARD_X - 2, -38);
  public static Vector2d BOARD_WHITE_RIGHT = new Vector2d(BOARD_X - 2, -35);

  public static Vector2d SPIKE_LEFT = new Vector2d(7, -35);
  public static Vector2d SPIKE_CENTER = new Vector2d(13, -33);
  public static Vector2d SPIKE_RIGHT = new Vector2d(15.5, -35);

  public static Vector2d PARK_CORNER = new Vector2d(60, -60);
  public static Vector2d PARK_CENTER = new Vector2d(55, -8);

  public static Vector2d AUDIENCE_TRUSS = new Vector2d(-30, -55);
  public static Vector2d BACKDROP_TRUSS = new Vector2d(10.5, -56);
  public static Vector2d NEAR_STACK = new Vector2d(-57, -30);

  public static double STACK_Y = -29.5;
  public static double LEFT_STACK_X = -61;
  public static double CENTER_STACK_X = -61;
  public static double RIGHT_STACK_X = -61;

  public static double SPIKE_LEFT_HEADING = Math.toRadians(180);
  public static double SPIKE_RIGHT_HEADING = Math.toRadians(90);
  public static double SPIKE_CENTER_HEADING = Math.toRadians(135);

  public static VelConstraint slowVel = new TranslationalVelConstraint(40);
  public static AccelConstraint slowAccel = new ProfileAccelConstraint(-40, 40);

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

  public RedClose(LinearOpMode opMode, CycleDirection cycle, ParkPosition parkPosition, double delaySeconds) {
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
    this.processor = new RedPropThreshold();
    this.robot = new Robot(this.opMode);
  }

  public void run() {

    ElapsedTime timer = new ElapsedTime();

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
    double stackX;
    double spikeHeading;
    switch (spikeDetection) {
      case CENTER:
        yellowPlacement = BOARD_CENTER;
        spikePlacement = SPIKE_CENTER;
        whitePlacement = BOARD_WHITE_CENTER;
        spikeHeading = SPIKE_CENTER_HEADING;
        stackX = CENTER_STACK_X;
        break;

      case LEFT:
        yellowPlacement = BOARD_LEFT;
        spikePlacement = SPIKE_LEFT;
        whitePlacement = BOARD_WHITE_LEFT;
        spikeHeading = SPIKE_LEFT_HEADING;
        stackX = LEFT_STACK_X;
        break;

      default:
        yellowPlacement = BOARD_RIGHT;
        spikePlacement = SPIKE_RIGHT;
        whitePlacement = BOARD_WHITE_RIGHT;
        spikeHeading = SPIKE_RIGHT_HEADING;
        stackX = RIGHT_STACK_X;
        break;
    }

    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            // Spike
            .splineTo(spikePlacement, spikeHeading)
            // Board
            .setTangent(Math.toRadians(270))
            .splineToConstantHeading(new Vector2d(spikePlacement.x + 6, spikePlacement.y - 6), Math.toRadians(0))
            .splineToSplineHeading(new Pose2d(yellowPlacement, Math.toRadians(180)), Math.toRadians(180))
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
    robot.turnByGyro(90);
    robot.driveToBackdrop();

    // Place Pixel
    robot.setSlidePos(1500, 1);
    robot.waitTime(200);
    robot.setSlidePos(0, 1);

    // TODO: add a delay somewhere
    // CYCLE
    boolean safeToContinue = true;
    while (this.cycle != CycleDirection.NO_CYCLE && safeToContinue && timer.seconds() >= 10
        && this.opMode.opModeIsActive()) {
      switch (this.cycle) {
        case GATE:
          this.gateCycle(whitePlacement, stackX);
          break;
        case TRUSS:
          this.trussCycle(whitePlacement, stackX);
          break;
      }
      // Place collected pixels if safely crossed field
      if (drive.pose.position.x > 40) {
        robot.turnByGyro(90);
        robot.driveToBackdrop();
        robot.setSlidePos(1800, 1);
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
              .splineToConstantHeading(parkVec, Math.toRadians(0))
              .build());
    }

    // STOP
    robot.setDriveTrainPower(0, 0, 0, 0);
    robot.setSlidePower(0);
    robot.intake.setPower(0);
  }

  private void trussCycle(Vector2d whitePlacement, double stackX) {
    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            // By truss
            .splineToConstantHeading(BACKDROP_TRUSS, Math.toRadians(180))
            .build());
    // intentional break so slowdown
    robot.turnByGyro(90);

    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            //through truss to corner
            .strafeToConstantHeading(AUDIENCE_TRUSS/*, slowVel, slowAccel*/)
            //near stack
            .splineToConstantHeading(NEAR_STACK, Math.toRadians(90))
            .build());

    robot.flipperControl(true); // open
    robot.turnByGyro(90);
    robot.waitTime(200);

    Actions.runBlocking(
        drive.actionBuilder(drive.pose).strafeToConstantHeading(new Vector2d(stackX, STACK_Y))
            .build());
    robot.flipperControl(false);
    robot.waitTime(350);
    robot.flipperControl(true);
    robot.waitTime(350);
    robot.flipperControl(false);
    robot.waitTime(350);

    Actions.runBlocking(drive.actionBuilder(drive.pose).lineToX(-58).build());
    robot.flipperControl(true);
    robot.waitTime(200);
    robot.intake.setPower(0);
    robot.intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    robot.intake.setPower(1);
    robot.waitTime(1200);
    robot.intake.setPower(-1);
    robot.waitTime(400);
    robot.intake.setPower(1);

    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            //back
            .setReversed(true)
            .setTangent(Math.toRadians(270))
            .splineToConstantHeading(AUDIENCE_TRUSS, Math.toRadians(0))
            .build());
    // Intentional stop
    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            // Cross truss
            .setReversed(true)
            .strafeToConstantHeading(BACKDROP_TRUSS/*, slowVel, slowAccel*/)
            //board
            .splineToConstantHeading(whitePlacement, Math.toRadians(0))
            .build());

    robot.intake.setPower(0);
    robot.flipperControl(false);
  }

  private void gateCycle(Vector2d whitePlacement, double stackX) {

  }

  private void grabFromStack(double stackX, double stackY) {

  }
}