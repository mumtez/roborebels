package org.firstinspires.ftc.teamcode.auton;

import android.util.Size;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.odom.MecanumDrive;
import org.firstinspires.ftc.teamcode.vision.BluePropThreshold;
import org.firstinspires.ftc.teamcode.vision.Position;
import org.firstinspires.ftc.teamcode.vision.RedPropThreshold;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Autonomous(name = "Red Far Spline Autonimous")
public class SplineRedFar extends LinearOpMode {

  public FtcDashboard dash = FtcDashboard.getInstance();
  Robot robot;
  private VisionPortal portal;
  RedPropThreshold processor;

  public static Vector2d placeSpikeLeft = new Vector2d(-34, -34);
  public static Vector2d placeSpikeCenter = new Vector2d(-36, -30);
  public static Vector2d placeSpikeRight = new Vector2d(-50, -23);
  public static Vector2d placeBoardLeft = new Vector2d(50, -30);
  public static Vector2d placeBoardCenter = new Vector2d(50, -27);
  public static Vector2d placeBoardRight = new Vector2d(50, -24);

  public static int rightTurn = 270;
  public static int leftTurn = 40;
  public static int centerTurn = 90;

  @Override
  public void runOpMode() throws InterruptedException {
    MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-36, 60, Math.toRadians(-270)));
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
      telemetry.update();
    }
    waitForStart();

    Vector2d placeSpike;
    Vector2d placeBoard;
    int spikeTurn;
    switch (x) {
      case LEFT:
        placeBoard = placeBoardLeft;
        placeSpike = placeSpikeLeft;
        spikeTurn = leftTurn;
        break;
      case CENTER:
        placeBoard = placeBoardCenter;
        placeSpike = placeSpikeCenter;
        spikeTurn = centerTurn;
        break;
      default:
      case RIGHT:
        placeBoard = placeBoardRight;
        placeSpike = placeSpikeRight;
        spikeTurn = rightTurn;
        break;
    }

    if (x == Position.RIGHT) {
      Actions.runBlocking(
              drive.actionBuilder(drive.pose)
                      .strafeToConstantHeading(new Vector2d(-40, -60))
                      .strafeToConstantHeading(new Vector2d(-40, -12))
                      .strafeToLinearHeading(placeSpike, Math.toRadians(spikeTurn))
                      .build());

      robot.flipperControl(true);
      robot.setIntakePos(-100, .1);
      robot.waitTime(100);

      Actions.runBlocking(
              drive.actionBuilder(drive.pose)
                      .strafeToConstantHeading(new Vector2d(-48, -12))
                      .turnTo(Math.toRadians(180))
                      .strafeToConstantHeading(new Vector2d(-63, -12))
                      .build());
    } else {
      Actions.runBlocking(
              drive.actionBuilder(drive.pose)
                      .strafeToConstantHeading(new Vector2d(-38, -60))
                      .strafeToConstantHeading(new Vector2d(-36, -36))
                      .turnTo(Math.toRadians(spikeTurn))
                      .strafeToConstantHeading(placeSpike)
                      .build());

      robot.flipperControl(true);
      robot.setIntakePos(-100, .1);
      robot.waitTime(100);

      Actions.runBlocking(
              drive.actionBuilder(drive.pose)
                      .strafeToConstantHeading(new Vector2d(-48, -48))
                      .strafeToConstantHeading(new Vector2d(-48, -12))
                      .turnTo(Math.toRadians(180))
                      .strafeToConstantHeading(new Vector2d(-63, -12))
                      .build());
    }
    // GRAB ONE
    robot.flipperControl(false);
    Actions.runBlocking(
            drive.actionBuilder(drive.pose)
                    .strafeToConstantHeading(new Vector2d(-55, -12))
                    .build());
    robot.flipperControl(true);
    robot.intake.setMode(RunMode.RUN_WITHOUT_ENCODER);
    robot.intake.setPower(0.6);
    robot.waitTime(600);
    robot.intake.setPower(0);
    robot.flipperControl(false);

    // going back to board
    Actions.runBlocking(
            drive.actionBuilder(drive.pose)
                    .strafeToConstantHeading(new Vector2d(0, -0))
                    .setTangent(Math.toRadians(0))
                    .splineToConstantHeading(placeBoard, Math.toRadians(-270))
                    .turnTo(Math.toRadians(180))
                    .build());

    // FIRST PLACE
    robot.setSlidePos(3000, 1);
    robot.setSlidePos(0, 1);

    // JIGGLE
    robot.setSlidePower(0);
    robot.slideL.setMode(RunMode.RUN_WITHOUT_ENCODER);
    robot.slideR.setMode(RunMode.RUN_WITHOUT_ENCODER);
    robot.setSlidePower(1);
    robot.waitTime(200);
    robot.setSlidePower(-1);
    robot.waitTime(200);
    robot.setSlidePos(0, 1);

    // SECOND PLACE
    robot.setSlidePos(3000, 1);
    robot.setSlidePos(0, 1);
  }


}

