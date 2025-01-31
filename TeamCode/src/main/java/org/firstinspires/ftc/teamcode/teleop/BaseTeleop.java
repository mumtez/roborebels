package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.VerticalSlides;

@Config
public class BaseTeleop {

  public enum ModeState {
    SPEC_WALL, SPEC_PRE_CLIP, SPEC_CLIP, SPEC_POST_CLIP,
    BUCKET_INTAKING, BUCKET_PRE_TRANSFER, BUCKET_TRANSFER, BUCKET_POST_TRANSFER, BUCKET_PLACE, BUCKET_POST_PLACE
  }

  public static double START_HEADING = Math.toRadians(0);
  public static double HORIZONTAL_SPEED = 100;

  final Robot robot;
  final LinearOpMode opMode;
  final Telemetry telemetry;

  double horizontalPos = Intake.SLIDE_TRANSFER;
  boolean manualOverride = false;
  boolean specimenMode = false;
  ModeState state = ModeState.BUCKET_INTAKING;
  final ElapsedTime stateTimer = new ElapsedTime();

  boolean intakeFlat = true;

  Gamepad currentGamepad1 = new Gamepad();
  Gamepad currentGamepad2 = new Gamepad();
  Gamepad previousGamepad1 = new Gamepad();
  Gamepad previousGamepad2 = new Gamepad();

  public BaseTeleop(LinearOpMode opMode, Robot robot) {
    this.opMode = opMode;
    this.telemetry = opMode.telemetry;
    this.robot = robot;
  }

  private void updateGamepads() {
    previousGamepad1.copy(currentGamepad1);
    previousGamepad2.copy(currentGamepad2);
    currentGamepad1.copy(this.opMode.gamepad1);
    currentGamepad2.copy(this.opMode.gamepad2);
  }

  public void run() {
    // --- INIT ---

    // --- INIT LOOP ---
    while (this.opMode.opModeInInit()) {
      updateGamepads();

      telemetry.addData("ALLIANCE COLOR", robot.getAllianceColor());
      telemetry.update();
    }

    // --- START ---
    robot.follower.setStartingPose(new Pose(0, 0, START_HEADING));
    robot.follower.startTeleopDrive();
    robot.slides.setTarget(VerticalSlides.DEFAULT);
    robot.claw.setTransfer();
    robot.intake.senseColor();
    robot.intake.senseDistance();
    stateTimer.reset();

    // --- LOOP ---
    while (opMode.opModeIsActive()) {
      updateGamepads();

      // Manual Override
      if (currentGamepad1.back && !previousGamepad1.back) {
        manualOverride = !manualOverride;
      }

      // Mode Switch
      if (currentGamepad2.back && !previousGamepad2.back) {
        specimenMode = !specimenMode;
      }

      // Field Centric Drive
      robot.follower.setTeleOpMovementVectors(
          -currentGamepad1.left_stick_y,
          -currentGamepad1.left_stick_x,
          -currentGamepad1.right_stick_x,
          false);
      robot.follower.update();

      // Manual Control
      if (manualOverride) {
        manualControls();
      } else {
        if (specimenMode) {
          specimenModeUpdate();
        } else {
          bucketModeUpdate();
        }
      }

      updateTelemetry();
    }
  }

  // TODO: fill out
  public void manualControls() {
    // V SLIDES
    robot.slides.setPower(-currentGamepad2.left_stick_y);

    // H SLIDES
    horizontalPos -= currentGamepad2.right_stick_y / HORIZONTAL_SPEED;
    horizontalPos = Range.clip(horizontalPos, Intake.SLIDE_TRANSFER, Intake.SLIDE_OUT);
    robot.intake.setHorizontalSlidePos(horizontalPos);

    // INTAKE
    robot.intake.setPower(currentGamepad2.right_trigger - currentGamepad2.left_trigger);
    if (currentGamepad2.right_stick_y > 0.1) {
      robot.intake.rotateFlat();
    } else if (currentGamepad2.ps) {
      robot.intake.rotateDown();
    }

    // CLAW
  }


  public void specimenModeUpdate() {
    switch (state) {
      case SPEC_WALL:
        if (currentGamepad2.right_bumper) {
          robot.claw.clawClose();
        } else {
          robot.claw.clawOpen();
          stateTimer.reset();
        }

        // If claw closed for 500ms, move to pre-clip
        if (stateTimer.milliseconds() > 500) {
          robot.claw.setUnder();
          state = ModeState.SPEC_PRE_CLIP;
        }
        break;

      // TRIANGLE --> CLIP | SQUARE --> WALL
      case SPEC_PRE_CLIP:
        if (currentGamepad2.triangle) {
          robot.claw.setPlace();
          state = ModeState.SPEC_CLIP;
        }
        if (currentGamepad2.square) {
          robot.claw.setWall();
          state = ModeState.SPEC_WALL;
        }
        break;

      // CIRCLE --> PRE-CLIP | SQUARE --> OPEN CLAW, WALL
      case SPEC_CLIP:
        if (currentGamepad2.circle) {
          robot.claw.setUnder();
          state = ModeState.SPEC_PRE_CLIP;
        }
        if (currentGamepad2.square) {
          robot.claw.clawOpen();
          state = ModeState.SPEC_POST_CLIP;
          stateTimer.reset();
        }
        break;

      case SPEC_POST_CLIP:
        if (stateTimer.milliseconds() > 500) {
          robot.claw.setWall();
          state = ModeState.SPEC_WALL;
        }
        break;

      // Prev state bucket mode --> move to WALL
      default:
        state = ModeState.SPEC_WALL;
        robot.slides.setTarget(VerticalSlides.DEFAULT);
        robot.claw.setWall();
        break;
    }
    robot.slides.updatePIDControl();

    // Independent Intake Control
    intakeControl();
  }

  public void bucketModeUpdate() {
    switch (state) {
      case BUCKET_INTAKING:
        intakeControl();

        if (currentGamepad2.cross) {
          state = ModeState.BUCKET_PRE_TRANSFER;
          robot.intake.rotateFlat();
          robot.intake.setHorizontalSlidePos(Intake.SLIDE_TRANSFER);
          robot.slides.setTarget(VerticalSlides.DEFAULT);
          robot.claw.setTransfer();
          robot.claw.clawOpen();
          stateTimer.reset();
        }
        break;

      case BUCKET_PRE_TRANSFER:
        if (stateTimer.milliseconds() > 500) {
          robot.slides.setTarget(VerticalSlides.TRANSFER);
          state = ModeState.BUCKET_TRANSFER;
          stateTimer.reset();
        }
        break;

      case BUCKET_TRANSFER:
        if (stateTimer.milliseconds() > 500) {
          robot.claw.clawClose();
          state = ModeState.BUCKET_POST_TRANSFER;
          stateTimer.reset();
        }
        break;

      case BUCKET_POST_TRANSFER:
        if (stateTimer.milliseconds() > 500) {
          if (robot.slides.getTarget() != VerticalSlides.DEFAULT) {
            robot.slides.setTarget(VerticalSlides.DEFAULT);
          }
          if (currentGamepad2.cross) {
            robot.claw.clawOpen();
            stateTimer.reset();
            state = ModeState.BUCKET_PRE_TRANSFER;
          }
          if (currentGamepad2.triangle) {
            robot.slides.setTarget(VerticalSlides.UP);
            robot.claw.setBucket();
            state = ModeState.BUCKET_PLACE;
            stateTimer.reset();
          }
        }
        break;

      case BUCKET_PLACE:
        if (stateTimer.milliseconds() > 500 && currentGamepad2.square) {
          robot.claw.clawOpen();
          state = ModeState.BUCKET_POST_PLACE;
          stateTimer.reset();
        }
        break;

      case BUCKET_POST_PLACE:
        if (stateTimer.milliseconds() > 500) {
          robot.claw.setTransfer();
          robot.slides.setTarget(VerticalSlides.DEFAULT);
          state = ModeState.BUCKET_INTAKING;
          stateTimer.reset();
        }
        break;

      // Prev state spec mode --> move to INTAKING
      default:
        state = ModeState.BUCKET_INTAKING;
        robot.slides.setTarget(VerticalSlides.DEFAULT);
        robot.claw.setTransfer();
        break;
    }
    robot.slides.updatePIDControl();
  }

  public void intakeControl() {
    horizontalPos -= currentGamepad2.right_stick_y / HORIZONTAL_SPEED;
    horizontalPos = Range.clip(horizontalPos, Intake.SLIDE_TRANSFER, Intake.SLIDE_OUT);

    if (currentGamepad2.dpad_down || currentGamepad2.right_trigger > 0.1) {
      intakeFlat = false;
    } else {
      intakeFlat = true;
    }

    robot.intake.update(
        currentGamepad2.right_trigger - currentGamepad2.left_trigger,
        intakeFlat,
        horizontalPos,
        robot.getAllianceColor()
    );
  }

  public void updateTelemetry() {
    telemetry.addData("MODE", specimenMode ? "SPECIMEN" : "BUCKET");
    telemetry.addData("OVERRIDE", manualOverride);
    telemetry.addData("STATE", state);

    telemetry.addData("Dist", robot.intake.getDist());
    telemetry.addData("Colors RED ", robot.intake.getColors().red);
    telemetry.addData("Colors BLUE ", robot.intake.getColors().blue);

    telemetry.update();
  }
}
