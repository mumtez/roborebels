package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.NewRobot;
import org.firstinspires.ftc.teamcode.subsystems.HorizontalSlides;
import org.firstinspires.ftc.teamcode.subsystems.VerticalSlides;

@Config
public class BaseTeleop {

  public enum ModeState {
    SPEC_WALL, SPEC_PRE_CLIP, SPEC_CLIP, SPEC_POST_CLIP,
    BUCKET_INTAKING, BUCKET_TRANSFER, BUCKET_POST_TRANSFER, BUCKET_PLACE, BUCKET_POST_PLACE
  }

  public static double HORIZONTAL_MODIFIER = 0.1;

  final NewRobot robot;
  final LinearOpMode opMode;
  final Telemetry telemetry;

  boolean hangOverride = false;
  boolean specimenMode = false;
  ModeState state = ModeState.BUCKET_INTAKING;
  final ElapsedTime stateTimer = new ElapsedTime();

  boolean intakeFlat = true;

  Gamepad currentGamepad1 = new Gamepad();
  Gamepad currentGamepad2 = new Gamepad();
  Gamepad previousGamepad1 = new Gamepad();
  Gamepad previousGamepad2 = new Gamepad();

  public BaseTeleop(LinearOpMode opMode, NewRobot robot) {
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
    robot.slides.setTarget(VerticalSlides.TRANSFER);
    robot.claw.setTransfer();
    robot.claw.clawOpen();
    robot.intake.senseColor();
    robot.intake.senseDistance();
    stateTimer.reset();

    // --- LOOP ---
    while (opMode.opModeIsActive()) {
      updateGamepads();

      robot.horSlide.updatePosition();

      if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
        robot.imu.resetYaw();
      }

      // Hang Override
      if (currentGamepad1.back && !previousGamepad1.back) {
        hangOverride = !hangOverride;
        robot.horSlide.setTarget(HorizontalSlides.TRANSFER_POS);
      }

      // Mode Switch
      if (currentGamepad2.back && !previousGamepad2.back) {
        specimenMode = !specimenMode;
      }

      // Field Centric Drive

      double y = -currentGamepad1.left_stick_y;
      double x = currentGamepad1.left_stick_x;
      double rx = currentGamepad1.right_stick_x;

      double botHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

      double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
      double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
      rotX = rotX * 1.1;  // Counteract imperfect strafing

      double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
      double frontLeftPower = (rotY + rotX + rx) / denominator;
      double backLeftPower = (rotY - rotX + rx) / denominator;
      double frontRightPower = (rotY - rotX - rx) / denominator;
      double backRightPower = (rotY + rotX - rx) / denominator;

      robot.fr.setPower(frontRightPower);
      robot.fl.setPower(frontLeftPower);
      robot.br.setPower(backRightPower);
      robot.bl.setPower(backLeftPower);

      // Manual Control
      if (hangOverride) {
        hangControls();
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

  public void hangControls() {
    robot.slides.setPower(currentGamepad1.right_trigger - currentGamepad1.left_trigger);

    robot.intake.setPower(0);
    robot.intake.rotateFlat();

    robot.claw.setInit();

    robot.horSlide.updatePIDControl();
  }

  public void specimenModeUpdate() {
    switch (state) {
      case SPEC_WALL:
        if (currentGamepad2.right_bumper) {
          robot.claw.clawClose();
        } else {
          robot.claw.clawOpenWall();
          stateTimer.reset();
        }

        // If claw closed for 500ms, move to placement pos
        if (stateTimer.milliseconds() > 500) {
          robot.claw.setPlace();
          state = ModeState.SPEC_PRE_CLIP;
        }
        break;

      // TRIANGLE --> CLIP | SQUARE --> WALL
      case SPEC_PRE_CLIP:
        // TODO: if can't do drive-in clip, this needs to move the arm
        if (currentGamepad2.triangle) {
          state = ModeState.SPEC_CLIP;
        }
        if (currentGamepad2.square) {
          robot.claw.setWall();
          state = ModeState.SPEC_WALL;
        }
        break;

      // CIRCLE --> PRE-CLIP | SQUARE --> OPEN CLAW
      case SPEC_CLIP:
        robot.slides.setTarget(VerticalSlides.BAR_PLACE);
        if (currentGamepad2.square) {
          robot.claw.clawOpen();
          state = ModeState.SPEC_POST_CLIP;
          stateTimer.reset();
        }
        break;

      case SPEC_POST_CLIP:
        if (stateTimer.milliseconds() > 100) {
          robot.slides.setTarget(VerticalSlides.TRANSFER);
          if (robot.slides.atTarget()){
            robot.claw.setWall();
          }
          state = ModeState.SPEC_WALL;
        }
        break;

      // Prev state bucket mode --> move to WALL
      default:
        state = ModeState.SPEC_WALL;
        robot.claw.setWall();
        robot.slides.setTarget(VerticalSlides.TRANSFER);
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
          robot.slides.setTarget(VerticalSlides.TRANSFER);
          robot.intake.rotateFlat();
          robot.claw.clawOpen();
          robot.claw.setTransfer();
          robot.horSlide.setTarget(HorizontalSlides.TRANSFER_POS);

          state = ModeState.BUCKET_TRANSFER;
          stateTimer.reset();
        }
        break;

      case BUCKET_TRANSFER:
        robot.horSlide.updatePIDControl();
        if (robot.horSlide.atTarget() && robot.slides.atTarget()) {
          robot.claw.clawClose();
          stateTimer.reset();
          state = ModeState.BUCKET_POST_TRANSFER;
        }
        break;

      case BUCKET_POST_TRANSFER:
        robot.horSlide.updatePIDControl();
        if (stateTimer.milliseconds() > 50) {
          if (currentGamepad2.square) {
            robot.claw.clawOpen();
            stateTimer.reset();
            state = ModeState.BUCKET_INTAKING;
          }

          if (currentGamepad2.triangle && !previousGamepad2.triangle) {
            robot.slides.setTarget(VerticalSlides.UP);
          }
          if (currentGamepad2.circle && !previousGamepad2.circle) {
            robot.slides.setTarget(VerticalSlides.LOWER_BUCKET);
          }

          // If the vert slides are moving to either bucket height
          int slideTarget = robot.slides.getTarget();
          if (slideTarget == VerticalSlides.UP || slideTarget == VerticalSlides.LOWER_BUCKET) {
            // TODO TUNE THRESHOLD FOR OPTIMAL ARM TURN
            if (robot.slides.atSetTarget(100, slideTarget - 600)) {
              robot.claw.setBucket();
              state = ModeState.BUCKET_PLACE;
              stateTimer.reset();
            }
          }
        }
        break;

      case BUCKET_PLACE:
        robot.horSlide.updatePIDControl();
        //TODO: timer needed here?
        if (stateTimer.milliseconds() > 500 && currentGamepad2.square) {
          robot.claw.clawOpen();
          state = ModeState.BUCKET_POST_PLACE;
          stateTimer.reset();
        }
        // TODO: cancel raise button
        break;

      case BUCKET_POST_PLACE:
        robot.horSlide.updatePIDControl();
        if (stateTimer.milliseconds() > 500) {
          robot.claw.setTransfer();
          robot.slides.setTarget(VerticalSlides.TRANSFER);
          state = ModeState.BUCKET_INTAKING;
          stateTimer.reset();
        }
        break;

      // Prev state spec mode --> move to INTAKING
      default:
        state = ModeState.BUCKET_INTAKING;
        robot.claw.setTransfer();
        robot.slides.setTarget(VerticalSlides.TRANSFER);
        break;
    }
    robot.slides.updatePIDControl();
  }

  public void intakeControl() {

    double hSlidePow = -currentGamepad2.right_stick_y;

    if ((hSlidePow < -0.05 && robot.horSlide.position < 100)
        || (hSlidePow > 0.05 && robot.horSlide.position > HorizontalSlides.OUT_POS - 100)) {
      robot.horSlide.setPower(hSlidePow * HORIZONTAL_MODIFIER);
      robot.horSlide.setTarget(robot.horSlide.position);
    } else if (Math.abs(hSlidePow) > 0.05) {
      robot.horSlide.setPower(hSlidePow);
      robot.horSlide.setTarget(robot.horSlide.position);
    } else {
      // User is NOT controlling the motor, enable PID to hold position
      robot.horSlide.updatePIDControl();
    }

    intakeFlat = !currentGamepad2.dpad_down && !(currentGamepad2.right_trigger > 0.1);

    robot.intake.update(
        currentGamepad2.right_trigger - currentGamepad2.left_trigger,
        intakeFlat,
        robot.getAllianceColor()
    );

    // TODO: possibly causing a lot of cycle delay? maybe theres a better way to do this instead
    //  of sending call for effect every loop
    if (robot.intake.validSampleIn(robot.getAllianceColor())) {
      opMode.gamepad1.rumble(300);
      opMode.gamepad2.rumble(300);
    }
  }

  public void updateTelemetry() {
    telemetry.addData("MODE", specimenMode ? "SPECIMEN" : "BUCKET");
    telemetry.addData("OVERRIDE", hangOverride);
    telemetry.addData("STATE", state);
    telemetry.addData("STATE Timer", stateTimer.milliseconds());
    telemetry.update();
  }
}
