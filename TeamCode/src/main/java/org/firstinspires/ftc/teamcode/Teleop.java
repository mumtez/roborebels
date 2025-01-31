package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Robot.AllianceColor;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.VerticalSlides;

@Disabled
//@Config
@TeleOp(name = "TELEOP", group = "TELEOP")
public class Teleop extends LinearOpMode {

  public static double START_HEADING = Math.toRadians(0);
  public static double HORIZONTAL_SPEED = 100; // INCREASE --> SLOW DOWN | DECREASE --> SPEED UP

  Robot robot;

  double horizontalPos = Intake.SLIDE_TRANSFER;
  boolean transferPos = false;
  boolean transferSlide = false;
  boolean wallPos = false;

  @Override
  public void runOpMode() throws InterruptedException {
    robot = new Robot(this);

    while (opModeInInit()) {
      if (gamepad1.triangle) {
        robot.setAllianceColor(AllianceColor.RED);
      }
      if (gamepad1.circle) {
        robot.setAllianceColor(AllianceColor.BLUE);
      }

      telemetry.addData("Team: ", robot.getAllianceColor());
      telemetry.update();
    }

    // START
    robot.follower.setStartingPose(new Pose(0, 0, START_HEADING));
    robot.follower.startTeleopDrive();

    robot.slides.setTarget(VerticalSlides.DEFAULT);

    robot.claw.clawClose();
    robot.waitTime(500);
    robot.claw.setTransfer();
    robot.intake.rotateFlat();

    // LOOP
    while (opModeIsActive()) {

      if (gamepad1.left_bumper) {
        robot.follower.resetOffset();
      }

      // === FIELD CENTRIC ===
      robot.follower.setTeleOpMovementVectors(
          -gamepad1.left_stick_y,
          -gamepad1.left_stick_x,
          -gamepad1.right_stick_x,
          false);
      robot.follower.update();

      if (!(transferPos && transferSlide)) {
        horizontalPos -= gamepad2.right_stick_y / HORIZONTAL_SPEED;
        horizontalPos = Range.clip(horizontalPos, Intake.SLIDE_TRANSFER, Intake.SLIDE_OUT);
        robot.intake.setHorizontalSlidePos(horizontalPos);
      }

      if (gamepad2.dpad_left) {
        robot.slides.setTarget(VerticalSlides.TRANSFER);
      } else if (gamepad2.dpad_up) {
        robot.slides.setTarget(VerticalSlides.UP);
      } else if (gamepad2.dpad_right) {
        robot.slides.setTarget(VerticalSlides.DEFAULT);
      }
      robot.slides.updatePIDControl();

//      robot.slides.setPower(-gamepad2.left_stick_y);
//      transferSlide = Math.abs(vSlideLPos - Robot.VERTICAL_SLIDE_DEFAULT) < 20;

//      if (gamepad2.cross && Math.abs(vSlideLPos) > 800 && wallPos) {
//        //robot.rotateIntakeBack();
//        robot.claw.setTransfer();
//        transferPos = true;
//        wallPos = false;
//
//      }

      //Combined transfer

//      if (gamepad2.dpad_up && Math.abs(vSlideLPos) > 800) {
//
//        robot.slides.setTarget(Robot.VERTICAL_SLIDE_PRE_TRANSFER);
//        robot.slides.updatePIDControl();
//
//        robot.waitTime(300);
//
//        robot.claw.setWall();
//        transferPos = false;
//        wallPos = true;
//
//        robot.waitTime(300);
//
//        robot.claw.setTransfer();
//        transferPos = true;
//        wallPos = false;
//
//        robot.waitTime(300);
//
//        robot.slides.setTarget(Robot.VERTICAL_SLIDE_DEFAULT);
//
//        robot.waitTime(300);
//
//        robot.claw.clawClose();
//      }

      if (!(transferPos && transferSlide)) {
        if (gamepad2.square) {
          robot.claw.setUnder();
          transferPos = false;
          wallPos = false;
        } else if (gamepad2.circle) {
          robot.claw.setPlace();
          transferPos = false;
          wallPos = false;
        } else if (gamepad2.triangle) {
          robot.claw.setBucket();
          transferPos = false;
          wallPos = false;
        } else if (gamepad2.touchpad) {
          robot.claw.setWall();
          transferPos = false;
          wallPos = true;
        }
      }

      if (gamepad2.right_bumper) {
        robot.claw.clawOpen();
      } else if (gamepad2.left_bumper) {
        robot.claw.clawClose();
      }

      robot.intake.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
      if (gamepad2.right_stick_y > 0.1) {
        robot.intake.rotateFlat();
      } else if (gamepad2.ps) {
        robot.intake.rotateDown();
      }

      robot.intake.senseColor(); // Important: only make 1 i2c call per loop
      NormalizedRGBA colors = robot.intake.getColors();
      if (((robot.getAllianceColor() == AllianceColor.RED && colors.red > Intake.COLOR_THRESHOLD) ||
          //team red and red in bot
          (robot.getAllianceColor() == AllianceColor.BLUE
              && colors.blue > Intake.COLOR_THRESHOLD)) && !(colors.red > 0.04 && colors.blue > 0.04)) {   //team blue
        // and
        // blue in bot
        gamepad1.rumble(150);
        gamepad2.rumble(150);
        robot.intake.rgb.setPosition(robot.getAllianceColor() == AllianceColor.RED ? 0.28 : 0.63);
      } else if (colors.red > 0.01 && colors.blue > 0.01) {
        robot.intake.rgb.setPosition(.388);
      } else {
        robot.intake.rgb.setPosition(0);
      }

      // TODO: this actually means the slide is at max extension, not just "out"
      if (
          (
              (robot.getAllianceColor() == AllianceColor.RED && colors.blue > Intake.COLOR_THRESHOLD)
                  //team red and blue in bot
                  || (robot.getAllianceColor() == AllianceColor.BLUE
                  && colors.red > Intake.COLOR_THRESHOLD) //team blue and red in bot
          ) && !(colors.red > 0.04 && colors.blue > .04)
      ) {

        robot.intake.rotateFlat();
        robot.intake.setPower(-1);
        robot.waitTime(500);// TODO: don't wait time in teleop -- will remove all control from the drivers. Use a
        // separate elapsed time object or integer countdown instead so that rest of controls are not impeded
        robot.intake.setPower(0);
      }

      telemetry.addData("Team: ", robot.getAllianceColor());
      telemetry.addData("Red in bot", colors.red);
      telemetry.addData("Blue in bot", colors.blue);
      telemetry.addData("color sum", colors.red + colors.blue + colors.alpha + colors.green);

      telemetry.update();
    }
  }
}