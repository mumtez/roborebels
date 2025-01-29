package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@Config
@TeleOp(name = "TELEOP", group = "TELEOP")
public class Teleop extends LinearOpMode {

  public static double START_HEADING = Math.toRadians(0);
  public static double HORIZONTAL_SPEED = 200; // INCREASE --> SLOW DOWN | DECREASE --> SPEED UP

  Robot robot;
  double horizontalPos = Intake.SLIDE_TRANSFER;
  boolean transferPos = false;
  boolean transferSlide = false;

  boolean wallPos = false;

  @Override
  public void runOpMode() throws InterruptedException {
    robot = new Robot(this);

    waitForStart();
    // START
    robot.follower.setStartingPose(new Pose(0, 0, START_HEADING));
    robot.follower.startTeleopDrive();

    robot.startSlideUpPos(Robot.VERTICAL_SLIDE_PRE_TRANSFER, 0.8);
    robot.claw.clawClose();
    robot.waitTime(500);
    robot.claw.setTransfer();
    robot.intake.rotateFlat();

    robot.setVerticalSlideMode(RunMode.RUN_WITHOUT_ENCODER);

    // LOOP
    while (opModeIsActive()) {

      int vSlideLPos = robot.slideLeft.getCurrentPosition();
      int vSlideRPos = robot.slideRight.getCurrentPosition();

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

      // Don't use because of belt skipping -- can be used later with addition of mag lim switch on slide
//      if (gamepad2.dpad_left) {
//        robot.startSlideUpPos(Robot.VERTICAL_SLIDE_DEFAULT, 0.7);
//        transferSlide = true;
//      } else if (gamepad2.dpad_up) {
//        robot.startSlideUpPos(Robot.VERTICAL_SLIDE_UP, 0.8);
//        transferSlide = false;
//      } else if (gamepad2.dpad_right) {
//        robot.startSlideUpPos(Robot.VERTICAL_SLIDE_PRE_TRANSFER, 0.7);
//        transferSlide = false;
//      }

      robot.setVerticalSlidePower(-gamepad2.left_stick_y);
      transferSlide = Math.abs(vSlideLPos - Robot.VERTICAL_SLIDE_DEFAULT) < 20;

      if (gamepad2.cross && Math.abs(vSlideLPos) > 800 && wallPos) {
        //robot.rotateIntakeBack();
        robot.claw.setTransfer();
        transferPos = true;
        wallPos = false;

      }

      //Combined transfer

      /*
      if (gamepad2.dpad_up && Math.abs(vSlideLPos) > 800){

        robot.startSlideUpPos(Robot.VERTICAL_SLIDE_PRE_TRANSFER, 0.8);

        robot.waitTime(300);

        robot.claw.setWall();
        transferPos = false;
        wallPos = true;

        robot.waitTime(300);

        robot.claw.setTransfer();
        transferPos = true;
        wallPos = false;

        robot.endSlideUpPos(Robot.VERTICAL_SLIDE_PRE_TRANSFER);
      }

       */

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


      /*
      if (gamepad2.right_bumper) {
        robot.claw.clawOpen();
      }
      else{
        robot.claw.clawClose();
      }

       */

      robot.intake.setPower(gamepad2.left_trigger - gamepad2.right_trigger);
      if (gamepad2.right_stick_y > 0.1) {
        robot.intake.rotateFlat();
      } else if (gamepad2.ps) {
        robot.intake.rotateDown();
      }

      NormalizedRGBA colors = robot.intake.senseColor(); // Important: only make 1 i2c call per loop
      double distance = robot.intake.senseDistance();

      if ((robot.team_color == 0 && colors.red > Intake.COLOR_THRESHOLD) ||    //team red and red in bot
          (robot.team_color == 1 && colors.blue > Intake.COLOR_THRESHOLD)) {   //team blue and blue in bot
        gamepad1.rumble(5);
        gamepad2.rumble(5);
      }

      // TODO: this actually means the slide is at max extension, not just "out"
      if ((horizontalPos >= Intake.SLIDE_OUT) && // Make sure slide is out and
          (
              (robot.team_color == 0 && colors.blue > Intake.COLOR_THRESHOLD) //team red and blue in bot
                  || (robot.team_color == 1 && colors.red > Intake.COLOR_THRESHOLD) //team blue and red in bot
          )
      ) {
        //outake
        //robot.intake.setPower(-1 or +1?)
        //prob check intakeColor first
      }

      telemetry.addData("Red in bot", colors.red);
      telemetry.addData("Blue in bot", colors.blue);
      telemetry.addData("color sum", colors.red + colors.blue + colors.alpha + colors.green);
      telemetry.addData("Intake dist", distance);

      telemetry.addData("V SLIDE L ENC", vSlideLPos);
      telemetry.addData("V SLIDE R ENC", vSlideRPos);
      telemetry.update();
    }
  }
}