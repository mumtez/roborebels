package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class RedCloseConfigurable extends LinearOpMode {

  @Override
  public void runOpMode() throws InterruptedException {

    CycleDirection cycleDirection = CycleDirection.TRUSS;
    ParkPosition parkPosition = ParkPosition.CORNER;

    boolean yHeld = false;
    boolean xHeld = false;

    //    this.circle = this.b;
    //    this.cross = this.a;
    //    this.triangle = this.y;
    //    this.square = this.x;

    while (opModeInInit() && !isStopRequested() && !gamepad1.start && !gamepad2.start) {
      // Swap Park Position
      if (gamepad1.x) {
        if (!xHeld) {
          if (parkPosition == ParkPosition.CORNER) {
            parkPosition = ParkPosition.CENTER;
          } else {
            parkPosition = ParkPosition.CORNER;
          }
        }
        xHeld = true;
      } else {
        xHeld = false;
      }

      // Swap Cycle Direction
      if (gamepad1.y) {
        if (!yHeld) {
          switch (cycleDirection) {
            case TRUSS:
              cycleDirection = CycleDirection.GATE;
              break;
            case GATE:
              cycleDirection = CycleDirection.NO_CYCLE;
              break;
            case NO_CYCLE:
              cycleDirection = CycleDirection.TRUSS;
              break;
          }
        }
        yHeld = true;
      } else {
        yHeld = false;
      }

      telemetry.addData("(SQUARE / X) | Park Position", parkPosition);
      telemetry.addData("(TRIANGLE / Y) | Cycle Direction", cycleDirection);
      telemetry.addLine("Press START on both controllers to lock-in configuration.");
      telemetry.update();
    }

    new RedClose(this, cycleDirection, parkPosition).run();
  }
}
