package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Canvas;
import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

@Config
public class BluePropThreshold implements VisionProcessor {

  public double averagedLeftBox;
  public double averagedRightBox;
  public static double blueThreshold = 0.15;
  private final Rect RIGHT_RECTANGLE = new Rect(
      new Point(0, 0),
      new Point(319, 479)
  );
  private final Rect LEFT_RECTANGLE = new Rect(
      new Point(320, 0),
      new Point(639, 479)
  );

  Mat testMat = new Mat();
  Mat finalMat = new Mat();
  private Position elePos = Position.NONE;

  @Override
  public void init(int width, int height, CameraCalibration calibration) {
    // Useless
  }

  @Override
  public Object processFrame(Mat frame, long captureTimeNanos) {
    Imgproc.cvtColor(frame, testMat, Imgproc.COLOR_RGB2HSV);

    Scalar HSVBlueLower = new Scalar(110, 100, 20);
    Scalar HSVBlueUpper = new Scalar(130, 255, 255);

    Core.inRange(testMat, HSVBlueLower, HSVBlueUpper, finalMat);

    testMat.release();

    double leftBox = Core.sumElems(finalMat.submat(LEFT_RECTANGLE)).val[0];
    double rightBox = Core.sumElems(finalMat.submat(RIGHT_RECTANGLE)).val[0];

    averagedLeftBox = leftBox / LEFT_RECTANGLE.area() / 255;
    averagedRightBox = rightBox / RIGHT_RECTANGLE.area() / 255; //Makes value [0,1]
    if (averagedLeftBox > blueThreshold) {        //Must Tune Red Threshold
      elePos = Position.LEFT;
    } else if (averagedRightBox > blueThreshold) {
      elePos = Position.CENTER;
    } else {
      elePos = Position.RIGHT;
    }

    finalMat.copyTo(frame); /*This line should only be added in when you want to see your custom pipeline
                                  on the driver station stream, do not use this permanently in your code as
                                  you use the "frame" mat for all of your pipelines, such as April Tags*/
    return null;            //You do not return the original mat anymore, instead return null


  }

  @Override
  public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
      float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
    // Useless
  }

  public Position getElePos() {
    return this.elePos;
  }


}