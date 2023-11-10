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
public class RedPropThreshold implements VisionProcessor {

  public static double redThreshold = 0.5;
  private final Rect LEFT_RECTANGLE = new Rect(
      new Point(0, 0),
      new Point(319, 479)
  );
  private final Rect RIGHT_RECTANGLE = new Rect(
      new Point(320, 0),
      new Point(639, 479)
  );
  Mat testMat = new Mat();
  Mat highMat = new Mat();
  Mat lowMat = new Mat();

  ;
  Mat finalMat = new Mat();
  private POSITION_OF_ELEMENT elePos = POSITION_OF_ELEMENT.NONE;

  @Override
  public void init(int width, int height, CameraCalibration calibration) {
    // Useless
  }

  @Override
  public Object processFrame(Mat frame, long captureTimeNanos) {
    Imgproc.cvtColor(frame, testMat, Imgproc.COLOR_RGB2HSV);

    Scalar lowHSVRedLower = new Scalar(0, 100, 20);  //Beginning of Color Wheel
    Scalar lowHSVRedUpper = new Scalar(10, 255, 255);

    Scalar redHSVRedLower = new Scalar(160, 100, 20); //Wraps around Color Wheel
    Scalar highHSVRedUpper = new Scalar(180, 255, 255);

    Core.inRange(testMat, lowHSVRedLower, lowHSVRedUpper, lowMat);
    Core.inRange(testMat, redHSVRedLower, highHSVRedUpper, highMat);

    testMat.release();

    Core.bitwise_or(lowMat, highMat, finalMat);

    lowMat.release();
    highMat.release();

    double leftBox = Core.sumElems(finalMat.submat(LEFT_RECTANGLE)).val[0];
    double rightBox = Core.sumElems(finalMat.submat(RIGHT_RECTANGLE)).val[0];

    double averagedLeftBox = leftBox / LEFT_RECTANGLE.area() / 255;
    double averagedRightBox = rightBox / RIGHT_RECTANGLE.area() / 255; //Makes value [0,1]

    if (averagedLeftBox > redThreshold) {        //Must Tune Red Threshold
      elePos = POSITION_OF_ELEMENT.LEFT;
    } else if (averagedRightBox > redThreshold) {
      elePos = POSITION_OF_ELEMENT.CENTER;
    } else {
      elePos = POSITION_OF_ELEMENT.RIGHT;
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

  public POSITION_OF_ELEMENT getElePos() {
    return this.elePos;
  }

  enum POSITION_OF_ELEMENT {LEFT, RIGHT, CENTER, NONE}
}