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
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;

@Config
public class CopyColorPipeline implements VisionProcessor {

    //public static Rect crop = new Rect(170,120,50, 70);
    public int location = -1;

    Mat testMat = new Mat();
    Mat highMat = new Mat();
    Mat lowMat = new Mat();
    Mat finalMat = new Mat();
    double redThreshold = 0.5;

    static final Rect LEFT_RECTANGLE = new Rect(
            new Point(0, 0),
            new Point(0, 0)
    );

    static final Rect RIGHT_RECTANGLE = new Rect(
            new Point(0, 0),
            new Point(0, 0)
    );


    public Scalar lower = new Scalar(0, 0, 0); // HSV threshold bounds
    public Scalar upper = new Scalar(255, 255, 255);

    private Mat hsvMat = new Mat(); // converted image
    private Mat binaryMat = new Mat(); // imamge analyzed after thresholding
    private Mat maskedInputMat = new Mat();


    public CopyColorPipeline(){

    }

    /*
        @Override
        public Mat processFrame(Mat input) {

        }
    */
    public void updateDet(Mat image){
        ArrayList<Double> sum = new ArrayList<>();
        sum.add(0.0);
        sum.add(0.0);
        sum.add(0.0);


        for (int row = 0; row <= image.rows() - 1; row++){ // todo check if this fix works
            for (int col = 0; col <= image.cols() - 1; col++){
                sum.set(0, sum.get(0) + image.get(row, col)[0]);
                sum.set(1, sum.get(1) + image.get(row, col)[1]);
                sum.set(2, sum.get(2) + image.get(row, col)[2]);
            }
        }

        double max = Collections.max(sum); // todo check this
    }

    public String getOutput() {
        return null;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }
        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
        /*
            Mat output = new Mat();
            Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);
            Core.inRange(hsvMat, lower, upper, binaryMat);

            double w1 = 0, w2 = 0, w3 = 0;
            // process the pixel value for each rectangle  (255 = W, 0 = B)
            for (int i = (int) topLeft1.x; i <= bottomRight1.x; i++) {
                for (int j = (int) topLeft1.y; j <= bottomRight1.y; j++) {
                    w1 += binaryMat.get(i, j)[0];
                }
            }

            for (int i = (int) topLeft2.x; i <= bottomRight2.x; i++) {
                for (int j = (int) topLeft2.y; j <= bottomRight2.y; j++) {
                    w2 += binaryMat.get(i, j)[0];
                }
            }

            for (int i = (int) topLeft3.x; i <= bottomRight3.x; i++) {
                for (int j = (int) topLeft3.y; j <= bottomRight3.y; j++) {
                    w3 += binaryMat.get(i, j)[0];
                }
            }

            if (w1 >= w2 && w1 >= w3) {location = 1;}

            if (w2 >= w1 && w2 >= w3) {location = 2;}

            if (w3 >= w1 && w3 >= w2) {location = 3;}


            //updateDet(output);
            return output;
            */
            return null;
        }
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}