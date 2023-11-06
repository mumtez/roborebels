package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;

@Config
public class ColorPipeline extends OpenCvPipeline {


    public static Rect crop = new Rect(170,120,50, 70);

    @Override
    public Mat processFrame(Mat input) {
        Mat cropped = input.submat(crop);
        Mat output = new Mat();
        Imgproc.cvtColor(cropped, output, Imgproc.COLOR_RGBA2RGB);
        updateDet(output);
        return output;
    }

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
}