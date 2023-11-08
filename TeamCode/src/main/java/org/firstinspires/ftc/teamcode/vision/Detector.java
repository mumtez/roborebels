package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;

public class Detector {
    public final OpenCvWebcam PHONE_CAM;
    public final ColorPipeline PIPELINE;

    public Detector(OpMode opMode){
        int cameraMonitorViewId = opMode.hardwareMap
                .appContext
                .getResources()
                .getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());

        PHONE_CAM = OpenCvCameraFactory.getInstance()
                .createWebcam(opMode.hardwareMap.get(WebcamName.class, "camera"), cameraMonitorViewId);


        PIPELINE = new ColorPipeline();
        //PHONE_CAM.setPipeline(PIPELINE);

        PHONE_CAM.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                PHONE_CAM.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                opMode.telemetry.addLine("FAILED TO GET CAMERA");
                opMode.telemetry.update();
            }
        });
    }

    public String getPipelineOutput(){
        return PIPELINE.getOutput();
    }
    public void stopStream(){
        PHONE_CAM.pauseViewport();
        PHONE_CAM.stopStreaming();
    }
}