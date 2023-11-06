package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;

public class Detector {
    public final OpenCvCamera PHONE_CAM;
    public final ColorPipeline PIPELINE;

    public Detector(OpMode opMode){
        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        PHONE_CAM = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId);

        PIPELINE = new ColorPipeline();
        PHONE_CAM.setPipeline(PIPELINE);

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
    public void stopStream(){
        PHONE_CAM.pauseViewport();
        PHONE_CAM.stopStreaming();
    }
}