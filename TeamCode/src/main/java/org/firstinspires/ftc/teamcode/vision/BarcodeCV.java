package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.teamcode.vision.pipelines.BarcodeCVPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

public class BarcodeCV {
    OpenCvInternalCamera phoneCam;
    BarcodeCVPipeline pipeline;

    public BarcodeCV(int cameraMonitorViewId) {
        pipeline = new BarcodeCVPipeline();

        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.setPipeline(pipeline);
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

    }

    private int readPosition() {
        int itemX = pipeline.itemX;
        int itemY = pipeline.itemY;

        if(itemX <= 0 || itemY <= 0 || Math.abs(130- itemY) > 30) return -1;

        if(itemX > 280) return 0;
        if(itemX > 180) return 1;
        return 2;
    }

    public int getXPosition() {
        return pipeline.itemX;
    }

    public int getYPosition() {
        return pipeline.itemY;
    }

    public int getBarcodePosition(){
        int[] votes = {0, 0, 0};
        long startTimer = System.currentTimeMillis();
        while ( (System.currentTimeMillis() - startTimer < 1000)) {
            int barcodePosition = readPosition();

            if (System.currentTimeMillis() - startTimer > 500 && barcodePosition != -1) {
                votes[barcodePosition]++;
            }
        }

        int bestPos = 2;
        for (int i = 0; i < votes.length; i++) {
            if (votes[i] >= votes[bestPos]) {
                bestPos = i;
            }
        }
        return bestPos;
    }

    public void close(){
        phoneCam.closeCameraDevice();
    }
}