package org.firstinspires.ftc.teamcode.vision;

import android.os.Build;

import androidx.annotation.RequiresApi;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.vision.pipelines.AutoAlignPipeline;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

public class AutoAlignCV {
    OpenCvInternalCamera phoneCam;
    AutoAlignPipeline pipeline;

    public AutoAlignCV(int cameraMonitorViewId, boolean redAlliance) {
        if(redAlliance) {
            pipeline = new AutoAlignPipeline(new Scalar(110, 150, 0), new Scalar(140, 250, 255));
        } else {
            // replace with correct values
            pipeline = new AutoAlignPipeline(new Scalar(110, 150, 0), new Scalar(140, 250, 255));
        }

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

    public int getXPosition() {
        return pipeline.itemX;
    }

    public int getItemWidth() {
        return pipeline.itemWidth;
    }
}
