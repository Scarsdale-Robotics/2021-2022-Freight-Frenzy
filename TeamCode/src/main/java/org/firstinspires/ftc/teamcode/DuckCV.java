package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class DuckCV {
    OpenCvInternalCamera phoneCam;
    private float duckX = -1;
    private float duckY = -1;

    public void beep(int cameraMonitorViewId) {
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.setPipeline(new CVPipeline());
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

    int getDuckPosition() {
        if (duckX == -1) return -1;
        if (duckX < 65) return 0;
        else if (duckX < 180) return 1;
        return 2;
    }

    class CVPipeline extends OpenCvPipeline {

        @Override
        public Mat processFrame(Mat frame) {
            Mat mask = new Mat();
            Imgproc.cvtColor(frame, mask, Imgproc.COLOR_BGR2HSV);

            Scalar lower_red = new Scalar(90, 50, 50);
            Scalar upper_red = new Scalar(110, 255, 255);

            Core.inRange(mask, lower_red, upper_red, mask);

            List<MatOfPoint> contours = new ArrayList<>();

            Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            Imgproc.cvtColor(mask, mask, Imgproc.COLOR_GRAY2BGR);

            Rect rect = new Rect();
            int size = 0;
            for (MatOfPoint c : contours) {
                Rect currentRect = Imgproc.boundingRect(c);
                if (currentRect.width * currentRect.height > size) {
                    rect = currentRect;
                    size = currentRect.height * currentRect.width;
                }
            }
            Imgproc.rectangle(frame, rect, new Scalar(0, 255, 0));

            duckY = rect.x + rect.width / 2;
            duckX = rect.y + rect.height / 2;

            return frame;
        }
    }
}