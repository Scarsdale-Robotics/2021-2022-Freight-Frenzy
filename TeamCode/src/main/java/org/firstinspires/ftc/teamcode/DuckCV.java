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

    public DuckCV(int cameraMonitorViewId) {
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
        if (duckY == -1) return -1;
        if (duckY < 80) return 0;
        else if (duckY < 200) return 1;
        return 2;
    }

    class CVPipeline extends OpenCvPipeline {
        final Scalar lower_green = new Scalar(30, 175, 75);
        final Scalar upper_green = new Scalar(100, 255, 200);

        final Scalar rectangle_color = new Scalar(0, 255, 0);

        Mat mask = new Mat();
        Mat trashMat = new Mat();

        Rect rect = new Rect();

        @Override
        public Mat processFrame(Mat frame) {
            Imgproc.cvtColor(frame, mask, Imgproc.COLOR_BGR2HSV);

            Core.inRange(mask, lower_green, upper_green, mask);

            List<MatOfPoint> contours = new ArrayList<>();

            Imgproc.findContours(mask, contours, trashMat, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            Imgproc.cvtColor(mask, mask, Imgproc.COLOR_GRAY2BGR);

            int size = 0;
            for (MatOfPoint c : contours) {
                Rect currentRect = Imgproc.boundingRect(c);
                if (currentRect.width * currentRect.height > size) {
                    rect = currentRect;
                    size = currentRect.height * currentRect.width;
                }

            }
            Imgproc.rectangle(mask, rect, rectangle_color);

            duckY = rect.x + rect.width / 2;
            duckX = rect.y + rect.height / 2;

            return mask;
        }
    }
}