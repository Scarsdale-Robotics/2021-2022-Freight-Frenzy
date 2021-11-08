package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import org.opencv.core.MatOfPoint;

import java.util.ArrayList;
import java.util.List;

public class duckCV {
    OpenCvInternalCamera phoneCam;
    private float duckX = -1;
    private float duckY = -1;

    public duckCV(int cameraMonitorViewId) {
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
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
        List<MatOfPoint> contours = new ArrayList<>();
        Rect rect = new Rect();

        // Everything unneeded goes in here. Rather that passing in a new mat each time we'll just allocate it once
        Mat trashMat = new Mat();

        @Override
        public Mat processFrame(Mat frame) {
            Mat mask = new Mat();
            Imgproc.cvtColor(frame, mask, Imgproc.COLOR_BGR2HSV);


            Scalar lower_red = new Scalar(90, 210, 180);

            Scalar upper_red = new Scalar(110, 255, 225);

            Core.inRange(mask, lower_red, upper_red, mask);

            contours = new ArrayList<>();

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
            Imgproc.rectangle(mask, rect, new Scalar(0, 255, 0));

            //Woman whore1 = new Woman();
            //telemetry.addData("Bitches": whore1)
            //womn be lke stop objectingy me

            //cv is dumb so xy are flipped becaue reaspons i wanna die
            duckY = rect.x + rect.width / 2;
            duckX = rect.y + rect.height / 2;

            return mask;
        }
    }
}