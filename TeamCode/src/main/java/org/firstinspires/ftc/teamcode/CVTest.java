package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

@TeleOp
public class CVTest extends LinearOpMode {
    OpenCvInternalCamera phoneCam;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
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

        waitForStart();

        while (opModeIsActive()) {
            sleep(50);
        }
    }


    class CVPipeline extends OpenCvPipeline {
        List<MatOfPoint> contours = new ArrayList<>();
        Rect rect = new Rect();
        @Override
        public Mat processFrame(Mat frame) {
            Mat mask = new Mat();
            Imgproc.cvtColor(frame, mask, Imgproc.COLOR_BGR2HSV);


            Scalar lower_red = new Scalar(120, 50, 50);

            Scalar upper_red = new Scalar(130, 255, 255);


            Core.inRange(mask, lower_red, upper_red, mask);

            contours = new ArrayList<>();


            Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            Imgproc.cvtColor(mask, mask, Imgproc.COLOR_GRAY2BGR);

            int size = 0;
            for (MatOfPoint c : contours) {
                ArrayList<MatOfPoint> cList = new ArrayList<MatOfPoint>();
                cList.add(c);
                Rect currentRect = Imgproc.boundingRect(c);
                if(currentRect.width * currentRect.height > size){
                    rect = currentRect;
                    size = currentRect.height * currentRect.width;
                }

            }
            Imgproc.rectangle(frame, rect, new Scalar(0, 255, 0));

//            Imgproc.drawContours(mask, contours, -1, new Scalar(255, 0, 0));




            return  frame;
        }
    }
}