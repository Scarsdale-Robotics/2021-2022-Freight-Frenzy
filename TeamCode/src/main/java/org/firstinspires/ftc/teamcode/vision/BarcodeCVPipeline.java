package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class BarcodeCVPipeline extends OpenCvPipeline {
    public int itemX = -1;
    public int itemY = -1;

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

        itemX = rect.x + rect.width / 2;
        itemY = rect.y + rect.height / 2;

        return mask;
    }
}
