package org.firstinspires.ftc.teamcode.vision.pipelines;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class AutoAlignPipeline extends OpenCvPipeline {

    final Scalar upper_red = new Scalar(140, 255, 255);
    final Scalar lower_red = new Scalar(100, 150, 60);

    public int itemX = -1;
    public int itemY = -1;

    final Scalar rectangle_color = new Scalar(0, 255, 0);

    Mat mask = new Mat();
    Mat trashMat = new Mat();

    Rect rect = new Rect();

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mask, Imgproc.COLOR_BGR2HSV);

        Core.inRange(mask, lower_red, upper_red, mask);

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, trashMat, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        int size = 0;
        for (MatOfPoint c : contours) {
            Rect currentRect = Imgproc.boundingRect(c);
            if (currentRect.width * currentRect.height > size) {
                rect = currentRect;
                size = currentRect.height * currentRect.width;
            }
        }
        itemX = rect.x + rect.width / 2;
        itemY = rect.y + rect.height / 2;

        input.copyTo(mask, mask);

        Imgproc.rectangle(mask, rect, rectangle_color);

        return mask;
    }
}