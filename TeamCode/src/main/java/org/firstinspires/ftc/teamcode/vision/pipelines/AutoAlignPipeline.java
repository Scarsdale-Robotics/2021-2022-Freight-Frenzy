package org.firstinspires.ftc.teamcode.vision.pipelines;

import android.os.Build;
import androidx.annotation.RequiresApi;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.function.Function;

@RequiresApi(api = Build.VERSION_CODES.N)
public class AutoAlignPipeline extends OpenCvPipeline {

    final Scalar upper_red = new Scalar(40, 100, 100);
    final Scalar lower_red = new Scalar(0, 0, 0);

    public int itemX = -1;
    public int itemY = -1;

    final Scalar rectangle_color = new Scalar(0, 255, 0);

    Mat mask = new Mat();
    Mat trashMat = new Mat();

    Rect rect = new Rect();

    Function<MatOfPoint, Rect> toRect = Imgproc::boundingRect;
    Comparator<Rect> byArea = Comparator.comparing(Rect::area);

    List<MatOfPoint> contours = new ArrayList<>();

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mask, Imgproc.COLOR_BGR2HSV);

        Core.inRange(mask, lower_red, upper_red, mask);

        Imgproc.findContours(mask, contours, trashMat, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        Imgproc.cvtColor(mask, mask, Imgproc.COLOR_GRAY2BGR);

        rect = contours.stream().map(toRect).max(byArea).orElse(rect);

        Imgproc.rectangle(mask, rect, rectangle_color);

        itemX = rect.x + rect.width / 2;
        itemY = rect.y + rect.height / 2;

        return mask;
    }
}
