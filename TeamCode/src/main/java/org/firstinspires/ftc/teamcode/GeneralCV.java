package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;
import org.openftc.easyopencv.OpenCvInternalCamera2Impl;
import org.openftc.easyopencv.OpenCvPipeline;
//
//import java.util.ArrayList;
//import java.util.List;
//
//public class GeneralCV {
//
//    OpenCvCamera phoneCam;
//
//    public int x = -1;
//    public int y = -1;
//    public int sizeH = 0;
//    public int sizeW = 0;
//    public int minX = 0;
//    public int minY = 0;
//    public int maxX = 99999;
//    public int maxY = 99999;
//    Scalar lower_1 = new Scalar(0, 0, 0);
//    Scalar upper_1 = new Scalar(255, 255, 255);
//
//    Scalar lower_2;
//    Scalar upper_2;
//    public void editLoc(int minXfun, int minYfun, int maxXfun, int maxYfun){
//        minX = minXfun;
//        minY = minYfun;
//        maxX = maxXfun;
//        maxY = maxYfun;
//    }
//    public void editScale(int W, int H){
//        sizeH = H;
//        sizeW = W;
//    }
//    public void editColor(Scalar l1, Scalar u1){
//
//        lower_1 = l1;
//        lower_2 = new Scalar(0, 0, 0);
//        upper_1 = u1;
//        upper_2 = new Scalar(255, 255, 255);
//    }
//
//    public void beep(int bob)
//    {
//
//        phoneCam = new OpenCvInternalCamera2Impl(OpenCvInternalCamera2.CameraDirection.BACK, bob);
//
//        phoneCam.openCameraDevice();
//
//        phoneCam.setPipeline(new SamplePipeline());
//
//        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//    }
//
//    public int[] getXY() {
//        return new int[]{x, y};
//    }
//
//    class SamplePipeline extends OpenCvPipeline
//    {
//        boolean displayCont = false;
//        List<Integer> black;
//
//        int counter = 0;
//        int counter2 = 0;
//        int size = 5;
//
//        Mat mask = new Mat();
//        Mat points = new Mat();
//
//        Rect rect = new Rect();
//        Rect yellowRect = new Rect();
//
//        List<MatOfPoint> contours = new ArrayList<>();
//
//        @Override
//        public Mat processFrame(Mat frame)
//        {
//            counter = 0;
//            counter2 = 0;
//
//            black = new ArrayList<>();
//
//            contours = new ArrayList<>();
//
//            Imgproc.cvtColor(frame, points, Imgproc.COLOR_BGR2HSV);
//
//            Core.inRange(points, lower_2, upper_2, mask);
//
//            Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
//
//            ArrayList<MatOfPoint> yellowContours = new ArrayList<MatOfPoint>();
//
//            for (MatOfPoint c : contours) {
//                ArrayList<MatOfPoint> cList = new ArrayList<MatOfPoint>();
//                cList.add(c);
//
//                rect = Imgproc.boundingRect(c);
//
//                counter2 = counter;
//                counter = 0;
//                displayCont = false;
//
//                if(rect.y + rect.height/2 > 100 && rect.width >= size && rect.height >= size && rect.width * rect.height > 2000) {
//                    yellowContours.add(c);
//                }
//
//            }
//
//            contours = new ArrayList<>();
//
//            Core.inRange(points, lower_1, upper_1, mask);
//
//            Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
//
//            Imgproc.cvtColor(mask, mask, Imgproc.COLOR_GRAY2BGR);
//
//            Imgproc.drawContours(mask, yellowContours, 0, new Scalar(255, 0, 0));
//
//            for (MatOfPoint c : contours) {
//
//                ArrayList<MatOfPoint> cList = new ArrayList<MatOfPoint>();
//                cList.add(c);
//
//                boolean broke = false;
//
//                rect = Imgproc.boundingRect(c);
//                //YES, IT IS INTENTIONAL THAT THESE ARE FLIPPED
//                if(rect.width >= sizeH && rect.height >= sizeW && rect.x > minY && rect.x < maxY && rect.y < maxX && rect.y > maxY) {
//                    for (MatOfPoint yC : yellowContours) {
//                        yellowRect = Imgproc.boundingRect(yC);
//
//                        x = rect.y + (rect.height/2);
//                        y = rect.x + (rect.width/2);
//                        //thhe x and y are swapped
//                        Imgproc.drawContours(mask, cList, 0, new Scalar(0, 0, 255));
//
//                        broke = true;
//                        break;
//                    }
//                }
//
//                if (!broke) {
//                    //give me money
//                    Imgproc.drawContours(mask, cList, 0, new Scalar(0, 255, 0));
//                }
//            }
//
//            return mask;
//        }
//    }
//}