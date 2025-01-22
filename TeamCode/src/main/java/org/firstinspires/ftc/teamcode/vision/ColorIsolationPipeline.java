package org.firstinspires.ftc.teamcode.vision;

import static org.opencv.core.Core.add;
import static org.opencv.core.Core.inRange;
import static org.opencv.core.Core.max;
import static org.opencv.imgproc.Imgproc.COLOR_BGR2HSV;
import static org.opencv.imgproc.Imgproc.contourArea;
import static org.opencv.imgproc.Imgproc.drawContours;

import android.provider.ContactsContract;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Config
public class ColorIsolationPipeline extends OpenCvPipeline {
    /*static public int hMax = 12;
    static public int sMax = 300;
    static public int lMax = 400;

    static public int hMin = 0;
    static public int sMin = 120;
    static public int lMin = 0;
    */

    public static boolean outputMode, blurry;
    public static int hMax = 10, hMin=0, sMax=255, sMin=50, lMax=255, lMin=50;
    public static int hMax2 = 130, hMin2=115, sMax2=225, sMin2=150, lMax2=255, lMin2=50;
    public int pos = 0;
    public double angle = 0;

    public double getAngle() {
        return angle;
    }

    public ColorIsolationPipeline() {

    }

    @Override
    public void init(Mat input) {
    }
    public Mat isolate(Mat input) {
        Mat temp = new Mat();
        Imgproc.cvtColor(input, temp, COLOR_BGR2HSV);

        Scalar lowR = new Scalar(hMin, sMin, lMin);
        Scalar highR = new Scalar(hMax, sMax, lMax);

        Scalar lowB = new Scalar(hMin2, sMin2, lMin2);
        Scalar highB = new Scalar(hMax2, sMax2, lMax2);

        Mat mask1 = new Mat();
        inRange(temp, lowR, highR, mask1);

        Mat mask2 = new Mat();
        inRange(temp, lowB, highB, mask2);


        Mat mask = new Mat();
        add(mask1, mask2, mask);

        List<MatOfPoint> contours = new ArrayList<>();
        double maxArea = 0;
        int maxIdx = 0;
        Mat hierarchey = new Mat();

        Imgproc.findContours(mask, contours, hierarchey, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        if (contours.isEmpty()) return input;

        for (MatOfPoint contour : contours) {
            double area = contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                maxIdx = contours.indexOf(contour);
            }
        }
        Imgproc.drawContours(input, contours, maxIdx, new Scalar(0, 255, 0));

        RotatedRect wallpos = Imgproc.minAreaRect(new MatOfPoint2f(contours.get(maxIdx).toArray()));
        Imgproc.drawMarker(input, wallpos.center, new Scalar(0, 255, 255));
        angle = wallpos.angle;
        return input;
    }
    public Mat ROI, blur;
    @Override
    public Mat processFrame(Mat input) {
        //Drawing the Contours

        return isolate(input);

    }

}