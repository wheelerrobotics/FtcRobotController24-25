package org.firstinspires.ftc.teamcode.vision;

import static org.opencv.core.Core.add;
import static org.opencv.core.Core.inRange;
import static org.opencv.core.Core.max;
import static org.opencv.imgproc.Imgproc.COLOR_BGR2HSV;
import static org.opencv.imgproc.Imgproc.contourArea;
import static org.opencv.imgproc.Imgproc.drawContours;

import static java.lang.Math.abs;
import static java.lang.Math.pow;

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
    public static int hMax = 20, hMin=0, sMax=255, sMin=100, lMax=100, lMin=10; // blue
    public static int hMax2 = 135, hMin2=120, sMax2=225, sMin2=100, lMax2=100, lMin2=10; // red
    public static int sizeTresh = 200;
    public static double powerer = 1.05; // make more weighted at edge of frame to offset bad fov/framerate? could fix with better framerate.
    public double angle = 0;

    public double getAngle() {
        return angle;
    }
    public void setAngle(double angle) {
        this.angle = angle;
    }

    public ColorIsolationPipeline() {

    }

    @Override
    public void init(Mat input) {
    }
    Mat hierarchey = new Mat();
    List<MatOfPoint> contours = new ArrayList<>();
    Mat mask1 = new Mat();
    Mat temp = new Mat();

    Scalar lowR = new Scalar(hMin, sMin, lMin);
    Scalar highR = new Scalar(hMax, sMax, lMax);

    Scalar lowB = new Scalar(hMin2, sMin2, lMin2);
    Scalar highB = new Scalar(hMax2, sMax2, lMax2);

    double maxArea = 0;
    int maxIdx = 0;
    public Mat isolate(Mat input) {
        Imgproc.cvtColor(input, temp, COLOR_BGR2HSV);
        Imgproc.GaussianBlur(temp, temp, new Size(5.0, 5.0), 0.00);

        inRange(temp, lowR, highR, mask1);
        inRange(temp, lowB, highB, temp);
        add(mask1, temp, temp);

        maxArea = 0;
        maxIdx = 0;

        Imgproc.findContours(temp, contours, hierarchey, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        if (contours.isEmpty()) return input;

        for (MatOfPoint contour : contours) {
            double area = contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                maxIdx = contours.indexOf(contour);
            }
        }

        if (maxArea < sizeTresh) return input;

        Imgproc.drawContours(input, contours, maxIdx, new Scalar(0, 255, 0));

        RotatedRect wallpos = Imgproc.minAreaRect(new MatOfPoint2f(contours.get(maxIdx).toArray()));
        Imgproc.drawMarker(input, wallpos.center, new Scalar(0, 255, 255));
        setAngle(pow(abs(wallpos.center.x-60), powerer) * (abs(wallpos.center.x-60)/(wallpos.center.x-60)));
        return input;
    }
    @Override
    public Mat processFrame(Mat input) {
        //Drawing the Contours

        return isolate(input);

    }

}