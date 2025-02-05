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
    public static int val = 0;
    public static int bhMax = 14, bhMin=0, bsMax=255, bsMin=100, blMax=255, blMin=50; // blue
    public static int rhMax = 120, rhMin=100, rsMax=255, rsMin=100, rlMax=255, rlMin=150; // red
    public static int yhMax = 105, yhMin=90, ysMax=255, ysMin=150, ylMax=255, ylMin=100; // red

    public static int sizeTresh = 200;
    public static int centerOffset = 140;
    public static double powerer = 1; // make more weighted at edge of frame to offset bad fov/framerate? could fix with better framerate.
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


    double maxArea = 0;
    int maxIdx = 0;
    public Mat isolate(Mat input) {
        Scalar lowB = new Scalar(bhMin, bsMin, blMin);
        Scalar highB = new Scalar(bhMax, bsMax, blMax);

        Scalar lowR = new Scalar(rhMin, rsMin, rlMin);
        Scalar highR = new Scalar(rhMax, rsMax, rlMax);

        Imgproc.cvtColor(input, temp, COLOR_BGR2HSV);
        //Imgproc.GaussianBlur(temp, temp, new Size(5.0, 5.0), 0.00);

        inRange(temp, lowR, highR, mask1);
        inRange(temp, lowB, highB, temp);
        add(mask1, temp, temp);

        maxArea = 0;
        maxIdx = 0;
        contours.clear();

        Imgproc.findContours(temp, contours, hierarchey, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        if (contours.isEmpty()) return val == 0 ? input : temp;

        Collections.sort(contours, (a, b) -> Double.compare(contourArea(b), contourArea(a)));

        maxArea = contourArea(contours.get(0));

        if (maxArea < sizeTresh) return val == 0 ? input : temp;

        Imgproc.drawContours(input, contours, maxIdx, new Scalar(0, 255, 0));

        RotatedRect wallpos = Imgproc.minAreaRect(new MatOfPoint2f(contours.get(maxIdx).toArray()));
        Imgproc.drawMarker(input, wallpos.center, new Scalar(0, 255, 255));
        setAngle(pow(abs(wallpos.center.x-centerOffset), powerer) * (abs(wallpos.center.x-centerOffset)/(wallpos.center.x-centerOffset)));
        return input;
    }
    @Override
    public Mat processFrame(Mat input) {
        //Drawing the Contours

        return isolate(input);

    }

}