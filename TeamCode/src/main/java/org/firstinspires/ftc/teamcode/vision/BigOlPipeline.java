package org.firstinspires.ftc.teamcode.vision;

import static org.opencv.core.Core.add;
import static org.opencv.core.Core.bitwise_and;
import static org.opencv.core.Core.inRange;
import static org.opencv.core.Core.multiply;
import static org.opencv.core.Core.sort;
import static org.opencv.imgproc.Imgproc.COLOR_BGR2GRAY;
import static org.opencv.imgproc.Imgproc.COLOR_BGR2HSV;
import static org.opencv.imgproc.Imgproc.COLOR_GRAY2BGR;
import static org.opencv.imgproc.Imgproc.COLOR_GRAY2BGRA;
import static org.opencv.imgproc.Imgproc.COLOR_HSV2BGR;
import static org.opencv.imgproc.Imgproc.contourArea;
import static org.opencv.imgproc.Imgproc.cvtColor;
import static org.opencv.imgproc.Imgproc.cvtColorTwoPlane;
import static java.lang.Math.abs;
import static java.lang.Math.pow;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

@Config
public class BigOlPipeline extends OpenCvPipeline {
    public static int bhMax = 20, bhMin=0, bsMax=255, bsMin=100, blMax=100, blMin=10; // blue
    public static int rhMax = 135, rhMin=120, rsMax=225, rsMin=100, rlMax=100, rlMin=10; // red
    public static int yhMax = 105, yhMin=90, ysMax=255, ysMin=150, ylMax=255, ylMin=100; // red

    public static int sizeTresh = 200;
    public static double powerer = 1.05; // make more weighted at edge of frame to offset bad fov/framerate? could fix with better framerate.
    public double angle = 0;
    public static double t1 = 190;
    public static double t2 = 190;
    public static int ap = 50;

    public double getAngle() {
        return angle;
    }
    public void setAngle(double angle) {
        this.angle = angle;
    }

    public BigOlPipeline() {

    }

    @Override
    public void init(Mat input) {
    }
    Mat hierarchey = new Mat();
    List<MatOfPoint> contours = new ArrayList<>();
    Mat mask1 = new Mat();
    Mat temp = new Mat();
    Mat canert = new Mat();


    double maxArea = 0;
    int maxIdx = 0;
    int val = 0;
    public Mat isolate(Mat input) {

        Scalar lowR = new Scalar(rhMin, rsMin, rlMin);
        Scalar highR = new Scalar(rhMax, rsMax, rlMax);

        Scalar lowB = new Scalar(bhMin, bsMin, blMin);
        Scalar highB = new Scalar(bhMax, bsMax, blMax);

        Scalar lowY = new Scalar(yhMin, ysMin, ylMin);
        Scalar highY = new Scalar(yhMax, ysMax, ylMax);
        Imgproc.cvtColor(input, temp, COLOR_BGR2HSV);
        Imgproc.cvtColor(input, canert, COLOR_BGR2GRAY);

        Imgproc.GaussianBlur(temp, temp, new Size(5.0, 5.0), 0.00);
        inRange(temp, lowY, highY, temp);


        Imgproc.Canny(canert, canert, t1, t2);
        Imgproc.GaussianBlur(canert, canert, new Size(5.0, 5.0), 0.00);

        //Core.inRange(canert, new Scalar(0), new Scalar(1), canert);

        //cvtColor(temp, temp, COLOR_HSV2BGR);
        //cvtColor(temp, temp, COLOR_BGR2GRAY);

        Mat mask = new Mat();
        //inRange(canert, new Scalar(1), new Scalar(255), canert);
        Imgproc.threshold(canert, canert, 250, 255, Imgproc.THRESH_BINARY_INV);
        Imgproc.threshold(temp, mask, 0, 255, Imgproc.THRESH_BINARY_INV);

        // Set corresponding pixels in m to black (0,0,0)
        //input.setTo(new Scalar(0, 0, 0), canert);
        //input.setTo(new Scalar(0, 0, 0), mask);
        temp.setTo(new Scalar(0,0,0), canert);


        //Core.bitwise_and(canert, temp, temp);
        //if(true) return input;



        //inRange(temp, lowR, highR, mask1);
        //inRange(temp, lowY, highY, temp);
        //add(mask1, temp, temp);
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
        Imgproc.line(input, wallpos.center, wallpos.boundingRect().tl(), new Scalar(255, 0, 0));
        Imgproc.line(input, wallpos.center, wallpos.boundingRect().br(), new Scalar(0, 255, 0));
        FtcDashboard.getInstance().getTelemetry().addData("sidetoside", wallpos.center.x);
        FtcDashboard.getInstance().getTelemetry().addData("fb", wallpos.center.y);
        FtcDashboard.getInstance().getTelemetry().addData("ang", wallpos.angle);

        return input;
    }
    @Override
    public Mat processFrame(Mat input) {
        //Drawing the Contours

        return isolate(input);

    }

}