package org.firstinspires.ftc.teamcode.vision;

import static java.lang.Math.PI;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

import org.ejml.simple.SimpleMatrix;
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
import java.util.List;
import java.util.Vector;

public class SampleOrientationPipeline extends OpenCvPipeline {
    double targetX = 0; // TODO: GET REAL VALUE
    double targetY = 0; // TODO: GET REAL VALUE
    double swivelPos = 0; // TODO: MAKE REAL
    double swivelCamRadius = 3; // TODO: MAKE REAL
    double swivelClawRadius = 3; // TODO: MAKE REAL
    public double[] estimateSampleOrientation(Mat input) {

        // canny edge detection then largest contour pretty much
        Mat gray = new Mat();
        Imgproc.cvtColor(input, gray, Imgproc.COLOR_BGR2GRAY);
        Imgproc.GaussianBlur(gray, gray, new Size(5, 5), 0);
        Mat edges = new Mat();
        Imgproc.Canny(gray, edges, 50, 150);
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        double angle = 0.0;
        Point center = new Point();
        double forwardError = 0;
        double lateralError = 0;

        for (MatOfPoint contour : contours) {

            RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));
            angle = rect.angle;
            center = rect.center;
            if (rect.size.width < rect.size.height) angle += 90;
            angle += swivelPositionToAngleOffset(swivelPos);


            // draw found rectangle
            Point[] boxPoints = new Point[4];
            rect.points(boxPoints);
            for (int i = 0; i < 4; i++) Imgproc.line(input, boxPoints[i], boxPoints[(i + 1) % 4], new Scalar(0, 255, 0), 2);


            // this code just takes the camera frame
            // position of the sample and linear
            // algebras it into an absolute position
            // relative to the claw position

            // idk how much its needed cause camera and claw both rotate and I didnt account for claw rotation ( >:( )
            // but itll be really useful when we get a non swivelly claw maybe?

            // also there probably a bajillion sign errors here, ill fix it when I can actually run the code.
            SimpleMatrix cent = new SimpleMatrix(new double[]{center.x+(swivelClawRadius * cos(angle)), center.y+(swivelClawRadius * sin(angle))});
            SimpleMatrix translation = new SimpleMatrix(new double[]{swivelCamRadius, 0});
            SimpleMatrix target = new SimpleMatrix(new double[]{targetX, targetY});
            SimpleMatrix rotation = new SimpleMatrix(new double[][]{{cos(angle), -sin(angle)},{sin(angle), cos(angle)}});
            SimpleMatrix error = target.minus(cent.minus(translation).mult(rotation));

            // ASSUMING FORWARD IS X AND LATERAL IS Y, FLIP IF NO
            // (forward, lateral)
            forwardError = error.get(1, 1);
            lateralError = error.get(2, 1);

            break;
        }

        // NOTE: ANGLE IS ABSOLUTE, ERRORS ARE RELATIVE TO CENTERPOINT
        // angle: should feed into swivelAngle: setSwivel(angleToSwivelPosition(angle))
        // forwardError: should feed into extendoPos: incrementExtendo(kP*forwardError)
        // forwardError: should feed into strafing control: motorDriveXYVectors("", ""+kP*lateralError, "")
        return new double[]{angle, forwardError, lateralError};
    }
    public static double swivelScaler = 1/(17*PI/16);//RANGE OF MOTION RADIANS // TODO: MAKE REAL NUMBER
    public static double swivelPositionToAngleOffset(double swivelPos) {
        return (swivelPos/swivelScaler);
    }
    public static double angleToSwivelPosition(double angle) {
        return (swivelScaler * angle);
    }
    public double[] getOrientation() {
        return orientation;
    }
    private double[] orientation = null;
    @Override
    public Mat processFrame(Mat input) {
        orientation = estimateSampleOrientation(input);
        return input;
    }
}
