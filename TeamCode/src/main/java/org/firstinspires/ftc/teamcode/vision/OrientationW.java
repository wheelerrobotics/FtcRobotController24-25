package org.firstinspires.ftc.teamcode.vision;

import static java.lang.Math.round;

import android.provider.ContactsContract;

import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;


public class OrientationW extends OpenCvPipeline {
    private double rotation = 0;
    public double getRotation() {
        return rotation;
    }

    @Override
    public Mat processFrame(Mat input) {
        int x = 100;
        int y = 100;
        int width = 200;
        int height = 150;
        Rect roi = new Rect(x, y, width, height);

        // Crop the image
        Mat crop = input; // new Mat(input, roi);
        Mat tmp = crop;
        Imgproc.cvtColor(crop, tmp, Imgproc.COLOR_RGB2GRAY);
        Imgproc.blur(tmp, tmp, new Size(9,9));
        Imgproc.Canny(tmp, tmp, 9, 22);
        Mat lines = new Mat();
        Imgproc.HoughLines(tmp, lines, 0.53, 3.14/180, 41);

        List<Integer> thetaValues = new ArrayList<>();

        for (int i = 0; i < lines.rows(); i++) {
            double[] data = lines.get(i, 0);
            if (data != null) {
                double theta = data[1]; // Extract theta value
                thetaValues.add((int) round(theta * (12 / 3.14)) * 15);
            }
        }
        if (thetaValues.size() > 0) {
            rotation = findMode(thetaValues).get(0);
        }

        return input;
    }

    public static List<Integer> findMode(List<Integer> list) {
        if (list == null || list.isEmpty()) {
            return new ArrayList<>(); // Return empty list for null or empty input
        }

        Map<Integer, Integer> frequencyMap = new HashMap<>();
        for (int num : list) {
            frequencyMap.put(num, frequencyMap.getOrDefault(num, 0) + 1);
        }

        int maxFrequency = 0;
        for (int frequency : frequencyMap.values()) {
            maxFrequency = Math.max(maxFrequency, frequency);
        }

        List<Integer> modeList = new ArrayList<>();
        for (Map.Entry<Integer, Integer> entry : frequencyMap.entrySet()) {
            if (entry.getValue() == maxFrequency) {
                modeList.add(entry.getKey());
            }
        }

        return modeList;
    }
}