package com.example.trobot;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import static org.opencv.android.CameraBridgeViewBase.*;

public class Vision {

    public static final int RED = 0;
    public static final int GREEN = 1;
    public static final int BLUE = 2;

    // Default HSV Values for RED, GREEN and BLUE
    private Scalar redHsvMin;
    private Scalar redHsvMax;
    private Scalar greenHsvMin;
    private Scalar greenHsvMax;
    private Scalar blueHsvMin;
    private Scalar blueHsvMax;

    public Mat mRgba;
    public Mat mRgbaF;
    public Mat mRgbaT;

    private List<VirtualColorSensor> activeSensors = new ArrayList<VirtualColorSensor>();
    private double colorFillPercentage = 0.0;
    private Point trackedColorPosition;

    // Initialize the first point for tracking line
    int[] PrevCenterLine = {160,200,160,200-50};

    public Mat getmRgba() {
        return mRgba;
    }

    public void setmRgba(Mat mRgba) {
        this.mRgba = mRgba;
    }

    public Mat getmRgbaF() {
        return mRgbaF;
    }

    public void setmRgbaF(Mat mRgbaF) {
        this.mRgbaF = mRgbaF;
    }

    public Mat getmRgbaT() {
        return mRgbaT;
    }

    public void setmRgbaT(Mat mRgbaT) {
        this.mRgbaT = mRgbaT;
    }

    public int[] getPrevCenterLine() {
        return PrevCenterLine;
    }

    public void setPrevCenterLine(int[] prevCenterLine) {
        PrevCenterLine = prevCenterLine;
    }

    public void initializeMRgba(int width, int height) {
        mRgba = new Mat(height, width, CvType.CV_8UC4);
        mRgbaF = new Mat(height, width, CvType.CV_8UC4);
        mRgbaT = new Mat(height, width, CvType.CV_8UC4);
    }

    public void initializePrevCenterLine(int width, int height) {
        PrevCenterLine[0] = width/2;
        PrevCenterLine[1] = height-1;
        PrevCenterLine[2] = width/2;
        PrevCenterLine[3] = height-50;
    }

    public void updatePrevCenterLineWithMRgba() {
        PrevCenterLine[0] = mRgba.cols()/2;
        PrevCenterLine[1] = mRgba.rows()-1;
        PrevCenterLine[2] = mRgba.cols()/2;
        PrevCenterLine[3] = mRgba.rows()-50;
    }

    public Mat preProcessFrame(CvCameraViewFrame inputFrame) {
        setmRgba(inputFrame.rgba());

        // Rotate mRgba 90 degrees
        Core.transpose(mRgba, mRgbaT);
        Imgproc.resize(mRgbaT, mRgbaF, mRgbaF.size(), 0, 0, 0);
        Core.flip(mRgbaF, mRgba, 1);

        return mRgba;
    }

    public Mat updateColorFillPercentage(Mat frame, int h_min, int s_min, int v_min, int h_max, int s_max, int v_max) {
        Mat detected = detectColoredObject(frame, h_min, s_min, v_min, h_max, s_max, v_max);
        long totalPixels = detected.total();
        int setPixelCount = Core.countNonZero(detected);

        this.colorFillPercentage = (((double) setPixelCount) / totalPixels) * 100;

        return detected;
    }

    public double getColorFillPercentage() {
        return colorFillPercentage;
    }

    public Mat updateSensorData(Mat frame, int h_min, int s_min, int v_min, int h_max, int s_max, int v_max, int numSensors) {
        Mat detected = detectColoredObject(frame, h_min, s_min, v_min, h_max, s_max, v_max);
        List<VirtualColorSensor> sensorData = getSensorData(detected, numSensors);
        activeSensors = getActiveSensors(sensorData);
        return detected;
    }

    public void drawSensorGrid(Mat frame) {
        if(frame.type() != CvType.CV_8UC3) {
            Imgproc.cvtColor(frame, frame, Imgproc.COLOR_GRAY2RGB);
        }
        Vision.drawGrid(frame);
        drawColoredFrameAroundActiveSensors(frame, activeSensors);
    }

    public static Mat detectColoredObject(Mat frame, int h_min, int s_min, int v_min, int h_max, int s_max, int v_max) {
        Mat thresholded = new Mat(frame.rows(), frame.cols(), CvType.CV_8UC3);
        Imgproc.cvtColor(frame, thresholded, Imgproc.COLOR_RGB2HSV);
        Core.inRange(thresholded, new Scalar(h_min, s_min, v_min), new Scalar(h_max, s_max, v_max), thresholded);

        //morphological opening (remove small objects from the foreground)
        Imgproc.erode(thresholded, thresholded, Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(5, 5)));
        Imgproc.dilate(thresholded, thresholded, Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(5, 5)));

        //morphological closing (fill small holes in the foreground)
        Imgproc.erode(thresholded, thresholded, Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(5, 5)));
        Imgproc.dilate(thresholded, thresholded, Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(5, 5)));

        return thresholded;
    }

    public void updatePositionOfTrackedColor(Mat frame) {
        Moments moments = Imgproc.moments(frame);

        double dM01 = moments.get_m01();
        double dM10 = moments.get_m10();
        double dArea = moments.get_m00();

        // if the area <= 10000, consider that the there are no objects in the image and it's because of the noise, the area is not zero
        if (dArea > 10000) {
            //calculate the position of the object
            double posX = dM10 / dArea;
            double posY = dM01 / dArea;

            trackedColorPosition = new Point(posX, posY);
        }
        else {
            trackedColorPosition = null;
        }
    }

    public Point getTrackedColorPosition() {
        return trackedColorPosition;
    }

    public List<VirtualColorSensor> getSensorData(Mat frame, int numDivisions) {
        if(Math.sqrt(numDivisions) % 1 != 0) {
            throw new IllegalArgumentException("Divide frame: Number of divisions must be a square number!");
        }

        int jump = (int) Math.sqrt(numDivisions);
        List<VirtualColorSensor> sensors = new ArrayList<VirtualColorSensor>();
        int sensorIndex = 0;

        for(int i = 0; i < jump; ++i) {
            int y0 = frame.rows() * i / jump;
            int y1 = frame.rows() * (i + 1) / jump;

            for(int j = 0; j < jump; ++j) {
                int x0 = frame.cols() * j / jump;
                int x1 = frame.cols() * (j + 1) / jump;
                Point p1 = new Point(x0, y0);
                Point p2 = new Point(x1, y1);

                Rect rect = new Rect(p1, p2);
                VirtualColorSensor virtualColorSensor = new VirtualColorSensor(p1, p2, sensorIndex, new Mat(frame, rect));
                sensors.add(virtualColorSensor);

                sensorIndex++;
            }
        }

        return sensors;
    }

    public List<VirtualColorSensor> getActiveSensors(List<VirtualColorSensor> sensors) {
        List<VirtualColorSensor> colorSensorsWithMoreColor = new ArrayList<VirtualColorSensor>();
        double maxFillRatio = 0.0;

        for(VirtualColorSensor sensor : sensors) {
            Mat data = sensor.sensorData;

            long totalPixelCount = data.total();
            int setPixelCount = Core.countNonZero(data);
            double fillRatio = ((double) setPixelCount) / totalPixelCount;

            if(fillRatio > maxFillRatio) {
                maxFillRatio = fillRatio;
                colorSensorsWithMoreColor.clear();
                colorSensorsWithMoreColor.add(sensor);
            }
            else if(fillRatio == maxFillRatio) {
                colorSensorsWithMoreColor.add(sensor);
            }
        }

        // If no sensor detected color, return an empty array
        if(maxFillRatio == 0.0) {
            return Collections.emptyList();
        }
        else {
            return colorSensorsWithMoreColor;
        }
    }

    public List<Integer> getActiveSensorIndices() {
        List<Integer> activeSensorIndices = new ArrayList<Integer>();
        for(VirtualColorSensor activeSensor : activeSensors) {
            activeSensorIndices.add(activeSensor.sensorIndex);
        }

        return activeSensorIndices;
    }

    public void drawColoredFrameAroundActiveSensors(Mat frame, List<VirtualColorSensor> activeSensors) {
        for(VirtualColorSensor sensor : activeSensors) {
            Core.rectangle(frame, sensor.p1, sensor.p2, new Scalar(255, 0, 0), 2);
        }
    }

    public void drawColorPosition(Mat frame) {
        if(frame.type() != CvType.CV_8UC3) {
            Imgproc.cvtColor(frame, frame, Imgproc.COLOR_GRAY2RGB);
        }

        if(trackedColorPosition != null) {
            Core.circle(frame, trackedColorPosition, 5, new Scalar(255, 0, 0), -1);
        }
    }

    public static void drawGrid(Mat frame) {
        int numOfLines = 2;

        for(int i = 1; i < numOfLines + 1; ++i) {
            //vertical lines
            Core.line(frame, new Point(frame.cols() * i / (numOfLines + 1), 0), new Point(frame.cols() * i / (numOfLines + 1), frame.rows()), new Scalar(255, 255, 255), 2);

            //horizontal lines
            Core.line(frame, new Point(0, frame.rows() * i / (numOfLines + 1)), new Point(frame.cols(), frame.rows() * i / (numOfLines + 1)), new Scalar(255, 255, 255), 2);
        }
    }

    public static Mat myInRange(Mat src, double h_min, double h_max, double s_min, double s_max, double v_min, double v_max) {
        if (h_min >= h_max && s_min <= s_max && v_min >= v_max)//0
        {
            Mat aux_1 = src, aux_2 = src;
            Core.inRange(src, new Scalar(h_min, s_min, v_min), new Scalar(255, s_max,255), aux_1);
            Core.inRange(src, new Scalar(0, s_min,0), new Scalar(h_max, s_max, v_max), aux_2);
            Core.bitwise_or(aux_1, aux_2, src);
        }
        else if (h_min >= h_max && s_min <= s_max && v_min <= v_max)//1
        {
            Mat aux_1 = src, aux_2 = src;
            Core.inRange(src, new Scalar(h_min, s_min, v_min), new Scalar(180, s_max, v_max), aux_1);
            Core.inRange(src, new Scalar(0, s_min, v_min), new Scalar(h_max, s_max, v_max), aux_2);
            Core.bitwise_or(aux_1, aux_2, src);
        }
        else if (h_min >= h_max && s_min >= s_max && v_min <= v_max)//2
        {
            Mat aux_1 = src, aux_2 = src;
            Core.inRange(src, new Scalar(h_min, s_min, v_min), new Scalar(180,255, v_max), aux_1);
            Core.inRange(src, new Scalar(0,0, v_min), new Scalar(h_max, s_max, v_max), aux_2);
            Core.bitwise_or(aux_1, aux_2, src);
        }
        else if (h_min >= h_max && s_min >= s_max && v_min >= v_max)//3
        {
            Mat aux_1 = src, aux_2 = src;
            Core.inRange(src, new Scalar(h_min, s_min, v_min), new Scalar(180,255,255), aux_1);
            Core.inRange(src, new Scalar(0,0,0), new Scalar(h_max, s_max, v_max), aux_2);
            Core.bitwise_or(aux_1, aux_2, src);
        }
        else if (h_min <= h_max && s_min <= s_max && v_min <= v_max)//4
        {
            Core.inRange(src, new Scalar(h_min, s_min, v_min), new Scalar(h_max, s_max, v_max), src);
        }
        else if (h_min <= h_max && s_min <= s_max && v_min >= v_max)//5
        {
            Mat aux_1 = src, aux_2 = src;
            Core.inRange(src, new Scalar(h_min, s_min, v_min), new Scalar(h_max, s_max,255), aux_1);
            Core.inRange(src, new Scalar(h_min, s_min,0), new Scalar(h_max, s_max, v_max), aux_2);
            Core.bitwise_or(aux_1, aux_2, src);
        }
        else if (h_min <= h_max && s_min >= s_max && v_min <= v_max)//6
        {
            Mat aux_1 = src, aux_2 = src;
            Core.inRange(src, new Scalar(h_min, s_min, v_min), new Scalar(h_max,255, v_max), aux_1);
            Core.inRange(src, new Scalar(h_min,0, v_min), new Scalar(h_max, s_max, v_max), aux_2);
            Core.bitwise_or(aux_1, aux_2, src);
        }
        else if (h_min <= h_max && s_min >= s_max && v_min >= v_max)//7
        {
            Mat aux_1 = src, aux_2 = src;
            Core.inRange(src, new Scalar(h_min, s_min, v_min), new Scalar(h_max,255,255), aux_1);
            Core.inRange(src, new Scalar(h_min,0,0), new Scalar(h_max, s_max, v_max), aux_2);
            Core.bitwise_or(aux_1, aux_2, src);
        }
        Imgproc.Canny(src, src, 50, 100);
        return src;
    }

    Mat processFrame(Mat frame, MainActivity activity) {
        int initial_row = frame.rows()-70;
        Mat ret = new Mat();
        Mat image = frame.submat(initial_row, frame.rows(), 0, frame.cols());
        Imgproc.cvtColor(frame, ret, Imgproc.COLOR_GRAY2BGR);
        Mat lines = new Mat();
        Imgproc.HoughLinesP(image, lines, 1, Math.PI/90, 15,20,15);

        double Dist		= 99e99;
        double bestDist = 99e99;

        int []PrevCenterLine1 = {0,0,0,0};
        for (int x = 0; x < lines.cols(); x++) {
            double[] vec = lines.get(0, x);
            double x1 = vec[0],
                    y1 = vec[1],
                    x2 = vec[2],
                    y2 = vec[3];
            Point start = new Point(x1, y1+initial_row);
            Point end = new Point(x2, y2+initial_row);
            Core.line(ret, start, end, new Scalar(255, 0, 0), 3);

            // Chose best line to track
            Dist = Math.min(  Math.sqrt(Math.pow(PrevCenterLine[0] - start.x, 2) + Math.pow(PrevCenterLine[1] - start.y, 2)),
                    Math.sqrt(Math.pow(PrevCenterLine[0] - end.x, 2) + Math.pow(PrevCenterLine[1] - end.y, 2)));

            if (Dist < bestDist && Dist < 40) {
                PrevCenterLine1[0] = (int)start.x;
                PrevCenterLine1[1] = (int)start.y;
                PrevCenterLine1[2] = (int)end.x;
                PrevCenterLine1[3] = (int)end.y;
            }
        }

        Point point = calculateIntersectionWithHorizontalLine(frame.rows(), frame.cols(), PrevCenterLine1);

        if (point.x > 0 && point.x < ret.cols()) {
            PrevCenterLine[0] = PrevCenterLine1[0];
            PrevCenterLine[1] = PrevCenterLine1[1];
            PrevCenterLine[2] = PrevCenterLine1[2];
            PrevCenterLine[3] = PrevCenterLine1[3];
        }

        return ret;
    }

    public Point calculateIntersectionWithHorizontalLine(int rows, int cols, int[] PrevCenterLine1) {
        /// CALCULATE INTERSECTION WITH HORIZONTAL LINE
        //Line 1 defined by x1,y1 and x2,y2
        float x_1 = 0;
        float y_1 = rows-25;
        float x_2 = cols;
        float y_2 = rows-25;

        //Line 2 defined by x3,y3 and x4,y4
        float x_3 = PrevCenterLine1[0];
        float y_3 = PrevCenterLine1[1];
        float x_4 = PrevCenterLine1[2];
        float y_4 = PrevCenterLine1[3];

        // Calculate the tracking point
        int xi = Math.round(((x_1 *y_2-y_1 *x_2)*(x_3-x_4)-(x_1-x_2)*(x_3 *y_4-y_3 *x_4))/((x_1-x_2)*(y_3-y_4)-(y_1-y_2)*(x_3-x_4)));
        int yi = Math.round(((x_1* y_2-y_1* x_2)*(y_3-y_4)-(y_1-y_2)*(x_3 *y_4-y_3 *x_4))/((x_1-x_2)*(y_3-y_4)-(y_1-y_2)*(x_3-x_4)));

        Point point = new Point(xi,yi);

        return point;
    }

    public static void drawRobotPosition(Mat img, Point point) {
        Core.circle(img, point, 6,new Scalar(255,255,0),2);
    }

    public void setHSVDefaults() {
        redHsvMin = new Scalar(0,207,50);
        redHsvMax = new Scalar(10,255,255);
        greenHsvMin = new Scalar(34,75,24);
        greenHsvMax = new Scalar(105,255,255);
        blueHsvMin = new Scalar(70,80,26);
        blueHsvMax = new Scalar(144,255,255);
    }

    public void setHSVMin(int hsvColor, Scalar min) {
        switch (hsvColor) {
            case RED:
                redHsvMin = min;
                break;
            case GREEN:
                greenHsvMin = min;
                break;
            case BLUE:
                blueHsvMin = min;
                break;
        }
    }

    public void setHSVMax(int hsvColor, Scalar max) {
        switch (hsvColor) {
            case RED:
                redHsvMax = max;
                break;
            case GREEN:
                greenHsvMax = max;
                break;
            case BLUE:
                blueHsvMax = max;
                break;
        }
    }

    public void setHSVMinMax(int hsvColor, Scalar min, Scalar max) {
        switch (hsvColor) {
            case RED:
                redHsvMin = min;
                redHsvMax = max;
                break;
            case GREEN:
                greenHsvMin = min;
                greenHsvMax = max;
                break;
            case BLUE:
                blueHsvMin = min;
                blueHsvMax = max;
                break;
        }
    }

    private class VirtualColorSensor {
        public Point p1;
        public Point p2;
        public int sensorIndex;
        public Mat sensorData;

        public VirtualColorSensor(Point p1, Point p2, int sensorIndex, Mat sensorData) {
            this.p1 = p1;
            this.p2 = p2;
            this.sensorIndex = sensorIndex;
            this.sensorData = sensorData;
        }
    }
}

