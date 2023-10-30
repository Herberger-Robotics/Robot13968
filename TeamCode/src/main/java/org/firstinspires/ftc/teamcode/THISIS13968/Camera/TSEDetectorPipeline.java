package org.firstinspires.ftc.teamcode.THISIS13968.Camera;//package org.firstinspires.ftc.teamcode.THISIS10111.Camera;

import org.firstinspires.ftc.teamcode.THISIS13968.hardwaremaps.Robot13968;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class TSEDetectorPipeline extends OpenCvPipeline {
    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar GREEN = new Scalar(0, 255, 0);
    static final Scalar RED_LOW =new Scalar(150, 100, 100);
    static final Scalar RED_HIGH = new Scalar(180, 255, 255);



    // values are for blue
    // not consistent at all.
    //Scalar lower = new Scalar(97,100,100);
    //Scalar upper = new Scalar(125,255,255);

    // values are for red
    // very consistent
    //Scalar lower = new Scalar(150, 100, 100);
    //Scalar upper = new Scalar(180, 255, 255);

    private Colors colorDetected = Colors.NOT_FOUND;
    public enum Colors{
        BLUE,

        PURPLE,
        WHITE,
        GREEN,
        RED,
        NOT_FOUND
    }
    static double PERCENT_COLOR_THRESHHOLD = 0.4; //only for detection beyond prop detect

    private Robot13968.DetectColor detectColor;
    /*
     * The core values which define the location and size of the sample regions
     */
    static final Rect LEFT_RECT = new Rect(
            new Point(10,470),
            new Point(200,10));
    static final Rect MIDDLE_RECT = new Rect(
            new Point(220,470),
            new Point(420,10));
    static final Rect RIGHT_RECT = new Rect(
            new Point(440,470),
            new Point(630,10));

    double[] arr = new double[6];
    private volatile TSEPosition position = TSEPosition.NOT_FOUND;
    //Telemetry telemetry;
    Mat mat = new Mat();

    public TSEDetectorPipeline(Robot13968.DetectColor color) {
        super();
        detectColor = color;
       // telemetry = t;
    }

    @Override
    public void init(Mat firstFrame) {
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Mat left= mat.submat(LEFT_RECT);
        Mat middle= mat.submat(MIDDLE_RECT);
        Mat right= mat.submat(RIGHT_RECT);

        if (detectColor == Robot13968.DetectColor.RED){
            Core.inRange(mat, RED_LOW, RED_HIGH, mat);
            colorDetected = Colors.BLUE;
        }
        else if (detectColor == Robot13968.DetectColor.BLUE){
            //Core.inRange(mat, BlUE_LOW, BLUE_HIGH, mat);
            //double leftValue = Core.sumElems(left).val[0]/LEFT_RECT.area()/255;

            colorDetected = Colors.RED;

        }
        double leftValue = Core.sumElems(left).val[0]/LEFT_RECT.area()/255;
        arr[0] = Core.sumElems(left).val[0];
        arr[3] =Math.round(leftValue*100);

        double middleValue = Core.sumElems(middle).val[0]/MIDDLE_RECT.area()/255;
        arr[1] = Core.sumElems(middle).val[0];
        arr[4] =Math.round(middleValue*100);

        double rightValue = Core.sumElems(right).val[0]/RIGHT_RECT.area()/255;
        arr[2] = Core.sumElems(right).val[0];
        arr[5] =Math.round(rightValue*100);
        left.release();
        right.release();
        middle.release();
      // telemetry.addData("left raw value", (int) Core.sumElems(left).val[0]);

       // telemetry.addData("right raw value", (int) Core.sumElems(right).val[0]);
       // telemetry.addData("left percentage", Math.round(leftValue*100) + "%");
    //    telemetry.addData("right percentage", Math.round(rightValue*100) + "%");
        double maxim = Math.max(leftValue,middleValue);
        maxim= Math.max(maxim, rightValue);

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2BGR);

        if (detectColor == Robot13968.DetectColor.BLUE || detectColor == Robot13968.DetectColor.RED){
            //visual aids: section outlines
            Imgproc.rectangle(
                    mat, // Buffer to draw on
                    new Point(10,470),// First point which defines the rectangle
                    new Point(200,10), // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            /*
             * Draw a rectangle showing sample region 2 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    mat, // Buffer to draw on

                    new Point(220,470),// First point which defines the rectangle
                    new Point(420,10), // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            /*
             * Draw a rectangle showing sample region 3 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    mat, // Buffer to draw on
                    new Point(440,470),// First point which defines the rectangle
                    new Point(630,10), // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines


            if (maxim == leftValue) // Was it from region 1?
            {
                position = TSEPosition.LEFT; // Record our analysis
                /* Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.*/
                Imgproc.rectangle(
                        mat, // Buffer to draw on
                        new Point(10,470),// First point which defines the rectangle
                        new Point(200,10), // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
                 }
            else if (maxim == middleValue){ // Was it from region 2?

                position = TSEPosition.CENTER; // Record our analysis

                Imgproc.rectangle(
                        mat, // Buffer to draw on
                        new Point(220,470),// First point which defines the rectangle
                        new Point(420,10), // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }
            else if (maxim == rightValue){ // Was it from region 3?

                position = TSEPosition.RIGHT; // Record our analysis

                Imgproc.rectangle(
                        mat, // Buffer to draw on
                        new Point(440,470),// First point which defines the rectangle
                        new Point(630,10), // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }

        }
        //telemetry.update();

        return mat;
    }

    public TSEPosition getObjPosition() {
        return position;
    }
    public double[] getTelemetryInfo() {

        return arr;
    }
    public Colors getColorDetected() {
        return colorDetected;
    }

}