package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.*;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class CenteringPipeline extends OpenCvPipeline {

    public Scalar lowHSV = new Scalar(0, 128, 122);
    public Scalar highHSV = new Scalar(0, 255, 255);


    private int rectangleIndex, max = 0;
    private double xDist = 0;

    private Rect rectangle;
    Telemetry telemetry;


    public CenteringPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public double getPixels() {
        return xDist;
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat mat = new Mat();
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        if (mat.empty()) {
            telemetry.addData("Status", "error");
            telemetry.update();
            return input;
        }



        Mat thresh = new Mat();
        Core.inRange(mat, lowHSV, highHSV, thresh);

        Mat edges = new Mat();
        Imgproc.Canny(thresh, edges, 100, 300);

        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];
        Rect[] boundRect = new Rect[contours.size()];
        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
        }


        for (int i = 0; i != boundRect.length; i++) {
            if (!(boundRect[i].width / boundRect[i].height > 1.7) && boundRect[i].area() > max) {
                max = (int) boundRect[i].area();
                rectangleIndex = i;
                Imgproc.rectangle(input, boundRect[i], new Scalar(255, 0, 0));
            }
        }

        rectangle = boundRect[rectangleIndex];
        Imgproc.rectangle(input, rectangle, new Scalar(255, 0, 0));


        Imgproc.line(input, new Point(rectangle.x + rectangle.width/2, rectangle.y + rectangle.height/ 2), new Point(160, rectangle.y + rectangle.height/ 2), new Scalar(255, 0, 0), 1);
        //Imgproc.line(input, new Point(160, 0), new Point(160, 240), new Scalar(255, 0, 0), 1);

        xDist = Math.abs((rectangle.x + rectangle.width / 2) - 160);

        telemetry.addData("Width", rectangle.width);
        telemetry.addData("Height", rectangle.height);
        telemetry.addData("Area", max);
        telemetry.addData("xDist", xDist);
        telemetry.update();


        return input;
    }

}
