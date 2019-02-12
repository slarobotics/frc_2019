package frc.robot;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import java.util.HashMap;

import edu.wpi.first.vision.VisionPipeline;

import org.opencv.core.*;
import org.opencv.core.Core.*;
import org.opencv.features2d.FeatureDetector;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.*;
import org.opencv.objdetect.*;

/**
 * GripPipeline class.
 *
 * <p>
 * An OpenCV pipeline generated by GRIP.
 *
 * @author GRIP
 */
public class drawRect implements VisionPipeline {

    // Outputs
    private Point newPoint0Output = new Point();
    private Point newPoint1Output = new Point();
    private Mat cvRectangleOutput = new Mat();

    static {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
    }

    /**
     * This is the primary method that runs the entire pipeline and updates the
     * outputs.
     */
    @Override
    public void process(Mat source0) {
        // Step New_Point0:
        double newPoint0X = 40.0;
        double newPoint0Y = 40.0;
        newPoint(newPoint0X, newPoint0Y, newPoint0Output);

        // Step New_Point1:
        double newPoint1X = 600.0;
        double newPoint1Y = 450.0;
        newPoint(newPoint1X, newPoint1Y, newPoint1Output);

        // Step CV_rectangle0:
        Mat cvRectangleSrc = source0;
        Point cvRectanglePt1 = newPoint0Output;
        Point cvRectanglePt2 = newPoint1Output;
        Scalar cvRectangleColor = new Scalar(0.0, 0.0, 0.0, 0.0);
        double cvRectangleThickness = 0;
        int cvRectangleLinetype = Core.LINE_8;
        double cvRectangleShift = 0;
        cvRectangle(cvRectangleSrc, cvRectanglePt1, cvRectanglePt2, cvRectangleColor, cvRectangleThickness,
                cvRectangleLinetype, cvRectangleShift, cvRectangleOutput);

    }

    /**
     * This method is a generated getter for the output of a New_Point.
     * 
     * @return Point output from New_Point.
     */
    public Point newPoint0Output() {
        return newPoint0Output;
    }

    /**
     * This method is a generated getter for the output of a New_Point.
     * 
     * @return Point output from New_Point.
     */
    public Point newPoint1Output() {
        return newPoint1Output;
    }

    /**
     * This method is a generated getter for the output of a CV_rectangle.
     * 
     * @return Mat output from CV_rectangle.
     */
    public Mat cvRectangleOutput() {
        return cvRectangleOutput;
    }

    /**
     * Fills a point with given x and y values.
     * 
     * @param x     the x value to put in the point
     * @param y     the y value to put in the point
     * @param point the point to fill
     */
    private void newPoint(double x, double y, Point point) {
        point.x = x;
        point.y = y;
    }

    /**
     * Draws a rectangle on an image.
     * 
     * @param src       Image to draw rectangle on.
     * @param pt1       one corner of the rectangle.
     * @param pt2       opposite corner of the rectangle.
     * @param color     Scalar indicating color to make the rectangle.
     * @param thickness Thickness of the lines of the rectangle.
     * @param lineType  Type of line for the rectangle.
     * @param shift     Number of decimal places in the points.
     * @param dst       output image.
     */
    private void cvRectangle(Mat src, Point pt1, Point pt2, Scalar color, double thickness, int lineType, double shift,
            Mat dst) {
        src.copyTo(dst);
        if (color == null) {
            color = Scalar.all(1.0);
        }
        Imgproc.rectangle(dst, pt1, pt2, color, (int) thickness, lineType, (int) shift);
    }

}