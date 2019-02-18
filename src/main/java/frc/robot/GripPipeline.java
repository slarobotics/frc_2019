package frc.robot;

import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.vision.VisionPipeline;
import org.opencv.core.*;
import org.opencv.imgproc.*;

/**
 * GripPipeline class.
 *
 * <p>
 * An OpenCV pipeline generated by GRIP.
 *
 * @author GRIP
 */
public class GripPipeline implements VisionPipeline {

	// Outputs
	private Mat hsvThresholdOutput = new Mat();
	private ArrayList<MatOfPoint> findContoursOutput = new ArrayList<MatOfPoint>();
	private ArrayList<MatOfPoint> filterContoursOutput = new ArrayList<MatOfPoint>();
	private ArrayList<MatOfPoint> convexHullsOutput = new ArrayList<MatOfPoint>();
	public Mat overlayOutput;

	private double aspectRatioOut = 0.0;
	private int contourNumber = 0;
	private int pipelineRunning = 0;
	public double targetAngle = 0.0;
	private boolean foundTarget = false; // indicate whether you found the target
	private int targetTop, targetBottom, targetLeft, targetRight, targetHeight, targetWidth;
	public double targetDistance;

	public int centerX;
	public int centerY;

	public double distanceCenterTarget;

	static {
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
	}

	/**
	 * This is the primary method that runs the entire pipeline and updates the
	 * outputs.
	 */
	@Override
	public void process(Mat source0) {
		// Step HSV_Threshold0:
		overlayOutput = source0;

		Mat hsvThresholdInput = source0;
		double[] hsvThresholdHue = { 0.0, 142.52560664362468 };
		double[] hsvThresholdSaturation = { 0.0, 255.0 };
		double[] hsvThresholdValue = { 213.72301458454817, 255.0 };
		hsvThreshold(hsvThresholdInput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue, hsvThresholdOutput);

		// Step Find_Contours0:
		Mat findContoursInput = hsvThresholdOutput;
		boolean findContoursExternalOnly = false;
		findContours(findContoursInput, findContoursExternalOnly, findContoursOutput);

		// Step Filter_Contours0:
		ArrayList<MatOfPoint> filterContoursContours = findContoursOutput;
		double filterContoursMinArea = 1000;
		double filterContoursMinPerimeter = 0.0;
		double filterContoursMinWidth = 0.0;
		double filterContoursMaxWidth = 1000;
		double filterContoursMinHeight = 0;
		double filterContoursMaxHeight = 1000;
		double[] filterContoursSolidity = { 0.0, 100.0 };
		double filterContoursMaxVertices = 1000000;
		double filterContoursMinVertices = 0;
		double filterContoursMinRatio = .5;
		double filterContoursMaxRatio = 1;
		filterContours(filterContoursContours, filterContoursMinArea, filterContoursMinPerimeter,
				filterContoursMinWidth, filterContoursMaxWidth, filterContoursMinHeight, filterContoursMaxHeight,
				filterContoursSolidity, filterContoursMaxVertices, filterContoursMinVertices, filterContoursMinRatio,
				filterContoursMaxRatio, filterContoursOutput);

		// Step Convex_Hulls0:
		ArrayList<MatOfPoint> convexHullsContours = filterContoursOutput;
		convexHulls(convexHullsContours, convexHullsOutput);
		findTarget(convexHullsContours);
	}

	public void findTarget(List<MatOfPoint> inputContours) {
		pipelineRunning++;
		if (pipelineRunning > 1000) {
			pipelineRunning = 0;
		}
		int n = inputContours.size();
		int i, j;
		Rect r1, r2;
		double temp, aspectRatio;

		foundTarget = false;

		contourNumber = inputContours.size();

		if (n >= 2) {
			// System.out.println("# of rectangles: " + n);

			// Create two bounding rectangles over the contours
			r1 = Imgproc.boundingRect(inputContours.get(0));
			r2 = Imgproc.boundingRect(inputContours.get(1));

			// Get center x and y for the two rectangles
			int x1 = r1.x + (r1.width / 2);
			int y1 = r1.y + (r1.height / 2);

			int x2 = r2.x + (r2.width / 2);
			int y2 = r2.y + (r2.height / 2);

			// Get center x and y between both rectangles
			centerX = (x1 + x2) / 2;
			centerY = (y1 + y2) / 2;

			final MatOfInt hull = new MatOfInt();
			final MatOfInt hull2 = new MatOfInt();

			// Draw circle for the center
			Imgproc.circle(overlayOutput, new Point(centerX, centerY), 5, new Scalar(128, 128, 0));

			// System.out.println("x: " + centerX + " y: " + centerY);

			// Subtact the center X point to half of the camera feed width
			distanceCenterTarget = centerX - 160.0;
			// Get angle based on camera settings
			targetAngle = 0.5934352563 * (distanceCenterTarget / 320);

			// Based on both of the targets get top/bottom pixel and height of targets.
			targetTop = Math.min(r1.y, r2.y);
			targetBottom = Math.max(r1.y + r1.height, r2.y + r2.height);
			targetHeight = targetBottom - targetTop;

			// Get the distance from target based on the width of target vs camera
			// sensors/real thing
			targetDistance = (8.0 / targetWidth) * 346.0; // distance in inches (+/- 4)

			// System.out.println("distance center target: " + distanceCenterTarget);
			// System.out.println("target angle " + targetAngle);
			// System.out.println("target distance " + targetAngle);

			// Draw the rectangles on the screen
			Imgproc.rectangle(overlayOutput, new Point(r1.x, r1.y), new Point(r1.x + r1.width, r1.y + r1.height),
					new Scalar(0, 225, 0));

			Imgproc.rectangle(overlayOutput, new Point(r2.x, r2.y), new Point(r2.x + r2.width, r2.y + r2.height),
					new Scalar(0, 0, 255));

			// Draw all the contours.
			/*
			 * for (int x = 0; x < n; x++) { Imgproc.drawContours(overlayOutput,
			 * inputContours, x, new Scalar(225, 0, 0)); }
			 */
		}
	}

	/**
	 * This method is a generated getter for the output of a HSV_Threshold.
	 * 
	 * @return Mat output from HSV_Threshold.
	 */
	public Mat hsvThresholdOutput() {
		return hsvThresholdOutput;
	}

	/**
	 * This method is a generated getter for the output of a Find_Contours.
	 * 
	 * @return ArrayList<MatOfPoint> output from Find_Contours.
	 */
	public ArrayList<MatOfPoint> findContoursOutput() {
		return findContoursOutput;
	}

	/**
	 * This method is a generated getter for the output of a Filter_Contours.
	 * 
	 * @return ArrayList<MatOfPoint> output from Filter_Contours.
	 */
	public ArrayList<MatOfPoint> filterContoursOutput() {
		return filterContoursOutput;
	}

	/**
	 * This method is a generated getter for the output of a Convex_Hulls.
	 * 
	 * @return ArrayList<MatOfPoint> output from Convex_Hulls.
	 */
	public ArrayList<MatOfPoint> convexHullsOutput() {
		return convexHullsOutput;
	}

	/**
	 * Segment an image based on hue, saturation, and value ranges.
	 *
	 * @param input  The image on which to perform the HSL threshold.
	 * @param hue    The min and max hue
	 * @param sat    The min and max saturation
	 * @param val    The min and max value
	 * @param output The image in which to store the output.
	 */
	private void hsvThreshold(Mat input, double[] hue, double[] sat, double[] val, Mat out) {
		Imgproc.cvtColor(input, out, Imgproc.COLOR_BGR2HSV);
		Core.inRange(out, new Scalar(hue[0], sat[0], val[0]), new Scalar(hue[1], sat[1], val[1]), out);
	}

	/**
	 * Sets the values of pixels in a binary image to their distance to the nearest
	 * black pixel.
	 * 
	 * @param input    The image on which to perform the Distance Transform.
	 * @param type     The Transform.
	 * @param maskSize the size of the mask.
	 * @param output   The image in which to store the output.
	 */
	private void findContours(Mat input, boolean externalOnly, List<MatOfPoint> contours) {
		Mat hierarchy = new Mat();
		contours.clear();
		int mode;
		if (externalOnly) {
			mode = Imgproc.RETR_EXTERNAL;
		} else {
			mode = Imgproc.RETR_LIST;
		}
		int method = Imgproc.CHAIN_APPROX_SIMPLE;
		Imgproc.findContours(input, contours, hierarchy, mode, method);
	}

	/**
	 * Filters out contours that do not meet certain criteria.
	 * 
	 * @param inputContours  is the input list of contours
	 * @param output         is the the output list of contours
	 * @param minArea        is the minimum area of a contour that will be kept
	 * @param minPerimeter   is the minimum perimeter of a contour that will be kept
	 * @param minWidth       minimum width of a contour
	 * @param maxWidth       maximum width
	 * @param minHeight      minimum height
	 * @param maxHeight      maximimum height
	 * @param Solidity       the minimum and maximum solidity of a contour
	 * @param minVertexCount minimum vertex Count of the contours
	 * @param maxVertexCount maximum vertex Count
	 * @param minRatio       minimum ratio of width to height
	 * @param maxRatio       maximum ratio of width to height
	 */
	private void filterContours(List<MatOfPoint> inputContours, double minArea, double minPerimeter, double minWidth,
			double maxWidth, double minHeight, double maxHeight, double[] solidity, double maxVertexCount,
			double minVertexCount, double minRatio, double maxRatio, List<MatOfPoint> output) {
		final MatOfInt hull = new MatOfInt();
		output.clear();
		// operation
		for (int i = 0; i < inputContours.size(); i++) {
			final MatOfPoint contour = inputContours.get(i);
			final Rect bb = Imgproc.boundingRect(contour);
			if (bb.width < minWidth || bb.width > maxWidth)
				continue;
			if (bb.height < minHeight || bb.height > maxHeight)
				continue;
			final double area = Imgproc.contourArea(contour);
			if (area < minArea)
				continue;
			if (Imgproc.arcLength(new MatOfPoint2f(contour.toArray()), true) < minPerimeter)
				continue;
			Imgproc.convexHull(contour, hull);
			MatOfPoint mopHull = new MatOfPoint();
			mopHull.create((int) hull.size().height, 1, CvType.CV_32SC2);
			for (int j = 0; j < hull.size().height; j++) {
				int index = (int) hull.get(j, 0)[0];
				double[] point = new double[] { contour.get(index, 0)[0], contour.get(index, 0)[1] };
				mopHull.put(j, 0, point);
			}
			final double solid = 100 * area / Imgproc.contourArea(mopHull);
			if (solid < solidity[0] || solid > solidity[1])
				continue;
			if (contour.rows() < minVertexCount || contour.rows() > maxVertexCount)
				continue;
			final double ratio = bb.width / (double) bb.height;
			if (ratio < minRatio || ratio > maxRatio)
				continue;
			output.add(contour);
		}
	}

	/**
	 * Compute the convex hulls of contours.
	 * 
	 * @param inputContours  The contours on which to perform the operation.
	 * @param outputContours The contours where the output will be stored.
	 */
	private void convexHulls(List<MatOfPoint> inputContours, ArrayList<MatOfPoint> outputContours) {
		final MatOfInt hull = new MatOfInt();
		outputContours.clear();
		for (int i = 0; i < inputContours.size(); i++) {
			final MatOfPoint contour = inputContours.get(i);
			final MatOfPoint mopHull = new MatOfPoint();
			Imgproc.convexHull(contour, hull);
			mopHull.create((int) hull.size().height, 1, CvType.CV_32SC2);
			for (int j = 0; j < hull.size().height; j++) {
				int index = (int) hull.get(j, 0)[0];
				double[] point = new double[] { contour.get(index, 0)[0], contour.get(index, 0)[1] };
				mopHull.put(j, 0, point);
			}
			outputContours.add(mopHull);
		}
	}

}