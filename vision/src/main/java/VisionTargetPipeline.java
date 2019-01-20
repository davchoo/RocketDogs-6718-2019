import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.VideoMode;
import edu.wpi.cscore.VideoSource;
import edu.wpi.first.vision.VisionPipeline;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

public class VisionTargetPipeline implements VisionPipeline {
    private CvSource pipelineOutputSource;
    private MjpegServer pipelineOutputServer;
    private Mat pipelineOutput;

    private Mat cameraMatrix;
    private Mat distCoeffs;

    private Mat undistortOutput;
    private Mat hslThresholdOutput;
    private Mat hierarchy;
    private ArrayList<MatOfPoint> contours;
    private ArrayList<MatOfPoint> filteredContours;
    public ArrayList<RotatedRect> contourBB;
    private ArrayList<MatOfPoint> contourBBPoints;
    private ArrayList<RotatedRect> leftTargets, rightTargets;
    public ArrayList<TargetPair> targetPairs;

    private Profiler profiler;

    // Vision parameters
    private final Scalar HSL_LOWER_BOUND = new Scalar(66, 122, 34); // BGR
    private final Scalar HSL_UPPER_BOUND = new Scalar(99, 255, 255); // BGR

    private final double MIN_CONTOUR_AREA = 100;
    private final double MAX_CONTOUR_RATIO = 0.75;
    private final double MAX_ABSOLUTE_ANGLE = 30;

    // Pipeline output settings
    private final Scalar CONTOUR_COLOR = new Scalar(0, 0, 255, 255); // BGRA
    private final Scalar BOUNDING_BOX_COLOR = new Scalar(255, 0, 0, 255); // BGRA

    // 5.08cm x 13.97cm
    private final double VISION_TARGET_WIDTH = 5.08;
    private final double VISION_TARGET_HEIGHT = 13.97;
    private final double VISION_TARGET_BETWEEN_CENTERS = 28.836;

    class TargetPair {
        public RotatedRect left, right;
        public Point center, lowest, highest;
        public double closestDistance;

        public TargetPair(RotatedRect left, RotatedRect right) {
            this.left = left;
            this.right = right;
            center = new Point((left.center.x + right.center.x) / 2d, (left.center.y + right.center.y) / 2d);
            lowest = new Point(1e10, 1e10);
            highest = new Point(0, 0);
            Point[] points = new Point[4];
            left.points(points);
            double leftClosestX = 0;
            for (Point point: points) {
                lowest.x = Math.min(lowest.x, point.x);
                lowest.y = Math.min(lowest.y, point.y);
                highest.y = Math.max(highest.y, point.y);
                leftClosestX = Math.max(leftClosestX, point.x);
            }

            right.points(points);
            double rightClosestX = 1e100;
            for (Point point: points) {
                lowest.y = Math.min(lowest.y, point.y);
                highest.x = Math.max(highest.x, point.x);
                highest.y = Math.max(highest.y, point.y);
                rightClosestX = Math.min(rightClosestX, point.x);
            }
            closestDistance = rightClosestX - leftClosestX;
        }
    }

    public VisionTargetPipeline(VideoSource source) {
        cameraMatrix = new Mat(3, 3, CvType.CV_32F);
        cameraMatrix.put(0, 0, 286.03910029056937, 0, 144.5345412035536, 0, 286.03910029056937, 91.2870399220077, 0., 0., 1.);

        distCoeffs = new Mat(1, 5, CvType.CV_32F);
        distCoeffs.put(0, 0, 0, 0, 0, 0, 1.5150660683721835);
        VideoMode mode = source.getVideoMode();
        int width = mode.width;
        int height = mode.height;
        int fps = mode.fps;
        Size imageSize = new Size(width, height);
        cameraMatrix = Calib3d.getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 0);

        undistortOutput = new Mat();
        hslThresholdOutput = new Mat();

        hierarchy = new Mat();
        contours = new ArrayList<>();
        filteredContours = new ArrayList<>();
        contourBB = new ArrayList<>();
        contourBBPoints = new ArrayList<>();
        leftTargets = new ArrayList<>();
        rightTargets = new ArrayList<>();
        targetPairs = new ArrayList<>();

        profiler = new Profiler();

        pipelineOutput = new Mat();

        System.out.println(String.format("Created Hatch Vision Pipeline with size: %d x %d", width, height));

        pipelineOutputSource = new CvSource("Pipeline Output", VideoMode.PixelFormat.kMJPEG, width, height, fps);
        pipelineOutputServer = new MjpegServer("Pipeline Output", 1185);
        pipelineOutputServer.setSource(pipelineOutputSource);
    }

    @Override
    public void process(Mat image) {
        profiler.start();
        // Undistort
        Imgproc.undistort(image, undistortOutput, cameraMatrix, distCoeffs);
        profiler.section("Undistort");
        // Hsl Threshold
        Imgproc.cvtColor(undistortOutput, hslThresholdOutput, Imgproc.COLOR_BGR2HLS);
        Core.inRange(hslThresholdOutput, HSL_LOWER_BOUND, HSL_UPPER_BOUND, hslThresholdOutput);
        profiler.section("HSL");
        // Find Contours
        contours.clear();
        Imgproc.findContours(hslThresholdOutput, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        profiler.section("Find Contour");
        // Filter Contours
        filteredContours.clear();
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > MIN_CONTOUR_AREA) {
                filteredContours.add(contour);
            }
        }
        profiler.section("Filter Contour");
        // Find Min Area Bounding Box
        contourBB.clear();
        contourBBPoints.clear();
        leftTargets.clear();
        rightTargets.clear();
        Point[] points = new Point[4];
        for (MatOfPoint contour : filteredContours) {
            RotatedRect BB = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));
            // Fix angles and sizes
            if (BB.size.width > BB.size.height) {
                double temp = BB.size.width;
                BB.size.width = BB.size.height;
                BB.size.height = temp;
                BB.angle += 90;
            }
            double ratio = BB.size.width / BB.size.height;
            if (ratio < MAX_CONTOUR_RATIO && Math.abs(BB.angle) < MAX_ABSOLUTE_ANGLE) {
                BB.points(points);
                contourBBPoints.add(new MatOfPoint(points));
                contourBB.add(BB);
                if (BB.angle > 0) {
                    leftTargets.add(BB);
                }else{
                    rightTargets.add(BB);
                }
            }
        }
        profiler.section("Min Area BB");
        // Draw Contours
        undistortOutput.copyTo(pipelineOutput);
        Imgproc.drawContours(pipelineOutput, filteredContours, -1, CONTOUR_COLOR, 2);
        Imgproc.drawContours(pipelineOutput, contourBBPoints, -1, BOUNDING_BOX_COLOR, 2);
        profiler.section("Draw");

        // Find Pairs
        targetPairs.clear();
        for (RotatedRect left : leftTargets) {
            RotatedRect likelyPair = null;
            double lastXDiff = 1e10;
            for (RotatedRect right : rightTargets) {
                double xDiff = right.center.x - left.center.x;
                if (xDiff > 0 && xDiff < lastXDiff) { // Targets that closest to the right
                    likelyPair = right;
                    lastXDiff = xDiff;
                }
            }
            if (likelyPair != null) {
                targetPairs.add(new TargetPair(left, likelyPair));
            }
        }
        profiler.section("Find Pairs");
        // Draw Pairs
        if (targetPairs.size() > 0) {
            for (TargetPair pair : targetPairs) {
                Imgproc.rectangle(pipelineOutput,
                        new Point(pair.lowest.x, pair.lowest.y),
                        new Point(pair.highest.x, pair.highest.y),
                        new Scalar(0, 255, 255, 255), 2);
                Imgproc.rectangle(pipelineOutput,
                        new Point(pair.center.x - pair.closestDistance / 2d, pair.lowest.y),
                        new Point(pair.center.x + pair.closestDistance / 2d, pair.highest.y),
                        new Scalar(255, 0, 255, 255), 2);
            }
        }
        profiler.section("Draw Pairs");

        // Display Pipeline Output
        pipelineOutputSource.putFrame(pipelineOutput);
        profiler.section("Display");
        profiler.stop();
    }
}
