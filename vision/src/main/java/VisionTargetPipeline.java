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

    // Vision parameters
    private final Scalar HSL_LOWER_BOUND = new Scalar(66, 122, 0); // BGR
    private final Scalar HSL_UPPER_BOUND = new Scalar(99, 255, 255); // BGR

    private final double MIN_CONTOUR_AREA = 100;

    private final double MAX_CONTOUR_RATIO = 1.5;

    // Pipeline output settings
    private final Scalar CONTOUR_COLOR = new Scalar(0, 0, 255, 255); //BGRA
    private final Scalar BOUNDING_BOX_COLOR = new Scalar(255, 0, 0, 255); //BGRA

    public VisionTargetPipeline(VideoSource source) {
        cameraMatrix = new Mat(3, 3, CvType.CV_32F);
        cameraMatrix.put(0, 0, 1.2015291176156757e+03, 0., 6.0968528848524579e+02, 0.,
                1.2015291176156757e+03, 4.0351544562818771e+02, 0., 0., 1.);

        distCoeffs = new Mat(1, 5, CvType.CV_32F);
        distCoeffs.put(0, 0, 0., 3.4668462851684589e-01, 1.9095461089056926e-02,
                1.0585419991906689e-02, -1.2734744546805421e+00);
        VideoMode mode = source.getVideoMode();
        int width = mode.width;
        int height = mode.height;
        int fps = mode.fps;
        Size imageSize = new Size(width, height);
        cameraMatrix = Calib3d.getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1);

        undistortOutput = new Mat();
        hslThresholdOutput = new Mat();
        hierarchy = new Mat();
        contours = new ArrayList<>();
        filteredContours = new ArrayList<>();
        contourBB = new ArrayList<>();
        contourBBPoints = new ArrayList<>();
        pipelineOutput = new Mat();

        System.out.println(String.format("Created Hatch Vision Pipeline with size: %d x %d", width, height));

        pipelineOutputSource = new CvSource("Pipeline Output", VideoMode.PixelFormat.kMJPEG, width, height, fps);
        pipelineOutputServer = new MjpegServer("Pipeline Output", 1185);
        pipelineOutputServer.setSource(pipelineOutputSource);
    }

    @Override
    public void process(Mat image) {
        ArrayList<Long> times = new ArrayList<>();
        times.add(System.nanoTime());
        // Undistort
        Imgproc.undistort(image, undistortOutput, cameraMatrix, distCoeffs);
        times.add(System.nanoTime());
        // Hsl Threshold
        Imgproc.cvtColor(image, hslThresholdOutput, Imgproc.COLOR_BGR2HLS);
        Core.inRange(hslThresholdOutput, HSL_LOWER_BOUND, HSL_UPPER_BOUND, hslThresholdOutput);
        times.add(System.nanoTime());
        // Find Contours
        contours.clear();
        Imgproc.findContours(hslThresholdOutput, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        times.add(System.nanoTime());
        // Filter Contours
        filteredContours.clear();
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > MIN_CONTOUR_AREA) {
                filteredContours.add(contour);
            }
        }
        times.add(System.nanoTime());
        // Find Min Area Bounding Box
        contourBB.clear();
        contourBBPoints.clear();
        Point[] points = new Point[4];
        for (MatOfPoint contour : filteredContours) {
            Rect AABB = Imgproc.boundingRect(contour);
            RotatedRect BB = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));
            double ratio = (double) AABB.width / AABB.height;
            if (ratio < MAX_CONTOUR_RATIO) {
                BB.points(points);
                contourBBPoints.add(new MatOfPoint(points));

                contourBB.add(BB);
                // Fix angles to be -180 to 0
                if (BB.size.width < BB.size.height) {
                    BB.angle -= 90;
                }
            }
        }
        times.add(System.nanoTime());
        // Display Contours
        image.copyTo(pipelineOutput);
        Imgproc.drawContours(pipelineOutput, filteredContours, -1, CONTOUR_COLOR, 2);
        Imgproc.drawContours(pipelineOutput, contourBBPoints, -1, BOUNDING_BOX_COLOR, 2);
        times.add(System.nanoTime());
        pipelineOutputSource.putFrame(pipelineOutput);
        times.add(System.nanoTime());

        System.out.print("Frame time: ");
        for (int i = 1; i < times.size(); i++) {
            System.out.print(String.format("%.2f, ", (times.get(i) - times.get(i - 1)) / 1e+6));
        }
        System.out.println(String.format("FPS: %.2f", 1000d/((times.get(7) - times.get(0)) / 1e+6)));
    }
}
