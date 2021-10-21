package org.firstinspires.ftc.teamcode.Subsystems.Vision.DetectMarker;

<<<<<<< HEAD
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Config.VisionConfig;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

public class DetectMarker {
    Robot robot;
    HardwareMap hardwareMap;
    OpenCvInternalCamera robotCamera;
    MarkerLocation markerLocation = MarkerLocation.NOT_FOUND;

    public DetectMarker(Robot robot, OpenCvInternalCamera camera) {
        this.robot = robot;
        this.hardwareMap = robot.getOpMode().hardwareMap;
        this.robotCamera = camera;
    }

    /**
     * This method waits until the search for the marker is done, and then it return the marker
     * location. It waits until {@link DetectMarkerPipeline#getSearchStatus() } returns the marker
     * location.
     *
     * @return Where the marker is
     *
     * @see DetectMarkerPipeline#getMarkerLocation()
     */
    public MarkerLocation DetectMarkerRun() {
        DetectMarkerPipeline detectMarkerPipeline = new DetectMarkerPipeline(robot);
        robotCamera.setPipeline(detectMarkerPipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        robotCamera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        robotCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                robotCamera.startStreaming(VisionConfig.CAMERA_WIDTH, VisionConfig.CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        markerLocation = detectMarkerPipeline.getMarkerLocation();
        return markerLocation;
    }
=======
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.AllianceColor;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.Vision;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

/**
 * This pipeline detects where the marker is.
 *
 * <p>It does this by splitting the camera input into 3 parts, the Left, Middle, and Right. It
 * checks each part for a custom marker (which is set to be green in the code), or some blue or red
 * tape, dependant on the alliance color.</p>
 * @see org.openftc.easyopencv.OpenCvPipeline
 * @see Vision
 */
public class DetectMarker extends OpenCvPipeline {
    Telemetry telemetry;

    private AllianceColor allianceColor;
    private MarkerLocation markerLocation = MarkerLocation.NOT_FOUND;
    private SearchStatus searchStatus = SearchStatus.INITIALIZING;


    static final Rect LEFT_RECT = new Rect(
            new Point(60, 35),
            new Point(110, 75));

    static final Rect MIDDLE_RECT = new Rect(
            new Point(110, 35),
            new Point(150, 75));

    static final Rect RIGHT_RECT = new Rect(
            new Point(150, 35),
            new Point(200, 75));

    static double PERCENT_COLOR_THRESHOLD = 0.4;


    Mat mat = new Mat();

    /**
     * Class instantiation
     * @param robot The robot (used for {@link Telemetry})
     * @param ac The alliance color in {@link AllianceColor} format.
     *
     * @see Robot
     * @see Telemetry
     * @see AllianceColor
     */
    public DetectMarker(Robot robot, AllianceColor ac) {
        telemetry = robot.getOpMode().telemetry;
        this.allianceColor = ac;
    }

    /**
     * This method detects where the marker is.
     *
     * <p>It does this by splitting the camera input into left, right, and middle rectangles, these
     * rectangles need to be calibrated. Combined, they do not have to encompass the whole camera
     * input, they probably will only check a small part of it. We then assume the alliance color is
     * either (255, 0, 0) or (0, 0, 255), we get the info when the object is instantiated
     * ({@link #allianceColor}), and that the marker color is (0, 255, 0), which is a bright green
     * ({@link Scalar}'s are used for colors). We compare the marker color with the alliance color
     * on each of the rectangles, if the marker color is on none or multiple of them, it is marked
     * as {@link MarkerLocation#NOT_FOUND}, if otherwise, the respective Location it is in is
     * returned via a {@link MarkerLocation} variable called {@link #markerLocation}</p>
     *
     * @param input A Mat
     * @return The marker location
     *
     * @see #allianceColor
     * @see Mat
     * @see Scalar
     * @see MarkerLocation
     */
    @Override
    public Mat processFrame(Mat input) {
        this.searchStatus = SearchStatus.SEARCHING;
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV); // TODO: Change COLOR_RGB2HSV to something more useful. (not possible)
        Scalar lowHSV = new Scalar(23, 50, 70);
        Scalar highHSV = new Scalar(32, 255, 255);

        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat left = mat.submat(LEFT_RECT);
        Mat middle = mat.submat(MIDDLE_RECT);
        Mat right = mat.submat(RIGHT_RECT);

        double leftValue = Core.sumElems(left).val[0] / LEFT_RECT.area() / 255;
        double middleValue = Core.sumElems(left).val[0] / MIDDLE_RECT.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_RECT.area() / 255;

        left.release();
        middle.release();
        right.release();

        telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]);
        telemetry.addData("Middle raw value", (int) Core.sumElems(left).val[0]);
        telemetry.addData("Right raw value", (int) Core.sumElems(right).val[0]);
        telemetry.update();

        telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
        telemetry.addData("Middle percentage", Math.round(leftValue * 100) + "%");
        telemetry.addData("Right percentage", Math.round(rightValue * 100) + "%");
        telemetry.update();

        boolean markerLeft = leftValue > PERCENT_COLOR_THRESHOLD;
        boolean markerMiddle = middleValue > PERCENT_COLOR_THRESHOLD;
        boolean markerRight = rightValue > PERCENT_COLOR_THRESHOLD;


        if (markerLeft) {
            markerLocation = MarkerLocation.LEFT;
            telemetry.addData("Marker Location", "right");
        } else if (markerMiddle) {
            markerLocation = MarkerLocation.MIDDLE;
            telemetry.addData("Marker Location", "middle");
        } else if (markerRight) {
            markerLocation = MarkerLocation.RIGHT;
            telemetry.addData("Marker Location", "left");
        } else {
            markerLocation = MarkerLocation.NOT_FOUND;
            telemetry.addData("Marker Location", "not found");
        }
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB); // TODO: Change COLOR_GRAY2RGB to something more useful.

        Scalar colorNormal;

        if (this.allianceColor == AllianceColor.RED) {
            colorNormal = new Scalar(255, 0, 0); // Pure Red
        }
        else if (this.allianceColor == AllianceColor.BLUE) {
            colorNormal = new Scalar(0, 0, 255); // Pure Blue
        }
        else {
            colorNormal = new Scalar(255, 0, 255); // Pure Blue
        }

        Scalar colorMarker = new Scalar(0, 255, 0); // Pure Green

        Imgproc.rectangle(mat, LEFT_RECT, markerLocation == MarkerLocation.LEFT ? colorMarker : colorNormal);
        Imgproc.rectangle(mat, MIDDLE_RECT, markerLocation == MarkerLocation.MIDDLE ? colorMarker : colorNormal);
        Imgproc.rectangle(mat, RIGHT_RECT, markerLocation == MarkerLocation.RIGHT ? colorMarker : colorNormal);

        this.searchStatus = SearchStatus.FOUND;
        return mat;
    }

    /**
     * Gets the Marker Location, might be not found because of the Search Status.
     * @return Where the marker is.
     * @see MarkerLocation
     */
    public MarkerLocation getMarkerLocation() {
        return markerLocation;
    }

    /**
     * Gets the search status
     * @return the search status, which is in the {@link #searchStatus} enum format.
     * @see #searchStatus
     */
    public SearchStatus getSearchStatus() {
        return searchStatus;
    }
>>>>>>> drive
}
