package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import java.util.ArrayList;
@Autonomous(name = "opencv Test")
public class OpenCVExampleOpMode extends OpMode {
    static final int STREAM_WIDTH = 640; // modify for your camera
    static final int STREAM_HEIGHT = 360; // modify for your camera
    OpenCvWebcam webcam;
    SamplePipeline pipeline;
    @Override
    public void init() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = null;
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1"); // put your camera's name here
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        pipeline = new SamplePipeline();
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(STREAM_WIDTH, STREAM_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Failed","");
                telemetry.update();
            }
        });

    }

    @Override
    public void loop() {
        telemetry.addData("Image Analysis:",pipeline.getAnalysis());
        telemetry.update();
    }


}
class SamplePipeline extends OpenCvPipeline {

    Mat YCrCb = new Mat();
    Mat output = new Mat();
    Mat Y = new Mat();
    Mat Cr = new Mat();
    Mat Cb = new Mat();
    Mat RectA_Y = new Mat();
    int avg;
    int avgA;
    static final int STREAM_WIDTH = 640; // modify for your camera
    static final int STREAM_HEIGHT = 360; // modify for your camera


    static final int WidthRectA = 130;
    static final int HeightRectA = 110;


    static final Point RectATopLeftAnchor = new Point((STREAM_WIDTH - WidthRectA) / 2 + 300, ((STREAM_HEIGHT - HeightRectA) / 2) - 100);
    Point RectATLCorner = new Point(
            RectATopLeftAnchor.x,
            RectATopLeftAnchor.y);
    Point RectABRCorner = new Point(
            RectATopLeftAnchor.x + WidthRectA,
            RectATopLeftAnchor.y + HeightRectA);




    /*
     * This function takes the RGB frame, converts to YCrCb,
     * and extracts the Y channel to the 'Y' variable
     */
    void inputToY(Mat input) {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        ArrayList<Mat> yCrCbChannels = new ArrayList<Mat>(3);
        Core.split(YCrCb, yCrCbChannels);
        Y = yCrCbChannels.get(0);
        Cr = yCrCbChannels.get(1);
        Cb = yCrCbChannels.get(2);

    }

    @Override
    public void init(Mat firstFrame) {
        inputToY(firstFrame);
        RectA_Y = Y.submat(new Rect(RectATLCorner, RectABRCorner));
    }

    @Override
    public Mat processFrame(Mat input) {
        inputToY(input);
        System.out.println("processing requested");
        avg = (int) Core.mean(Y).val[0];
        avgA = (int) Core.mean(RectA_Y).val[0];
//        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
//        input.copyTo(output);

        YCrCb.release(); // don't leak memory!
        Y.release(); // don't leak memory!
        Cr.release(); // don't leak memory!
        Cb.release(); // don't leak memory!
        output.release();


        Imgproc.rectangle( input, // Buffer to draw on
                RectATLCorner, // First point which defines the rectangle
                RectABRCorner, // Second point which defines the rectangle
                new Scalar(0,0,255), // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        return input;
    }

    public int getAnalysis() {
        return avg;
    }
    public int getRectA_Analysis() {
        return avgA;
    }

}