package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;


class Green_Pink_YCrCb_Detection extends OpenCvPipeline {
   /*
   YELLOW  = Parking Left
   CYAN    = Parking Middle
   MAGENTA = Parking Right
    */

   public enum ParkingPosition {
       LEFT,
       CENTER,
       RIGHT
   }
    Telemetry telemetry;

   // TOPLEFT anchor point for the bounding box
   private static Point SLEEVE_TOPLEFT_ANCHOR_POINT = new Point(145, 420);

   // Width and height for the bounding box
   public static int REGION_WIDTH = 30;
   public static int REGION_HEIGHT = 100;

   // Lower and upper boundaries for colors
   private static final Scalar
           //green
           lower_yellow_bounds  = new Scalar(0, 0, 50),
           upper_yellow_bounds  = new Scalar(255, 120, 130),
            //pink
           lower_cyan_bounds    = new Scalar(0, 150, 120),
           upper_cyan_bounds    = new Scalar(255, 255, 255);


   // Color definitions
   private final Scalar
           YELLOW  = new Scalar(255, 255, 0),
           CYAN    = new Scalar(0, 255, 255),
            MAGENTA = new Scalar(0, 255, 255);

   // Percent and mat definitions
   private double yelPercent, cyaPercent;
   private Mat yelMat = new Mat(), cyaMat = new Mat(), blurredMat = new Mat(),kernel=new Mat();
   int x=0;
   // Anchor point definitions
   Point sleeve_pointA = new Point(
           SLEEVE_TOPLEFT_ANCHOR_POINT.x,
           SLEEVE_TOPLEFT_ANCHOR_POINT.y);
   Point sleeve_pointB = new Point(
           SLEEVE_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
           SLEEVE_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

   // Running variable storing the parking position
   public ParkingPosition position;// = ParkingPosition.RIGHT
   public Green_Pink_YCrCb_Detection(Telemetry telemetry)
   {
       this.telemetry=telemetry;
   }
   @Override

   public Mat processFrame(Mat input) {
       Imgproc.cvtColor(input, blurredMat, Imgproc.COLOR_RGB2YCrCb);
       // Noise reduction
       Imgproc.blur(blurredMat, blurredMat, new Size(5, 5));
       blurredMat = blurredMat.submat(new Rect(sleeve_pointA, sleeve_pointB));

       // Apply Morphology
       Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
       Imgproc.morphologyEx(blurredMat, blurredMat, Imgproc.MORPH_CLOSE, kernel);

       // Gets channels from given source mat
       Core.inRange(blurredMat, lower_yellow_bounds, upper_yellow_bounds, yelMat);
       Core.inRange(blurredMat, lower_cyan_bounds, upper_cyan_bounds, cyaMat);
//       Core.inRange(blurredMat, lower_magenta_bounds, upper_magenta_bounds, magMat);

       // Gets color specific values
       yelPercent = Core.countNonZero(yelMat);
       cyaPercent = Core.countNonZero(cyaMat);
//       magPercent = Core.countNonZero(magMat);
       x++;
       // Calculates the highest amount of pixels being covered on each side
       double maxPercent = Math.max(yelPercent, cyaPercent);
       telemetry.addData("loop ",x+" yelPercent "+yelPercent+" cyaPercent "+cyaPercent);

       if(yelPercent>cyaPercent)       telemetry.addData("yell ",maxPercent);
       else if(cyaPercent>yelPercent)       telemetry.addData("cya ",maxPercent);
       else if(cyaPercent==yelPercent)       telemetry.addData("whi ",maxPercent);

       telemetry.update();
       // Checks all percentages, will highlight bounding box in camera preview
       // based on what color is being detected
       if (maxPercent == yelPercent) {
           position = ParkingPosition.LEFT;
           Imgproc.rectangle(
                   input,
                   sleeve_pointA,
                   sleeve_pointB,
                   YELLOW,
                   2
           );
       } else if (maxPercent == cyaPercent) {
           position = ParkingPosition.CENTER;
           Imgproc.rectangle(
                   input,
                   sleeve_pointA,
                   sleeve_pointB,
                   CYAN,
                   2
           );
       } else  {
           position = ParkingPosition.RIGHT;
           Imgproc.rectangle(
                   input,
                   sleeve_pointA,
                   sleeve_pointB,
                   MAGENTA,
                   2
           );
       }

       // Memory cleanup
       blurredMat.release();
       yelMat.release();
       cyaMat.release();
//       magMat.release();
       kernel.release();
       return input;
   }

   // Returns an enum being the current position where the robot will park
   public ParkingPosition getPosition() {
       return position;
   }
}

