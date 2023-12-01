package org.firstinspires.ftc.teamcode;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class BluePropDetector extends OpenCvPipeline {
    public final int SIG_1_LEFT = 1;
    public final int SIG_2_MIDDLE = 2;
    public final int SIG_3_RIGHT = 3;
    public final double zoneRange_left = 0.3;
    public final double zoneRange_right = 0.25;
    public final double zoneRange_topFilter = 0.7;
    

    private int m_signalNumber = 0;
    
    private final boolean DEBUG_MODE = true;
    
    public final Scalar RED = new Scalar(255, 0, 0);
    public final Scalar BLUE = new Scalar(0, 0, 255);
   
    private int width = 0; // width of the image
    private int height = 0;

    private final Scalar lowHSV_2 = new Scalar(100,150,0);
    private final Scalar highHSV_2 = new Scalar(140,255,255);

    private Mat m_mat = null;

    private Telemetry telemetry;

    public BluePropDetector(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    /**
     * @param width The width of the image (check your camera)
     */
    public BluePropDetector(int width) {
        this.width = width;
    }

    @Override
    public Mat processFrame(Mat input) {
        
        // get the size range for the image check
        if (width == 0) width = input.width();
        if (height == 0) height = input.height();

        // all the temporary shape related variables
        Mat edges = null;
        Mat thresh = null;
        
        Mat hierarchy = null;
        MatOfPoint2f[] contoursPoly = null;
        Rect[] boundRect = null;
        List<MatOfPoint> contours = null;
  
        // Make a working copy of the input matrix in HSV
        if (m_mat != null) {
            m_mat.release();
            m_mat = null; // make sure the memory for the previous matrix is released
        } 

        m_mat = new Mat();
        Imgproc.cvtColor(input, m_mat, Imgproc.COLOR_RGB2HSV);

        // get three zones
        int line1 = (int)(input.width() * zoneRange_left);
        int line2 = (int)(input.width() * (1 - zoneRange_right));
        int verticalFilterLine = (int)(input.height() * zoneRange_topFilter); 
                    
        thresh = new Mat();
        Core.inRange(m_mat, lowHSV_2, highHSV_2, thresh);
        
        // Use Canny Edge Detection to find edges, you might have to tune the thresholds for hysteresis
        edges = new Mat();
        Imgproc.Canny(thresh, edges, 100, 300);

        // https://docs.opencv.org/3.4/da/d0c/tutorial_bounding_rects_circles.html
        // Oftentimes the edges are disconnected. findContours connects these edges.
        contours = new ArrayList<>();
        hierarchy = new Mat();
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        boundRect = new Rect[contours.size()];
        contoursPoly = new MatOfPoint2f[contours.size()];
        
        int maxWidth = 0;
        int maxHeight = 0;
       
        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
            maxWidth = Math.max(maxWidth, boundRect[i].width);
            maxHeight = Math.max(maxHeight, boundRect[i].height);
        }
        
        if (DEBUG_MODE && telemetry != null) {
            telemetry.addData("input(width,height)=", "(%d, %d)", input.width(), input.height()); //input(width,height)= : (320, 180)
            telemetry.addData("2 lines", "(%d, %d)", line1, line2);
            telemetry.addData("verticalFilterLine", "(%d)", verticalFilterLine);
            telemetry.addData("maxWidth, maxHeight", "(%d, %d)", maxWidth, maxHeight);
            telemetry.addData("===", "===") ;
        }
          
        int leftZoneCnt = 0;
        int middleZoneCnt = 0;
        int rightZoneCnt = 0;
        int tmpArea = 0;
        
        // ok, we've got the contours. let's check the shape
        int counter = 0;
        for (int i = 0; i != boundRect.length; i++) 
        {
          if (boundRect[i].x < line1) {
            tmpArea = getAreaAfterFilter(boundRect[i], verticalFilterLine);
            leftZoneCnt += tmpArea;
          }
          else if (boundRect[i].x >= line1 && boundRect[i].x <= line2)
          {
            // middleZoneCnt += boundRect[i].width * boundRect[i].height;
            tmpArea = getAreaAfterFilter(boundRect[i], verticalFilterLine);
            middleZoneCnt += tmpArea;
            
          }else{
            tmpArea = getAreaAfterFilter(boundRect[i], verticalFilterLine);
            rightZoneCnt += tmpArea;
          } 
          
          if (DEBUG_MODE) {
              if (tmpArea == 0)
                Imgproc.rectangle(m_mat, boundRect[i], BLUE, 1); 
              else
                Imgproc.rectangle(m_mat, boundRect[i], RED, 1); 
          }
          
          if (DEBUG_MODE && telemetry != null)
          {
              telemetry.addData("boundRect(x,y)", "(%d, %d)", boundRect[i].x, boundRect[i].y);
              telemetry.addData("boundRect(width, height)", "(%d, %d)", boundRect[i].width, boundRect[i].height);
              telemetry.addData("(leftZoneCnt,middleZoneCnt,rightZoneCnt) =", "(%d, %d, %d)", leftZoneCnt,middleZoneCnt,rightZoneCnt);
              telemetry.addData("====", "====");
          }
           
        }
        if (DEBUG_MODE && telemetry != null) {
            telemetry.addData("(leftZoneCnt,middleZoneCnt,rightZoneCnt) =", "(%d, %d, %d)", leftZoneCnt,middleZoneCnt,rightZoneCnt);
        }
        
        double maxZoneCnt = leftZoneCnt;
        maxZoneCnt = Math.max(maxZoneCnt,middleZoneCnt);
        maxZoneCnt = Math.max(maxZoneCnt,rightZoneCnt);

        String str = "";

        if (leftZoneCnt == maxZoneCnt)
        {
          m_signalNumber = SIG_1_LEFT;
          str = "Left";
        }
        else if (middleZoneCnt == maxZoneCnt)
        {
          m_signalNumber = SIG_2_MIDDLE;
          str = "Middle";
        }
        else
        { //if can't see the other two, it must be left
          m_signalNumber = SIG_3_RIGHT;
          str = "Right";
        }

        //Adding text to the image 
        int scale = 1;
        int thickness = 2;
        Imgproc.putText(m_mat, str, new Point(120, 40), Imgproc.FONT_HERSHEY_SIMPLEX, scale, RED, thickness);

        if (telemetry != null) {
            telemetry.addData("Signal number", m_signalNumber);
            telemetry.update();
            telemetry = null;
        }
        if (edges != null) {
            edges.release();
            edges = null;
        }
        if (contours != null) {
            contours = null;
        }
        if (contoursPoly != null) contoursPoly = null;
        if (hierarchy != null) {
            hierarchy.release();
            hierarchy = null;
        }
        if (thresh != null) {
            thresh.release();
            thresh = null;
        }
 
        return m_mat;
    }
    private int getAreaAfterFilter(Rect rect, int verticalFilterLine){
      int retRectArea = 0;
      if (rect.y >= verticalFilterLine){
        retRectArea = rect.width * rect.height;        
      }
      else {
        if (rect.y + rect.height < verticalFilterLine) {
            retRectArea = 0;
        }
        else {
           retRectArea = rect.width * (rect.y + rect.height - verticalFilterLine);
        }
      }
        
      return retRectArea;
      
    }
    public int getSignalNumber() {
        return this.m_signalNumber;
    }
    public String getSignalString() {
        if (this.m_signalNumber == SIG_1_LEFT) return "SIG_1_LEFT";
        else if (this.m_signalNumber == SIG_2_MIDDLE) return "SIG_2_MIDDLE";
        else if (this.m_signalNumber == SIG_3_RIGHT) return "SIG_3_RIGHT";
        else return ("");
    }
}