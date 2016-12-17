package roboticsas.linetrack;

import android.widget.Toast;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Iterator;
import java.util.List;

/**
 * Created by zersya on 10/5/16.
 */

public class TrackLine {

    //Variable Wadah buat Gambar
    private Mat mMask = new Mat();
    private Mat mDillateMask = new Mat();
    private Mat mHsvMat = new Mat();
    private Mat mFinal = new Mat();
    //Variable untuk Pengaturan/Pemilihan Warna
    private Scalar mLowerBound = new Scalar(0);
    private Scalar mUpperBound = new Scalar(0);
    //Variable untuk mencari Contour warna
    private List<MatOfPoint> mContour = new ArrayList<MatOfPoint>();
    //Variable untuk menyimpan titik Tengah Warna
    private Point centerMoment = new Point();
    //Variable untuk menyimpan TopLeft dan BottomRight dari Drawing Box
    private Rect bounding_rect = null;
    private int thresh = 50, N = 11;


    public void setScalar(int lowerH, int lowerS, int lowerV, int upperH, int upperS, int upperV){

        mLowerBound = new Scalar(0, 0, 173);
        mUpperBound = new Scalar(215, 224, 255);
//        mLowerBound = new Scalar(lowerH, lowerS, lowerV);
//        mUpperBound = new Scalar(upperH, upperS, upperV);
    }

    public void ProcThresh(Mat mRgba){
        Imgproc.blur(mRgba, mRgba, new Size(8, 8));
        Imgproc.cvtColor(mRgba, mHsvMat, Imgproc.COLOR_RGB2HSV_FULL);
        Core.inRange(mHsvMat, mLowerBound, mUpperBound, mMask);
        Imgproc.dilate(mMask, mDillateMask, new Mat());
        mFinal = mRgba;
        ProcContour();
    }
    public void ProcContour(){
        int largestArea = 0;
        int largestContourIndex = 0;

        Mat mHierarchy = new Mat();
        Imgproc.findContours(mDillateMask, mContour, mHierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        List<MatOfPoint2f> mc = new ArrayList<MatOfPoint2f>(getContours().size());
        List<Moments> mu = new ArrayList<Moments>(getContours().size());
        int x = 0, y = 0;

        for(int i = 0; i < mContour.size(); i++){

            mu.add(i, Imgproc.moments(getContours().get(i), false));
            Moments p = mu.get(i);

            x = (int) (p.get_m10() / p.get_m00());
            y = (int) (p.get_m01() / p.get_m00());
            double a = Imgproc.contourArea(getContours().get(i), false);
            if(a > largestArea){
                largestArea = (int) a;
                largestContourIndex = i;
                bounding_rect = Imgproc.boundingRect(mContour.get(i));
            }
        }
        centerMoment.x = x;
        centerMoment.y = y;
        try {
            Imgproc.circle(mFinal, centerMoment, 2, new Scalar(255, 0, 150, 255), 2);

//            Core.rectangle(mFinal, bounding_rect.tl(), bounding_rect.br(), new Scalar(255, 0, 150, 255), 1);
//            Imgproc.drawContours(mFinal, getContours(), largestContourIndex, new Scalar(255, 0, 0, 255), 2);

            mContour.clear();
        }catch(Exception e) {
            System.out.println("Error " + e);
        }
    }

    public Mat findRectangle(Mat img, List<MatOfPoint> square){
        Mat edge = new Mat();
        Mat hierarchy = new Mat();

        Imgproc.cvtColor(img, edge, Imgproc.COLOR_BGR2GRAY);
        Imgproc.GaussianBlur(edge,edge, new Size(5,5),0);
        Imgproc.Canny(edge, edge, 100, 160);

        List<MatOfPoint> _contour = new ArrayList<MatOfPoint>();
        Imgproc.findContours(edge, _contour, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        double max_area = 0;
        double peri;
        MatOfPoint approx = new MatOfPoint();
        MatOfPoint biggest = new MatOfPoint();

        for(int i = 0; i < _contour.size(); i++){
            double area = Imgproc.contourArea(_contour.get(i), false);

            if(area > 50) {
                peri = Imgproc.arcLength(new MatOfPoint2f(_contour.get(i).toArray()), true);
                approx = approxPolyDP(_contour.get(i), peri * 0.03, true);
                if(area > max_area && approx.toArray().length == 4){
                    biggest = approx;
                    max_area = area;
                    square.add(approx);
                }
            }
        }

        return edge;

    }

    public void findSquare(Mat img, List<MatOfPoint> squares){
        squares.clear();

        Mat smallerImg = new Mat(new Size(img.width()/2, img.height()/2), img.type());
        Mat gray = new Mat(img.size(), img.type());
        Mat gray0 = new Mat(img.size(), CvType.CV_8U);

        Imgproc.pyrDown(img,smallerImg,smallerImg.size());
        Imgproc.pyrUp(smallerImg,img,img.size());

        for(int c = 0; c < 3; c++){
            extractChannel(img, gray,c);

            for(int l = 1; l < N; l++){
                Imgproc.threshold(gray,gray0,(l+1)*255/N, 255, Imgproc.THRESH_BINARY);
                List<MatOfPoint> contour = new ArrayList<MatOfPoint>();

                Imgproc.findContours(gray0, contour, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

                MatOfPoint approx = new MatOfPoint();

                for(int i = 0; i < contour.size(); i++){
                    approx = approxPolyDP(contour.get(i), Imgproc.arcLength(new MatOfPoint2f(contour.get(i).toArray()), true)*0.02, true);

                    if(approx.toArray().length == 4 &&
                            Math.abs(Imgproc.contourArea(approx)) > 1000 &&
                            Imgproc.isContourConvex(approx)){

                        double maxCosine = 0;

                        for(int j = 2; j < 5; j++){

                            double cosine = Math.abs(angle(approx.toArray()[j%4], approx.toArray()[j-2], approx.toArray()[j-1]));
                            maxCosine = Math.max(maxCosine,cosine);
                        }

                        if(maxCosine < 0.3){
                            squares.add(approx);
                        }
                    }
                }
            }
        }
    }

    public void drawContour(Mat img){
        List<MatOfPoint> squares = new ArrayList<MatOfPoint>();

        squares.clear();

        findRectangle(img, squares);
        Imgproc.drawContours(img,squares,-1,new Scalar(0,0,255), 1);

    }

    public void extractChannel(Mat source, Mat out, int channelNum){
        List<Mat> sourceChannel = new ArrayList<Mat>();
        List<Mat> outChannel = new ArrayList<Mat>();

        Core.split(source, sourceChannel);
        outChannel.add(new Mat(sourceChannel.get(0).size(), sourceChannel.get(0).type()));
        Core.mixChannels(sourceChannel, outChannel, new MatOfInt(channelNum,0));
        Core.merge(outChannel, out);
    }

    public MatOfPoint approxPolyDP(MatOfPoint curve, double epsilon, boolean closed){
        MatOfPoint2f tempMat = new MatOfPoint2f();

        Imgproc.approxPolyDP(new MatOfPoint2f(curve.toArray()), tempMat, epsilon, closed);

        return new MatOfPoint(tempMat.toArray());
    }

    public static double angle( Point pt1, Point pt2, Point pt0 )
    {
        double dx1 = pt1.x - pt0.x;
        double dy1 = pt1.y - pt0.y;
        double dx2 = pt2.x - pt0.x;
        double dy2 = pt2.y - pt0.y;

        return (dx1*dx2 + dy1*dy2)/Math.sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
    }

    public Rect getBounding_rect() { return bounding_rect; }
    public Point getCenterMoment() { return centerMoment ; }
    public Mat getDillate(){ return mDillateMask; }
    public List<MatOfPoint> getContours(){
        return mContour;
    }


}
