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
    private Mat mFinal = new Mat();
    private Mat BFinal = new Mat();
    private Mat mADillateMask = new Mat();
    private Mat mBDillateMask = new Mat();
    //Variable untuk Pengaturan/Pemilihan Warna
    private Scalar mLowerBound = new Scalar(0);
    private Scalar mUpperBound = new Scalar(0);
    // Color radius for range checking in HSV color space
    private Scalar mColorRadius = new Scalar(25, 50, 50, 0);
    private Mat mSpectrum = new Mat();
    // Minimum contour area in percent for contours filtering
    private static double mMinContourArea = 0.1;
    //Variable untuk mencari Contour warna
    private List<MatOfPoint> mContour = new ArrayList<MatOfPoint>();
    private List<MatOfPoint> BContour = new ArrayList<MatOfPoint>();
    //Variable untuk menyimpan titik Tengah Warna
    private Point centerMoment = new Point();
    private Point centerCamera = new Point();
    //Variable untuk menyimpan TopLeft dan BottomRight dari Drawing Box
    private Rect bounding_rect = null;

    private int ArotatedBox[] = new int[3];
    private int BrotatedBox[] = new int[3];
    private int Time = 0;

    private RotatedRect box = null;
    private boolean ambil = false;
    private boolean aktif = false;
    private boolean hitung = false;
    //Use ROI
    private Rect Aroi;
    private Rect Broi;
    private Mat Aoutput;
    private Mat Boutput;
    private Thread count;


    public void setColorRadius(Scalar radius) {
        mColorRadius = radius;
    }

    public void setHsvColor(Scalar hsvColor) {
        double minH = (hsvColor.val[0] >= mColorRadius.val[0]) ? hsvColor.val[0] - mColorRadius.val[0] : 0;
        double maxH = (hsvColor.val[0] + mColorRadius.val[0] <= 255) ? hsvColor.val[0] + mColorRadius.val[0] : 255;

        mLowerBound.val[0] = minH;
        mUpperBound.val[0] = maxH;

        mLowerBound.val[1] = hsvColor.val[1] - mColorRadius.val[1];
        mUpperBound.val[1] = hsvColor.val[1] + mColorRadius.val[1];

        mLowerBound.val[2] = hsvColor.val[2] - mColorRadius.val[2];
        mUpperBound.val[2] = hsvColor.val[2] + mColorRadius.val[2];

        mLowerBound.val[3] = 0;
        mUpperBound.val[3] = 255;

        Mat spectrumHsv = new Mat(1, (int) (maxH - minH), CvType.CV_8UC3);

        for (int j = 0; j < maxH - minH; j++) {
            byte[] tmp = {(byte) (minH + j), (byte) 255, (byte) 255};
            spectrumHsv.put(0, j, tmp);
        }

        Imgproc.cvtColor(spectrumHsv, mSpectrum, Imgproc.COLOR_HSV2RGB_FULL, 4);
    }

    public Mat getSpectrum() {
        return mSpectrum;
    }

    public void setMinContourArea(double area) {
        mMinContourArea = area;
    }

    //Using Trackbar
    public void setScalar(int lowerH, int lowerS, int lowerV, int upperH, int upperS, int upperV) {


//        mLowerBound = new Scalar(lowerH, lowerS, lowerV);
//        mUpperBound = new Scalar(upperH, upperS, upperV);
    }

    public void ProcThresh(Mat mRgba) {

        //Ball Detection(Oranye)
//        mLowerBound = new Scalar(0, 167, 205, 0);
//        mUpperBound = new Scalar(35, 267, 305, 255);
//
//
//        mLowerBound = new Scalar(0.734, 205, 123, 0);
//        mUpperBound = new Scalar(50.734, 305, 223.5, 255);

//        mLowerBound = new Scalar(4.128, 153.5, 205.0, 0.0);
//        mUpperBound = new Scalar(54.218, 253.5, 305.0, 255);
//
        mLowerBound = new Scalar(7.204, 186.348, 204.73, 0.0);
        mUpperBound = new Scalar(57.204, 286.348, 305.0, 255);

//        Imgproc.blur(mRgba, mRgba, new Size(1, 1));

        Mat mMask = new Mat();
        Mat mDillateMask = new Mat();
        Mat mHsvMat = new Mat();

        Aroi = new Rect(new Point(30, mRgba.rows() / 2 - 230), new Point(200, mRgba.rows() / 2 + 230));
        Aoutput = new Mat(mRgba, Aroi);

        centerCamera = new Point(Aoutput.cols() / 2, Aoutput.rows() / 2);

        Core.rectangle(mRgba, new Point(30, mRgba.rows() / 2 - 230), new Point(200, mRgba.rows() / 2 + 230), new Scalar(0, 100, 255, 255), 1);

        Imgproc.cvtColor(Aoutput, mHsvMat, Imgproc.COLOR_RGB2HSV_FULL);
        Core.inRange(mHsvMat, mLowerBound, mUpperBound, mMask);
        Imgproc.dilate(mMask, mADillateMask, new Mat());

        mFinal = Aoutput;

        Core.circle(mFinal, centerCamera, 3, new Scalar(255, 205, 0, 255), 2);
        ProcContour();
    }

    public void ProcThresh2(Mat mRgba) {

        int largestArea = 0;
        Mat mMask = new Mat();
        Mat mHsvMat = new Mat();
        if (aktif) {
            Broi = new Rect(new Point(mRgba.cols() - 30, mRgba.rows() / 2 - 180), new Point(mRgba.cols() - 80, mRgba.rows() / 2 + 180));
            Boutput = new Mat(mRgba, Broi);
            Core.rectangle(mRgba, new Point(mRgba.cols() - 30, mRgba.rows() / 2 - 180), new Point(mRgba.cols() - 80, mRgba.rows() / 2 + 180), new Scalar(0, 100, 255, 255), 1);

            Imgproc.cvtColor(Boutput, mHsvMat, Imgproc.COLOR_RGB2HSV_FULL);
            Core.inRange(mHsvMat, mLowerBound, mUpperBound, mMask);
            Imgproc.dilate(mMask, mBDillateMask, new Mat());
            BFinal = Boutput;

            Imgproc.findContours(mBDillateMask, BContour, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            RotatedRect box = null;
            double contourAngle = 0;
            MatOfPoint2f allPoints = new MatOfPoint2f();
            int largestContourIndex = 0;

            if (BContour.size() > 0) {
                for (int i = 0; i < BContour.size(); i++) {
                    double a = Imgproc.contourArea(BContour.get(i), false);

                    if (a > largestArea) {
                        largestArea = (int) a;
                        MatOfPoint2f points = new MatOfPoint2f(BContour.get(i).toArray());
                        allPoints.push_back(points);

                        box = Imgproc.minAreaRect(points);

                        contourAngle = box.angle;
                    }
                }
                if (box != null) {
                    if (box.size.width < box.size.height) {
                        contourAngle = 90 + contourAngle;
                    }

                    Point[] points = new Point[4];
                    box.points(points);
                    for (int j = 0; j < 4; j++) {
                        Core.line(BFinal, points[j], points[(j + 1) % 4], new Scalar(255, 0, 150, 255), 2);

//            Mat mapMatrix = Imgproc.getRotationMatrix2D(new Point(x, y), contourAngle, 1.0);
//
                    }

                    BrotatedBox[0] = box.boundingRect().height;
                    BrotatedBox[1] = (int) box.size.height;
                    BrotatedBox[2] = (int) contourAngle;
                }
            } else {
                BrotatedBox[0] = 0;
                BrotatedBox[2] = 0;
                aktif = false;
            }

            BContour.clear();
        }
    }

    public void ProcContour() {
        int largestArea = 0;

        Mat mHierarchy = new Mat();
        Imgproc.findContours(mADillateMask, mContour, mHierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        List<Moments> mu = new ArrayList<Moments>(mContour.size());

        double contourAngle = 0;
        int positionContour = 0;
        MatOfPoint2f allPoints = new MatOfPoint2f();

        int x = 0, y = 0;

        if (mContour.size() > 0) {
            for (int i = 0; i < mContour.size(); i++) {
//Cari titikTengah
                mu.add(i, Imgproc.moments(mContour.get(i), false));
                Moments p = mu.get(i);

                x = (int) (p.get_m10() / p.get_m00());
                y = (int) (p.get_m01() / p.get_m00());
                double a = Imgproc.contourArea(mContour.get(i), false);
                int largestContourIndex = 0;

                if (a > largestArea) {
                    largestArea = (int) a;
                    largestContourIndex = i;
                    bounding_rect = Imgproc.boundingRect(mContour.get(i));
                    centerMoment.x = x;
                    centerMoment.y = y;

                    MatOfPoint2f points = new MatOfPoint2f(mContour.get(i).toArray());
                    allPoints.push_back(points);

                    box = Imgproc.minAreaRect(points);

                    contourAngle = box.angle;

                }
            }
            if (box != null) {
                if (box.size.width < box.size.height) {
                    contourAngle = 90 + contourAngle;
                }

                Point[] points = new Point[4];
                box.points(points);
                for (int j = 0; j < 4; j++) {
                    Core.line(mFinal, points[j], points[(j + 1) % 4], new Scalar(255, 0, 150, 255), 2);
                }

                ArotatedBox[0] = box.boundingRect().height;
                ArotatedBox[1] = (int) box.size.height;
                ArotatedBox[2] = (int) contourAngle;
                Time = 0;
            }
        } else {
            Time += 1;
            if (Time == 50) {
                centerMoment.y = Aoutput.rows()/2;
                centerMoment.x = Aoutput.cols()/2;

            }
            ArotatedBox[2] = 9999;
        }

        try

        {
            Core.circle(mFinal, centerMoment, 2, new Scalar(0, 250, 150, 255), 2);
//            Core.rectangle(mFinal, bounding_rect.tl(), bounding_rect.br(), new Scalar(255, 0, 150, 255), 1);
//            Imgproc.drawContours(mFinal, getContours(), largestContourIndex, new Scalar(255, 0, 255, 255), 2);

            mContour.clear();
        } catch (Exception e) {
            System.out.println("Error " + e);
        }

    }

    public void findRectangle(Mat img, List<MatOfPoint> square) {
        Mat edge = new Mat();
        Mat hierarchy = new Mat();

        Imgproc.cvtColor(img, edge, Imgproc.COLOR_BGR2GRAY);
        Imgproc.GaussianBlur(edge, edge, new Size(5, 5), 0);
        Imgproc.Canny(edge, edge, 100, 160);

        List<MatOfPoint> _contour = new ArrayList<MatOfPoint>();
        Imgproc.findContours(edge, _contour, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        double max_area = 0;
        double peri;
        MatOfPoint approx = new MatOfPoint();
        MatOfPoint biggest = new MatOfPoint();

        for (int i = 0; i < _contour.size(); i++) {
            double area = Imgproc.contourArea(_contour.get(i), false);

            if (area > 150) {
                peri = Imgproc.arcLength(new MatOfPoint2f(_contour.get(i).toArray()), true);
                approx = approxPolyDP(_contour.get(i), peri * 0.03, true);
                if (area > max_area && approx.toArray().length == 4) {
                    biggest = approx;
                    max_area = area;
                    square.add(approx);
                    ambil = true;
                }
            } else {
                ambil = false;
            }
        }


    }

    public void drawContour(Mat img) {
        List<MatOfPoint> squares = new ArrayList<MatOfPoint>();

        findRectangle(img, squares);
        if (ambil) {
            Imgproc.drawContours(img, squares, -1, new Scalar(0, 250, 150, 255), 2);
            squares.clear();
        }


    }

    public MatOfPoint approxPolyDP(MatOfPoint curve, double epsilon, boolean closed) {
        MatOfPoint2f tempMat = new MatOfPoint2f();

        Imgproc.approxPolyDP(new MatOfPoint2f(curve.toArray()), tempMat, epsilon, closed);

        return new MatOfPoint(tempMat.toArray());
    }

    public Rect getBounding_rect() {
        return bounding_rect;
    }

    public Point getCenterMoment() {
        return centerMoment;
    }

    public Mat getDillate() {
        return mADillateMask;
    }

    public List<MatOfPoint> getAContours() {
        return mContour;
    }

    public Scalar getmLowerBound() {
        return mLowerBound;
    }

    public Scalar getmUpperBound() {
        return mUpperBound;
    }

    public int[] getRotatedBox() {
        return ArotatedBox;
    }

    public boolean getAktiv() {
        return aktif;
    }

    public Point getCenterCamera() {
        return centerCamera;
    }

    public Thread getCount() {
        return count;
    }
}
