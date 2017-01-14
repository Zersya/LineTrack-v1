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
    // Color radius for range checking in HSV color space
    private Scalar mColorRadius = new Scalar(25, 50, 50, 0);
    private Mat mSpectrum = new Mat();
    // Minimum contour area in percent for contours filtering
    private static double mMinContourArea = 0.1;
    //Variable untuk mencari Contour warna
    private List<MatOfPoint> mContour = new ArrayList<MatOfPoint>();
    //Variable untuk menyimpan titik Tengah Warna
    private Point centerMoment = new Point();
    //Variable untuk menyimpan TopLeft dan BottomRight dari Drawing Box
    private Rect bounding_rect = null;
    private int rotatedBox[] = new int[3];

    private boolean ambil = false;

    private int largestContourIndex = 0;


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

//        mLowerBound = new Scalar(10.234, 173.218, 205.0, 0.0);
//        mUpperBound = new Scalar(60.234, 273.218, 305.0, 255);

//        Imgproc.blur(mRgba, mRgba, new Size(1, 1));
        Rect roi = new Rect(new Point(30, mRgba.rows() / 2 - 130), new Point(80, mRgba.rows() / 2 + 130));
        Mat output = new Mat(mRgba, roi);
        Imgproc.rectangle(mRgba, new Point(30, mRgba.rows() / 2 - 140), new Point(80, mRgba.rows() / 2 + 140), new Scalar(255, 255, 0, 255), 1);
        Imgproc.cvtColor(output, mHsvMat, Imgproc.COLOR_RGB2HSV_FULL);
        Core.inRange(mHsvMat, mLowerBound, mUpperBound, mMask);
        Imgproc.dilate(mMask, mDillateMask, new Mat());
        mFinal = output;
        ProcContour();
    }

    public void ProcContour() {
        int largestArea = 0;

        Mat mHierarchy = new Mat();
        Imgproc.findContours(mDillateMask, mContour, mHierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        List<Moments> mu = new ArrayList<Moments>(getContours().size());

        RotatedRect box = null;
        double contourAngle = 0;
        int positionContour = 0;
        MatOfPoint2f allPoints = new MatOfPoint2f();


        int x = 0, y = 0;
        if (mContour.size() > 0) {
            for (int i = 0; i < mContour.size(); i++) {
//
//                mu.add(i, Imgproc.moments(getContours().get(i), false));
//                Moments p = mu.get(i);
//
//                x = (int) (p.get_m10() / p.get_m00());
//                y = (int) (p.get_m01() / p.get_m00());
                double a = Imgproc.contourArea(getContours().get(i), false);

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

            if (box.size.width < box.size.height) {
                contourAngle = 90 + contourAngle;
            }

            Point[] points = new Point[4];
            box.points(points);
            for (int j = 0; j < 4; j++) {
                Imgproc.line(mFinal, points[j], points[(j + 1) % 4], new Scalar(255, 0, 150, 255), 2);

//            Mat mapMatrix = Imgproc.getRotationMatrix2D(new Point(x, y), contourAngle, 1.0);

//            System.out.println("Contour Tinggi : " + (int) box.size.height);
//            System.out.println("Contour Lebar  : " + (int) box.size.width);
//            System.out.println("Contour angle  : " + (int) contourAngle);
//            Core.rectangle(mFinal, box.boundingRect().tl(), box.boundingRect().br(), new Scalar(255, 0, 150, 255), 1);

            }

            rotatedBox[0] = box.boundingRect().height;
            rotatedBox[1] = (int) box.size.height;
            rotatedBox[2] = (int) contourAngle;
        }

        if (mContour.isEmpty()) {
            centerMoment.x = -1;
            centerMoment.y = -1;
            x = (int) centerMoment.x;
            y = (int) centerMoment.y;
        }
        try {
//            Imgproc.circle(mFinal, centerMoment, 2, new Scalar(0, 250, 150, 255), 2);
//            Core.rectangle(mFinal, bounding_rect.tl(), bounding_rect.br(), new Scalar(255, 0, 150, 255), 1);
            Imgproc.drawContours(mFinal, getContours(), largestContourIndex, new Scalar(255, 0, 255, 255),2);

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

    public void samainWarna(Mat input) {

        for (int y = 0; y < input.cols(); y++) {
            for (int x = 0; x < input.rows(); x++) {
                if (y >= 20 && y <= 50) {
                    if (x >= input.rows() / 2 - 30 && x <= input.rows() / 2 + 30) {

                    }
                }
            }
        }

        double[] rgb = input.get(input.cols() / 2, input.rows() / 2);

        System.out.println("color : " + rgb[0] + ":" + rgb[1] + ":" + rgb[2]);

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
        return mDillateMask;
    }

    public List<MatOfPoint> getContours() {
        return mContour;
    }

    public Scalar getmLowerBound() {
        return mLowerBound;
    }

    public Scalar getmUpperBound() {
        return mUpperBound;
    }

    public int[] getRotatedBox() {
        return rotatedBox;
    }

    public int getLargestContourIndex() {
        return largestContourIndex;
    }
}
