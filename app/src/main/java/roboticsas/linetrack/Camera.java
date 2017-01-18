package roboticsas.linetrack;

import android.app.PendingIntent;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.pm.PackageManager;
import android.graphics.Color;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbManager;
import android.os.Build;
import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.view.MotionEvent;
import android.view.SurfaceView;
import android.view.View;
import android.view.View.OnTouchListener;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.ScrollView;
import android.widget.SeekBar;
import android.widget.TabHost;
import android.widget.TextView;
import android.widget.Toast;

import com.felhr.usbserial.UsbSerialDevice;
import com.felhr.usbserial.UsbSerialInterface;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.JavaCameraView;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
//import org.opencv.highgui.Highgui;
import org.opencv.imgproc.Imgproc;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.TimeUnit;

public class Camera extends AppCompatActivity implements OnTouchListener, CameraBridgeViewBase.CvCameraViewListener2, SensorEventListener {


    //Init Method
    private static final String TAG = "OCVSample::Activity";
    public final String ACTION_USB_PERMISSION = "roboticsas.linetrack.USB_PERMISSION";
    private CameraBridgeViewBase mOpenCVCameraView;


    private SeekBar lowerH, lowerS, lowerV, upperH, upperS, upperV;
    private TextView tlowerH, tlowerS, tlowerV, tupperH, tupperS, tupperV, ScalarT, Status, SensorT;
    private Button btnOpen, btnClose;
    private ScrollView scrollView;
    private int tlH, tlS, tlV, tuH, tuS, tuV, vId;
    private float accelX, accelY, accelZ;
    private boolean stat = false;
    private int Time = 0, State = 0;
    private boolean mIsColorSelected = false;
    private Mat mSpectrum;
    private Size SPECTRUM_SIZE;
    private Scalar mBlobColorRgba;
    private Scalar mBlobColorHsv;

    //Init Attribute Citra
    private Mat mRgba;
    private Mat mHsv;
    private TrackLine trackLine;
    private SensorManager mSensorManager;
    private Sensor mSensor;
    UsbManager usbManager;
    UsbDevice device;
    UsbSerialDevice serialPort;
    UsbDeviceConnection connection;

    public String getAndroidVersion() {
        String release = Build.VERSION.RELEASE;
        int sdkVersion = Build.VERSION.SDK_INT;

        return ("Android SDK : " + sdkVersion + "(" + release + ")");
    }

    private final BroadcastReceiver broadcastReceiver = new BroadcastReceiver() { //Broadcast Receiver to automatically start and stop the Serial connection.
        @Override
        public void onReceive(Context context, Intent intent) {
            if (intent.getAction().equals(ACTION_USB_PERMISSION)) {
                Status.setText("get USB Permission");
                boolean granted = intent.getExtras().getBoolean(UsbManager.EXTRA_PERMISSION_GRANTED);
                if (granted) {
                    connection = usbManager.openDevice(device);
                    serialPort = UsbSerialDevice.createUsbSerialDevice(device, connection);
                    if (serialPort != null) {
                        if (serialPort.open()) { //Set Serial Connection Parameters.
                            Status.setText("Connected");
                            stat = true;
//                            t.start();
                            serialPort.setBaudRate(9600);
                            serialPort.setDataBits(UsbSerialInterface.DATA_BITS_8);
                            serialPort.setStopBits(UsbSerialInterface.STOP_BITS_1);
                            serialPort.setParity(UsbSerialInterface.PARITY_NONE);
                            serialPort.setFlowControl(UsbSerialInterface.FLOW_CONTROL_OFF);


                        } else {
                            Log.d("SERIAL", "PORT NOT OPEN");
                        }
                    } else {
                        Log.d("SERIAL", "PORT IS NULL");
                    }
                } else {
                    Log.d("SERIAL", "PERM NOT GRANTED");
                }
            } else if (intent.getAction().equals(UsbManager.ACTION_USB_DEVICE_ATTACHED)) {
                onClickStart(btnOpen);
            } else if (intent.getAction().equals(UsbManager.ACTION_USB_DEVICE_DETACHED)) {
                onClickStop(btnClose);

            }
        }

        ;
    };
    private BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status) {

            switch (status) {
                case LoaderCallbackInterface.SUCCESS: {
                    Log.i(TAG, "OpenCV Loaded Succesfuly");
                    mOpenCVCameraView.enableView();
                    mOpenCVCameraView.setOnTouchListener((OnTouchListener) Camera.this);
                }
                break;
                default: {
                    super.onManagerConnected(status);
                }
                break;
            }
        }
    };

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_camera);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        getAndroidVersion();

        TabHost tabHost = (TabHost) findViewById(R.id.tabHost);
        tabHost.setup();
        //Tab 1
        TabHost.TabSpec spec1 = tabHost.newTabSpec("Camera");
        spec1.setContent(R.id.tab1);
        spec1.setIndicator("Camera");
        tabHost.addTab(spec1);
//        //Tab 2
        TabHost.TabSpec spec2 = tabHost.newTabSpec("Connection");
        spec2.setContent(R.id.tab2);
        spec2.setIndicator("Connection");
        tabHost.addTab(spec2);

        btnOpen = (Button) findViewById(R.id.btnOpen);
        btnOpen.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {

                onClickStart(view);

            }
        });
        btnClose = (Button) findViewById(R.id.btnClose);
        btnClose.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                onClickStop(view);
                stat = false;
                System.exit(0);
            }
        });

        Status = (TextView) findViewById(R.id.Status);
        SensorT = (TextView) findViewById(R.id.SensorT);
        scrollView = (ScrollView) findViewById(R.id.sv);

        Status.setTextColor(Color.WHITE);
        SensorT.setTextColor(Color.WHITE);

        PackageManager pm = this.getPackageManager();
        boolean accel = pm.hasSystemFeature(PackageManager.FEATURE_SENSOR_ACCELEROMETER);
        boolean compas = pm.hasSystemFeature(PackageManager.FEATURE_SENSOR_COMPASS);
        if (accel) {
            Toast.makeText(getApplicationContext(), "Ada Sensor Accel", Toast.LENGTH_SHORT).show();
        } else if (compas && accel) {
            Toast.makeText(getApplicationContext(), "Ada Sensor Compas dan Accel", Toast.LENGTH_LONG).show();
        }
        mSensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);
        mSensor = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        usbManager = (UsbManager) getSystemService(this.USB_SERVICE);

        IntentFilter filter = new IntentFilter();
        filter.addAction(ACTION_USB_PERMISSION);
        filter.addAction(UsbManager.ACTION_USB_DEVICE_ATTACHED);
        filter.addAction(UsbManager.ACTION_USB_DEVICE_DETACHED);
        registerReceiver(broadcastReceiver, filter);

        mOpenCVCameraView = (JavaCameraView) findViewById(R.id.show_camera_activity_java_surface_view);

        mOpenCVCameraView.setMaxFrameSize(500, 500);
        mOpenCVCameraView.setVisibility(SurfaceView.VISIBLE);
        mOpenCVCameraView.setCvCameraViewListener(this);

//        new Thread() {
//            @Override
//            public void run() {
//                while (true) {
//                    if (connection != null) {
//                        serialPort.write(sendData().getBytes());
//                    }
//
//                    tvAppend(Status, "\nData Arduino : " + sendData());
//
//                    scrollView.post(new Runnable() {
//                        @Override
//                        public void run() {
//                            scrollView.fullScroll(View.FOCUS_DOWN);
//                        }
//                    });
//
//                    try {
//                        sleep(20);
//                    } catch (InterruptedException e) {
//                    }
//                }
//            }
//        }.start();


    }


    public void onClickStart(View view) {

        try {
            HashMap<String, UsbDevice> usbDevices = usbManager.getDeviceList();
            if (!usbDevices.isEmpty()) {
                boolean keep = true;
                for (Map.Entry<String, UsbDevice> entry : usbDevices.entrySet()) {
                    device = entry.getValue();
                    int deviceVID = device.getVendorId();

                    if (deviceVID == 0x1A86)//Arduino Vendor ID
                    {
                        PendingIntent pi = PendingIntent.getBroadcast(this, 0, new Intent(ACTION_USB_PERMISSION), 0);
                        usbManager.requestPermission(device, pi);
                        keep = false;

                    } else {
                        connection = null;
                        device = null;
                    }

                    if (!keep)
                        break;
                }


            }
        } catch (Exception e) {

        }

    }


    public void onClickStop(View view) {
        try {
            serialPort.close();
            tvAppend(Status, "\nSerial Connection Closed! \n");
        } catch (Exception e) {

        }
    }


    public String sendData() {
        try {
//            String data = (int) trackLine.getCenterMoment().y + ":" + (int) trackLine.getCenterMoment().x + "\n";
//            String data = kondisiDataBola();
            String data = kondisiDataGaris();

            return data;
        } catch (Exception e) {
            return null;
        }
    }

    private void tvAppend(TextView tv, CharSequence text) {
        final TextView ftv = tv;
        final CharSequence ftext = text;

        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                ftv.append(ftext);
            }
        });
    }


    public void onPause() {
        super.onPause();
        if (mOpenCVCameraView != null) {
            mOpenCVCameraView.disableView();
        }
        mSensorManager.unregisterListener(this);
    }

    public void onResume() {
        super.onResume();

        if (!OpenCVLoader.initDebug()) {
            Log.d(TAG, "Internal OpenCV not found, Using OpenCV manager for Initialize");
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_2_4_13, this, mLoaderCallback);
        } else {
            Log.d(TAG, "OpenCV loader found inside The Package, Using it!!");
            mLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }
        mSensorManager.registerListener(this, mSensor, SensorManager.SENSOR_DELAY_NORMAL);

    }

    public void onDestroy() {
        super.onDestroy();

        if (mOpenCVCameraView != null) {
            mOpenCVCameraView.disableView();
        }
    }

    @Override
    public void onCameraViewStarted(int width, int height) {
        mRgba = new Mat(width, height, CvType.CV_8UC4);
        mHsv = new Mat();
        mSpectrum = new Mat();
        trackLine = new TrackLine();
        mBlobColorRgba = new Scalar(255);
        mBlobColorHsv = new Scalar(255);
        SPECTRUM_SIZE = new Size(200, 64);
    }

    @Override
    public void onCameraViewStopped() {
        mRgba.release();
        mHsv.release();
    }

    @Override
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {

        mRgba = inputFrame.rgba();


//        if (mIsColorSelected) {

            trackLine.ProcThresh(mRgba);

//            trackLine.drawContour(mRgba);
            if (connection != null) {
                serialPort.write(sendData().getBytes());
            }

            tvAppend(Status, "\nData Arduino : " + sendData());

            scrollView.post(new Runnable() {
                @Override
                public void run() {
                    scrollView.fullScroll(View.FOCUS_DOWN);
                }
            });

//        }

        Mat spectrumLabel = mRgba.submat(4, 4 + mSpectrum.rows(), 70, 70 + mSpectrum.cols());
        mSpectrum.copyTo(spectrumLabel);

        return mRgba;

    }


    @Override
    public void onSensorChanged(SensorEvent sensorEvent) {
        final float alpha = 0.8f;

        float gravity[] = new float[4];
        float linear_accel[] = new float[4];

        gravity[0] = alpha * gravity[0] + (1 - alpha) * sensorEvent.values[0];
        gravity[1] = alpha * gravity[1] + (1 - alpha) * sensorEvent.values[1];
        gravity[2] = alpha * gravity[2] + (1 - alpha) * sensorEvent.values[2];

        linear_accel[0] = sensorEvent.values[0] - gravity[0];
        linear_accel[1] = sensorEvent.values[1] - gravity[1];
        linear_accel[2] = sensorEvent.values[2] - gravity[2];
        accelX = linear_accel[0];
        accelY = linear_accel[1];
        accelZ = linear_accel[2];
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int i) {

    }

    @Override
    public boolean onTouch(View v, MotionEvent event) {

        int cols = mRgba.cols();
        int rows = mRgba.rows();

        int xOffset = (mOpenCVCameraView.getWidth() - cols) / 2;
        int yOffset = (mOpenCVCameraView.getHeight() - rows) / 2;

        int x = (int) event.getX() - xOffset;
        int y = (int) event.getY() - yOffset;

        Log.i(TAG, "Touch image coordinates: (" + x + ", " + y + ")");

        if ((x < 0) || (y < 0) || (x > cols) || (y > rows)) return false;

        Rect touchedRect = new Rect();

        touchedRect.x = (x > 4) ? x - 4 : 0;
        touchedRect.y = (y > 4) ? y - 4 : 0;

        touchedRect.width = (x + 4 < cols) ? x + 4 - touchedRect.x : cols - touchedRect.x;
        touchedRect.height = (y + 4 < rows) ? y + 4 - touchedRect.y : rows - touchedRect.y;

        Mat touchedRegionRgba = mRgba.submat(touchedRect);

        Mat touchedRegionHsv = new Mat();
        Imgproc.cvtColor(touchedRegionRgba, touchedRegionHsv, Imgproc.COLOR_RGB2HSV_FULL);

        // Calculate average color of touched region
        mBlobColorHsv = Core.sumElems(touchedRegionHsv);
        int pointCount = touchedRect.width * touchedRect.height;
        for (int i = 0; i < mBlobColorHsv.val.length; i++)
            mBlobColorHsv.val[i] /= pointCount;

        mBlobColorRgba = converScalarHsv2Rgba(mBlobColorHsv);

        Log.i(TAG, "Touched rgba color: (" + mBlobColorRgba.val[0] + ", " + mBlobColorRgba.val[1] +
                ", " + mBlobColorRgba.val[2] + ", " + mBlobColorRgba.val[3] + ")");

        trackLine.setHsvColor(mBlobColorHsv);

        SensorT.setText("Lower Mask : " + trackLine.getmLowerBound().val[0] + "," + trackLine.getmLowerBound().val[1] + "," + trackLine.getmLowerBound().val[2] + "," + trackLine.getmLowerBound().val[3] + "\n" +
                "Upper Mask : " + trackLine.getmUpperBound().val[0] + "," + trackLine.getmUpperBound().val[1] + "," + trackLine.getmUpperBound().val[2] + "," + trackLine.getmUpperBound().val[3]);

        Imgproc.resize(trackLine.getSpectrum(), mSpectrum, SPECTRUM_SIZE);

        mIsColorSelected = true;

        touchedRegionRgba.release();
        touchedRegionHsv.release();

        return false;
    }

    private Scalar converScalarHsv2Rgba(Scalar hsvColor) {
        Mat pointMatRgba = new Mat();
        Mat pointMatHsv = new Mat(1, 1, CvType.CV_8UC3, hsvColor);
        Imgproc.cvtColor(pointMatHsv, pointMatRgba, Imgproc.COLOR_HSV2RGB_FULL, 4);

        return new Scalar(pointMatRgba.get(0, 0));
    }

    private String kondisiDataBola() {
        String dataSend = " ";

        double x = trackLine.getCenterMoment().y;
        double y = trackLine.getCenterMoment().x;

        if (x >= 1 && x <= 55) {
            dataSend = "f";
        } else if (x >= 51 && x <= 105) {
            dataSend = "e";
        } else if (x >= 106 && x <= 160) {
            dataSend = "d";
        } else if (x >= 426 && x <= 480) {
            dataSend = "a";
        } else if (x >= 371 && x <= 425) {
            dataSend = "b";
        } else if (x >= 315 && x <= 370) {
            dataSend = "c";
        }
        //Mid
        else if (x >= 160 && x <= 315 && y >= 551 && y <= 690) {
            dataSend = "1";
        } else if (x >= 160 && x <= 315 && y >= 384 && y <= 550) {
            dataSend = "2";
        } else if (x >= 160 && x <= 315 && y >= 256 && y <= 383) {
            dataSend = "3";
        } else if (x >= 160 && x <= 315 && y >= 129 && y <= 255) {
            dataSend = "4";
        } else if (x >= 160 && x <= 315 && y >= 1 && y <= 128) {
            dataSend = "5";
        } else if (trackLine.getAContours().isEmpty()) {
            dataSend = " ";
        }


        return dataSend;

    }

    private String kondisiDataGaris() {

        String mt = " ";
        String nt = " ";

        int x = (int) trackLine.getCenterMoment().y;
        int y = (int) trackLine.getCenterMoment().x;
        int errorCenter = 0;
        int lebarJalur = 0;
        int tinggiJalur = 0;
        int rotasiJalur = 0;
        if (trackLine.getRotatedBox() != null) {
            lebarJalur = trackLine.getRotatedBox()[0];
            tinggiJalur = trackLine.getRotatedBox()[1];
            rotasiJalur = trackLine.getRotatedBox()[2];
            errorCenter = (int) trackLine.getCenterCamera().y - x;
        }

        mt = String.valueOf(errorCenter);
        nt = String.valueOf(rotasiJalur);


        return mt + ":" + nt + "\n";
    }

}
