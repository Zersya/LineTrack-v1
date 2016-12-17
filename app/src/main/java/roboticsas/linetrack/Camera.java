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
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.MotionEvent;
import android.view.SurfaceView;
import android.view.View;
import android.view.View.OnTouchListener;
import android.widget.Button;
import android.widget.SeekBar;
import android.widget.SeekBar.OnSeekBarChangeListener;
import android.widget.TabHost;
import android.widget.TabWidget;
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
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.io.UnsupportedEncodingException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class Camera extends AppCompatActivity implements OnSeekBarChangeListener, CameraBridgeViewBase.CvCameraViewListener2, SensorEventListener {


    //Init Method
    private static final String TAG = "OCVSample::Activity";
    public final String ACTION_USB_PERMISSION = "roboticsas.linetrack.USB_PERMISSION";
    private CameraBridgeViewBase mOpenCVCameraView;

    SeekBar lowerH,lowerS,lowerV,upperH,upperS,upperV;
    TextView tlowerH,tlowerS,tlowerV,tupperH,tupperS,tupperV, ScalarT, Status, SensorT;
    Button btnOpen, btnClose;
    int tlH,tlS,tlV,tuH,tuS,tuV;
    float accelX, accelY, accelZ;
    boolean stat = false;
    Camera camera;


    //Init Attribute Citra
    private Mat             mRgba;
    private Mat             mHsv;
    private TrackLine       trackLine;
    private Context cont;
    private SensorManager mSensorManager;
    private Sensor mSensor;
    UsbManager usbManager;
    UsbDevice device;
    UsbSerialDevice serialPort;
    UsbDeviceConnection connection;

    public String getAndroidVersion(){
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

        getAndroidVersion();

        TabHost tabHost = (TabHost) findViewById(R.id.tabHost);
        tabHost.setup();
        //Tab 1
        TabHost.TabSpec spec1 = tabHost.newTabSpec("Camera");
        spec1.setContent(R.id.tab1);
        spec1.setIndicator("Camera");
        tabHost.addTab(spec1);
        //Tab 2
        TabHost.TabSpec spec2 = tabHost.newTabSpec("Scalar");
        spec2.setContent(R.id.tab2);
        spec2.setIndicator("Scalar");
        tabHost.addTab(spec2);
//        //Tab 3
        TabHost.TabSpec spec3 = tabHost.newTabSpec("Connection");
        spec3.setContent(R.id.tab3);
        spec3.setIndicator("Connection");
        tabHost.addTab(spec3);

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
            }
        });

        ScalarT = (TextView) findViewById(R.id.Scalar);
        lowerH = (SeekBar) findViewById(R.id.barlowerH);
        tlowerH = (TextView) findViewById(R.id.lowerH);
        lowerS = (SeekBar) findViewById(R.id.barlowerS);
        tlowerS = (TextView) findViewById(R.id.lowerS);
        lowerV = (SeekBar) findViewById(R.id.barlowerV);
        tlowerV = (TextView) findViewById(R.id.lowerV);

        upperH = (SeekBar) findViewById(R.id.barupperH);
        tupperH = (TextView) findViewById(R.id.upperH);
        upperS = (SeekBar) findViewById(R.id.barupperS);
        tupperS = (TextView) findViewById(R.id.upperS);
        upperV = (SeekBar) findViewById(R.id.barupperV);
        tupperV = (TextView) findViewById(R.id.upperV);

        Status = (TextView) findViewById(R.id.Status);
        SensorT = (TextView) findViewById(R.id.SensorT);

        ScalarT.setTextColor(Color.WHITE);
        tupperH.setTextColor(Color.WHITE);
        tlowerH.setTextColor(Color.WHITE);
        tlowerS.setTextColor(Color.WHITE);
        tlowerV.setTextColor(Color.WHITE);
        tupperH.setTextColor(Color.WHITE);
        tupperS.setTextColor(Color.WHITE);
        tupperV.setTextColor(Color.WHITE);
        Status.setTextColor(Color.WHITE);
        SensorT.setTextColor(Color.WHITE);

        PackageManager pm = this.getPackageManager();
        boolean accel = pm.hasSystemFeature(PackageManager.FEATURE_SENSOR_ACCELEROMETER);
        if (accel) {
            if (accel) {
                Toast.makeText(getApplicationContext(), "Ada Sensor Accel", Toast.LENGTH_SHORT).show();
            }
        }
        mSensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);
        mSensor = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        usbManager = (UsbManager) getSystemService(this.USB_SERVICE);

        IntentFilter filter = new IntentFilter();
        filter.addAction(ACTION_USB_PERMISSION);
        filter.addAction(UsbManager.ACTION_USB_DEVICE_ATTACHED);
        filter.addAction(UsbManager.ACTION_USB_DEVICE_DETACHED);
        registerReceiver(broadcastReceiver, filter);

        lowerH.setOnSeekBarChangeListener((SeekBar.OnSeekBarChangeListener) this);
        lowerS.setOnSeekBarChangeListener((SeekBar.OnSeekBarChangeListener) this);
        lowerV.setOnSeekBarChangeListener((SeekBar.OnSeekBarChangeListener) this);
        upperH.setOnSeekBarChangeListener((SeekBar.OnSeekBarChangeListener) this);
        upperS.setOnSeekBarChangeListener((SeekBar.OnSeekBarChangeListener) this);
        upperV.setOnSeekBarChangeListener((SeekBar.OnSeekBarChangeListener) this);

        mOpenCVCameraView = (JavaCameraView) findViewById(R.id.show_camera_activity_java_surface_view);
        mOpenCVCameraView.setMaxFrameSize(220, 620);
        mOpenCVCameraView.setVisibility(SurfaceView.VISIBLE);
        mOpenCVCameraView.setCvCameraViewListener(this);


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
        }catch(Exception e){
            Status.setText(e.toString());
        }

    }


    public void onClickStop(View view) {
        try {
            serialPort.close();
            Status.setText("Disconnected");
        }catch(Exception e){
            Status.setText(e.toString());
        }
    }


    public String sendData(){
        try {
            String data = String.valueOf(trackLine.getCenterMoment().y + "\n");
//            if(trackLine.getCenterMoment().y < 235 && trackLine.getCenterMoment().y > 150){
//                data = "1";
//            }
//            else if(trackLine.getCenterMoment().y < 150 && trackLine.getCenterMoment().y > 100){
//                data = "2";
//            }
//            else if(trackLine.getCenterMoment().y > 10 && trackLine.getCenterMoment().y < 100){
//                data = "3";
//            }else { data = ""; }


            return data;
        }catch (Exception e){
            return null;
        }
    }

    @Override
    public void onProgressChanged(SeekBar seekBar, int i, boolean b) {
        switch(seekBar.getId()){
            case R.id.barlowerH:
                tlowerH.setText("LowerH " + i);
                tlH = 0;
                break;
            case R.id.barlowerS:
                tlowerS.setText("LowerS " + i);
                tlS = 0;
                break;
            case R.id.barlowerV:
                tlowerV.setText("LowerV " + i);
                tlV = 173;
                break;
            case R.id.barupperH:
                tupperH.setText("UpperH " + i);
                tuH = 215;
                break;
            case R.id.barupperS:
                tupperS.setText("UpperS " + i);
                tuS = 224;
                break;
            case R.id.barupperV:
                tupperV.setText("UpperV " + i);
                tuV = 255;
                break;
        }
    }

    @Override
    public void onStartTrackingTouch(SeekBar seekBar) {

    }

    @Override
    public void onStopTrackingTouch(SeekBar seekBar) {

    }

    public void onPause(){
        super.onPause();
        if(mOpenCVCameraView != null){
            mOpenCVCameraView.disableView();
        }
        mSensorManager.unregisterListener(this);
    }

    public void onResume(){
        super.onResume();

        if(!OpenCVLoader.initDebug()){
            Log.d(TAG, "Internal OpenCV not found, Using OpenCV manager for Initialize");
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_1_0, this, mLoaderCallback);
        } else{
            Log.d(TAG, "OpenCV loader found inside The Package, Using it!!");
            mLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }
        mSensorManager.registerListener(this, mSensor, SensorManager.SENSOR_DELAY_NORMAL);

    }

    public void onDestroy(){
        super.onDestroy();

        if(mOpenCVCameraView != null){
            mOpenCVCameraView.disableView();
        }
    }

    @Override
    public void onCameraViewStarted(int width, int height) {
        mRgba = new Mat(width, height, CvType.CV_8UC4);
        mHsv = new Mat();
        trackLine = new TrackLine();
    }

    @Override
    public void onCameraViewStopped() {
        mRgba.release();
        mHsv.release();
    }


    @Override
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {

        mRgba = inputFrame.rgba();
//        trackLine.setScalar(tlH,tlS,tlV,tuH,tuS,tuV);
//        trackLine.ProcThresh(mRgba);
        trackLine.drawContour(mRgba);

        if (trackLine.getBounding_rect() != null) {
            System.out.println("Data BR : " + trackLine.getBounding_rect().x + " , " + trackLine.getBounding_rect().y);
        }
        if (connection != null) {
            serialPort.write(sendData().getBytes());
        }

        return mRgba;
    }


    @Override
    public void onSensorChanged(SensorEvent sensorEvent) {
        final float alpha =  0.8f;

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

        Status.setText(sendData());
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int i) {

    }

    public float getAccelX(){ return accelX; }
    public float getAccelY(){ return accelY; }
    public float getAccelZ(){ return accelZ; }
    public Context getCont(){
        return cont;
    }

}
