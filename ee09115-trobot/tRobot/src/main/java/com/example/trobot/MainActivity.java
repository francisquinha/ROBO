package com.example.trobot;

import android.app.Activity;
import android.content.Intent;
import android.hardware.Camera.Size;
import android.os.Bundle;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.SubMenu;
import android.view.SurfaceView;
import android.view.View;
import android.view.View.OnClickListener;
import android.view.WindowManager;
import android.widget.AdapterView;
import android.widget.AdapterView.OnItemClickListener;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.ListView;
import android.widget.SeekBar;
import android.widget.SeekBar.OnSeekBarChangeListener;
import android.widget.TextView;
import android.widget.Toast;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.List;
import java.util.ListIterator;

import bsh.EvalError;
import bsh.Interpreter;

public class MainActivity extends Activity implements CvCameraViewListener2, OnItemClickListener{



    public static final String 	TAG 					= "App Valter";
    private static final int 		REQUEST_CODE_SCRIPT		= 1000;
    public static String			stringCode				= null;
    long i_frame											= 0;

    Interpreter 					interpreter 			= new Interpreter();

    // Camera Modes
    public static final int VIEW_MODE_RGB = 0;
    public static final int VIEW_MODE_RED = 1;
    public static final int VIEW_MODE_GREEN = 2;
    public static final int VIEW_MODE_BLUE = 3;
    public static final int VIEW_MODE_COLOR_SENSOR = 4;
    public static final int VIEW_MODE_COLOR_PROXIMITY = 5;

    // Opencv Camera
    private Tutorial3View 	mOpenCvCameraView;
    private List<Size> mResolutionList;
    private MenuItem[] mResolutionMenuItems;

    // Menu
    private SubMenu mItemChangeView	= null;
    private SubMenu	mItemChangeResolution = null;

    public int viewMode = VIEW_MODE_RGB;

    // Opencv Stuff to rotate portrait orientation
//    public Mat mRgba;
//    public Mat mRgbaF;
//    public Mat mRgbaT;
    public Vision vision = new Vision();
    public BluetoothConnection bluetoothConnection;

    int red_h_min 	= 0;
    int red_h_max 	= 10;
    int red_s_min 	= 207;
    int red_s_max 	= 255;
    int red_v_min 	= 50;
    int red_v_max 	= 255;
    int green_h_min = 34;
    int green_h_max = 105;
    int green_s_min = 75;
    int green_s_max = 255;
    int green_v_min = 24;
    int green_v_max = 255;
    int blue_h_min 	= 70;
    int blue_h_max 	= 144;
    int blue_s_min 	= 80;
    int blue_s_max 	= 255;
    int blue_v_min 	= 26;
    int blue_v_max 	= 255;

    int h_min 		= red_h_min;
    int h_max 		= red_h_max;
    int s_min 		= red_s_min;
    int s_max 		= red_s_max;
    int v_min 		= red_v_min;
    int v_max 		= red_v_max;

    public int robotPosition = -1;

    boolean start = false;
    boolean stop = true;

    // Layout Seekbar Stuff
    SeekBar HSeekBarMin;
    SeekBar HSeekBarMax;
    SeekBar SSeekBarMin;
    SeekBar SSeekBarMax;
    SeekBar VSeekBarMin;
    SeekBar VSeekBarMax;

    TextView textViewHue;
    TextView textViewSat;
    TextView textViewVal;
    private static TextView textViewDebug;

    // Bluetooth list
    ArrayAdapter<String> bluetoothListAdapter;
    ListView bluetoothListView;

    // Initialize the first point for tracking line
//    int[] PrevCenterLine = {160,200,160,200-50};

    static{ System.loadLibrary("opencv_java"); }

    static {
        if (!OpenCVLoader.initDebug()) {
            // Handle initialization error
            Log.e(TAG, "OpenCVLoader Failed");
        }
    }

    private BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status) {
            switch (status) {
                case LoaderCallbackInterface.SUCCESS:
                {
                    Log.i(TAG, "OpenCV loaded successfully");
                    mOpenCvCameraView.enableView();
                    break;
                }
                case LoaderCallbackInterface.INIT_FAILED:
                {
                    finish();
                    break;
                }
                default:
                {
                    super.onManagerConnected(status);
                } break;
            }
        }
    };

    public MainActivity() {
        Log.i(TAG, "Instantiated new " + this.getClass());
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        setContentView(R.layout.activity_main);

        if (!OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_2_4_9, this, mLoaderCallback)) {
            finish();
        }

        // Create a Camera
        mOpenCvCameraView = (Tutorial3View) findViewById(R.id.viewFinder);
        mOpenCvCameraView.setVisibility(SurfaceView.VISIBLE);
        mOpenCvCameraView.setCvCameraViewListener(this);

        // Initialize Bluetooth
        bluetoothListView = (ListView)findViewById(R.id.listView);
        bluetoothListView.setOnItemClickListener(this);
        bluetoothListAdapter = new ArrayAdapter<String>(this, android.R.layout.simple_expandable_list_item_1,0);
        bluetoothListView.setAdapter(bluetoothListAdapter);
        bluetoothConnection = new BluetoothConnection(this);


        // Call a second intent
        Button btn = (Button)findViewById(R.id.button1);
        btn.setOnClickListener(new OnClickListener() {
            @Override
            public void onClick(View v) {
                Intent intent = new Intent(getApplicationContext(),ActivityTwo.class);
                if (stringCode != null)
                    intent.putExtra("code", stringCode);
                if(vision.getTrackedColorPosition() != null) {
                    intent.putExtra("colorPositionX", vision.getTrackedColorPosition().x);
                    intent.putExtra("colorPositionY", vision.getTrackedColorPosition().y);
                }
                intent.putExtra("sensor1Active", vision.getActiveSensorIndices().contains(0));
                intent.putExtra("sensor2Active", vision.getActiveSensorIndices().contains(1));
                intent.putExtra("sensor3Active", vision.getActiveSensorIndices().contains(2));
                intent.putExtra("sensor4Active", vision.getActiveSensorIndices().contains(3));
                intent.putExtra("sensor5Active", vision.getActiveSensorIndices().contains(4));
                intent.putExtra("sensor6Active", vision.getActiveSensorIndices().contains(5));
                intent.putExtra("sensor7Active", vision.getActiveSensorIndices().contains(6));
                intent.putExtra("sensor8Active", vision.getActiveSensorIndices().contains(7));
                intent.putExtra("sensor9Active", vision.getActiveSensorIndices().contains(8));
                intent.putExtra("colorFillPercentage", vision.getColorFillPercentage());
                startActivityForResult(intent, REQUEST_CODE_SCRIPT);
            }
        });

        // Initialize HSV textViews
        textViewHue = (TextView)findViewById(R.id.textViewHue);
        textViewHue.setText("H: " + String.valueOf(h_min) + " " + String.valueOf(h_max));
        textViewSat = (TextView)findViewById(R.id.textViewSat);
        textViewSat.setText("H: " + String.valueOf(s_min) + " " + String.valueOf(s_max));
        textViewVal = (TextView)findViewById(R.id.textViewVal);
        textViewVal.setText("H: " + String.valueOf(v_min) + " " + String.valueOf(v_max));

        textViewDebug = (TextView)findViewById(R.id.textViewDebug1);
        textViewDebug.setText("Alive");

        //editTextDebug = (EditText)findViewById(R.id.editTextDebug);
        //editTextDebug.setText("Alive");

        Button btnStart = (Button)findViewById(R.id.buttonStart);
        btnStart.setOnClickListener(new OnClickListener() {
            @Override
            public void onClick(View v) {
                start = true;
                stop = false;
            }
        });
        Button btnStop = (Button)findViewById(R.id.buttonStop);
        btnStop.setOnClickListener(new OnClickListener() {
            @Override
            public void onClick(View v) {
                stop = true;
                start = false;
            }
        });



        // Initialize SeekBars
        HSeekBarMin = (SeekBar)findViewById(R.id.seekBarHMin);
        HSeekBarMax = (SeekBar)findViewById(R.id.seekBarHMax);
        SSeekBarMin = (SeekBar)findViewById(R.id.seekBarSMin);
        SSeekBarMax = (SeekBar)findViewById(R.id.seekBarSMax);
        VSeekBarMin = (SeekBar)findViewById(R.id.seekBarVMin);
        VSeekBarMax = (SeekBar)findViewById(R.id.seekBarVMax);

        // Initialize hsv trackbars
        HSeekBarMin.setProgress(red_h_min);
        HSeekBarMax.setProgress(red_h_max);
        SSeekBarMin.setProgress(red_s_min);
        SSeekBarMax.setProgress(red_s_max);
        VSeekBarMin.setProgress(red_v_min);
        VSeekBarMax.setProgress(red_v_max);

    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        // TODO Auto-generated method stub
        super.onActivityResult(requestCode, resultCode, data);
        if (resultCode == RESULT_CANCELED && requestCode != REQUEST_CODE_SCRIPT)
        {
            Toast.makeText(getApplicationContext(), "Bluetooth must be enabled to continue.", Toast.LENGTH_SHORT).show();
        }
        if (requestCode == REQUEST_CODE_SCRIPT)
        {
            if (resultCode == RESULT_OK)
            {
                if (stringCode!=null)
                    stringCode=null;
                stringCode = data.getStringExtra("code");
                Toast.makeText(getApplicationContext(), stringCode, Toast.LENGTH_SHORT).show();
            }
            else if (resultCode == RESULT_CANCELED)
            {
                Toast.makeText(getApplicationContext(), "No code passed.", Toast.LENGTH_SHORT).show();

            }
        }

    }

    @Override
    protected void onPause() {
        super.onPause();
        try {
            unregisterReceiver(bluetoothConnection.getReceiver());
        } catch(Exception e){
            e.printStackTrace();
        }

        mOpenCvCameraView.disableView();
        bluetoothConnection.cancelDiscovery();
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        //getMenuInflater().inflate(R.menu.main, menu);

        mItemChangeView = menu.addSubMenu("View");
        mItemChangeView.add(1,VIEW_MODE_RGB,Menu.NONE,"RGB View");
        mItemChangeView.add(1,VIEW_MODE_RED,Menu.NONE,"RED View");
        mItemChangeView.add(1,VIEW_MODE_GREEN,Menu.NONE,"GREEN View");
        mItemChangeView.add(1,VIEW_MODE_BLUE,Menu.NONE,"BLUE View");
        mItemChangeView.add(1,VIEW_MODE_COLOR_SENSOR,Menu.NONE,"Color Sensor View");
        mItemChangeView.add(1,VIEW_MODE_COLOR_PROXIMITY,Menu.NONE,"Color Proximity View");

        mItemChangeResolution = menu.addSubMenu("Resolution");
        mResolutionList = mOpenCvCameraView.getResolutionList();
        mResolutionMenuItems = new MenuItem[mResolutionList.size()];

        ListIterator<Size> resolutionItr = mResolutionList.listIterator();
        int idx = 0;
        while(resolutionItr.hasNext()) {
            Size element = resolutionItr.next();
            if (element.width <= 320)
                mResolutionMenuItems[idx] = mItemChangeResolution.add(2, idx, Menu.NONE,Integer.valueOf(element.width).toString() + "x" + Integer.valueOf(element.height).toString());

            idx++;
        }
        for (int i = 0; i<mResolutionList.size(); i++) {
            Size resolution = mResolutionList.get(i);
            if (resolution.width==320){
                mOpenCvCameraView.setResolution(resolution);
                vision.initializeMRgba(resolution.width, resolution.height);
                vision.initializePrevCenterLine(resolution.width, resolution.height);
            }
        }


        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        Log.i(TAG, "called onOptionsItemSelected; selected item: " + item);
        if (item.getGroupId() == 1) {
            int id = item.getItemId();
            switch(id)
            {
                case VIEW_MODE_RGB:
                {
                    viewMode = VIEW_MODE_RGB;
                    break;
                }
                case VIEW_MODE_RED:
                {
                    viewMode = VIEW_MODE_RED;
                    vision.setHSVMinMax(Vision.RED, new Scalar(red_h_min,red_s_min,red_v_min), new Scalar(red_h_max,red_s_max,red_v_max));
                    HSeekBarMin.setProgress(red_h_min);
                    HSeekBarMax.setProgress(red_h_max);
                    SSeekBarMin.setProgress(red_s_min);
                    SSeekBarMax.setProgress(red_s_max);
                    VSeekBarMin.setProgress(red_v_min);
                    VSeekBarMax.setProgress(red_v_max);
                    break;
                }
                case VIEW_MODE_GREEN:
                {
                    viewMode = VIEW_MODE_GREEN;
                    vision.setHSVMinMax(Vision.GREEN, new Scalar(green_h_min,green_s_min,green_v_min), new Scalar(green_h_max,green_s_max,green_v_max));
                    HSeekBarMin.setProgress(green_h_min);
                    HSeekBarMax.setProgress(green_h_max);
                    SSeekBarMin.setProgress(green_s_min);
                    SSeekBarMax.setProgress(green_s_max);
                    VSeekBarMin.setProgress(green_v_min);
                    VSeekBarMax.setProgress(green_v_max);
                    break;
                }
                case VIEW_MODE_BLUE:
                {
                    viewMode = VIEW_MODE_BLUE;
                    vision.setHSVMinMax(Vision.BLUE, new Scalar(blue_h_min,blue_s_min,blue_v_min), new Scalar(blue_h_max,blue_s_max,blue_v_max));
                    HSeekBarMin.setProgress(blue_h_min);
                    HSeekBarMax.setProgress(blue_h_max);
                    SSeekBarMin.setProgress(blue_s_min);
                    SSeekBarMax.setProgress(blue_s_max);
                    VSeekBarMin.setProgress(blue_v_min);
                    VSeekBarMax.setProgress(blue_v_max);
                    break;
                }
                case VIEW_MODE_COLOR_SENSOR: {
                    viewMode = VIEW_MODE_COLOR_SENSOR;
                    break;
                }
                case VIEW_MODE_COLOR_PROXIMITY: {
                    viewMode = VIEW_MODE_COLOR_PROXIMITY;
                    break;
                }
                default:
                    break;
            }
            vision.updatePrevCenterLineWithMRgba();
        }
        else if (item.getGroupId() == 2) {
            int id = item.getItemId();
            Size resolution = mResolutionList.get(id);
            mOpenCvCameraView.setResolution(resolution);
            resolution = mOpenCvCameraView.getResolution();
            String caption = Integer.valueOf(resolution.width).toString() + "x" + Integer.valueOf(resolution.height).toString();
            Toast.makeText(this, caption, Toast.LENGTH_SHORT).show();
            vision.initializeMRgba(resolution.width, resolution.height);
            vision.initializePrevCenterLine(resolution.width, resolution.height);
        }

        return true;
    }

    public void onDestroy() {
        super.onDestroy();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
    }

    @Override
    public void onBackPressed() {
        // TODO Auto-generated method stub
        super.onBackPressed();
        //if (btAdapter.isDiscovering())
        //btAdapter.cancelDiscovery();
        mOpenCvCameraView.disableView();
        //if (receiver != null)
        //unregisterReceiver(receiver);
        finish();

    }

    @Override
    public void onResume()
    {
        super.onResume();
        mLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        //OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_2_4_9, this, mLoaderCallback);
        mOpenCvCameraView.enableView();
    }

    public void onCameraViewStarted(int width, int height) {
        vision.setHSVDefaults();

        vision.initializeMRgba(width, height);

        HSeekBarMin.setOnSeekBarChangeListener(new OnSeekBarChangeListener()
        {
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress,boolean fromUser)
            {
                switch(viewMode)
                {
                    case MainActivity.VIEW_MODE_RGB:
                        h_min = progress;
                        textViewHue.setText("H: " + String.valueOf(h_min) + " " + String.valueOf(h_max));
                        break;
                    case MainActivity.VIEW_MODE_RED:
                        h_min = progress;
//                        vision.setHSVMin(Vision.RED, new Scalar(h_min,s_min,v_min));
                        textViewHue.setText("H: " + String.valueOf(h_min) + " " + String.valueOf(h_max));
                        break;
                    case MainActivity.VIEW_MODE_GREEN:
                        h_min = progress;
//                        vision.setHSVMin(Vision.GREEN, new Scalar(h_min,s_min,v_min));
                        textViewHue.setText("H: " + String.valueOf(h_min) + " " + String.valueOf(h_max));
                        break;
                    case MainActivity.VIEW_MODE_BLUE:
                        h_min = progress;
//                        vision.setHSVMin(Vision.BLUE, new Scalar(h_min,s_min,v_min));
                        textViewHue.setText("H: " + String.valueOf(h_min) + " " + String.valueOf(h_max));
                        break;
                    case MainActivity.VIEW_MODE_COLOR_SENSOR:
                        h_min = progress;
                        textViewHue.setText("H: " + String.valueOf(h_min) + " " + String.valueOf(h_max));
                        break;
                    case MainActivity.VIEW_MODE_COLOR_PROXIMITY:
                        h_min = progress;
                        textViewHue.setText("H: " + String.valueOf(h_min) + " " + String.valueOf(h_max));
                        break;
                    default:break;
                }
            }
            @Override
            public void onStopTrackingTouch(SeekBar seekBar)
            {
                //Do some operation in this block
            }
            @Override
            public void onStartTrackingTouch(SeekBar seekBar)
            {
                //Do some operation in this block
            }
        });
        HSeekBarMax.setOnSeekBarChangeListener(new OnSeekBarChangeListener()
        {
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress,boolean fromUser)
            {
                switch(viewMode)
                {
                    case MainActivity.VIEW_MODE_RGB:
                        h_max = progress;
                        textViewHue.setText("H: " + String.valueOf(h_min) + " " + String.valueOf(h_max));
                        break;
                    case MainActivity.VIEW_MODE_RED:
                        h_max = progress;
//                        vision.setHSVMax(Vision.RED, new Scalar(h_max,s_max,v_max));
                        textViewHue.setText("H: " + String.valueOf(h_min) + " " + String.valueOf(h_max));
                        break;
                    case MainActivity.VIEW_MODE_GREEN:
                        h_max = progress;
//                        vision.setHSVMax(Vision.GREEN, new Scalar(h_max,s_max,v_max));
                        textViewHue.setText("H: " + String.valueOf(h_min) + " " + String.valueOf(h_max));
                        break;
                    case MainActivity.VIEW_MODE_BLUE:
                        h_max = progress;
//                        vision.setHSVMax(Vision.BLUE, new Scalar(h_max,s_max,v_max));
                        textViewHue.setText("H: " + String.valueOf(h_min) + " " + String.valueOf(h_max));
                        break;
                    case MainActivity.VIEW_MODE_COLOR_SENSOR:
                        h_max = progress;
                        textViewHue.setText("H: " + String.valueOf(h_min) + " " + String.valueOf(h_max));
                        break;
                    case MainActivity.VIEW_MODE_COLOR_PROXIMITY:
                        h_max = progress;
                        textViewHue.setText("H: " + String.valueOf(h_min) + " " + String.valueOf(h_max));
                        break;
                    default:break;
                }

            }
            @Override
            public void onStopTrackingTouch(SeekBar seekBar)
            {
                //Do some operation in this block
            }
            @Override
            public void onStartTrackingTouch(SeekBar seekBar)
            {
                //Do some operation in this block
            }
        });
        SSeekBarMin.setOnSeekBarChangeListener(new OnSeekBarChangeListener()
        {
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress,boolean fromUser)
            {
                switch(viewMode)
                {
                    case MainActivity.VIEW_MODE_RGB:
                        s_min = progress;
                        textViewSat.setText("S: " + String.valueOf(s_min) + " " + String.valueOf(s_max));
                        break;
                    case MainActivity.VIEW_MODE_RED:
                        s_min = progress;
//                        vision.setHSVMin(Vision.RED, new Scalar(h_min,s_min,v_min));
                        textViewSat.setText("S: " + String.valueOf(s_min) + " " + String.valueOf(s_max));
                        break;
                    case MainActivity.VIEW_MODE_GREEN:
                        s_min = progress;
//                        vision.setHSVMin(Vision.GREEN, new Scalar(h_min,s_min,v_min));
                        textViewSat.setText("S: " + String.valueOf(s_min) + " " + String.valueOf(s_max));
                        break;
                    case MainActivity.VIEW_MODE_BLUE:
                        s_min = progress;
//                        vision.setHSVMin(Vision.BLUE, new Scalar(h_min,s_min,v_min));
                        textViewSat.setText("S: " + String.valueOf(s_min) + " " + String.valueOf(s_max));
                        break;
                    case MainActivity.VIEW_MODE_COLOR_SENSOR:
                        s_min = progress;
                        textViewSat.setText("S: " + String.valueOf(s_min) + " " + String.valueOf(s_max));
                        break;
                    case MainActivity.VIEW_MODE_COLOR_PROXIMITY:
                        s_min = progress;
                        textViewSat.setText("S: " + String.valueOf(s_min) + " " + String.valueOf(s_max));
                        break;
                    default:break;
                }
            }
            @Override
            public void onStopTrackingTouch(SeekBar seekBar)
            {
                //Do some operation in this block
            }
            @Override
            public void onStartTrackingTouch(SeekBar seekBar)
            {
                //Do some operation in this block
            }
        });
        SSeekBarMax.setOnSeekBarChangeListener(new OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                switch (viewMode) {
                    case MainActivity.VIEW_MODE_RGB:
                        s_max = progress;
                        textViewSat.setText("S: " + String.valueOf(s_min) + " " + String.valueOf(s_max));
                        break;
                    case MainActivity.VIEW_MODE_RED:
                        s_max = progress;
//                        vision.setHSVMax(Vision.RED, new Scalar(h_max,s_max,v_max));
                        textViewSat.setText("S: " + String.valueOf(s_min) + " " + String.valueOf(s_max));
                        break;
                    case MainActivity.VIEW_MODE_GREEN:
                        s_max = progress;
//                        vision.setHSVMax(Vision.GREEN, new Scalar(h_max,s_max,v_max));
                        textViewSat.setText("S: " + String.valueOf(s_min) + " " + String.valueOf(s_max));
                        break;
                    case MainActivity.VIEW_MODE_BLUE:
                        s_max = progress;
//                        vision.setHSVMax(Vision.BLUE, new Scalar(h_max,s_max,v_max));
                        textViewSat.setText("S: " + String.valueOf(s_min) + " " + String.valueOf(s_max));
                        break;
                    case MainActivity.VIEW_MODE_COLOR_SENSOR:
                        s_max = progress;
                        textViewSat.setText("S: " + String.valueOf(s_min) + " " + String.valueOf(s_max));
                        break;
                    case MainActivity.VIEW_MODE_COLOR_PROXIMITY:
                        s_max = progress;
                        textViewSat.setText("S: " + String.valueOf(s_min) + " " + String.valueOf(s_max));
                        break;
                    default:
                        break;
                }
            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {
                //Do some operation in this block
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {
                //Do some operation in this block
            }
        });
        VSeekBarMin.setOnSeekBarChangeListener(new OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                switch (viewMode) {
                    case MainActivity.VIEW_MODE_RGB:
                        v_min = progress;
                        textViewVal.setText("V: " + String.valueOf(v_min) + " " + String.valueOf(v_max));
                        break;
                    case MainActivity.VIEW_MODE_RED:
                        v_min = progress;
//                        vision.setHSVMin(Vision.RED, new Scalar(h_min,s_min,v_min));
                        textViewVal.setText("V: " + String.valueOf(v_min) + " " + String.valueOf(v_max));
                        break;
                    case MainActivity.VIEW_MODE_GREEN:
                        v_min = progress;
//                        vision.setHSVMin(Vision.GREEN, new Scalar(h_min,s_min,v_min));
                        textViewVal.setText("V: " + String.valueOf(v_min) + " " + String.valueOf(v_max));
                        break;
                    case MainActivity.VIEW_MODE_BLUE:
                        v_min = progress;
//                        vision.setHSVMin(Vision.BLUE, new Scalar(h_min,s_min,v_min));
                        textViewVal.setText("V: " + String.valueOf(v_min) + " " + String.valueOf(v_max));
                        break;
                    case MainActivity.VIEW_MODE_COLOR_SENSOR:
                        v_min = progress;
                        textViewVal.setText("V: " + String.valueOf(v_min) + " " + String.valueOf(v_max));
                        break;
                    case MainActivity.VIEW_MODE_COLOR_PROXIMITY:
                        v_min = progress;
                        textViewVal.setText("V: " + String.valueOf(v_min) + " " + String.valueOf(v_max));
                        break;
                    default:
                        break;
                }
            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {
                //Do some operation in this block
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {
                //Do some operation in this block
            }
        });
        VSeekBarMax.setOnSeekBarChangeListener(new OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                switch (viewMode) {
                    case MainActivity.VIEW_MODE_RGB:
                        v_max = progress;
                        textViewVal.setText("V: " + String.valueOf(v_min) + " " + String.valueOf(v_max));
                        break;
                    case MainActivity.VIEW_MODE_RED:
                        v_max = progress;
//                        vision.setHSVMax(Vision.RED, new Scalar(h_max,s_max,v_max));
                        textViewVal.setText("V: " + String.valueOf(v_min) + " " + String.valueOf(v_max));

                        break;
                    case MainActivity.VIEW_MODE_GREEN:
                        v_max = progress;
//                        vision.setHSVMax(Vision.GREEN, new Scalar(h_max,s_max,v_max));
                        textViewVal.setText("V: " + String.valueOf(v_min) + " " + String.valueOf(v_max));

                        break;
                    case MainActivity.VIEW_MODE_BLUE:
                        v_max = progress;
//                        vision.setHSVMax(Vision.BLUE, new Scalar(h_max,s_max,v_max));
                        textViewVal.setText("V: " + String.valueOf(v_min) + " " + String.valueOf(v_max));
                        break;
                    case MainActivity.VIEW_MODE_COLOR_SENSOR:
                        v_max = progress;
                        textViewVal.setText("V: " + String.valueOf(v_min) + " " + String.valueOf(v_max));
                        break;
                    case MainActivity.VIEW_MODE_COLOR_PROXIMITY:
                        v_max = progress;
                        textViewVal.setText("V: " + String.valueOf(v_min) + " " + String.valueOf(v_max));
                        break;
                    default:
                        break;
                }

            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {
                //Do some operation in this block
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {
                //Do some operation in this block
            }
        });
    }

    public void onCameraViewStopped() {
    }

    @Override
    public Mat onCameraFrame(CvCameraViewFrame inputFrame) {
        Mat src = vision.preProcessFrame(inputFrame);

        if(viewMode == MainActivity.VIEW_MODE_RED || viewMode == MainActivity.VIEW_MODE_GREEN || viewMode == MainActivity.VIEW_MODE_BLUE) {
            Imgproc.cvtColor(src, src, Imgproc.COLOR_RGB2HSV);
            src = Vision.myInRange(src, h_min, h_max, s_min, s_max, v_min, v_max);
            if (start) {
                Mat previousSrc = src;
                src = vision.processFrame(src, this);
                Point robotPos = vision.calculateIntersectionWithHorizontalLine(previousSrc.rows(), previousSrc.cols(), vision.getPrevCenterLine());

                if(robotPosition != robotPos.x) {
                    robotPosition = (int) robotPos.x;
                    Vision.drawRobotPosition(src, robotPos);

                    Log.i(MainActivity.TAG, "xi " + robotPos.x + " yi " + robotPos.y);
                    Log.i(MainActivity.TAG, "robotP " + robotPos.x);
                }

                evaluateInterpreter(robotPos);
            }
        }
        else if(viewMode == MainActivity.VIEW_MODE_COLOR_SENSOR) {
            src = vision.updateSensorData(src, h_min, s_min, v_min, h_max, s_max, v_max, 9);

            String sensorIndicatorString = "Active sensor: ";
            List<Integer> activeSensorIndices = vision.getActiveSensorIndices();
            if(activeSensorIndices.isEmpty()) {
                sensorIndicatorString += "None";
            }
            else {
                for(int sensorIndex : activeSensorIndices) {
                    sensorIndicatorString += (sensorIndex + 1);
                    sensorIndicatorString += ", ";
                }
                sensorIndicatorString = sensorIndicatorString.replaceFirst(", $", "");
            }

            final String finalSensorIndicatorString = sensorIndicatorString;
            runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    textViewDebug.setText(finalSensorIndicatorString);
                }
            });

            vision.updatePositionOfTrackedColor(src);
            Point position = vision.getTrackedColorPosition();

            vision.drawSensorGrid(src);
            if(position != null) {
                vision.drawColorPosition(src);
            }

            if(start) {
                evaluateInterpreter(null);
            }
        }
        else if(viewMode == MainActivity.VIEW_MODE_COLOR_PROXIMITY) {
            src = vision.updateColorFillPercentage(src, h_min, s_min, v_min, h_max, s_max, v_max);
            double colorFillPercentage = vision.getColorFillPercentage();

            final String colorFillIndicatorString = String.format("%.1f%% of view filled by the tracked color.", colorFillPercentage);

            runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    textViewDebug.setText(colorFillIndicatorString);
                }
            });

            vision.updatePositionOfTrackedColor(src);
            Point position = vision.getTrackedColorPosition();

            if(position != null) {
                vision.drawColorPosition(src);
            }

            if(start) {
                evaluateInterpreter(null);
            }
        }

        if (stop) {
//            byte[] data = new byte[]{(byte)64,(byte)64};
            byte[] data = new byte[]{(byte)0,(byte)0};
            BluetoothDataFrame bluetoothDataFrame = new BluetoothDataFrame(BluetoothDataFrame.MOTOR_VELOCITY, data);
            bluetoothConnection.writeToConnectedThread(bluetoothDataFrame);
        }

        return src;
    }

    private void evaluateInterpreter(Point robotPos) {
//        int v1 = 64;
//        int v2 = 64;
        int v1 = 0;
        int v2 = 0;


        //stringCode = "v1 = 84 + 0.2*(160-robotPosition); v2 = 84 - 0.2*(160-robotPosition);";
        int bumper = 0;
        BluetoothDataFrame bluetoothDataFrame = bluetoothConnection.readFromConnectedThread();
        if(bluetoothDataFrame != null ) {
            if (bluetoothDataFrame.getFrameType() == BluetoothDataFrame.BUMPER) {
                int value = bluetoothDataFrame.extractBumperInfo();
                if (value == 13) {
                    bumper = 1;
                }
                Log.i(MainActivity.TAG, "received " + Integer.toString(value));
            }

            Log.i(MainActivity.TAG, "Received bluetooth data frame of type: " + Integer.toString(bluetoothDataFrame.getFrameType()));
        }

        if (MainActivity.stringCode!=null) {
            try {
                interpreter.set("robotPosition", robotPosition);
                interpreter.set("v1", v1);
                interpreter.set("v2", v2);
                interpreter.set("bumper",bumper);
                if(vision.getTrackedColorPosition() != null) {
                    interpreter.set("colorPositionX", vision.getTrackedColorPosition().x);
                    interpreter.set("colorPositionY", vision.getTrackedColorPosition().y);
                }
                else {
                    interpreter.set("colorPositionX", -1);
                    interpreter.set("colorPositionY", -1);
                }
                interpreter.set("sensor1Active", vision.getActiveSensorIndices().contains(0));
                interpreter.set("sensor2Active", vision.getActiveSensorIndices().contains(1));
                interpreter.set("sensor3Active", vision.getActiveSensorIndices().contains(2));
                interpreter.set("sensor4Active", vision.getActiveSensorIndices().contains(3));
                interpreter.set("sensor5Active", vision.getActiveSensorIndices().contains(4));
                interpreter.set("sensor6Active", vision.getActiveSensorIndices().contains(5));
                interpreter.set("sensor7Active", vision.getActiveSensorIndices().contains(6));
                interpreter.set("sensor8Active", vision.getActiveSensorIndices().contains(7));
                interpreter.set("sensor9Active", vision.getActiveSensorIndices().contains(8));
                interpreter.set("colorFillPercentage", vision.getColorFillPercentage());
            } catch (EvalError e1) {
                // TODO Auto-generated catch block
                e1.printStackTrace();
            }

            try {
                interpreter.eval(MainActivity.stringCode);
                v1 = (int)(Float.parseFloat(interpreter.get("v1").toString()));
                v2 = (int)(Float.parseFloat(interpreter.get("v2").toString()));
                //console.setText("Success: " + interpreter.get("robotPosition").toString());

            }catch (Exception e) {
                //console.setText("Error: " + code_string);
            }
        }

        if(v1 > 100) {
            v1 = 100;
        }
        else if(v1 < -100) {
            v1 = -100;
        }

        if(v2 > 100) {
            v2 = 100;
        }
        else if(v2 < -100) {
            v2 = -100;
        }

        if ((viewMode == VIEW_MODE_RED || viewMode == VIEW_MODE_GREEN || viewMode == VIEW_MODE_BLUE) && robotPos != null && robotPos.y > 100) {
            byte[] data = new byte[]{(byte) v1, (byte) v2};
            BluetoothDataFrame velocityDataFrame = new BluetoothDataFrame(BluetoothDataFrame.MOTOR_VELOCITY, data);
            bluetoothConnection.writeToConnectedThread(velocityDataFrame);
            Log.i(MainActivity.TAG, "v1 " + Integer.toString(v1));
            Log.i(MainActivity.TAG, "v2 " + Integer.toString(v2));
        }
        else if (viewMode == VIEW_MODE_COLOR_PROXIMITY || viewMode == VIEW_MODE_COLOR_SENSOR) {
            byte[] data = new byte[]{(byte) v1, (byte) v2};
            BluetoothDataFrame velocityDataFrame = new BluetoothDataFrame(BluetoothDataFrame.MOTOR_VELOCITY, data);
            bluetoothConnection.writeToConnectedThread(velocityDataFrame);
            Log.i(MainActivity.TAG, "v1 " + Integer.toString(v1));
            Log.i(MainActivity.TAG, "v2 " + Integer.toString(v2));
        }
    }

    @Override
    public void onItemClick(AdapterView<?> arg0, View arg1, int arg2, long arg3) {
        // TODO Auto-generated method stub
        bluetoothConnection.cancelDiscovery();
        if (bluetoothListAdapter.getItem(arg2).contains("Paired")){
            bluetoothConnection.connectDevice(arg2);
        }
        else {
            Toast.makeText(getApplicationContext(), "Device is not paired", Toast.LENGTH_LONG).show();
        }
    }
}
