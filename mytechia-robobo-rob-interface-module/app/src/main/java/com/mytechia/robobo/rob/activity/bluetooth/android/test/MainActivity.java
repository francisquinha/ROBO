package com.mytechia.robobo.rob.activity.bluetooth.android.test;

import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.view.View;
import android.view.Menu;
import android.view.MenuItem;
import android.widget.AdapterView;
import android.widget.CheckedTextView;
import android.widget.GridView;
import android.widget.ListView;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.UUID;

import com.mytechia.robobo.rob.activity.bluetooth.android.AndroidBluetoothSPPChannel;
import com.mytechia.robobo.rob.DefaultRob;
import com.mytechia.robobo.rob.comm.RoboCommandFactory;
import com.mytechia.robobo.rob.comm.SmpRobComm;

import org.slf4j.LoggerFactory;

import ch.qos.logback.classic.Level;
import ch.qos.logback.classic.Logger;
import ch.qos.logback.classic.LoggerContext;
import ch.qos.logback.classic.encoder.PatternLayoutEncoder;
import ch.qos.logback.core.ConsoleAppender;

public class MainActivity extends AppCompatActivity {

    private ListView luminare360List;

    private List<BluetoothDevice> bluetoothDevices = new ArrayList<BluetoothDevice>();

    private BluetoothListAdapter bluetoothListAdapter;

    private CheckedTextView checkedView;

    private static final UUID UUID_BLUETOOTH_CONNECTION = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");

    //private static final UUID UUID_BLUETOOTH_CONNECTION= UUID.fromString("f292db4f-026e-406d-95a4-5e44e4d02691");

    private BluetoothDevice actualBluetoothDevice;

    private AndroidBluetoothSPPChannel androidBluetoothSPPChannel;

    private  BluetoothSocket mmSocket;

    private RoboCommandFactory roboCommandFactory;

    private GridView gridview;

    private DefaultRob defaultRob;

    private SmpRobComm smpRoboCom;


    @Override
    protected void onCreate(Bundle savedInstanceState) {

        super.onCreate(savedInstanceState);

        configureLogback();

        setContentView(R.layout.activity_main);

        roboCommandFactory=  new RoboCommandFactory();

        this.luminare360List = (ListView) findViewById(R.id.listBluetoothDevice);

        this.luminare360List.setChoiceMode(ListView.CHOICE_MODE_MULTIPLE);

        bluetoothListAdapter = new BluetoothListAdapter(this, R.layout.bluetooth_device_list, bluetoothDevices);

        luminare360List.setAdapter(bluetoothListAdapter);

        luminare360List.setOnItemClickListener(new AdapterView.OnItemClickListener() {

            @Override
            public void onItemClick(AdapterView<?> adapterView, View view, int i, long l) {

                if (checkedView != null) {
                    checkedView.setChecked(false);
                }

                checkedView = ((CheckedTextView) view.findViewById(R.id.checkedBluetoothDevice));

                checkedView.setChecked(true);

                BluetoothDevice bluetoothDevice = (BluetoothDevice) adapterView.getItemAtPosition(i);

                connect(bluetoothDevice);

            }
        });

        BluetoothAdapter mBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();

        Set<BluetoothDevice> pairedDevices=mBluetoothAdapter.getBondedDevices();

        bluetoothListAdapter.addAll(pairedDevices);

        this.gridview = (GridView)findViewById(R.id.gridRoboCommands);

        this.gridview.setAdapter(new ButtonAdapter(this));


    }

    private void configureLogback(){

        LoggerContext loggerContext = (LoggerContext) LoggerFactory.getILoggerFactory();

        loggerContext.reset();


        Logger root = loggerContext.getLogger(Logger.ROOT_LOGGER_NAME);


        PatternLayoutEncoder patternLayout = new PatternLayoutEncoder();
        patternLayout.setContext(loggerContext);
        patternLayout.setPattern("%d{dd/MM/yyyy-HH:mm:ss} %-5p: %-20c{1} - %m%n");
        patternLayout.start();

        ConsoleAppender consoleAppender= new ConsoleAppender();
        consoleAppender.setName("default-console");
        consoleAppender.setEncoder(patternLayout);
        consoleAppender.start();

        root.setLevel(Level.TRACE);

        root.addAppender(consoleAppender);

        /*
        FileAppender<ILoggingEvent> fileAppender = new FileAppender<ILoggingEvent>();
        fileAppender.setContext(loggerContext);

        String path= System.getProperty("java.io.tmpdir");

        File file= new File(path+File.separator+"simplemessageprotocol.log");

        fileAppender.setFile(file.toString());
        fileAppender.setEncoder(patternLayout);
        fileAppender.setAppend(false);
        fileAppender.start();*/


    }

    private void connect(BluetoothDevice bluetoothDevice){

        if(this.actualBluetoothDevice!=null){

            if(androidBluetoothSPPChannel!=null) {

                try {
                    androidBluetoothSPPChannel.close();
                } catch (IOException e) {
                    e.printStackTrace();
                }

            }
        }

        this.actualBluetoothDevice= bluetoothDevice;

        try {
            BluetoothSocket mmSocket = bluetoothDevice.createRfcommSocketToServiceRecord(UUID_BLUETOOTH_CONNECTION);

            if (smpRoboCom != null) {
                smpRoboCom.stop();
            }

            this.androidBluetoothSPPChannel = new AndroidBluetoothSPPChannel(mmSocket, roboCommandFactory);

            androidBluetoothSPPChannel.connect();

            this.smpRoboCom = new SmpRobComm(androidBluetoothSPPChannel, roboCommandFactory);

            this.defaultRob = new DefaultRob(smpRoboCom);

            this.smpRoboCom.start();


            ButtonAdapter buttonAdapter = (ButtonAdapter) this.gridview.getAdapter();

            buttonAdapter.setDefaultRob(defaultRob, smpRoboCom);



        } catch (IOException e) {
            e.printStackTrace();
        }


    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.menu_main, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        // Handle action bar item clicks here. The action bar will
        // automatically handle clicks on the Home/Up button, so long
        // as you specify a parent activity in AndroidManifest.xml.
        int id = item.getItemId();

        //noinspection SimplifiableIfStatement
        if (id == R.id.action_settings) {
            return true;
        }

        return super.onOptionsItemSelected(item);
    }
}
