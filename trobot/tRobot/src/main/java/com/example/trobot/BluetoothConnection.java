package com.example.trobot;

import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.os.Handler;
import android.os.Message;
import android.util.Log;
import android.widget.ArrayAdapter;
import android.widget.Toast;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.ArrayList;
import java.util.Set;
import java.util.UUID;

public class BluetoothConnection {
    // Bluetooth Stuff
    private final MainActivity mainActivity;
    private BluetoothAdapter btAdapter;
    private Set<BluetoothDevice> devicesArray;
    private ArrayList<String> pairedDevices;
    private ArrayList<BluetoothDevice> devices;
    private IntentFilter filter;
    private BroadcastReceiver receiver;

    public static final UUID MY_UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");

    private ConnectedThread connectedThread = null;

    protected static final int SUCCESS_CONNECT = 0;
    protected static final int MESSAGE_READ = 1;

    public BroadcastReceiver getReceiver() {
        return receiver;
    }

    public BluetoothConnection(MainActivity activity) {
        this.mainActivity = activity;
        btAdapter = BluetoothAdapter.getDefaultAdapter();
        pairedDevices = new ArrayList<String>();
        filter = new IntentFilter(BluetoothDevice.ACTION_FOUND);
        devices = new ArrayList<BluetoothDevice>();
        receiver = new BroadcastReceiver() {

            @Override
            public void onReceive(Context context, Intent intent) {
                // TODO Auto-generated method stub
                String action = intent.getAction();
                if (BluetoothDevice.ACTION_FOUND.equals(action)) {
                    BluetoothDevice device = intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE);
                    devices.add(device);
                    String s = "";
                    for (int j = 0; j < pairedDevices.size(); j++) {
                        if (device.getName().equals(pairedDevices.get(j))) {
                            //append
                            s = "(Paired)";
                            break;
                        }
                    }


                    mainActivity.bluetoothListAdapter.add(device.getName() + " " + s + " " + "\n" + device.getAddress());
                }
                else if (BluetoothAdapter.ACTION_DISCOVERY_STARTED.equals(action)) {

                }
                else if (BluetoothAdapter.ACTION_DISCOVERY_FINISHED.equals(action)) {

                }
                else if (BluetoothAdapter.ACTION_STATE_CHANGED.equals(action)) {
                    if (btAdapter.getState() == btAdapter.STATE_OFF) {
                        turnOnBluetooth();
                    }
                    Log.i(MainActivity.TAG, "Something changed");
                }
                else if (BluetoothDevice.ACTION_ACL_DISCONNECTED.equals(action)) {
                    //Device has disconnected
                    Toast.makeText(mainActivity.getApplicationContext(), "Connection lost", Toast.LENGTH_LONG).show();
                }
            }
        };

        mainActivity.registerReceiver(receiver, filter);
        filter = new IntentFilter(BluetoothAdapter.ACTION_DISCOVERY_STARTED);
        mainActivity.registerReceiver(receiver, filter);
        filter = new IntentFilter(BluetoothAdapter.ACTION_DISCOVERY_FINISHED);
        mainActivity.registerReceiver(receiver, filter);
        filter = new IntentFilter(BluetoothAdapter.ACTION_STATE_CHANGED);
        mainActivity.registerReceiver(receiver, filter);
        filter = new IntentFilter(BluetoothDevice.ACTION_ACL_DISCONNECTED);
        mainActivity.registerReceiver(receiver, filter);

        if (null == btAdapter){
            Toast.makeText(mainActivity.getApplicationContext(), "No bluetooth detected", Toast.LENGTH_LONG).show();
        }
        else {
            if (!btAdapter.isEnabled()) {
                turnOnBluetooth();
            }
            getPairedDevices();
            startDiscovery();
        }
    }

    public void connectDevice(int deviceIndex) {
        BluetoothDevice selectedDevice = devices.get(deviceIndex);
        ConnectThread connect = new ConnectThread(selectedDevice);
        connect.start();
    }

    public void startDiscovery() {
        // TODO Auto-generated method stub
        btAdapter.cancelDiscovery();
        btAdapter.startDiscovery();
    }

    public void cancelDiscovery() {
        if(btAdapter.isDiscovering()) {
            btAdapter.cancelDiscovery();
        }
    }

    public void turnOnBluetooth() {
        Intent intent = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
        mainActivity.startActivityForResult(intent, 1);
    }

    public void getPairedDevices() {
        // TODO Auto-generated method stub
        devicesArray = btAdapter.getBondedDevices();
        if (devicesArray.size() > 0) {
            for (BluetoothDevice device:devicesArray)
                pairedDevices.add(device.getName());
        }
    }

    public boolean writeToConnectedThread(BluetoothDataFrame bluetoothDataFrame) {
        if(connectedThread != null) {
            connectedThread.write(bluetoothDataFrame.getFrameBytes());
            return true;
        }
        else {
            return false;
        }
    }

    public BluetoothDataFrame readFromConnectedThread() {
        byte value = -1;
        ArrayList<Byte> bytes = new ArrayList<Byte>();

        if (connectedThread != null) {
            try {
                if (connectedThread.mmInStream.available() > 0) {
                    do {
                        value = (byte) connectedThread.mmInStream.read();
                        bytes.add(value);
                    }
                    while (bytes.size() < 2 || !bytes.get(bytes.size() - 2).equals(BluetoothDataFrame.endChar1) || !bytes.get(bytes.size() - 1).equals(BluetoothDataFrame.endChar2));
                }
                else {
                    return null;
                }
            } catch (Exception e) {
                e.printStackTrace();
            }
        }

        byte[] frame = new byte[bytes.size()];
        for (int i = 0; i < bytes.size(); i++) {
            frame[i] = bytes.get(i);
        }

        if(frame.length == 0) {
            return null;
        }

        return new BluetoothDataFrame(frame);
    }

    Handler mHandler = new Handler(){
        @Override
        public void handleMessage(Message msg) {
            super.handleMessage(msg);
            switch(msg.what)
            {
                case SUCCESS_CONNECT:
                    connectedThread = new ConnectedThread((BluetoothSocket)msg.obj);
                    Toast.makeText(mainActivity.getApplicationContext(), "Successfully Connected to the Device", Toast.LENGTH_LONG).show();
                    break;
                case MESSAGE_READ:
                    byte[] readBuf = (byte[])msg.obj;
                    String string = new String(readBuf);
                    Log.i(MainActivity.TAG, "------------------------------------------Received ");
                    break;
            }
        }
    };

    private class ConnectThread extends Thread {
        private final BluetoothSocket mmSocket;
        private final BluetoothDevice mmDevice;

        public ConnectThread(BluetoothDevice device) {
            // Use a temporary object that is later assigned to mmSocket,
            // because mmSocket is final
            BluetoothSocket tmp = null;
            mmDevice = device;

            // Get a BluetoothSocket to connect with the given BluetoothDevice
            try {
                // MY_UUID is the app's UUID string, also used by the server code
                tmp = device.createRfcommSocketToServiceRecord(MY_UUID);
            } catch (IOException e) { }
            mmSocket = tmp;
        }

        public void run() {
            // Cancel discovery because it will slow down the connection
            btAdapter.cancelDiscovery();

            try {
                // Connect the device through the socket. This will block
                // until it succeeds or throws an exception
                mmSocket.connect();
            } catch (IOException connectException) {
                // Unable to connect; close the socket and get out
                try {
                    mmSocket.close();
                } catch (IOException closeException) { }
                return;
            }

            // Do work to manage the connection (in a separate thread)
            //manageConnectedSocket(mmSocket);
            mHandler.obtainMessage(SUCCESS_CONNECT,mmSocket).sendToTarget();
        }

        // Will cancel an in-progress connection, and close the socket
        public void cancel() {
            try {
                mmSocket.close();
            } catch (IOException e) { }
        }
    }

    private class ConnectedThread extends Thread {
        private final BluetoothSocket mmSocket;
        private final InputStream mmInStream;
        private final OutputStream mmOutStream;

        public ConnectedThread(BluetoothSocket socket) {
            mmSocket = socket;
            InputStream tmpIn = null;
            OutputStream tmpOut = null;

            // Get the input and output streams, using temp objects because
            // member streams are final
            try {
                tmpIn = socket.getInputStream();
                tmpOut = socket.getOutputStream();
            } catch (IOException e) { }

            mmInStream = tmpIn;
            mmOutStream = tmpOut;
        }

        public void run() {
            byte[] buffer;  // buffer store for the stream
            int bytes; // bytes returned from read()

            // Keep listening to the InputStream until an exception occurs
            while (true) {
                try {
                    // Read from the InputStream
                    buffer = new byte[1];
                    bytes = mmInStream.read(buffer);
                    // Send the obtained bytes to the UI activity
                    mHandler.obtainMessage(MESSAGE_READ, bytes, -1, buffer).sendToTarget();
                    Log.i(MainActivity.TAG, "------------------------------------------Received ");

                } catch (IOException e) {
                    break;
                }
            }
        }

        // Call this from the main activity to send data to the remote device
        public void write(byte[] bytes) {
            try {
                mmOutStream.write(bytes);
            } catch (IOException e) {
            }
        }

        // Call this from the main activity to shutdown the connection
        public void cancel() {
            try {
                mmSocket.close();
            } catch (IOException e) { }
        }
    }

}
