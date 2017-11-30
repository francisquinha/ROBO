/*******************************************************************************
 *
 *   Copyright 2016 Mytech Ingenieria Aplicada <http://www.mytechia.com>
 *   Copyright 2016 Gervasio Varela <gervasio.varela@mytechia.com>
 *   Copyright 2016 Julio Gómez <julio.gomez@mytechia.com>
 *
 *   This file is part of Robobo ROB Interface Module.
 *
 *   Robobo ROB Interface Module is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU Lesser General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   Robobo ROB Interface Module is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with Robobo ROB Interface Module.  If not, see <http://www.gnu.org/licenses/>.
 *
 ******************************************************************************/

package com.mytechia.commons.framework.simplemessageprotocol.bluetooth.android;

import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;

import java.util.ArrayList;
import java.util.List;

import static android.bluetooth.BluetoothAdapter.ACTION_DISCOVERY_FINISHED;
import static android.bluetooth.BluetoothAdapter.ACTION_REQUEST_DISCOVERABLE;
import static android.bluetooth.BluetoothAdapter.EXTRA_DISCOVERABLE_DURATION;
import static android.bluetooth.BluetoothAdapter.EXTRA_PREVIOUS_SCAN_MODE;
import static android.bluetooth.BluetoothAdapter.EXTRA_SCAN_MODE;
import static android.bluetooth.BluetoothDevice.ACTION_FOUND;

/**
 * @author Julio Gómez
 */
public class DiscovererBluetoothDevices {


    private final BluetoothAdapter defaultAdapter;

    private final Activity activity;

    private BroadcastReceiver mReceiver;

    private final List<DiscoverOtherBluetoothDeviceCallback> callbacks= new ArrayList<>();



    public DiscovererBluetoothDevices(Activity activity){

        if(activity==null){
            throw new NullPointerException("The parameter activity is required");
        }

        this.activity= activity;

        defaultAdapter=BluetoothAdapter.getDefaultAdapter();

        if(defaultAdapter==null){
            throw new BluetoothNotSupported("Device does not support Bluetooth");
        }


        this.mReceiver= new BroadcastReceiverDiscoveryBluetooth();

        IntentFilter filterActionFound = new IntentFilter(ACTION_FOUND);
        activity.registerReceiver(this.mReceiver, filterActionFound);

        IntentFilter  filterDiscoveryFinished = new IntentFilter(ACTION_DISCOVERY_FINISHED);
        activity.registerReceiver(mReceiver, filterDiscoveryFinished);


    }



    public void addDiscoverOtherBluetoothDeviceCallback(DiscoverOtherBluetoothDeviceCallback callback){

        if(!callbacks.contains(callback)){
            callbacks.add(callback);
        }

    }

    public void removeDiscoverOtherBluetoothDeviceCallback(DiscoverOtherBluetoothDeviceCallback callback){
        callbacks.remove(callback);
    }


    public void startDiscovery() {
        boolean startedDiscovery=defaultAdapter.startDiscovery();

        if (startedDiscovery){

            for (DiscoverOtherBluetoothDeviceCallback callback : callbacks) {
                callback.startDiscovering();
            }
        }
    }

    public void cancelDiscovering(){
        defaultAdapter.cancelDiscovery();
    }


    private Intent createDiscoverableIntent(long time){

        Intent discoverableIntent = new Intent(ACTION_REQUEST_DISCOVERABLE);

        discoverableIntent.putExtra(EXTRA_DISCOVERABLE_DURATION, time);

        return discoverableIntent;

    }

    public void makeDiscoverable(long time){

        this.activity.startActivity(createDiscoverableIntent(time));

    }



    private class BroadcastReceiverDiscoverableMode extends BroadcastReceiver {

        @Override
        public void onReceive(Context context, Intent intent) {

            //Posibles valores
            //SCAN_MODE_CONNECTABLE_DISCOVERABLE
            // SCAN_MODE_CONNECTABLE -> No es descubrible pero aun es capaz de recibir conexiones
            //SCAN_MODE_NONE
            intent.getBundleExtra(EXTRA_SCAN_MODE);

            intent.getBundleExtra(EXTRA_PREVIOUS_SCAN_MODE);


        }
    }


    private class BroadcastReceiverDiscoveryBluetooth extends BroadcastReceiver {

        @Override
        public void onReceive(Context context, Intent intent) {

            String intentAction = intent.getAction();

            if (ACTION_FOUND.equals(intentAction)) {

                BluetoothDevice device = intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE);

                for (DiscoverOtherBluetoothDeviceCallback callback : callbacks) {
                    callback.discovered(device);
                }

            } else if (ACTION_DISCOVERY_FINISHED.equals(intentAction)) {
                for (DiscoverOtherBluetoothDeviceCallback callback : callbacks) {
                    callback.endDiscovering();
                }
            }

        }
    }


    public void free(){
        cancelDiscovering();
        activity.unregisterReceiver(this.mReceiver);
    }



}
