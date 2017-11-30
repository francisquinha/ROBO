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

import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothServerSocket;
import android.bluetooth.BluetoothSocket;
import android.util.Log;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.UUID;

/**
 * @author Julio Gómez
 */
public class PublisherBlueetoothConnection extends Thread{

    public static final  String TAG_PUBLISHER_BLUETOOTH_CONNECTION =PublisherBlueetoothConnection.class.getName();

    private final String applicationName;

    private final UUID uuidConnection;

    private BluetoothServerSocket bluetoothServerSocket;

    private List<ConnectionAcceptedCallback> callbacks= new ArrayList();



    public PublisherBlueetoothConnection(String applicationName, UUID uuidConnection) throws IOException {

        this.applicationName= applicationName;

        this.uuidConnection= uuidConnection;

        BluetoothAdapter defaultAdapter=BluetoothAdapter.getDefaultAdapter();

        if(defaultAdapter==null){
            throw new BluetoothNotSupported("Device does not support Bluetooth");
        }

        bluetoothServerSocket=defaultAdapter.listenUsingRfcommWithServiceRecord(applicationName, uuidConnection);

    }


    public void addReceivedBluetoothConnectionRequestCallback(ConnectionAcceptedCallback callback){

        if(!callbacks.contains(callback)){
            callbacks.add(callback);
        }

    }

    public void removeReceivedBluetoothConnectionRequestCallback(ConnectionAcceptedCallback callback){
        callbacks.remove(callback);
    }




    public BluetoothSocket accept() throws IOException {
        BluetoothSocket bluetoothSocket=bluetoothServerSocket.accept();
        bluetoothServerSocket.close();
        return bluetoothSocket;
    }


    public void close() throws IOException {
        bluetoothServerSocket.close();
    }


    public void run(){

        while (true){

            try {

                BluetoothSocket  bluetoothSocket= this.accept();

                for (ConnectionAcceptedCallback callback : callbacks) {
                    callback.bluetoothConnectionAccepted(bluetoothSocket);
                }

                break;

            } catch (IOException e) {
                Log.w(TAG_PUBLISHER_BLUETOOTH_CONNECTION, e);
                return;
            }

        }
    }


}
