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

import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.util.Log;

import com.mytechia.commons.framework.simplemessageprotocol.Command;
import com.mytechia.commons.framework.simplemessageprotocol.MessageFactory;
import com.mytechia.commons.framework.simplemessageprotocol.channel.IBasicCommunicationChannel;
import com.mytechia.commons.framework.simplemessageprotocol.exception.CommunicationException;
import com.mytechia.commons.framework.simplemessageprotocol.exception.MessageFormatException;
import com.mytechia.commons.framework.simplemessageprotocol.exception.TimeoutException;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;

/**
 * @author Julio Gómez
 */
public class AndroidBluetoothSPPChannel implements IBasicCommunicationChannel{

    private static final String TAG_ANDROID_BLUETOOTH_SPPCHANNEL =AndroidBluetoothSPPChannel.class.getName();

    private final BluetoothSocket mmSocket;

    private final BluetoothDevice bluetoothDevice;

    private InputStream mmInStream;

    private OutputStream mmOutStream;

    private MessageFactory messageFactory;

    private boolean closed= false;




    public AndroidBluetoothSPPChannel(BluetoothSocket bluetoothSocket, MessageFactory messageFactory) throws IOException {

        if(bluetoothSocket==null){
            throw new NullPointerException("The parameter bluetoothSocket is required");
        }

        if(messageFactory==null){
            throw new NullPointerException("The parameter messageFactory is required");
        }

        this.registerMessageFactory(messageFactory);

        this.mmSocket= bluetoothSocket;

        this.bluetoothDevice= bluetoothSocket.getRemoteDevice();

    }


    public void connect() throws IOException {


        mmSocket.connect();

        mmInStream= mmSocket.getInputStream();

        mmOutStream = mmSocket.getOutputStream();

        this.closed=false;

    }

    public void close() throws IOException {

        try {
            if(mmSocket!=null) {
                mmSocket.close();
            }

        }finally {


            try {
                if(mmInStream!=null) {
                    mmInStream.close();
                }
            }finally {
                if(mmOutStream!=null) {
                    mmOutStream.close();
                }
            }

            closed=true;
        }

    }

    public boolean isClosed(){
        return closed;
    }



    @Override
    public void send(byte[] data, int offset, int count) throws CommunicationException {

        try {
            mmOutStream.write(data, offset, count);
        } catch (IOException e) {
            throw new CommunicationException(e);
        }

    }

    @Override
    public void send(byte[] data) throws CommunicationException {

        if ((data == null) || (data.length == 0)) {
            Log.d(TAG_ANDROID_BLUETOOTH_SPPCHANNEL, "Not send data. The parameter data is empty or  null");
            return;
        }

        this.send(data, 0, data.length);

    }

    @Override
    public void send(Command msg) throws CommunicationException {

        if(msg==null){
            Log.d(TAG_ANDROID_BLUETOOTH_SPPCHANNEL, "Not send command. The parameter msg is null");
        }

        byte[] encodedMessage= msg.codeMessage();

        this.send(encodedMessage);

    }

    @Override
    public int receive(byte[] data, int offset, int count, long timeout) throws CommunicationException {
        throw new UnsupportedOperationException("Not supported yet.");
    }

    @Override
    public int receive(byte[] data, int offset, int count) throws CommunicationException {

        try {
            int readedBytes= mmInStream.read(data, offset, count);
            return readedBytes;
        } catch (IOException e) {
            throw new CommunicationException(e);
        }
    }

    @Override
    public int receive(byte[] data) throws CommunicationException {

        if((data==null) || (data.length==0)){
            Log.d(TAG_ANDROID_BLUETOOTH_SPPCHANNEL, "The parameter data is empty or  null");
            return 0;
        }

        return this.receive(data, 0, data.length);
    }

    @Override
    public Command receive() throws CommunicationException, MessageFormatException {

        byte[] data = new byte[Command.MAX_MESSAGE_SIZE];

        try {

            this.mmInStream.read(data);

        } catch (IOException ex) {
            throw new CommunicationException(ex);
        }

        return messageFactory.decodeMessage(data);


    }

    @Override
    public Command receive(long timeout) throws CommunicationException {
        throw new UnsupportedOperationException("Not supported yet.");
    }

    @Override
    public void receiveComplete(byte[] data) throws CommunicationException, TimeoutException {
        throw new UnsupportedOperationException("Not supported yet.");
    }

    @Override
    public int sendReceive(byte[] sendData, int sendOffset, int sendCount, byte[] recvData, int recvOffset, int recvCount, long timeout) throws CommunicationException {
        throw new UnsupportedOperationException("Not supported yet.");
    }

    @Override
    public int sendReceive(byte[] sendData, int sendOffset, int sendCount, byte[] recvData, int recvOffset, int recvCount) throws CommunicationException {
        throw new UnsupportedOperationException("Not supported yet.");
    }

    @Override
    public int sendReceive(byte[] sendData, byte[] recvData) throws CommunicationException {
        throw new UnsupportedOperationException("Not supported yet.");
    }

    @Override
    public void registerMessageFactory(MessageFactory messageFactory) {
        if(messageFactory==null){
            throw  new NullPointerException("The parameter messageFactory is required");
        }

        this.messageFactory= messageFactory;
    }
}
