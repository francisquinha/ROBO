package com.mytechia.commons.framework.simplemessageprotocol.bluetooth.android;

import com.mytechia.commons.framework.exception.InternalErrorException;

/**
 * Created by julio on 25/09/17.
 */

public class NotBoundedBluetoothDevice extends InternalErrorException {

    private final String deviceName;

    public NotBoundedBluetoothDevice(String message, String deviceName) {
        super(message);
        this.deviceName= deviceName;
    }

    public String getDeviceName() {
        return deviceName;
    }
}
