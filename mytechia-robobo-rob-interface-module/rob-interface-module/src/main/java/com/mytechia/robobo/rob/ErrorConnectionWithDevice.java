package com.mytechia.robobo.rob;

import com.mytechia.commons.framework.exception.InternalErrorException;


/**
 * Created by julio on 25/09/17.
 */
public class ErrorConnectionWithDevice extends InternalErrorException {

    private final String nameDevice;

    public ErrorConnectionWithDevice(String message, String nameDevice) {
        super(message);
        this.nameDevice = nameDevice;
    }

    public ErrorConnectionWithDevice(Exception exception, String message, String nameDevice) {
        super(exception, message);
        this.nameDevice = nameDevice;
    }

    public String getNameDevice() {
        return nameDevice;
    }
}
