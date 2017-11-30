package com.example.trobot;

public class BluetoothDataFrame {
    public static final byte CONTROL = 0x00;
    public static final byte MOTOR_VELOCITY = 0x01;
    public static final byte BUMPER = 0x02;

    public static final byte endChar1 = '\n';
    public static final byte endChar2 = '\r';

    private static final int frameTypeIndex = 0;
    private static final int bumperInfoIndex = 1;
    private static final int v1Index = 1;
    private static final int v2Index = 2;

    private byte[] frame;

    public BluetoothDataFrame(byte frameType, byte[] data) {
        // Data frame format: < frameType || data || parity || \n || \r >
        int size = 4 + data.length;
        frame = new byte[size];
        frame[0] = frameType;
        int i = 1;
        for (; i <= data.length; i++) {
            frame[i] = data[i - 1];
        }
        frame[i] = calculateOddParity(frame);
        i++;
        frame[i] = endChar1;
        i++;
        frame[i] = endChar2;
    }

    public BluetoothDataFrame(byte[] frame) {
        this.frame = frame;
    }

    public BluetoothDataFrame(Byte[] frame) {
        this.frame = new byte[frame.length];

        for(int i = 0; i < frame.length; i++) {
            this.frame[i] = frame[i];
        }
    }

    public int getFrameType() {
        return frame[frameTypeIndex];
    }

    public int extractBumperInfo() {
        if (getFrameType() != BUMPER) {
            throw new RuntimeException("Error, the data frame is not of BUMPER type!");
        }
        return frame[bumperInfoIndex];
    }

    public int extractV1Info() {
        if (getFrameType() != MOTOR_VELOCITY) {
            throw new RuntimeException("Error, the data frame is not of MOTOR_VELOCITY type!");
        }
        return frame[v1Index];
    }

    public int extractV2Info() {
        if (getFrameType() != MOTOR_VELOCITY) {
            throw new RuntimeException("Error, the data frame is not of MOTOR_VELOCITY type!");
        }
        return frame[v2Index];
    }

    public byte[] getFrameBytes() {
        return frame;
    }

    public int getParityByteIndex()  {
        byte frameType = frame[frameTypeIndex];

        switch (frameType) {
            case BUMPER:
                return 2;
            case MOTOR_VELOCITY:
                return 3;
            default:
                return -1;
        }
    }

    public boolean verifyOddParity() {
        byte parityByte = frame[getParityByteIndex()];
        byte[] dataToVerify = new byte[frame.length];

        // The frame end bytes do not count towards the parity check
        for(int i = 0; i < frame.length - 2; i++) {
            dataToVerify[i] = frame[i];
        }

        byte calculatedParity = calculateOddParity(dataToVerify);
        return parityByte == calculatedParity;
    }

    public byte calculateOddParity(byte[] data) {
        int sumOfOnes = 0;
        for (byte b : data) {
            sumOfOnes += Integer.bitCount(b);
        }

        // Check if sum is even or odd
        if (sumOfOnes % 2 == 0) {
            return 0;
        } else {
            return 1;
        }
    }
}
