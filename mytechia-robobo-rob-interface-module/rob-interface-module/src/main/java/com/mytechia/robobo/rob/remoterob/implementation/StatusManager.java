/*******************************************************************************
 * Copyright 2016 Mytech Ingenieria Aplicada <http://www.mytechia.com>
 * Copyright 2016 Gervasio Varela <gervasio.varela@mytechia.com>
 * <p>
 * This file is part of Robobo Remote Control Module.
 * <p>
 * Robobo Remote Control Module is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * <p>
 * Robobo Remote Control Module is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 * <p>
 * You should have received a copy of the GNU Lesser General Public License
 * along with Robobo Remote Control Module.  If not, see <http://www.gnu.org/licenses/>.
 ******************************************************************************/

package com.mytechia.robobo.rob.remoterob.implementation;


import com.mytechia.robobo.framework.remote_control.remotemodule.IRemoteControlModule;
import com.mytechia.robobo.framework.remote_control.remotemodule.Status;

import com.mytechia.robobo.rob.BatteryStatus;
import com.mytechia.robobo.rob.GapStatus;
import com.mytechia.robobo.rob.IRSensorStatus;
import com.mytechia.robobo.rob.LedStatus;
import com.mytechia.robobo.rob.MotorStatus;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Iterator;
import java.util.NoSuchElementException;

/**
 * Manager to control the status production and sending
 */
public class StatusManager {

    private String TAG = "RemoteRob:StatusManager";

    private static final long PT_STATUS_PERIOD = 500;
    private static final long WHEEL_STATUS_PERIOD = 500;
    private static final long BATTERY_STATUS_PERIOD = 30000;

    private static final int MIN_IR_CHANGE_TO_STATUS = 15;

    private long lastWheelStatusTime = 0;
    private long lastPanStatusTime = 0;
    private long lastTiltStatusTime = 0;
    private long lastBatteryStatusTime = 0;
    private int lastWheelPosR = 0;
    private int lastWheelPosL = 0;
    private int lastPanPos = 0;
    private int lastTiltPos = 0;

    private ArrayList<Boolean> lastGaps ;
    private ArrayList<Boolean> lastObstacles;
    private ArrayList<Integer> lastIRs;



    private IRemoteControlModule rcmodule;


    /**
     * Public constructor
     * @param rcmodule Remote control module to send status
     */
    public StatusManager(IRemoteControlModule rcmodule) {
        this.rcmodule = rcmodule;
    }


    private boolean timeout(long lastTime, long period) {
        return (lastTime + period) < System.currentTimeMillis();
    }

    /**
     * Sends motor status periodically
     * @param left Left motor status
     * @param right Right motor status
     */
    public void sendWheelsStatus(MotorStatus left, MotorStatus right) {

        if (timeout(this.lastWheelStatusTime, WHEEL_STATUS_PERIOD)) {

            this.lastWheelStatusTime = System.currentTimeMillis();
            if ((left.getVariationAngle() != lastWheelPosL)||right.getVariationAngle() != lastWheelPosR) {
                lastWheelPosL = left.getVariationAngle();
                lastWheelPosR = right.getVariationAngle();
                Status s = new Status("WHEELS");
                s.putContents("wheelPosR", String.valueOf(right.getVariationAngle()));
                s.putContents("wheelPosL", String.valueOf(left.getVariationAngle()));
                s.putContents("wheelSpeedR", String.valueOf(right.getAngularVelocity()));
                s.putContents("wheelSpeedL", String.valueOf(left.getAngularVelocity()));
                rcmodule.postStatus(s);
            }

        }

    }

    /**
     * Sends Pan status periodically
     * @param status Pan status
     */
    public void sendPanStatus(MotorStatus status) {

        if (timeout(this.lastPanStatusTime, PT_STATUS_PERIOD)) {

            this.lastPanStatusTime = System.currentTimeMillis();
            if (status.getVariationAngle() != lastPanPos) {
                lastPanPos = status.getVariationAngle();
                Status s = new Status("PAN");
                s.putContents("panPos", String.valueOf(status.getVariationAngle()));
                rcmodule.postStatus(s);
            }

        }

    }

    /**
     * Sends Tilt status periodically
     * @param status Tilt status
     */
    public void sendTiltStatus(MotorStatus status) {

        if (timeout(this.lastTiltStatusTime, PT_STATUS_PERIOD)) {

            this.lastTiltStatusTime = System.currentTimeMillis();
            if (status.getVariationAngle() != lastTiltPos) {
                lastTiltPos = status.getVariationAngle();
                Status s = new Status("TILT");
                s.putContents("tiltPos", String.valueOf(status.getVariationAngle()));
                rcmodule.postStatus(s);
            }

        }

    }

    /**
     * Sends Battery status periodically
     * @param batteryStatus battery status
     */
    public void sendBatteryStatus(BatteryStatus batteryStatus) {

        if (timeout(this.lastBatteryStatusTime, BATTERY_STATUS_PERIOD)) {

            this.lastBatteryStatusTime = System.currentTimeMillis();

            Status s  = new Status("BAT-BASE");
            s.putContents("level",String.valueOf(batteryStatus.getBattery()));
            rcmodule.postStatus(s);

        }

    }

    /**
     * Sends gap status periodically
     * @param gaps gap status
     */
    public void sendGapsStatus(Collection<GapStatus> gaps) {

        if (this.lastGaps == null) { //first time ever
            sendGapsMessage(gaps);
            int i = 0;
            lastGaps= new ArrayList<Boolean>();
            for (GapStatus gap : gaps){
                lastGaps.add(i,gap.isGap());
                i = i+1;
            }

        }
        else {

            if (gapsChanged(gaps)) {
                sendGapsMessage(gaps);
                int i = 0;
                for (GapStatus gap : gaps){
                    lastGaps.add(i,gap.isGap());
                    i = i+1;
                }
            }

        }

        //update last gaps information
        //sendGapsMessage(gaps);


    }

    /**
     * Returns true if the gaps changed
     * @param newGaps Current gap status
     * @return true if gaps changed
     */
    public boolean gapsChanged(Collection<GapStatus> newGaps) {

        Iterator<GapStatus> newIterator = newGaps.iterator();

        try {
            int i = 0;
            while (newIterator.hasNext()) {

                GapStatus newGap = newIterator.next();
                if ((lastGaps.get(i)) ^ (newGap.isGap())) {
                    return true;
                }
            }
        }
        catch(NoSuchElementException ex) {
            return true;
        }

        return false;


    }

    /**
     * Sends the gaps status immediately
     * @param gaps Gap status array
     */
    public void sendGapsMessage(Collection<GapStatus> gaps) {
        Status s  = new Status("GAPSTATUS");
        for (GapStatus status:gaps){
            s.putContents(gapIndexToString(status.getId()),String.valueOf(status.isGap()));
        }
        rcmodule.postStatus(s);
    }


    /**
     * Checks whether IR values has changed significally since last update
     * @param irSensorStatus Collection of ir sensor status
     * @return true if it changed
     */
    private boolean checkIRsChanged(Collection<IRSensorStatus> irSensorStatus) {

        boolean changed = false;

        if (lastIRs == null) {
            lastIRs = new ArrayList<>();
        }

        if (lastIRs.size() != irSensorStatus.size()) {
            changed = true;
        }
        else {
            int i=0;
            for(IRSensorStatus ir : irSensorStatus) {
                if (Math.abs(ir.getDistance() - lastIRs.get(i))
                        > MIN_IR_CHANGE_TO_STATUS)
                {
                    changed = true;
                    break;
                }
                i++;
            }
        }

        if (changed) {
            updateLastIRs(irSensorStatus);
        }

        return changed;

    }

    private void updateLastIRs(Collection<IRSensorStatus> irSensorStatus) {
        this.lastIRs.clear();
        for(IRSensorStatus ir : irSensorStatus) {
            this.lastIRs.add(ir.getDistance());
        }
    }


    /**
     * Sends ir sensor status if it has changed
     * @param irSensorStatus collection of ir status
     */
    public void sendIrStatus(Collection<IRSensorStatus> irSensorStatus) {

        if (checkIRsChanged(irSensorStatus)) {
            sendIrStatusMessage(irSensorStatus);
        }

    }

    /**
     * Sends ir sensor status
     * @param irSensorStatus collection of ir status
     */
    public void sendIrStatusMessage(Collection<IRSensorStatus> irSensorStatus) {
        Status s  = new Status("IRS");
        for (IRSensorStatus status : irSensorStatus){
            s.putContents(irIndexToString(status.getId()),String.valueOf(status.getDistance()));
        }
        rcmodule.postStatus(s);
    }

    public void sendObstaclesMessage(ArrayList<Boolean> irSensorStatus) {
        Status s  = new Status("OBSTACLES");
        int i = 0;
        for (IRSensorStatus.IRSentorStatusId status : IRSensorStatus.IRSentorStatusId.values()){
            s.putContents(irIndexToString(status),irSensorStatus.get(i).toString());
            i = i+1;
        }
        rcmodule.postStatus(s);
    }


    private ArrayList<Boolean> calcObstacles(Collection<IRSensorStatus> irSensorStatus) {

        //TODO CAMBIAR IR SENSOR STATUS ACORDE AL NUEVO ROB
        ArrayList<Boolean> obstacles = new ArrayList<Boolean>(irSensorStatus.size()-1);

        for(IRSensorStatus ir : irSensorStatus) {

            obstacles.add(ir.getDistance() > MIN_IR_CHANGE_TO_STATUS);

        }

        return obstacles;

    }

    /**
     * Sends current led status
     * @param led Led status
     */
    public void sendLedStatus(LedStatus led){
        Status s = new Status("LED");
        s.putContents("id",ledIndexToString(led.getId().ordinal()+1));
        int [] color = led.getColor();
        s.putContents("R",color[0]+"");
        s.putContents("G",color[1]+"");
        s.putContents("B",color[2]+"");

        rcmodule.postStatus(s);

    }

    public String irIndexToString(IRSensorStatus.IRSentorStatusId index){
        switch (index){
            case IRSensorStatus1:
                return  "Front-LL";
            case IRSensorStatus2:
                return  "Front-L";
            case IRSensorStatus3:
                return  "Front-C";
            case IRSensorStatus4:
                return  "Front-R";
            case IRSensorStatus5:
                return  "Front-RR";
            case IRSensorStatus6:
                return  "Back-R";
            case IRSensorStatus7:
                return  "Back-C";
            case IRSensorStatus8:
                return  "Back-L";
            default:
                return null;

        }
    }


    public String ledIndexToString(int index){
        switch (index){
            case 1:
                return  "Front-LL";
            case 2:
                return  "Front-L";
            case 3:
                return  "Front-C";
            case 4:
                return  "Front-R";
            case 5:
                return  "Front-RR";
            case 6:
                return  "Back-R";
            case 7:
                return  "Back-L";
            default:
                return null;

        }
    }
    public String gapIndexToString(GapStatus.GapStatusId index){
        switch (index){
            case Gap1:
                return  "Front-L";
            case Gap2:
                return  "Front-R";
            case Gap3:
                return  "Back-R";
            case Gap4:
                return  "Back-L";
            default:
                return null;

        }
    }


}