/*******************************************************************************
 * Copyright 2016 Mytech Ingenieria Aplicada <http://www.mytechia.com>
 * Copyright 2016 Luis Llamas <luis.llamas@mytechia.com>
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

import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.os.BatteryManager;
import android.util.Log;

import com.mytechia.commons.framework.exception.InternalErrorException;
import com.mytechia.commons.framework.simplemessageprotocol.exception.CommunicationException;
import com.mytechia.robobo.framework.LogLvl;
import com.mytechia.robobo.framework.RoboboManager;
import com.mytechia.robobo.rob.remoterob.IRemoteRobModule;
import com.mytechia.robobo.framework.remote_control.remotemodule.Command;
import com.mytechia.robobo.framework.remote_control.remotemodule.ICommandExecutor;
import com.mytechia.robobo.framework.remote_control.remotemodule.IRemoteControlModule;
import com.mytechia.robobo.framework.remote_control.remotemodule.Status;

import com.mytechia.robobo.rob.BatteryStatus;
import com.mytechia.robobo.rob.FallStatus;
import com.mytechia.robobo.rob.GapStatus;
import com.mytechia.robobo.rob.IRSensorStatus;
import com.mytechia.robobo.rob.IRob;
import com.mytechia.robobo.rob.IRobErrorListener;
import com.mytechia.robobo.rob.IRobInterfaceModule;
import com.mytechia.robobo.rob.IRobStatusListener;
import com.mytechia.robobo.rob.LedStatus;
import com.mytechia.robobo.rob.MotorStatus;


import com.mytechia.robobo.rob.RobMotorEnum;
import com.mytechia.robobo.rob.WallConnectionStatus;
import com.mytechia.robobo.rob.movement.IRobMovementModule;
import com.mytechia.robobo.util.Color;

import java.util.Collection;
import java.util.HashMap;
import java.util.Timer;
import java.util.TimerTask;

public class RemoteRobModuleImplementation implements IRemoteRobModule {

    private IRemoteControlModule rcmodule;
    private IRob irob;
    private IRobMovementModule movementModule;
    private String TAG = "RemoteRob";
    private Context context;
    private int lastPanPosition = 0;
    private int lastTiltPosition = 0;
    private PanWaitThread panThread;
    private TiltWaitThread tiltThread;
    private StatusManager statusManager;
    private RoboboManager roboboManager;

    @Override
    public void startup(final RoboboManager roboboManager) throws InternalErrorException {

        context = roboboManager.getApplicationContext();

        rcmodule = roboboManager.getModuleInstance(IRemoteControlModule.class);

        movementModule = roboboManager.getModuleInstance(IRobMovementModule.class);

        irob = roboboManager.getModuleInstance(IRobInterfaceModule.class).getRobInterface();

        this.roboboManager = roboboManager;

        this.statusManager = new StatusManager(rcmodule);

        irob.setOperationMode((byte) 1);

        irob.setRobStatusPeriod(100);


        irob.addRobErrorListener(new IRobErrorListener() {
            @Override
            public void robError(CommunicationException ex) {

                Log.e(TAG, "Error communication with Rob", ex);

                roboboManager.notifyModuleError(ex);
            }
        });

        irob.addRobStatusListener(new IRobStatusListener() {

            @Override
            public void statusMotorsMT(MotorStatus left, MotorStatus right) {
                statusManager.sendWheelsStatus(left, right);

            }

            @Override
            public void statusMotorPan(MotorStatus status) {
                if (status.getVariationAngle() != lastPanPosition) {
                    lastPanPosition = status.getVariationAngle();
                    statusManager.sendPanStatus(status);
                }
            }

            @Override
            public void statusMotorTilt(MotorStatus status) {
                if (status.getVariationAngle() != lastTiltPosition) {
                    lastTiltPosition = status.getVariationAngle();
                    statusManager.sendTiltStatus(status);
                }
            }

            @Override
            public void statusGaps(Collection<GapStatus> gaps) {
                statusManager.sendGapsStatus(gaps);
            }

            @Override
            public void statusFalls(Collection<FallStatus> fall) {
                Status s = new Status("FALLSTATUS");
                for (FallStatus status : fall) {
                    s.putContents(status.getId().toString(), String.valueOf(status.isFall()));
                }
                //rcmodule.postStatus(s); --> NEVER SENT
            }

            @Override
            public void statusIRSensorStatus(Collection<IRSensorStatus> irSensorStatus) {
                statusManager.sendIrStatus(irSensorStatus);
            }

            @Override
            public void statusBattery(BatteryStatus battery) {
                statusManager.sendBatteryStatus(battery);
            }

            @Override
            public void statusWallConnectionStatus(WallConnectionStatus wallConnectionStatus) {

            }


            @Override
            public void statusLeds(LedStatus led) {
                statusManager.sendLedStatus(led);

            }
        });


        rcmodule.registerCommand("MOVEBY-DEGREES", new ICommandExecutor() {
            @Override
            public void executeCommand(Command c, IRemoteControlModule rcmodule) {
                HashMap<String, String> par = c.getParameters();
                String wheel = par.get("wheel");
                int degrees = Math.abs(Integer.parseInt(par.get("degrees")));
                int speed = Integer.parseInt(par.get("speed"));
                RemoteRobModuleImplementation.this.roboboManager.log(LogLvl.TRACE, TAG, "MOVEBY-DEGREES Degrees: " + degrees + " Speed: " + speed);


                if (wheel.equals("right")) {
                    //FF
                    try {

                        irob.moveMT(speed, degrees, 0, 0);
                    } catch (CommunicationException e) {
                        Log.e(TAG, "Error movement", e);
                        roboboManager.notifyModuleError(e);

                    }

                } else if (wheel.equals("left")) {
                    //FF
                    try {
                        irob.moveMT(0, 0, speed, degrees);
                    } catch (CommunicationException e) {
                        Log.e(TAG, "Error movement", e);
                        roboboManager.notifyModuleError(e);
                    }


                } else if (wheel.equals("both")) {

                    try {
                        irob.moveMT(speed, degrees, speed, degrees);
                    } catch (CommunicationException e) {
                        Log.e(TAG, "Error movement", e);
                        roboboManager.notifyModuleError(e);
                    }


                }
            }
        });

        rcmodule.registerCommand("MOVEBY-TIME", new ICommandExecutor() {
            @Override
            public void executeCommand(Command c, IRemoteControlModule rcmodule) {
                HashMap<String, String> par = c.getParameters();
                String wheel = par.get("wheel");
                int time = Math.round(Float.parseFloat(par.get("time")) * 1000);
                int speed = Integer.parseInt(par.get("speed"));
                RemoteRobModuleImplementation.this.roboboManager.log(LogLvl.TRACE, TAG, "MOVEBY-TIME Time: " + time + " Speed: " + speed);


                if (wheel.equals("right")) {
                    try {
                        irob.moveMT(speed, 0, time);
                    } catch (CommunicationException e) {
                        Log.e(TAG, "Error movement", e);
                        roboboManager.notifyModuleError(e);
                    }


                } else if (wheel.equals("left")) {

                    try {
                        irob.moveMT(0, speed, time);
                    } catch (CommunicationException e) {
                        Log.e(TAG, "Error movement", e);
                        roboboManager.notifyModuleError(e);
                    }


                } else if (wheel.equals("both")) {

                    try {
                        irob.moveMT(speed, speed, time);
                    } catch (CommunicationException e) {
                        Log.e(TAG, "Error movement", e);

                        roboboManager.notifyModuleError(e);
                    }

                }
            }
        });

        rcmodule.registerCommand("TURNINPLACE", new ICommandExecutor() {
            @Override
            public void executeCommand(Command c, IRemoteControlModule rcmodule) {
                HashMap<String, String> par = c.getParameters();
                int degrees = Integer.parseInt(par.get("degrees"));
                RemoteRobModuleImplementation.this.roboboManager.log(LogLvl.TRACE, TAG, "TURNINPLACE Degrees: " + degrees);

                try {
                    irob.moveMT(50, degrees, 50, degrees);

                } catch (CommunicationException e) {
                    Log.e(TAG, "Error movement", e);
                    roboboManager.notifyModuleError(e);
                }


            }
        });

        rcmodule.registerCommand("MOVE", new ICommandExecutor() {
            @Override
            public void executeCommand(Command c, IRemoteControlModule rcmodule) {
                HashMap<String, String> par = c.getParameters();
                int time = Math.round(Float.parseFloat(par.get("time")) * 1000);
                int lspeed = Integer.parseInt(par.get("lspeed"));
                int rspeed = Integer.parseInt(par.get("rspeed"));
                RemoteRobModuleImplementation.this.roboboManager.log(LogLvl.TRACE, TAG, "MOVE Left: " + lspeed + " Right: " + rspeed);


                //FF - BIEN
                try {
                    irob.moveMT(rspeed, lspeed, time);
                } catch (CommunicationException e) {
                    Log.e(TAG, "Error movement", e);
                    roboboManager.notifyModuleError(e);
                }


            }
        });

        rcmodule.registerCommand("MOVE-FOREVER", new ICommandExecutor() {
            @Override
            public void executeCommand(Command c, IRemoteControlModule rcmodule) {
                HashMap<String, String> par = c.getParameters();
                String rmotor = par.get("rmotor");
                String lmotor = par.get("lmotor");
                int speed = Integer.parseInt(par.get("speed"));
                RemoteRobModuleImplementation.this.roboboManager.log(LogLvl.TRACE, TAG, "MOVE-FOREVER Left: " + lmotor + " Right: " + rmotor);


                try {
                    irob.moveMT(speed, speed, Integer.MAX_VALUE);
                } catch (CommunicationException e) {
                    Log.e(TAG, "Error movement", e);
                    roboboManager.notifyModuleError(e);
                }


            }
        });

        rcmodule.registerCommand("MOVEPAN", new ICommandExecutor() {
            @Override
            public void executeCommand(Command c, IRemoteControlModule rcmodule) {
                HashMap<String, String> par = c.getParameters();
                int pos = Integer.parseInt(par.get("pos"));
                int speed = Integer.parseInt(par.get("speed"));
                RemoteRobModuleImplementation.this.roboboManager.log(LogLvl.TRACE, TAG, "MOVEPAN Degrees: " + pos + " Speed: " + speed);

                try {
                    irob.movePan((short) speed, pos);
                } catch (CommunicationException e) {
                    Log.e(TAG, "Error movement", e);
                    roboboManager.notifyModuleError(e);
                }

            }
        });

        rcmodule.registerCommand("MOVETILT", new ICommandExecutor() {
            @Override
            public void executeCommand(Command c, IRemoteControlModule rcmodule) {
                HashMap<String, String> par = c.getParameters();
                int pos = Integer.parseInt(par.get("pos"));
                int speed = Integer.parseInt(par.get("speed"));
                RemoteRobModuleImplementation.this.roboboManager.log(LogLvl.TRACE, TAG, "MOVETILT Degrees: " + pos + " Speed: " + speed);


                try {
                    irob.moveTilt((short) speed, pos);
                } catch (CommunicationException e) {
                    Log.e(TAG, "Error movement", e);
                    roboboManager.notifyModuleError(e);
                }
            }
        });

        rcmodule.registerCommand("MOVEPAN-BLOCKING", new ICommandExecutor() {
            @Override
            public void executeCommand(Command c, IRemoteControlModule rcmodule) {
                HashMap<String, String> par = c.getParameters();
                int pos = Integer.parseInt(par.get("pos"));
                int speed = Integer.parseInt(par.get("speed"));
                int blockid = Integer.parseInt(par.get("blockid"));
                RemoteRobModuleImplementation.this.roboboManager.log(LogLvl.TRACE, TAG, "MOVEPANBLOCK Degrees: " + pos + " Speed: " + speed);

                try {
                    irob.movePan((short) speed, pos);
                } catch (CommunicationException e) {
                    Log.e(TAG, "Error movement", e);
                    roboboManager.notifyModuleError(e);
                }
                if (panThread != null) {
                    if (!panThread.isInterrupted()) {
                        panThread.interrupt();
                    }
                }
                panThread = new PanWaitThread(blockid, pos, lastPanPosition);
                panThread.run();

            }
        });

        rcmodule.registerCommand("MOVETILT-BLOCKING", new ICommandExecutor() {
            @Override
            public void executeCommand(Command c, IRemoteControlModule rcmodule) {
                HashMap<String, String> par = c.getParameters();
                int pos = Integer.parseInt(par.get("pos"));
                int speed = Integer.parseInt(par.get("speed"));
                int blockid = Integer.parseInt(par.get("blockid"));
                RemoteRobModuleImplementation.this.roboboManager.log(LogLvl.TRACE, TAG, "MOVTILTBLOCK Degrees: " + pos + " Speed: " + speed);


                try {
                    irob.moveTilt((short) speed, pos);
                } catch (CommunicationException e) {
                    Log.e(TAG, "Error movement", e);
                    roboboManager.notifyModuleError(e);
                }

                if (tiltThread != null) {
                    if (!tiltThread.isInterrupted()) {
                        tiltThread.interrupt();
                    }
                }
                tiltThread = new TiltWaitThread(blockid, pos, lastTiltPosition);
                tiltThread.run();
            }
        });

        rcmodule.registerCommand("SET-LEDCOLOR", new ICommandExecutor() {
            @Override
            public void executeCommand(Command c, IRemoteControlModule rcmodule) {
                HashMap<String, String> par = c.getParameters();
                String led = ledToIndex(par.get("led"));
                int ledint = 0;
                Color color = new Color(0, 0, 0);
                boolean all = false;
                if (led.equals("all")) {
                    all = true;
                } else {
                    ledint = Integer.parseInt(led);
                }

                String colorST = par.get("color");
                RemoteRobModuleImplementation.this.roboboManager.log(LogLvl.TRACE, TAG, "SET-LEDCOLOR Color: " + colorST + " led: " + led);

                switch (colorST) {
                    case "white":
                        color = new Color(3000, 3000, 3000);
                        break;
                    case "red":
                        color = new Color(3000, 0, 0);
                        break;
                    case "blue":
                        color = new Color(0, 0, 3000);
                        break;
                    case "cyan":
                        color = new Color(0, 3000, 3000);
                        break;
                    case "magenta":
                        color = new Color(3000, 0, 3000);
                        break;
                    case "yellow":
                        color = new Color(3000, 3000, 0);
                        break;
                    case "green":
                        color = new Color(0, 3000, 0);
                        break;
                    case "orange":
                        color = new Color(3000, 1500, 0);
                        break;
                    case "on":
                        color = new Color(3000, 3000, 3000);
                        break;
                    case "off":
                        color = new Color(0, 0, 0);
                        break;
                }
                if (all) {
                    for (int i = 1; i < 8; i++) {
                        try {
                            irob.setLEDColor(i, color);
                        } catch (CommunicationException e) {
                            Log.e(TAG, "Error change color leds", e);
                            roboboManager.notifyModuleError(e);
                        }
                    }
                } else {
                    try {
                        irob.setLEDColor(ledint, color);
                    } catch (CommunicationException e) {
                        Log.e(TAG, "Error change color leds", e);
                        roboboManager.notifyModuleError(e);
                    }
                }
            }
        });

        rcmodule.registerCommand("MOVE-BLOCKING", new ICommandExecutor() {
            @Override
            public void executeCommand(Command c, final IRemoteControlModule rcmodule) {
                HashMap<String, String> par = c.getParameters();
                int time = Math.round(Float.parseFloat(par.get("time")) * 1000);
                int lspeed = Integer.parseInt(par.get("lspeed"));
                int rspeed = Integer.parseInt(par.get("rspeed"));
                int commandid = Integer.parseInt(par.get("blockid"));
                Timer timer = new Timer();

                class UnlockClass extends TimerTask {

                    int id = 0;

                    public UnlockClass(int bid) {
                        this.id = bid;
                    }

                    @Override
                    public void run() {
                        Status s = new Status("UNLOCK-MOVE");
                        s.putContents("blockid", this.id + "");
                        rcmodule.postStatus(s);
                        RemoteRobModuleImplementation.this.roboboManager.log(LogLvl.TRACE, TAG, "UNLOCK-MOVE message");

                    }
                }

                UnlockClass ulclass = new UnlockClass(commandid);

                timer.schedule(ulclass, time - 100);

                RemoteRobModuleImplementation.this.roboboManager.log(LogLvl.TRACE, TAG, "MOVE-BLOCKING Left: " + lspeed + " Right: " + rspeed);

                try {
                    irob.moveMT(rspeed, lspeed, time);
                } catch (CommunicationException e) {
                    Log.e(TAG, "Error movement", e);
                    roboboManager.notifyModuleError(e);
                }

            }
        });

        rcmodule.registerCommand("RESET-WHEELS", new ICommandExecutor() {
            @Override
            public void executeCommand(Command c, IRemoteControlModule rcmodule) {
                try {
                    irob.resetWheelEncoders(RobMotorEnum.ALL_MOTOR);
                } catch (CommunicationException e) {
                    Log.e(TAG, "Error movement", e);
                    roboboManager.notifyModuleError(e);
                }
            }
        });
    }


    private String ledToIndex(String led) {
        switch (led) {
            case "Front-LL":
                return "1";
            case "Front-L":
                return "2";
            case "Front-C":
                return "3";
            case "Front-R":
                return "4";
            case "Front-RR":
                return "5";
            case "Back-R":
                return "6";
            case "Back-L":
                return "7";
            case "all":
                return "all";
            default:
                return "all";

        }
    }

    private String indexToLed(int index) {
        switch (index) {
            case 1:
                return "Front-LL";
            case 2:
                return "Front-L";
            case 3:
                return "Front-C";
            case 4:
                return "Front-R";
            case 5:
                return "Front-RR";
            case 6:
                return "Back-R";
            case 7:
                return "Back-L";
            default:
                return "all";

        }
    }

    @Override
    public void shutdown() throws InternalErrorException {

    }

    @Override
    public String getModuleInfo() {
        return "Remote Rob Module";
    }

    @Override
    public String getModuleVersion() {
        return "v0.3.0";
    }

    public float getBatteryLevel() {

        Intent batteryIntent = context.registerReceiver(null, new IntentFilter(Intent.ACTION_BATTERY_CHANGED));
        int level = batteryIntent.getIntExtra(BatteryManager.EXTRA_LEVEL, -1);
        int scale = batteryIntent.getIntExtra(BatteryManager.EXTRA_SCALE, -1);

        // Error checking that probably isn't needed but I added just in case.
        if (level == -1 || scale == -1) {
            return 50.0f;
        }

        return ((float) level / (float) scale) * 100.0f;
    }


    private class PanWaitThread extends Thread {
        private boolean terminate = false;

        private int oripos = 0;
        private int newpos = 0;
        private int blockid = 0;

        public PanWaitThread(int blockid, int newpos, int originalpos) {
            this.blockid = blockid;
            this.oripos = originalpos;
            this.newpos = newpos;
        }

        @Override
        public void run() {
            super.run();
            int lastTrackedPos = -1;
            boolean isblocked = false;
            int blockedCount = 0;

            if (newpos > oripos) {
                while (!terminate) {
                    //Log.d(TAG, "Blocked count: "+blockedCount);
                    if (lastTrackedPos != lastPanPosition) {
                        blockedCount = 0;
                    } else {
                        blockedCount = blockedCount + 1;
                        if (blockedCount >= 20) {
                            isblocked = true;
                            this.interrupt();
                        }
                    }
                    lastTrackedPos = lastPanPosition;

                    if (lastPanPosition > (newpos - 5)) {
//                        Status s = new Status("UNLOCKPAN");
//                        s.putContents("blockid",this.blockid+"");
//                        rcmodule.postStatus(s);

                        if (!this.isInterrupted()) {
                            this.interrupt();
                        }
                    }
                    try {
                        this.sleep(75);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }


                }
            } else {
                while (!terminate) {
                    if (lastTrackedPos != lastPanPosition) {
                        blockedCount = 0;
                    } else {
                        blockedCount = blockedCount + 1;
                        if (blockedCount >= 20) {
                            isblocked = true;
                            this.interrupt();
                        }
                    }
                    lastTrackedPos = lastPanPosition;
                    if (lastPanPosition < (newpos + 5)) {
//                        Status s = new Status("UNLOCKPAN");
//                        s.putContents("blockid",this.blockid+"");
//                        rcmodule.postStatus(s);
                        if (!this.isInterrupted()) {
                            this.interrupt();

                        }
                    }
                    try {
                        this.sleep(75);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }


                }
            }
        }


        @Override
        public void interrupt() {
            terminate = true;
            Status s = new Status("UNCLOK-PAN");
            s.putContents("blockid", this.blockid + "");
            rcmodule.postStatus(s);

            super.interrupt();

        }

        @Override
        public boolean isInterrupted() {
            return terminate;
        }
    }

    private class TiltWaitThread extends Thread {
        private boolean terminate = false;

        private int oripos = 0;
        private int newpos = 0;
        private int blockid = 0;

        public TiltWaitThread(int blockid, int newpos, int originalpos) {
            this.blockid = blockid;
            this.oripos = originalpos;
            this.newpos = newpos;
        }

        @Override
        public void run() {
            super.run();
            int lastTrackedPos = -1;
            boolean isblocked = false;
            int blockedCount = 0;

            if (newpos > oripos) {
                while (!terminate) {
                    if (lastTrackedPos != lastTiltPosition) {
                        blockedCount = 0;

                    } else {
                        blockedCount += 1;
                        if (blockedCount >= 30) {
                            isblocked = true;
                            this.interrupt();
                        }
                    }
                    lastTrackedPos = lastTiltPosition;
                    if (lastTiltPosition > (newpos - 5)) {
//                        Status s = new Status("UNLOCKTILT");
//                        s.putContents("blockid",this.blockid+"");
//                        rcmodule.postStatus(s);

                        if (!this.isInterrupted()) {
                            this.interrupt();
                        }
                    }
                    try {
                        this.sleep(75);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
            } else {
                while (!terminate) {
                    if (lastTrackedPos != lastTiltPosition) {
                        blockedCount = 0;
                    } else {
                        blockedCount += 1;
                        if (blockedCount >= 30) {
                            isblocked = true;
                            this.interrupt();
                        }
                    }
                    lastTrackedPos = lastTiltPosition;
                    if (lastTiltPosition < (newpos + 5)) {
//                        Status s = new Status("UNLOCKTILT");
//                        s.putContents("blockid",this.blockid+"");
//                        rcmodule.postStatus(s);

                        if (!this.isInterrupted()) {
                            this.interrupt();
                        }
                    }
                    try {
                        this.sleep(75);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }

                }
            }

        }

        @Override
        public void interrupt() {
            terminate = true;
            Status s = new Status("UNLOCK-TILT");
            s.putContents("blockid", this.blockid + "");
            rcmodule.postStatus(s);
            super.interrupt();

        }

        @Override
        public boolean isInterrupted() {
            return terminate;
        }
    }
}