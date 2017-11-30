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

package com.mytechia.robobo.rob.movement;

import com.mytechia.commons.framework.exception.InternalErrorException;
import com.mytechia.robobo.framework.RoboboManager;
import com.mytechia.robobo.rob.IRob;
import com.mytechia.robobo.rob.IRobInterfaceModule;


/**
 * The default implementation of the IRobMovementInterfaceModule.
 *
 * @author Gervasio Varela
 */
public class DefaultRobMovementModule implements IRobMovementModule {


    private static final String MODULE_INFO = "Robobo-ROB movement module.";
    private static final String MODULE_VERSION= "0.1.0";

    private static final int MAX_PANTILT_VELOCITY =  6; //safe velocity

    private IRobInterfaceModule robModule;
    private IRob rob;
    private RoboboManager m;


    @Override
    public void moveForwardsTime(int velocity, long time) throws InternalErrorException {

        m.log("ROB-INTERFACE", String.format("Move forwards: %d - %d", velocity, time));
        //this.rob.moveMT(MoveMTMode.FORWARD_FORWARD, velocity, velocity, time);

    }

    @Override
    public void moveForwardsAngle(int velocity, int angle) throws InternalErrorException {
        m.log("ROB-INTERFACE", String.format("Move forwards: %d - %d", velocity, angle));
        //this.rob.moveMT(MoveMTMode.FORWARD_FORWARD, velocity, angle, velocity, angle);
    }

    @Override
    public void moveBackwardsTime(int velocity, long time) throws InternalErrorException {
        m.log("ROB-INTERFACE", String.format("Move backwards: %d - %d", velocity, time));
        //this.rob.moveMT(MoveMTMode.REVERSE_REVERSE, velocity, velocity, time);
    }

    @Override
    public void moveBackwardsAngle(int velocity, int angle) throws InternalErrorException {
        m.log("ROB-INTERFACE", String.format("Move backwards: %d - %d", velocity, angle));
        //this.rob.moveMT(MoveMTMode.REVERSE_REVERSE, velocity, angle, velocity, angle);
    }

    @Override
    public void stop() throws InternalErrorException {
        m.log("ROB-INTERFACE", "Stop.");
        //this.rob.moveMT(MoveMTMode.STOP_STOP, 0, 0, 0);
    }

    @Override
    public void turnLeftTime(int velocity, long time) throws InternalErrorException {
        m.log("ROB-INTERFACE", String.format("Turn left: %d - %d", velocity, time));
        //this.rob.moveMT(MoveMTMode.FORWARD_FORWARD, 0, velocity, time);
    }

    @Override
    public void turnLeftAngle(int velocity, int angle) throws InternalErrorException {
        m.log("ROB-INTERFACE", String.format("Turn left: %d - %d", velocity, angle));
        //this.rob.moveMT(MoveMTMode.FORWARD_FORWARD,  0, 0, velocity, angle);
    }

    @Override
    public void turnLeftBackwardsTime(int velocity, long time) throws InternalErrorException {
        m.log("ROB-INTERFACE", String.format("Turn left backwards: %d - %d", velocity, time));
        //this.rob.moveMT(MoveMTMode.REVERSE_REVERSE, 0, velocity, time);
    }

    @Override
    public void turnRightTime(int velocity, long time) throws InternalErrorException {
        m.log("ROB-INTERFACE", String.format("Turn right: %d - %d", velocity, time));
        //this.rob.moveMT(MoveMTMode.FORWARD_FORWARD, velocity, 0, time);
    }

    @Override
    public void turnRightAngle(int velocity, int angle) throws InternalErrorException {
        m.log("ROB-INTERFACE", String.format("Turn right: %d - %d", velocity, angle));
        //this.rob.moveMT(MoveMTMode.FORWARD_FORWARD, velocity, angle,  0, 0);
    }

    @Override
    public void turnRightBackwardsTime(int velocity, long time) throws InternalErrorException {
        m.log("ROB-INTERFACE", String.format("Turn right backwards: %d - %d", velocity, time));
        //this.rob.moveMT(MoveMTMode.REVERSE_REVERSE, 0, velocity, time);
    }

    @Override
    public void movePan(int velocity, int angle) throws InternalErrorException {
        m.log("ROB-INTERFACE", String.format("Move pan: %d ", angle));
        this.rob.movePan(velocity, angle);
    }

    @Override
    public void moveTilt(int velocity, int angle) throws InternalErrorException {
        m.log("ROB-INTERFACE", String.format("Move tilt: %d ", angle));
        this.rob.moveTilt(velocity, angle);
    }

    @Override
    public void movePanTilt(int velocitypan, int anglepan, int velocitytilt, int angletilt) throws InternalErrorException {
        //this.rob.movePanTilt(velocitypan, anglepan, velocitytilt, angletilt);
    }


    @Override
    public void startup(RoboboManager manager) throws InternalErrorException {

        this.robModule = manager.getModuleInstance(IRobInterfaceModule.class);
        m = manager;
        if (this.robModule != null) {
            this.rob = this.robModule.getRobInterface();
        }

    }

    @Override
    public void shutdown() throws InternalErrorException {

    }

    @Override
    public String getModuleInfo() {
        return MODULE_INFO;
    }

    @Override
    public String getModuleVersion() {
        return MODULE_VERSION;
    }
}
