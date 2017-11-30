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
package com.mytechia.robobo.rob;

import com.mytechia.commons.framework.exception.InternalErrorException;
import com.mytechia.robobo.framework.RoboboManager;

//import org.slf4j.LoggerFactory;
//
//import ch.qos.logback.classic.Level;
//import ch.qos.logback.classic.Logger;
//import ch.qos.logback.classic.LoggerContext;
//import ch.qos.logback.classic.encoder.PatternLayoutEncoder;
//import ch.qos.logback.core.ConsoleAppender;


/**
 * This class only prints logs of actions. It is not communicating with the Robobo-ROB
 *
 * @author Julio
 */

public class DummyRobInterfaceModule implements IRobInterfaceModule{


//    static {
//
//        LoggerContext loggerContext = (LoggerContext) LoggerFactory.getILoggerFactory();
//
//        loggerContext.reset();
//
//
//        Logger root = loggerContext.getLogger(Logger.ROOT_LOGGER_NAME);
//
//
//        PatternLayoutEncoder patternLayout = new PatternLayoutEncoder();
//        patternLayout.setContext(loggerContext);
//        patternLayout.setPattern("%d{dd/MM/yyyy-HH:mm:ss} %-5p: %-20c{1} - %m%n");
//        patternLayout.start();
//
//        ConsoleAppender consoleAppender= new ConsoleAppender();
//        consoleAppender.setName("dummy-rob-interface-console");
//        consoleAppender.setEncoder(patternLayout);
//        consoleAppender.start();
//
//        root.setLevel(Level.INFO);
//
//        root.addAppender(consoleAppender);
//    }


    private static final String MODULE_INFO = "Dummy rob Interface Module";
    private static final String MODULE_VERSION = "0.1.0";

    private DummyRob dummyRob= new DummyRob();


    @Override
    public IRob getRobInterface() {
        return this.dummyRob;
    }

    @Override
    public void startup(RoboboManager manager) throws InternalErrorException {

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
