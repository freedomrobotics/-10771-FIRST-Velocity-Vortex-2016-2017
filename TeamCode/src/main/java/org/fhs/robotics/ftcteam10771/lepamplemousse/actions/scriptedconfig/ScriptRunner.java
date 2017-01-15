package org.fhs.robotics.ftcteam10771.lepamplemousse.actions.scriptedconfig;

/**
 * Created by Adam Li on 1/11/2017.
 */

public interface ScriptRunner {
    /**
     * refer to RobotAutoTake2 in the archived code from last year under com.qualcomm.ftcrobotcontroller.opmodes
     * @param commandParser
     * @throws InterruptedException
     */
    void commandPicker(ScriptLoader.CommandParser commandParser) throws InterruptedException;
}
