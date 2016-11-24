package org.fhs.robotics.ftcteam10771.lepamplemousse.modes;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.fhs.robotics.ftcteam10771.lepamplemousse.core.components.Aliases;

/**
 * Autonomous class
 */
public class Autonomous {

    Telemetry telemetry;
    private long lastTime;      // The time at the last time check (using System.currentTimeMillis())


    /**
     * The constructor for the autonomous class
     *
     * @param telemetry   Reference to the Telemetry object of the OpMode
     */
    public Autonomous(Telemetry telemetry) {
        this.telemetry = telemetry;
        lastTime = System.currentTimeMillis();
    }

    /**
     * The loop of the autonomous class. Does not contain a loop, since it's expected to be within a loop.
     * It lets the drivers drive.
     */
    public void loop(){
        //To be determined
        //TODO: Come up with something last minute
    }

    public void cleanup(){
        //cleanup code
        Aliases.clearAll();
    }

}