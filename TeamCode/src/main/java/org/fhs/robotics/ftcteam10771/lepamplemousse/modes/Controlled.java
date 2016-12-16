package org.fhs.robotics.ftcteam10771.lepamplemousse.modes;

import org.fhs.robotics.ftcteam10771.lepamplemousse.core.Controllers;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.fhs.robotics.ftcteam10771.lepamplemousse.core.components.Aliases;

/**
 * Driver controlled class
 */
public class Controlled {
    Controllers controls;
    Telemetry telemetry;
    private long lastTime;      // The time at the last time check (using System.currentTimeMillis())


    /**
     * The constructor for the driver controlled class
     *
     * @param controls    Reference to the object of an initialized Controllers class
     * @param telemetry   Reference to the Telemetry object of the OpMode
     */
    public Controlled(Controllers controls, Telemetry telemetry) {
        this.controls = controls;
        this.telemetry = telemetry;
        lastTime = System.currentTimeMillis();

        // code here
    }

    /**
     * The loop of the controlled class. Does not contain a loop, since it's expected to be within a loop.
     * It lets the drivers drive.
     */
    public void loop() {
        // code here

    }

    public void cleanup(){
        //cleanup code
        Aliases.clearAll();
    }
}