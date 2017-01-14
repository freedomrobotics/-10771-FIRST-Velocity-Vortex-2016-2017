package org.fhs.robotics.ftcteam10771.lepamplemousse.modes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.fhs.robotics.ftcteam10771.lepamplemousse.actions.VectorDrive;
import org.fhs.robotics.ftcteam10771.lepamplemousse.config.Config;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.Components;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.Controllers;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.components.Aliases;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.vars.Static;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.core.Coordinate;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.core.Rotation;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.entities.Robot;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.vector.VectorR;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Adam Li on 1/12/2017.
 */

@TeleOp(name="Basic Drive", group="Not So Basic Stuff")
public class BasicNotBasicDrive extends OpMode {
    Controllers controls;
    private long lastTime;      // The time at the last time check (using System.currentTimeMillis())
    private Config rawSettings;
    private Config.ParsedData settings;
    private Components components;
    private VectorDrive drive;
    private VectorR driveVector = new VectorR(new Coordinate(), new Rotation());

    /**
     * User defined init method
     * <p>
     * This method will be called once when the INIT button is pressed.
     */
    @Override
    public void init() {
        Config keymapping = new Config(Static.configPath, Static.configControlFileName + Static.configFileSufffix, telemetry, "keymapping");
        // should catch other errors, but oh well
        // add data logging
        if (keymapping.read() == Config.State.DEFAULT_EXISTS) {
            keymapping.create(true);
            //read without creating file if it still fails.
            if (keymapping.read() == Config.State.DEFAULT_EXISTS)
                keymapping.read(true);
        }

        Config components = new Config(Static.configPath, Static.configCompFileName + Static.configFileSufffix, telemetry, "components");
        if (components.read() == Config.State.DEFAULT_EXISTS) {
            components.create(true);
            if (components.read() == Config.State.DEFAULT_EXISTS)
                components.read(true);
        }

        this.rawSettings = new Config(Static.configPath, Static.configVarFileName + Static.configFileSufffix, telemetry, "settings");
        if (rawSettings.read() == Config.State.DEFAULT_EXISTS) {
            rawSettings.create(true);
            if (rawSettings.read() == Config.State.DEFAULT_EXISTS)
                rawSettings.read(true);
        }

        this.settings = rawSettings.getParsedData();


        this.components = new Components(hardwareMap, telemetry, components);
        this.components.initialize();
        this.controls = new Controllers(gamepad1, gamepad2, keymapping);
        this.controls.initialize();

        drive = new VectorDrive(driveVector, new Robot(), Aliases.motorMap.get("left_drive"),
                Aliases.motorMap.get("left_drive"), Aliases.motorMap.get("left_drive"),
                Aliases.motorMap.get("left_drive"), rawSettings.getParsedData().subData("drivetrain"));

        this.lastTime = System.currentTimeMillis();
    }

    public void start() {
        drive.startVelocity();
        //should be true for driving relative to itself, so please fix that
        drive.setRelative(true);
    }

    /**
     * User defined loop method
     * <p>
     * This method will be called repeatedly in a loop while this op mode is running
     */
    @Override
    public void loop() {
        driveVector.setX(controls.getAnalog("drive_x"));
        driveVector.setY(controls.getAnalog("drive_y"));
        driveVector.setRawR(controls.getAnalog("drive_rotation"));
    }

    @Override
    public void stop(){
        //cleanup code
        Aliases.clearAll();
        drive.stop();
    }
}
