package org.fhs.robotics.ftcteam10771.lepamplemousse.modes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.fhs.robotics.ftcteam10771.lepamplemousse.actions.Drive;
import org.fhs.robotics.ftcteam10771.lepamplemousse.config.Config;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.Components;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.Controllers;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.vars.Static;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.fhs.robotics.ftcteam10771.lepamplemousse.core.components.Aliases;

/**
 * Example
 */
@TeleOp(name = "Framework Example", group = "Example")
@Disabled
public class ExampleUsingFramework extends OpMode {
    Controllers controls;
    Telemetry telemetry;
    private long lastTime;      // The time at the last time check (using System.currentTimeMillis())
    private Config rawSettings;
    private Config.ParsedData settings;
    private Components components;
    private Drive drive;
    private Config.ParsedData otherComponents;

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

        rawSettings = new Config(Static.configPath, Static.configVarFileName + Static.configFileSufffix, telemetry, "settings");
        if (rawSettings.read() == Config.State.DEFAULT_EXISTS) {
            rawSettings.create(true);
            if (rawSettings.read() == Config.State.DEFAULT_EXISTS)
                rawSettings.read(true);
        }

        settings = rawSettings.getParsedData();

        // access other components data structure from here
        this.otherComponents = components.getParsedData().subData("other");

        this.components = new Components(hardwareMap, telemetry, components);
        this.components.initialize();
        controls = new Controllers(gamepad1, gamepad2, keymapping);

        /*
        drive = new Drive(new VectorR(), new Robot(), Aliases.motorMap.get("left_drive"),
                Aliases.motorMap.get("left_drive"), Aliases.motorMap.get("left_drive"),
                Aliases.motorMap.get("left_drive"), rawSettings.getParsedData().subData("drivetrain"));
        */

        lastTime = System.currentTimeMillis();
    }

    @Override
    public void init_loop() {

    }

    public void start() {
        //this is where we send the begin drive code
        //drive.start() or something

        //and field tracking
    }

    /**
     * The loop of the controlled class. Does not contain a loop, since it's expected to be within a loop.
     * It lets the drivers drive.
     */
    public void loop() {
        // code here

        //access settings (config.yml)
        settings.subData("something").getInt("int_name");

        //get gamepad function (controls.yml)
        //functions are programmer defined
        controls.getAnalog("function_name");

        //get components (components.yml)
        //*still needs improvement -- will do for next year*
        Aliases.motorMap.get("left_drive");

    }

    @Override
    public void stop(){
        cleanup();

    }

    public void cleanup(){
        //cleanup code
        Aliases.clearAll();
        //drive.stop() or something
    }
}