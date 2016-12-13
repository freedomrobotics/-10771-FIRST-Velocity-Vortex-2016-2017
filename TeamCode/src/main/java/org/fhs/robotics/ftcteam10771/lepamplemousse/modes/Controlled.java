package org.fhs.robotics.ftcteam10771.lepamplemousse.modes;

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
 * Driver controlled class
 */
@TeleOp(name = "pool")
public class Controlled extends OpMode {
    Controllers controls;
    Components components;
    Config settings;
    Drive drive;
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

    @Override
    public void init() {
        Config keymapping = new Config(Static.configPath, Static.configControlFileName + Static.configFileSufffix, telemetry, "keymapping");
        Config components = new Config(Static.configPath, Static.configCompFileName + Static.configFileSufffix, telemetry, "components");
        keymapping.read();
        components.read();
        settings = new Config(Static.configPath, Static.configVarFileName + Static.configFileSufffix, telemetry, "settings");
        settings.read();
        this.components = new Components(hardwareMap, telemetry, components);
        this.components.initialize();
        controls = new Controllers(gamepad1, gamepad2, keymapping);

        drive = new Drive(new VectorR(), new Robot(), Aliases.motorMap.get("left_drive"),
                Aliases.motorMap.get("left_drive"), Aliases.motorMap.get("left_drive"),
                Aliases.motorMap.get("left_drive"), settings.getParsedData().subData("drivetrain"));



        if (gamepad1.a){
            //turn on LED
        }

        if (controls.getDigital("led_button")){
            //turn on led
        }

        Aliases.motorMap.get("left_drive")

        lastTime = System.currentTimeMillis();

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