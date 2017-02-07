package org.fhs.robotics.ftcteam10771.lepamplemousse.actions;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.fhs.robotics.ftcteam10771.lepamplemousse.config.Config;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.Components;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.Controllers;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.components.Aliases;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.vars.Static;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.core.Coordinate;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.core.Rotation;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.entities.Robot;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.vector.VectorR;
import org.firstinspires.ftc.robotcore.internal.opengl.AutoConfigGLSurfaceView;

/**
 * Created by joelv on 2/6/2017.
 */

public class PositionalOpMode extends LinearOpMode {

         // The time at the last time check (using System.currentTimeMillis())
    private Config rawSettings;
    private Config.ParsedData settings;
    private Config scriptConfig;
    private Config fieldMapConfig;
    private Components components;
    private Drive drive;
    private VectorR driveVector = new VectorR(new Coordinate(), new Rotation());
    private PositionalDrive positionalDrive;

    @Override
    public void runOpMode() throws InterruptedException {

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

        fieldMapConfig = new Config("/Position", "fieldmap.yml", telemetry, "field");
        if (fieldMapConfig.read()== Config.State.DEFAULT_EXISTS){
            fieldMapConfig.create(true);
            if (fieldMapConfig.read()== Config.State.DEFAULT_EXISTS)
                fieldMapConfig.read(true);
        }

        scriptConfig = new Config("/Position", "script.yml", telemetry, "script");
        if (scriptConfig.read()== Config.State.DEFAULT_EXISTS){
            scriptConfig.create(true);
            if (scriptConfig.read()== Config.State.DEFAULT_EXISTS)
                scriptConfig.read(true);
        }

        Config.ParsedData parsedField = fieldMapConfig.getParsedData();
        Config.ParsedData parsedScript = scriptConfig.getParsedData();

        this.settings = rawSettings.getParsedData();


        this.components = new Components(hardwareMap, telemetry, components);
        this.components.initialize();

        Config.ParsedData drivetrainMotors = settings.subData("drivetrain").subData("motor");

        Aliases.motorMap.get(drivetrainMotors.subData("front_right").getString("map_name"));

        drive = new Drive(driveVector, new Robot(),
                Aliases.motorMap.get(drivetrainMotors.subData("front_right").getString("map_name")),
                Aliases.motorMap.get(drivetrainMotors.subData("front_left").getString("map_name")),
                Aliases.motorMap.get(drivetrainMotors.subData("back_left").getString("map_name")),
                Aliases.motorMap.get(drivetrainMotors.subData("back_right").getString("map_name")),
                settings, telemetry);

        positionalDrive = new PositionalDrive(drive, settings, parsedField, parsedScript);

        waitForStart();
        drive.setRelative(true);
        //FIXME: I don't know how to deal with thread starts, runs, and stops
        positionalDrive.startScript();
        drive.driveThread.interrupt();
    }
}