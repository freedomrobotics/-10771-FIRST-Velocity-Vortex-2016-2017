package org.fhs.robotics.ftcteam10771.lepamplemousse.modes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.fhs.robotics.ftcteam10771.lepamplemousse.actions.Drive;
import org.fhs.robotics.ftcteam10771.lepamplemousse.config.Config;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.Components;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.Controllers;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.components.Aliases;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.vars.Static;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.core.Coordinate;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.core.Rotation;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.entities.Robot;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.vector.VectorR;

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
    private Drive drive;
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

        Config.ParsedData drivetrainMotors = settings.subData("drivetrain").subData("motor");

        drive = new Drive(driveVector, new Robot(),
                Aliases.motorMap.get(drivetrainMotors.subData("front_right").getString("map_name")),
                Aliases.motorMap.get(drivetrainMotors.subData("front_left").getString("map_name")),
                Aliases.motorMap.get(drivetrainMotors.subData("back_left").getString("map_name")),
                Aliases.motorMap.get(drivetrainMotors.subData("back_right").getString("map_name")),
                settings, telemetry);

        //intake = hardwareMap.dcMotor.get()

        this.lastTime = System.currentTimeMillis();
    }

    public void start() {
        drive.startVelocity();
        drive.setRelative(true);
    }

    /**
     * User defined loop method
     * <p>
     * This method will be called repeatedly in a loop while this op mode is running
     */
    @Override
    public void loop() {
        driveVector.setX(controls.getAnalog("drivetrain_x"));
        driveVector.setY(controls.getAnalog("drivetrain_y"));
        driveVector.setRawR(controls.getAnalog("drivetrain_rotate"));
        telemetry.addData("drive_x", driveVector.getX());
        telemetry.addData("drive_y", driveVector.getY());
        telemetry.addData("drive_rot", driveVector.getRawR());
        // the beauty of this is that the driveVector references a vectorR object
        // with the controllor values, and the drive class's input vectorR is a
        // reference to the same object, so updates out here apply to the vectorR
        // in the drive thread
    }

    @Override
    public void stop(){
        //cleanup code
        Aliases.clearAll();
        drive.stop();
    }
}
/*
 public void startScript(){
        List<String> commands = (List<String>) parsedField.getObject("script");
        for (String command : commands){
            /*
            while(!controls.getDigital("script")){}
            try {
                Thread.sleep(2000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
long startTime = System.currentTimeMillis();
while (System.currentTimeMillis()-startTime<10000){
        telemetry.addData("Location", command);
        telemetry.addData("X", parsedField.subData(command).getFloat("x"));
        telemetry.addData("Y", parsedField.subData(command).getFloat("y"));
        telemetry.update();
                }
            }
        }
 */