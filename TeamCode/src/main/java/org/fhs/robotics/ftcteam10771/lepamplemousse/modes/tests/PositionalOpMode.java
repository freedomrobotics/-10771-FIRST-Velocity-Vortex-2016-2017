package org.fhs.robotics.ftcteam10771.lepamplemousse.modes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
import org.firstinspires.ftc.robotcore.internal.opengl.AutoConfigGLSurfaceView;

import java.util.List;

/**
 * Op mode to test out the drive class and the fieldmap file
 * Created by joelv on 2/6/2017.
 */
@Autonomous(name="PositionalTest", group = "Test")
public class PositionalOpMode extends LinearOpMode {

         // The time at the last time check (using System.currentTimeMillis())
    private Config rawSettings;
    private Config.ParsedData settings;
    private Config.ParsedData fieldmap;
    private Config scriptConfig;
    private Config fieldMapConfig;
    private Components components;
    private Drive drive;
    private VectorR driveVector = new VectorR(new Coordinate(), new Rotation());

    Runnable runnable = new Runnable() {
        @Override
        public void run() {
            while (!Thread.currentThread().isInterrupted()) {
                telemetry.addData("Radius", driveVector.getRadius());
                telemetry.addData("Theta", driveVector.getTheta());
                telemetry.addData("FR", drive.getMotorPower(1));
                telemetry.addData("FL", drive.getMotorPower(2));
                telemetry.addData("BL", drive.getMotorPower(3));
                telemetry.addData("BR", drive.getMotorPower(4));
                telemetry.update();
            }
        }
    };

    Thread telemetryThread = new Thread(runnable);

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

        fieldmap = fieldMapConfig.getParsedData();

        this.settings = rawSettings.getParsedData();


        this.components = new Components(hardwareMap, telemetry, components);
        this.components.initialize();

        Config.ParsedData drivetrainMotors = settings.subData("drivetrain").subData("motor");

        drive = new Drive(driveVector, new Robot(),
                Aliases.motorMap.get(drivetrainMotors.subData("front_right").getString("map_name")),
                Aliases.motorMap.get(drivetrainMotors.subData("front_left").getString("map_name")),
                Aliases.motorMap.get(drivetrainMotors.subData("back_left").getString("map_name")),
                Aliases.motorMap.get(drivetrainMotors.subData("back_right").getString("map_name")),
                settings,telemetry);
        telemetryThread.start();
        waitForStart();
        boolean proceed = false;
        drive.setRelative(true);
        drive.startPosition();
        if (opModeIsActive()) driveTo("custom1");
        while(!proceed && opModeIsActive()){
            proceed = distance() < 3.0;
        } proceed = false;
        if (opModeIsActive()) driveTo("custom2");
        while(!proceed && opModeIsActive()){
            proceed = distance() < 3.0;
        } proceed = false;
        if (opModeIsActive()) driveTo("custom3");
        while(!proceed && opModeIsActive()){
            proceed = distance() < 3.0;
        }
        telemetryThread.interrupt();
        drive.stop();
    }

    public void driveTo(String location){
        setCoordinate(location);
    }
    public void setCoordinate(String location){
        driveVector.setX(fieldmap.subData("coordinates").subData("red").subData(location).getFloat("x"));
        driveVector.setY(fieldmap.subData("coordinates").subData("red").subData(location).getFloat("y"));
    }

    public double distance(){
        double currentX = drive.getCurrentX();
        double currentY = drive.getCurrentY();
        double targetX = driveVector.getX();
        double targetY = driveVector.getY();
        double xDistance = Math.abs(currentX-targetX);
        double yDistance = Math.abs(currentY-targetY);
        return Math.sqrt((xDistance*xDistance)+(yDistance*yDistance));
    }
}
