package org.fhs.robotics.ftcteam10771.lepamplemousse.modes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.fhs.robotics.ftcteam10771.lepamplemousse.actions.ApproachBeacon;
import org.fhs.robotics.ftcteam10771.lepamplemousse.actions.Drive;
import org.fhs.robotics.ftcteam10771.lepamplemousse.config.Config;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.Alliance;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.Controllers;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.vars.Static;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.entities.Robot;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.vector.VectorR;

import java.util.List;

/**
 * Created by joelv on 2/22/2017.
 */
@Autonomous (name = "Beacon Test")
public class ClaimBeacon extends LinearOpMode{

    ApproachBeacon approachBeacon;
    private Alliance alliance;
    private String teamColor;
    private Config rawSettings;
    private Robot robot;
    private Config.ParsedData settings;
    private Controllers controls;
    private Drive drive;
    private VectorR driveVector;
    private int test = 0;

    private Thread telemetryThread = new Thread(new Runnable() {
        @Override
        public void run() {
            telemetry.addData("Alliance", alliance.getAlliance());
            telemetry.addData("BeaconColor", approachBeacon.beaconAlliance());
            telemetry.addData("BeaconSide", approachBeacon.beaconStat());
        }
    });

    @Override
    public void runOpMode() throws InterruptedException {

        loadConfigurations();
        teamColor = settings.getString("alliance");

        alliance = settings.getString("alliance").equals("red") ? Alliance.RED_ALLIANCE : Alliance.UNKNOWN;
        if (alliance!=Alliance.RED_ALLIANCE)
            alliance = settings.getString("alliance").equals("blue")
                    ? Alliance.BLUE_ALLIANCE : Alliance.UNKNOWN;

        //PREP THE COMMAND LIST!
        if (!settings.subData("autonomous").valueExists("command_list"))
            // TODO: 2/16/2017 add more throws into framework to reduce silent errors when in development
            throw new RuntimeException("Empty command list");

        //setup command list
        List<String> commandList = (List<String>) settings.subData("autonomous").getObject("command_list");

        // TODO: 2/16/2017 initialize robot based on configuration and alliance color if implemented
        robot = new Robot();

        //setup drive thread
        Config.ParsedData drivetrainMotors = settings.subData("drivetrain").subData("motor");

        drive = new Drive(driveVector, robot,
                hardwareMap.dcMotor.get(drivetrainMotors.subData("front_right").getString("map_name")),
                hardwareMap.dcMotor.get(drivetrainMotors.subData("front_left").getString("map_name")),
                hardwareMap.dcMotor.get(drivetrainMotors.subData("back_left").getString("map_name")),
                hardwareMap.dcMotor.get(drivetrainMotors.subData("back_right").getString("map_name")),
                settings, telemetry);

        approachBeacon = new ApproachBeacon(settings, drive, this);

        test = settings.subData("beacon").getInt("test");

        telemetryThread.start();
        drive.startVelocity();

        waitForStart();
        try{
            switch (test){
                case 1:
                    approachBeacon.chooseSide(true);
                    break;
                case 2:
                    approachBeacon.chooseSide(false);
                    break;
                case 3:
                    approachBeacon.claimBeacon();
                    break;
                default:
                    break;
            }
        }catch (Exception e){
            drive.stop();
            telemetryThread.interrupt();
        }
        drive.stop();
        if (telemetryThread.isAlive()) telemetryThread.interrupt();
    }

    /**
     * loads all of the configuration files and stuff
     */
    // TODO: 2/16/2017 turn this into an easier to access and use generalized function
    private void loadConfigurations(){
        Config keymapping = new Config(Static.configPath, Static.configControlFileName + Static.configFileSufffix, telemetry, "keymapping");
        // should catch other errors, but oh well
        // add data logging
        if (keymapping.read() == Config.State.DEFAULT_EXISTS) {
            keymapping.create(true);
            //read without creating file if it still fails.
            if (keymapping.read() == Config.State.DEFAULT_EXISTS)
                keymapping.read(true);
        }

        this.rawSettings = new Config(Static.configPath, Static.configVarFileName + Static.configFileSufffix, telemetry, "settings");
        if (rawSettings.read() == Config.State.DEFAULT_EXISTS) {
            rawSettings.create(true);
            if (rawSettings.read() == Config.State.DEFAULT_EXISTS)
                rawSettings.read(true);
        }

        // Store the parsed data into the settings variable
        this.settings = rawSettings.getParsedData();

        // Setup the keymapping
        this.controls = new Controllers(gamepad1, gamepad2, keymapping);
        this.controls.initialize();
    }
}

