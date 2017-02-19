package org.fhs.robotics.ftcteam10771.lepamplemousse.modes.final_op_modes;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.fhs.robotics.ftcteam10771.lepamplemousse.actions.Drive;
import org.fhs.robotics.ftcteam10771.lepamplemousse.actions.scriptedconfig.ScriptLoader;
import org.fhs.robotics.ftcteam10771.lepamplemousse.actions.scriptedconfig.ScriptRunner;
import org.fhs.robotics.ftcteam10771.lepamplemousse.config.Config;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.Controllers;
import org.fhs.robotics.ftcteam10771.lepamplemousse.mechanisms.Catapult;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.IMU;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.vars.Static;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.core.Rotation;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.entities.Robot;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.vector.VectorR;

import java.util.List;

/**
 * Created by Adam Li on 2/16/2017.
 */
@Autonomous(name = "Scripted Autonomous")
public class ScriptedAutonomous extends LinearOpMode implements ScriptRunner {
    Controllers controls;
    private long lastTime;      // The time at the last time check (using System.currentTimeMillis())
    private Config rawSettings;
    private Config.ParsedData settings;
    private Drive drive;
    private VectorR driveVector = new VectorR();
    private Robot robot;
    private IMU imuHandler;
    private IMU.Gyrometer gyrometer;
    private Servo ballDropper;
    private Catapult catapult;
    private DcMotor intakeMotor;

    /**
     * Override this method and place your code here.
     * <p>
     * Please do not swallow the InterruptedException, as it is used in cases
     * where the op mode needs to be terminated early.
     *
     * @throws InterruptedException
     */
    @Override
    public void runOpMode() throws InterruptedException {
        loadConfigurations();

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

        //setup imu
        imuHandler = new IMU(hardwareMap.get(BNO055IMU.class, settings.subData("imu").getString("map_name")));
        imuHandler.imuInit(); //todo remember to init imu
        gyrometer = imuHandler.getGyrometer();
        gyrometer.enableStream(true);
        imuHandler.streamIMUData();

        //reset all servo positions + some if needed
        resetPositions();

        //setup launcher
        // TODO: 2/16/2017 clean up catapult module
        catapult = new Catapult(hardwareMap.dcMotor.get(settings.subData("catapult").getString("map_name")),
                                hardwareMap.opticalDistanceSensor.get(settings.subData("catapult")
                                        .subData("light_sensor").getString("map_name")), settings.subData("catapult"));
        catapult.start();

        int counter = 1; //debug
        //WAIT FOR THE STARTI!@HTIOgRFOIWUQ#GQIOH
        telemetry.addData("ready", true);
        telemetry.addData("orient", gyrometer.getOrientation(IMU.Axis.Z));
        telemetry.update();
        waitForStart();
        // run through all of the commands
        for(String command : commandList){
            //check if opmode is active
            if (!opModeIsActive()) break;
            this.lastTime = System.currentTimeMillis();

            ScriptLoader.CommandParser commandParser = new ScriptLoader.CommandParser(command);

            if (commandParser.command().equalsIgnoreCase("stop")){
                telemetry.addData("done", true);
                break;
            }

            //Debug section
            telemetry.addData("robot", robot.toString()); // TODO: 2/16/2017 add tostring implementation back in
            telemetry.addData(counter + "", command);
            counter++;
            telemetry.update();

            //check commands
            commandPicker(commandParser);
        }
        telemetry.update();
        drive.stop();
        catapult.stop();
    }

    /**
     * refer to RobotAutoTake2 in the archived code from last year under com.qualcomm.ftcrobotcontroller.opmodes
     *
     * @param commandParser
     * @throws InterruptedException
     */
    @Override
    public void commandPicker(ScriptLoader.CommandParser commandParser) throws InterruptedException {

        if (commandParser.command().equalsIgnoreCase("move_time")){
            if (commandParser.getArgsSize() == 2){
                drive.startVelocity();
                drive.setRelative(true);
                driveVector.setPolar(commandParser.getArgFloat(0), (float)(3.0/2.0*Math.PI));
                sleep(commandParser.getArgInt(1));
                driveVector.setPolar(0, 0);
                //define function for a 2 argument command
                //drive(commandParser.getArgFloat(0), commandParser.getArgFloat(1));

            } if(commandParser.getArgsSize() >= 3){
                drive.startVelocity();
                drive.setRelative(true);
                driveVector.setPolar(commandParser.getArgFloat(0), (float)Math.toRadians(commandParser.getArgFloat(1)));
                sleep(commandParser.getArgInt(2));
                driveVector.setPolar(0, 0);
                //move_position 3.5,3,500
            }
            return;
        }


        if (commandParser.command().equalsIgnoreCase("approach_beacon")){
            drive.setRelative(true);
            drive.startVelocity();
            //drive.driveThread.start();
            //rotate();
            //center();
            /*
            while (!targeted() && System.currentTimeMillis() - lastTime < commandParser.getArgInt(0) && opModeIsActive()){}
            if (!targeted())*/
            // return;
            //centerRotate();
            //approach();
            //
            driveVector.setPolar(0, 0);
            return;
        }

        if (commandParser.command().equalsIgnoreCase("rotate")){
            if (commandParser.getArgsSize() == 2){
                rotate(commandParser.getArgFloat(0), commandParser.getArgBool(1));
            } else {
                rotate(commandParser.getArgFloat(0), false);
            }
            return;
        }

        if (commandParser.command().equalsIgnoreCase("catapult")) {
            catapult.launch();
            sleep(settings.subData("catapult").getInt("grace"));
            while (!catapult.catapultReady() && opModeIsActive());
            return;
        }

        if (commandParser.command().equalsIgnoreCase("ball_drop")){
            dropBalls(commandParser.getArgString(0).equalsIgnoreCase("up"));
            return;
        }

        if (commandParser.command().equalsIgnoreCase("intake")){
            intakeMotor.setPower(commandParser.getArgFloat(0));
        }

        if (commandParser.command().equalsIgnoreCase("pause")){
            sleep(commandParser.getArgInt(0));
            return;
        }

        if (commandParser.command().equalsIgnoreCase("coordinate")){
            coordinate(commandParser.getArgFloat(0), commandParser.getArgFloat(1));
            return;
        }

        if (!settings.subData("autonomous").valueExists(commandParser.command())) {
            telemetry.addData("ERROR", "command: \"" + commandParser.command() + "\" does not exist!");
            return;
        }

        List<String> commandList = (List<String>) settings.subData("autonomous").getObject(commandParser.command());
        for(String command : commandList) {
            if (!opModeIsActive()) break;

            ScriptLoader.CommandParser internalScript = new ScriptLoader.CommandParser(command);

            commandPicker(internalScript);

            if (internalScript.command().equalsIgnoreCase("stop")){
                return;
            }
        }
    }

    /**
     * Rotates the robot a certain amount of angle
     * or to a certain orientation
     * @param degrees  provided angle in degrees
     * @param relative true = rotate a certain amount of degrees
     *                 false = rotate to a certain orientation from 0 to 360 degrees
     */
    /*private void rotate(double degrees, boolean relative){
        drive.startVelocity();
        drive.setRelative(true);
        driveVector.setRad(0);
        driveVector.setPolar(0, 0);
        final double fullRotation = Math.toRadians(360.0);
        double orientation = fullRotation + gyrometer.getOrientation(IMU.Axis.Z, false, false);
        double targetOrientation;
        if (relative) {
            targetOrientation = orientation + Math.toRadians(degrees);
            while (Math.abs(targetOrientation) > 2 * Math.PI || Math.abs(targetOrientation) < 0) {
                if (Math.abs(targetOrientation) > 2 * Math.PI)
                    targetOrientation -= 2 * Math.PI;
                else if (Math.abs(targetOrientation) < 0)
                    targetOrientation += 2 * Math.PI;
            }
        }else{
            targetOrientation = Math.toRadians(degrees);
        }
        float rotate_margin = (float) Math.toRadians(settings.subData("scriptauto").subData("rotate").getFloat("angle_margin"));
        float rotate_speed = settings.subData("scriptauto").subData("rotate").getFloat("rotate_speed");
        float min_speed = settings.subData("scriptauto").subData("rotate").getFloat("min_speed");
        boolean targetNearZero = (targetOrientation < rotate_margin || targetOrientation > fullRotation - rotate_margin);
        if (targetOrientation > fullRotation - rotate_margin) targetOrientation -= fullRotation;
        while(orientation < targetOrientation - rotate_margin
                || orientation > targetOrientation + rotate_margin && opModeIsActive()){
            imuHandler.streamIMUData();
            if (targetNearZero){
                orientation = gyrometer.convertAngletoSemiPossibleRange(
                        IMU.Axis.Z, gyrometer.getOrientation(IMU.Axis.Z));
            }
            else orientation = Math.toRadians(360.0) + gyrometer.getOrientation(IMU.Axis.Z);
            //telemetry.addData("Orientation", Math.toDegrees(orientation));
            float factor = (float) (Math.abs(orientation - targetOrientation) / 6.28);
            driveVector.setRad((float)Math.copySign(Range.scale(factor, 0, 1, min_speed, rotate_speed), targetOrientation - orientation));
            //telemetry.update();
        }
        //telemetry.addData("Rotation", "Done");
        //telemetry.update();
        driveVector.setRad(0);
        driveVector.setPolar(0, 0);
    }*/

    private void rotate(float degrees, boolean relative){
        // FIXME: 2/19/2017 put in thread maybe?
        imuHandler.streamIMUData();
        robot.getRotation().setHeading(gyrometer.getOrientation(IMU.Axis.Z));

        float target;
        if (relative){
            target = (float) Math.toRadians(robot.getRotation().getDegrees() + degrees);
        } else {
            target = (float) Math.toRadians(degrees + 90);
        }
        drive.startPosition();
        driveVector.setAllRad(robot.getPosition().getX(), robot.getPosition().getY(), target);
        while(!drive.isAtRotation() && opModeIsActive()){
            imuHandler.streamIMUData();
            robot.getRotation().setHeading(gyrometer.getOrientation(IMU.Axis.Z));
        }
        drive.startVelocity();
        driveVector.setAllRad(0, 0, 0);
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

    /**
     * resets positions of all objects (specifically servos)
     */
    private void resetPositions(){
        //bumpers
        Config.ParsedData bumpers = settings.subData("bumper");

        Servo bumperLeft = hardwareMap.servo.get(bumpers.subData("left_servo").getString("map_name"));
        Servo bumperRight = hardwareMap.servo.get(bumpers.subData("right_servo").getString("map_name"));
        if (bumpers.subData("left_servo").getBool("reversed"))
            bumperLeft.setDirection(Servo.Direction.REVERSE);
        if (bumpers.subData("right_servo").getBool("reversed"))
            bumperRight.setDirection(Servo.Direction.REVERSE);
        float bumperRange = bumpers.getFloat("full_rotate");
        bumperLeft.setPosition(bumpers.subData("left_servo").getFloat("offset") / bumperRange);
        bumperRight.setPosition(bumpers.subData("right_servo").getFloat("offset") / bumperRange);

        //ball dropper
        ballDropper = hardwareMap.servo.get("drop");
        if (settings.subData("drop").getBool("reversed")){
            ballDropper.setDirection(Servo.Direction.REVERSE);
        } else{
            ballDropper.setDirection(Servo.Direction.FORWARD);
        }
        dropBalls(false);

        //intake motor
        intakeMotor = hardwareMap.dcMotor.get("motorIntake");
    }

    /**
     * handles the gate for the balls
     */
    private void dropBalls(boolean dropBalls){
        Config.ParsedData drop = settings.subData("drop");
        float fullRange = drop.getFloat("full_rotate");
        float offset = drop.getFloat("offset") / fullRange;
        float up = drop.getFloat("up_angle") / fullRange;
        float down = drop.getFloat("down_angle") / fullRange;
        if (dropBalls){
            ballDropper.setPosition(up + offset);
            telemetry.addData("dropper", "up");
        }else {
            ballDropper.setPosition(down + offset);
            telemetry.addData("dropper", "down");
        }
    }

    /**
     * Direct the robot to FTC field coordinate (x,y)
     * @param x coordinate
     * @param y coordinate
     */
    /*private void coordinate(float x, float y){
        drive.startPosition();
        driveVector.setX(x);
        driveVector.setY(y);
        boolean proceed = false;
        //private float margin = settings.subData("scripted_auto").getFloat("positional_margin");
        while(!proceed && opModeIsActive()){
            proceed = distance() < 3.0f;//todo put in settings
        }
        driveVector.setX(drive.getCurrentX());
        driveVector.setY(drive.getCurrentY());
    }*/

    private void coordinate(float x, float y){
        imuHandler.streamIMUData();
        robot.getRotation().setHeading(gyrometer.getOrientation(IMU.Axis.Z));

        drive.startPosition();
        driveVector.setAllRad(x, y, robot.getRotation().getRadians());
        while(!drive.isAtPosition() && opModeIsActive()){
            imuHandler.streamIMUData();
            robot.getRotation().setHeading(gyrometer.getOrientation(IMU.Axis.Z));
        }
        drive.startVelocity();
        driveVector.setAllRad(0, 0, 0);
    }

    /**
     * The method that calculates the distance between the robot's position
     * and its target position
     * @return the position
     */
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
