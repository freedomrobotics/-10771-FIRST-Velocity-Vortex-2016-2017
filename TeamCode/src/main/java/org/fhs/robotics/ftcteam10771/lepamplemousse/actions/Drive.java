package org.fhs.robotics.ftcteam10771.lepamplemousse.actions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.fhs.robotics.ftcteam10771.lepamplemousse.config.Config;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.entities.Robot;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.vector.VectorR;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.Math;
import java.util.List;
/**
 * Created by Adam Li on 10/27/2016.
 * Class to manage how the robot drives (smart control schemes, drivetrain types, etc.)
 */
public class Drive {
    private final Telemetry telemetry;
    VectorR vectorR;
    Robot robot;
    DcMotor frMotor;
    DcMotor flMotor;
    DcMotor brMotor;
    DcMotor blMotor;
    Config.ParsedData driveSettings;
    boolean vectorDriveActive;
    boolean blueTeam;
    boolean relativeDrive;
    boolean joystickControl;
    float motorScale;
    boolean driveThreadActive = false;

    int pastPosition;
    long pastPositionTime;

    //Positional drive stuff
    private float initialX = 0.0f;
    private float initialY = 0.0f;
    private Config.ParsedData settings;
    private Config.ParsedData fieldmap;
    List<String> commands;

    SensorHandler sensorHandler;

    Runnable driveRunnable = new Runnable() {
        @Override
        public void run() {

            while (!Thread.currentThread().interrupted() && driveThreadActive) {

                float joystickTheta;
                float absoluteTheta;
                float robotTheta;
                float robotVelocity;
                float rotationalPower;
                float robotRotation;

                if (vectorDriveActive) {
                    //sets values from the vectorR needed for movement
                    joystickTheta = (float) Math.atan2(vectorR.getY(), vectorR.getX());
                    robotVelocity = vectorR.getRadius();
                    rotationalPower = vectorR.getRad();
                    robotRotation = robot.getVectorR().getRad();

                    if (relativeDrive) { //if the robot drives relative to the field
                        robotTheta = joystickTheta; //the direction of the joystick is the direction of motion
                    } else {
                        if (blueTeam) { //if our team is on blue
                            absoluteTheta = (float) (joystickTheta + Math.PI * 2);
                        } else {
                            absoluteTheta = joystickTheta; //the field coordinate does not need to be adjusted
                        }

                        robotTheta = absoluteTheta + robotRotation;
                    }
                } else {
                    updatePosition();
                    float vectorX = vectorR.getX() - robot.getVectorR().getX();
                    float vectorY = vectorR.getY() - robot.getVectorR().getY();
                    robotTheta = (float) Math.atan2(vectorY, vectorX);

                    if(Math.abs(robotTheta) < Math.PI*2*(driveSettings.subData("positional").getFloat("rotational_tolerance")/360)){
                        //at position
                        Thread.currentThread().interrupt();
                    }

                    robotVelocity = driveSettings.subData("positional").getFloat("speed");
                    float rotationalMagnitude = driveSettings.subData("positional").getFloat("rotation");
                    if (vectorR.getRad() > robot.getVectorR().getRad()){
                        if(Math.abs(vectorR.getRad()-robot.getVectorR().getRad())<(Math.PI/4)){
                            rotationalPower = (float) (rotationalMagnitude * (Math.PI/4 - Math.abs(vectorR.getRad()-robot.getVectorR().getRad())));
                        }else{
                            rotationalPower = rotationalMagnitude;
                        }
                    } else if (vectorR.getRad() < robot.getVectorR().getRad()){
                        if(Math.abs(vectorR.getRad()-robot.getVectorR().getRad())<(Math.PI/4)){
                            rotationalPower = (float) (-rotationalMagnitude * (Math.PI/4 - Math.abs(vectorR.getRad()-robot.getVectorR().getRad())));
                        }else{
                            rotationalPower = -rotationalMagnitude;
                        }
                    } else {
                        rotationalPower = 0;
                    }
                }

                //calculates the shaft magnitude (AC shaft has diagonal motors "A" and "C")
                float ACShaftPower = (float) -((Math.sin(robotTheta - (Math.PI / 4))) * robotVelocity);
                float BDShaftPower = (float) -((Math.cos(robotTheta - (Math.PI / 4))) * robotVelocity);

                //sets the motor power where the ratio of input from translational motion is dictated by the magnitude of the rotational motion
                double ACRotationalPower = (rotationalPower+ACShaftPower) == 0 ? 0 : (rotationalPower*Math.abs(rotationalPower))/(Math.abs(rotationalPower)+Math.abs(ACShaftPower));
                double BDRotationalPower = (rotationalPower+BDShaftPower) == 0 ? 0 : (rotationalPower*Math.abs(rotationalPower))/(Math.abs(rotationalPower)+Math.abs(BDShaftPower));

                double fr = (-ACRotationalPower)+(ACShaftPower*(1.0-Math.abs(ACRotationalPower)));
                double fl = (BDRotationalPower)+(BDShaftPower*(1.0-Math.abs(BDRotationalPower)));
                double bl = (ACRotationalPower)+(ACShaftPower*(1.0-Math.abs(ACRotationalPower)));
                double br = (-BDRotationalPower)+(BDShaftPower*(1.0-Math.abs(BDRotationalPower)));

                //calculates the motor powers
                frMotor.setPower(Range.scale(fr, -1, 1, -motorScale, motorScale));
                flMotor.setPower(Range.scale(fl, -1, 1, -motorScale, motorScale));
                blMotor.setPower(Range.scale(bl, -1, 1, -motorScale, motorScale));
                brMotor.setPower(Range.scale(br, -1, 1, -motorScale, motorScale));

                double currentVelocity = (blMotor.getCurrentPosition()-pastPosition)*(/*time**/pastPositionTime);
                //TODO: Continue work here(uncomment first)
                pastPositionTime = System.nanoTime();

                if(currentVelocity>0){
                    velocityFeedback.setX((float) ((currentVelocity)*(Math.cos(Math.PI/3))));
                    velocityFeedback.setY((float) ((currentVelocity)*(Math.cos(Math.PI/3))));
                }
                pastPosition = blMotor.getCurrentPosition();

                /* spams the console
                telemetry.addData("Speed-FR", fr);
                telemetry.addData("Speed-FL", fl);
                telemetry.addData("Speed-BL", bl);
                telemetry.addData("Speed-BR", br);
                */
            }
        }
    };

    private Thread driveThread = new Thread(driveRunnable); //initializes driveThread based on driveRunnable

    /**
     * Constructor
     */
    public Drive(VectorR vectorR, Robot robot, DcMotor frMotor,
                 DcMotor flMotor, DcMotor blMotor, DcMotor brMotor,
                 Config.ParsedData settings, Telemetry telemetry){

        this.vectorR = vectorR;
        this.robot = robot;
        this.frMotor = frMotor;
        this.flMotor = flMotor;
        this.brMotor = brMotor;
        this.blMotor = blMotor;
        this.driveSettings = settings.subData("drivetrain");
        this.vectorDriveActive = true;
        this.joystickControl = false;
        this.telemetry = telemetry;

        this.blueTeam = false;
        if (settings.getString("alliance") == "blue")
            this.blueTeam = true;
        this.relativeDrive = false;

        this.settings = settings;
        String team = settings.getString("alliance");
        //if (fieldmap!=null) this.fieldmap = fieldmap.subData("coordinates").subData(team);

        /*
        if (this.settings.subData("drive").getBool("init_with_fieldmap")){
            String initial_position = settings.getString("position");
            if (((!initial_position.equals("inside"))) && (!initial_position.equals("outside"))){
                initial_position = "inside";
            }
            robot.position.setX(this.fieldmap.subData(initial_position).getFloat("x"));
            robot.position.setY(this.fieldmap.subData(initial_position).getFloat("y"));
            robot.rotation.setRadians(0.0f);
        }

        else {
            robot.position.setX(settings.subData("robot").subData("initial_position").getFloat("x"));
            robot.position.setY(settings.subData("robot").subData("initial_position").getFloat("y"));
            robot.rotation.setRadians(settings.subData("robot").getFloat("initial_rotation"));
        }*/

        robot.position.setX(settings.subData("robot").subData("initial_position").getFloat("x"));
        robot.position.setY(settings.subData("robot").subData("initial_position").getFloat("y"));
        robot.rotation.setRadians(settings.subData("robot").getFloat("initial_rotation"));

        //sensorHandler =

        DcMotor.RunMode runMode = DcMotor.RunMode.RUN_USING_ENCODER;
        frMotor.setMode(runMode);
        flMotor.setMode(runMode);
        brMotor.setMode(runMode);
        blMotor.setMode(runMode);

        if(driveSettings.subData("motor").subData("front_right").getBool("reversed")){frMotor.setDirection(DcMotor.Direction.REVERSE);}else{frMotor.setDirection(DcMotor.Direction.FORWARD);}
        if(driveSettings.subData("motor").subData("front_left").getBool("reversed")){flMotor.setDirection(DcMotor.Direction.REVERSE);}else{flMotor.setDirection(DcMotor.Direction.FORWARD);}
        if(driveSettings.subData("motor").subData("back_left").getBool("reversed")){blMotor.setDirection(DcMotor.Direction.REVERSE);}else{blMotor.setDirection(DcMotor.Direction.FORWARD);}
        if(driveSettings.subData("motor").subData("back_right").getBool("reversed")){brMotor.setDirection(DcMotor.Direction.REVERSE);}else{brMotor.setDirection(DcMotor.Direction.FORWARD);}
        motorScale = driveSettings.getFloat("motor_scale");
    }

    /**
     * Starts driveThread and changes vectorDriveActive to true
     */
    public void startVelocity(){
        vectorDriveActive = true;
        driveThreadActive = true;
        if (!driveThread.isAlive())driveThread.start();
    }

    /**
     * Uses driveThread to move robot to position
     */
    public void startPosition(){
        //this flag should be enough to announce that a math change is needed. Robot's current
        // position can be gained from getVectorR and the vectorR provided is the aim position.
        vectorDriveActive = false;
        driveThreadActive = true;
        if (!driveThread.isAlive())driveThread.start();
    }

    /**
     * Stops driveThread and changes vectorDriveActive to false
     * Resets relativeDrive to true
     */
    public void stop(){
        //friendly stop
        driveThreadActive = false;
        this.relativeDrive = true;
        vectorDriveActive = false;
        //if(driveThread.isAlive()){driveThread.interrupt();}
        /* todo ask if this is needed
        frMotor.setPower(0.0);
        flMotor.setPower(0.0);
        brMotor.setPower(0.0);
        blMotor.setPower(0.0);
        */
    }

    /**
     * Sets what the robot drives relative to
     *
     * @param isRelative (true: robot drives relative to field/ false: robot drives relative to own orientation)
     */
    public void setRelative(boolean isRelative){
        relativeDrive = isRelative;
    }

    VectorR velocityFeedback = new VectorR();

    VectorR returnVelocityFeedback(){
        return velocityFeedback;
    }

    public void setVelocity(float radius, float theta){
        this.vectorR.setPolar(radius, theta);
        startVelocity();
    }

    public void setPosition(float x, float y){
        this.vectorR.setX(x);
        this.vectorR.setY(y);
        startPosition();
    }

    public void setRoation(float rotation){
        this.vectorR.setRad(rotation);
        if (vectorDriveActive) startVelocity();
        else startPosition();
    }

    /*
        I really do not know how the robot object would update its position
        THIS ONLY THEORETICALLY WORKS WITHOUT ROBOT ROTATION
        Method is untested but there is a test in the framework_test branch under
        Test_drive3, where the encoder outputs are calculated into X and Y coordinates
    */
    public void updatePosition(){
        String updateDevice = settings.subData("drive").getString("update_using");
        switch(updateDevice){
            case "encoders":
                robot.position.setX(getEncoderX());
                robot.position.setY(getEncoderY());
                break;
            case "camera":
                //todo: finish algorithm for gaining position in reference to an image
                break;
            case "imu":
                //todo: check status of imu readiness
                break;
            default:
                robot.position.setX(getEncoderX());
                robot.position.setY(getEncoderY());
        }
    }

    /**
     * Gets the x coordinate of the robot using the 4 encoders
     * assuming that the robot does not rotate
     * @return the x coordinate
     */
    private float getEncoderX(){
        float inch_per_pulse = 4f  * (float)Math.PI / settings.subData("encoder").getFloat("output_pulses");
        double motorAngle = Math.toRadians(driveSettings.getFloat("motor_angle"));
        float A = -frMotor.getCurrentPosition()*inch_per_pulse;
        float B = -flMotor.getCurrentPosition()*inch_per_pulse;
        float C = -blMotor.getCurrentPosition()*inch_per_pulse;
        float D = -brMotor.getCurrentPosition()*inch_per_pulse;
        float AC = ((A*(float)Math.cos(Math.PI-motorAngle)) + (C*(float)Math.cos(Math.PI-motorAngle)))/2.0f;
        float BD = ((B*(float)Math.cos(motorAngle)) + (D*(float)Math.cos(motorAngle)))/2.0f;
        return  ((AC + BD) / 2.0f) + initialX;
    }

    /**
     * Gets the y coordinate of the robot using the 4 encoders
     * assuming that the robot does not rotate
     * @return the y coordinate
     */
    private float getEncoderY(){
        float inch_per_pulse = 4f  * (float)Math.PI / settings.subData("encoder").getFloat("output_pulses");
        double motorAngle = Math.toRadians(driveSettings.getFloat("motor_angle"));
        float A = -frMotor.getCurrentPosition()*inch_per_pulse;
        float B = -flMotor.getCurrentPosition()*inch_per_pulse;
        float C = -blMotor.getCurrentPosition()*inch_per_pulse;
        float D = -brMotor.getCurrentPosition()*inch_per_pulse;
        float AC = ((A*(float)Math.sin(Math.PI-motorAngle)) + (C*(float)Math.sin(Math.PI-motorAngle)))/2.0f;
        float BD = ((B*(float)Math.sin(motorAngle)) + (D*(float)Math.sin(motorAngle)))/2.0f;
        return  ((AC + BD) / 2.0f) + initialY;
    }

    public boolean atLocation(){
        //todo learn how to convert yml list to a list of strings
        float xMargin = settings.subData("drive").getFloat("x_margin");
        float yMargin = settings.subData("drive").getFloat("y_margin");
        float robotX = robot.position.getX();
        float robotY = robot.position.getY();
        float setX = vectorR.getX();
        float setY = vectorR.getY();
        return ((xMargin>Math.abs(robotX-setX))&&(yMargin>Math.abs(robotY-setY)));
    }


}
