package org.fhs.robotics.ftcteam10771.lepamplemousse.actions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.fhs.robotics.ftcteam10771.lepamplemousse.config.Config;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.entities.Robot;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.vector.VectorR;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.Math;

/**
 * Created by Adam Li on 10/27/2016.
 * Class to manage how the robot drives (smart control schemes, drivetrain types, etc.)
 * todo CLEANUP
 */
public class Drive {

    // State-related objects
    private VectorR vectorR;
    private Robot robot;
    private VectorR velocityFeedback = new VectorR();
    private VectorR lastPosition = new VectorR();
    private int pastPosition;
    private long pastPositionTime;

    // IO
    //private final Telemetry telemetry;
    private DcMotor frMotor;
    private DcMotor flMotor;
    private DcMotor brMotor;
    private DcMotor blMotor;
    private final Config.ParsedData driveSettings;
    private final Config.ParsedData settings;

    //Flags
    private boolean vectorDriveActive;
    private boolean relativeDrive;
    // TODO: 2/19/2017 change to alliance
    private final boolean blueTeam;

    //Positional Flags
    private boolean atPosition;
    private boolean atRotation;

    //Values
    private final float motorScale;

    //Positional drive stuff
    // FIXME: 2/19/2017 move this stuff out
    private float initialEncoderX = 0.0f;
    private float initialEncoderY = 0.0f;

    //Remove magic number!
    public enum Motors {
        FRONT_RIGHT,
        FRONT_LEFT,
        BACK_LEFT,
        BACK_RIGHT,
    }

    //Motor Percents
    private double fr;
    private double fl;
    private double bl;
    private double br;

    //Motor Powers
    private double frPow;
    private double flPow;
    private double blPow;
    private double brPow;

    //settings
    private final float position_tolerance;
    private final float position_speed;
    private final float rotation_tolerance;
    private final float rotation_speed;
    private final float rotation_speed_min;

    private final float centimeters_per_pulse;
    private final double motorAngle;

    private Runnable driveRunnable = new Runnable() {
        @Override
        public void run() {

            while (!Thread.currentThread().isInterrupted()) {

                float theta = vectorR.getTheta();
                float velocity;
                float rotation;
                updatePosition();
                if (vectorDriveActive) {
                    //sets values from the vectorR needed for movement
                    velocity = vectorR.getRadius();
                    rotation = vectorR.getRawR();

                    if (!relativeDrive) { //if the robot drives relative to the field
                        theta = theta - (float)((Math.PI * 2.0) + robot.getRotation().getHeading());
                    }
                } else {
                    VectorR difference = VectorR.sub(vectorR, robot.getVectorR());
                    theta = difference.getTheta();

                    //todo put drivetrain/positional/position_margin
                    atPosition = difference.getRadius() < Math.abs(position_tolerance);

                    if (!atPosition)
                        velocity = position_speed;
                    else velocity = 0;


                    float factor = difference.getRad() > 3.1415927f ? (6.2831854f - difference.getRad()) / 3.1415927f : difference.getRad() / 3.1415927f;
                    atRotation = factor < rotation_tolerance;

                    if (!atRotation) {
                        float sign = vectorR.getRad() - robot.getVectorR().getRad();
                        rotation = (float) Math.copySign(Range.scale(factor, 0, 1,
                                rotation_speed_min, rotation_speed),
                                difference.getRad() > 3.1415927f ? -sign : sign);
                    } else rotation = 0;
                }

                //calculates the shaft magnitude (AC shaft has diagonal motors "A" and "C")
                float ACShaftPower = (float) -((Math.sin(theta - (Math.PI / 4))) * velocity);
                float BDShaftPower = (float) -((Math.cos(theta - (Math.PI / 4))) * velocity);

                //sets the motor power where the ratio of input from translational motion is dictated by the magnitude of the rotational motion
                double ACRotationalPower = (rotation+ACShaftPower) == 0 ? 0 : (rotation*Math.abs(rotation))/(Math.abs(rotation)+Math.abs(ACShaftPower));
                double BDRotationalPower = (rotation+BDShaftPower) == 0 ? 0 : (rotation*Math.abs(rotation))/(Math.abs(rotation)+Math.abs(BDShaftPower));

                fr = (-ACRotationalPower)+(ACShaftPower*(1.0-Math.abs(ACRotationalPower)));
                fl = (BDRotationalPower)+(BDShaftPower*(1.0-Math.abs(BDRotationalPower)));
                bl = (ACRotationalPower)+(ACShaftPower*(1.0-Math.abs(ACRotationalPower)));
                br = (-BDRotationalPower)+(BDShaftPower*(1.0-Math.abs(BDRotationalPower)));

                frPow = Range.scale(fr, -1, 1, -motorScale, motorScale);
                flPow = Range.scale(fl, -1, 1, -motorScale, motorScale);
                blPow = Range.scale(bl, -1, 1, -motorScale, motorScale);
                brPow = Range.scale(br, -1, 1, -motorScale, motorScale);

                //calculates the motor powers
                if (!Thread.currentThread().isInterrupted()){
                    frMotor.setPower(frPow);
                    flMotor.setPower(flPow);
                    blMotor.setPower(blPow);
                    brMotor.setPower(brPow);
                }
                else{
                    frMotor.setPower(0.0f);
                    flMotor.setPower(0.0f);
                    blMotor.setPower(0.0f);
                    brMotor.setPower(0.0f);
                }


                double currentVelocity = (blMotor.getCurrentPosition()-pastPosition)*(/*time**/pastPositionTime);
                //TODO: Continue work here(uncomment first)
                pastPositionTime = System.nanoTime();

                if(currentVelocity>0){
                    velocityFeedback.setX((float) ((currentVelocity)*(Math.cos(Math.PI/3))));
                    velocityFeedback.setY((float) ((currentVelocity)*(Math.cos(Math.PI/3))));
                }
                pastPosition = blMotor.getCurrentPosition();
            }
            frMotor.setPower(0.0);
            flMotor.setPower(0.0);
            blMotor.setPower(0.0);
            brMotor.setPower(0.0);
            frMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            flMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    };

    private Thread driveThread = new Thread(driveRunnable); //initializes driveThread based on driveRunnable

    /**
     * Constructor
     * Use Constructor without telemetry instead
     */
    @Deprecated
    public Drive(VectorR vectorR, Robot robot, DcMotor frMotor,
                 DcMotor flMotor, DcMotor blMotor, DcMotor brMotor,
                 Config.ParsedData settings, Telemetry telemetry){
        this(vectorR, robot, frMotor, flMotor, blMotor, brMotor, settings);
    }

    /**
     * Constructor
     */
    public Drive(VectorR vectorR, Robot robot, DcMotor frMotor,
                DcMotor flMotor, DcMotor blMotor, DcMotor brMotor,
                Config.ParsedData settings){

        this.vectorR = vectorR;
        this.robot = robot;
        this.frMotor = frMotor;
        this.flMotor = flMotor;
        this.brMotor = brMotor;
        this.blMotor = blMotor;
        this.driveSettings = settings.subData("drivetrain");
        this.vectorDriveActive = true;

        this.blueTeam = settings.getString("alliance").equals("blue");
        this.relativeDrive = false;

        this.settings = settings;

        initialEncoderX = robot.getPosition().getX();
        initialEncoderY = robot.getPosition().getY();

        reset();

        if(driveSettings.subData("motor").subData("front_right").getBool("reversed")){
            frMotor.setDirection(DcMotor.Direction.REVERSE);
        } else{
            frMotor.setDirection(DcMotor.Direction.FORWARD);
        }

        if(driveSettings.subData("motor").subData("front_left").getBool("reversed")){
            flMotor.setDirection(DcMotor.Direction.REVERSE);
        } else{
            flMotor.setDirection(DcMotor.Direction.FORWARD);
        }

        if(driveSettings.subData("motor").subData("back_left").getBool("reversed")){
            blMotor.setDirection(DcMotor.Direction.REVERSE);
        } else{
            blMotor.setDirection(DcMotor.Direction.FORWARD);
        }

        if(driveSettings.subData("motor").subData("back_right").getBool("reversed")){
            brMotor.setDirection(DcMotor.Direction.REVERSE);
        } else{
            brMotor.setDirection(DcMotor.Direction.FORWARD);
        }

        motorScale = driveSettings.getFloat("motor_scale");

        atPosition = false;
        atRotation = false;

        position_tolerance = driveSettings.subData("positional").getFloat("position_tolerance");
        position_speed = driveSettings.subData("positional").getFloat("speed");
        rotation_tolerance = driveSettings.subData("positional").getFloat("rotational_tolerance") / 180f;
        rotation_speed = driveSettings.subData("positional").getFloat("rotation");
        rotation_speed_min = driveSettings.subData("positional").getFloat("rotation_min");

        centimeters_per_pulse = settings.subData("encoder").getFloat("centimeters_per_pulse");
        motorAngle = Math.toRadians(settings.subData("drivetrain").getFloat("motor_angle"));
    }

    /**
     * Starts driveThread and changes vectorDriveActive to true
     */
    public void startVelocity(){
        vectorDriveActive = true;
        if (!driveThread.isAlive())
            driveThread.start();
    }

    /**
     * Uses driveThread to move robot to position
     */
    public void startPosition(){
        //this flag should be enough to announce that a math change is needed. Robot's current
        // position can be gained from getVectorR and the vectorR provided is the aim position.
        vectorDriveActive = false;
        atRotation = false;
        atPosition = false;
        if (!driveThread.isAlive())
            driveThread.start();
    }

    /**
     * Stops driveThread and changes vectorDriveActive to false
     * Resets relativeDrive to true
     */
    public void stop(){
        relativeDrive = true;
        vectorDriveActive = false;
        if(driveThread.isAlive())
            driveThread.interrupt();
    }

    /**
     * Sets what the robot drives relative to
     *
     * @param isRelative (true: robot drives relative to field/ false: robot drives relative to own orientation)
     */
    public void setRelative(boolean isRelative){
        relativeDrive = isRelative;
    }

    VectorR returnVelocityFeedback(){
        return velocityFeedback;
    }

    /*
        I really do not know how the robot object would update its position
        THIS ONLY THEORETICALLY WORKS WITHOUT ROBOT ROTATION
        Method is untested but there is a test in the framework_test branch under
        Test_drive3, where the encoder outputs are calculated into X and Y coordinates
    */
    public void updatePosition(){
        float xPos = getEncoderX();
        float yPos = getEncoderY();
        float x = xPos - lastPosition.getX();
        float y = yPos - lastPosition.getY();
        double theta = Math.atan2(y, x);
        double radius = Math.sqrt(x*x + y*y);
        theta -= (float)((Math.PI * 2.0) + robot.getRotation().getHeading());
        robot.getPosition().movePolar((float)radius, (float)theta);
        lastPosition.setX(xPos);
        lastPosition.setY(yPos);
    }

    /**
     * Uses Encoders and roller angle
     * to obtain X coordinate in centimeter
     * Note: Only accurate when robot is not rotated at all
     * @return X coodinate in centimeters
     */
    public float getEncoderX(){
        float A = -frMotor.getCurrentPosition()*centimeters_per_pulse;
        float B = -flMotor.getCurrentPosition()*centimeters_per_pulse;
        float C = -blMotor.getCurrentPosition()*centimeters_per_pulse;
        float D = -brMotor.getCurrentPosition()*centimeters_per_pulse;
        float AC = ((A*(float)Math.cos(Math.PI-motorAngle)) + (C*(float)Math.cos(Math.PI-motorAngle)))/2.0f;
        float BD = ((B*(float)Math.cos(motorAngle)) + (D*(float)Math.cos(motorAngle)))/2.0f;
        return  ((AC + BD) / 2.0f) + initialEncoderX;
    }

    /**
     * Uses Encoder outputs to obtain y coordinate
     * Note: only works if robot does not rotate
     * @return Y coordinate of the robot
     */
    public float getEncoderY(){
        float A = -frMotor.getCurrentPosition()*centimeters_per_pulse;
        float B = -flMotor.getCurrentPosition()*centimeters_per_pulse;
        float C = -blMotor.getCurrentPosition()*centimeters_per_pulse;
        float D = -brMotor.getCurrentPosition()*centimeters_per_pulse;
        float AC = (A+C)/2.0f;
        float BD = (B+D)/2.0f;
        return  ((AC + BD) / 2.0f) + initialEncoderY;
    }

    public float getMotorPercent(Motors motor){
        switch (motor) {
            case FRONT_RIGHT:
                return (float) fr;
            case FRONT_LEFT:
                return (float) fl;
            case BACK_LEFT:
                return (float) bl;
            case BACK_RIGHT:
                return (float) br;
        }
        return 0.0f;
    }

    public float getMotorPower(Motors motor){
        switch (motor) {
            case FRONT_RIGHT:
                return (float) frPow;
            case FRONT_LEFT:
                return (float) flPow;
            case BACK_LEFT:
                return (float) blPow;
            case BACK_RIGHT:
                return (float) brPow;
        }
        return 0.0f;
    }

    public int getMotorEncoder(Motors motor){
        switch (motor) {
            case FRONT_RIGHT:
                return frMotor.getCurrentPosition();
            case FRONT_LEFT:
                return flMotor.getCurrentPosition();
            case BACK_LEFT:
                return blMotor.getCurrentPosition();
            case BACK_RIGHT:
                return brMotor.getCurrentPosition();
        }
        return 0;
    }

    public float getCurrentX(){
        return robot.getPosition().getX();
    }

    public float getCurrentY(){
        return robot.getPosition().getY();
    }

    public boolean isVectorDriveActive(){
        return vectorDriveActive;
    }

    public void reset(){
        initialEncoderX = getEncoderX();
        initialEncoderY = getEncoderY();
        frMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        blMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * delete when possible Does nothing now
     */
    @Deprecated
    public void initPosition(float x, float y){
        //reset();
        //initialX = x;
        //initialY = y;
    }

    public boolean isAtPosition(){
        return atPosition;
    }

    public boolean isAtRotation(){
        return atRotation;
    }

    public VectorR getVectorR(){
        return vectorR;
    }
}
