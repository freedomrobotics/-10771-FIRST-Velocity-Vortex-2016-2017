package org.fhs.robotics.ftcteam10771.lepamplemousse.actions.imu;

import android.hardware.Sensor;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.MagneticFlux;

/**
 * The class that handles magnetic readings
 * by the IMU
 * Created by joelv on 1/14/2017.
 */
public class Magnetometer{

    private IMU handler;
    private MagneticFlux magneticFlux;
    private boolean magnetStreamEnabled = true;

    /*
        Default constructor
     */
    public Magnetometer(){

    }

    /*
        Constructor that requires reference to an IMU
     */
    public Magnetometer(IMU handler){
        new Magnetometer(handler, true);
    }

    /*
        Constructor that requires user to indicate both IMU
        and whether to initialize the IMU instantly
     */
    public Magnetometer(IMU handler, boolean initilize){
        this.handler = handler;
        if (initilize){
            handler.imu.initialize();
        }
    }

    //The runnable to stream the magnetic readings
    public final Runnable magnetRunnable = new Runnable() {
        @Override
        public void run() {
            while(!Thread.interrupted()){
                streamMagnetometerData();
            }
        }
    };

    //The respective thread
    public final Thread magnetThread = new Thread(magnetRunnable);

    /**
     * Reads the magnetic strength from IMU
     * @return the magetic field strength
     */
    public MagneticFlux magneticFlux(){
        return handler.imu.getMagneticFieldStrength();
    }

    /**
     * Function that streams data to
     * the private variable at one instant;
     * To be used in a looping thread
     */
    public void streamMagnetometerData(){
         if (magnetStreamEnabled){
             magneticFlux = magneticFlux();
         }
    }

    /**
     * Toggle the stream method
     * @param state of the stream method
     */
    public void enableStream(boolean state){
        magnetStreamEnabled = state;
    }

    /**
     * Getter for the stream private variable
     * @return the private object variable
     */
    public MagneticFlux getRunnableMagneticFlux(){
        return magneticFlux;
    }

    /**
     * Obtains the magnetic field readings of one axis
     * @param axis the chosen axis
     * @param useRunnable whether to use private streaming variable
     * @return the magnetic readings of one axis
     */
    public double getMagneticFlux(IMU.Axis axis, boolean useRunnable){
        switch (axis){
            case X:
                return useRunnable ? magneticFlux.x : magneticFlux().x;
            case Y:
                return useRunnable ? magneticFlux.y : magneticFlux().y;
            case Z:
                return useRunnable ? magneticFlux.z : magneticFlux().z;
            default:
                return 0.0;
        }
    }

    /**
     * Obtains magnetic field strength of one axis
     * Default useRunnable state: false
     * @param axis the chosen axis
     * @return the magnetic readings at that instant
     */
    public double getMagneticFlux(IMU.Axis axis){
        return getMagneticFlux(axis, false);
    }

    /**
     * Test function for the IMU's magnetic readings
     * @param opMode the op mode used to test sensor
     * @param updateTelemetry whether to update telemetry
     */
    public void testMagnetometer(LinearOpMode opMode, boolean updateTelemetry){
        opMode.telemetry.addData("MagnetX", getMagneticFlux(IMU.Axis.X));
        opMode.telemetry.addData("MagnetY", getMagneticFlux(IMU.Axis.Y));
        opMode.telemetry.addData("MagnetZ", getMagneticFlux(IMU.Axis.Z));
        if (updateTelemetry) opMode.telemetry.update();
    }
}
