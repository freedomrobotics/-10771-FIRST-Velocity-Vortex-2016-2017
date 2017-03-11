package org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.lsm9ds0;

import android.support.annotation.NonNull;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.MagneticFlux;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Temperature;

/**
 * A home-made interface for an incompatible Adafruit
 * LSM9DS0 sensor
 * Created by joelv on 3/10/2017.
 */
public interface LSM9DS0{

    boolean initialize(@NonNull Parameters parameters);

    boolean initialize();

    class Parameters{

    }

    void close();

    Orientation getOrientation();

    Acceleration getOverallAcceleration();

    AngularVelocity getAngularVelocity();

    Acceleration getLinearAcceleration();

    Acceleration getGravity();

    Temperature getTemperature();

    MagneticFlux getMagneticFieldStrength();

    interface GyroIntegrator{

    }

    interface AcclerationIntegrator{

    }

    //System Status

    //System error here

    byte  read8(Register register);
    /**
     * Low level: read data starting at the indicated register
     * @param register  the location from which to read the data
     * @param cb        the number of bytes to read
     * @return          the data that was read
     */
    byte[] read(Register register, int cb);

    /**
     * Low level: write a byte to the indicated register
     * @param register  the location at which to write the data
     * @param bVal      the data to write
     */
    void write8(Register register, int bVal);
    /**
     * Low level: write data starting at the indicated register
     * @param register  the location at which to write the data
     * @param data      the data to write
     */
    void write (Register register, byte[] data);

    enum Register
        {
            //RESERVED from 00-0E
            WHO_AM_I_G(0x0F),           //READ-ONLY
            //RESERVED from 10-1F
            CTRL_REG1_G(0x20),          //READ/WRITE
            CTRL_REG2_G(0x21),          //READ/WRITE
            CTRL_REG3_G(0x22),          //READ/WRITE
            CTRL_REG4_G(0x23),          //READ/WRITE
            CTRL_REG5_G(0x24),          //READ/WRITE
            REFERENCE_G(0X25),          //READ/WRITE
            //RESERVED at 0x26
            STATUS_REG_G(0x27),         //READ-ONLY
            OUT_X_L_G(0x28),            //READ-ONLY
            OUT_X_H_G(0X29),            //READ-ONLY
            OUT_Y_L_G(0x2A),            //READ-ONLY
            OUT_Y_H_G(0x2B),            //READ-ONLY
            OUT_Z_L_G(0x2C),            //READ-ONLY
            OUT_Z_H_G(0x2D),            //READ-ONLY
            FIFO_CTRL_REG_G(0x2E),      //READ/WRITE
            FIFO_SRC_REG_G(0x2F),       //READ-ONLY
            INT1_CFG_G(0x30),           //READ/WRITE
            INT1_SRC_G(0x31),           //READ-ONLY
            INT1_TSH_XH_G(0x32),        //READ/WRITE
            INT1_TSH_XL_G(0x33),        //READ/WRITE
            INT1_TSH_YH_G(0x34),        //READ/WRITE
            INT1_TSH_YL_G(0x35),        //READ/WRITE
            INT1_TSH_ZH_G(0x36),        //READ/WRITE
            INT1_TSH_ZL_G(0x37),        //READ/WRITE
            INT1_DURATION_G(0x38),      //READ/WRITE
            //RESERVED AT 00-04
            OUT_TEMP_L_XM(0x05),        //READ-ONLY
            OUT_TEMP_H_XM(0x06),        //READ-ONLY
            STATUS_REG_M(0x07),         //READ-ONLY
            OUT_X_L_M(0x08),            //READ-ONLY
            OUT_X_H_M(0x09),            //READ-ONLY
            OUT_Y_L_M(0x0A),            //READ-ONLY
            OUT_Y_H_M(0x0B),            //READ-ONLY
            OUT_Z_L_M(0x0C),            //READ-ONLY
            OUT_Z_H_M(0x0D),            //READ-ONLY
            //RESERVED AT 0E
            WHO_AM_I_XM(0x0F),          //READ-ONLY
            //RESERVED AT 0X10 AND 0X11
            INT_CTRL_REG_M(0x12),       //READ/WRITE
            INT_SRC_REG_M(0x13),        //READ-ONLY
            INT_THS_L_M(0x14),          //READ/WRITE
            INT_THS_H_M(0x15),          //READ/WRITE
            OFFSET_X_L_M(0x16),         //READ/WRITE
            OFFSET_X_H_M(0x17),         //READ/WRITE
            OFFSET_Y_L_M(0x18),         //READ/WRITE
            OFFSET_Y_H_M(0x19),         //READ/WRITE
            OFFSET_Z_L_M(0x1A),         //READ/WRITE
            OFFSET_Z_H_M(0x1B),         //READ/WRITE
            REFERENCE_X(0x1C),          //READ/WRITE
            REFERENCE_Y(0x1D),          //READ/WRITE
            REFERENCE_Z(0x1E),          //READ/WRITE
            CTRL_REG0_XM(0x1F),         //READ/WRITE
            CTRL_REG1_XM(0x20),         //READ/WRITE
            CTRL_REG2_XM(0x21),         //READ/WRITE
            CTRL_REG3_XM(0x22),         //READ/WRITE
            CTRL_REG4_XM(0x23),         //READ/WRITE
            CTRL_REG5_XM(0x24),         //READ/WRITE
            CTRL_REG6_XM(0x25),         //READ/WRITE
            CTRL_REG7_XM(0x26),         //READ/WRITE
            STATUS_REG_A(0x27),         //READ-ONLY
            OUT_X_L_A(0x28),            //READ-ONLY
            OUT_X_H_A(0x29),            //READ-ONLY
            OUT_Y_L_A(0x2A),            //READ-ONLY
            OUT_Y_H_A(0x2B),            //READ-ONLY
            OUT_Z_L_A(0x2C),            //READ-ONLY
            OUT_Z_H_A(0x2D),            //READ-ONLY
            FIFO_CTRL_REG(0x2E),        //READ/WRITE
            FIFO_SRC_REG(0x2F),         //READ-ONLY
            INT_GEN_1_REG(0x30),        //READ/WRITE
            INT_GEN_1_SRC(0x31),        //READ-ONLY
            INT_GEN_1_THS(0x32),        //READ/WRITE
            INT_GEN_1_DURATION(0x33),   //READ/WRITE
            INT_GEN_2_REG(0x34),        //READ/WRITE
            INT_GEN_2_SRC(0x35),        //READ-ONLY
            INT_GEN_2_THS(0x36),        //READ/WRITE
            INT_GEN_2_DURATION(0x37),   //READ/WRITE
            CLICK_CFG(0x38),            //READ/WRITE
            CLICK_SRC(0x39),            //READ-ONLY
            CLICK_THS(0x3A),            //READ/WRITE
            TIME_LIMIT(0x3B),           //READ/WRITE
            TIME_LATENCY(0x3C),         //READ/WRITE
            TIME_WINDOW(0x3D),          //READ/WRITE
            Act_THS(0x3E),              //READ/WRITE
            Act_DUR(0x3F)               //READ/WRITE
            ;

        public final byte bVal;
        Register(int i){
            this.bVal = (byte)i;
        }
    }
}
