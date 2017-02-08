/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.fhs.robotics.ftcteam10771.lepamplemousse.position.calculations;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.util.RobotLog;

import org.fhs.robotics.ftcteam10771.lepamplemousse.actions.imu.IMU;
import org.fhs.robotics.ftcteam10771.lepamplemousse.config.Config;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import static org.fhs.robotics.ftcteam10771.lepamplemousse.actions.imu.IMU.Axis.X;
import static org.fhs.robotics.ftcteam10771.lepamplemousse.actions.imu.IMU.Axis.Y;
import static org.fhs.robotics.ftcteam10771.lepamplemousse.actions.imu.IMU.Axis.Z;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.NavUtil.meanIntegrate;
import static org.firstinspires.ftc.robotcore.external.navigation.NavUtil.minus;
import static org.firstinspires.ftc.robotcore.external.navigation.NavUtil.plus;
import static org.firstinspires.ftc.robotcore.external.navigation.NavUtil.scale;

/**
 * {@link KalmanFilterAccelerationIntegrator} is based on the
 * {@link com.qualcomm.hardware.adafruit.NaiveAccelerationIntegrator}.
 * It uses the Kalman filter to filter noise.
 */
public class KalmanFilterAccelerationIntegrator implements BNO055IMU.AccelerationIntegrator {
    //------------------------------------------------------------------------------------------
    // State
    //------------------------------------------------------------------------------------------

    BNO055IMU.Parameters parameters;
    BNO055IMU imu;
    Position position;
    Velocity velocity;
    Acceleration acceleration;

    Config.ParsedData kalmanConfig;

    Position positionError;
    float velocityError;
    float accelerationError;

    float processNoise;
    float sensorNoise;

    public Position getPosition() {
        return this.position;
    }

    public Velocity getVelocity() {
        return this.velocity;
    }

    public Acceleration getAcceleration() {
        return this.acceleration;
    }

    //------------------------------------------------------------------------------------------
    // Construction
    //------------------------------------------------------------------------------------------

    public KalmanFilterAccelerationIntegrator(Config.ParsedData kalmanConfig, BNO055IMU imu) {
        this.parameters = null;
        this.position = null;
        this.velocity = null;
        this.acceleration = null;
        this.kalmanConfig = kalmanConfig;
        this.imu = imu;
    }

    //------------------------------------------------------------------------------------------
    // Operations
    //------------------------------------------------------------------------------------------

    @Override
    public void initialize(BNO055IMU.Parameters parameters, Position initialPosition, Velocity initialVelocity) {
        this.parameters = parameters;
        this.position = initialPosition;
        this.velocity = initialVelocity;
        this.acceleration = null;
        this.positionError = new Position();
        positionError.x = kalmanConfig.subData("initial_error").getFloat("x");
        positionError.y = kalmanConfig.subData("initial_error").getFloat("y");
        positionError.z = kalmanConfig.subData("initial_error").getFloat("z");
        velocityError = kalmanConfig.getFloat("initial_velocity_error");
        processNoise = kalmanConfig.getFloat("process_noise");
        sensorNoise = kalmanConfig.getFloat("sensor_noise");
        accelerationError = kalmanConfig.getFloat("initial_acceleration_error");
    }

    //This is where to perform the actual math
    @Override
    public void update(Acceleration linearAcceleration) {
        // We should always be given a timestamp here
        if (linearAcceleration.acquisitionTime != 0) {
            // We can only integrate if we have a previous acceleration to baseline from
            if (acceleration != null) {
                Acceleration accelPrev = acceleration;
                Velocity velocityPrev = velocity;

                acceleration = getAbsoluteAcceleration(linearAcceleration);

                // filter acceleration
                accelerationError += processNoise;
                float accelGain = accelerationError / (accelerationError + sensorNoise);
                accelerationError = (1.0f - accelGain) * accelerationError;
                acceleration = plus(accelPrev,scale(minus(acceleration, accelPrev), accelGain));

                if (accelPrev.acquisitionTime != 0) {
                    velocityError = velocityError + processNoise;

                    float kalmanGain = velocityError/(velocityError+sensorNoise);
                    velocityError = (1.0f - kalmanGain) * velocityError;

                    Velocity gainVelocity = meanIntegrate(acceleration, accelPrev);
                    gainVelocity = scale(gainVelocity, kalmanGain);
                    velocity = plus(velocity, gainVelocity);
                }

                if (velocityPrev.acquisitionTime != 0) {
                    positionError.x = positionError.x + processNoise;
                    positionError.y = positionError.y + processNoise;
                    positionError.z = positionError.z + processNoise;

                    Position kalmanGain = positionError;
                    kalmanGain.x = positionError.x / (positionError.x + sensorNoise);
                    kalmanGain.y = positionError.y / (positionError.y + sensorNoise);
                    kalmanGain.z = positionError.z / (positionError.z + sensorNoise);

                    Position gainPosition = meanIntegrate(velocity, velocityPrev);
                    gainPosition.x = gainPosition.x * kalmanGain.x;
                    gainPosition.y = gainPosition.y * kalmanGain.y;
                    gainPosition.z = gainPosition.z * kalmanGain.z;

                    positionError.x = (1.0f - kalmanGain.x) * positionError.x;
                    positionError.y = (1.0f - kalmanGain.y) * positionError.y;
                    positionError.z = (1.0f - kalmanGain.z) * positionError.z;

                    position = plus(position, gainPosition);
                }

                if (parameters.loggingEnabled) {
                    RobotLog.vv(parameters.loggingTag, "dt=%.3fs accel=%s vel=%s pos=%s", (acceleration.acquisitionTime - accelPrev.acquisitionTime) * 1e-9, acceleration, velocity, position);
                }
            } else
                acceleration = getAbsoluteAcceleration(linearAcceleration);
        }
    }

    //FIXME: Take this out later
    /**
     * Returns an acceeration in respect to field coordinates
     * @param axis the chosen axis
     * @return the converted vector value
     */
    public double getAbsoluteAcceleration(IMU.Axis axis){
        final double full_rotation = 2.0 * Math.PI;
        double intrinsicAccelX = imu.getAcceleration().xAccel;
        double intrinsicAccelY = imu.getAcceleration().yAccel;
        double robotRotation = (double)imu.getAngularOrientation().toAxesOrder(XYZ).thirdAngle;
        double intrinsicVectorAngle = Math.atan2(intrinsicAccelY, intrinsicAccelX);
        if (intrinsicVectorAngle<0){
            intrinsicVectorAngle += full_rotation;
        }
        double absoluteRotation = intrinsicVectorAngle + robotRotation;
        if (absoluteRotation > full_rotation){
            absoluteRotation = absoluteRotation % full_rotation;
        }
        double hypothenusLength = Math.sqrt((intrinsicAccelX*intrinsicAccelX)+
                (intrinsicAccelY*intrinsicAccelY));
        switch (axis){
            case X:
                return hypothenusLength * Math.cos(absoluteRotation);
            case Y:
                return hypothenusLength * Math.sin(absoluteRotation);
            default:
                return 0.0;
        }
    }

    /**
     * Absolute converter for the acceleration object
     * @param acceleration with respect to the sensor
     * @return the absolute acceleration
     */
    public Acceleration getAbsoluteAcceleration(Acceleration acceleration){
        acceleration.xAccel = getAbsoluteAcceleration(X);
        acceleration.yAccel = getAbsoluteAcceleration(Y);
        acceleration.zAccel = getAbsoluteAcceleration(Z);
        return acceleration;
    }
}
