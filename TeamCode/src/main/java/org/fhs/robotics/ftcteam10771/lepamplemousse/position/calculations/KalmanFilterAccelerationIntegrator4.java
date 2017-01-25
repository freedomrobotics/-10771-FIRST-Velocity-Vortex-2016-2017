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

import java.util.Vector;

import org.fhs.robotics.ftcteam10771.lepamplemousse.config.Config;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import static org.firstinspires.ftc.robotcore.external.navigation.NavUtil.meanIntegrate;
import static org.firstinspires.ftc.robotcore.external.navigation.NavUtil.minus;
import static org.firstinspires.ftc.robotcore.external.navigation.NavUtil.plus;
import static org.firstinspires.ftc.robotcore.external.navigation.NavUtil.scale;

/**
 * {@link KalmanFilterAccelerationIntegrator4} is based on the
 * {@link com.qualcomm.hardware.adafruit.NaiveAccelerationIntegrator}.
 * It uses the Kalman filter to filter noise.
 */
public class
KalmanFilterAccelerationIntegrator4 implements BNO055IMU.AccelerationIntegrator {
    //------------------------------------------------------------------------------------------
    // State
    //------------------------------------------------------------------------------------------

    BNO055IMU.Parameters parameters;
    Position position;
    Velocity velocity;
    Acceleration acceleration;

    Config.ParsedData kalmanConfig;

    //Persistent Matrices
    float[] currentState;
    float[] currentUncertainty; //also a covariance
    float[] modelCovariance;
    float[] sensorCovariance;

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

    public KalmanFilterAccelerationIntegrator4(Config.ParsedData kalmanConfig) {
        this.parameters = null;
        this.position = null;
        this.velocity = null;
        this.acceleration = null;
        this.kalmanConfig = kalmanConfig;

        this.currentState = new float[4];
        this.currentUncertainty = new float[16];
        this.modelCovariance = new float[16];
        this.sensorCovariance = new float[16];
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

        //Refer to my notes - Adam Li
        this.currentState[0] = (float) position.x;
        this.currentState[1] = (float) position.y;
        this.currentState[2] = (float) velocity.xVeloc;
        this.currentState[3] = (float) velocity.yVeloc;

        float[] modelNoise = new float[4];
        float[] sensorNoise = new float[4];
        float[] initialError = new float[4];

        modelNoise[0] = kalmanConfig.subData("model_noise").getFloat("x");
        modelNoise[1] = kalmanConfig.subData("model_noise").getFloat("y");
        modelNoise[2] = kalmanConfig.subData("model_noise").getFloat("x_vel");
        modelNoise[3] = kalmanConfig.subData("model_noise").getFloat("y_vel");

        //have to derive from the noise of the acceleration
        sensorNoise[0] = kalmanConfig.subData("sensor_noise").getFloat("x");
        sensorNoise[1] = kalmanConfig.subData("sensor_noise").getFloat("y");
        sensorNoise[2] = kalmanConfig.subData("sensor_noise").getFloat("x_vel");
        sensorNoise[3] = kalmanConfig.subData("sensor_noise").getFloat("y_vel");

        initialError[0] = kalmanConfig.subData("initial_error").getFloat("x");
        initialError[1] = kalmanConfig.subData("initial_error").getFloat("y");
        initialError[2] = kalmanConfig.subData("initial_error").getFloat("x_vel");
        initialError[3] = kalmanConfig.subData("initial_error").getFloat("y_vel");

        sToCov(modelCovariance, modelNoise);
        sToCov(sensorCovariance, sensorNoise);
        sToCov(currentUncertainty, initialError);
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
                if (parameters.loggingEnabled) {
                    RobotLog.vv(parameters.loggingTag, "dt=%.3fs accel=%s vel=%s pos=%s", (acceleration.acquisitionTime - accelPrev.acquisitionTime) * 1e-9, acceleration, velocity, position);
                }
            } else {
            }
        }
    }

//region matrix maths
    //m1+m2 = out
    private void add(float[] out, float[] m1, float[] m2){
        for (int i = 0; i < 16; i++){
            out[i] = m1[i] + m2[i];
        }
    }
    //m1-m2 = out
    private void sub(float[] out, float[] m1, float[] m2){
        for (int i = 0; i < 16; i++){
            out[i] = m1[i] - m2[i];
        }
    }
    //cov(s) = out
    private void sToCov(float[] out, float[] s){
        for (int i = 0; i < 16; i++){
            out[i] = s[i/4] * s[i%4];
        }
    }
//endregion
}
