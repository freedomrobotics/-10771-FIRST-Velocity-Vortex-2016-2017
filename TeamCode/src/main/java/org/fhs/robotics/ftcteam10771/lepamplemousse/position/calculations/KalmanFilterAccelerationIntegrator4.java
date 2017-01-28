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

import android.opengl.Matrix;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.Vector;

import org.fhs.robotics.ftcteam10771.lepamplemousse.actions.maneuvers.UltrasonicRange;
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
    float[] transformationPrediction;
    float[] matrixIdentity;

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
        this.transformationPrediction = new float[16];
        this.matrixIdentity = new float[16];
        // Generate a 4x4 identity matrix
        Matrix.setIdentityM(this.matrixIdentity, 0);
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

        // Make the transformation matrix for the prediction
        // based off of the previous state. It's the identity matrix + some
        Matrix.setIdentityM(transformationPrediction, 0);
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

    public void maths(){
        //Z = Current Uncertainty
        //Z1 = Previous Uncertainty

        //S = Current State
        //P = Previous State

        //Q = Model Noise
        //R = Sensor Noise

        //C = Control

        //X = Predicted State
        //M = Measured State

        //E = Measured Uncertainty

        //K = Kalman Gain

        //I = Identity Matrix
        //F = Temporary Matrix

        //A = Prediction Transformation of Previous State
        //B = Transformation dependent on form/structure of C
        //D = Acceleration Transformation Matrix

        //a = Acceleration vector

        /*
        Structure of Vector: [X Y vX vY]
         */
        /*
        Model:
        X = AP + BC
        Z = A Z1 A^t + Q

        Measurements:
        M = AP + Da
        E = A Z1 A^t + R

        Filter:
        K = Z(Z+E)^-1
        S = X+K(M-X)

        Uncertainty:
        F = I-K
        Z = FZF^t + KRK^t
         */

        //model
        // X = AP + BC
        float time = 0;
        float[] predictedState = new float[4];
        float[] controlState = new float[4];
        // transformation prediction
        // 1 0 t 0
        // 0 1 0 t
        // 0 0 1 0
        // 0 0 0 1
        transformationPrediction[2] = transformationPrediction[7] = time;
        Matrix.multiplyMV(predictedState, 0, transformationPrediction, 0, currentState, 0);
        addV(currentState, predictedState, controlState);

        //Z = A Z1 A^t + Q
        float[] transformationPredictionTranspose = new float[16];
        Matrix.transposeM(transformationPredictionTranspose, 0, transformationPrediction, 0);
        Matrix.multiplyMM(currentUncertainty, 0, transformationPrediction, 0, currentUncertainty, 0);
        Matrix.multiplyMM(currentUncertainty, 0, currentUncertainty, 0, transformationPredictionTranspose, 0);
        // refer to measurement error. Here, currentUncertainty represents the shard beginning of both now
        float[] measuredUncertainty = new float[16];
        add(measuredUncertainty, currentUncertainty, sensorCovariance);
        //back to the model.
        add(currentUncertainty, currentUncertainty, modelCovariance);


        //measurements
        // M = AP + Ba
        float[] transformedAcceleration = new float[4];
        float[] measuredState = new float[4];
        transformedAcceleration[0] = (float)(.5 * time * time * acceleration.xAccel);
        transformedAcceleration[1] = (float)(.5 * time * time * acceleration.yAccel);
        transformedAcceleration[2] = (float)(time * acceleration.xAccel);
        transformedAcceleration[3] = (float)(time * acceleration.yAccel);
        // taking shared start of equation from model
        addV(measuredState, predictedState, transformedAcceleration);

        // E = A Z1 A^t + R
        // refer to model error (merged in for optimization)


        //filter
        //K = Z(Z + E)^-1
        float[] kalmanGain = new float[16];
        add(kalmanGain, currentUncertainty, measuredUncertainty);
        Matrix.invertM(kalmanGain, 0, kalmanGain, 0);
        Matrix.multiplyMM(kalmanGain, 0, currentUncertainty, 0, kalmanGain, 0);

        //S = X + K(M-X)
        //because current state is actually the predicted and predicted state is just the shard start
        sub(measuredState, measuredState, currentState);
        Matrix.multiplyMM(measuredState, 0, kalmanGain, 0, measuredState, 0);
        add(currentState, currentState, measuredState);


        //Uncertainty
        //F = I - K
        float[] temp = new float[16];
        sub(temp, matrixIdentity, kalmanGain);

        //FZF^t + KRK^t
        float[] transposes = new float[16];
        Matrix.transposeM(transposes, 0, temp, 0);
        Matrix.multiplyMM(temp, 0, temp, 0, currentUncertainty, 0);
        Matrix.multiplyMM(temp, 0, temp, 0, transposes, 0);
        Matrix.transposeM(transposes, 0, kalmanGain, 0);
        Matrix.multiplyMM(kalmanGain, 0, kalmanGain, 0, sensorCovariance, 0);
        Matrix.multiplyMM(kalmanGain, 0, kalmanGain, 0, transposes, 0);
        add(currentUncertainty, temp, kalmanGain);
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
    //v1+v2 = out
    private void addV(float[] out, float[] v1, float[] v2){
        for (int i = 0; i < 4; i++){
            out[i] = v1[i] + v2[i];
        }
    }
    //v1-v2 = out
    private void subV(float[] out, float[] v1, float[] v2){
        for (int i = 0; i < 4; i++){
            out[i] = v1[i] - v2[i];
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
