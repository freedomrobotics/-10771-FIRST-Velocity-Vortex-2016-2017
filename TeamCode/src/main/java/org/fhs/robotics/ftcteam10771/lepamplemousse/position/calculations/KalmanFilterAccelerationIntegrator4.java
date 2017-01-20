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
public class KalmanFilterAccelerationIntegrator4 implements BNO055IMU.AccelerationIntegrator {
    //------------------------------------------------------------------------------------------
    // State
    //------------------------------------------------------------------------------------------

    BNO055IMU.Parameters parameters;
    Position position;
    Velocity velocity;
    Acceleration acceleration;

    Config.ParsedData kalmanConfig;

    Position positionError;
    float velocityError;0
    float accelerationError;

    float processNoise;
    float sensorNoise;
    private float lowPassAlpha;
    
    /**
     * todo: Move into own class later
     */
    class Matrix<E> {
        Vector<Vector<E>> matrix;
        int vectorSize;
        int vectorCount;
        
        Matrix(int vectorSize, int vectorCount){
            matrix = new Vector<Vector<E>>(vectorCount);
            for (int i = 0; i < vectorCount; i++){
                matrix.add(i, new Vector<E>(vectorSize));
            }
            this.vectorSize = vectorSize;
            this.vectorCount = vectorCount;
        }
        Matrix(Matrix matrix){
            this.matrix = matrix.get2dVector();
            vectorSize = matrix.getRowCount();
            vectorCount = matrix.getColumnCount();
        }
        
        E getEntry(int row, int column){
            return matrix.get(column).get(row);
        }
        
        void setEntry(E e, int row, int column){
            matrix.get(column).get(row) = e;
        
        Matrix getTranspose(){
            Vector<Vector<E>> transpose = new Vector<Vector<E>>(vectorSize);
            for (int i = 0; i < vectorSize; i++){
                transpose.add(i, new Vector<E>(vectorCount));
                for (int j = 0; j < vectorCount; j++){
                    transpose.get(i).add(j, getEntry(i, j));
                }
            }
        }
        
        void add(Matrix matrixToAdd){
            if (matrixToAdd.getRowSize() != getRowSize() && matrixToAdd.getColumnSize() != getColumnSize())
                throw new RuntimeException("Illegal matrix operation.");
            //E e = matrix.getEntry(i, j);
        }
        
        Matrix add(Matrix matrix1, Matrix matrix2){
            
        }
        
        Vector<E> getVector(int column){
            return matrix.get(position);
        }
        
        int getRowSize(){
            return vectorCount;
        }
        
        int getRowCount(){
            return vectorSize;
        }
        
        int getColumnSize(){
            return vectorSize;
        }
        
        int getColumnCount(){
            return vectorCount;
        }
        
        Vector<Vector<E>> get2dVector(){
            return matrix;
        }
    }

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
        lowPassAlpha = kalmanConfig.getFloat("low_pass_alpha");
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

                acceleration = plus(accelPrev, scale(minus(linearAcceleration, accelPrev), lowPassAlpha));

                // filter acceleration
                accelerationError += processNoise;
                float accelGain = accelerationError / (accelerationError + sensorNoise);
                accelerationError = (1.0f - accelGain) * accelerationError;
                acceleration = plus(accelPrev,scale(minus(acceleration, accelPrev), accelGain));

                if (accelPrev.acquisitionTime != 0) {
                    Velocity gainVelocity = meanIntegrate(acceleration, accelPrev);
                    velocity = plus(velocity, gainVelocity);
                }

                if (velocityPrev.acquisitionTime != 0) {
                    Position gainPosition = meanIntegrate(velocity, velocityPrev);
                    position = plus(position, gainPosition);
                }

                if (parameters.loggingEnabled) {
                    RobotLog.vv(parameters.loggingTag, "dt=%.3fs accel=%s vel=%s pos=%s", (acceleration.acquisitionTime - accelPrev.acquisitionTime) * 1e-9, acceleration, velocity, position);
                }
            } else
                accelerationError += processNoise;
                float accelGain = accelerationError / (accelerationError + sensorNoise);
                accelerationError = (1.0f - accelGain) * accelerationError;

                acceleration = scale(linearAcceleration, accelGain);
        }
    }
}
