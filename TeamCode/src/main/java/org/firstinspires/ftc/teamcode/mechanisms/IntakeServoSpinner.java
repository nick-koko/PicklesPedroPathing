/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 😎👌👌👌
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.mechanisms;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class IntakeServoSpinner {

    // Define class members
    CRServo intakeServo;
    NormalizedColorSensor intakeColorSensor;
    double  power   = 0.0;
    float gain = 10;
    final float[] hsvValues = new float[3];


    public enum INTAKE_SPINNER_STATES{
        SPINNER_INTAKING, SPINNER_OUTTAKING, SPINNER_STOP
    }

    private INTAKE_SPINNER_STATES curIntakeState = null;

    public void init(HardwareMap hwMap) {
        intakeServo = hwMap.get(CRServo.class, "intake_servo");   //TODONE 👌👌😎👌👌kirbyrules.
        intakeColorSensor = hwMap.get(NormalizedColorSensor.class, "intake_color_sensor");
        intakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        curIntakeState = INTAKE_SPINNER_STATES.SPINNER_STOP;
        intakeColorSensor.setGain(gain);
    }

    public void Intake() {
        power = 0.5 ;

        // Set the motor to the new power
        intakeServo.setPower(power);

        curIntakeState = INTAKE_SPINNER_STATES.SPINNER_INTAKING;
    }

    public void Outtake() {

        power = -0.4 ;

        // Set the motor to the new power
        intakeServo.setPower(power);

        curIntakeState = INTAKE_SPINNER_STATES.SPINNER_OUTTAKING;

    }

    public void Stop() {

        // Set power to zero
        power = 0.0 ;

        // Set the motor to the new power;
        intakeServo.setPower(power);

        curIntakeState = INTAKE_SPINNER_STATES.SPINNER_STOP;

    }
    public INTAKE_SPINNER_STATES getIntakeState() {
        return curIntakeState;
    }
    public int getIntakeColors() {  //Colors!!!
        // 0 = Blue 1 = Undefined 2 = Red
        int returnIntColor;
        float intakeHue;

        NormalizedRGBA colors = intakeColorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);

        intakeHue = hsvValues[0];

        if (intakeHue < 45.0) {
            returnIntColor = 2;
        } else if (intakeHue > 200) {
            returnIntColor = 0;
        } else {
            returnIntColor = 1;
        }

        return returnIntColor;
    }

    public boolean sampleGrabbed() {  //Distance!!!

        double intakeDistance;

        intakeDistance = ((DistanceSensor) intakeColorSensor).getDistance(DistanceUnit.CM);
        if (intakeDistance < 2.5) {
            return true;
        } else {
            return false;
        }
    }

}
