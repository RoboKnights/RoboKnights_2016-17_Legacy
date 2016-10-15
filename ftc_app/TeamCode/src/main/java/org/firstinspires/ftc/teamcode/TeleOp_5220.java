/* Copyright (c) 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.*;

import org.firstinspires.ftc.teamcode.OpMode_5220;

@TeleOp(name = "TeleOp 5220", group = "Main") //change to autonomous if making this called a teleop changes something for the worse
//@Disabled
public class TeleOp_5220 extends OpMode_5220 //this is a comment. It is a long comment.
{
    private static final double JOYSTICK_THRESHOLD = 0.08; //below this joysticks won't cause movement.
    private static final double SLOW_POWER = 0.10;


    private static final double SWIVEL_INCREMENT = 0.00658; //changed from 0.005

    private static final double SWIVEL_INCREMENT_TIME = 60; //in millis, every incrmeent time, it goes 0.01 counts. about 24 increments to go 180 then.
    private static final double SWIVEL_INERTIA_CORRECTION_MULTIPLIER = 0.5;

    private static final double DUMPER_INCREMENT = 0.025; //changed from 0.005
    private static final double DUMPER_INCREMENT_TIME = 60; //in millis, every incrmeent time, it goes 0.01 counts. about 24 increments to go 180 then.


    private static final double ARM_INCREMENT = 0.04;
    private static final double ARM_INCREMENT_TIME = 30; //in millis, every incrmeent time, it goes 0.01 counts. about 24 increments to go 180 then.


    private static final double HOOK_TILT_INCREMENT = 0.072;

    private static final double POLAR_CONTROL_R_THRESHOLD = 0.8; //max R value should be about 1

    private double g1Stick1Xinit;
    private double g1Stick1Yinit;

    private boolean color;

    private boolean reverseDriveOn = false;
    private boolean slowDriveOn = false;
    private boolean polarOn = false;
    private boolean resetAutomationOn = false;
    private boolean scoringAutomationOn = false;

    public ProgramType getProgramType ()
    {
        return ProgramType.TELEOP;
    }

    //INITIALZATION:

    public void initialize ()
    {
        super.initialize();
        g1Stick1Xinit = gamepad1.left_stick_x;
        g1Stick1Yinit = gamepad1.left_stick_y;
        //color = ftcRCA.color;
    }

    //MAIN PROGRAM:

    public void loop5220()
    {
        //STATE VARIABLES FOR LOOP:
        Stopwatch topHatXTime = null;
        Stopwatch topHatYTime = null;
        Stopwatch dumperTime = null;
        Stopwatch hookTiltTime = null;

        double swivelMovementStart = 0.0;

        boolean strafing = false;
        boolean reverse = false;

        Gamepad prevGamepad1 = new Gamepad(); //IF USING THESE GAMEPAD OBJECTS WORKS, REPLACE ALL THE INDIVIDUAL BOOLEANS BELOW WITH THEIR PREVGAMEPAD OBJECT COUNTERPARTS.
        Gamepad prevGamepad2 = new Gamepad();


        boolean prevTopHatUp1 = false; //maybe change these initialization if they mess something up
        boolean prevTopHatDown1 = false;
        boolean prevTopHatLeft1 = false;
        boolean prevTopHatRight1 = false;
        boolean prevTopHatUp2 = false; //maybe change these initialization if they mess something up
        boolean prevTopHatDown2 = false;
        boolean prevTopHatLeft2 = false;
        boolean prevTopHatRight2 = false;
        boolean prevX1 = false;
        boolean prevRSB1 = false;
        boolean prevB1 = false;
        boolean prevLB = false;
        boolean prevLT = false;
        boolean prevBack = false;
        boolean prevY2 = false;
        boolean prevX2 = false;
        boolean prevB2 = false;
        boolean prevA2 = false;
        boolean prevLB2 = false;
        boolean prevLT2 = false;
        boolean prevRB2 = false;
        boolean prevRT2 = false;

        while (runConditions())
        {
            //DRIVETRAIN CONTROL:
/*
            double throttle = (-(gamepad1.left_stick_y - g1Stick1Yinit));
            double direction = (gamepad1.left_stick_x - g1Stick1Xinit);

            if (reverseDriveOn)
            {
                throttle = -throttle;
            }

            double right = throttle - direction;
            double left = throttle + direction;

            right = Range.clip(right, -2, 2);
            left = Range.clip(left, -2, 2);

            if (false) //Slow control
            {
                if (right > SLOW_POWER)
                {
                    right = SLOW_POWER;
                }

                if (right < -SLOW_POWER)
                {
                    right = -SLOW_POWER;
                }

                if (left > SLOW_POWER)
                {
                    left = SLOW_POWER;
                }

                if (left < -SLOW_POWER)
                {
                    left = -SLOW_POWER;
                }
            }

            else
            {

                if (rightPower > 1)
                {
                    rightPower = 1;
                }

                if (rightPower < -1)
                {
                    rightPower = -1;
                }

                if (leftPower > 1)
                {
                    leftPower = 1;
                }

                if (leftPower < -1)
                {
                    leftPower = -1;
                }
            }

            if (Math.abs(right) < JOYSTICK_THRESHOLD)
            {
                right = 0;
            }

            if (Math.abs(left) < JOYSTICK_THRESHOLD)
            {
                left = 0;
            }

            if (left == 0 && right == 0)
            {

            }

            setLeftDrivePower(left);
            setRightDrivePower(right);

            if (gamepad1.start)
            {
                g1Stick1Xinit = gamepad1.left_stick_x;
                g1Stick1Yinit = gamepad1.left_stick_y;
            }

            //we don't need reverse drive for now

            if (gamepad2.b != prevB2 && gamepad2.b) //acts on button press
            {
                reverseDriveOn = !reverseDriveOn;
            }
*/



                double leftPower;
                double rightPower;
                // Driving wheels using y1,x1 joystick
/*
                if (!reverse)
                {
                    leftPower = gamepad1.left_stick_y * 0.8 + gamepad1.left_stick_x * 0.8;
                    rightPower = gamepad1.left_stick_y * 0.8 - gamepad1.left_stick_x * 0.8;
                }

                else
                {
                    leftPower = gamepad1.left_stick_y * 0.8 - gamepad1.left_stick_x * 0.8;
                    rightPower = gamepad1.left_stick_y * 0.8 + gamepad1.left_stick_x * 0.8;
                }
*/
                double throttle = (-(gamepad1.left_stick_y - g1Stick1Yinit));
                double direction = (gamepad1.left_stick_x - g1Stick1Xinit);

                if (reverse)
                {
                    throttle = -throttle;
                }

                rightPower = throttle - direction;
                leftPower = throttle + direction;

                rightPower = Range.clip(rightPower, -2, 2);
                leftPower = Range.clip(leftPower, -2, 2);

/* NOT USING DPAD FOR NOW
                if ((leftPower < 15 && leftPower> -15) && (rightPower < 15 && rightPower > -15))
                {
                    int powerVal = 15;
                    int turnPowerVal = 15;
                    int topHatVal = joystick.joy1_TopHat;
                    if (topHatVal == 0)
                    {
                        // go forward
                        leftPower = powerVal;
                        rightPower = powerVal;
                    }
                    else if (topHatVal == 4)
                    {
                        // go backward
                        leftPower = -powerVal;
                        rightPower = -powerVal;
                    }
                }
                */
                boolean powersZero = true;
                if (Math.abs (leftPower) < 0.05)
                {
                    leftPower = 0;
                }
                else powersZero = false;

                if (Math.abs (rightPower) < 0.05)
                {
                    rightPower = 0;
                }
                else powersZero = false;

                if (powersZero)
                {
                    double frontPower;
                    double backPower;
/*
                    frontPower = joystick.joy1_y2 * 0.8 + joystick.joy1_x2 * 0.8; //reverse + and - signs if direction is wrong.
                    backPower = joystick.joy1_y2 * 0.8 - joystick.joy1_x2 * 0.8;

                    if (reverse)
                    {
                        frontPower = -frontPower;
                        backPower = -backPower;
                    }
*/

                    throttle = (-(gamepad1.right_stick_y - g1Stick1Yinit));
                    direction = (gamepad1.right_stick_x - g1Stick1Xinit);

                    if (reverse)
                    {
                        throttle = -throttle;
                    }

                    backPower = throttle - direction; //SWITCH THESE AROUND IF THIS ENDS UP BEING THE WRONG WAY
                    frontPower = throttle + direction;

                    backPower = Range.clip(backPower, -2, 2);
                    frontPower = Range.clip(frontPower, -2, 2);

                    if (Math.abs (frontPower) < 0.05)
                    {
                        frontPower = 0;
                    }

                    if (Math.abs (backPower) < 0.05)
                    {
                        backPower = 0;
                    }
/*
                    if (frontPower == 0 && backPower == 0)
                    {
                        int strafePowerVal = 15;
                        int topHatVal = joystick.joy1_TopHat;

                        if (topHatVal == 6)
                        {
                            // go forward
                            frontPower = strafePowerVal;
                            backPower = -strafePowerVal;
                        }
                        else if (topHatVal == 2)
                        {
                            // go backward
                            frontPower = -strafePowerVal;
                            backPower = strafePowerVal;
                        }
                    }
*/
                    setMotorPower(leftFrontMotor, frontPower);
                    setMotorPower(rightFrontMotor, backPower);
                    setMotorPower(leftBackMotor, backPower);
                    setMotorPower(rightBackMotor, frontPower);
                    strafing = true;
                }

                else
                {
                    strafing = false;
                }

                if (!strafing)
                {
                    if (reverse)
                    {
                        leftPower = -leftPower;
                        rightPower = -rightPower;
                    }

                    setLeftDrivePower(leftPower);
                    setRightDrivePower(rightPower);
                }

                if (gamepad1.back)
                {
                    reverse = !reverse;
                    sleep (20);
                }

            if (gamepad1.a) setMotorPower(shooterMotor, -1.0);
            else setMotorPower(shooterMotor, 0.0);

            //PREVIOUS VALUE SETTINGS

            prevTopHatUp1 = gamepad1.dpad_up;
            prevTopHatDown1 = gamepad1.dpad_down;
            prevTopHatRight1 = gamepad1.dpad_right;
            prevTopHatLeft1 = gamepad1.dpad_left;
            prevTopHatUp2 = gamepad2.dpad_up;
            prevTopHatDown2 = gamepad2.dpad_down;
            prevTopHatRight2 = gamepad2.dpad_right;
            prevTopHatLeft2 = gamepad2.dpad_left;

            prevRSB1 = gamepad1.right_stick_button;
            prevX1 = gamepad1.x;
            prevB1 = gamepad1.b;
            prevLB = gamepad1.left_bumper;
            prevLT = gamepad1.left_trigger > 0.7;
            prevBack = gamepad1.back;

            prevY2 = gamepad2.y;
            prevX2 = gamepad2.x;
            prevB2 = gamepad2.b;
            prevA2 = gamepad2.a;
            prevLB2 = gamepad2.left_bumper;
            prevLT2 = gamepad2.left_trigger > 0.7;
            prevRB2 = gamepad2.right_bumper;
            prevRT2 = gamepad2.right_trigger > 0.7;

            try
            {
                prevGamepad1.copy(gamepad1);
                prevGamepad2.copy(gamepad2);
            }

            catch (RobotCoreException rce)
            {
                writeToLog("prevGamepad Copying Error.");
            }

           // telemetry.addData("9", "RSA: " + resetAutomationOn);
            waitNextCycle();


        }
    }

    public void main ()
    {

        new DebuggerDisplayLoop().start();
        //for (DcMotor dcm: driveMotors) dcm.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        /*
        waitFullCycle();
        colorSensorFront.enableLed(true);
        waitFullCycle();
        colorSensorDown.enableLed(true);
        waitFullCycle();
*.*/
        while (runConditions())
        {
            try
            {
                loop5220();
            }
            catch (Exception e)
            {
                DbgLog.error(e.getMessage());
            }
        }

        //loop5220();

    }
}