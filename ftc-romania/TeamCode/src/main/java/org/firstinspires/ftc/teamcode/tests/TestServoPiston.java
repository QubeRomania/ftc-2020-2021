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
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
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

package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardware.servo_block;
import org.firstinspires.ftc.teamcode.hardware.servo_piston;

@TeleOp
public class TestServoPiston extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    public servo_piston servoPiston = new servo_piston();
    public servo_block servoBlock = new servo_block();

    DcMotorEx outtakeMotor = null;
    double outtakePower = 0;
    double basePowerOuttake = 1550;
    double powerShotPower = 1440;
    double powerUnit = 25;
    Boolean cheieOuttakeUp = Boolean.FALSE;
    Boolean cheieOuttakeDown = Boolean.FALSE;
    Boolean cheieOutake = Boolean.FALSE;
    Boolean okOuttake = Boolean.FALSE;
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(52, 0, 4.6, 14);

    Boolean cheiePiston = Boolean.FALSE;
    int pistonRelease = -165;
    int pistonClose = 0;
    double pistonPower = 0.4;

    public double close = 1;
    public double open = 0;

    @Override
    public void runOpMode() {
        servoPiston.initPiston(hardwareMap);
        servoBlock.initBlock(hardwareMap);

        outtakeMotor = (DcMotorEx)hardwareMap.get(DcMotor.class, "outtakeMotor");
        outtakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
        outtakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        servoBlock.open();

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            outtakeController();
            pistonController();
            outtakeMotor.setVelocity(outtakePower);
        }
    }

    void outtakeController()
    {
        //toggle b to turn outtake motor on/off
        if (gamepad2.b && !cheieOutake) {
            cheieOutake = !cheieOutake;
            okOuttake = !okOuttake;
        }
        if (!gamepad2.b) {
            cheieOutake = false;
        }
        if (okOuttake)
            outtakePower = basePowerOuttake;
        else
            outtakePower = 0;

        //dpad gives more or less power to outtake motor
        if(gamepad2.dpad_up) {
            if (!cheieOuttakeUp) {
                basePowerOuttake += powerUnit;
                cheieOuttakeUp = true;
            }
        }
        else
            cheieOuttakeUp = false;
        if(gamepad2.dpad_down) {
            if (!cheieOuttakeDown) {
                basePowerOuttake -= powerUnit;
                cheieOuttakeDown = true;
            }
        }
        else
            cheieOuttakeDown = false;
    }

    void pistonController()
    {
        if(gamepad2.left_bumper && !cheiePiston)
        {
            servoPiston.open();
            sleep(100);
            servoPiston.close();
            cheiePiston = true;
        }
        if(!gamepad2.left_bumper)
            cheiePiston = false;

    }
}
