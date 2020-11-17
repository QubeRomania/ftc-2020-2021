package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.RoadRunner.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.RoadRunner.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.RoadRunner.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.RoadRunner.drive.DriveConstants.getMotorVelocityF;

import android.support.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.mecanum.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.util.AxesSigns;
import org.firstinspires.ftc.teamcode.RoadRunner.util.LynxModuleUtil;
import org.firstinspires.ftc.teamcode.RoadRunner.util.BNO055IMUUtil;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

@Config
public class OdometryDrive extends SampleMecanumDrive {

    private ExpansionHubEx hub2, hub5;
    private ExpansionHubMotor leftFront, leftRear, rightRear, rightFront;
    private List<ExpansionHubMotor> motors;
    private BNO055IMU imu;

    public OdometryDrive(HardwareMap hardwareMap) {
        super();

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        // TODO: adjust the names of the following hardware devices to match your configuration
        // for simplicity, we assume that the desired IMU and drive motors are on the same hub
        // if your motors are split between hubs, **you will need to add another bulk read**
        hub2 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        hub5 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 5");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        // TODO: if your hub is mounted vertically, remap the IMU axes so that the z-axis points
        // upward (normal to the floor) using a command like the following:
        BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);

        leftFront = hardwareMap.get(ExpansionHubMotor.class, "leftFront");
        leftRear = hardwareMap.get(ExpansionHubMotor.class, "leftRear");
        rightRear = hardwareMap.get(ExpansionHubMotor.class, "rightRear");
        rightFront = hardwareMap.get(ExpansionHubMotor.class, "rightFront");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (ExpansionHubMotor motor : motors) {
            if (RUN_USING_ENCODER) {
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        // TODO: reverse any motors using DcMotor.setDirection()

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        // TODO: if desired, use setLocalizer() to change the localization method
        // for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));
        setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));
    }

    @Override
    public PIDCoefficients getPIDCoefficients(DcMotor.RunMode runMode) {
        PIDFCoefficients coefficients = leftFront.getPIDFCoefficients(runMode);
        return new PIDCoefficients(coefficients.p, coefficients.i, coefficients.d);
    }

    @Override
    public void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients) {
        for (ExpansionHubMotor motor : motors) {
            motor.setPIDFCoefficients(runMode, new PIDFCoefficients(
                    coefficients.kP, coefficients.kI, coefficients.kD, getMotorVelocityF()
            ));
        }
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        RevBulkData bulkData2 = hub2.getBulkInputData();
        RevBulkData bulkData5 = hub5.getBulkInputData();

        List<Double> wheelVelocities = new ArrayList<>();

        if (bulkData2 == null) {
            wheelVelocities.add(.0);
            wheelVelocities.add(.0);
        } else {
            wheelVelocities.add(encoderTicksToInches(bulkData2.getMotorCurrentPosition(leftFront)));
            wheelVelocities.add(encoderTicksToInches(bulkData2.getMotorCurrentPosition(leftRear)));
        }
        if (bulkData5 == null) {
            wheelVelocities.add(.0);
            wheelVelocities.add(.0);
        } else {
            wheelVelocities.add(encoderTicksToInches(bulkData5.getMotorCurrentPosition(rightFront)));
            wheelVelocities.add(encoderTicksToInches(bulkData5.getMotorCurrentPosition(rightRear)));
        }

        return wheelVelocities;
    }

    @Override
    public List<Double> getWheelVelocities() {
        RevBulkData bulkData2 = hub2.getBulkInputData();
        RevBulkData bulkData5 = hub5.getBulkInputData();

        List<Double> wheelVelocities = new ArrayList<>();

        if (bulkData2 == null) {
            wheelVelocities.add(.0);
            wheelVelocities.add(.0);
        } else {
            wheelVelocities.add(encoderTicksToInches(bulkData2.getMotorVelocity(leftFront)));
            wheelVelocities.add(encoderTicksToInches(bulkData2.getMotorVelocity(leftRear)));

        }if (bulkData5 == null) {
            wheelVelocities.add(.0);
            wheelVelocities.add(.0);
        } else {
            wheelVelocities.add(encoderTicksToInches(bulkData5.getMotorVelocity(rightFront)));
            wheelVelocities.add(encoderTicksToInches(bulkData5.getMotorVelocity(rightRear)));
        }

        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.748; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double FRONT_LATERAL_DISTANCE = 5.693; // in; distance between the left and right wheels
    public static double LATERAL_DISTANCE = 13.433;
    public static double FORWARD_LATERAL_OFFSET = 3.22;
    public static double SIDE_OFFSET = 2.843;
    public static double SIDE_LATERAL_OFFSET = -3.126;

    public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
        public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
            super(Arrays.asList(
                    new Pose2d(FRONT_LATERAL_DISTANCE, FORWARD_LATERAL_OFFSET, Math.toRadians(90)), // left
                    new Pose2d(FRONT_LATERAL_DISTANCE - LATERAL_DISTANCE, FORWARD_LATERAL_OFFSET, Math.toRadians(90)), // right
                    new Pose2d(SIDE_OFFSET, SIDE_LATERAL_OFFSET, Math.toRadians(180)) // front
            ));
        }

        public double encoderTicksToInches(int ticks) {
            return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
        }

        /// stanga - dreapta - spate
        @NonNull
        @Override
        public List<Double> getWheelPositions() {
            RevBulkData bulkData2 = hub2.getBulkInputData();
            RevBulkData bulkData5 = hub5.getBulkInputData();

            List<Double> wheelVelocities = new ArrayList<>();

            if (bulkData2 == null) {
                wheelVelocities.add(.0);
                wheelVelocities.add(.0);
            } else {
                wheelVelocities.add(encoderTicksToInches(bulkData2.getMotorCurrentPosition(leftFront)));
                wheelVelocities.add(encoderTicksToInches(bulkData2.getMotorCurrentPosition(leftRear)));
            }
            if (bulkData5 == null) {
                wheelVelocities.add(.0);
            } else {
                wheelVelocities.add(encoderTicksToInches(bulkData5.getMotorCurrentPosition(rightFront)));
            }

            return wheelVelocities;
        }
    }
}