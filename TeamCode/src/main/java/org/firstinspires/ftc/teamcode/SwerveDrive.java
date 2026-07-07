package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SwerveDrive {
    /**
     * @param Vy Desired velocity m/s in y direction
     * @param Vx Desired velocity m/s in x direction
     * @param omega Desired angular velocity rad/s
     *
     * @retrun double[double[]] [[wheel1 speed, wheel1 angle, [wheel2 speed, wheel2 angle]]
     *
     * calculates the speed and angle of two swerve pods based off of positions and measurments in swerve constants
     * uses rigid body velocity formula
     * and cartesian to polar conversions
     */
    public double[][] swerveCalculations(double Vy, double Vx, double omega){
        double[][] returnArray = new double[2][2];
        double[] forwardPod = PodCalculation(Vy, Vx, omega, Constants.DriveTrain.ForwardPod.X, Constants.DriveTrain.ForwardPod.Y);
        double[] backPod = PodCalculation(Vy, Vx, omega, Constants.DriveTrain.BackPod.X, Constants.DriveTrain.BackPod.Y);

        // Normalize speeds if any speed is greater than MAX_MOTOR_SPEED
        double maxSpeed = Math.max(Math.abs(forwardPod[0]), Math.abs(backPod[0]));
        if (maxSpeed > Constants.DriveTrain.MAX_SPEED) {
            forwardPod[0] = forwardPod[0] / maxSpeed * Constants.DriveTrain.MAX_SPEED;
            backPod[0] = backPod[0] / maxSpeed * Constants.DriveTrain.MAX_SPEED;
        }

        returnArray[0] = forwardPod;
        returnArray[1] = backPod;
        return returnArray;
    }

    /**
     * @param Vy Desired velocity m/s in y direction
     * @param Vx Desired velocity m/s in x direction
     * @param Omega Desired angular velocity rad/s
     * @param X X position of pod relative to center of robot
     * @param Y Y position of pod relative to center of robot
     *
     * @return double[] [wheel speed, wheel angle]
     *
     * calculates the speed and angle of a swerve pod based off of positions and measurments in swerve constants
     * uses rigid body velocity formula
     * and cartesian to polar conversions
     */
    private double[] PodCalculation(double Vy, double Vx, double Omega, double X, double Y){
        double vxi = Vx - Omega * Y;
        double vyi = Vy + Omega * X;
        double speed = Math.sqrt(vxi*vxi + vyi*vyi); //speed in m/s
        double angle = Math.atan2(vyi, vxi); // returns angle in radians
        return new double[]{speed, angle};
    }

    //Motors
    private DcMotorEx frontDriveMotor;
    private DcMotorEx frontTurnMotor;
    private DcMotorEx backDriveMotor;
    private DcMotorEx backTurnMotor;

    public SwerveDrive(){
        // Initialize motors
        frontDriveMotor = hardwareMap.get(DcMotorEx.class, "frontDriveMotor");
        frontTurnMotor = hardwareMap.get(DcMotorEx.class, "frontTurnMotor");
        backDriveMotor = hardwareMap.get(DcMotorEx.class, "backDriveMotor");
        backTurnMotor = hardwareMap.get(DcMotorEx.class, "backTurnMotor");

        //set motor directions
        frontDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        frontTurnMotor.setDirection(DcMotor.Direction.FORWARD);
        backDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        backTurnMotor.setDirection(DcMotor.Direction.FORWARD);

        //set to run using encoders
        frontDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontTurnMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backTurnMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //setup PID controllers for velocity control
        //might have to wrap turn motors into separate class to combine with throughbore encoders for position control
        frontDriveMotor.setVelocityPIDFCoefficients(
                Constants.DriveTrain.ForwardPod.driveMotor.p,
                Constants.DriveTrain.ForwardPod.driveMotor.i,
                Constants.DriveTrain.ForwardPod.driveMotor.d,
                Constants.DriveTrain.ForwardPod.driveMotor.f
        );
        frontTurnMotor.setPositionPIDFCoefficients(Constants.DriveTrain.ForwardPod.turnMotor.p);

        backDriveMotor.setVelocityPIDFCoefficients(
                Constants.DriveTrain.BackPod.driveMotor.p,
                Constants.DriveTrain.BackPod.driveMotor.i,
                Constants.DriveTrain.BackPod.driveMotor.d,
                Constants.DriveTrain.BackPod.driveMotor.f
        );
        backTurnMotor.setPositionPIDFCoefficients(Constants.DriveTrain.BackPod.turnMotor.p);

    }

    public void periodicUpdate(double y, double x, double omega){
        //convert joystick inputs to m/s and rad/s
        double Vy = y * Constants.DriveTrain.MAX_SPEED; // m/s
        double Vx = x * Constants.DriveTrain.MAX_SPEED; // m/s
        double Omega = omega * Constants.DriveTrain.MAX_ANGULAR_SPEED; // rad/s
        double magnitude = Math.hypot(Vx, Vy);
        if (magnitude > 1.0) {
            Vx /= magnitude;
            Vy /= magnitude;
        }
        double[][] podSpeedsAndAngles = swerveCalculations(Vy, Vx, Omega);
        // Set motor powers and angles using velocity controllers with PID controllers
        frontDriveMotor.setVelocity(podSpeedsAndAngles[0][0] * Constants.DriveTrain.ForwardPod.driveMotor.TICKS_PER_METER); //pass in ticks/s
        frontTurnMotor.setTargetPosition((int)(podSpeedsAndAngles[0][1] * Constants.DriveTrain.ForwardPod.turnMotor.TICKS_PER_REV / (2 * Math.PI))); //pass in ticks
        backDriveMotor.setVelocity(podSpeedsAndAngles[1][0] * Constants.DriveTrain.BackPod.driveMotor.TICKS_PER_METER); //pass in ticks/s
        backTurnMotor.setTargetPosition((int)(podSpeedsAndAngles[1][1] * Constants.DriveTrain.BackPod.turnMotor.TICKS_PER_REV / (2 * Math.PI))); //pass in ticks

    }
}
