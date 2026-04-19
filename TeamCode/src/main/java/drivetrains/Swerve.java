package drivetrains;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import hardware.SwerveModule;
import drivetrains.constants.SwerveConstants;
import hardware.MotorEx;
import util.XaddyUnit;

/**
 * @author Xander Haemel
 */
public class Swerve extends Drivetrain{
    SwerveConstants constants;

    // Motors
    MotorEx flMotor;
    MotorEx blMotor;
    MotorEx frMotor;
    MotorEx brMotor;


    SwerveModule fl;
    SwerveModule rl;
    SwerveModule fr;
    SwerveModule rr;



    /**
     * default constructor
     * @param hardwareMap is the hardwaremap
     * @param constants: swerveconstants, containing the configuration for your drivetrain
     */
    public Swerve(HardwareMap hardwareMap, @NonNull SwerveConstants constants){
        this.constants = constants;
        //motors
        flMotor = new MotorEx(hardwareMap, constants.flData);
        frMotor = new MotorEx(hardwareMap, constants.frData);
        blMotor = new MotorEx(hardwareMap, constants.blData);
        brMotor = new MotorEx(hardwareMap, constants.brData);
        //new swerve modules
        fl = new SwerveModule(hardwareMap, flMotor);
        rl = new SwerveModule(hardwareMap, blMotor);
        fr = new SwerveModule(hardwareMap, frMotor);
        rr = new SwerveModule(hardwareMap, brMotor);
    }

    /**
     * set power based on inputs
     * @param lfPower is left front power from 0.0 - 0.1
     * @param lrPower is left rear power from 0.0 - 0.1
     * @param rfPower is right front power from 0.0 - 0.1
     * @param rrPower is right rear power from 0.0 - 0.1
     */
    private void setPower(double lfPower, double lrPower, double rfPower, double rrPower){

    }

    @Override
    protected void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {

    }


    @Override
    public void moveWithVectors(double drive, double strafe, double turn){
        //make turn angle clockwise
        turn /= -1;
        //used for the sideways motion of both rear wheels
        double strafeRear = strafe - turn * (constants.wheelbase/constants.diagonalDistance);
        //used for the sideways motion of both front wheels
        double strafeFront  = strafe + turn * (constants.wheelbase/constants.diagonalDistance);
        //used for the forward motion of the right wheels
        double forwardRight = drive  - turn * (constants.trackwidth/constants.diagonalDistance);
        //used for the forward motion of the left wheels
        double forwardLeft  = drive  + turn * (constants.trackwidth/constants.diagonalDistance);

        //now we calculate wheel speeds based on these variables
        double frontRightSpeed = Math.sqrt(Math.pow(strafeFront,2) + Math.pow(forwardRight, 2));
        double frontLeftSpeed = Math.sqrt(Math.pow(strafeFront,2) + Math.pow(forwardLeft, 2));
        double rearLeftSpeed = Math.sqrt(Math.pow(strafeRear,2) + Math.pow(forwardLeft, 2));
        double rearRightSpeed = Math.sqrt(Math.pow(strafeRear,2) + Math.pow(forwardRight, 2));

        //optimize and calculate wheel angles rather than turning 180 degrees

        XaddyUnit frontRight = optimizeWheelAngle(fr.getPodHeading(), Math.atan2(strafeFront, forwardRight)*180/Math.PI, frontRightSpeed);
        XaddyUnit frontLeft  = optimizeWheelAngle(fl.getPodHeading(), Math.atan2(strafeFront, forwardLeft)*180/Math.PI,  frontLeftSpeed);
        XaddyUnit rearLeft   = optimizeWheelAngle(rl.getPodHeading(), Math.atan2(strafeRear, forwardLeft)*180/Math.PI, rearLeftSpeed);
        XaddyUnit rearRight  = optimizeWheelAngle(rr.getPodHeading(), Math.atan2(strafeRear,forwardRight)*180/Math.PI, rearRightSpeed);

        //scale powers to be =<1
        double max = frontRight.getMotorPower();
        if(frontLeft.getMotorPower()> max){
            max = frontLeft.getMotorPower();
        } if(rearLeft.getMotorPower() > max){
            max = rearLeft.getMotorPower();
        } if(rearRight.getMotorPower()> max){
            max = rearRight.getMotorPower();
        }
        //scale down if powers are greater than 1
       /* if(max > 1){
            frontRight.setMotorSpeed(forwardRight./= max;
            frontLeftSpeed  /= max;
            rearLeftSpeed   /= max;
            rearRightSpeed  /= max;
        }*/ //todo fix power scaling
    }

    /**
     * optimizes wheel angle by flipping the motor direction when possible
     * @param currentAngle is the angle of the current pod (degrees)
     * @param targetAngle is the target angle for the pod (degrees)
     * @param power the power of the swerve pod 0.0-1.0
     * @return the pod angle (degrees)
     */
    private XaddyUnit optimizeWheelAngle(double currentAngle, double targetAngle, double power){
        double delta = targetAngle- currentAngle;
        double wrappedDelta = delta - (360 * Math.round(delta /360.0));
        if(Math.abs(wrappedDelta) > 90){
            power *= -1;
            wrappedDelta -= Math.copySign(180, wrappedDelta);
        }
        return new XaddyUnit(power, currentAngle + wrappedDelta);
    }

    @Override
    public void drive(double x, double y, double turn, double robotHeading) {

    }

    @Override
    public void stop() {

    }

    @Override
    public void debug(Telemetry telemetry) {

    }
}
