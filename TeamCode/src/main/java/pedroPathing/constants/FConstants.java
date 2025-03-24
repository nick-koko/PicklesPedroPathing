package pedroPathing.constants;

import com.pedropathing.localization.Localizers;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FConstants {
    static {
        FollowerConstants.localizers = Localizers.PINPOINT;

        FollowerConstants.leftFrontMotorName = "front_left";
        FollowerConstants.leftRearMotorName = "back_left";
        FollowerConstants.rightFrontMotorName = "front_right";
        FollowerConstants.rightRearMotorName = "back_right";

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

        FollowerConstants.mass = 10.8571; //change to kgs!!!!

        FollowerConstants.xMovement = 69.4659;
        FollowerConstants.yMovement = 51.656;
        FollowerConstants.forwardZeroPowerAcceleration = -32.6722;
        FollowerConstants.lateralZeroPowerAcceleration = -66.4537;

        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.2,0,0.01,0);
        FollowerConstants.useSecondaryTranslationalPID = false;
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.075,0,0.05,0); // Not being used, @see useSecondaryTranslationalPID

        FollowerConstants.headingPIDFCoefficients.setCoefficients(3,0,0.1,0);
        FollowerConstants.useSecondaryHeadingPID = false;
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(1.5,0,0.1,0); // Not being used, @see useSecondaryHeadingPID

        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.030,0,0.00020,0.6,0);  //start with .01, increase to 0.015, 0.02 - Probably try to increase to .02-.03 to speed things up
        FollowerConstants.useSecondaryDrivePID = false;
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.02,0,0.0005,0.6,0); // Not being used, @see useSecondaryDrivePID

        FollowerConstants.zeroPowerAccelerationMultiplier = 7; //Higher value means quicker stopping. Try up to 7 - can set these per pathchain
        FollowerConstants.centripetalScaling = 0.0001;

        FollowerConstants.pathEndTimeoutConstraint = 100; //Might want to try 50
        FollowerConstants.pathEndTValueConstraint = 0.99; //Might want to try 97.5 or 95% if need it to move on earlier
        FollowerConstants.pathEndVelocityConstraint = 0.1; //Increase it to ??? 3 or 5 to help with overshooting
        FollowerConstants.pathEndTranslationalConstraint = 0.1;
        FollowerConstants.pathEndHeadingConstraint = 0.007;
    }
}
