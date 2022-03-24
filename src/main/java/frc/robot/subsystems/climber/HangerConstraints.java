package frc.robot.subsystems.climber;

import static frc.robot.Constants.*;
import static java.lang.Math.*;

public class HangerConstraints {
    // kBaseClimberLength  -  the resting length of the climber at Home position
    // kClimberLengthPerRotation - convert encoder position to extension length
    // kPivotLength - from the pivot point to the center of the bar
    // kBarDistance - from the center of one bar to the other
    // kPivotSupportLength = the polyCarb support
    // kPivotMountLength = along the pivot from the pivot point to the polyCarb attachment point
    // kLeadOffset = the distance between home and the point where the pivot is exactly vertical

    public static double getPivotPosition(double climberPosition) {
        double currentClimberLength = kClimberBaseLength + kClimberLengthPerRotation * climberPosition;

        // Use the law of cosines to calculate the idealPivotAngle
        // a is the length of the pivot arm.
        // b is the distance between the bars
        // c is the length of the climber
        // cos B =  [a^2 + c^2 - b^2]/2ac
        double angleB = acos( (currentClimberLength * currentClimberLength + kPivotLength * kPivotLength - kBarDistance * kBarDistance) /
                (2.0 * currentClimberLength * kPivotLength));

        // Find the angles for the triangle between the pivot arm, the support arm, and the lead screw.
        // a is the length of the support piece
        // b is the mount point on the pivot (center to center)
        // c is length along the lead screw
        double alpha = PI / 2.0 - angleB;

        // back to the law of cosines.
        // a^2 = b^2 + c^2 - 2bc cos(alpha)
        // We want to calculate c. Use the quadratic formula
        // 0 = b^2 - a^2 + c^2 - 2bc cos(alpha)
        // 0 = (1)c^2 - (2b * cos(alpha))c + b^2-a^2
        // using p c^2 + q c + r for the quadratic....
        // p = 1
        // q = -2b cos(alpha)
        // D = q^2 - 4pr
        double q = -2.0 * kPivotMountLength * cos(alpha);
        double D = q * q - 4.0 * (kPivotMountLength * kPivotMountLength - kPivotSupportLength * kPivotSupportLength);
        double c = (-q + sqrt(D))/2.0; // the other solution isn't relevant

        // c is the length along the lead screw to the base of the triangle. But this isn't home.
        // we need to add in the distance from home to the pivot point before converting back to position
        return ((c + kPivotLeadOffset) / kPivotLengthPerRotation);
    }

}
