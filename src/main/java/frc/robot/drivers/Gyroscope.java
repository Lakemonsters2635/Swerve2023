// package org.frcteam2910.common.drivers;
package frc.robot.drivers;

import edu.wpi.first.math.geometry.Rotation2d;

// import org.frcteam2910.common.math.Rotation2;

public abstract class Gyroscope {
	private Rotation2d adjustmentAngle = new Rotation2d();
	private boolean inverted;

	public abstract void calibrate();

	public final Rotation2d getAdjustmentAngle() {
		return adjustmentAngle;
	}

	public void setAdjustmentAngle(Rotation2d adjustmentAngle) {
		this.adjustmentAngle = adjustmentAngle;
	}

	public final boolean isInverted() {
		return inverted;
	}

	public final void setInverted(boolean inverted) {
		this.inverted = inverted;
	}

	public abstract Rotation2d getUnadjustedAngle();
	public abstract double getUnadjustedRate();

    private static Rotation2d inverse(Rotation2d angle) {
        return new Rotation2d(angle.getCos(), -angle.getSin());
    }

	public final Rotation2d getAngle() {
		Rotation2d angle = getUnadjustedAngle().rotateBy(inverse(adjustmentAngle));

		if (inverted) {
			System.out.println(angle);
			return inverse(angle);
		}

		return angle;
	}

	public final double getRate() {
		double rate = getUnadjustedRate();

		if (inverted) {
			return -rate;
		}

		return rate;
	}
}
