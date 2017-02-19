package org.usfirst.frc.team3223.robot;

public class RotationalProfiler {
	double absAccel; // rad/s^2
	double accel;
	double absVMaxTra;// rad/s
	double vMaxTra;
	boolean isTrapezoid;
	double initialVelocity;
	boolean isOnlyDeccelerating;
	boolean isNegativeAngle;

	/**
	 * time, in seconds, of acceleration phase
	 */
	double t1;

	/**
	 * time, in seconds, of constant velocity phase
	 */
	double t2;

	/**
	 * time, in seconds, of deceleration phase
	 */
	double t3;

	double absVMaxTri;
	double vMaxTri;
	boolean hasOvershooted = false;

	/**
	 * calculate profile assuming current velocity is 0 and
	 * 
	 * @param angle
	 *            the desired distance to travel in radians between -pi and +pi
	 */
	public void calculate(double angle) {
		if (angle < 0) {
			isNegativeAngle = true;
			angle = Math.abs(angle);
			accel = -absAccel;
			vMaxTra = -absVMaxTra;
		} else {
			isNegativeAngle = false;
			accel = absAccel;
			vMaxTra = absVMaxTra;
		}
		isOnlyDeccelerating = false;
		initialVelocity = 0;
		absVMaxTri = absAccel * Math.sqrt(angle / absAccel);
		if (isNegativeAngle) {
			vMaxTri = -absVMaxTri;
		} else {
			vMaxTri = absVMaxTri;
		}
		isTrapezoid = absVMaxTri > absVMaxTra;
		if (isTrapezoid) {
			t1 = (absVMaxTra / absAccel);
			t2 = ((angle / absVMaxTra) - (absVMaxTra / absAccel));
			t3 = t1;
		} else {
			t1 = (absVMaxTri / absAccel);
			t2 = 0;
			t3 = t1;
		}

	}

	/**
	 * recalculate profile given
	 * 
	 * @param remainingAngle
	 *            desired distance to travel in radians
	 * @param currentVelocity
	 *            radians/second
	 */
	public void recalculate(double remainingAngle, double currentVelocity) {
		if (hasOvershooted) {
			t1 = t2 = t3 = 0;
			return;
		}
		this.initialVelocity = currentVelocity;
		double quadraticB = 2 * currentVelocity;
		double sortaAbsInitialVelocity;
		if (remainingAngle < 0) {
			accel = -absAccel;
			sortaAbsInitialVelocity = -currentVelocity;
			vMaxTra = -absVMaxTra;
		} else {
			accel = absAccel;
			sortaAbsInitialVelocity = currentVelocity;
			vMaxTra = absVMaxTra;
		}
		// if we started decelerating now, this is the shortest distance we can
		// travel
		double minimumAngle = currentVelocity * currentVelocity / 2 / absAccel;

		if (minimumAngle > Math.abs(remainingAngle)) {
			// and we're going to overshoot.
			hasOvershooted = true;
			if (currentVelocity < 0) {
				accel = -absAccel;
			} else {
				accel = absAccel;
			}
			isOnlyDeccelerating = true;
			isTrapezoid = false;
			t1 = t2 = 0;
			t3 = Math.abs(currentVelocity / absAccel);
			return;
		}
		if (remainingAngle < 0) {
			quadraticB *= -1;
		}
		// ta is time we can accelerate assuming no max velocity until we have
		// to start decelerating
		double ta = quadratic1(absAccel, quadraticB, minimumAngle - Math.abs(remainingAngle));
		if (ta < 0 || Double.isNaN(ta)) {
			t1 = 0;
			t2 = 0;
			t3 = 0;
			return;
		}
		vMaxTri = currentVelocity + accel * ta;
		absVMaxTri = Math.abs(vMaxTri);

		isTrapezoid = absVMaxTri > absVMaxTra;
		if (isTrapezoid) {
			t1 = (absVMaxTra - sortaAbsInitialVelocity) / absAccel;
			t3 = Math.sqrt(2 * absVMaxTra / absAccel);

			double theta1 = sortaAbsInitialVelocity * t1 + 0.5 * absAccel * t1 * t1;
			double theta3 = absVMaxTra * t3 - 0.5 * absAccel * t3 * t3;
			t2 = (Math.abs(remainingAngle) - theta1 - theta3) / absVMaxTra;
		} else {
			t1 = ta;
			t2 = 0;
			t3 = absVMaxTri / absAccel;
		}
	}

	public void reset() {
		hasOvershooted = false;
	}

	/**
	 * quadratic formula, the part that might give us a positive number
	 */
	private double quadratic1(double a, double b, double c) {
		double way1 = (-b + Math.sqrt(b * b - 4 * a * c)) / 2 / a;
		double way2 = (-b - Math.sqrt(b * b - 4 * a * c)) / 2 / a;
		if (way1 > 0) {
			return way1;
		} else {
			return way2;
		}
	}

	/**
	 * @param timeMs
	 * @return velocity at timeMs in rad/s
	 */
	public double getVelocity(long timeMs) {
		double time = timeMs / 1000.00;
		if (0 <= time && time < t1) {
			return accel * time + initialVelocity;
		} else if (t1 <= time && time < t1 + t2) {
			return vMaxTra;
		} else if (t1 + t2 <= time && time < t1 + t2 + t3) {
			if (isOnlyDeccelerating) {
				return initialVelocity - accel * time;
			}
			if (isTrapezoid) {
				return vMaxTra - accel * (time - (t1 + t2));
			} else {
				return vMaxTri - accel * (time - t1);
			}
		}
		return 0;
	}

	public boolean isDone(long timeMs) {
		return timeMs / 1000.00 >= t1 + t2 + t3;
	}
	
	public RotationalProfiler(double maxVel, double accel){
		this.absVMaxTra = maxVel;
		this.absAccel = accel;
	}
}

