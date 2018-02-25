package ca.mcgill.ecse211.lab5;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

public class USLocalizer {

	// vehicle constants
	public static int ROTATION_SPEED = 100;
	private double deltaTheta;

	private Odometer odometer;
	private float[] usData;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private SampleProvider usDistance;
	private double[] startingCoordinates = new double[2];

	// Create a navigation
	public Navigation navigation;

	private double d = 40.0;
	private double k = 2;
	public static double TILESIZE = 30.48;

	/**
	 * Constructor to initialize variables
	 * 
	 * @param Odometer
	 * @param EV3LargeRegulatedMotor
	 * @param EV3LargeRegulatedMotor
	 * @param boolean
	 * @param SampleProvider
	 */
	public USLocalizer(Odometer odo, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			SampleProvider usDistance, int corner, Navigation nav) {
		this.odometer = odo;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.usDistance = usDistance;
		this.usData = new float[this.usDistance.sampleSize()];
		navigation = nav;

		setStartingCoordinates(corner);

		leftMotor.setSpeed(ROTATION_SPEED);
		rightMotor.setSpeed(ROTATION_SPEED);
	}

	public void setStartingCoordinates(int corner) {
		switch (corner) {
		case 0:
			this.startingCoordinates[0] = 0.0;
			this.startingCoordinates[1] = 0.0;
			break;
		case 1:
			this.startingCoordinates[0] = 6 * TILESIZE;
			this.startingCoordinates[1] = 0.0;
			break;
		case 2:
			this.startingCoordinates[0] = 6 * TILESIZE;
			this.startingCoordinates[1] = 6 * TILESIZE;
			break;
		case 3:
			this.startingCoordinates[0] = 0.0;
			this.startingCoordinates[1] = 6 * TILESIZE;
			break;
		}
	}

	/**
	 * A method to determine which localization method to write
	 * 
	 */
	public void localize() {
		// if we are facing a wall we use our rising edge localization
		if (fetchUS() < 40) {
			localizeRisingEdge();
		} else {
			localizeFallingEdge();
		}
	}

	/**
	 * A method to localize position using the rising edge
	 * 
	 */
	public void localizeRisingEdge() {

		double angleA, angleB, turningAngle;

		// Rotate to the wall
		while (fetchUS() > d) {
			leftMotor.backward();
			rightMotor.forward();
		}
		// Rotate until it sees the open space
		while (fetchUS() < d + k) {
			leftMotor.backward();
			rightMotor.forward();
		}
		leftMotor.stop(true);
		rightMotor.stop(true);
		Sound.buzz();
		// record angle
		angleA = odometer.getXYT()[2];

		// rotate the other way all the way until it sees the wall
		while (fetchUS() > d) {
			leftMotor.forward();
			rightMotor.backward();
		}

		// rotate until it sees open space
		while (fetchUS() < d + k) {
			leftMotor.forward();
			rightMotor.backward();
		}
		leftMotor.stop(true);
		rightMotor.stop(true);
		Sound.buzz();
		angleB = odometer.getXYT()[2];

		

		// calculate angle of rotation
		if (angleA < angleB) {
			deltaTheta = 45 - (angleA + angleB) / 2 + 180;
		} else if (angleA > angleB) {
			deltaTheta = 225 - (angleA + angleB) / 2 + 180;
		}

		turningAngle = deltaTheta + odometer.getXYT()[2];

		// rotate robot to the theta = 0.0 using turning angle and we account for small
		// error
		leftMotor.rotate(-convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, turningAngle - 6), true);
		rightMotor.rotate(convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, turningAngle - 6), false);

		// set theta to coordinate starting corner
		odometer.setXYT(startingCoordinates[0], startingCoordinates[1], 0.0);
	}

	/**
	 * A method to localize position using the falling edge
	 * 
	 */
	public void localizeFallingEdge() {

		double angleA, angleB, turningAngle;

		// Rotate to open space
		while (fetchUS() < d + k) {
			leftMotor.backward();
			rightMotor.forward();
		}
		// Rotate to the first wall
		while (fetchUS() > d) {
			leftMotor.backward();
			rightMotor.forward();
		}
		Sound.buzz();
		// record angle
		angleA = odometer.getXYT()[2];

		// rotate out of the wall range
		while (fetchUS() < d + k) {
			leftMotor.forward();
			rightMotor.backward();
		}

		// rotate to the second wall
		while (fetchUS() > d) {
			leftMotor.forward();
			rightMotor.backward();
		}
		Sound.buzz();
		angleB = odometer.getXYT()[2];

		leftMotor.stop(true);
		rightMotor.stop();

		// calculate angle of rotation
		if (angleA < angleB) {
			deltaTheta = 45 - (angleA + angleB) / 2;

		} else if (angleA > angleB) {
			deltaTheta = 225 - (angleA + angleB) / 2;
		}

		turningAngle = deltaTheta + odometer.getXYT()[2];

		// rotate robot to the theta = 0.0 and we account for small error
		leftMotor.rotate(-convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, turningAngle - 1), true);
		rightMotor.rotate(convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, turningAngle - 1), false);

		// set odometer to theta to starting corner
		odometer.setXYT(startingCoordinates[0], startingCoordinates[1], 0.0);

	}

	/**
	 * A method to get the distance from our sensor
	 * 
	 * @return
	 */
	public int fetchUS() {
		usDistance.fetchSample(usData, 0);
		return (int) (usData[0] * 100);
	}

	/**
	 * This method allows the conversion of a distance to the total rotation of each
	 * wheel need to cover that distance.
	 * 
	 * @param radius
	 * @param distance
	 * @return
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	/**
	 * This method allows the conversion of a angle to the total rotation of each
	 * wheel need to cover that distance.
	 * 
	 * @param radius
	 * @param distance
	 * @param angle
	 * @return
	 */
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

}
