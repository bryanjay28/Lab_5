package ca.mcgill.ecse211.lab5;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Navigation extends Thread {

	private Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;

	private double deltaX;
	private double deltaY;

	// current location of the vehicle
	private double currX;
	private double currY;
	private double currTheta;

	// set constants
	private static final int FORWARD_SPEED = 100;
	private static final int ROTATE_SPEED = 60;
	public static final int ACCELERATION = 2000;  

	private boolean navigate = true;
	
	public USLocalizer usLoc;	

	// constructor for navigation
	public Navigation(Odometer odo, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
		this.odometer = odo;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.leftMotor.setAcceleration(ACCELERATION);
		this.rightMotor.setAcceleration(ACCELERATION);
	}

	/**
	 * A method to drive our vehicle to a certain Cartesian coordinate
	 * 
	 * @param x
	 *            X-Coordinate
	 * @param y
	 *            Y-Coordinate
	 */
	public void travelTo(double x, double y, boolean lookForBlocks, SearchAndLocalize search) {

		/*
		 * The search instance of SearchAndLocalize in the parameters is only necessary
		 * when lookForBlocks is true. Therefore, in calls where lookForBlocks is false,
		 * we pass null to the search variable.
		 */

		currX = odometer.getXYT()[0];
		currY = odometer.getXYT()[1];

		deltaX = x - currX;
		deltaY = y - currY;

		// Calculate the angle to turn around
		currTheta = (odometer.getXYT()[2]) * Math.PI / 180;
		double mTheta = Math.atan2(deltaX, deltaY) - currTheta;
		double hypot = Math.hypot(deltaX, deltaY);

		// Turn to the correct angle towards the endpoint
		turnTo(mTheta);

		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);

		if (!lookForBlocks) {
			// We are going to our destination without bothering to check for blocks
			leftMotor.rotate(convertDistance(Lab5.WHEEL_RAD, hypot), true);
			rightMotor.rotate(convertDistance(Lab5.WHEEL_RAD, hypot), false);
		} else {
			leftMotor.rotate(convertDistance(Lab5.WHEEL_RAD, hypot), true);
			rightMotor.rotate(convertDistance(Lab5.WHEEL_RAD, hypot), true);
			while (true) {
				// if a block is detected (regardless of whether it's the right one)
				if (blockDetected()) {
					leftMotor.stop(true);
					rightMotor.stop(true);
					ColourCalibration cc = search.getCC();
					// assess the colour of the block
					cc.colourDetection();
					if (cc.isBlock()) {
						// if it's the right block, save that fact and return
						search.setFoundBlock(true);
						return;
					} else {
						// otherwise, reset the value of currentBlock to null
						cc.resetBlock();
						// avoid the obstacle
						goAround();
						// and start travelling back to where we were going
						travelTo(x, y, lookForBlocks, search);
						return;
					}
				}
			}
		}
		// stop vehicle
		leftMotor.stop(true);
		rightMotor.stop(false);
	}
	
	private void goAround() {
		
		/*
		 * TODO: Add code here so that if the robot is on the left edge of the grid,
		 * it turns 90 degrees clockwise instead of anti-clockwise 
		 * to avoid falling from the grid floor.
		 */
		
		double currentHeading = odometer.getXYT()[2] * Math.PI / 180;
		double firstTurn = currentHeading - (Math.PI / 2);
		int firstDist = 15; // distance to travel after the first turn
		int secondDist = 20; // distance to travel after the second turn
		
		// turn 90 degrees anti-clockwise to circle around the block and go forward 15 cm
		turnTo(firstTurn);
		moveDistance(firstDist);
		
		// turn back to our original heading and go forward 20 cm
		turnTo(currentHeading);
		moveDistance(secondDist);
	}
	
	private void moveDistance(int dist) {
		leftMotor.rotate(convertDistance(Lab5.WHEEL_RAD, dist), true);
		rightMotor.rotate(convertDistance(Lab5.WHEEL_RAD, dist), false);
		leftMotor.stop(true);
		rightMotor.stop(false);
	}

	private boolean blockDetected() {
		int distance = this.usLoc.fetchUS();
		return distance < 5;
	}

	/**
	 * A method to turn our vehicle to a certain angle
	 * 
	 * @param theta
	 */
	public void turnTo(double theta) {

		// ensures minimum angle for turning
		if (theta > Math.PI) {
			theta -= 2 * Math.PI;
		} else if (theta < -Math.PI) {
			theta += 2 * Math.PI;
		}

		// set Speed
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);

		// rotate motors at set speed

		// if angle is negative, turn to the left
		if (theta < 0) {
			leftMotor.rotate(-convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, -(theta * 180) / Math.PI), true);
			rightMotor.rotate(convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, -(theta * 180) / Math.PI), false);

		} else {
			// angle is positive, turn to the right
			leftMotor.rotate(convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, (theta * 180) / Math.PI), true);
			rightMotor.rotate(-convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, (theta * 180) / Math.PI), false);
		}
		leftMotor.stop(true);
		rightMotor.stop(false);
	}

	/**
	 * A method to determine whether another thread has called travelTo and turnTo
	 * methods or not
	 * 
	 * @return
	 */
	boolean isNavigating() throws OdometerExceptions {
		return navigate;
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

	public static double calculateDistance(double x1, double y1, double x2, double y2) {
		return Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
	}
}
