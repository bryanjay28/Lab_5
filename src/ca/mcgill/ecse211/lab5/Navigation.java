package ca.mcgill.ecse211.lab5;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

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
	private static final int FORWARD_SPEED = 180;
	private static final int ROTATE_SPEED = 60;
	public static final int ACCELERATION = 2000;

	private boolean navigate = true;

	public USLocalizer usLoc;

	private static final Port usSidePort = LocalEV3.get().getPort("S4");
	private static SensorModes sideUltrasonicSensor;
	private static SampleProvider sideUsDistance;
	private float[] sideUsData;

	private static int distanceSensorToBlock = 2;

	// constructor for navigation
	public Navigation(Odometer odo, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
		this.odometer = odo;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.leftMotor.setAcceleration(ACCELERATION);
		this.rightMotor.setAcceleration(ACCELERATION);

		// usSensor is the instance
		sideUltrasonicSensor = new EV3UltrasonicSensor(usSidePort);
		// usDistance provides samples from this instance
		sideUsDistance = sideUltrasonicSensor.getMode("Distance");
		sideUsData = new float[sideUsDistance.sampleSize()];
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
			double dist = hypot;
			while ((dist = calculateDistance(odometer.getXYT()[0], odometer.getXYT()[1], x,
					y)) > distanceSensorToBlock) {
				
				leftMotor.rotate(convertDistance(Lab5.WHEEL_RAD, dist), true);
				rightMotor.rotate(convertDistance(Lab5.WHEEL_RAD, dist), true);
				if (blockDetected(search) == 1) {
					leftMotor.stop(true);
					rightMotor.stop(false);
					Sound.beep();

					double[] initialCoords = odometer.getXYT();
					turnTo(Math.toRadians(odometer.getXYT()[2]) + Math.PI / 2);
					goToBlock(search);
					if (search.getFoundBlock()) {
						return;
					}
					travelTo(initialCoords[0], initialCoords[1], false, null);
					
					leftMotor.rotate(convertDistance(Lab5.WHEEL_RAD, 10), true);
					rightMotor.rotate(convertDistance(Lab5.WHEEL_RAD, 10), true);
					
					travelTo(x, y, true, search);
					return;
					
				}
//				else if (blockDetected(search) == 2) {
//					leftMotor.stop(true);
//					rightMotor.stop(false);
//					
//					Sound.beep();
//					goToBlock(search);
//					if (search.getFoundBlock()) {
//						return;
//					}
//					goAround(search);
//					travelTo(x, y, true, search);
//					return;
//				}
			}

		}
		// stop vehicle
		leftMotor.stop(true);
		rightMotor.stop(false);
	}

	private void goToBlock(SearchAndLocalize searcher) {
		int dist = this.usLoc.fetchUS();
		if (dist > distanceSensorToBlock) {
			leftMotor.rotate(convertDistance(Lab5.WHEEL_RAD, dist - distanceSensorToBlock), true);
			rightMotor.rotate(convertDistance(Lab5.WHEEL_RAD, dist - distanceSensorToBlock), false);
		}
		searcher.getCC().colourDetection();
		if (searcher.getCC().isBlock()) {
			searcher.setFoundBlock(true);
		}
	}
//	private void goAround2(SearchAndLocalize searcher) {
//          0-------1-------2
//		    |               |
//		    |               |
//		    3               4
//		    |               |
//         	|               |	
//		    5-------6-------7
//		
//		int location = whereAmI(searcher);
//		
//	}
//	
//	private int whereAmI(SearchAndLocalize searcher) {
//		int location = -1;
//		double x = odometer.getXYT()[0];
//		double y = odometer.getXYT()[1];
//		if (y <= searcher.lowerLeftY + 3 || y >= searcher.lowerLeftY - 3) {
//			location = 5;
//		}
//		els
//		
//		
//		return location;
//	}
	
	private void goAround(SearchAndLocalize searcher) {

		int multiplier = 0;
		if (odometer.getXYT()[0] > searcher.lowerLeftX
				&& odometer.getXYT()[0] < searcher.lowerLeftX + USLocalizer.TILESIZE) {
			multiplier = 1;
			
		} else {
			multiplier = -1;
		}

		leftMotor.rotate(convertDistance(Lab5.WHEEL_RAD, -4 * distanceSensorToBlock), true);
		rightMotor.rotate(convertDistance(Lab5.WHEEL_RAD, -4 * distanceSensorToBlock), false);

		double currentHeading = odometer.getXYT()[2] * Math.PI / 180;
		double firstTurn = currentHeading + (multiplier * (Math.PI / 2));
		int firstDist = 15; // distance to travel after the first turn
		int secondDist = 20; // distance to travel after the second turn

		// turn 90 degrees anti-clockwise to circle around the block and go forward 15
		// cm
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

	private int blockDetected(SearchAndLocalize searcher) {
		/*
		 * 0: no block 1: side block 2: front block
		 */
		int sideDistance = fetchUS();
		int frontDistance = this.usLoc.fetchUS();
		if (sideDistance < (searcher.lowerLeftX - searcher.upperRightX)/2 * USLocalizer.TILESIZE + 5) {
			return 1;
		} else if (frontDistance < 5) {
			return 2;
		}
		return 0;
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

	public int fetchUS() {
		sideUsDistance.fetchSample(sideUsData, 0);
		return (int) (sideUsData[0] * 100);
	}
}
