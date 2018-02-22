package ca.mcgill.ecse211.lab5;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;

public class LightLocalizer {

	// vehicle constants
	public static int ROTATION_SPEED = 100;
	private double SENSOR_LENGTH = 11.8;

	private Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	public Navigation navigation;
	// Instantiate the EV3 Color Sensor
	private static final EV3ColorSensor lightSensor = new EV3ColorSensor(LocalEV3.get().getPort("S1"));
	private float sample;
	private float prevSample;
	private float deltaSample;

	private SensorMode idColour;

	double[] lineData;

	public LightLocalizer(Odometer odometer, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Navigation nav) {

		this.odometer = odometer;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		prevSample = 0;

		idColour = lightSensor.getRedMode(); // set the sensor light to red
		lineData = new double[4];
		this.navigation = nav;
	}

	/**
	 * This method localizes the robot using the light sensor to precisely move to
	 * the right location
	 */
	public void localize() {

		int index = 0;
		leftMotor.setSpeed(ROTATION_SPEED);
		rightMotor.setSpeed(ROTATION_SPEED);

		// ensure that we are close to origin before rotating
		moveToOneOne();

		// Scan all four lines and record our angle
		while (index < 4) {

			leftMotor.forward();
			rightMotor.backward();

			sample = fetchSample();
			deltaSample = sample - prevSample;
			if (deltaSample < 0.15) {

				lineData[index] = odometer.getXYT()[2];
				Sound.beepSequenceUp();
				index++;
			}
			
			prevSample = sample;
		}

		leftMotor.stop(true);
		rightMotor.stop();

		double deltax, deltay, thetax, thetay;

		// calculate our location from 0 using the calculated angles
		thetay = lineData[3] - lineData[1];
		thetax = lineData[2] - lineData[0];

		deltax = -1 * SENSOR_LENGTH * Math.cos(Math.toRadians(thetay / 2));
		deltay = -1 * SENSOR_LENGTH * Math.cos(Math.toRadians(thetax / 2));

		// travel to one-one to correct position
		odometer.setXYT(deltax, deltay, odometer.getXYT()[2] + 6);
		navigation.travelTo(USLocalizer.getTileSize(), USLocalizer.getTileSize());

		leftMotor.setSpeed(ROTATION_SPEED / 2);
		rightMotor.setSpeed(ROTATION_SPEED / 2);

		// if we are not facing 0.0 then turn ourselves so that we are
		if (odometer.getXYT()[2] <= 350 && odometer.getXYT()[2] >= 10.0) {
			Sound.beep();
			leftMotor.rotate(convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, -odometer.getXYT()[2]), true);
			rightMotor.rotate(-convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, -odometer.getXYT()[2]), false);
		}

		leftMotor.stop(true);
		rightMotor.stop();

	}

	/**
	 * This method moves the robot towards the point (1,1) of the grid (unit coordinates)
	 */
	public void moveToOneOne() {

		navigation.turnTo(Math.PI / 4);

		leftMotor.setSpeed(ROTATION_SPEED);
		rightMotor.setSpeed(ROTATION_SPEED);

		// get sample
		sample = fetchSample();
		
		// move forward past the origin until light sensor sees the line
		while (Math.abs(deltaSample) > 0.15) {
			sample = fetchSample();
			deltaSample = sample - prevSample;
			prevSample = sample;
			
			leftMotor.forward();
			rightMotor.forward();
		}
		
		leftMotor.stop(true);
		rightMotor.stop();
		Sound.beep();

		// Move backwards so our origin is close to origin
		leftMotor.rotate(convertDistance(Lab5.WHEEL_RAD, -12), true);
		rightMotor.rotate(convertDistance(Lab5.WHEEL_RAD, -12), false);

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

	/**
	 * This method gets the color value of the light sensor
	 * 
	 */
	private float fetchSample() {
		float[] colorValue = new float[idColour.sampleSize()];
		idColour.fetchSample(colorValue, 0);
		return colorValue[0];
	}

}
