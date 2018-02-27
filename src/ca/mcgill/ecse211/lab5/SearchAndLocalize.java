package ca.mcgill.ecse211.lab5;

import lejos.hardware.Sound;

public class SearchAndLocalize {
	public double lowerLeftX, lowerLeftY;
	public double upperRightX, upperRightY;
	private int targetBlock;
	private Navigation navigation;
	private boolean foundBlock = false;
	private ColourCalibration colourCalib;
	private double minCoord = 0.0, maxCoord = 8 * USLocalizer.TILESIZE;
	private double[][] destinations;

	public SearchAndLocalize(double llx, double lly, double urx, double ury, int tb, Navigation nav,
			ColourCalibration cc) {
		this.lowerLeftX = llx;
		this.lowerLeftY = lly;
		this.upperRightX = urx;
		this.upperRightY = ury;
		this.targetBlock = tb;

		this.navigation = nav;

		this.colourCalib = cc;

		/*
		 * The destinations array contains the list of target points that the robot will
		 * travel to one after the other in its quest to cover the entire grid.
		 */
		setDestinations();

	}

	
	/**
	 * fieldTest implements the second part of the demo.
	 * Go to LL, detect blocks, go to UR
	 */
	public void fieldTest() {

		// Set currentBlock to null, to avoid the last detected color from interfering
		this.colourCalib.resetBlock();

		// Travel to the lower-left corner
		this.navigation.travelTo(this.lowerLeftX, this.lowerLeftY, false, null);
		Sound.beep();
		// this.navigation.travelTo(this.lowerLeftX, this.lowerLeftY, false, null);
		// Sound.beep();

		/*
		 * Travel to each destination one by one, stopping the for loop if the correct
		 * block was found
		 */
		this.navigation.travelTo(this.lowerLeftX, this.upperRightY, true, this);
		this.navigation.travelTo(this.upperRightX, this.upperRightY, true, this);
		this.navigation.travelTo(this.upperRightX, this.lowerLeftY, true, this);
		this.navigation.travelTo(this.lowerLeftX, this.lowerLeftY, true, this);

		// for (double[] dest : destinations) {
		// if (foundBlock) {
		// break;
		// }
		// this.navigation.travelTo(dest[0], dest[1], true, this);
		// }

		// Once the correct block is found, go to to the upper right corner.
		this.navigation.travelTo(this.upperRightX, this.upperRightY, false, null);
	}

	/**
	 * sets the value of the boolean foundBlock 
	 * @param newVal : boolean value to set
	 */
	public void setFoundBlock(boolean newVal) {
		this.foundBlock = newVal;
	}

	/**
	 * set the destinations array that indicates to the robot where to go
	 */
	private void setDestinations() {

		this.destinations = new double[4][2];
		for (int i = 0; i < this.destinations.length; i++) {
			this.destinations[i][0] = getValue(i)[0];
			this.destinations[i][1] = getValue(i)[1];
		}

	}

	/**
	 * Helper method for the set destinations method.
	 * @param val
	 * @return
	 */
	private double[] getValue(int val) {
		/*
		 * llx, ury = 0 urx, ury = 1 urx, lly = 2 llx, lly = 3
		 */

		double value1 = 0, value2 = 0;

		switch (val) {
		case 0:
			value1 = this.lowerLeftX;
			value2 = this.upperRightY;
			break;
		case 1:
			value1 = this.upperRightX;
			value2 = this.upperRightY;
			break;
		case 2:
			value1 = this.upperRightX;
			value2 = this.lowerLeftY;
			break;
		case 3:
			value1 = this.lowerLeftX;
			value2 = this.lowerLeftY;
			break;
		}

		double[] returned = { value1, value2 };
		return returned;
	}

	/**
	 * Return the ColourCalibration instance
	 * @return
	 */
	public ColourCalibration getCC() {
		return this.colourCalib;
	}

	/**
	 * Return the current value of foundBlock
	 * @return
	 */
	public boolean getFoundBlock() {
		return this.foundBlock;
	}
}
