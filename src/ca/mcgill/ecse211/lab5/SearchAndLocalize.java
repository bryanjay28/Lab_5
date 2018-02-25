package ca.mcgill.ecse211.lab5;

import lejos.hardware.Sound;

public class SearchAndLocalize {
	private double lowerLeftX, lowerLeftY;
	private double upperRightX, upperRightY;
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
		
		int widthOfSearchArea = (int)((urx - llx) / USLocalizer.TILESIZE);
		
		/*
		 * The destinations array contains the list of target points
		 * that the robot will travel to one after the other in its
		 * quest to cover the entire grid.
		 */
		destinations = new double[2 * widthOfSearchArea][2];
		setDestinations();

	}

	public void fieldTest() {
		
		// Set currentBlock to null, to avoid the last detected color from interfering
		this.colourCalib.resetBlock();
		
		// Travel to the lower-left corner
		this.navigation.travelTo(this.lowerLeftX, this.lowerLeftY, false, null);
		Sound.beep();
		
		/*
		 * Travel to each destination one by one,
		 * stopping the for loop if the correct block was found
		 */
		for (double[] dest : destinations) {
			if (foundBlock) {
				break;
			}
			this.navigation.travelTo(dest[0], dest[1], true, this);
		}
				
		// Once the correct block is found, go to to the upper right corner.
		this.navigation.travelTo(this.upperRightX, this.upperRightY, false, null);
	}

	private double switchXValue(double x) {
		// Returns llx if x == urx, returns urx if x == llx
		return this.lowerLeftX + this.upperRightX - x;
	}

	public void setFoundBlock(boolean newVal) {
		this.foundBlock = newVal;
	}
	
	private void setDestinations() {
		for (int i = 0; i < destinations.length; i++) {
			double xDest = this.lowerLeftX + (i + 1) * (USLocalizer.TILESIZE / 2);
			double yDest = (i % 2 == 0 ? this.upperRightY : this.lowerLeftY);
			destinations[i][0] = xDest;
			destinations[i][1] = yDest;
		}
	}
	
	public ColourCalibration getCC() {
		return this.colourCalib;
	}
}
