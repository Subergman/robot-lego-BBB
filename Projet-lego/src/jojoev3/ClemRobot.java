package jojoev3;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.utility.Delay;
import lejos.hardware.motor.*;
import lejos.hardware.port.*;
import lejos.hardware.sensor.*;
import lejos.robotics.Color;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MeanFilter;
import lejos.robotics.chassis.*;
import java.io.*;  
import java.util.*;

public class ClemRobot {
	EV3MediumRegulatedMotor mG;
	EV3MediumRegulatedMotor mD;
	EV3MediumRegulatedMotor mP;
	Wheel rG;
	Wheel rD;
	Chassis chassis;

	EV3ColorSensor col;
	SampleProvider sampleCol;

	EV3TouchSensor pression;
	SampleProvider samplePress;
	float [] tabPress;

	EV3UltrasonicSensor ultrason;
	SampleProvider sampleUS;
	float [] usSample;
	float distanceValue;
	FileWriter fichierCouleurs;



	public ClemRobot() {
		mG=new EV3MediumRegulatedMotor(LocalEV3.get().getPort("A"));
		mD=new EV3MediumRegulatedMotor(LocalEV3.get().getPort("B"));
		rG= WheeledChassis.modelWheel(mG,56).offset(-61.5);
		rD= WheeledChassis.modelWheel(mD,56).offset(61.5);
		chassis=new WheeledChassis(new Wheel[] {rG,rD},WheeledChassis.TYPE_DIFFERENTIAL);

		/*EV3ColorSensor colorSensor = new EV3ColorSensor(port);
			SampleProvider average = new MeanFilter(colorSensor.getRGBMode(), 1);
			colorSensor.setFloodlight(Color.WHITE);

			System.out.println("Press enter to calibrate blue...");
			Button.ENTER.waitForPressAndRelease();
			float[] blue = new float[average.sampleSize()];
			average.fetchSample(blue, 0);*/

		mP=new EV3MediumRegulatedMotor(LocalEV3.get().getPort("C"));

		col=new EV3ColorSensor(LocalEV3.get().getPort("S4"));

		sampleCol=new MeanFilter(col.getRGBMode(), 1);

		pression=new EV3TouchSensor(LocalEV3.get().getPort("S2"));
		samplePress= pression.getTouchMode();
		tabPress=new float[samplePress.sampleSize()];
		
		ultrason=new EV3UltrasonicSensor(LocalEV3.get().getPort("S1"));
		sampleUS=ultrason.getDistanceMode();
		usSample= new float[sampleUS.sampleSize()];
		sampleUS.fetchSample(usSample, 0);
		distanceValue = usSample[0];
		// mG.synchronizeWith(new EV3MediumRegulatedMotor[] {mD});
		try {
			fichierCouleurs = new FileWriter("/home/lejos/programs/couleurs.txt");
		}catch(Exception e) {System.out.println(e);}

	}

	public int getSample() {
		return sampleCol.sampleSize();
	}

	public void Calibrage() {
		Properties codeRVB = new Properties();
		Set col;
		String code;
		System.out.println("Press enter to calibrate green...");
		Button.ENTER.waitForPressAndRelease();
		float[] green = new float[sampleCol.sampleSize()];
		sampleCol.fetchSample(green, 0);
		codeRVB.put(green, green[0]+","+green[1]+","+green[2]);
		col=codeRVB.keySet();  // get set-view of keys
		Iterator itr = col.iterator();

		while(itr.hasNext()) {
			code = (String) itr.next();
			System.out.println("The capital of " + str + " is " + 
					capitals.getProperty(str) + ".");
		}      
	}
	public void CalibrageCol() {
		boolean again=true;


		System.out.println("Press enter to calibrate blue...");
		Button.waitForAnyPress();
		col.setFloodlight(Color.WHITE);
		float[] blue = new float[sampleCol.sampleSize()];
		String bl= (String) "bleu: R "+blue[0]+" /V "+blue[1]+"   /B "+blue[2];
		sampleCol.fetchSample(blue, 0);
		System.out.println("bleu: R "+blue[0]+" /V "+blue[1]+"   /B "+blue[2]);
		try {
			fichierCouleurs.write(bl);
			fichierCouleurs.close();
		}catch(Exception e) {System.out.println(e);}
		Button.waitForAnyPress();

		System.out.println("Press enter to calibrate red...");
		Button.waitForAnyPress();
		col.setFloodlight(Color.WHITE);
		float[] red = new float[sampleCol.sampleSize()];
		sampleCol.fetchSample(red, 0);
		String re= (String) "rouge: R "+blue[0]+" /V "+blue[1]+"   /B "+blue[2];
		sampleCol.fetchSample(blue, 0);
		System.out.println("rouge: R "+blue[0]+" /V "+blue[1]+"   /B "+blue[2]);
		try {
			fichierCouleurs.write(re);
			fichierCouleurs.close();
		}catch(Exception e) {System.out.println(e);}
		Button.waitForAnyPress();

		/*
		System.out.println("Press enter to calibrate green...");
		Button.ENTER.waitForPressAndRelease();
		float[] green = new float[sampleCol.sampleSize()];
		sampleCol.fetchSample(green, 0);

		System.out.println("Press enter to calibrate black...");
		Button.ENTER.waitForPressAndRelease();
		float[] black = new float[sampleCol.sampleSize()];
		sampleCol.fetchSample(black, 0);
		System.out.println("Black calibrated");

		System.out.println("Press enter to calibrate yellow...");
		Button.ENTER.waitForPressAndRelease();
		float[] yellow = new float[sampleCol.sampleSize()];
		sampleCol.fetchSample(yellow, 0);
		System.out.println("Yellow calibrated");

		System.out.println("Press enter to calibrate white...");
		Button.ENTER.waitForPressAndRelease();
		float[] white = new float[sampleCol.sampleSize()];
		sampleCol.fetchSample(white, 0);
		System.out.println("White calibrated");


		while (again) {
			float[] sample = new float[sampleCol.sampleSize()];
			System.out.println("\nPress enter to detect a color...");
			Button.ENTER.waitForPressAndRelease();
			average.fetchSample(sample, 0);
			double minscal = Double.MAX_VALUE;
			String color = "";

			double scalaire = TestColor.scalaire(sample, blue);
			//Button.ENTER.waitForPressAndRelease();
			//System.out.println(scalaire);

			if (scalaire < minscal) {
				minscal = scalaire;
				color = "blue";
			}

			scalaire = TestColor.scalaire(sample, red);
			//System.out.println(scalaire);
			//Button.ENTER.waitForPressAndRelease();
			if (scalaire < minscal) {
				minscal = scalaire;
				color = "red";
			}

			scalaire = TestColor.scalaire(sample, green);
			//System.out.println(scalaire);
			//Button.ENTER.waitForPressAndRelease();
			if (scalaire < minscal) {
				minscal = scalaire;
				color = "green";
			}

			scalaire = TestColor.scalaire(sample, black);
			//System.out.println(scalaire);
			//Button.ENTER.waitForPressAndRelease();
			if (scalaire < minscal) {
				minscal = scalaire;
				color = "black";
			}

			scalaire = TestColor.scalaire(sample, yellow);
			//System.out.println(scalaire);
			//Button.ENTER.waitForPressAndRelease();
			if (scalaire < minscal) {
				minscal = scalaire;
				color = "yellow";
			}

			scalaire = TestColor.scalaire(sample, white);
			//System.out.println(scalaire);
			//Button.ENTER.waitForPressAndRelease();
			if (scalaire < minscal) {
				minscal = scalaire;
				color = "white";
			}
		 */
	}

	public void avancer() {
		System.out.println("Les affaires reprennent");
		System.out.println("vitesse max en mm/sec ? "+chassis.getMaxLinearSpeed());
		Button.waitForAnyPress();

		/* mG.setSpeed((int)(0.5*mG.getMaxSpeed()));
		mD.setSpeed((int)(0.5*mD.getMaxSpeed()));
		mG.synchronizeWith(new EV3MediumRegulatedMotor[] {mD});
		mG.rotate(1800, true);
		mD.rotate(1800, true);
		mG.waitComplete();
		mD.waitComplete(); */

		//chassis.rotate(-90);
		//Delay.msDelay((long)(3000));

	}
	public void avancerPlusArret() {
		System.out.println("Les affaires reprennent");
		Button.waitForAnyPress();
		System.out.println("test capteur US");
		sampleUS.fetchSample(usSample, 0); // attribution valeur capteur US à case 0 du tableau usSample
		distanceValue = usSample[0];
		System.out.println(distanceValue);


		for(int i = 0; i <50; i++) {

			sampleUS.fetchSample(usSample, 0);
			distanceValue = usSample[0];
			if(distanceValue>0.10) {
				System.out.println("le if est dans cette pute");
			} else {
				mG.stop();
				mD.stop();
				ouvrirPinces();
				System.out.println("le else est dans cette pute");
				break;
			}
			Delay.msDelay(200);


		}
		ouvrirPinces();


		Button.waitForAnyPress();
		Button.waitForAnyPress();
		//sampleUS.fetchSample(usSample, 0);

		/*mG.setSpeed((int)(0.5*mG.getMaxSpeed()));
		mD.setSpeed((int)(0.5*mD.getMaxSpeed()));
		mG.synchronizeWith(new EV3MediumRegulatedMotor[] {mD});
		//mG.rotate(1800, true);
		//mD.rotate(1800, true);
		mG.waitComplete();
		mD.waitComplete();
		Button.waitForAnyPress();*/

	}
	public void ouvrirPinces() {
		System.out.println("tacho avant de lancer l'ouverture"+mP.getTachoCount());
		System.out.println("get pos avant de lancer l'ouverture"+mP.getPosition());
		Button.waitForAnyPress();
		//mP.rotate(700);
		mP.forward();
		Button.waitForAnyPress();
		mP.stop();
		System.out.println("tacho après fermeture"+mP.getTachoCount()); //486
		System.out.println("pos après fermeture"+mP.getPosition()); //485
		Button.waitForAnyPress();

	}

	public void fermerPinces() {
		System.out.println("tacho avant de lancer l'ouverture"+mP.getTachoCount());
		System.out.println("get pos avant de lancer l'ouverture"+mP.getPosition());
		Button.waitForAnyPress();
		//mP.rotate(700);
		mP.backward();
		Button.waitForAnyPress();
		mP.stop();
		System.out.println("tacho après fermeture"+mP.getTachoCount()); //-467
		System.out.println("pos après fermeture"+mP.getPosition()); //-467
		Button.waitForAnyPress();
	}
	
	public boolean paletDansPince() {
		if(tabPress[0]==1) //1 ==> pression activée
			return true;
		return false;
	}

	public void ramasserPalet() {
		mP.forward(); //ouverture des pinces
		while(!paletDansPince() || mP.getPosition()<550) {
			System.out.println("ENZO LA PUTE"); 
			
		}
		mP.stop();
		while(!paletDansPince()) {
			
		}
		mP.backward();
		while(mP.getPosition()>20) {
			System.out.println("close"); 
			
		}
		mP.close();
		Button.waitForAnyPress();
	}

	public void virage(double rotation){

		//pour un rotate de 5000, un tour complet en 1?5sec environ
		//mG.synchronizeWith(new EV3MediumRegulatedMotor[] {mD});
		//chassis.rotate(angle);
		chassis.rotate(rotation);
		Delay.msDelay((long)((rotation/360)*2000));
	}



	public void avancerDe(double distance) {
		chassis.travel(distance);
	}

	public static double[] getMin(double tab[]) {
		int l=tab.length;
		double crt=tab[0];
		double index=-1;
		for (int i=1;i<l;i++)
			if(tab[i]<crt) {
				crt=tab[i];
				index=i;
			}
		double tabRetour[]= {crt,index};
		return tabRetour;
	}

	public void chercherObstaclePlusProche(int angle) {//donner angle divisible par 10
		int tailleEch=angle/10; //si angle divisible par 10, on peut stoquer des valeurs dans un tableau des distances tous les 10 degres
		int tabAngle[]=new int[2*tailleEch+1];//on tournera a gauche et a droite, donc *2 et +1 pour la valeur 0
		double tabDis[]=new double[2*tailleEch+1];
		int angleCrt=0;

		sampleUS.fetchSample(usSample, 0); // attribution valeur capteur US à case 0 du tableau usSample
		distanceValue = usSample[0];

		tabAngle[0]=0;
		tabDis[0]=distanceValue;//on stoque la valeur a l'angle 0
		System.out.println("teest");
		Button.waitForAnyPress();
		//if(dansTerrain()) {
		for(int i=1;i<tailleEch;i++) {   
			virage(10);
			angleCrt+=10;
			sampleUS.fetchSample(usSample, 0); 
			distanceValue = usSample[0];
			tabAngle[i]=angleCrt;//a chaque rotation de 10 degres on stoque les distances et les angles dans les tableaux
			tabDis[i]=distanceValue;  
		}
		virage(-angle);//on reviens dans la position initiale avant de tourner dans l'autre sens
		angleCrt=0;
		for(int i=1;i<tailleEch;i++) {   
			virage(-10);
			angleCrt-=10;
			sampleUS.fetchSample(usSample, 0); 
			distanceValue = usSample[0];
			tabAngle[tailleEch+i]=angleCrt;//a chaque rotation de 5 degres on stoque les distances et les angles dans les tableaux
			tabDis[tailleEch+i]=distanceValue;  
		}
		virage(angle);// on reviens dans position initiale
		double distanceMin=getMin(tabDis)[0];
		double angleMin=getMin(tabDis)[1];//on a trouvé dans quelle direction se trouvais l'obstacle le plus proche

		virage(angleMin);
		avancerDe(distanceMin-0.10);//on avance jusqua 10 cm avant l'obstacle, si il n'y est plus c'est un palet (ou un robot) il doit alors le saisir. si cetais un robot ou que le palet n'y est plus la fonction chercherObstacle le plus proche sera rappelee

		sampleUS.fetchSample(usSample, 0); 
		distanceValue = usSample[0];

		if (distanceValue<0.12) {//si la distance par rapport a l'objet initialement identifiée, alors c'etait un palet ou un robot une erreur de 2 cm est anticipée sur les mesures du robot(10 +-2)
			//saisirPalet();
			System.out.print("ya un palet");
		}
		else {
			//virage(90);
			System.out.print("ya pas de palet");
			//chercherObstaclePlusProche(angle);
		}
	}





	public static void main(String[] args) {
		ClemRobot TAMERE= new ClemRobot();
		//TAMERE.ouvrirPinces();
		//System.out.println(TAMERE.getSample());
		//TAMERE.ouvrirPinces();
		//TAMERE.fermerPinces();
		//TAMERE.avancer();
		//TAMERE.virage(720);
		TAMERE.ramasserPalet();
		//TAMERE.chercherObstaclePlusProche(50);
		//TAMERE.CalibrageCol();
	}

}