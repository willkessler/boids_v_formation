/**
 * v-formation flocking 
 * By Will Kessler
 * inspired by original flocking algorithm by Daniel Shiffman.  
 * 
 **/

int windowSize;
int partWindow;
float birdWidth = 15;
float birdHeight = birdWidth * 1.5;
float halfbirdHeight = birdHeight / 2;
float halfbirdWidth = birdWidth / 2;

int numBirds = 1;
Bird bird;
Bird leadingBird;
int FORWARDTHRUST = 0;
int LATERALTHRUST = 1;


void setup() {
  size(1000, 1000);
  windowSize = 1000; // make sure match to the size() call on previous line
  partWindow = windowSize / 2;

  leadingBird = new Bird(0, partWindow, partWindow, color(0,0,255));
  bird = new Bird(1, partWindow, partWindow * 1.75, color(0,255,0));

}

void draw() {
  background(50);
  
  //leadingBird.updateAutoThrust();
  leadingBird.generateRandomTurn();
  leadingBird.update();
  leadingBird.render();
  leadingBird.showTrailingSpot(true);

  bird.thrustOrAlignWithLeadingBird(leadingBird);
  bird.update();
  bird.render();
}

/* ------------------------------------------------------------------------------------------------------- */

void keyPressed() {  
  switch (key) {
 
   case 's':
    bird.applyThrust(FORWARDTHRUST);
    break;
   case 'a':
    bird.startTurning(-1);
    break;
   case 'd':
    bird.startTurning(1);
    break;
   case 'i':
    leadingBird.moveLinear(0,-10);
    break;
   case 'j':
    leadingBird.moveLinear(-10,0);
    break;
   case 'k':
    leadingBird.moveLinear(0,10);
    break;
   case 'l':
    leadingBird.moveLinear(10,0);
    break;
  }
  bird.cancelThrust(FORWARDTHRUST);
  bird.cancelThrust(LATERALTHRUST);
}

void keyReleased() {
 switch (key) {
   case 's':
    bird.cancelThrust(FORWARDTHRUST);
    break;
   case 'a':
   case 'd':
    bird.stopTurning();
     break;
  }
}

// wrap a moving object around screen edges
void wrapAroundEdges(PVector pos) {
  if (pos.x < 0) {
   pos.x = windowSize; 
  }
  if (pos.y < 0) {
    pos.y = windowSize;
  }
  if (pos.x > windowSize) {
   pos.x = 0; 
  }
  if (pos.y > windowSize) {
    pos.y = 0;
  }
}

float angleBetweenVectors(PVector v1, PVector v2) {
  float dp = v1.dot(v2);
  float denom = v1.mag() * v2.mag();
  float angle = acos(dp/denom);
  return degrees(angle);
}

String printPVector(String label, PVector p1) {
  return (label + ": [" + p1.x + "," + p1.y + "] ");
}