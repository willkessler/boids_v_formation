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

void setup() {
  size(1000, 1000);
  windowSize = 1000; // make sure match to the size() call on previous line
  partWindow = windowSize /8;

  leadingBird = new Bird(0, partWindow, partWindow, color(0,0,255));
  bird = new Bird(1, partWindow, partWindow, color(0,255,0));

}

void draw() {
  background(50);
  
  leadingBird.updateAutoThrust();
  leadingBird.generateRandomTurn();
  leadingBird.update();
  leadingBird.render();

  bird.update();
  bird.render();
}

/* ------------------------------------------------------------------------------------------------------- */

void keyPressed() {  
  switch (key) {
 
   case 's':
    bird.applyThrust();
    break;
   case 'a':
    bird.startTurning(-1);
    break;
   case 'd':
    bird.startTurning(1);
    break;
  }   
}

void keyReleased() {
 switch (key) {
   case 's':
    bird.cancelThrust();
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
