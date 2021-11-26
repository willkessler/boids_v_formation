/**
 * v-formation flocking 
 * By Will Kessler
 * inspired by original flocking algorithm by Daniel Shiffman.  
 * 
 **/

int windowSize = 800;
int partWindow = windowSize /8;
float birdWidth = 15;
float birdHeight = birdWidth * 1.5;
float halfbirdHeight = birdHeight / 2;
float halfbirdWidth = birdWidth / 2;

int numBirds = 1;
Bird bird;

void setup() {
  size(1000, 600);

  bird = new Bird(0, partWindow, partWindow, color(0,255,0));

}

void draw() {
  background(50);
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
