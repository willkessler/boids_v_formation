class Bird {
  
  PVector pos, vel, accel;
  float thrustConstant = 0.2;
  float maxSpeed, friction;
  color birdColor;
  int birdId;
  float birdWidth;
  float rot, rotChange, rotIncrement;
  float accelFactor;
  float mass = 1.0;
  double randomTurn;
  boolean thrustOn;

  Bird(int id, float x, float y, color sColor) {
    accel = new PVector(0,0);
    vel = new PVector(0,0);
    pos = new PVector(x,y);
    maxSpeed = 5;
    birdWidth = 15;
    birdColor = sColor;
    rot = 90;
    rotChange = 0;
    rotIncrement = 3;
    thrustOn = false;
    accelFactor = 0.009;
    birdId = id;
    friction = 0.995; 
  }
  
  void update() {
    
    if (thrustOn) {
      accel.x = sin(radians(rot)) * accelFactor;
      accel.y = -cos(radians(rot)) * accelFactor;
    }
    vel.add(accel);
    vel.mult(friction);
    //if (birdId == 0) {
    //  println("speed:", vel.mag());
    //}
    vel.limit(maxSpeed);
    pos.add(vel);
    rot = rot + rotChange;
    wrapAroundEdges(pos);
    accel.mult(0);
  }
  
 void drawBird(PVector pos, float rot, float proportion, color birdColor, boolean drawThrust) {
    pushMatrix();
    translate(pos.x,pos.y);
    scale(proportion);
    rotate(radians(rot));
    fill(0);
    stroke(birdColor);
    beginShape();
    vertex(-halfbirdWidth,  halfbirdHeight);
    vertex(0,  -halfbirdHeight);
    vertex(halfbirdWidth,  halfbirdHeight);
    vertex(0, halfbirdHeight / 2);
    endShape(CLOSE);
    if (drawThrust) {
      // draw flames
      float flicker = random(0,10) / 10 + 1;
      fill(255 * flicker,255 * flicker,0);
      stroke(255 * flicker,255 * flicker,0);
      beginShape();
      vertex(-halfbirdWidth / 2, halfbirdHeight * 1.1);
      vertex(0, halfbirdHeight * 1.6 * flicker);
      vertex(halfbirdWidth / 2, halfbirdHeight * 1.1);
      vertex(0, halfbirdHeight * 1.4);
      endShape(CLOSE);
    }
    popMatrix();
  }

  void applyThrust() {
    thrustOn = true; 
  }

  void cancelThrust() {
    thrustOn = false;
  }

 void startTurning(float direction) {
    rotChange = direction * rotIncrement;
  }
  
 void stopTurning() {
    rotChange = 0;
  }

  boolean isTurning() {
    return rotChange > 0;
  }

  void generateRandomTurn() {
    if (!isTurning()) {
      randomTurn = Math.random() + 1;
    }
  }

  void render() {
    drawBird(pos, rot, 1.0, birdColor, thrustOn);
   }
}