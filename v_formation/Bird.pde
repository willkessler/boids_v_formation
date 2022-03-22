class Bird {
  
  PVector pos, vel, accel;
  PVector thrustVector;
  float thrustConstant = 0.2;
  float maxSpeed, friction;
  color birdColor;
  int birdId;
  float birdWidth;
  float rot, rotChange, rotIncrement;
  float initialThrustStrength;
  float[] currentThrustStrength = new float[2];
  float mass = 1.0;
  float randomTurn;
  float turnDecay = 0.95;
  float bankDecay = 0.9;
  int thrustDuration = 10;
  int thrustPauseDuration = 30;
  int[] thrustCounter = new int[2];
  int[] thrustPauseCounter = new int[2];
  boolean[] thrustOn = new boolean[2];

  int[] thrustTimer = new int[2];
  Boolean renderTrailingSpot;
  
  float leadingBirdRange = windowSize; // can track you over entire screen right now
  float leadingBirdAngleTolerance = 360;
  float birdSmartFactor = 1;

  Bird(int id, float x, float y, color sColor) {
    accel = new PVector(0,0);
    vel = new PVector(0,0);
    pos = new PVector(x,y);
    thrustVector = new PVector(0,0);
    maxSpeed = 5;
    birdWidth = 15;
    birdColor = sColor;
    rot = -90;
    rotChange = 0;
    rotIncrement = 3;
    thrustOn[FORWARDTHRUST] = false;
    thrustOn[LATERALTHRUST] = false;

    initialThrustStrength = 0.009;
    currentThrustStrength[FORWARDTHRUST] = initialThrustStrength;
    currentThrustStrength[LATERALTHRUST] = initialThrustStrength;
    birdId = id;
    friction = 0.985;
    randomTurn = 0;
    thrustTimer[FORWARDTHRUST] = 0;
    thrustTimer[LATERALTHRUST] = 0;
    renderTrailingSpot = false;
  }
  
  void update() {
    
    // set forward acceleration and add to the velocity
    if (thrustOn[FORWARDTHRUST]) {
      accel.x = cos(radians(rot)) * currentThrustStrength[FORWARDTHRUST];
      accel.y = sin(radians(rot)) * currentThrustStrength[FORWARDTHRUST];
    }
    vel.add(accel);

    // set lateral accel/decel (banking) and add to the velocity as well
    if (thrustOn[LATERALTHRUST]) {
      vel.add(thrustVector);
    }
    
    vel.mult(friction);
    
    //if (birdId == 0) {
    //  println("speed:", vel.mag());
    //}
    //println("accel:",accel.x,accel.y, "vel:", vel.x, vel.y);
    vel.limit(maxSpeed);
    pos.add(vel);
    rot = rot + rotChange;
    wrapAroundEdges(pos);
    accel.mult(0);
  }
  
 void drawBird(PVector pos, float rot, float proportion, color birdColor, boolean drawThrust) {
    float bankingFactor = 1.0;
    pushMatrix();
    translate(pos.x,pos.y);
    scale(proportion);
    rotate(radians(rot+90));
    fill(0);
    stroke(birdColor);
    beginShape();
    if (birdId > 0) {
      bankingFactor = 1.0 - (getThrustVector().mag() * 2);
      println("bankingFactor:", bankingFactor);
    }
    vertex(-halfbirdWidth * bankingFactor,  halfbirdHeight);
    vertex(0,  -halfbirdHeight);
    vertex(halfbirdWidth * bankingFactor,  halfbirdHeight);
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
    
    if (renderTrailingSpot) {
      stroke(birdColor);
      fill(birdColor);
      PVector trailingSpot = getTrailingSpot();
      ellipse(trailingSpot.x, trailingSpot.y,10, 10);
      PVector parallelLineAtTrailingSpot = new PVector(1,0);
      parallelLineAtTrailingSpot.rotate(radians(rot));
      parallelLineAtTrailingSpot.mult(50);

      pushMatrix();
      translate(trailingSpot.x, trailingSpot.y);
      fill(0);
      stroke(birdColor);
      beginShape();
      vertex(0,0);
      vertex(parallelLineAtTrailingSpot.x, parallelLineAtTrailingSpot.y);
      endShape(CLOSE);
      popMatrix();
    }
  }

  void moveLinear(float xMove, float yMove) {
    float scalar = 5;
    pos.x += xMove * scalar;
    pos.y += yMove * scalar;
  }

  void applyThrust(int whichThrust) {
    if (!thrustOn[whichThrust] && thrustCounter[whichThrust] == 0 && thrustPauseCounter[whichThrust] == 0) {
      thrustOn[whichThrust] = true;
      thrustCounter[whichThrust] = thrustDuration;
      return;
    }
    
    if (thrustCounter[whichThrust] > 0) {
      thrustCounter[whichThrust]--;
      if (thrustCounter[whichThrust] == 0) {
        thrustPauseCounter[whichThrust] = thrustPauseDuration;
        thrustOn[whichThrust] = false;
      }
    }
  
    if (thrustPauseCounter[whichThrust] > 0) {
      thrustPauseCounter[whichThrust]--;
      if (thrustPauseCounter[whichThrust] == 0) {
        thrustOn[whichThrust] = true;
        thrustCounter[whichThrust] = thrustDuration;
      }
    }

  }

  void setThrustStrength(float thrustStrength, int whichThrust) {
    currentThrustStrength[whichThrust] = thrustStrength;    
  }

  PVector getThrustVector() {
    return new PVector(thrustVector.x, thrustVector.y);
  }

  void setThrustVector(PVector tv) {
    thrustVector.set(tv.x,tv.y);
  }

  void cancelThrust(int whichThrust) {
    thrustOn[whichThrust] = false;
    thrustCounter[whichThrust] = 0;
  }

 void startTurning(float direction) {
    rotChange = direction * rotIncrement;
  }
  
 void stopTurning() {
    rotChange = 0;
  }

  boolean isTurning() {
    return (Math.abs(rotChange) > 0);
  }

  float getRot() {
    return rot;
  }

  float getVelocity() {
    return vel.mag();
  }
  
  PVector getTrailingSpot() {
    PVector trailingSpot = new PVector(1,0);
    trailingSpot.normalize();
    trailingSpot.rotate(radians(45));
    trailingSpot.mult(100.0);
    trailingSpot.add(pos);
    return trailingSpot;
  }

  float getDistanceToPos(PVector pos1, PVector pos2) {
    PVector distanceVec = new PVector(pos2.x, pos2.y);
    distanceVec.sub(pos1);
    float distance = distanceVec.mag();
    return (distance);
  }

  void showTrailingSpot(Boolean dts) {
    renderTrailingSpot = dts;    
  }

  float getAngleToTrailingSpot(Bird leadingBird) {
    PVector trailingSpot = leadingBird.getTrailingSpot();
    PVector trailingSpotToThisBird = new PVector(trailingSpot.x, trailingSpot.y);
    trailingSpotToThisBird.sub(pos);
    trailingSpotToThisBird.normalize();
    float rotRadians = radians(rot);
    PVector pointingDirection = new PVector(cos(rotRadians), sin(rotRadians));
    pointingDirection.normalize();
    float angleToTrailingSpot = angleBetweenVectors(pointingDirection, trailingSpotToThisBird);

    return angleToTrailingSpot;
  }
  
  // calculate if the leading bird is near enough to "see" and in front of this bird. Return a zero vector if not.
  // otherwise return a vector indicating a velocity change to track the leading bird.
  PVector calculateLeadingBirdVelocityAdjustment(Bird leadingBird) {
    PVector adjustment = new PVector(0,0);
    PVector leadingBirdToThisBird = new PVector(0,0);
    leadingBirdToThisBird.set(leadingBird.pos);
    leadingBirdToThisBird.sub(pos);
    float distanceToLeadingBird = leadingBirdToThisBird.mag();
    if (distanceToLeadingBird > leadingBirdRange) { // have to be close to it, first off
      return adjustment;
    }
    leadingBirdToThisBird.normalize();
    float rotRadians = radians(rot);
    PVector pointingDirection = new PVector(cos(rotRadians), sin(rotRadians));
    pointingDirection.normalize();
    float angleToLeadingBird = angleBetweenVectors(pointingDirection, leadingBirdToThisBird);
    
    if (angleToLeadingBird <= leadingBirdAngleTolerance) { // limited view in front of the bird
       PVector crossProduct = pointingDirection.cross(leadingBirdToThisBird);
       // calculate turn based on how large the angle is
       float adjustmentAngle = angleToLeadingBird  * crossProduct.z;

/* 
       float adjustmentAngleRadians = radians(adjustmentAngle);
       float ca = cos(adjustmentAngleRadians);
       float sa = sin(adjustmentAngleRadians);
       adjustment.set(ca * pointingDirection.x - sa * pointingDirection.y,
                      sa * pointingDirection.x + ca * pointingDirection.y);
       adjustment.normalize();
       adjustment.mult(birdSmartFactor);
       
 */       // println("  Adjustment vector:", adjustment);
       rotChange = adjustmentAngle / 10.0;
       println("rot:", rot,
               " | AngleToLeadingBird:", angleToLeadingBird, 
               " | adjustmentAngle:", adjustmentAngle, 
               " | pointingDirection: [", pointingDirection.x, pointingDirection.y, "]");
    }
    return adjustment;
  }

  void pointSameWayAsLeadingBird(Bird leadingBird) {
    float adjustment = leadingBird.getRot() - rot;
    rotChange = adjustment / 10.0;
  }

  void pointAtTrailingSpot(Bird leadingBird) {
    PVector trailingSpot = leadingBird.getTrailingSpot();
    trailingSpot.sub(pos);
    float distanceToLeadingBird = trailingSpot.mag();
    //if (distanceToLeadingBird > leadingBirdRange) { // have to be close to it, first off
    //  return;
   // }
    trailingSpot.normalize();
    float rotRadians = radians(rot);
    PVector pointingDirection = new PVector(cos(rotRadians), sin(rotRadians));
    pointingDirection.normalize();
    float angleToTrailingSpot = angleBetweenVectors(pointingDirection, trailingSpot);
    PVector crossProduct = pointingDirection.cross(trailingSpot);
    // calculate turn based on how large the angle is
    float adjustmentAngle = angleToTrailingSpot  * crossProduct.z;
    if (abs(adjustmentAngle) < 91) {
      rotChange = adjustmentAngle / 2;
    }
    
    /*
    println(printPVector("pointingDirection", pointingDirection),
            printPVector("trailingSpot", trailingSpot),
            "adjustmentAngle:", adjustmentAngle);
    */

    // draw a line from bird to trailing spot
    /*
    stroke(birdColor);
    beginShape();
    vertex(pos.x,pos.y);
    PVector ts2 = leadingBird.getTrailingSpot();
    vertex(ts2.x, ts2.y);
    endShape(CLOSE);
    */
  }

  void pointAtLeadingBird(Bird leadingBird) {
    float lineLen = 10000;
    PVector leadingBirdVelocityAdjustment = calculateLeadingBirdVelocityAdjustment(leadingBird);
    //println("Leading bird direction:[" + leadingBirdDirection.x + "," + leadingBirdDirection.y + "]");
    
    stroke(birdColor);
    beginShape();
    //vertex(leadingBirdVelocityAdjustment.x * lineLen, leadingBirdVelocityAdjustment.y * lineLen);
    PVector trailingSpot = leadingBird.getTrailingSpot();
    vertex(pos.x,pos.y);
    vertex(trailingSpot.x, trailingSpot.y);
    endShape(CLOSE);

    //vel.add(leadingBirdVelocityAdjustment);
  }

  void matchLeadingBirdDirection(Bird leadingBird) {
    float rotRadians = radians(rot);
    PVector pointingDirection = new PVector(cos(rotRadians), sin(rotRadians));
    pointingDirection.normalize();
    float lbRotRadians = radians(leadingBird.getRot());
    PVector lbPointingDirection = new PVector(cos(lbRotRadians), sin(lbRotRadians));
    lbPointingDirection.normalize();

    float angleToLeadingBirdDirection = angleBetweenVectors(pointingDirection, lbPointingDirection);
    
    PVector crossProduct = pointingDirection.cross(lbPointingDirection);
    // calculate turn based on how large the angle is
    float adjustmentAngle = angleToLeadingBirdDirection  * crossProduct.z;
    rotChange = adjustmentAngle / 10.0;
  }

  void bankBird(Bird leadingBird) {
    // Compute amount to bank. Banking will apply a "thrust" at 90 degrees to the direction of the bird.
    // To compute which way to bank, take the cross prod of a vector from the point to bank towards with
    // the bird's direction vector. If positive, bank right, otherwise bank left.
    float distanceToTrailingSpot = getDistanceToPos(leadingBird.getTrailingSpot(), pos);
    PVector leadingBirdToThisBird = leadingBird.getTrailingSpot();
    leadingBirdToThisBird.sub(pos);    
    leadingBirdToThisBird.normalize();
    float rotRadians = radians(rot);
    PVector pointingDirection = new PVector(cos(rotRadians), sin(rotRadians));
    pointingDirection.normalize();
    PVector crossProduct = pointingDirection.cross(leadingBirdToThisBird).normalize();
    // Compute bank thrust relative to distance to trailing spot. at 90 to the pointing vector in the direction of the trailing spot.
    float bankFactor = distanceToTrailingSpot / 1000;
    if (crossProduct.z < 0) {
      // println("Go left, pd:[", pointingDirection.x, pointingDirection.y, "], lb:[", leadingBirdToThisBird.x, leadingBirdToThisBird.y, "], crossproduct.z", crossProduct.z);
      pointingDirection.rotate(radians(-90)).mult(bankFactor);
    } else {
      pointingDirection.rotate(radians(90)).mult(bankFactor);
      //println("Go right, pd:[", pointingDirection.x, pointingDirection.y, "], lb:[", leadingBirdToThisBird.x, leadingBirdToThisBird.y, "], crossproduct.z", crossProduct.z);
    }
    //println("Bank vector:[", pointingDirection.x, pointingDirection.y, "]");
    applyThrust(LATERALTHRUST);
    setThrustStrength(bankFactor, LATERALTHRUST);
    setThrustVector(pointingDirection);
  }


  void thrustOrAlignWithLeadingBird(Bird leadingBird) {
    // if bird trailing spot is nearby, try to align your direction with the leading bird. If it isn't, thrust
    // to the trailing spot
    float angleToTrailingSpot = getAngleToTrailingSpot(leadingBird);
    float distanceToTrailingSpot = getDistanceToPos(leadingBird.getTrailingSpot(), pos);
    if (distanceToTrailingSpot < 100) {
      // if we're close to trailing spot, try to line up with leading bird direction
      matchLeadingBirdDirection(leadingBird);
      bankBird(leadingBird);
    } else {  
      pointAtTrailingSpot(leadingBird);
      // compute thrust strength by merging together the diff btwn the lead bird's speed and this bird's speed with
      // the distance to the trailing spot target
      float speedDiff = max(0, leadingBird.getVelocity() - getVelocity());
      float thrustStrength = distanceToTrailingSpot / 400 + speedDiff;
      applyThrust(FORWARDTHRUST);
      setThrustStrength(thrustStrength, FORWARDTHRUST);
    }

  }


  void generateRandomTurn() {
    if (!isTurning()) {
      if (randomTurn == 0) {
        randomTurn = (float) Math.random() * 3 - 1.5;
      } 
    } else {
      randomTurn = rotChange * turnDecay;
      if (Math.abs(randomTurn) < 0.05) {
        randomTurn = 0; // cancel turning if tapered off, or not thrusting
      }
    }
    rotChange = randomTurn;
  }

  void updateAutoThrust() {
    if (thrustOn[FORWARDTHRUST]) {
      thrustTimer[FORWARDTHRUST]--;
      if (thrustTimer[FORWARDTHRUST] == 0) {
        cancelThrust(FORWARDTHRUST);
      }
    } else {
      float shouldStartThrust = (float) Math.random();
      if (shouldStartThrust > 0.985) {
        thrustTimer[FORWARDTHRUST] = (int) (Math.random() * 200);
        applyThrust(FORWARDTHRUST);  
        float randomStrength = (float) Math.random() * 0.05;
        setThrustStrength(randomStrength, FORWARDTHRUST);
      }
    }
  }

  void render() {
    drawBird(pos, rot, 1.0, birdColor, thrustOn[FORWARDTHRUST] );
   }
}
