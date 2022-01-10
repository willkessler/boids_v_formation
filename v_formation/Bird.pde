class Bird {
  
  PVector pos, vel, accel;
  float thrustConstant = 0.2;
  float maxSpeed, friction;
  color birdColor;
  int birdId;
  float birdWidth;
  float rot, rotChange, rotIncrement;
  float initialThrustStrength;
  float currentThrustStrength;
  float mass = 1.0;
  float randomTurn;
  float turnDecay = 0.95;
  int thrustDuration = 10;
  int thrustPauseDuration = 60;
  int thrustCounter = 0, thrustPauseCounter = 0;
  
  int thrustTimer;
  float leadingBirdRange = windowSize; // can track you over entire screen right now
  float leadingBirdAngleTolerance = 120;
  float birdSmartFactor = 1;
  boolean thrustOn;

  Bird(int id, float x, float y, color sColor) {
    accel = new PVector(0,0);
    vel = new PVector(0,0);
    pos = new PVector(x,y);
    maxSpeed = 5;
    birdWidth = 15;
    birdColor = sColor;
    rot = -90;
    rotChange = 0;
    rotIncrement = 3;
    thrustOn = false;
    initialThrustStrength = 0.009;
    currentThrustStrength = initialThrustStrength;
    birdId = id;
    friction = 0.98;
    randomTurn = 0;
    thrustTimer = 0;
  }
  
  void update() {
    
    if (thrustOn) {
      accel.x = cos(radians(rot)) * currentThrustStrength;
      accel.y = sin(radians(rot)) * currentThrustStrength;
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
    rotate(radians(rot+90));
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
    
    /*
    float rearRot = radians(rot + 135);
    PVector trailingSpot = new PVector(cos(rearRot), sin(rearRot));
    trailingSpot.mult(100.0);
    pushMatrix();
    translate(pos.x,pos.y);
    beginShape();
    vertex(trailingSpot.x-10,trailingSpot.y-10);
    vertex(trailingSpot.x+10,trailingSpot.y+10);
    endShape(CLOSE);
    beginShape();
    vertex(trailingSpot.x-10,trailingSpot.y+10);
    vertex(trailingSpot.x+10,trailingSpot.y-10);
    endShape(CLOSE);
    popMatrix();
    */
  }

  void moveLinear(float xMove, float yMove) {
    float scalar = 5;
    pos.x += xMove * scalar;
    pos.y += yMove * scalar;
  }

  void applyThrust() {
    if (!thrustOn && thrustCounter == 0 && thrustPauseCounter == 0) {
      thrustOn = true;
      thrustCounter = thrustDuration;
      return;
    }
    
    if (thrustCounter > 0) {
      thrustCounter--;
      if (thrustCounter == 0) {
        thrustPauseCounter = thrustPauseDuration;
        thrustOn = false;
      }
    }
  
    if (thrustPauseCounter > 0) {
      thrustPauseCounter--;
      if (thrustPauseCounter == 0) {
        thrustOn = true;
        thrustCounter = thrustDuration;
      }
    }

  }

  void setThrustStrength(float thrustStrength) {
    currentThrustStrength = thrustStrength;    
  }

  void cancelThrust() {
    thrustOn = false;
    thrustCounter = 0;
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

  PVector getTrailingSpot() {
    float rearRot = radians(rot + 135);
    PVector trailingSpot = new PVector(cos(rearRot), sin(rearRot));
    trailingSpot.mult(100.0);
    trailingSpot.add(pos);
    return trailingSpot;
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
    float rotRadians = radians(rot-90);
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
    
    println(printPVector("pointingDirection", pointingDirection),
            printPVector("trailingSpot", trailingSpot),
            "adjustmentAngle:", adjustmentAngle);

    stroke(birdColor);
    beginShape();
    //vertex(leadingBirdVelocityAdjustment.x * lineLen, leadingBirdVelocityAdjustment.y * lineLen);
    vertex(pos.x,pos.y);
    PVector ts2 = leadingBird.getTrailingSpot();
    vertex(ts2.x, ts2.y);
    endShape(CLOSE);

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

  void thrustOrAlignWithLeadingBird(Bird leadingBird) {
    // if bird trailing spot is nearby, try to align your direction with the leading bird. If it isn't, thrust
    // to the trailing spot
    PVector distanceToTrailingSpotVec = leadingBird.getTrailingSpot();
    distanceToTrailingSpotVec.sub(pos);
    float distanceToTrailingSpot = distanceToTrailingSpotVec.mag();
    float thrustStrength = distanceToTrailingSpot / 400;
    if (distanceToTrailingSpot > 1) {
      applyThrust();
      setThrustStrength(thrustStrength);
    } else {
      cancelThrust();
    }
  }


  void generateRandomTurn() {
    if (!isTurning()) {
      if (randomTurn == 0 && !thrustOn) {
        randomTurn = (float) Math.random() * 3 - 1.5;
      } 
    } else {
      randomTurn = rotChange * turnDecay;
      if (Math.abs(randomTurn) < 0.01 || !thrustOn) {
        randomTurn = 0; // cancel turning if tapered off, or not thrusting
      }
    }
    rotChange = randomTurn;
  }

  void updateAutoThrust() {
    if (thrustOn) {
      thrustTimer--;
      if (thrustTimer == 0) {
        cancelThrust();
      }
    } else {
      float shouldStartThrust = (float) Math.random();
      if (shouldStartThrust > 0.99) {
        thrustTimer = (int) (Math.random() * 250);
        applyThrust();  
      }
    }
  }

  void render() {
    drawBird(pos, rot, 1.0, birdColor, thrustOn);
   }
}
