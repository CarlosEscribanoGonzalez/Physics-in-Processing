//Spring endings //<>//
PVector[] ma, mb;   // Spring endings in global reference system   
PVector[] mblocal;  // Position of mb spring endings in the local reference system of the box 

//Box settings
int boxSide = 80;        // Side of the square box
float boxMass  = 1.0;    // Total mass of the box
float boxInertia = 1/6.0 * boxMass * boxSide * boxSide;  // Inertia for a square shape 

//Cannon and bullet
int cannonX = 360;                 // X position of the cannon
PVector bulletPos, bulletPosNew;   // Position of the bullet in times t-1 (bulletPos) and t (bulletPosNew)
int bulletVel = -1200;             // Current velocity of the bullet
float bulletMass = 0.06;           // Mass of the bullet
float restitution = 0.5;           // Coefficient of restitution to compute the impulse 

//Box kinematics
PVector boxPos, boxVel;            // Position and velocity of the center of mass of the box
float boxRot, boxOmega;            // Rotation and angular velocity of the box

//Spring settings
float l0 = 210;                   // Rest length of springs
float k = 1;                      // Stiffness of springs

//Simulation settings
int fps = 60;                      // Refresh rate
int substeps = 5;                  // Number of substeps per rendering frame
float h = 1.0 / (fps * substeps);  // Time step of the animation
boolean fire = false;              // If true, the bullet is being fired
boolean pause = false;             // If true, the animation and the bullet are paused

/////////////////////////////////////////
// Function for the computation of spring force
// inputs: Positions of the end points of the spring, rest length, and stiffness
// outputs: Force on end point 'a'
/////////////////////////////////////////

PVector SpringForce(PVector a, PVector b, float L0, float k)
{
  return PVector.mult(PVector.sub(a, b), k * (L0 / PVector.dist(a, b) - 1));
}

/////////////////////////////////////////
// Function for the computation of a ray-segment intersection test
// inputs: The ray is defined by the positions a0 (t=0) and a1 (t=1).
// The segment is defined by the positions b and c,
// with the outward normal obtained by rotating c-b 90deg around the z axis.
// outputs: If there is no collision, the method returns -1.0
// If there is a collision, the method returns the collision time
// and writes the collision position and outward normal
/////////////////////////////////////////

float CollisionTest(PVector a0, PVector a1, PVector b, PVector c, PVector p, PVector N)
{
  float t = -1.0;
  
  PVector da = PVector.sub(a1, a0);
  float den = c.x * da.y - c.y * da.x + da.x * b.y - da.y * b.x;
  float num = c.x * b.y - b.x * c.y + c.y * a0.x - c.x * a0.y + a0.y * b.x - a0.x * b.y;
  if (den == 0)
  {
    return -1.0;
  }
  
  t = num / den;
  if (t < 0.0 || t > 1.0)
  {
    return -1.0;
  }
  
  p.set(PVector.add(a0, PVector.mult(da, t)));
  
  float pdot = PVector.dot(PVector.sub(c, b), PVector.sub(p, b));
  if (pdot < 0.0 || pdot > (PVector.sub(c, b)).magSq())
  {
    return -1.0;
  }
  
  N.set(PVector.sub(c, b));
  N.normalize(); //<>//
  N.rotate(PI / 2.0);
  
  return t;
}

/////////////////////////////////////////
// Function for the computation of an impulse between particle 'a' and rigid body 'b'.
// inputs: 'normal' pointing from body 'b' toward particle 'a', velocity 'va' of the particle,
// position 'posb' of the collision relative to the center of mass of 'b',
// linear velocity 'vb' and angular velocity 'wb' of body 'b',
// mass 'ma' of the particle, mass 'mb' of the body, inertia 'inertiab' of the body,
// coefficient of restitution 'rest'.
// outputs: The impulse on particle 'a'.
/////////////////////////////////////////

float CollisionImpulse(PVector normal, PVector va, PVector posb, PVector vb, float wb, float ma, float mb, float inertiab, float rest)
{
  PVector wbvec = new PVector (0, 0, wb);
  
  PVector vq = PVector.add(vb, wbvec.cross(posb));
  
  PVector vrel = PVector.sub(va, vq);
  
  PVector sa = PVector.mult(normal, 1 / ma);
  
  PVector sb = PVector.add(PVector.mult(normal, 1 / mb), PVector.mult((posb.cross(normal)).cross(posb), 1/inertiab));
  
  return -(1 + rest)*PVector.dot(normal,vrel) / PVector.dot(normal, PVector.add(sa,sb));
}

void setup()
{
  size(720, 480);
  frameRate(fps);

  //Initialize box
  boxPos = new PVector(width/2, height/2);
  boxRot = 45*PI/180;
  boxVel = new PVector(0.0, 0.0);
  boxOmega = 0;
  
  //Initialize bullet
  bulletPos = new PVector(0.0, 0.0);
  bulletPosNew = new PVector(0.0, 0.0);

  //Initialize springs
  ma = new PVector[4];
  ma[0] = new PVector(10, 10);
  ma[1] = new PVector(width-10, 10);
  ma[2] = new PVector(width-10, height-10);
  ma[3] = new PVector(10, height-10);
  mblocal = new PVector[4];
  mblocal[0] = new PVector(-boxSide/2, -boxSide/2);
  mblocal[1] = new PVector(boxSide/2, -boxSide/2);
  mblocal[2] = new PVector(boxSide/2, boxSide/2);
  mblocal[3] = new PVector(-boxSide/2, boxSide/2);
  mb = new PVector[4];
  for (int i = 0; i < 4; i++)
  {
    mb[i] = new PVector(0, 0);
    mb[i].set(mblocal[i]);
    mb[i].rotate(boxRot);
    mb[i].add(boxPos);
  }
}

void draw()
{
  //Update cannon
  if (keyPressed)
  {
    if (key == CODED)
    {
      if (keyCode == LEFT)
      {
        cannonX = cannonX - 2;
        if (cannonX < 20)
        {
          cannonX = 20;
        }
      }
      else if (keyCode == RIGHT)
      {
        cannonX = cannonX + 2;
        if (cannonX > width-20)
        {
          cannonX = width-20;
        }
      }
    }
  }
  
  //Update bullet
  if (keyPressed && (pause || !fire))
  {
    if (key == ENTER || key == RETURN)
    {
      fire = true;
      pause = false;
      bulletPosNew.set(cannonX, height);
    }
  }
  if (fire && !pause)
  {
    bulletPos.set(bulletPosNew);
    bulletPosNew.y = bulletPosNew.y + h * substeps * bulletVel;
    if(bulletPosNew.y < 0)
    {
      fire = false;
    }
  }
      
  ///////////////////////////////////////////////////
  // COLLISION-FREE DYNAMICS
  ///////////////////////////////////////////////////
  
  for (int step = 0; step < substeps && !pause; step++)
  {
    //Initialize force and torque
    PVector force = new PVector(0, boxMass * 9.8);
    float torque = 0;
    
    //Accumulate spring forces
    for (int i=0; i<4; i++)
    {
      PVector springForce = SpringForce(mb[i], ma[i], l0, k);
      force.add(springForce);
      torque += ((PVector.sub(mb[i], boxPos)).cross(springForce)).z;
    }
    
    //Integrate velocities
    boxVel.add(PVector.mult(force, h / boxMass));
    boxOmega += h / boxInertia * torque;
  
    //Integrate positions
    boxPos.add(PVector.mult(boxVel, h));
    boxRot += h * boxOmega;
  
    //Recompute spring end points
    for (int i = 0; i < 4; i++)
    {
      mb[i].set(mblocal[i]);
      mb[i].rotate(boxRot);
      mb[i].add(boxPos);
    }
  }
  
  ///////////////////////////////////////////////////
  // COLLISION HANDLING
  ///////////////////////////////////////////////////
  
  if (fire)
  {
    // COLLISION DETECTION
    
    // Initialize contact variables (collision flag, time, position, normal)
    boolean collision = false;
    float colTime = 10.0; 
    PVector colPos = new PVector();
    PVector colNormal = new PVector();
    
    // Loop through all sides of the cube and execute ray-segment intersections
    for (int i=0; i<4; i++)
    {
      PVector thisPos = new PVector();
      PVector thisNormal = new PVector();

      float thisTime = CollisionTest(bulletPos, bulletPosNew, mb[(i+1)%4], mb[i], thisPos, thisNormal);
      if (thisTime != -1.0 && thisTime < colTime)
      {
        collision = true;
        colTime = thisTime;
        colPos.set(thisPos);
        colNormal.set(thisNormal);
      }
    }

    // COLLISION RESPONSE
    if (collision)
    {
      fire = false;
      float lambda = CollisionImpulse (colNormal, new PVector(0, bulletVel), PVector.sub(colPos, boxPos), boxVel, boxOmega, bulletMass, boxMass, boxInertia, restitution);
      PVector impulso = PVector.mult(colNormal, -lambda);
      boxVel.add(PVector.mult(impulso, 1.0/boxMass));
      boxOmega += (PVector.sub(colPos, boxPos).cross(impulso)).z /boxInertia;
    }
  }
  
  ///////////////////////////////////////////////////
  // RENDERING
  ///////////////////////////////////////////////////

  background(255);

  //Draw cannon
  fill(255, 0, 0);
  rect(cannonX-10, height-20, 20, 40);
  
  //Draw spring endings
  fill(0);
  for (int i = 0; i < 4; i++)
  {
    rect(ma[i].x - 10, ma[i].y - 10, 20, 20);
  }

  //Draw box
  fill(0, 0, 255);
  pushMatrix();
  translate(boxPos.x, boxPos.y);
  rotate(boxRot);
  rect(-boxSide/2, -boxSide/2, boxSide, boxSide);
  popMatrix();
  
  //Draw springs
  stroke(0);
  for (int i = 0; i < 4; i++)
  {
    line(ma[i].x, ma[i].y, mb[i].x, mb[i].y);
  }

  //Draw bullet
  if (fire)
  {
    fill(0, 255, 0);
    ellipse(bulletPosNew.x, bulletPosNew.y, 10, 10);
  }
  
}
