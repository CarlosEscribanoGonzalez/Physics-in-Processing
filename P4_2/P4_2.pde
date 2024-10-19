//Scene parameters and refresh rate
int ground = 400;          // location of the ground
int fps = 30;              // refresh rate

//Springs defined using node indices
int[] springNodeA, springNodeB;  // arrays of indices of spring end points 'a' and 'b'

//Spring and node parameters
int nNodes = 4;                  // number of nodes
int nSprings = 5;                // number of springs
float[] springLength0;           // array of spring rest lengths
float springStiffness = 1000;    // spring stiffness
float springDamping = 10;        // spring damping
float nodeMass = 1;              // node mass
float nodeDamping = 1;           // node damping
float contactStiffness = 1000;   // penalty contact stiffness
float mouseStiffness = 200;      // stiffness of mouse spring

//Node state
PVector[] nodePos, nodeVel;      // arrays of node positions and velocities

//Simulation settings
int substeps = 5;                  // number of substeps per rendering frame
float h = 1.0 / (fps * substeps);  // time step of the animation
float g = 500;                     // gravity constant, in pixels/sec^2

//Wall parameters
int nWalls = 4;                    // number of walls
PVector[] wallNormal, wallPoint;   // arrays of wall normals and wall points

//Mouse parameters
int mouseId;              // id of node grabbed by the mouse
PVector mousePos;         // position of the mouse

/////////////////////////////////////////
// Function for the computation of spring force
// inputs: Positions of the end points of the spring, rest length, and stiffness
// outputs: Force on end point 'xa'
/////////////////////////////////////////

PVector SpringElasticForce(PVector xa, PVector xb, float L0, float k)
{
  float L = PVector.dist(xa, xb);
  return PVector.mult(PVector.sub(xa, xb), - k * (L - L0) / L);
}

/////////////////////////////////////////
// Function for the computation of spring force, for a zero-rest-length spring
// inputs: Positions of the end points of the spring, and stiffness
// outputs: Force on end point 'xa'
/////////////////////////////////////////

PVector SpringElasticForceZeroLength(PVector xa, PVector xb, float k)
{
  return new PVector(0, 0);
}

/////////////////////////////////////////
// Function for the computation of spring damping
// inputs: Positions and velocities of the end points of the spring, and damping
// outputs: Force on end point 'xa'
/////////////////////////////////////////

PVector SpringDampingForce(PVector xa, PVector xb, PVector va, PVector vb, float d)
{
  PVector u = PVector.sub(xa, xb);
  u.normalize();
  return PVector.mult(u, -d * PVector.dot(u, PVector.sub(va, vb)));
}

/////////////////////////////////////////
// Function for the computation of penalty force
// inputs: Normal, penetration depth, and stiffness
// outputs: Force on the node
/////////////////////////////////////////

PVector ContactPenaltyForce(PVector n, float delta, float k)
{
  return new PVector(0, 0);
}

/////////////////////////////////////////
// This function is called at initialization
/////////////////////////////////////////

void setup()
{
  size(720, 480);
  frameRate(fps);
  
  //Initialize nodes and springs
  springNodeA = new int[nSprings];
  springNodeB = new int[nSprings];
  springNodeA[0] = 0; springNodeB[0] = 1;
  springNodeA[1] = 1; springNodeB[1] = 2;
  springNodeA[2] = 2; springNodeB[2] = 3;
  springNodeA[3] = 3; springNodeB[3] = 0;
  springNodeA[4] = 2; springNodeB[4] = 0;
  
  nodePos = new PVector[nNodes];
  nodePos[0] = new PVector(320, 200);
  nodePos[1] = new PVector(400, 200);
  nodePos[2] = new PVector(400, 280);
  nodePos[3] = new PVector(320, 280);
  nodeVel = new PVector[nNodes];
  for (int i=0; i<nNodes; i++)
  {
    nodeVel[i] = new PVector(0, 0);
  }
  
  springLength0 = new float[nSprings];
  for (int i=0; i<nSprings; i++)
  {
    springLength0[i] = PVector.dist(nodePos[springNodeB[i]], nodePos[springNodeA[i]]);
  }
  
  //Initialize wall parameters
  wallNormal = new PVector[nWalls];
  wallNormal[0] = new PVector(0, -1);
  wallNormal[1] = new PVector(-1, 0);
  wallNormal[2] = new PVector(0, 1);
  wallNormal[3] = new PVector(1, 0);
  wallPoint = new PVector[nWalls];
  wallPoint[0] = new PVector(0, ground);
  wallPoint[1] = new PVector(width, 0);
  wallPoint[2] = new PVector(0, 0);
  wallPoint[3] = new PVector(0, 0);
  
  //Initialize mouse parameters
  mouseId = -1;
  mousePos = new PVector(0, 0);
}

/////////////////////////////////////////
// This function is called when the mouse is pressed
/////////////////////////////////////////

void mousePressed()
{
  mousePos.set(mouseX, mouseY);
  for (int i=0; i<nNodes; i++)
  {
    if (PVector.dist(mousePos, nodePos[i]) < 20)
    {
      mouseId = i;
      break;
    }
  }
}

/////////////////////////////////////////
// This function is called when the mouse is released
/////////////////////////////////////////

void mouseReleased()
{
  mouseId = -1;
}

/////////////////////////////////////////
// This function is called when the mouse moves while being pressed
/////////////////////////////////////////

void mouseDragged()
{
  mousePos.set(mouseX, mouseY);
}

/////////////////////////////////////////
// This function is called every time that the display is refreshed
/////////////////////////////////////////

void draw()
{
  background(255);

  for (int step = 0; step < substeps; step++)
  {
    // Initialize node forces with gravity
    PVector[] nodeForce = new PVector[nNodes];
    for (int i=0; i<nNodes; i++)
    {
      nodeForce[i] = new PVector(0, nodeMass * g);
    }
  
    // Add spring and damping forces
  for (int i = 0; i < nNodes; i++)
  {
    PVector force = SpringElasticForce(nodePos[springNodeA[i]], nodePos[springNodeB[i]], springLength0[i], springStiffness);
    force.add(SpringDampingForce(nodePos[springNodeA[i]], nodePos[springNodeB[i]], nodeVel[springNodeA[i]], nodeVel[springNodeB[i]], springDamping));
    nodeForce[springNodeA[i]].add(force);
    nodeForce[springNodeB[i]].sub(force); 
  }
    // Contact handling: loop over nodes and walls

  
  
  
  
    // Mouse interaction
    if (mouseId != -1)
    {

    }
  
    // Numerical integration
    for (int i = 0; i < nNodes; i++)
    {
      nodeVel[i].add(PVector.mult(nodeForce[i], h / nodeMass));
      nodePos[i].add(PVector.mult(nodeVel[i], h));
    }
  }
  
  //Draw springs
  stroke(50);
  for (int i=0; i<nSprings; i++)
  {
    line(nodePos[springNodeA[i]].x, nodePos[springNodeA[i]].y, nodePos[springNodeB[i]].x, nodePos[springNodeB[i]].y);
  }

  //Draw nodes
  fill(50, 50, 200);
  for (int i=0; i<nNodes; i++)
  {
    ellipse(nodePos[i].x, nodePos[i].y, 10, 10);
  }
  
  //Draw ground
  fill(150, 50, 50);
  rect(0, ground, width, height - ground);
}
