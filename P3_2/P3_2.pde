//Window size and refresh rate
int wide = 720;            // width of the window, in pixels 
int high = 480;            // height of the window, in pixels
int fps = 30;              // refresh frame rate

//Size and position of the wall; size, position and motion of the gap in the wall
int wallX = 3 * wide / 4;  // position of the wall on the X axis
int wallW = 20;            // width of the wall
int gapY = 0;              // (initial) position of the gap on the Y axis
int gapH = 40;             // height of the gap
int gapVY = high / 4;      // velocity of the gap

//Orientation and velocity of the cannon
float cannonTheta = 0;               // angle of the cannon, in radians
float cannonDeltaTheta = PI / 100.0; // change of angle per step
int cannonV = 0;                     // velocity of the ball as it leaves the cannon
int cannonMaxV = 1500;               // maximum velocity of the ball as it leaves the cannon
int cannonDeltaV = 30;               // change of velocity per step

//Variables for the animation of the cannonball
boolean shooting = false;         // toggle animation on/off
int substeps = 16;                 // number of substeps per rendering frame
float h = 1.0 / (fps * substeps); // time step of the animation
float ballX;                      // position of the ball on the X axis //<>//
float ballY;                      // position of the ball on the Y axis
float ballVX;                     // velocity of the ball on the X axis 
float ballVY;                     // velocity of the ball on the Y axis
float ballG = 1000;               // gravity constant, in pixels/sec^2

//Other integration methods
float ballXAnalytical, ballXExplicit, ballXMidpoint, ballXVerlet, ballXVerletOld;
float ballVXAnalytical, ballVXExplicit, ballVXMidpoint, ballVXVerlet;
float ballYAnalytical, ballYExplicit, ballYMidpoint, ballYVerlet, ballYVerletOld;
float ballVYAnalytical, ballVYExplicit, ballVYMidpoint, ballVYVerlet;
boolean firstStep = true;

/////////////////////////////////////////
// This function is called at initialization
/////////////////////////////////////////

void setup()
{
  size(720, 480);
  frameRate(fps);
}

/////////////////////////////////////////
// This function is called every time that the display is refreshed
/////////////////////////////////////////

void draw()
{
  background(255);

  //Update the gap
  gapY = gapY + int(gapVY / fps);
  if (gapY >= high - gapH)
  {
    gapY = high - gapH;
    gapVY = -gapVY;
  }
  else if (gapY <= 0)
  {
    gapY = 0;
    gapVY = -gapVY;
  }
  
  //Draw the wall and the gap
  fill(0);
  rect(wallX, 0, wallW, gapY);
  rect(wallX, gapY + gapH, wallW, high - gapH - gapY);
  fill(255);
  rect(wallX, gapY, wallW, gapH);
    
  //Update the cannon
  if (keyPressed)
  {
    if (key == CODED)
    {
      if (keyCode == UP)
      {
        cannonTheta = cannonTheta + cannonDeltaTheta;
        if (cannonTheta > PI/2)
        {
          cannonTheta = PI/2;
        }
      }
      else if (keyCode == DOWN)
      {
        cannonTheta = cannonTheta - cannonDeltaTheta;
        if (cannonTheta < 0)
        {
          cannonTheta = 0;
        }
      }
      else if (keyCode == LEFT)
      {
        cannonV = cannonV - cannonDeltaV;
        if (cannonV < 0)
        {
          cannonV = 0;
        }
      }
      else if (keyCode == RIGHT)
      {
        cannonV = cannonV + cannonDeltaV;
        if (cannonV > cannonMaxV)
        {
          cannonV = cannonMaxV;
        }
      }
    }
  }

  //Draw the cannon
  fill(255 * cannonV / cannonMaxV, 0, 255 * (cannonMaxV - cannonV) / cannonMaxV);
  pushMatrix();
  translate(0, high-7.5);
  rotate(-cannonTheta);
  rect(-40, -15, 80, 30);
  popMatrix();
  
  // Draw velocity
  fill(0, 0, 0);
  pushMatrix();
  translate(0, high-7.5); //para que el giro (mas desplazamiento) de ambos rectángulos vaya sincronizado y se toquen las bases.
  translate(cos(-cannonTheta)*40, -sin(cannonTheta)*40); // Coseno es la proyección sobre X, Seno la proyección sobre Y, habos quedan multiplicado por la hipotenusa que mide 40
  rotate(-cannonTheta);
  rect(0, -3.0, 80*cannonV/cannonMaxV, 5); //el offset local de 3 lo centra un poco más
  popMatrix();
  
  //Start animation of the cannonball
  if (keyPressed && shooting == false)
  {
    if (key == ENTER || key == RETURN)
    {
      shooting = true;
      
      //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
      //INITIALIZATION
      //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
      ballX = 0;
      ballY = high;
      ballVX = cannonV * cos(cannonTheta);
      ballVY = - cannonV * sin(cannonTheta);
      
      ballXAnalytical = ballXExplicit = ballXMidpoint = ballXVerlet = ballX;
      ballVXAnalytical = ballVXExplicit = ballVXMidpoint = ballVXVerlet = ballVX;
      ballYAnalytical = ballYExplicit = ballYMidpoint = ballYVerlet = ballY;
      ballVYAnalytical = ballVYExplicit = ballVYMidpoint = ballVYVerlet = ballVY;
      firstStep = true;
    } //<>//
  }
    
  //Animation
  if (shooting)
  { //<>//
    for (int i = 0; i < substeps; i++)
    {
      //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
      //NUMERICAL INTEGRATION
      //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
      
      //Euler simpléctico:
      ballVY += h * ballG;
      ballX += h * ballVX;
      ballY += h * ballVY;
      
      //Solución analítica:
      ballXAnalytical += ballVXAnalytical * h;
      ballYAnalytical += ballVYAnalytical * h + h * h * ballG / 2;
      ballVYAnalytical += ballG * h;
      
      //Euler explícito:
      ballXExplicit += ballVXExplicit * h;
      ballYExplicit += ballVYExplicit * h;
      ballVYExplicit += h * ballG;
      
      //Midpoint:
      ballXMidpoint += ballVXMidpoint * h;
      float aux = ballVYMidpoint + 0.5 * h * ballG;
      ballYMidpoint += h * aux;
      ballVYMidpoint += h * ballG;
      
      //Verlet:
      if(firstStep)
      {
        ballXVerletOld = ballXVerlet;
        ballXVerlet += ballVXVerlet * h;
        ballYVerletOld = ballYVerlet;
        ballYVerlet += ballVYVerlet * h + 0.5 * h * h * ballG;
        firstStep = false;
      } 
      else 
      {
        aux = ballXVerlet;
        ballXVerlet = 2 * ballXVerlet * ballXVerletOld;
        ballXVerletOld = aux;
        aux = ballYVerlet;
        ballYVerlet = 2 * ballYVerlet - ballYVerletOld + h*h*ballG;
        ballYVerletOld = aux;
      }
    }
      
      //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
      //COLLISION DETECTION
      //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
      if (ballX > wallX && ballX < (wallX + wallW) && ballY > 0 && ((ballY < gapY)||(ballY > (gapY + gapH))) )
      {
        shooting = false;
      }
  
      //The animation ends if the ball leaves the screen
      if (ballY > high || ballY < 0 || ballX > wide)
      {
        shooting = false;
      }
  
    }
    //Draw the ball
    fill(0, 255, 0);
    ellipse(ballX, ballY, 8, 8);

    fill(255, 0, 0);
    ellipse(ballXAnalytical, ballYAnalytical, 8, 8);

    fill(0, 0, 255);
    ellipse(ballXExplicit, ballYExplicit, 8, 8);

    fill(255, 255, 0);
    ellipse(ballXMidpoint, ballYMidpoint, 8, 8);

    fill(0, 255, 255);
    ellipse(ballXVerlet, ballYVerlet, 8, 8);
}
