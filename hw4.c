/*
 *  Lorenz Attractor Visualization
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>
#ifdef USEGLEW
#include <GL/glew.h>
#endif
#define GL_GLEXT_PROTOTYPES
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif
//  Default resolution
//  For Retina displays compile with -DRES=2
#ifndef RES
#define RES 1
#endif

//-----------------------------------------------------------
// Struct declarations
//-----------------------------------------------------------
// Define struct for RGB color value
typedef struct Color
{
   float r;
   float g;
   float b;
} Color;

typedef struct Point
{
   double x;
   double y;
   double z;
} Point;

typedef struct Angle
{
   double psi; // Rotation about the X axis in degrees
   double ph;  // Rotation about the Y axis in degrees
   double th;  // Rotation about the Z axis in degrees
} Angle;

// Define struct for view parameters
typedef struct ViewParams
{
   Point pos;
   Angle angle;
} ViewParams;

typedef struct Cylinder
{
   Point base;  // Base position of cylinder
   Angle angle; // Rotation angles
   double r;    // Radius of cylinder
   double h;    // Height of cylinder
} Cylinder;

typedef struct Torus{
   Point center; // Center position of torus
   Point axis;  // Axis vector of torus
   double rMajor; // Major radius (distance from center to tube center)
   double rMinor; // Minor radius (radius of the tube)
} Torus;

typedef struct EllipseStruct{
   Point center; // Center position of ellipse
   Point axis;  // Axis vector of ellipse
   double rMajor; // Major radius
   double rMinor; // Minor radius
} EllipseStruct;

double th; // Rotation about the Z axis in degrees


//-----------------------------------------------------------
// Global variables
//-----------------------------------------------------------

Color color = {1.0, 1.0, 1.0}; // Default color white
ViewParams view = {
    (Point){0.0, 0.0, 0.0},
    (Angle){NAN, 0.0, 0.0},
}; // Default to back a bit so the origin is visible

int m = 0; // perspective mode switcher

/*
 *  Check for OpenGL errors
 */
void ErrCheck(const char *where)
{
   int err = glGetError();
   if (err)
      fprintf(stderr, "ERROR: %s [%s]\n", gluErrorString(err), where);
}

/*
 *  Print message to stderr and exit
 */
void Fatal(const char *format, ...)
{
   va_list args;
   va_start(args, format);
   vfprintf(stderr, format, args);
   va_end(args);
   exit(1);
}

/*
 *  Convenience routine to output raster text
 *  Use VARARGS to make this more flexible
 */
#define LEN 8192 //  Maximum length of text string
void Print(const char *format, ...)
{
   char buf[LEN];
   char *ch = buf;
   va_list args;
   //  Turn the parameters into a character string
   va_start(args, format);
   vsnprintf(buf, LEN, format, args);
   va_end(args);
   //  Display the characters one at a time at the current raster position
   while (*ch)
      glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, *ch++);
}

void SetCamera(ViewParams view)
{
   //  Set view angle
   glLoadIdentity();
   glRotated(view.angle.ph, 1.0, 0.0, 0.0);  // Apply pitch rotation first
   glRotated(view.angle.th, 0.0, 1.0, 0.0);  // Apply yaw rotation second
   glTranslated(view.pos.x, view.pos.y, view.pos.z);  // Move camera to position

   glWindowPos2i(5, 45);
   Print("Camera Position: x=%.2f, y=%.2f, z=%.2f", view.pos.x, view.pos.y, view.pos.z);
   glWindowPos2i(5, 25);
   Print("Camera Angle: yaw=%.1f, pitch=%.1f", view.angle.th, view.angle.ph);
}


// Cos and Sin in degrees
double Cos(double theta)
{
   return cos(theta * 3.1415926535 / 180.0);
}

double Sin(double theta)
{
   return sin(theta * 3.1415926535 / 180.0);
}

double Tan(double theta)
{
   return tan(theta * 3.1415926535 / 180.0);
}

// Compute angles for aligning the khat vector with a given direction vector
Angle computeAngles(Point dir)
{
   Angle angles;
   // Compute the angles
   angles.ph = acos(dir.z / sqrt(dir.x * dir.x + dir.y * dir.y + dir.z * dir.z)) * 180.0 / 3.1415926535;
   angles.th = atan2(dir.y, dir.x) * 180.0 / 3.1415926535;
   angles.psi = 0.0; // No roll angle needed for this application
   return angles;
}

// Convert HSV color to RGB color
// Github Copilot generated this function - I didn't write or verify it myself
Color hsv2rgb( float h, float s, float v )
{
   Color rgb;
   int i;
   float f, p, q, t;

   if( s == 0 ) { // Achromatic (grey)
      rgb.r = rgb.g = rgb.b = v;
      return rgb;
   }

   h /= 60;            // sector 0 to 5
   i = floor( h );
   f = h - i;          // factorial part of h
   p = v * ( 1 - s );
   q = v * ( 1 - s * f );
   t = v * ( 1 - s * ( 1 - f ) );

   switch( i ) {
      case 0:
         rgb.r = v;
         rgb.g = t;
         rgb.b = p;
         break;
      case 1:
         rgb.r = q;
         rgb.g = v;
         rgb.b = p;
         break;
      case 2:
         rgb.r = p;
         rgb.g = v;
         rgb.b = t;
         break;
      case 3:
         rgb.r = p;
         rgb.g = q;
         rgb.b = v;
         break;
      case 4:
         rgb.r = t;
         rgb.g = p;
         rgb.b = v;
         break;
      default:
         rgb.r = v;
         rgb.g = p;
         rgb.b = q;
         break;
   }
   return rgb; 
}

// Helper function to compute color based on global distance from origin
Color computeDistanceColor(double x, double y, double z)
{
   double distance = sqrt(x*x + y*y + z*z);
   // Scale distance to hue range (0-360 degrees)
   // Adjust the scaling factor as needed for your scene
   double hue = fmod(distance * 500.0, 360.0);  // Scale and wrap around
   return hsv2rgb(hue, 0.8, 1.0);  // High saturation, full brightness
}

// Lets you specify the center of the two end points of the cylinder and draws it with the associated radius
// Enhanced version with global coordinate coloring
void drawCylinder(Point p1, Point p2, double r)
{
   // Compute the direction vector from p1 to p2
   Point dir = {
       p2.x - p1.x,
       p2.y - p1.y,
       p2.z - p1.z,
   };
   // Compute the length of the cylinder
   double length = sqrt(dir.x * dir.x + dir.y * dir.y + dir.z * dir.z);

   // Compute the angles for aligning the cylinder with the direction vector
   Angle angles = computeAngles(dir);

   // Save current transformation matrix
   glPushMatrix();

   // Set the origin to be the base of the cylinder
   glTranslated(p1.x, p1.y, p1.z);

   // Apply rotations
   glRotated(angles.th, 0.0, 0.0, 1.0);  // Rotate about Z axis
   glRotated(angles.ph, 0.0, 1.0, 0.0);  // Rotate about Y axis
   glRotated(angles.psi, 1.0, 0.0, 0.0); // Rotate about X axis

   // Body of the cylinder with global coordinate coloring
   const int deltaDegree = 15; // degrees per segment
   glBegin(GL_QUAD_STRIP);
   for (int degree = 0; degree <= 360; degree += deltaDegree)
   {
      double x = r * Cos(degree);
      double y = r * Sin(degree);
      
      // Bottom vertex - compute global position
      double globalX1 = p1.x + x * Cos(angles.th) - y * Sin(angles.th);
      double globalY1 = p1.y + x * Sin(angles.th) + y * Cos(angles.th);
      double globalZ1 = p1.z;
      Color color1 = computeDistanceColor(globalX1, globalY1, globalZ1);
      glColor3f(color1.r, color1.g, color1.b);
      glVertex3d(x, y, 0.0);
      
      // Top vertex - compute global position  
      double globalX2 = p2.x + x * Cos(angles.th) - y * Sin(angles.th);
      double globalY2 = p2.y + x * Sin(angles.th) + y * Cos(angles.th);
      double globalZ2 = p2.z;
      Color color2 = computeDistanceColor(globalX2, globalY2, globalZ2);
      glColor3f(color2.r, color2.g, color2.b);
      glVertex3d(x, y, length);
   }
   glEnd();

   // Top circle
   glBegin(GL_TRIANGLE_FAN);
   // Center vertex
   Color centerColor2 = computeDistanceColor(p2.x, p2.y, p2.z);
   glColor3f(centerColor2.r, centerColor2.g, centerColor2.b);
   glVertex3d(0.0, 0.0, length);
   
   for (int degree = 0; degree <= 360; degree += deltaDegree)
   {
      double x = r * Cos(degree);
      double y = r * Sin(degree);
      double globalX = p2.x + x * Cos(angles.th) - y * Sin(angles.th);
      double globalY = p2.y + x * Sin(angles.th) + y * Cos(angles.th);
      double globalZ = p2.z;
      Color color = computeDistanceColor(globalX, globalY, globalZ);
      glColor3f(color.r, color.g, color.b);
      glVertex3d(x, y, length);
   }
   glEnd();

   // Bottom circle
   glBegin(GL_TRIANGLE_FAN);
   // Center vertex
   Color centerColor1 = computeDistanceColor(p1.x, p1.y, p1.z);
   glColor3f(centerColor1.r, centerColor1.g, centerColor1.b);
   glVertex3d(0.0, 0.0, 0.0);
   
   for (int degree = 0; degree <= 360; degree += deltaDegree)
   {
      double x = r * Cos(degree);
      double y = r * Sin(degree);
      double globalX = p1.x + x * Cos(angles.th) - y * Sin(angles.th);
      double globalY = p1.y + x * Sin(angles.th) + y * Cos(angles.th);
      double globalZ = p1.z;
      Color color = computeDistanceColor(globalX, globalY, globalZ);
      glColor3f(color.r, color.g, color.b);
      glVertex3d(x, y, 0.0);
   }
   glEnd();

   // Restore color to white
   glColor3f(1.0, 1.0, 1.0);

   // Restore transformation matrix
   glPopMatrix();
}

void drawTorus( Torus t )
{
   // Save current transformation matrix
   glPushMatrix();

   // Set the origin to be the center of the torus
   glTranslated(t.center.x, t.center.y, t.center.z);

   // Compute the angles for aligning the torus with the axis vector
   Angle angles = computeAngles(t.axis);

   // Apply rotations
   glRotated(angles.th, 0.0, 0.0, 1.0);  // Rotate about Z axis
   glRotated(angles.ph, 0.0, 1.0, 0.0);  // Rotate about Y axis
   glRotated(angles.psi, 1.0, 0.0, 0.0); // Rotate about X axis

   // Color based on global position of torus center
   Color torusColor = computeDistanceColor(t.center.x, t.center.y, t.center.z);
   glColor3f(torusColor.r, torusColor.g, torusColor.b);

   // Draw the torus using GLUT function
   // glutSolidTorus(t.rMinor, t.rMajor, 30, 30); // TODO WRITE MY OWN
   double deltaDegree = 15; // degrees per segment
   for( double theta = 0; theta <= 360; theta += deltaDegree )
   {
   glBegin(GL_QUAD_STRIP);

      for (double phi = 0; phi <= 360; phi += deltaDegree )
      {
         // Equation for torus
         double x1 = (t.rMajor + t.rMinor * Cos(theta)) * Cos(phi);
         double y1 = (t.rMajor + t.rMinor * Cos(theta)) * Sin(phi);
         double z1 = t.rMinor * Sin(theta);
         double x2 = (t.rMajor + t.rMinor * Cos(theta+deltaDegree)) * Cos(phi + deltaDegree);
         double y2 = (t.rMajor + t.rMinor * Cos(theta+deltaDegree)) * Sin(phi + deltaDegree);
         double z2 = t.rMinor * Sin(theta+deltaDegree);

         // Compute global position for coloring
         double globalX1 = t.center.x + x1 * Cos(angles.th) - y1 * Sin(angles.th); 
         double globalY1 = t.center.y + x1 * Sin(angles.th) + y1 * Cos(angles.th);
         double globalZ1 = t.center.z + z1;

         double globalX2 = t.center.x + x2 * Cos(angles.th) - y2 * Sin(angles.th);
         double globalY2 = t.center.y + x2 * Sin(angles.th) + y2 * Cos(angles.th);
         double globalZ2 = t.center.z + z2;

         Color color1 = computeDistanceColor(globalX1, globalY1, globalZ1);
         Color color2 = computeDistanceColor(globalX2, globalY2, globalZ2);

         glColor3f(color1.r, color1.g, color1.b);
         glVertex3d(x1, y1, z1);

         glColor3f(color2.r, color2.g, color2.b);
         glVertex3d(x2, y2, z2);
      }
      glEnd();
   }

   // Restore transformation matrix to whatever it was before we drew the torus
   glPopMatrix();
}

void drawEllipse( EllipseStruct e )
{
   // Save current transformation matrix2
   glPushMatrix();

   // Set the origin to be the center of the ellipse
   glTranslated(e.center.x, e.center.y, e.center.z);

   // Compute the angles for aligning the ellipse with the axis vector
   Angle angles = computeAngles(e.axis);

   // Apply rotations
   glRotated(angles.th, 0.0, 0.0, 1.0);  // Rotate about Z axis
   glRotated(angles.ph, 0.0, 1.0, 0.0);  // Rotate about Y axis
   glRotated(angles.psi, 1.0, 0.0, 0.0); // Rotate about X axis
   
   // Scale by the major and minor axes
   glScaled(e.rMinor, 1.0*e.rMajor, e.rMajor);
   
   //  Latitude bands
   double deltaDegree = 15; // degrees per segment
   for (int ph=-90;ph<90;ph+=deltaDegree)
   {
      glBegin(GL_QUAD_STRIP);
      for (int th=0;th<=360;th+=deltaDegree)
      {
         double x1 = Sin(th)*Cos(ph);
         double y1 = Sin(ph);
         double z1 = Cos(th)*Cos(ph);
         double globalX1 = e.center.x + e.rMinor * x1 * Cos(angles.th) - e.rMinor * y1 * Sin(angles.th);
         double globalY1 = e.center.y + e.rMinor * x1 * Sin(angles.th) + e.rMinor * y1 * Cos(angles.th);
         double globalZ1 = e.center.z + e.rMajor * z1;
         Color color1 = computeDistanceColor(globalX1, globalY1, globalZ1);

         double x2 = Sin(th)*Cos(ph+deltaDegree);
         double y2 = Sin(ph+deltaDegree);
         double z2 = Cos(th)*Cos(ph+deltaDegree);
         double globalX2 = e.center.x + e.rMinor * x2 * Cos(angles.th) - e.rMinor * y2 * Sin(angles.th);
         double globalY2 = e.center.y + e.rMinor * x2 * Sin(angles.th) + e.rMinor * y2 * Cos(angles.th);
         double globalZ2 = e.center.z + e.rMajor * z2;
         Color color2 = computeDistanceColor(globalX2, globalY2, globalZ2);

         glColor3f(color1.r, color1.g, color1.b);
         glVertex3d(x1, y1, z1);

         glColor3f(color2.r, color2.g, color2.b);
         glVertex3d(x2, y2, z2);
      }
      glEnd();
   }

   // Restore color to white
   glColor3f(1.0, 1.0, 1.0);

   // Restore transformation matrix to whatever it was before we drew the ellipse
   glPopMatrix();
}

void drawBicycle(Point origin, Point direction, Point scale)
{
   // Reference angle for forward direction
   Angle forward = computeAngles(direction);

   // Bike parameters for Specialized S-Works Diverge
   // Sourced: https://geometrygeeks.bike/compare/specialized-diverge-s-works-2021-54,cannondale-topstone-carbon-2020-md,3t-cycling-exploro-2020-m/
   // All dimensions in m and degrees
   double seatAngle = 74.0;
   double headAngle = 70.0;
   double r = 0.0254;
   // double topTubeLength = 0.358 / Sin(90.0 - seatAngle);
   double topTubeEff = 0.529;
   double headTubeLength = 0.116;
   double headTubeTopLength = 0.086; // Not specified in source, just a guess
   double chainStayLength = 0.425;
   double handleBarLength = 0.580; // Not specified in source, just a guess
   double seatTubeCC = 0.460;
   double seatTubeLength = 0.120; // Not specified in source, just a guess
   double wheelBase = 1.019;
   double axleWidth = 0.300;
   double wheelRadius = 0.311;

   // Build the bicycle geometry relative to (0,0,0), with seatpost at origin
   Point seatPost = {0.0, 0.0, 0.0};
   Point midHeadTube = {seatPost.x,
                          seatPost.y + topTubeEff * Tan( 90.0 - headAngle),
                          seatPost.z + topTubeEff};

   Point headTubeBottom = {midHeadTube.x,
                        midHeadTube.y + headTubeLength * Cos(90.0 + headAngle),
                        midHeadTube.z + headTubeLength * Sin(90.0 + headAngle)};

   Point headTubeTop = {midHeadTube.x,
   midHeadTube.y - headTubeTopLength * Cos(90.0 + headAngle),
   midHeadTube.z - headTubeTopLength * Sin(90.0 + headAngle)};

   Point seatTubeBottom = {seatPost.x,
                           seatPost.y + seatTubeCC * Cos(90.0 + seatAngle),
                           seatPost.z + seatTubeCC * Sin(90.0 + seatAngle)};

   Point seatTubeTop = {seatPost.x,
   seatPost.y - seatTubeLength * Cos(90.0 + seatAngle),
   seatPost.z - seatTubeLength * Sin(90.0 + seatAngle)};

   Point rearAxle = {seatTubeBottom.x,
                        seatTubeBottom.y - chainStayLength * Cos(104.0),
                        seatTubeBottom.z - chainStayLength * Sin(104.0)};

   Point frontAxle = {rearAxle.x, rearAxle.y, rearAxle.z + wheelBase};
   Point frontAxleLeft = {frontAxle.x - axleWidth / 2.0, frontAxle.y, frontAxle.z};
   Point frontAxleRight = {frontAxle.x + axleWidth / 2.0, frontAxle.y, frontAxle.z};
   Point rearAxleLeft = {rearAxle.x - axleWidth / 2.0, rearAxle.y, rearAxle.z};
   Point rearAxleRight = {rearAxle.x + axleWidth / 2.0, rearAxle.y, rearAxle.z};

   Point handlebarLeft = {headTubeTop.x - handleBarLength / 2.0, headTubeTop.y, headTubeTop.z};
   Point handlebarRight = {headTubeTop.x + handleBarLength / 2.0, headTubeTop.y, headTubeTop.z};

   // Apply rotation
   glPushMatrix();
   glTranslated(-origin.x, -origin.y, -origin.z);
   glRotated(forward.th, 0.0, 0.0, 1.0);  // Rotate about Z axis
   glRotated(forward.ph, 0.0, 1.0, 0.0);  // Rotate about Y axis
   glRotated(forward.psi, 1.0, 0.0, 0.0); // Rotate about X axis
   glScaled(scale.x, scale.y, scale.z);         // Scale to desired size 

   drawCylinder(seatPost, midHeadTube, r);     // Top tube
   drawCylinder(headTubeBottom, headTubeTop, r);  // Head tube
   drawCylinder(seatPost, seatTubeTop, r);       // Actual seat post
   drawCylinder(seatPost, seatTubeBottom, r);    // Seat tube
   drawCylinder(seatTubeBottom, headTubeBottom, r); // Down tube? No name on the diagram
   drawCylinder(seatPost, rearAxleRight, r);          // Chain stay right
   drawCylinder(seatTubeBottom, rearAxleRight, r);       // Seat stay right
   drawCylinder(seatPost, rearAxleLeft, r);          // Chain stay left
   drawCylinder(seatTubeBottom, rearAxleLeft, r);       // Seat stay left
   drawCylinder(rearAxleLeft, rearAxleRight, r);   // Rear axle
   drawCylinder(frontAxleLeft, frontAxleRight, r); // Front axle
   drawCylinder(headTubeBottom, frontAxleRight, r); // Right fork
   drawCylinder(headTubeBottom, frontAxleLeft, r); // Left fork
   drawCylinder(handlebarLeft, handlebarRight, r); // Handlebars

   // Draw wheels
   Torus frontWheel = { (Point){frontAxle.x, frontAxle.y, frontAxle.z}, (Point){1.0, 0.0, 0.0}, wheelRadius, 0.0254};
   drawTorus(frontWheel);
   Torus rearWheel = { (Point){rearAxle.x, rearAxle.y, rearAxle.z}, (Point){1.0, 0.0, 0.0}, wheelRadius, 0.0254};
   drawTorus(rearWheel);   

   // Draw seat
   EllipseStruct seat = { seatTubeTop, midHeadTube, 0.1, 0.05};
   drawEllipse(seat);

   glPopMatrix();
}

void display()
{  
   // Clear the image
   glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

   // Set the camera
   SetCamera(view);

   drawBicycle((Point){0.0, 0.0, 2.0}, (Point){0.0, 0.0, -1.0}, (Point){1.0, 1.0, 1.0});

   drawBicycle((Point){1.5, 0.0, 6.0}, (Point){1.0, 0.0, 1.0}, (Point){2.0, 2.0, 2.0});

   drawBicycle((Point){0.0, 0.0, 4.0}, (Point){1.0, 0.0, 0.0}, (Point){5.0, 5.0, 5.0});

   drawBicycle((Point){1.5, 2.0, 3.0}, (Point){1.0, 1.0, 0.0}, (Point){0.2, 0.2, 0.2});

   drawBicycle((Point){-2.5, -2.0, 3.0}, (Point){1.0, 1.0, 0.0}, (Point){0.2, 0.2, 0.2});

   // Error check
   ErrCheck("display");

   // Flush and swap buffer
   glFlush();
   glutSwapBuffers();
}

/*
 * This function is called by GLUT when the window is resized
 */
void reshape(int width, int height)
{
   // Check the perspective mode
   switch( m )
   {
      // Orthogonal projection
      case 0:

         break;
      case 1:

         break;
      case 2:
         break;
      default:
         Fatal("Unknown perspective mode %d", m);
         break;
   }
   // Avoid divide by zero
   if (height == 0)
   {
      height = 1;
   }
   float aspect = (float)width / height;

   glViewport(0, 0, width, height);

   glMatrixMode(GL_PROJECTION);
   glLoadIdentity();
   gluPerspective(110.0, aspect, 1.0, 30.0);

   glMatrixMode(GL_MODELVIEW);
   glLoadIdentity();
}

void key(unsigned char ch, int x, int y)
{
   // Keys to move the camera around (FPS-style controls)
   // Camera movement step size
   double step = 0.2;
   
   if (ch == 'w' || ch == 'W')        // Move forward (into the scene)
      view.pos.z -= step;
   else if (ch == 's' || ch == 'S')   // Move backward (out of the scene)
      view.pos.z += step;
   else if (ch == 'a' || ch == 'A')   // Strafe left
      view.pos.x -= step;
   else if (ch == 'd' || ch == 'D')   // Strafe right
      view.pos.x += step;
   else if (ch == 'q' || ch == 'Q')   // Move up
      view.pos.y += step;
   else if (ch == 'e' || ch == 'E')   // Move down
      view.pos.y -= step;
   else if (ch == 'r' || ch == 'R')   // Reset camera to a good viewing position
   {
      view.pos = (Point){0.0, 0.0, 0.0};  // Back and slightly above for good view
      view.angle = (Angle){0.0, 0.0, 0.0}; // Slight downward angle
   }
   else if( ch == 'm' or 'M' )
   {
      m = (m + 1) % 3;
   }
   //  Request display update
   glutPostRedisplay();
}

/*
 *  Functionality to move the camera position
 */
void special(int key, int x, int y)
{
   double rotStep = 5.0; // degrees per key press
   // These seem backwards from the code perspective but make more sense when controlling the camera
   if (key == GLUT_KEY_RIGHT)
   {
      view.angle.th += rotStep;
   }
   else if (key == GLUT_KEY_LEFT)    // Look left (yaw left)
   {
      view.angle.th -= rotStep;
   }
   else if (key == GLUT_KEY_UP)      // Look up (pitch up)
   {
      view.angle.ph += rotStep;
   }
   else if (key == GLUT_KEY_DOWN)    // Look down (pitch down)
   {
      view.angle.ph -= rotStep;
   }

   //  Keep angles to +/-360 degrees
   view.angle.th = fmod(view.angle.th, 360.0);
   view.angle.ph = fmod(view.angle.ph, 360.0);

   //  Request display update
   glutPostRedisplay();
}

// Function for basic animations
void idle()
{
}

// Main
int main(int argc, char *argv[])
{
   //  Initialize GLUT
   glutInit(&argc, argv);
   //  Request double buffered true color window without Z-buffer
   glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
   //  Create window
   glutCreateWindow("Brendan Chong - Bicycle");
#ifdef USEGLEW
   //  Initialize GLEW
   if (glewInit() != GLEW_OK)
      Fatal("Error initializing GLEW\n");
#endif
   //  Register display, reshape, idle and key callbacks
   glutDisplayFunc(display);
   glutReshapeFunc(reshape);
   glutKeyboardFunc(key);
   glutSpecialFunc(special);
   glutIdleFunc(idle);
   //  Enable Z-buffer depth test
   glEnable(GL_DEPTH_TEST);
   //  Pass control to GLUT for events
   glutMainLoop();
   return 0;
}