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


//-----------------------------------------------------------
// Global variables
//-----------------------------------------------------------

Color color = {1.0, 1.0, 1.0}; // Default color white

double asp = 1;   // Aspect ratio
int fov = 110;     // Field of view for perspective
int m = 0;        // perspective mode switcher
double dim = 10.0; // Size of the world
double off = -3.6; // Z offset for orthographic and perspective projections
double ph = 20;  // Elevation of view angle
double th = 45;   // Azimuth of view angle
double Ex = 0.0; // First person camera x position
double Ey = 0.0; // first person camera y position
double Ez = 0.0; // first person camera z position
double dx = 0.0; // first person view direction x component
double dy = 0.0; // first person view direction y component
double dz = 1.0; // first person view direction z component

// Mouse control variables
int mouseX = 0, mouseY = 0;  // Current mouse position
int mouseCaptured = 0;       // Whether mouse is captured for camera control

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
   glTranslated(origin.x, origin.y, origin.z);
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

   // Reset transformations
   glLoadIdentity();

   // Orthogonal projection
   if( m == 0 )
   {
      // Rotate down to give oblique angle
      glRotated(ph, 1.0, 0.0, 0.0);
      glRotated(th, 0.0, 1.0, 0.0);
   }
   // Perspective projection
   else if( m == 1 )
   {
      Ex = -2*dim*Sin(th)*Cos(ph);
      Ey = +2*dim        *Sin(ph);
      Ez = +2*dim*Cos(th)*Cos(ph);
      gluLookAt(Ex,Ey,Ez , 0,0,0 , 0,Cos(ph),0);
   }
   // First person
   else
   {
      Ey = 0.1*dim;
      dx = Cos(th);
      dy = Sin(ph);
      dz = Sin(th);

      gluLookAt(Ex,Ey,Ez , Ex+dx,Ey+dy,Ez+dz , 0, 1, 0);
   }
   // Draw the "ground" as white plane at y=0
   glColor3f(1.0, 1.0, 1.0);
   glBegin(GL_QUADS);
   glVertex3f(-dim, 0.0, -dim);
   glVertex3f(+dim, 0.0, -dim);
   glVertex3f(+dim, 0.0, +dim);
   glVertex3f(-dim, 0.0, +dim);
   glEnd();

   drawBicycle((Point){0.0, 3.0, 0.1*dim}, (Point){0.0, 0.0, -1.0}, (Point){1.0, 1.0, 1.0});

   drawBicycle((Point){0.4*dim*asp, 2.5, -0.8*dim}, (Point){1.0, 0.0, 1.0}, (Point){2.0, 2.0, 2.0});

   drawBicycle((Point){0.0, 3.5, 0.75*dim}, (Point){1.0, 0.0, 0.0}, (Point){5.0, 5.0, 5.0});

   drawBicycle((Point){-0.3*dim*asp, 1.2, 0.4*dim}, (Point){-0.8, 0.3, 1.0}, (Point){1.4, 1.4, 1.4});

   drawBicycle((Point){-0.65*dim*asp, 2.0, -0.5*dim}, (Point){1.0, 1.0, 1.0}, (Point){1.3, 1.3, 1.3});

   glColor3f(1.0, 0.0, 0.0 );
   glWindowPos2i(5, 25);
   Print("th = %.1f ph = %.1f", th, ph);
   Print("Ex = %.1f Ey = %.1f Ez = %.1f", Ex, Ey, Ez);
   Print("dx = %.2f dy = %.2f dz = %.2f", dx, dy, dz);
   switch( m )
   {
      case 0:
         glWindowPos2i(5,5);
         Print("Projection = Orthogonal");
         glWindowPos2i(5, 45);
         Print("Z offset = %.1f", off);
      break;
      case 1:
         glWindowPos2i(5,5);
         Print("Projection = Perspective");
      break;
      case 2:
         glWindowPos2i(5,5);
         Print("Projection = First person");
      break;
      default:
         Fatal("Unknown projection mode: %d", m);
   }
   
   
   // Error check
   ErrCheck("display");

   // Flush and swap buffer
   glFlush();
   glutSwapBuffers();
}


/*
 *  Set projection
 */
static void Project()
{
   //  Tell OpenGL we want to manipulate the projection matrix
   glMatrixMode(GL_PROJECTION);
   //  Undo previous transformations
   glLoadIdentity();
   //  Orthogonal projection
   if ( m == 0 )
   {
      glOrtho(-asp*dim,+asp*dim, -dim,+dim, -dim+off,2*dim+off);  // shifted back to see more of the scene
   }
   //  Perspective projection
   else if( m == 1)
   {
      gluPerspective(fov,asp, dim/4, 4*dim);
   }
   // First person
   else
   {
      gluPerspective(fov,asp, 0.001, 4*dim);
   }
   //  Switch to manipulating the model matrix
   glMatrixMode(GL_MODELVIEW);
   //  Undo previous transformations
   glLoadIdentity();
}


/*
 * This function is called by GLUT when the window is resized
 */
void reshape(int width, int height)
{   
   // Avoid divide by zero
   asp = (height>0) ? (double)width/height : 1;

   glViewport(0, 0, width, height);

   // Apply projection based on the current mode
   Project();
}

void key(unsigned char ch, int x, int y)
{
   // Keys to move the camera around (FPS-style controls)   
   if (ch == 'w' || ch == 'W')        // Move forward (into the scene)
   {
      if( m == 0 )
         off += 0.1;
      else
      {
         Ex += 0.1*dx;
         Ez += 0.1*dz;
      }
   }
   else if (ch == 's' || ch == 'S')   // Move backward (out of the scene)
   {
      if( m == 0 )
         off -= 0.1;
      else
      {
         Ex -= 0.1*dx;
         Ez -= 0.1*dz;
      }
   }
   else if (ch == 'a' || ch == 'A')   // Strafe left
   {
      if( m == 2 )
      {
         Ex += 0.1*Cos(90.0-th);
         Ez -= 0.1*Sin(90.0-th);
      }
   }
   else if (ch == 'd' || ch == 'D')   // Strafe right
   {
      if( m == 2 )
      {
         Ex -= 0.1*Cos(90.0-th);
         Ez += 0.1*Sin(90.0-th);
      }
   }
   else if (ch == 'r' || ch == 'R')   // Reset to original configuration
   {
      m = 0;
      Ex = 0;
      Ey = 1;
      Ez = 5;
      th = 45;
      ph = 20;
      off = -3.6;
   }
   else if( ch == 'm' || ch == 'M' )
   {
      m = (m + 1) % 3;

      if( m < 2 )
      {
         ph = 20.0; // Reset pitch to 20

         // Determine the theta angle based on current Ex, Ez position
         if( m == 0 )
         {
            th = atan2(Ez, Ex) * 180.0 / 3.1415926535 - 90.0;
         }
      }
      else
      {
         // When switching to first person, look at the origin         
         // Calculate the angles needed to look in this direction
         double distance = sqrt(Ex*Ex + Ey*Ey + Ez*Ez);
         if (distance > 0.001) 
         {
            // Calculate theta (horizontal angle) - atan2 gives us the angle in the XZ plane
            th = atan2(-Ez, -Ex) * 180.0 / 3.1415926535;
         } 
         else 
         {
            // look forward if at origin
            th = 0.0;
         }
      }
   }
   else if (ch == 27) // Escape key
      exit(0);
      
   // Keep the first person view to only inside the world box
   if( m == 2 )
   {
      Ex = (Ex < -dim) ? -dim+0.001 : (Ex > dim) ? dim-0.001 : Ex;
      Ez = (Ez < -dim) ? -dim+0.001 : (Ez > dim) ? dim-0.001 : Ez;
   }

   Project();
   //  Request display update
   glutPostRedisplay();
}

/*
 *  Functionality to move the camera position
 */
void special(int key, int x, int y)
{
   // These seem backwards from the code perspective but make more sense when controlling the camera
   if (key == GLUT_KEY_RIGHT)
   {
      th += 5;
   }
   else if (key == GLUT_KEY_LEFT)    // Look left (yaw left)
   {
      th -= 5;
   }
   else if (key == GLUT_KEY_UP)      // Look up (pitch up)
   {
      ph += 5;
   }
   else if (key == GLUT_KEY_DOWN)    // Look down (pitch down)
   {
      ph -= 5;
   }

   Project();

   //  Request display update
   glutPostRedisplay();
}

/*
 *  Mouse motion callback
 */
void motion(int x, int y)
{
   if (mouseCaptured)
   {
      // Calculate mouse movement
      int deltaX = x - mouseX;
      int deltaY = y - mouseY;
      
      // Update angles based on mouse movement
      th += deltaX * 0.1;  // Horizontal movement controls azimuth (yaw)
      if( m < 2 )
         ph += deltaY * 0.1;  // Vertical movement controls elevation (pitch)
      else
      ph -= deltaY * 0.1;  // Vertical movement controls elevation (pitch), invert for first person
      
      // Keep phi within reasonable bounds
      if (ph > 89) ph = 89;
      if (ph < -89) ph = -89;
      
      // Wrap theta around 360 degrees
      if (th > 360) th -= 360;
      if (th < 0) th += 360;
      
      // Store current mouse position
      mouseX = x;
      mouseY = y;
      
      Project();
      glutPostRedisplay();
   }
}

/*
 *  Mouse button callback
 */
void mouse(int button, int state, int x, int y)
{
   if (button == GLUT_LEFT_BUTTON)
   {
      if (state == GLUT_DOWN)
      {
         // Capture mouse for camera control
         mouseCaptured = 1;
         mouseX = x;
         mouseY = y;
         glutSetCursor(GLUT_CURSOR_NONE);  // Hide cursor
      }
      else
      {
         // Release mouse capture
         mouseCaptured = 0;
         glutSetCursor(GLUT_CURSOR_INHERIT);  // Show cursor
      }
   }
}

/*
 *  Passive mouse motion (when no button is pressed)
 */
void passiveMotion(int x, int y)
{
   // Store mouse position even when not captured
   mouseX = x;
   mouseY = y;
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
   glutMouseFunc(mouse);           // Mouse button callback
   glutMotionFunc(motion);         // Mouse motion callback (with button pressed)
   glutPassiveMotionFunc(passiveMotion); // Mouse motion callback (without button pressed)
   //  Enable Z-buffer depth test
   glEnable(GL_DEPTH_TEST);
   //  Pass control to GLUT for events
   glutMainLoop();
   return 0;
}