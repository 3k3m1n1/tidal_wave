/*
      Tidal Wave
 by Ekemini Nkanta
 
 
 * hello!! :)
 * i could not have pulled this off without:
 
 * PixelFlow - Thomas Diewald
 *   A Processing/Java library for high performance GPU-Computing.
 *   github.com/diwi/PixelFlow.git
 *   thomasdiewald.com
 *
 * KinectPV2 - Thomas Sanchez Lengeling
 *   Kinect for Windows v2 library for Processing.
 *   codigogenerativo.com
 *
 
 MIT License: https://opensource.org/licenses/MIT
 
 */

/*
  TO-DO:
 [*] tweak particle parameters to feel like waterbending / ultimate control
       (slow it down some, high attraction for a commanding pull, flow should feel like a dance)
 [*] change particle colors to ocean blues
 [*] add gravity
 [*] explore particle shaders + blend modes
 [*] play with bloom
 [ ] how many particles should we start with?
 
 [*] import kinect library
 [*] make hand apply impulse instead of mouseX/Y
   [*] okay now do two hands (left and right)
 [*] map skeleton depth coordinates from 512x424 to full window
 [*] add support for multiple skeletons (in case people step into the circle with their kids)
   [ ] should i specify a cut-off depth so that the audience isn't tracked?

 [ ] idle state: turbulent waves? come up with some kind of motion to draw people in 

 [ ] ??interactive sound??
 (start off realistic, then remix / abstract it - samplefocus is your friend)

 [ ] subtle/elegant hand indicators?
       not just for debugging, but to give feedback to participants (whether they're tech-savvy or not)

 
 EXTRAS:

 [ ] oooh the glitched particles made that ocean-filtering-through-sand effect...
       spawn some invisible dot obstacles on purpose?
 
 [ ] sprite swap?
 [ ] background treatment?
     *how do i get the background to have a soft blurry fill where particles just passed??
        translucent backgrounds didn't work the first time i tried - probably bc of the way we're creating/applying the PGraphics
        (+ it would look like hard lines anyway, which isn't the look i'm going for)
       
       would i be proccessing previous frames with a gaussian blur pass of some sort?
         (how slow is that?)
       
       if anything maybe just throw a video of the effect you want back there??? (i hate this idea)
 
 [ ] i really loved the attractor effect - maybe there's a *cough* avatar effect where for a brief period of time (random or manually triggered) your control switches from impulses to attractors
 (less like pushing waves to shore, more like guiding a water whip)
 
 */
  
import java.util.Locale;

import com.thomasdiewald.pixelflow.java.DwPixelFlow;
import com.thomasdiewald.pixelflow.java.flowfieldparticles.DwFlowFieldParticles;
import com.thomasdiewald.pixelflow.java.imageprocessing.DwFlowField;
import com.thomasdiewald.pixelflow.java.imageprocessing.filter.DwFilter;
import com.thomasdiewald.pixelflow.java.imageprocessing.filter.Merge;

import processing.core.*;
import processing.opengl.PGraphics2D;

import KinectPV2.*;


int viewport_w = 1920;  //1280
int viewport_h = 1080;  //720
int viewport_x = 0;  //230
int viewport_y = 0;

PGraphics2D pg_canvas;
PGraphics2D pg_obstacles;
PGraphics2D pg_impulse;
PGraphics2D pg_gravity;
PGraphics2D pg_luminance;

DwPixelFlow context;

DwFlowFieldParticles particles;
DwFlowField ff_acc;
DwFlowField ff_impulse;
DwFilter filter;

float gravity = 1;

KinectPV2 kinect;
ArrayList<KSkeleton> skeletonArray;
Person[] waterbenders;


class Hand {
  float scaledX, scaledY;
  float prevX, prevY;
  
  Hand() {
    scaledX = 0;
    scaledY = 0;
    
    prevX = width / 2;
    prevY = height / 2;
  }
  
  void updatePos(KJoint joint) {
    // scale to full window (depth map is 512x424)
    scaledX = map(joint.getX(), 0, 512, 0, width);
    scaledY = map(joint.getY(), 0, 424, 0, height);
  }
}

class Person {
  Hand[] hands;
  
  Person() {
    hands = new Hand[2];
    for (int i = 0; i < hands.length; i++) {
      hands[i] = new Hand();
    }
  }
}


public void settings() {
  //viewport_w = (int) min(viewport_w, displayWidth  * 0.9f);
  //viewport_h = (int) min(viewport_h, displayHeight * 0.9f);
  //size(viewport_w, viewport_h, P2D);
  fullScreen(P2D, 1);
  smooth(0);
}

public void setup() {
  surface.setLocation(viewport_x, viewport_y);

  context = new DwPixelFlow(this);
  context.print();
  context.printGL();
  
  filter = DwFilter.get(context);

  particles = new DwFlowFieldParticles(context, 1024 * 1024);

  // blue! *-*
  particles.param.col_A = new float[]{0.10f, 0.50f, 1.00f, 1};
  particles.param.col_B = new float[]{0.05f, 0.25f, 0.50f, 0};

  particles.param.velocity_damping  = 1f;  // originally 0.995f, keep this high for responsiveness but never 1
  particles.param.steps = 1;

  particles.param.size_display = 10;
  particles.param.size_collision = 9;  // keep slightly smaller than display for overlap
  particles.param.size_cohesion  = 14;

  particles.param.mul_col = 1f;  // collision *multiplier*
  particles.param.mul_coh = 0.12f;  // cohesion, originally 1.00f (the higher, the more compact. too high and you get bee swarms, too low and you get scattering ball pit)
  particles.param.mul_obs = 3f;  // obstacles
  
  particles.param.shader_type = 1;  // particles glow bright upon collision + fade to black as they lose velocity! just like bioluminescent plankton!!!
  particles.param.shader_collision_mult = 0.17;    // keep this mid to low for pretty contrast + the element of surprise :)
  
  //particles.param.blend_mode = 1;  // blend mode: add -> almost a winner, but the brightness is just so blown out. if bloom wasn't a thing i'd choose you
                                    // use with particles.param.col_A = new float[]{0.10f, 0.50f, 1.00f, .2f} and particles.param.shader_collision_mult = 0.24
  
  ff_acc = new DwFlowField(context);
  ff_acc.param.blur_iterations = 0;
  ff_acc.param.blur_radius     = 1;

  ff_impulse = new DwFlowField(context);
  ff_impulse.param.blur_iterations = 1;
  ff_impulse.param.blur_radius     = 1;


  pg_canvas = (PGraphics2D) createGraphics(width, height, P2D);
  pg_canvas.smooth(0);

  pg_obstacles = (PGraphics2D) createGraphics(width, height, P2D);
  pg_obstacles.smooth(8);

  pg_impulse = (PGraphics2D) createGraphics(width, height, P2D);
  pg_impulse.smooth(0);

  pg_gravity = (PGraphics2D) createGraphics(width, height, P2D);
  pg_gravity.smooth(0);
  
  pg_luminance = (PGraphics2D) createGraphics(width, height, P2D);
  pg_luminance.smooth(0);
  
  
  kinect = new KinectPV2(this);
  kinect.enableSkeletonDepthMap(true);
  kinect.init();
  
  
  waterbenders = new Person[6];
  for (int i = 0; i < waterbenders.length; i++) {
    waterbenders[i] = new Person();
  }
  
  
  noCursor();
  frameRate(1000);
  spawnParticles();
}


float impulse_max = 556;
float impulse_mul = 15;
float impulse_tsmooth = 0.90f;
int   impulse_blur  = 0;

public void addImpulse() {

  int w = width;
  int h = height;

  float vx, vy;
  final int mid = 127;

  // render "velocity"
  pg_impulse.beginDraw();
  pg_impulse.blendMode(REPLACE);
  pg_impulse.background(mid, mid, mid, 0);
  pg_impulse.noStroke();
  pg_impulse.rectMode(CENTER);

  // apply impulse with kinect hands instead of mouse press
  skeletonArray = kinect.getSkeletonDepthMap();
  for (int i = 0; i < skeletonArray.size(); i++) {
    KSkeleton skeleton = (KSkeleton) skeletonArray.get(i);
    if (skeleton.isTracked()) {                                 //if (mousePressed) {
      // get impulse center
      KJoint[] joints = skeleton.getJoints();
      waterbenders[i].hands[0].updatePos( joints[KinectPV2.JointType_HandLeft] );
      waterbenders[i].hands[1].updatePos( joints[KinectPV2.JointType_HandRight] );
      
      for (Hand hand : waterbenders[i].hands) {     //<>//
        // calculate impulse velocity
        vx = (hand.scaledX - hand.prevX) * +impulse_mul;                           //vx = (mouseX - pmouseX) * +impulse_mul; //<>//
        vy = (hand.scaledY - hand.prevY) * -impulse_mul;  // flip vertically       //vy = (mouseY - pmouseY) * -impulse_mul; //<>//
        
        // clamp velocity
        float vv_sq = vx*vx + vy*vy;
        float vv_sq_max = impulse_max*impulse_max;
        if (vv_sq > vv_sq_max) {
          vx = impulse_max * vx / sqrt(vv_sq);
          vy = impulse_max * vy / sqrt(vv_sq);
        }
        
        // map velocity, to UNSIGNED_BYTE range
        vx = 127 * vx / impulse_max;
        vy = 127 * vy / impulse_max;
        
        // finally... draw impulse 
        if (vv_sq != 0) {
          pg_impulse.fill(mid+vx, mid+vy, 0);
          pg_impulse.ellipse(hand.scaledX, hand.scaledY, 300, 300);                         //pg_impulse.ellipse(mx, my, 300, 300);
        }
        
        // save kinect hand position for next velocity calc
        hand.prevX = hand.scaledX; //<>//
        hand.prevY = hand.scaledY; //<>//
        
      }  // end of hands for loop
    }  // end of if(skeleton.isTracked())
  }  // end of skeletons for loop
  
  // fallback:
  if (mousePressed) {
    // impulse center/velocity
    float mx = mouseX;
    float my = mouseY;
    vx = (mouseX - pmouseX) * +impulse_mul;
    vy = (mouseY - pmouseY) * -impulse_mul; // flip vertically
    // clamp velocity
    float vv_sq = vx*vx + vy*vy;
    float vv_sq_max = impulse_max*impulse_max;
    if (vv_sq > vv_sq_max) {
      vx = impulse_max * vx / sqrt(vv_sq);
      vy = impulse_max * vy / sqrt(vv_sq);
    }
    // map velocity, to UNSIGNED_BYTE range
    vx = 127 * vx / impulse_max;
    vy = 127 * vy / impulse_max;
    if (vv_sq != 0) {
      pg_impulse.fill(mid+vx, mid+vy, 0);
      pg_impulse.ellipse(mx, my, 300, 300);  //pg_impulse.ellipse(mx, my, 100, 100);
    }
  }
  pg_impulse.endDraw();

  // create impulse texture
  ff_impulse.resize(w, h);
  {
    Merge.TexMad ta = new Merge.TexMad(ff_impulse.tex_vel, impulse_tsmooth, 0);
    Merge.TexMad tb = new Merge.TexMad(pg_impulse, 1, -mid/255f);
    DwFilter.get(context).merge.apply(ff_impulse.tex_vel, ta, tb);
    ff_impulse.blur(1, impulse_blur);
  }
}

public void addGravity() {

  int w = width;
  int h = height;

  // render "gravity"
  pg_gravity.beginDraw();
  pg_gravity.blendMode(REPLACE);
  pg_gravity.background(0, 255, 0);
  pg_gravity.endDraw();

  // create acceleration texture
  ff_acc.resize(w, h);
  {
    float mul_gravity = -gravity/10f;
    Merge.TexMad ta = new Merge.TexMad(ff_impulse.tex_vel, 1, 0);
    Merge.TexMad tb = new Merge.TexMad(pg_gravity, mul_gravity, 0);
    DwFilter.get(context).merge.apply(ff_acc.tex_vel, ta, tb);
  }
}

public void draw() {

  particles.param.timestep = 1f/frameRate;

  updateScene();

  //spawnParticles();

  addImpulse();

  addGravity();

  // update particle simulation
  particles.resizeWorld(width, height);
  particles.createObstacleFlowField(pg_obstacles, new int[]{0, 0, 0, 255}, false);
  particles.update(ff_acc);

  // render obstacles + particles
  pg_canvas.beginDraw();
  pg_canvas.background(0);
  pg_canvas.image(pg_obstacles, 0, 0);
  // + hand indicators
  /*
  for (int i = 0; i < waterbenders.length; i++) {
  //for (int i = 0; i < skeletonArray.size(); i++) {
    //KSkeleton skeleton = (KSkeleton) skeletonArray.get(i);
    //if (skeleton.isTracked()) {
      for (Hand hand : waterbenders[i].hands) {   
        pg_canvas.stroke(255, 100);
        pg_canvas.strokeWeight(8);
        pg_canvas.noFill();
        pg_canvas.ellipse(hand.scaledX, hand.scaledY, 70, 70);
      }
    //}
  }
  */
  pg_canvas.endDraw();
  particles.displayParticles(pg_canvas);
  
  addBloom();

  blendMode(REPLACE);
  image(pg_canvas, 0, 0);
  blendMode(BLEND);

  String txt_fps = String.format(Locale.ENGLISH, "[%s]   [%7.2f fps]   [particles %,d] ", getClass().getSimpleName(), frameRate, particles.getCount() );
  surface.setTitle(txt_fps);
}


void updateScene() {

  int w = pg_obstacles.width;
  int h = pg_obstacles.height;
  float dim = 3 * h/4f;

  pg_obstacles.beginDraw();
  pg_obstacles.clear();
  pg_obstacles.noStroke();
  pg_obstacles.blendMode(REPLACE);
  pg_obstacles.rectMode(CORNER);

  // border
  pg_obstacles.fill(0, 255);
  pg_obstacles.rect(0, 0, w, h);        // outer boundary? (adjust this too or else your particles will pour out)
      // to-do: push boundaries up slightly
  pg_obstacles.fill(0, 0);
  pg_obstacles.rect(10, 10, w-20, h-20);  // sets the bounding box (can be much smaller as long as contained by above)

  // animated obstacles
  //pg_obstacles.rectMode(CENTER);
  //pg_obstacles.pushMatrix();
  //{
  //  float px = sin(frameCount/240f) * 0.8f * w/2;
  //  pg_obstacles.translate(w/2 + px, h/2);
  //  pg_obstacles.rotate(frameCount/120f);
  //  pg_obstacles.fill(0, 255);
  //  pg_obstacles.rect(0, 0,  10, dim);
  //}
  //pg_obstacles.popMatrix();
  pg_obstacles.endDraw();
}


public void spawnParticles() {

  float px, py, vx, vy, radius;
  int count, vw, vh;

  vw = width;
  vh = height;

  count = 30000;  // adjust as desired
  radius = 200;
  px = vw/2f;
  py = 3 * vh/4f;
  vx = 0;
  vy = -500;

  DwFlowFieldParticles.SpawnRadial sr = new DwFlowFieldParticles.SpawnRadial();
  sr.num(count);
  sr.dim(radius, radius);
  sr.pos(px, vh-1-py);
  sr.vel(vx, vy);
  particles.spawn(vw, vh, sr);

}

public void addBloom() {
  
  filter.luminance_threshold.param.threshold = 0.48f; // when 0, all colors are used
  filter.luminance_threshold.param.exponent  = 5;
  filter.luminance_threshold.apply(pg_canvas, pg_luminance);
      
  filter.bloom.setBlurLayers(10);
  filter.bloom.gaussianpyramid.setBlurLayers(10);
  filter.bloom.param.blur_radius = 1;
  filter.bloom.param.mult   = 0.7f; 
  filter.bloom.param.radius = 0.3f;
  filter.bloom.apply(pg_luminance, null, pg_canvas);
}
