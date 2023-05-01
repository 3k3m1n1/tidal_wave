/**
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
 [*] how many particles should we start with?
 
 [*] import kinect library
 [*] make hand apply impulse instead of mouseX/Y
   [*] okay now do two hands (left and right)
 [*] map skeleton depth coordinates from 512x424 to full window
 [*] add support for multiple skeletons (in case people step into the circle with their kids)
 [ ] should i specify a cut-off depth so that the audience isn't tracked?
 
 
 EXTRAS:
 [*] interactive sound??

 [*] hand indicators? good idea to give feedback to participants
   [?] any way to make them more subtle? or at least not some debugging eyesore?
 
 [ ] idle state: turbulent waves? come up with some kind of motion to draw people in 
 
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

import processing.sound.*;


int viewport_w = 1800;    // window scale
int viewport_h = 1080;
int viewport_x = 0;        //  window position
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

float gravity = 1.46;

float impulse_max = 556;
float impulse_mul = 15;
float impulse_tsmooth = 0.90f;
int   impulse_blur  = 0;
float vx_threshold = 30;

float timer = 0;
float timerStart = 0;
float countdown = 0;
float countdownStart = 0.3;  // seconds

SoundFile[] waveSounds;
String[] waveFiles = {
  "442944__qubodup__ocean-wave__edit1.wav", 
  "442944__qubodup__ocean-wave__edit2.wav",
  "587078__danielwalsh__wave__edit.wav"
};
int random = 0;

KinectPV2 kinect;
ArrayList<KSkeleton> skeletonArray;
Person[] waterbenders;

class Hand {
  float scaledX, scaledY;
  float prevX, prevY;
  
  Hand() {
    scaledX = 0;
    scaledY = 0;
    
    prevX = 0;
    prevY = 0;
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
  size(viewport_w, viewport_h, P2D);
  
  // or
  
  //fullScreen(P2D, 1);
  smooth(0);
}

public void setup() {
  // move sketch window
  //viewport_x = displayWidth / 2;
  surface.setLocation(viewport_x, viewport_y);

  // particles setup
  context = new DwPixelFlow(this);
  context.print();
  context.printGL();
  
  filter = DwFilter.get(context);

  particles = new DwFlowFieldParticles(context, 1024 * 1024);

  particles.param.col_A = new float[]{0.10f, 0.50f, 1.00f, 1};
  particles.param.col_B = new float[]{0.05f, 0.25f, 0.50f, 0};

  particles.param.velocity_damping  = 0.999f;  // originally 0.995f, keep this high for responsiveness but never 1
  particles.param.steps = 1;

  particles.param.size_display = 10;
  particles.param.size_collision = 9;  // keep slightly smaller than display for overlap
  particles.param.size_cohesion  = 14;

  particles.param.mul_col = 1f;  // collision *multiplier*
  particles.param.mul_coh = 1.22f;  // cohesion multiplier, originally 1.00f (the higher, the more compact. too high and you get bee swarms, too low and you get scattering ball pit)
  particles.param.mul_obs = 3f;  // obstacles multiplier
  
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
  
  // initialize kinect
  kinect = new KinectPV2(this);
  kinect.enableSkeletonDepthMap(true);
  kinect.init();
  
  // and objects to store hand positions in
  waterbenders = new Person[6];
  for (int i = 0; i < waterbenders.length; i++) {
    waterbenders[i] = new Person();
  }
  
  // start timer
  timerStart = millis() / 1000;
  countdown  = countdownStart;
  
  // load sounds from data folder
  waveSounds = new SoundFile[waveFiles.length];
  for (int i = 0; i < waveFiles.length; i++) {
    waveSounds[i] = new SoundFile(this, waveFiles[i]);
  }
  
  // some final settings:
  frameRate(1000);
  noCursor();
  
  // alright let's do this
  spawnParticles();
} //<>//

public void draw() {

  particles.param.timestep = 1f/frameRate;

  addObstacles();
  addImpulse();
  addGravity(); //<>//

  updateParticles(); //<>//
  renderScene();
  
  // debug: write stats to window title
  String txt_fps = String.format(Locale.ENGLISH, "[%s]   [%7.2f fps]   [particles %,d] ", getClass().getSimpleName(), frameRate, particles.getCount() );
  surface.setTitle(txt_fps);
}


public void addObstacles() {

  int w = pg_obstacles.width;
  int h = pg_obstacles.height;

  pg_obstacles.beginDraw();
  pg_obstacles.clear();
  pg_obstacles.noStroke();
  pg_obstacles.blendMode(REPLACE);
  pg_obstacles.rectMode(CORNERS);  // so much easier

  // border, with raised ceiling (can't seem to really draw past the canvas sadly)
  pg_obstacles.fill(0, 255);
  pg_obstacles.rect(0, -10, w, h);  // outer boundary
  pg_obstacles.fill(0, 0);
  pg_obstacles.rect(10, 0, w-10, h-10);  // inner boundary (leave enough padding or else particles will jump through if they're going fast enough - just like unity)

  pg_obstacles.endDraw();
}

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
    if (skeleton.isTracked()) {
      
      // get impulse center
      KJoint[] joints = skeleton.getJoints();
      waterbenders[i].hands[0].updatePos( joints[KinectPV2.JointType_HandLeft] );
      waterbenders[i].hands[1].updatePos( joints[KinectPV2.JointType_HandRight] );
      
      for (Hand hand : waterbenders[i].hands) {    
        
        // calculate velocity
        vx = (hand.scaledX - hand.prevX) * +impulse_mul;                           //vx = (mouseX - pmouseX) * +impulse_mul;
        vy = (hand.scaledY - hand.prevY) * -impulse_mul;  // flip vertically       //vy = (mouseY - pmouseY) * -impulse_mul;
        
        // trigger wave sound effects with side-to-side movement
        // (only counts if hands are "below sea level")
        if (vx > vx_threshold && hand.scaledX > height/2) {
          triggerSoundEffect(hand.scaledX);
        }
        
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
          pg_impulse.ellipse(hand.scaledX, hand.scaledY, 300, 300);               //pg_impulse.ellipse(mouseX, mouseY, 300, 300);
        }
        
        // save kinect hand position for next velocity calc
        hand.prevX = hand.scaledX;
        hand.prevY = hand.scaledY;
        
      }  // end of hands for loop
    }  // end of if(skeleton.isTracked())
  }  // end of skeletons for loop
  
  // fallback: mouse controls for development / debugging
  // impulse center/velocity
  vx = (mouseX - pmouseX) * +impulse_mul;
  vy = (mouseY - pmouseY) * -impulse_mul; // flip vertically
  
  // trigger wave sound effects with side-to-side movement
  // (only counts if hands are "below sea level")
  if (vx > vx_threshold && mouseX > height/2) {
    triggerSoundEffect(mouseX);
  }
  
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
    pg_impulse.ellipse(mouseX, mouseY, 300, 300);
  }
  pg_impulse.endDraw();

  // create impulse texture
  ff_impulse.resize(w, h);
  {
    Merge.TexMad ta = new Merge.TexMad(ff_impulse.tex_vel, impulse_tsmooth, 0);
    Merge.TexMad tb = new Merge.TexMad(pg_impulse, 1, -mid/255f);
    DwFilter.get(context).merge.apply(ff_impulse.tex_vel, ta, tb);
    //ff_impulse.blur(1, impulse_blur);
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

public void updateParticles() {
  particles.resizeWorld(width, height);
  particles.createObstacleFlowField(pg_obstacles, new int[]{0, 0, 0, 255}, false);
  particles.update(ff_acc);
}

public void renderScene() {
  // render obstacles
  pg_canvas.beginDraw();
  pg_canvas.background(0);
  pg_canvas.image(pg_obstacles, 0, 0);
  
  // render hand indicators
  /*
  for (int i = 0; i < waterbenders.length; i++) {
  //for (int i = 0; i < skeletonArray.size(); i++) {
    //KSkeleton skeleton = (KSkeleton) skeletonArray.get(i);
    //if (skeleton.isTracked()) {
      for (Hand hand : waterbenders[i].hands) {   
        pg_canvas.stroke(99, 224, 255, 255);
        pg_canvas.strokeWeight(6);
        pg_canvas.noFill();
        pg_canvas.ellipse(hand.scaledX, hand.scaledY, 70, 70);
      }
    //}
  }
  */
  pg_canvas.endDraw();
  
  // render particles
  particles.displayParticles(pg_canvas);
  
  // post-processing (bloom!)
  filter.luminance_threshold.param.threshold = 0.48f;  // when 0, all colors are used
  filter.luminance_threshold.param.exponent  = 5;
  filter.luminance_threshold.apply(pg_canvas, pg_luminance);
  filter.bloom.setBlurLayers(10);
  filter.bloom.gaussianpyramid.setBlurLayers(10);
  filter.bloom.param.blur_radius = 1;
  filter.bloom.param.mult   = 0.7f; 
  filter.bloom.param.radius = 0.3f;
  filter.bloom.apply(pg_luminance, null, pg_canvas);
  
  // draw final image to screen
  blendMode(REPLACE);
  image(pg_canvas, 0, 0);
  blendMode(BLEND);
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

public void triggerSoundEffect(float x) {
  timer = millis() / 1000 - timerStart;    // counts up from 0
  countdown = countdownStart - timer;      // counts down from cd start time
  
  waveSounds[random].pan(map(x, 0, width, -1.0, 1.0));
  if (countdown < 0) {
    random = int(random(waveSounds.length));  // pick a new sound at random
    waveSounds[random].play();
    timerStart = millis() / 1000;  // reset timer
  }
}
