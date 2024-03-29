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
  [ ] swap sprite
  [ ] play around with trails / alpha background
  [ ] how many should we start with?
  [ ] oooh the glitched-out particles unintentionally made a realistic ocean-filtering-through-sand effect.
       spawn some slow-moving dot obstacles on purpose?

  [ ] connect kinect + import library
  [ ] attach attractors #1 and #2 to left & right hands
      (you can practice with the mouse cursor first)
  [ ] do we need a fallback for loss of tracking? (or does that handle itself?)

  [ ] create an alternate state with gravity (+ extra collision mul) instead of attraction
  [ ] switch between them with a keyboard toggle
  [ ] now switch between them with an idle timer: after x seconds of no/barely any movement, turn on gravity
  [ ] new movement should reset state + timer
  
  EXTRAS:
  [ ] ??interactive sound??
        (start off realistic, then remix / abstract it - samplefocus is your friend)
  [ ] idle state: turbulent waves? come up with some kind of motion
  [ ] make the sides apply opposite force / impulse for a "splash" / wave crash effect
  [ ] tweak bloom. use misc>bloom_demo for reference
  
*/

import java.util.Locale;

import com.jogamp.opengl.GL2;
import com.thomasdiewald.pixelflow.java.DwPixelFlow;
import com.thomasdiewald.pixelflow.java.dwgl.DwGLSLProgram;
import com.thomasdiewald.pixelflow.java.dwgl.DwGLTexture;
import com.thomasdiewald.pixelflow.java.flowfieldparticles.DwFlowFieldParticles;
import com.thomasdiewald.pixelflow.java.imageprocessing.DwFlowField;
import com.thomasdiewald.pixelflow.java.imageprocessing.filter.DwFilter;
import com.thomasdiewald.pixelflow.java.imageprocessing.filter.Merge;
import com.thomasdiewald.pixelflow.java.utils.DwUtils;

import controlP5.Accordion;
import controlP5.CColor;
import controlP5.ControlP5;
import controlP5.Group;
import processing.core.*;
import processing.opengl.PGraphics2D;
import processing.opengl.PGraphicsOpenGL;

boolean START_FULLSCREEN = !true;

int viewport_w = 1680;
int viewport_h = 1024;
int viewport_x = 230;
int viewport_y = 0;

int gui_w = 200;
int gui_x = 30;
int gui_y = 30;

PGraphics2D pg_canvas;
PGraphics2D pg_obstacles;
PGraphics2D pg_impulse;
PGraphics2D pg_luminance;

DwPixelFlow context;

DwFlowFieldParticles particles;
DwFlowField ff_acc;
DwFlowField ff_impulse;
DwFlowField ff_attractors;

DwGLSLProgram shd_attractors;

public boolean UPDATE_PHYSICS  = true;
public boolean DISPLAY_DIST    = !true;
public boolean DISPLAY_FLOW    = !true;  
public boolean AUTO_SPAWN      = true;
public boolean APPLY_BLOOM     = true;

float mul_attractors = 5.5f;  // so fun at 8+

ParticleAttractor[] attractors;  // the blue orbs

public void settings() {
  if(START_FULLSCREEN){
    viewport_w = displayWidth;
    viewport_h = displayHeight;
    viewport_x = 0;
    viewport_y = 0;
    fullScreen(P2D);
  } else {
    viewport_w = (int) min(viewport_w, displayWidth  * 0.9f);
    viewport_h = (int) min(viewport_h, displayHeight * 0.9f);
    size(viewport_w, viewport_h, P2D);
  }
  smooth(0);
}


public void setup(){
  surface.setLocation(viewport_x, viewport_y);
  surface.setResizable(true);

  context = new DwPixelFlow(this);
  context.print();
  context.printGL();

  particles = new DwFlowFieldParticles(context, 1024 * 1024 * 4);
  
  // blue! *-*
  particles.param.col_A = new float[]{0.10f, 0.50f, 1.00f, 5};  // basic: new float[]{0.10f, 0.50f, 0.80f, 5}; , devdemo: new float[]{0.10f, 0.50f, 1.00f, 5};
  particles.param.col_B = new float[]{0.05f, 0.25f, 0.50f, 0};  // basic: new float[]{0.05f, 0.25f, 0.40f, 0}; , devdemo: half of A which means: new float[]{0.05f, 0.25f, 0.50f, 0}; , coolors.co: new float[]{0.01f, 0.18f, 0.39f, 0};
  
  particles.param.velocity_damping  = 0.985f;  // originally 1.00f
  particles.param.steps = 1;
  particles.param.shader_collision_mult = 0.2f;
  
  particles.param.size_display   = 8;
  particles.param.size_collision = 6;  // keep slightly smaller than display for overlap
  particles.param.size_cohesion  = 18;

  particles.param.mul_col = 6f;  // collision *multiplier*
  particles.param.mul_coh = 0.22f;  // cohesion
  particles.param.mul_obs = 5.75f;  // obstacles
  

  ff_acc = new DwFlowField(context); 
  ff_impulse = new DwFlowField(context);
  ff_attractors = new DwFlowField(context);
  
  shd_attractors = context.createShader("data/attractors.frag");
  
  attractors = new ParticleAttractor[2];
  attractors[0] = new ParticleAttractor(0, 1*width/3f, 1*height/2f, 20);  // left
  attractors[1] = new ParticleAttractor(0, 2*width/3f, 1*height/2f, 20);  // right
  
  resizeScene();

  createGUI();
  //toggleGUI();
  
  frameRate(1000);
}


// dynamically resize if surface-size changes
public boolean resizeScene(){

  boolean[] RESIZED = { false };
  pg_canvas     = DwUtils.changeTextureSize(this, pg_canvas    , width, height, 0, RESIZED);
  pg_obstacles  = DwUtils.changeTextureSize(this, pg_obstacles , width, height, 0, RESIZED);
  pg_impulse    = DwUtils.changeTextureSize(this, pg_impulse   , width, height, 0, RESIZED);
  pg_luminance  = DwUtils.changeTextureSize(this, pg_luminance , width, height, 0, RESIZED);

  if(RESIZED[0]){
    //setParticleColor(2);
  }
  return RESIZED[0];
}


//////////////////////////////////////////////////////////////////////////////
//
// DRAW
//
//////////////////////////////////////////////////////////////////////////////

float impulse_max = 256;
float impulse_mul = 15;
float impulse_tsmooth = 0.90f;
int   impulse_blur  = 0;

public void addImpulse(){
  
  // impulse center/velocity
  float mx = mouseX;
  float my = mouseY;
  float vx = (mouseX - pmouseX) * +impulse_mul;
  float vy = (mouseY - pmouseY) * -impulse_mul; // flip vertically
  // clamp velocity
  float vv_sq = vx*vx + vy*vy;
  float vv_sq_max = impulse_max*impulse_max;
  if(vv_sq > vv_sq_max){
    vx = impulse_max * vx / sqrt(vv_sq);
    vy = impulse_max * vy / sqrt(vv_sq);
  }
  // map velocity, to UNSIGNED_BYTE range
  final int mid = 127;
  vx = map(vx, -impulse_max, +impulse_max, 0, mid<<1);
  vy = map(vy, -impulse_max, +impulse_max, 0, mid<<1);
  // render "velocity"
  pg_impulse.beginDraw();
  pg_impulse.background(mid, mid, mid);
  pg_impulse.noStroke();
  if(mousePressed && mouseButton != LEFT){
    pg_impulse.fill(vx, vy, mid);
    pg_impulse.ellipse(mx, my, 100, 100);
  }
  pg_impulse.endDraw();
  
  // create impulse texture
  ff_impulse.resize(width, height);
  {
    Merge.TexMad ta = new Merge.TexMad(ff_impulse.tex_vel, impulse_tsmooth, 0);
    Merge.TexMad tb = new Merge.TexMad(pg_impulse,  1, -mid/255f);
    DwFilter.get(context).merge.apply(ff_impulse.tex_vel, ta, tb);
    ff_impulse.blur(1, impulse_blur);
  }
}


public void addAttractors(){
  int w = width;
  int h = height;
  
  int     attr_num = attractors.length;
  float[] attr_mass = new float[attr_num];
  float[] attr_pos  = new float[attr_num*2];
  
  for(int i = 0; i < attractors.length; i++){
    attr_mass[i] = 10f;  // originally 3, really responsive at 6
    attr_pos[i*2+0] = attractors[i].px;
    attr_pos[i*2+1] = height-1-attractors[i].py;
  }
  
  context.begin();
  ff_attractors.resize(w, h);
  context.beginDraw(ff_attractors.tex_vel);
  shd_attractors.frag.setDefine("ATTRACTOR_NUM", attr_num);
  shd_attractors.begin();
  shd_attractors.uniform1f ("attractor_mult", mul_attractors);
  shd_attractors.uniform1fv("attractor_mass", attr_num, attr_mass);
  shd_attractors.uniform2fv("attractor_pos" , attr_num, attr_pos );
  shd_attractors.drawFullScreenQuad();
  shd_attractors.end();
  context.endDraw();
  context.end();
}



public void particleSimulation(){
  int w = width;
  int h = height;

  // create acceleration texture
  ff_acc.resize(w, h);
  {
    Merge.TexMad ta = new Merge.TexMad(ff_impulse.tex_vel, 1, 0);
    Merge.TexMad tb = new Merge.TexMad(ff_attractors.tex_vel, 1, 0);
    DwFilter.get(context).merge.apply(ff_acc.tex_vel, ta, tb);
  }

  // resize buffers
  boolean resized = particles.resizeWorld(w, h);
  
  // check if obstacles changed
  boolean UPDATE_OBSTACLES = resized;
  for(int i = 0; i < attractors.length; i++){
    UPDATE_OBSTACLES |= attractors[i].moving;
  }
  
  // update obstacles, in case something changed
  if(UPDATE_OBSTACLES){
    particles.createObstacleFlowField(pg_obstacles, BG, true);
  }
  
  if(UPDATE_PHYSICS){
    // update physics
    particles.update(ff_acc);
  }
}


public void draw(){
  
  particles.param.timestep = 1/frameRate;
  
  resizeScene();
  updateScene();
  autoSpawnParticles();
  addImpulse();
  addAttractors();
  particleSimulation();

  if(!DISPLAY_DIST){
    pg_canvas.beginDraw(); 
    pg_canvas.blendMode(REPLACE);
    pg_canvas.background(1);
    pg_canvas.blendMode(BLEND);   
    pg_canvas.image(pg_obstacles, 0, 0);
    pg_canvas.endDraw();
    particles.displayParticles(pg_canvas);
  }
  
  if(DISPLAY_DIST){
    int Z = DwGLTexture.SWIZZLE_0;
    int R = DwGLTexture.SWIZZLE_R;
    int G = DwGLTexture.SWIZZLE_G;
    int B = DwGLTexture.SWIZZLE_B;
    int A = DwGLTexture.SWIZZLE_A;
    int[] RGBA = {R,G,B,A};
    
    Merge.TexMad texA = new Merge.TexMad(particles.tex_obs_dist, 0.030f * particles.param.mul_obs, 0.0f);
    Merge.TexMad texB = new Merge.TexMad(particles.tex_col_dist, 0.500f * particles.param.mul_col, 0.0f);
    Merge.TexMad texC = new Merge.TexMad(particles.tex_coh_dist, 0.005f * particles.param.mul_coh, 0.0f);

    particles.tex_obs_dist.swizzle(new int[]{R, R, R, Z});
    particles.tex_col_dist.swizzle(new int[]{R, R, R, Z});
    particles.tex_coh_dist.swizzle(new int[]{R, Z, Z, Z});
    
    DwFilter.get(context).merge.apply(pg_canvas, texA, texB, texC);
    
    particles.tex_coh_dist.swizzle(RGBA);
    particles.tex_col_dist.swizzle(RGBA);
    particles.tex_obs_dist.swizzle(RGBA);
  }

  if(DISPLAY_FLOW){
    particles.ff_sum.displayPixel(pg_canvas); 
  }
     
  if(APPLY_BLOOM){
    DwFilter filter = DwFilter.get(context);
    filter.luminance_threshold.param.threshold = 0.3f; // when 0, all colors are used
    filter.luminance_threshold.param.exponent  = 5;
    filter.luminance_threshold.apply(pg_canvas, pg_luminance);
    
    filter.bloom.setBlurLayers(10);
//      filter.bloom.gaussianpyramid.setBlurLayers(10);
    filter.bloom.param.blur_radius = 1;
    filter.bloom.param.mult   = 1.2f;    //map(mouseX, 0, width, 0, 10);
    filter.bloom.param.radius = 0.1f;//map(mouseY, 0, height, 0, 1);
    filter.bloom.apply(pg_luminance, null, pg_canvas);
  }
  
  blendMode(REPLACE);
  image(pg_canvas, 0, 0);
  blendMode(BLEND);
  
  info();
}

void info(){
  String txt_device = context.gl.glGetString(GL2.GL_RENDERER).trim().split("/")[0];
  String txt_app = getClass().getSimpleName();
  String txt_fps = String.format(Locale.ENGLISH, "[%s]   [%s]   [%d/%d]   [%7.2f fps]   [particles %,d] ", 
      txt_app, txt_device, 
      pg_canvas.width, 
      pg_canvas.height, 
      frameRate, particles.getCount()
      );
  surface.setTitle(txt_fps);
}



//////////////////////////////////////////////////////////////////////////////
//
// SCENE
//
//////////////////////////////////////////////////////////////////////////////


int[] BG      = { 0, 0, 0,  0};
int[] FG      = {16,16,16,255};
int[] FG_ATTRACTORS = {96,192,255,255};
void setFill(PGraphicsOpenGL pg, int[] rgba){
  pg.fill(rgba[0], rgba[1], rgba[2], rgba[3]);
}

void updateScene(){
  int w = pg_obstacles.width;
  int h = pg_obstacles.height;

  pg_obstacles.beginDraw();
  pg_obstacles.clear();
  pg_obstacles.noStroke();
  pg_obstacles.blendMode(REPLACE);
  pg_obstacles.rectMode(CORNER);
  setFill(pg_obstacles, FG);
  pg_obstacles.rect(0, 0, w, h);
  setFill(pg_obstacles, BG);
  pg_obstacles.rect(10, 10, w-20, h-20);
  pg_obstacles.blendMode(BLEND);
  for(int i = 0; i < attractors.length; i++){
    attractors[i].draw(pg_obstacles, FG_ATTRACTORS);
  }
  pg_obstacles.endDraw();
}


public void autoSpawnParticles(){
  if(AUTO_SPAWN && (frameCount%6) == 0){
    float px = 100;
    float py = height-100;
    
    DwFlowFieldParticles.SpawnRadial sr = new DwFlowFieldParticles.SpawnRadial();
    sr.num(1);
    sr.dim(10, 10);
    sr.pos(px, height-1 - py);
    sr.vel(0, 0);
    particles.spawn(width, height, sr);
    
//      px = width-100;
//      py = 100;
//      sr.pos(px, height-1 - py);
//      particles.spawn(width, height, sr);
  }
  
  boolean IS_GUI = cp5.isMouseOver();

  if(!IS_GUI && mousePressed){     
    if(mouseButton == RIGHT){
      int count = ceil(particles.getCount() * 0.01f);
      count = min(max(count, 1), 50000);  
      int radius = ceil(sqrt(count));
      spawn(radius, count);
    }
  }
 
}



//////////////////////////////////////////////////////////////////////////////
//
// GUI
// Helper
// Parameters
// Interaction (Mouse, Keys)
//
//////////////////////////////////////////////////////////////////////////////

static class ParticleAttractor{
  int idx = 0;
  float px = 500;
  float py = 200;
  float r  = 60;
  float dx, dy;
  boolean moving = false;
  
  public ParticleAttractor(int idx, float px, float py, float r){
    this.idx = idx;
    this.px = px;
    this.py = py;
    this.r = r;
  }
  void draw(PGraphics pg, int[] rgba){
    int cr = rgba[0];
    int cg = rgba[1];
    int cb = rgba[2];
    int ca = 255;
    
    pg.noStroke();
    pg.fill(cr,cg,cb,ca);
    pg.ellipse(px, py, r*2, r*2);
  }
  
  // did we click on/inside the attractor? (for mouse dragging only)
  public boolean inside(float mx, float my){
    dx = px - mx;
    dy = py - my;
    return (dx*dx + dy*dy) < (r*r);
  } 
  public void startMove(float mx, float my){
    moving = inside(mx, my);
  }
  
  // update attractor position (remove `if(moving)` when you deprecate mouse dragging)
  public void move(float mx, float my){
    if(moving){
      px = mx + dx;
      py = my + dy;
    }
  }
  public void endMove(float mx, float my){
    moving = false;
  }
}

public void mousePressed(){
  if(mouseButton == LEFT && !cp5.isMouseOver()){
    for(int i = 0; i < attractors.length; i++){
      attractors[i].startMove(mouseX, mouseY);
    }
  }
}
public void mouseDragged(){
  for(int i = 0; i < attractors.length; i++){
    attractors[i].move(mouseX, mouseY);
  }
}
public void mouseReleased(){
  for(int i = 0; i < attractors.length; i++){
    attractors[i].endMove(mouseX, mouseY);
  }
}

// debug
public void keyReleased(){
  if(key == 'r') reset();
  if(key == 't') UPDATE_PHYSICS = !UPDATE_PHYSICS;
  if(key == 'f') DISPLAY_FLOW   = !DISPLAY_FLOW;
  if(key == 'd') DISPLAY_DIST   = !DISPLAY_DIST;
  if(key == 'h') toggleGUI(); 
}
public void toggleGUI(){
  if(cp5.isVisible()){
    cp5.hide();
  } else {
    cp5.show();
  }
}


public void spawn(int rad, int count){
  int vw = width;
  int vh = height;
  int px = mouseX;
  int py = mouseY; py = vh - 1 - py;
  
  DwFlowFieldParticles.SpawnRadial sr = new DwFlowFieldParticles.SpawnRadial();
  sr.num(count);
  sr.dim(rad, rad);
  sr.pos(px, py);
  
  particles.spawn(vw, vh, sr);
}


public void reset(){
  particles.reset();
}
public void set_size_display(int val){
  particles.param.size_display = val;
}
public void set_size_cohesion(int val){
  particles.param.size_cohesion = val;  
}
public void set_size_collision(int val){
  particles.param.size_collision = val;  
}
public void set_velocity_damping(float val){
  particles.param.velocity_damping = val;  
}
public void set_collision_steps(int val){
  particles.param.steps = val;
}
public void set_mul_acc(float val){
  particles.param.mul_acc = val;
}
public void set_mul_col(float val){
  particles.param.mul_col = val;
}
public void set_mul_coh(float val){
  particles.param.mul_coh = val;
}
public void set_mul_obs(float val){
  particles.param.mul_obs = val;
}
public void set_shader_collision_mult(float val){
  particles.param.shader_collision_mult = val;
}

public void setParticleColor(int val){
  float r=1f, g=1f, b=1f, a=1f, s=1f;
  float[] ca = particles.param.col_A;
  switch(val){
    case 0: r = 0.10f; g = 0.50f; b = 1.00f; a = 10.0f; s = 0.50f;  break;
    case 1: r = 0.40f; g = 0.80f; b = 0.10f; a = 10.0f; s = 0.50f;  break;
    case 2: r = 0.80f; g = 0.40f; b = 0.10f; a = 10.0f; s = 0.50f;  break;
    case 3: r = 0.50f; g = 0.50f; b = 0.50f; a = 10.0f; s = 0.25f;  break;
    case 4: r = ca[0]; g = ca[1]; b = ca[2]; a =  1.0f; s = 1.00f;  break;
  }
  particles.param.col_A = new float[]{ r  , g  , b  , a };
  particles.param.col_B = new float[]{ r*s, g*s, b*s, 0 };
}

public void updateSelections(float[] val){
  int ID = 0;
  DISPLAY_DIST        = val[ID++] > 0;
  DISPLAY_FLOW        = val[ID++] > 0;
  AUTO_SPAWN          = val[ID++] > 0;
  APPLY_BLOOM         = val[ID++] > 0;
}


float mult_fg = 1f;
float mult_active = 2f;
float CR = 96;
float CG = 16;
float CB =  0;

int col_bg    ;
int col_fg    ;
int col_active;

ControlP5 cp5;

public void createGUI(){
  
  col_bg     = color(16);
  col_fg     = color(CR*mult_fg, CG*mult_fg, CB*mult_fg);
  col_active = color(CR*mult_active, CG*mult_active, CB*mult_active);
  
  int col_group = color(8,224);
  
  CColor theme = ControlP5.getColor();
  theme.setForeground(col_fg);
  theme.setBackground(col_bg);
  theme.setActive(col_active);

  cp5 = new ControlP5(this);
  cp5.setAutoDraw(true);
  
  int sx, sy, px, py;
  sx = 100; 
  sy = 14; 

  int dy_group = 20;
  int dy_item = 4;

  ////////////////////////////////////////////////////////////////////////////
  // GUI - FLUID
  ////////////////////////////////////////////////////////////////////////////
  Group group_particles = cp5.addGroup("particles");
  {
    group_particles.setHeight(20).setSize(gui_w, 370)
    .setBackgroundColor(col_group).setColorBackground(col_group);
    group_particles.getCaptionLabel().align(CENTER, CENTER);
    
    px = 15; py = 15;
    
    cp5.addButton("reset").setGroup(group_particles).plugTo(this, "reset"     ).setSize(80, 18).setPosition(px, py);

    py += sy + dy_group;
    cp5.addSlider("collision steps").setGroup(group_particles).setSize(sx, sy).setPosition(px, py)
    .setRange(0, 3).setValue(particles.param.steps).plugTo(this, "set_collision_steps")
    .snapToTickMarks(true).setNumberOfTickMarks(4);
    py += sy + dy_group;
    
    cp5.addSlider("damping").setGroup(group_particles).setSize(sx, sy).setPosition(px, py)
    .setRange(0.95f, 1.00f).setValue(particles.param.velocity_damping).plugTo(this, "set_velocity_damping");
    py += sy + dy_group;
    

    cp5.addSlider("attractors").setGroup(group_particles).setSize(sx, sy).setPosition(px, py)
    .setRange(0, 10).setValue(mul_attractors).plugTo(this, "mul_attractors");
    py += sy + dy_group;

    
    cp5.addSlider("size display").setGroup(group_particles).setSize(sx, sy).setPosition(px, py)
    .setRange(0, 20).setValue(particles.param.size_display).plugTo(this, "set_size_display");
    py += sy + dy_item;
    
    cp5.addSlider("size collision").setGroup(group_particles).setSize(sx, sy).setPosition(px, py)
    .setRange(1, 20).setValue(particles.param.size_collision).plugTo(this, "set_size_collision");
    py += sy + dy_item;
    
    cp5.addSlider("size cohesion").setGroup(group_particles).setSize(sx, sy).setPosition(px, py)
    .setRange(1, 40).setValue(particles.param.size_cohesion).plugTo(this, "set_size_cohesion");
    py += sy + dy_group;
    

    cp5.addSlider("mult collision").setGroup(group_particles).setSize(sx, sy).setPosition(px, py)
    .setRange(0.0f, 8.0f).setValue(particles.param.mul_col).plugTo(this, "set_mul_col");
    py += sy + dy_item;
    
    cp5.addSlider("mult cohesion").setGroup(group_particles).setSize(sx, sy).setPosition(px, py)
    .setRange(0.0f, 8.0f).setValue(particles.param.mul_coh).plugTo(this, "set_mul_coh");
    py += sy + dy_item;
    
    cp5.addSlider("mult obstacles").setGroup(group_particles).setSize(sx, sy).setPosition(px, py)
    .setRange(0.0f, 8.0f).setValue(particles.param.mul_obs).plugTo(this, "set_mul_obs");
    py += sy + dy_group;
    

    int ID = -1;
    cp5.addCheckBox("updateSelections").setGroup(group_particles).setSize(sy,sy).setPosition(px, py)
        .setSpacingColumn(2).setSpacingRow(2).setItemsPerRow(1)
        .addItem("DISPLAY DIST"       , ++ID).activate(DISPLAY_DIST        ? ID : 10)
        .addItem("DISPLAY FLOW"       , ++ID).activate(DISPLAY_FLOW        ? ID : 10)
        .addItem("AUTO SPAWN"         , ++ID).activate(AUTO_SPAWN          ? ID : 10)
        .addItem("APPLY BLOOM"        , ++ID).activate(APPLY_BLOOM         ? ID : 10)
      ; 
  }

  ////////////////////////////////////////////////////////////////////////////
  // GUI - ACCORDION
  ////////////////////////////////////////////////////////////////////////////
  cp5.addAccordion("acc").setPosition(gui_x, gui_y).setWidth(gui_w).setSize(gui_w, height)
    .setCollapseMode(Accordion.MULTI)
    .addItem(group_particles)
    .open()
    ;

}
