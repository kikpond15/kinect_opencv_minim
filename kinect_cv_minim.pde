import gab.opencv.*;
import org.openkinect.freenect.*;
import org.openkinect.freenect2.*;
import org.openkinect.processing.*;
import org.openkinect.tests.*;

import ddf.minim.analysis.*;
import ddf.minim.*;

Kinect kinect;
OpenCV opencv;
Minim minim;
AudioPlayer player;
FFT fft;

PImage depthImg, dst;
int minDepth =  60;
int maxDepth = 860;
float angle;

ArrayList<Contour> contours;
ArrayList<PVector> points;
int thresVal = 70;
int blurVal = 5;

void setup() {
  size(640, 480);
  kinect = new Kinect(this);
  kinect.initDepth();
  angle = kinect.getTilt();
  depthImg = new PImage(kinect.width, kinect.height);

  opencv = new OpenCV(this, kinect.width, kinect.height);
  points = new ArrayList<PVector>();

  minim = new Minim(this);
  player = minim.loadFile("deep_moon.mp3");
  player.loop();
  fft = new FFT( player.bufferSize(), player.sampleRate());
}


void draw() {
  background(0);
  fft.forward(player.mix);
  int[] rawDepth = kinect.getRawDepth();
  for (int i=0; i < rawDepth.length; i++) {
    if (rawDepth[i] >= minDepth && rawDepth[i] <= maxDepth) {
      depthImg.pixels[i] = color(255);
    } else {
      depthImg.pixels[i] = color(0);
    }
  }
  depthImg.updatePixels();
  //image(depthImg, 0, 0);

  opencv.loadImage(depthImg);
  opencv.gray();
  opencv.blur(blurVal);
  opencv.threshold(thresVal);
  dst = opencv.getOutput();
  contours = opencv.findContours();
  
  pushStyle();
  colorMode(HSB, 100);
  noFill();
  strokeWeight(6);
  int count = 0;
  for (Contour contour : contours) {
    count+=1;
    float sumX=0;
    float sumY=0;
    int pointsNum = contour.getPolygonApproximation().numPoints();
    
    for (PVector point : contour.getPolygonApproximation().getPoints()) {
      sumX += point.x;
      sumY += point.y;
    }
    PVector center = new PVector(sumX/pointsNum, sumY/pointsNum);
    
    beginShape();
    stroke(map(count, 1, pointsNum, 1, 100), 100, 100);
    for(int i=0; i<pointsNum; i++){
      PVector point = contour.getPolygonApproximation().getPoints().get(i);    
      float angle = 360/pointsNum *i;
      float fftX = (fft.getBand(100+i)*30) * cos(radians(angle) );
      float fftY = (fft.getBand(100+i)*30) * sin(radians(angle) );
      vertex(point.x+fftX, point.y+fftY);
    }
    endShape();
  }
  popStyle();
}


void keyPressed() {
  if (key == CODED) {
    if (keyCode == UP) thresVal = constrain(thresVal+10, 1, 255);
    else if (keyCode == DOWN) thresVal = constrain(thresVal-10, 1, 255);
    else if (keyCode == LEFT) blurVal = constrain(blurVal-5, 1, 50);
    else if (keyCode == RIGHT) blurVal = constrain(blurVal+5, 1, 50);
  } else if (key == 'a') {
    minDepth = constrain(minDepth+10, 0, maxDepth);
  } else if (key == 's') {
    minDepth = constrain(minDepth-10, 0, maxDepth);
  } else if (key == 'z') {
    maxDepth = constrain(maxDepth+10, minDepth, 2047);
  } else if (key =='x') {
    maxDepth = constrain(maxDepth-10, minDepth, 2047);
  }
}

void stop() {
  player.close();  //サウンドデータを終了
  minim.stop();
  super.stop();
}
