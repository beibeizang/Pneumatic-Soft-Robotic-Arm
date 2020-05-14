import processing.serial.*;
import KinectPV2.KJoint;
import KinectPV2.*;
KinectPV2 kinect;
Serial myPort;
PVector p1,p2,p3;
float distance;
float distance2;
float getdist;
float predist;
int val;

ArrayList<Branch> BranchCollection;
int pulse = 0;
int maxlevel = 11;
float theta;

int timeCnt = 0;
int stateTimeLimit = 300;
int timeStep = 50;

void setup(){
  size(1920, 1080, P3D);
  kinect = new KinectPV2(this);
  kinect.enableSkeletonColorMap(true);
  kinect.enableColorImg(true);
  kinect.init();
  //String portName = Serial.list()[0];
 
  myPort = new Serial(this,"COM3",9600);
  
  frameRate(maxlevel);
  //frameRate(7);
  BranchCollection = new ArrayList<Branch>();
  //Generate();
}



void draw(){
  background(0);
  image(kinect.getColorImage(), 0, 0, 320, 240); 
  //Generate();
  
   ArrayList<KSkeleton> skeletonArray =  kinect.getSkeletonColorMap();

  //individual JOINTS
  for (int i = 0; i < skeletonArray.size(); i++) {
    KSkeleton skeleton = (KSkeleton) skeletonArray.get(i);
    if (skeleton.isTracked()) {
      KJoint[] joints = skeleton.getJoints();
      
       color col  = skeleton.getIndexColor();
      fill(col);
      stroke(col);
     
       
      drawShoulder(joints[KinectPV2.JointType_ShoulderRight]);
      drawElbow(joints[KinectPV2.JointType_ElbowRight]);
      drawWrist(joints[KinectPV2.JointType_WristRight]);
       distance = dist(p1.x,p1.y,p2.x,p2.y);
       println(distance);
       distance2 = dist(p1.x,p1.y,p3.x,p3.y);
 // value = (int)distance;
   PVector centerLoc = new PVector(p2.x,p2.y);
   PVector mouseLoc = new PVector(p1.x,p1.y);
   PVector v = PVector.sub(mouseLoc,centerLoc);
   v.normalize();
   v.mult(distance);
   PVector xaxis = new PVector(1,0);
   float ang = PVector.angleBetween(v,xaxis);
   float rad = radians(int(degrees(ang)));
  // float ang2= PVector.angleBetween(xaxis,v);
  //println(degrees(ang));
   
  
  //float a = atan2(mouseY-mouseX,mouseX-mouseY);
  BranchCollection.clear();
  BranchCollection.add(new Branch(-rad,(int)distance/3,new PVector(p1.x/2+p2.x/2,p1.y/2+p2.y/2),0));
  BranchCollection.add(new Branch(PI-rad,(int)distance/3,new PVector(p1.x/2+p2.x/2,p1.y/2+p2.y/2),0));
  for( int m=0; m<maxlevel; m++){
    int limit = BranchCollection.size();
    for(int k=0; k<limit; k++){
      if (BranchCollection.get(k).Last){
       BranchCollection.add(new Branch(BranchCollection.get(k).Angle + map(distance,230,290,0.1,0.7) ,(int)random(10,50-4*m),BranchCollection.get(k).End,m));
      //BranchCollection.add(new Branch(BranchCollection.get(k).Angle + map(mouseX,0,width,0.1,0.7) ,(int)map(mouseX,0,width,50-4*i,10),BranchCollection.get(k).End,i));
      BranchCollection.add(new Branch(BranchCollection.get(k).Angle - map(distance,230,290,0.1,0.7),(int)random(10,50-4*m),BranchCollection.get(k).End,m));
      //BranchCollection.add(new Branch(BranchCollection.get(k).Angle - map(mouseX,0,width,0.1,0.7),(int)map(mouseX,0,width,10,50-4*i),BranchCollection.get(k).End,i));
        BranchCollection.get(k).Last=false;
     }
    }
  }
  }
  } 
  
  
  pulse++;
  if (pulse== maxlevel+10) pulse=0;
  if(pulse==0){
    //fill(225,200,5,255);
    fill(#F2B3B3,255);
  }
  else{
    fill(#F2B3B3,200);
    //fill(225,200,5,255);
  }
  //ellipse(270, 270, 40,40);
  for(Branch b : BranchCollection){
    b.Display(pulse);
  }
  getdist = distance2;
  if (timeCnt != stateTimeLimit){
    predist = getdist;
    timeCnt += timeStep;
    return;
  }
  float T = getdist - predist;
  
  if (T >0){
    myPort.write('I');
    println("I");
  }
  else if(T<0){
    myPort.write('D');
    println("D");
  }
  else{
    myPort.write('H');
    println("H");
  }
  predist = getdist;
  
  
}

 
 class Branch{
    Boolean Last = true;
    int Level=0;
    float Angle=0;
    int Length = 20;
    PVector Org = new PVector();
    PVector End ;
    Branch(float _angle, int _length, PVector _org, int _level){
       Org=_org;
       Angle =_angle;
       Length =_length;
       Level=_level;
       End = new PVector(Org.x + Length * cos(Angle), Org.y +Length *sin(Angle) );
       //End = new PVector(
    }
    
        
      
    void Display(int level){
      if(Level==level){
        stroke(#F2B3B3,255-10*Level);
      }
      else{
        stroke(#F2B3B3,255-10*Level);
      }
      strokeWeight(maxlevel/2-Level/2);
      line(Org.x,Org.y,End.x,End.y);
    }
}
    
  void drawBody(KJoint[] joints) {
  
  drawBone(joints, KinectPV2.JointType_SpineShoulder, KinectPV2.JointType_ShoulderRight);
  drawBone(joints, KinectPV2.JointType_SpineShoulder, KinectPV2.JointType_ShoulderLeft);
  
  // Right Arm
  drawBone(joints, KinectPV2.JointType_ShoulderRight, KinectPV2.JointType_ElbowRight);
  drawBone(joints, KinectPV2.JointType_ElbowRight, KinectPV2.JointType_WristRight);
  drawBone(joints, KinectPV2.JointType_WristRight, KinectPV2.JointType_HandRight);
 

  // Left Arm
  drawBone(joints, KinectPV2.JointType_ShoulderLeft, KinectPV2.JointType_ElbowLeft);
  drawBone(joints, KinectPV2.JointType_ElbowLeft, KinectPV2.JointType_WristLeft);
  drawBone(joints, KinectPV2.JointType_WristLeft, KinectPV2.JointType_HandLeft);
 
 
}
    //void drawBody(KJoint[] joints) {
  //drawBone(joints,KinectPV2.JointType_ShoulderRight,KinectPV2.JointType_WristRight);
//}

void drawJoint(KJoint[] joints, int jointType) {
  pushMatrix();
  translate(map(joints[jointType].getX(),0,320,0,width), map(joints[jointType].getY(),0,240,0,height), map(joints[jointType].getZ(),0.57,4.5,0,100));
  ellipse(0, 0, 25, 25);
  popMatrix();
}

void drawBone(KJoint[] joints, int jointType1, int jointType2) {
  pushMatrix();
  translate(joints[jointType1].getX(), joints[jointType1].getY(), joints[jointType1].getZ());
  ellipse(0, 0, 25, 25);
  popMatrix();
  line(joints[jointType1].getX(), joints[jointType1].getY(), joints[jointType1].getZ(), joints[jointType2].getX(), joints[jointType2].getY(), joints[jointType2].getZ());
  }

void drawShoulder (KJoint joint) {
  noStroke();
  pushMatrix();
  translate(joint.getX(), joint.getY(), joint.getZ());
  ellipse(0, 0, 50, 50);
  popMatrix();
  
 p1 = new PVector(joint.getX(),joint.getY());
}

void drawElbow (KJoint joint) {
  noStroke();
  pushMatrix();
  translate(joint.getX(), joint.getY(), joint.getZ());
  ellipse(0, 0, 50, 50);
  popMatrix();
  
  p2 = new PVector(joint.getX(),joint.getY());
}

void drawWrist (KJoint joint) {
  noStroke();
  pushMatrix();
  translate(joint.getX(), joint.getY(), joint.getZ());
  ellipse(0, 0, 50, 50);
  popMatrix();
  
  p3 = new PVector(joint.getX(),joint.getY());
}

void drawHandState(KJoint joint) {
  noStroke();
  handState(joint.getState());
  pushMatrix();
  translate(joint.getX(), joint.getY(), joint.getZ());
  ellipse(0, 0, 30, 30);
  popMatrix();
}

/*
Different hand state
 KinectPV2.HandState_Open
 KinectPV2.HandState_Closed
 KinectPV2.HandState_Lasso
 KinectPV2.HandState_NotTracked
 */
void handState(int handState) {
  switch(handState) {
  case KinectPV2.HandState_Open:
    fill(0, 255, 0);
    break;
  case KinectPV2.HandState_Closed:
    fill(255, 0, 0);
    break;
  case KinectPV2.HandState_Lasso:
    fill(0, 0, 255);
    break;
  case KinectPV2.HandState_NotTracked:
    fill(255, 255, 255);
    break;
  }
}


  
