// ------------------------------- //
// 
// the kinect data results from the code 
// ./bin/testSaveFeetTrajectories [~/Documents/data/handibio/essai1/ 2000 10000 1] 
//
// FOR PRENSENTATION PAL
//
// 1. Read data
// 2. Synchro times : search the min time and start there as 0
// 3. Align Frames
// -------------------------------- //

clear all
xdel(winsid());
getd(".");

L = 0.5;
d = 0.0992;
LPval = 0.1;
manualRotate = "OFF";

// path to KBM measurements
path = '/home/dune/Documents/data/handibio/essai1/';
pathres = 'results/WSPal-essai1-'

//-----------------------kinect data ---------------------------//
pathKinect = path + '/trajectories.dat';
Mkinect = fscanfMat(pathKinect);
firstK = 5;
lastK = $;
Mkinect = Mkinect(firstK:lastK,:);

timeKinect = Mkinect(:,1);
timeKinect = repairTime(timeKinect);
Mkinect = Mkinect*1000;

// odom data in the kinect file
x    = Mkinect(:,2);
y    = Mkinect(:,3);
theta= Mkinect(:,4);

FkdX = Mkinect(:,12);
FkdY = Mkinect(:,13);
FkdZ = Mkinect(:,14);
FMIkdX = Mkinect(:,6);
FMIkdY = Mkinect(:,7);
FMIkdZ = Mkinect(:,8);

FkgX = Mkinect(:,22);
FkgY = Mkinect(:,23);
FkgZ = Mkinect(:,24);
FMIkgX = Mkinect(:,16);
FMIkgY = Mkinect(:,17);
FMIkgZ = Mkinect(:,18);

//------------------------ qualisys data ----------------------//
pathMatFile = path+ 'qualysis/essai1.mat';
loadmatfile(pathMatFile);
//essai1
startTimeQualysis = 1359736781.106;
//essai2
//startTimeQualysis = 1359736945.821;
Mqualysis = essai1.Trajectories.Labeled.Data ;
startFrame = essai1.StartFrame;
frequency = essai1.FrameRate;

Mspacial = fscanfMat(path+'CI/spatial');
Mencoder = fscanfMat(path+'CI/encoder');
startEncoder = 1;
endEncoder = $ ;
//Mqualysis data
droite = 35;//;29;
gauche = 36;//;30;
droiteF = 1; // toe
gaucheF = 7; // toe
droiteMI = 3; // middle
gaucheMI = 9; // middle
droiteF1 = 2; // thumb
gaucheF1 = 8; // thumb
droiteH = 6; // heel
gaucheH = 12; // heel

period = 1/frequency;
delay = period*startFrame;
samples = size(Mqualysis,3);
startTimeQualysis = startTimeQualysis + delay;
timeQualysis = (startTimeQualysis:period:startTimeQualysis+samples*period)';
timeQualysis=timeQualysis(2:$);


//roue droite
rdX = Mqualysis(droite,1,:);
rdY = Mqualysis(droite,2,:);
rdZ = Mqualysis(droite,3,:);

//roue gauche
rgX = Mqualysis(gauche,1,:);
rgY = Mqualysis(gauche,2,:);
rgZ = Mqualysis(gauche,3,:);

//deduce ground truth
gtX = (rdX(1,:)+rgX(1,:))/2;
gtY = (rdY(1,:)+rgY(1,:))/2;
gtZ = (rdZ(1,:)+rgZ(1,:))/2;

//left foot
F5gX = Mqualysis(gaucheF,1,:);
F5gY = Mqualysis(gaucheF,2,:);
F5gZ = Mqualysis(gaucheF,3,:);
// right foot
F5dX = Mqualysis(droiteF,1,:);
F5dY = Mqualysis(droiteF,2,:);
F5dZ = Mqualysis(droiteF,3,:);
//left foot
F1gX = Mqualysis(gaucheF1,1,:);
F1gY = Mqualysis(gaucheF1,2,:);
F1gZ = Mqualysis(gaucheF1,3,:);
// right foot
F1dX = Mqualysis(droiteF1,1,:);
F1dY = Mqualysis(droiteF1,2,:);
F1dZ = Mqualysis(droiteF1,3,:);

FgX = (F5gX(1,:)+F1gX(1,:))/2;
FgY = (F5gY(1,:)+F1gY(1,:))/2;
FgZ = (F5gZ(1,:)+F1gZ(1,:))/2;

FdX = (F5dX(1,:)+F1dX(1,:))/2;
FdY = (F5dY(1,:)+F1dY(1,:))/2;
FdZ = (F5dZ(1,:)+F1dZ(1,:))/2;

//left foot
FMIgX = Mqualysis(gaucheMI,1,:);
FMIgY = Mqualysis(gaucheMI,2,:);
FMIgZ = Mqualysis(gaucheMI,3,:);
// right foot
FMIdX = Mqualysis(droiteMI,1,:);
FMIdY = Mqualysis(droiteMI,2,:);
FMIdZ = Mqualysis(droiteMI,3,:);

//left heel
HgX = Mqualysis(gaucheH,1,:);
HgY = Mqualysis(gaucheH,2,:);
HgZ = Mqualysis(gaucheH,3,:);
// right heel
HdX = Mqualysis(droiteH,1,:);
HdY = Mqualysis(droiteH,2,:);
HdZ = Mqualysis(droiteH,3,:);


//-------------------------IMU ---------------------------------//
//// Inertial meas. unit
pathSpacial = path+'/CI/spatial';
Mspacial = fscanfMat(pathSpacial );
first = 1;
last = $ ; 
Mspacial=Mspacial(first:last,:);
timeSpacial = Mspacial(:,1);

// -------------------------------------------------//
//          Synchronisation of times                //
//--------------------------------------------------//

//startTime = min(min(timeSpacial(1),timeKinect(1)),min(time(1),timeQualysis(1)));
startTime = min(min(timeSpacial(1),timeKinect(1)),timeQualysis(1));
//time = time - startTime;
timeQualysis = timeQualysis - startTime;
timeSpacial = timeSpacial-startTime;
timeKinect = timeKinect-startTime;

minTimeQ = timeQualysis(1);
maxTimeQ = timeQualysis($);

//----- Cut Spacial
indexMax = 1;
while(timeSpacial(indexMax) < maxTimeQ)
    indexMax = indexMax+1 ;
end
indexMax = indexMax-1;

indexMin =1;
while(timeSpacial(indexMin) < minTimeQ)
    indexMin = indexMin+1 ;
end

timeSpacial = timeSpacial(indexMin:indexMax);
Mspacial =Mspacial(indexMin:indexMax,:);

//----- Cut 
indexMax = 1;
while(timeKinect(indexMax) < maxTimeQ)
    indexMax = indexMax+1 ;
end
indexMax = indexMax-1;

//compute min time kinect
indexMin =1;
while(timeKinect(indexMin) < minTimeQ)
    indexMin = indexMin+1 ;
end

timeKinect = timeKinect(indexMin:indexMax);
timeKinect = repairTime(timeKinect);
x    = x(indexMin:indexMax);
y    = y(indexMin:indexMax);
theta= theta(indexMin:indexMax);

FkgX = FkgX(indexMin:indexMax);
FkgY = FkgY(indexMin:indexMax);
FkgZ = FkgZ(indexMin:indexMax);
//
FkdX = FkdX(indexMin:indexMax);
FkdY = FkdY(indexMin:indexMax);
FkdZ = FkdZ(indexMin:indexMax);

//------- ALIGN FRAME-------//
tx= 0;//x(1);
ty= 0;//y(1);
tz= 0;
T = [tx;ty;tz];
rx=0;
ry=0;
rgt = atan((gtY($)-gtY(1)),(gtX($)-gtX(1)));
rodo  = atan((y($)-y(1)),(x($)-x(1)));

//align deambframe
rz= -rodo;
R = rotationMatrix(rx,ry,rz);
M = homogeneousMatrix(R,T);
//change Frame
[xe,ye]=changeFrame(x,y,M);
xe=xe-xe(1);
ye=ye-ye(1);
[FkgXe,FkgYe]=changeFrame(FkgX,FkgY,M);
[FkdXe,FkdYe]=changeFrame(FkdX,FkdY,M);

// align qualisys   
rz= -rgt;
R = rotationMatrix(rx,ry,rz);
M = homogeneousMatrix(R,T);
//change Frame
[gtXe,gtYe]=changeFrame(gtX(1,:),gtY(1,:),M);
gtXe=gtXe-gtXe(1);
gtYe=gtYe-gtYe(1);
[FgXe,FgYe]=changeFrame(FgX,FgY,M);
[FdXe,FdYe]=changeFrame(FdX,FdY,M);
[HgXe,HgYe]=changeFrame(HgX(1,:),HgY(1,:),M);
[HdXe,HdYe]=changeFrame(HdX(1,:),HdY(1,:),M);

shiftX = ((FgXe(1)-FkgXe(1))+(FdYe(1)-FkdYe(1)))/2;
shiftY = ((FgYe(1)-FkgYe(1))+(FdYe(1)-FkdYe(1)))/2;

FkdXeS = FkdXe + shiftX;
FkdYeS = FkdYe + shiftY;
FkgXeS = FkgXe + shiftX;
FkgYeS = FkgYe + shiftY;



//
//graphique(1,4);
//xtitle("Feet position Motion Capture vs RGB-D Data","time (s)","z (mm)")
//plot(FdXe,FdYe,'g:');
//plot(FgXe,FgYe,'r:');
//plot(FkdXeS,FkdYeS,'g');
//plot(FkgXeS,FkgYeS,'r');
//legend("motion capture right foot","motion capture left foot", "kinect right foot", "kinect left foot",opt=5);
//xs2eps(gcf(),pathres+"feet-topview.eps");



//---- IMU----//
gyro = Mspacial (:,7);


// Interpolation
xval = linspace(timeQualysis(1),timeQualysis($),3000)';
xx_l = xval;
FkdXeS_l = interp1(timeKinect,FkdXeS,xx_l,'nearest','extrap');
FkdYeS_l = interp1(timeKinect,FkdYeS,xx_l,'nearest','extrap');
FkdZeS_l = interp1(timeKinect,FkdZ,xx_l,'nearest','extrap');
FkgXeS_l = interp1(timeKinect,FkgXeS,xx_l,'nearest','extrap');
FkgYeS_l = interp1(timeKinect,FkgYeS,xx_l,'nearest','extrap');
FkgZeS_l = interp1(timeKinect,FkgZ,xx_l,'nearest','extrap');
xe_l = interp1(timeKinect,xe,xx_l,'nearest','extrap');
ye_l = interp1(timeKinect,ye,xx_l,'nearest','extrap');

Mkinect = [timeKinect,FkdXeS,FkdYeS,FkdZ,FkgXeS,FkgYeS,FkgZ];

FdXe_l = interp1(timeQualysis,FdXe,xx_l,'nearest','extrap');
FdYe_l = interp1(timeQualysis,FdYe,xx_l,'nearest','extrap');
FdZe_l = interp1(timeQualysis,FdZ,xx_l,'nearest','extrap');
FgXe_l = interp1(timeQualysis,FgXe,xx_l,'nearest','extrap');
FgYe_l = interp1(timeQualysis,FgYe,xx_l,'nearest','extrap');
FgZe_l = interp1(timeQualysis,FgZ,xx_l,'nearest','extrap');

HdXe_l = interp1(timeQualysis,HdXe,xx_l,'nearest','extrap');
HdYe_l = interp1(timeQualysis,HdYe,xx_l,'nearest','extrap');
HdZe_l = interp1(timeQualysis,HdZ,xx_l,'nearest','extrap');
HgXe_l = interp1(timeQualysis,HgXe,xx_l,'nearest','extrap');
HgYe_l = interp1(timeQualysis,HgYe,xx_l,'nearest','extrap');
HgZe_l = interp1(timeQualysis,HgZ,xx_l,'nearest','extrap');

gtXe_l = interp1(timeQualysis,gtXe,xx_l,'nearest','extrap');
gtYe_l = interp1(timeQualysis,gtYe,xx_l,'nearest','extrap');

Mqual = [timeQualysis,FdXe,FdYe,FdZ(1,:)',FgXe,FgYe,FgZ',HdXe,HdYe,HdZ(1,:)',HgXe,HgYe,HgZ(1,:)'];

gyro_l = interp1(timeSpacial,gyro,xx_l,'nearest','extrap');

Mgyro = [timeSpacial, gyro];


M = [xx_l,FkdXeS_l,FkdYeS_l,FkdZeS_l];
M = [M, FkgXeS_l,FkgYeS_l,FkgZeS_l];
M = [M, FdXe_l,FdYe_l,FdZe_l];
M = [M, FgXe_l,FgYe_l,FgZe_l];
M = [M, xe_l,ye_l,gtXe_l,gtYe_l];
M = [M, HdXe_l,HdYe_l,HdZe_l];
M = [M, HgXe_l,HgYe_l,HgZe_l,gyro_l];


save(pathres+'positions.dat',M,Mqual,Mkinect, Mgyro);

//FgXLP=fillGapConstant(FgXe);
//FdXLP=fillGapConstant(FdXe);
////FgXLP=FgXe;
////FdXLP=FdXe;
////FdXLP= interp1(timeQualysis,FdXe,timeQualysis,'nearest','extrap');
//graphique(1,4)
//xtitle("Step determination using the sagital motion","time(s)","position (m) / velocity (mm/s)")
//start =1;
//plot(timeQualysis(start:$),FdXLP(start:$)/1000 ,'g');
////plot(timeQualysis(start:$),FgXLP(start:$)/1000,'r');
//
