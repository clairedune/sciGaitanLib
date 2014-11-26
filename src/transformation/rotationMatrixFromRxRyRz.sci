function [Rotx, Roty, Rotz] = RxRyRz(v)
// auteur CLaire dune
// Date novembre 2009
// Mouvement de rotation
// voir Diebel06
Rx = v(1) ;
Ry = v(2);
Rz = v(3) ;
Rotx = [1, 0 ,0
     0,cos(mtlb_double(Rx)),-sin(mtlb_double(Rx))
     0,sin(mtlb_double(Rx)),cos(mtlb_double(Rx))];


Roty = [cos(mtlb_double(Ry)),0,sin(mtlb_double(Ry))
     0,1,0
     -sin(mtlb_double(Ry)),0,cos(mtlb_double(Ry))];

Rotz = [cos(mtlb_double(Rz)),-sin(mtlb_double(Rz)),0
     sin(mtlb_double(Rz)),cos(mtlb_double(Rz)),0
     0,0,1];

//Rotx = [1, 0 ,0
//     0,cos(mtlb_double(Rx)),sin(mtlb_double(Rx))
//     0,-sin(mtlb_double(Rx)),cos(mtlb_double(Rx))];
//
//
//Roty = [cos(mtlb_double(Ry)),0,-sin(mtlb_double(Ry))
//     0,1,0
//     sin(mtlb_double(Ry)),0,cos(mtlb_double(Ry))];
//
//Rotz = [cos(mtlb_double(Rz)),sin(mtlb_double(Rz)),0
//     -sin(mtlb_double(Rz)),cos(mtlb_double(Rz)),0
//     0,0,1];

endfunction

function Rxyz = rotationMatrixFromRxRyRz(v)
[Rx ,Ry, Rz] = RxRyRz(v);
Rxyz=Rx*Ry*Rz;

endfunction
  

function Exyz = eulerAngleRatesMatrix(v)
// auteur CLaire dune
// Date mars 2012
// lien entre les angle d'euler et la vitesse  angulaire

// Mouvement de rotation
[Rx Ry Rz] = RxRyRz(v);

ex =[1;0;0];
ey =[0;1;0];
ez =[0;0;1];

col1 = Rz'*Ry'*ex;
col2 = Rz'*ey;
col3 = ez;
//col3 = ex;

Exyz = [col1,col2,col3];
//Exyz = Exyz.*(abs(Exyz)>100*%eps);
endfunction


function Exyz = eulerAngleRatesConj(v)
// auteur CLaire dune
// Date mars 2012
// lien entre les angle d'euler et la vitesse  angulaire

// Mouvement de rotation
[Rx Ry Rz] = RxRyRz(v);
         
ei =[1;0;0];
ej =[0;1;0];
ek =[0;0;1];

Exyz= [ei,Rx*ej,Rx*Ry*ek];
//Exyz=Exyz.*(abs(Exyz)>100*%eps);
endfunction


//------------------------------------------------//
//   Compute camera vel from dotx                 //
// date : fevrier 2013
//------------------------------------------------//

function cv = cameraVelFromDeltaX(cMo, dx, dt)
// this function gives the camera velocity
// expressed in the camera frame
// given a small displacement of the object in the camera frame
// the current camera/object pose dans the 
// sampling time dt
  
  //divide dx into two parts
  dotx          = dx/dt;
  
  // compute the Lx matrix / v=Lx dotx
  Lx = computeLx(cMo)
  
  // velocity matrix
  cv            = Lx*dotx;
  
  
endfunction


//------------------------------------------------//
//   Compute camera vel from dotx                 //
// date : fevrier 2013
//------------------------------------------------//

function Lx = computeLxInCam(cMo)
  
  //rotation matrix and vectorr
  cRo           = cMo(1:3,1:3)
  r             = RxRyRzfromRotationMatrix(cRo);
  
  //translation vector ans skew
  translation = cMo(1:3,4);
  tx = skew(translation);
  
  // AngleRateConj Conversion  Matrix
  Eijkprime     = eulerAngleRatesConj(r);
  
  // velocity matrix
  Lx           = [cRo         tx*Eijkprime
                  zeros(3,3)     Eijkprime];
  
  
endfunction

//------------------------------------------------//
//   Compute camera vel from dotx in object frame //
// date : fevrier 2013
//------------------------------------------------//

function Lx = computeLxInObj(cMo)
  
  //rotation matrix and vectorr
  cRo           = cMo(1:3,1:3)
  r             = RxRyRzfromRotationMatrix(cRo);
  
  //translation vector ans skew
  translation = cMo_m(1:3,4);
  tx = skew(translation);
  
  // AngleRateConj Conversion  Matrix
  Eijk    = eulerAngleRatesMatrix(r);
  
  // velocity matrix
  Lx           = [eye(3,3)      zeros(3,3)
                  zeros(3,3)     Eijk];
  
  
endfunction
