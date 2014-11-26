function oM1 = computeoM1KK(a1,d1,th1, r1)
// compute the transformation matrix between
// two susscessive frames using Khalil Kleinfinger
// notation
oM1 = zeros(4,4);

oM1(1,1) = cos(th1);
oM1(1,2) = -sin(th1);
oM1(1,3) = 0 ;
oM1(1,4) = d1;

oM1(2,1) = cos(a1)*sin(th1);
oM1(2,2) = cos(a1)*cos(th1);
oM1(2,3) = -sin(a1);
oM1(2,4) = -r1*sin(a1);

oM1(3,1) = sin(a1)*sin(th1);
oM1(3,2) = sin(a1)*cos(th1);
oM1(3,3) = cos(a1) ;
oM1(3,4) = r1*cos(a1);

oM1(4,1) = 0;
oM1(4,2) = 0;
oM1(4,3) = 0 ;
oM1(4,4) = 1;

endfunction



function [M_01, M_12, M_23, M_34, M_45, M_56, A_0 ,B_0 ,C_0, P_0] = computeMGDsagitalMan(d,q)

d1=d(1); d2=d(2); d3=d(3); d4=d(4); d5=d(5); d6=d(6); dh=d(7);dw=d(8);
q1=q(1); q2=q(2); q3=q(3); q4=q(4); q5=q(5); q6=q(6);

//d1=0;
M_01 = computeoM1KK(0,d1,q1,0); // expression du repere 1 dans le repere 0
M_12 = computeoM1KK(0,d2,q2,0); // expression du repere 2 dans le repere 1
M_23 = computeoM1KK(0,d3,q3,0); // expression du repere 3 dans le repere 2
M_34 = computeoM1KK(0,d4,q4,0); // expression du repere 4 dans le repere 3
M_45 = computeoM1KK(0,d5,q5,0); // expression du repere 5 dans le repere 4
M_56 = computeoM1KK(0,d6,q6,0); // expression du repere 6 dans le repere 5


// transformation entre le 6e repere et le point B, premier pied du deambulateur.
B_6 = [0 dh 0 1]';

// transformation entre le 6e repere et le point C, second pied du deambulateur.
C_6 = [dw dh 0 1]';

// transformation entre le 6e repere et le point A, coin haut du deambulateur
A_6 = [dw 0 0 1]';

// on exprime le centre de tous les reperes dans le repere R0 et on les dessine
origin = [0,0,0,1]'; // centre du repere de base sous le pied

P1_0 = M_01*origin; // centre du repere 1

M_02 = M_01*M_12;
P2_0 = M_02*origin; // centre du repere 2

M_03 = M_01*M_12*M_23;
P3_0 = M_03*origin; // centre du repere 3 

M_04 = M_01*M_12*M_23*M_34;
P4_0 = M_04*origin; // centre du repere 4

M_05 = M_01*M_12*M_23*M_34*M_45;
P5_0 = M_05*origin; // centre du repere 5

M_06 = M_01*M_12*M_23*M_34*M_45*M_56;
P6_0 = M_06*origin; // centre du repere 6

P_0 = [[0 0 0 1]' P1_0 P2_0 P3_0 P4_0 P5_0 P6_0];

A_0 = M_06*A_6; // position du point 0 dans le repere de base
B_0 = M_06*B_6; // position du point B dans le repere de base
C_0 = M_06*C_6; // position du point C dans le repere de base
  
  
endfunction



//----------------------------------------------//
// Human Walker segment length
//----------------------------------------------//
function dhw = H2LWSagSeg()
  // gives the lenght of the human and walker
  // segments starting from the foot
  // with two legs
  
  // les tailles sont tirees de DUMAS 2007 pour un homme
// hauteur du pied en m
d0 = 0.2; // longueur du pied gauche
d8 = 0.2; // longueur du pied droit
d1 = .1;
d7 = d1;
// hauteur du tibias
d2 = .432;
d6 = d2;
// hauteur de la jambe
d3 = .433;
d5 = d3;
// hauteur du torse
d4 = .477;
// longueur du bras
d9 = .271;
d12 = d9;
// longueur de l'avant bras
d10 = .283;
d13 = d10;
// FIXME : pour l'instant les dimension du deambulateur sont donnees  titre indicatif.
// largeur deambulateur
dw = 0.5;
// hauteur deambulateur
dh = d1+d3+d5+d7-d9-d10;
dhw = [d0,d1, d2, d3, d4, d5, d6, d7,d8, d9,d10,d12,d13, dw, dh];

endfunction 

function Points3D = H2LWScomputeMGD(d,q)

d0  = d(1) ;  // longueur du pied D
d1  = d(2);   // hauteur maleole D
d2  = d(3);   // tibias D
d3  = d(4);   // jambe D
d4  = d(5);   // torse
d5  = d(6);   // jambre G
d6  = d(7);   // tibias G  
d7  = d(8);   // hauteur pied G
d8  = d(9);   // longueur du pied G
d9  = d(10);  // bras D
d10 = d(11);  // avant bras D
d12 = d(12);  // bras G
d13 = d(13);  // avant bras G
dw  = d(14) ; // profondeur du deamb
dh  = d(15);  // hauteur du deamb

q1 = q(1);  // plante du pied/sol
q2 = q(2);  // cheville D
q3 = q(3);  // genou D
q4 = q(4);  // hanche D
q5 = q(5);  // hanche G 
q6 = q(6);  // genou G
q7 = q(7);  // cheville G

M_0_1   = computeoM1KK(0,0,q1,0); // expression du pied dans
M_1_2   = computeoM1KK(0,d1,q2,0); // expression du repere 1 dans le repere 0
M_2_3   = computeoM1KK(0,d2,q3,0); // expression du repere 2 dans le repere 1
M_3_4   = computeoM1KK(0,d3,q4,0); // expression du repere 3 dans le repere 2
M_3_5   = computeoM1KK(0,d3,q5-q4+%pi,0); // expression du repere 4 dans le repere 3
M_5_6   = computeoM1KK(0,d5,-q6,0); // expression du repere 5 dans le repere 4
M_6_7   = computeoM1KK(0,d6,-q7,0); // expression du repere 6 dans le repere 5
M_7_8   = computeoM1KK(0,d7,%pi,0); // expression du repere 6 dans le repere 5

M_0_8 = M_0_1*M_1_2*M_2_3*M_3_5*M_5_6*M_6_7*M_7_8;

M_4_9   = computeoM1KK(0,d4,q9,0);//epaule
M_9_10  = computeoM1KK(0,d10,q10,0);//coude
M_10_11 = computeoM1KK(0,0.2,q11,0);//main
M_4_12  = computeoM1KK(0,d4,q12,0);//epaule
M_12_13 = computeoM1KK(0,d13,q13,0);//coude
M_13_14 = computeoM1KK(0,0.2,q14,0);//main



// transformation entre le 6e repere et le point B, premier pied du deambulateur.
B_6 = [0 dh 0 1]';
// transformation entre le 6e repere et le point C, second pied du deambulateur.
C_6 = [dw dh 0 1]';
// transformation entre le 6e repere et le point A, coin haut du deambulateur
A_6 = [dw 0 0 1]';
// on exprime le centre de tous les reperes dans le repere R0 et on les dessine
origin = [0,0,0,1]'; // centre du repere de base sous le pied

/// le pied au sol
d0 = d(1);
Pp1_0 = [0, -d0/3 ,  0, 1];
Pp2_0 = [ d1,   0  , 0, 1];
Pp3_0 = [0, 2*d0/3,  0, 1];

P1_0  = M_0_1*origin;
P2_0  = M_0_1*M_1_2*origin;
P3_0  = M_0_1*M_1_2*M_2_3*origin;
P4_0  = M_0_1*M_1_2*M_2_3*M_3_4*origin;
P5_0  = M_0_1*M_1_2*M_2_3*M_3_5*M_5_6*origin;
P6_0  = M_0_1*M_1_2*M_2_3*M_3_5*M_5_6*M_6_7*origin;


P7_0  = M_0_8*origin;
Pp1_8 = M_0_8*Pp1_0';
Pp2_8 = M_0_8*Pp2_0';
Pp3_8 = M_0_8*Pp3_0';

//tronc
P8_0  = M_0_1*M_1_2*M_2_3*M_3_4*M_4_9*origin;
disp(P8_0);

//bras G
P9_0  = M_0_1*M_1_2*M_2_3*M_3_4*M_4_9*M_9_10*origin;//coude
P10_0  = M_0_1*M_1_2*M_2_3*M_3_4*M_4_9*M_9_10*M_10_11*origin;//poignet

P11_0  = M_0_1*M_1_2*M_2_3*M_3_4*M_4_12*origin;
P12_0  = M_0_1*M_1_2*M_2_3*M_3_4*M_4_12*M_12_13*origin;
P13_0  = M_0_1*M_1_2*M_2_3*M_3_4*M_4_12*M_12_13*M_13_14*origin;

Points3D   = [P1_0,...
              P2_0, P3_0, P4_0,...
              P5_0, P6_0,P7_0,P8_0,P9_0,P10_0, P11_0, P12_0, P13_0,...
              Pp1_0',Pp2_0', Pp3_0',Pp1_8,Pp2_8,Pp3_8];   
endfunction

// on reorganise les points pour les afficher avec un plot
function Points3D =organiseForDisplay(P)

P1_0 = P(:,1);
P2_0 = P(:,2);
P3_0 = P(:,3);
P4_0 = P(:,4);
P5_0 = P(:,5);
P6_0 = P(:,6);
P7_0 = P(:,7);
P8_0 = P(:,8);
P9_0 = P(:,9);
P10_0 = P(:,10);
P11_0 = P(:,11);
P12_0 = P(:,12);
P13_0 = P(:,13);
Pp1_0 = P(:,14);
Pp2_0 = P(:,15);
Pp3_0 = P(:,16);
Pp1_8 = P(:,17);
Pp2_8 = P(:,18);
Pp3_8 = P(:,19);

Points3D   = [Pp3_0,Pp1_0,Pp2_0, Pp3_0,... //pied
              P2_0, P3_0, P4_0, ...//jambe
              P5_0, P6_0,... // jambe
              Pp1_8,Pp3_8,Pp2_8, ...  //pied
              P6_0,P5_0,P4_0, P8_0, ...//tronc
              P9_0, P10_0,P9_0,P8_0,... //bras
              P11_0, P12_0, P13_0// bras
              ];   

  
endfunction



function [Mdirect_01,   Mdirect_02, Mdirect_03, Mdirect_04, Mdirect_05, Mdirect_06, A_0, B_0, C_0, P_0] = computeMGDsagitalManDir(d,q)
  
  d1=d(1); d2=d(2); d3=d(3); d4=d(4); d5=d(5); d6=d(6); dh=d(7);dw=d(8);
  q1=q(1); q2=q(2); q3=q(3); q4=q(4); q5=q(5); q6=q(6);
  
  // transformation entre le 6e repere et le point B, premier pied du deambulateur.
  B_6 = [0 dh 0 1]';

  // transformation entre le 6e repere et le point C, second pied du deambulateur.
  C_6 = [dw dh 0 1]';

  // transformation entre le 6e repere et le point A, coin haut du deambulateur
  A_6 = [dw 0 0 1]';

  
  
  
  Mdirect_01 = [cos(q1) -sin(q1) 0 d1 
              sin(q1) cos(q1) 0 0
              0 0 1 0
              0 0 0 1];

  t2x = d1+d2*cos(q1);
  t2y = d2*sin(q1);
  Mdirect_02 = [cos(q1+q2) -sin(q1+q2) 0 t2x 
              sin(q1+q2) cos(q1+q2) 0 t2y
              0 0 1 0
              0 0 0 1];     


  t3x = t2x + cos(q1+q2)*d3;
  t3y = t2y + d3*sin(q1+q2);

  Mdirect_03 = [cos(q1+q2+q3) -sin(q1+q2+q3) 0 t3x 
              sin(q1+q2+q3) cos(q1+q2+q3) 0 t3y
              0 0 1 0
              0 0 0 1];  
              
              
  t4x = t3x + cos(q1+q2+q3)*d4;
  t4y = t3y + d4*sin(q1+q2+q3);

  Mdirect_04 = [cos(q1+q2+q3+q4) -sin(q1+q2+q3+q4) 0 t4x
              sin(q1+q2+q3+q4) cos(q1+q2+q3+q4) 0 t4y
              0 0 1 0
              0 0 0 1];  

  t5x = t4x + cos(q1+q2+q3+q4)*d5;
  t5y = t4y + d5*sin(q1+q2+q3+q4);
  Mdirect_05 = [cos(q1+q2+q3+q4+q5) -sin(q1+q2+q3+q4+q5) 0 t5x
              sin(q1+q2+q3+q4+q5) cos(q1+q2+q3+q4+q5) 0 t5y
              0 0 1 0
              0 0 0 1];  

  t6x = t5x + cos(q1+q2+q3+q4+q5)*d6;
  t6y = t5y + d6*sin(q1+q2+q3+q4+q5);            
  Mdirect_06 = [cos(q1+q2+q3+q4+q5+q6) -sin(q1+q2+q3+q4+q5+q6) 0 t6x
              sin(q1+q2+q3+q4+q5+q6) cos(q1+q2+q3+q4+q5+q6) 0 t6y
              0 0 1 0
              0 0 0 1];  
              
  origin = zeros(4,1);              
              
  Pdirect1_0 =  Mdirect_01 *origin;
  Pdirect2_0 =  Mdirect_02 *origin;
  Pdirect3_0 =  Mdirect_03 *origin;
  Pdirect4_0 =  Mdirect_04 *origin;
  Pdirect5_0 =  Mdirect_05 *origin;
  Pdirect6_0 =  Mdirect_06 *origin;
  
  P_0 = [[0 0 0 1]' Pdirect1_0 Pdirect2_0 Pdirect3_0 Pdirect4_0 Pdirect5_0 Pdirect6_0];

  A_0 = Mdirect_06*A_6; // position du point 0 dans le repere de base
  B_0 = Mdirect_06*B_6; // position du point B dans le repere de base
  C_0 = Mdirect_06*C_6; // position du point C dans le repere de base
  
  
endfunction  


function J06 = computeJ06SagMan(d,q)
// assuming the 3 values of the effector pose are
// Px, Py, theta


  d1=d(1); d2=d(2); d3=d(3); d4=d(4); d5=d(5); d6=d(6); dh=d(7);dw=d(8);
  q1=q(1); q2=q(2); q3=q(3); q4=q(4); q5=q(5); q6=q(6);
  
  
  
  dTxdq1= -d2*sin(q1) - d3*sin(q1+q2) - d4*sin(q1+q2+q3)-d5*sin(q1+q2+q3+q4)-d6*sin(q1+q2+q3+q4+q5); 
  dTxdq2= -d3*sin(q1+q2) - d4*sin(q1+q2+q3)-d5*sin(q1+q2+q3+q4)-d6*sin(q1+q2+q3+q4+q5); 
  dTxdq3= -d4*sin(q1+q2+q3)-d5*sin(q1+q2+q3+q4)-d6*sin(q1+q2+q3+q4+q5); 
  dTxdq4= -d5*sin(q1+q2+q3+q4)-d6*sin(q1+q2+q3+q4+q5); 
  dTxdq5= -d6*sin(q1+q2+q3+q4+q5); 
  dTxdq6= 0; 

  dTydq1= d2*cos(q1)+d3*cos(q1+q2)+d4*cos(q1+q2+q3)+d5*cos(q1+q2+q3+q4)+d6*cos(q1+q2+q3+q4+q5);
  dTydq2= d3*cos(q1+q2)+d4*cos(q1+q2+q3)+d5*cos(q1+q2+q3+q4)+d6*cos(q1+q2+q3+q4+q5);
  dTydq3= d4*cos(q1+q2+q3)+d5*cos(q1+q2+q3+q4)+d6*cos(q1+q2+q3+q4+q5);
  dTydq4= d5*cos(q1+q2+q3+q4)+d6*cos(q1+q2+q3+q4+q5);
  dTydq5= d6*cos(q1+q2+q3+q4+q5);  
  dTydq6= 0;
  
  dThetadq1 = 1;
  dThetadq2 = 1;  
  dThetadq3 = 1;
  dThetadq4 = 1;
  dThetadq5 = 1;
  dThetadq6 = 1;
  
  J06 = [dTxdq1 dTxdq2 dTxdq3 dTxdq4 dTxdq5 dTxdq6;
         dTydq1 dTydq2 dTydq3 dTydq4 dTydq5 dTydq6;
         dThetadq1 dThetadq2 dThetadq3 dThetadq4 dThetadq5 dThetadq6;]

  
  
endfunction





//----------------------------------------------//
// Hiuman Walker SEgment Length Model 
//----------------------------------------------//
function dhw = humanWalkerSagSeg()
  // gives the lenght of the human and walker
  // segments starting from the foot
  
  // les tailles sont tirees de DUMAS 2007 pour un homme
// hauteur du pied en m
d1 = .1;
// hauteur du tibias
d2 = .432;
// hauteur de la jambe
d3 = .433;
// hauteur du torse
d4 = .477;
// longueur du bras
d5 = .271;
// longueur de l'avant bras
d6 = .283;
// FIXME : pour l'instant les dimension du deambulateur sont donnees  titre indicatif.
// largeur deambulateur
dw = 0.5;
// hauteur deambulateur
dh = d1+d2+d3+d4-d5-d6;
dhw = [d1, d2, d3, d4, d5, d6, dh, dw];

endfunction  



 

//----------------------------------------------//
// Hiuman Walker Sagital Joint limit
//----------------------------------------------//
function qlimit = humanWalkerSagJointLimit()
qlimit = [ -20*%pi/180      50*%pi/180
           -135*%pi/180     -10*%pi/180
           10*%pi/180    120*%pi/180
           0              240*%pi/180
           -150*%pi/180    -30*%pi/180
           -20*%pi/180-%pi/2     30*%pi/180-%pi/2]'; 
           //-20*%pi/180-%pi/2     30*%pi/180-%pi/2]';    
endfunction

function qlimit = H2LWSagJointLimit()
qlimit = [ -%pi/2             %pi/2            // angle entre la plante du pied et le sol
           -20*%pi/180        50*%pi/180       // premiere cheville q2
           -135*%pi/180       -10*%pi/180      // genou gauche q3
           10*%pi/180         120*%pi/180      // hanche gauche q4
           10*%pi/180         120*%pi/180      // hanche droite q5
           -135*%pi/180       -10*%pi/180      // genou droit q6
           -20*%pi/180        50*%pi/180       // cheville droite q7
           -%pi/2             %pi/2            // angle entre la plante du pied et le sol q8
           0                  240*%pi/180      // epaule gauche q9
           -150*%pi/180       -30*%pi/180      // coude gauche q10
           -20*%pi/180-%pi/2  30*%pi/180-%pi/2 //poignet gauche q11
           0                  240*%pi/180      // epaule droite q12
           -150*%pi/180       -30*%pi/180      // coude droit q13
           -20*%pi/180-%pi/2  30*%pi/180-%pi/2 //poignet droit q14
           ]';  
            
endfunction

//----------------------------------------------//
// Compute Inertia Center Positions
//----------------------------------------------//

function [PI, COM, m] = humanWalkerSagInertia(q,d,masse)
//  // q sont les positions articulaires
  // d sont les longueurs des segments
  // masse est la masse totale de la personne
  // on retourne la position des centres de masse dans le repere Ro
  // et la valeur des masses calculee d'apresWINTER 90.
  
  d1=d(1); d2=d(2); d3=d(3); d4=d(4); d5=d(5); d6=d(6); dh=d(7);dw=d(8);
  q1=q(1); q2=q(2); q3=q(3); q4=q(4); q5=q(5); q6=q(6);
  //FIXME : c'est pas optimal de calculer a nouveau MGD ici
  [M_01, M_12, M_23, M_34, M_45, M_56, A_0, B_0, C_0, P_0] = computeMGDsagitalMan(d,q);


  // Masse du pied exprime dans le repere Xo du pied
  m0 = 0.0145*masse;
  PI_00 = [0.429*d1 0 0 1]';
  
  // Masse de la jambe exprimee dans le repere X1 de la jambe
  m1 = masse * 0.0465;
  PI_11 = [d2*0.567 0 0 1]';
  PI_01 = M_01*PI_11;
  
  // Masse de la cuisse exprimee dans le repere X2 de la cuisse
  m2 = masse * 0.1;
  PI_22 = [d3*0.567 0 0 1]';
  PI_02 = M_01*M_12*PI_22;
  

 // Masse du Tronc exprimee dans le repere X3 du tronc
  m3 = masse * 0.578;
  PI_33 = [d4*0.396 0 0 1]';
  PI_03 = M_01*M_12*M_23*PI_33;



// Masse du bras exprimee dans le repere X4 u bras
  m4 = masse * 0.028;
  PI_44 = [d5*0.436 0 0 1]';
  PI_04 = M_01*M_12*M_23*M_34*PI_44;
  
  
// Masse de l'avant bras exprimee dans le repere X5 de l'avant bras
  m5 = masse * (0.016+0.006);
  PI_55 = [d6*0.43 0 0 1]';
  PI_05 = M_01*M_12*M_23*M_34*M_45*PI_55;
  
  PI = [PI_00 PI_01 PI_02 PI_03 PI_04 PI_05];
  m = [2*m0 2*m1 2*m2 m3 2*m4 2*m5];
  
  
  
  COM = PI*m';
  COM(4)=1;
  COM=COM/masse;
  
    
  
endfunction


//----------------------------------------------//
// Display CoM 2d
//----------------------------------------------//
function displayCOM2d(COM)
plot([COM(2) COM(2)], [COM(1) 0],'x--r');
endfunction 

//----------------------------------------------//
// Display Inertia center 2d
//----------------------------------------------//
function displayInertiaCenter(PI)
plot(PI(2,:),PI(1,:),'or');
endfunction 

//----------------------------------------------//
// Display Forces 2d
//----------------------------------------------//
function displayAllForces(Fpts,P_0)
  
  for i=1:7
    // on exprime l'extremite du vecteur force normalise
    if(abs(norm(Fpts(:,i)))>0)
      Fdraw = Fpts(:,i)/(5*norm(Fpts(:,i))) + P_0(1:3,i);
      plot([Fdraw(2) P_0(2,i)],[Fdraw(1) P_0(1,i)],'k-.');
    end
  end

endfunction  

//----------------------------------------------//
// Display Ext Forces 2d
//----------------------------------------------//
function displayExtForces(Fpts,P_0)
  
    // on exprime l'extremite du vecteur force normalise
    if(abs(norm(Fpts(:,1)))>0)
      Fdraw = Fpts(:,1)/(5*norm(Fpts(:,1))) ;//+ 
      Fdraw =-Fdraw;
      Fdraw =Fdraw + P_0(1:3,1);
      plot([Fdraw(2) P_0(2,1)],[Fdraw(1) P_0(1,1)],'r-');
    end
    // on exprime l'extremite du vecteur force normalise
    if(abs(norm(Fpts(:,$)))>0)
      Fdraw = Fpts(:,$)/(5*norm(Fpts(:,$))) + P_0(1:3,$);
      plot([Fdraw(2) P_0(2,$)],[Fdraw(1) P_0(1,$)],'r-');
      
    end
 

endfunction

//----------------------------------------------//
// Display Ext Forces 2d
//----------------------------------------------//
function displayOneForces(Fpts,P_0)
  
    // on exprime l'extremite du vecteur force normalise
    if(abs(norm(Fpts(:,1)))>0)
      Fdraw = Fpts(:,1)/(5*norm(Fpts(:,1))) ;//+ 
      Fdraw =-Fdraw;
      Fdraw =Fdraw + P_0(1:3,1);
      plot([Fdraw(2) P_0(2,1)],[Fdraw(1) P_0(1,1)],'r-');
    end
    // on exprime l'extremite du vecteur force normalise
    if(abs(norm(Fpts(:,$)))>0)
      Fdraw = Fpts(:,$)/(5*norm(Fpts(:,$))) + P_0(1:3,$);
      plot([Fdraw(2) P_0(2,$)],[Fdraw(1) P_0(1,$)],'r-');
      
    end
 

endfunction 

//----------------------------------------------//
// Compute Propagation Force FromWrist
//----------------------------------------------//

function F= humanWalkerForceWrist(masses, Fext)
  // on propage les force sur chaque
  // axe en partant du poignet
  
  // g
  g=9.81;
  
  //force courante
  Fisurj = Fext;

  // vecteurs de stockage
  F=[Fisurj];
  
  for k = 0 : 5
    
    i=7-k; // index courant
    j=i-1; // index corps precedent
    
    // calcul de la force
    Poidsj = [-masses(j)*g 0 0]';
    Fjsuri = -Poidsj - Fisurj;
    
    //mise a jour
    Fisurj = -Fjsuri;
    F      =[Fisurj F];
  end
  
endfunction  



function [F,G]= humanWalkerForcTorqWrist(masses, Fext, Gext, P_0, PI_0)
  // on propage les force et le moment sur chaque
  // axe en partant du poignet
  // masses est le vecteur des masses des segments
  
  // g
  g=9.81;
  
  //force courante
  Fisurj = Fext;
  //couple courant
  Gi = Gext;
  
  // vecteurs de stockage
  F=[Fisurj];
  G=[Gi];
  
  for k = 0 : 5
    
    i=7-k; // index courant
    j=i-1; // index corps precedent
    
    // calcul de la force
    Poidsj = [-masses(j)*g 0 0]';
    Fjsuri = -Poidsj - Fisurj;
  
    // points d'application des forces
    // par rapport  l'axe j
    Icenter  = P_0(1:3,j)-PI_0(1:3,j);
    SegExtrem = P_0(1:3,j)-P_0(1:3,i);
 
    // calcul du moment
    Gj = crossProd(SegExtrem,Fisurj) + crossProd(Icenter,Poidsj)+Gi;

    Fisurj = -Fjsuri;
    Gi     = Gj;
    F      =[Fisurj F];
    G      =[Gi G];
   
  end
  
  
endfunction  

//----------------------------------------------//
// Compute Couples et forces pour le modele Human4
//----------------------------------------------//
function [F0,G0]= human4WalkerForcTorqWrist(masses, Fext, Gext, P_0, PI_0)
  // on propage les force et le moment sur chaque
  // axe en partant du poignet
  // masses est le vecteur des masses des segments
  
  // g
  g=9.81;
   
  // masse total
  mtotale=sum(masses);
  
  // poids
  Poids = [-mtotale*g 0 0]';
  F0 = -Fext-Poids;
  Icenter  = -PI_0(1:3,4);
  SegExtrem = -P_0(1:3,7);
  
  G0 = crossProd(SegExtrem,Fext) + crossProd(Icenter,Poids)+Gext;
  
  
endfunction  

//----------------------------------------------//
// Compute Propagation Force From Foot
//----------------------------------------------//

function F = humanWalkerForceFoot(masses, Fext)
  // on propage les force sur chaque
  // axe en partant du pied
  
  // g
  g=9.81;
  
  // Force au sol
  F0=Fext;
  
   // Force a la cheville
  // --- Poids du pied
  Ppied = [-masses(1)*g 0 0]';
  // force a la cheville
  F1 = -Ppied-F0;  

 
  // Force au genou
  // --- Poids jambe
  Pjambe = [-masses(2)*g 0 0]';
  // --- Force genou
  F0sur1=-F1;
  F2 = -Pjambe-F0sur1;
  
   // Force hanche
  // --- Poids cuisse
  Pcuisse = [-masses(3)*g 0 0]';
  // --- Force hanche
  F1sur2=-F2;
  F3 = -Pcuisse-F1sur2;
  
  // Force epaule
  // --- Poids tronc
  Ptronc = [-masses(4)*g 0 0]';
  // --- Forceepaule
  F2sur3=-F3;
  F4 = -Ptronc-F2sur3;
  
  // Force coude
  // --- Poids bras
  Pbras = [-masses(5)*g 0 0]';
  // --- Forceepaule
  F3sur4=-F4;
  F5 = -Pbras-F3sur4;
  
  // Force poignet
  // --- Poids avant bras
  Pavbras = [-masses(6)*g 0 0]';
  // --- Forceepaule
  F4sur5=-F5;
  F6 = -Pavbras-F4sur5;
 
  
  // construction du vecteur
  F = [F0 -F1 -F2 -F3 -F4 -F5 F6];
  
  
endfunction 

//----------------------------------------------//
// Compute ZMP
//----------------------------------------------//

function ZMP_0 = humanWalkerZMP(masses, F, G, P_0, PI)
 
 mPied = masses(1);
 Fcheville = F(:,2);
 Gcheville = G(:,2);
 Pcheville = P_0(:,2);
 PIpied = PI(:,1);

 
 // forces
 g = 9.81; 
 
 //disp('Cy')
// disp(PIpied(2))
// disp('Fy')
// disp(Fcheville(2))
// disp('Fx')
// disp(Fcheville(1))
//  disp('Py')
// disp(Pcheville(2))
// disp('Px')
// disp(Pcheville(1))
// disp('GamaZ')
// disp(Gcheville(3))

 y = (mPied*g*PIpied(2)+...
 Fcheville(2)*Pcheville(1)-...
 Fcheville(1)*Pcheville(2)-...
 Gcheville(3))/(mPied*g-Fcheville(1)); 
 
 ZMP_0=[0 y 0]';
  
endfunction


//----------------------------------------------//
// Compute ZMP for model Huamn 4
//----------------------------------------------//

function ZMP_0 = human4WalkerZMP(masses, Fext, Gext, P_0, PI)
 
 mtotale = sum(masses);
 Fpoignet = Fext;
 Gpoignet = Gext;
 Ppoignet = P_0(:,7);
 PItronc = PI(:,4);

 
 // forces
 g = 9.81; 

 y = ( mtotale*g*PItronc(2)+...
 Fpoignet(2)*Ppoignet(1)-...
 Fpoignet(1)*Ppoignet(2)-...
 Gpoignet(3))/(mtotale*g-Fpoignet(1)); 
 
 ZMP_0=[0 y 0]';
  
endfunction

//----------------------------------------------//
// Compute ZMP from Fext and Gext
//----------------------------------------------//
function [ZMP_0,P_0] = humanWalkerZMPdirect(masses, Fext, Gext, P_0, PI)
  [F,G]= humanWalkerForcTorqWrist(masses, Fext, Gext, P_0, PI);
  ZMP_0 = humanWalkerZMP(m, F, G, P_0, PI);
  P_0(:,1)= [ZMP_0 ;1];
endfunction  

////----------------------------------------------//
//// Compute Propagation Force and Torque FromWrist
////----------------------------------------------//
//
//function [F,G]= humanWalkerForcTorqWrist(masses, Fext, Gext, P_0, PI_0)
//  // on propage les force et le moment sur chaque
//  // axe en partant du poignet
//  // masses est le vecteur des masses des segments
//  
//  // g
//  g=9.81;
//  
//  //force au poignet
//  Fextsur6 = Fext;
//
//  ///-------------------------//
//  // Segment 6 - avant bras
//  //--------------------------//
//  // Force au coude
//  // --- Poids avant bras seg 6
//  Pavbras = [-masses(6)*g 0 0]';
//  // --- Force avant bras
//  F5sur6 = -Pavbras-Fextsur6;
//  
//  // Couple au coude
//  // --- Position du centre d'inertie 
//  // de l'avant bras
//  // par rapport au coude
//  Icenter = P_0(1:3,6)-PI_0(1:3,6);
//  // --- Position du poignet par rapport au coude
//  SegExtrem =P_0(1:3,6)-P_0(1:3,7);
//  // --- Moment exerce sur le coude
//  Gcoude = crossProd(SegExtrem,Fextsur6) + crossProd(Icenter,Pavbras)+Gext;
//
//  
//  
//  //-------------------------//
//  // Segment 5 - bras
//  //--------------------------//
//  // Force a l'epaule
//  // --- Poids du bras seg 5
//  Pbras = [-masses(5)*g 0 0]';
//  // --- Force du bras
//  F6sur5 = -F5sur6;
//  F4sur5 = -Pbras-F6sur5;
//  
//  // Couple a l'epaule
//  // --- Position du centre d'inertie 
//  // du bras
//  // par rapport a l'epaule
//  Icenter = P_0(1:3,5)-PI_0(1:3,5);
//  // --- Position du coude par rapport a l'epaule
//  SegExtrem =P_0(1:3,5)-P_0(1:3,6);
//  // --- Moment exerce sur l'epaule
//  Gepaule = crossProd(SegExtrem,F6sur5) + crossProd(Icenter,Pbras)+Gcoude;
//
//  //-------------------------//
//  // Segment 4 - tronc
//  //--------------------------//  
//  // Force a la hanche
//  // --- Poids du tronc
//  Ptronc = [-masses(4)*g 0 0 ]';
//  // --- Force hanche
//  F5sur4 = -F4sur5;
//  F3sur4 = -Ptronc-F5sur4;
//  
//  // Couple a la hanche
//  // --- Position du centre d'inertie 
//  // de du tronc
//  // par rapport a la hanche
//  Icenter = P_0(1:3,4)-PI_0(1:3,4);
//  // --- Position l'epaule/hanche
//  SegExtrem =P_0(1:3,4)-P_0(1:3,5);
//  // --- Moment exerce sur la hanche
//  Ghanche = crossProd(SegExtrem,F5sur4) + crossProd(Icenter,Ptronc)+Gepaule;
//  
//  //-------------------------//
//  // Segment 3 - cuisse
//  //--------------------------// 
//  // Force au genou
//  // ---- Poids cuisse
//  Pcuisse = [-masses(3)*g 0 0]';
//  // --- Force cuisse
//  F4sur3 = -F3sur4;
//  F2sur3 = -Pcuisse - F4sur3;
//  
//  // Couple au genou
//  // --- Position du centre d'inertie 
//  // de la cuisse
//  // par rapport au genou
//  Icenter = P_0(1:3,3)-PI_0(1:3,3);
//  // --- Position du poignet par rapport a l'epaule
//  SegExtrem =P_0(1:3,3)-P_0(1:3,4);
//  // --- Moment exerce sur l'epaule
//  Ggenou = crossProd(SegExtrem,F4sur3) + crossProd(Icenter,Pcuisse)+Ghanche;
//  
//  //-------------------------//
//  // Segment 2 - jambe
//  //--------------------------// 
//  
//  // Force a la cheville
//  // --- Poids jambe
//  Pjambe = [-masses(2)*g 0 0]';
//  // --- Force cheville
//  F3sur2=-F2sur3;
//  F1sur2 = -Pjambe-F3sur2;
//  // Couple a la cheville
//  // --- Position du centre d'inertie 
//  // de la jambe
//  // par rapport a la cheville
//  Icenter = P_0(1:3,2)-PI_0(1:3,2);
//  // --- Position du poignet par rapport a l'epaule
//  SegExtrem =P_0(1:3,2)-P_0(1:3,3);
//  // --- Moment exerce sur l'epaule
//  Gcheville= crossProd(SegExtrem,F3sur2) + crossProd(Icenter,Pjambe)+Ggenou;
//  
//  //-------------------------//
//  // Segment 1 - pieds
//  //--------------------------// 
//  // Force au sol
//  // --- Poids du pied
//  Ppied = [-masses(1)*g 0 0]';
//  // force au sol
//  F2sur1 = -F1sur2;
//  F0sur1 = -Ppied-F2sur1;
//  F1sur0 = -F0sur1;
//  // Couple au sol
//  // --- Position du centre d'inertie 
//  // du pied
//  // par rapport a lorigine
//  Icenter = P_0(1:3,1)-PI_0(1:3,1);
//  // --- Position du poignet par rapport a l'epaule
//  SegExtrem =P_0(1:3,1)-P_0(1:3,2);
//  // --- Moment exerce sur l'epaule
//  Gsol = crossProd(SegExtrem,F2sur1) + crossProd(Icenter,Ppied)+Gcheville;
//  
//  // construction du vecteur
//  F = [F1sur0 F2sur1 F3sur2 F4sur3 F5sur4 F6sur5 Fextsur6];
//  G = [Gsol Gcheville Ggenou Ghanche Gepaule Gcoude Gext]; 
//  
//  
//endfunction  


