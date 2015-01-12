// 21 janvier 2013
// construction du modele du bonhomme
// modelisation 2 D 5 segments pour le bonhomme et 
// le deambulateur est un objet rigide a trois branches
// On ajoute un PI pour vrifier que la pose est telle que les pieds touchent le sol.



// simuInit 
q1  = 0 ;           // angle plante du pied/sol G
q2  = 10*%pi/180 ;  // cheville G
q3  = -40*%pi/180 ; // genou G
q4  = 40*%pi/180;   // hanche gauche 
q5  = 42*%pi/180 ;  // hanche droite
q6  = -40*%pi/180;   // genou D
q7  = 10*%pi/180 ;  // cheville D
q8  = 0;            // pied D
q9  = 110*%pi/180 ; // epaule gauche
q10 = -30*%pi/180 ; // coude G
q11 = -%pi/2 ;      // poignet G
q12 = 130*%pi/180;  // epaule droite
q13 = -20*%pi/180;  // coude droit
q14 = -%pi/2 ;      // poignet D

// nouveau modele
q   = [q1,q2,q3,q4,q5,q6, q7, q8, q9, q10, q11, q12, q13, q14];
d   = H2LWSagSeg();
P   = H2LWScomputeMGD(d,q);

// ancien model
q2  = [q2,q3, q4, q9,q10,q11];
d2  = [d(2),d(3),d(4),d(5),d(6),d(7),d(14),d(15)];
[M_01, M_12, M_23, M_34, M_45, M_56, A_0 ,B_0 ,C_0, P_1] = computeMGDsagitalMan(d2,q2);


// le robot est un robot plan sur xy
P3D = organiseForDisplay(P);

sommets = [P3D(:,1:$)];
y       = sommets (1,:);
x       = sommets (2,:);

sommets1=[P_1];
y1 = sommets1 (1,:);
x1 = sommets1 (2,:);



//trace de la figure
xset("window",2);
//xset("pixmap",1);
clear_pixmap()//et buffer
h1=scf(2);
h1.figure_name = "Sagital plane";
plot(x,y,'bo-');
///plot(x1,y1,'r-s');
//displayInertiaCenter(PI)
//displayCOM2d(COM)
show_pixmap();





