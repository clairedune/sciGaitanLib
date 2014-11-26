// 21 janvier 2013
// construction du modele du bonhomme
// modelisation 2 D 5 segments pour le bonhomme et 
// le deambulateur est un objet rigide a trois branches

// posture 
q1 = 10*%pi/180 ;
q2 = 30*%pi/180 ;
q3 = 40*%pi/180 ;
q4 = 160*%pi/180 ;
q5 = 40*%pi/180 ;
q6 = 90*%pi/180-(q1+q2+q3+q4+q5);//+5*%pi/180;


//-------- Parametres du robot ------- //

// les tailles sont tirees de DUMAS 2007 pour un homme
// hauteur du pied en m
d1 = .05;
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
dw = 0.6;
// hauteur deambulateur
dh = 1;

d = [d1, d2, d3, d4, d5, d6, dw, dh];
q = [q1,q2,q3,q4,q5,q6];


// ----------------- // 
// Calcul du MGD 
//-------------------//
M_base0 = homogeneousMatrixFromPos([0 0 0 0 -%pi/2 0]); // expression du premier repere dans le monde
[M_01, M_12, M_23, M_34, M_45, M_56, A_0, B_0, C_0, P_0] = computeMGDsagitalMan(d,q)

scale =.2;

h1 = createFigure3D(1,"Robot Frame position",2);
Camera3DDraw(scale,M_base0)
show_pixmap()
Camera3DDrawColor(scale,M_base0*M_01,1)
Camera3DDrawColor(scale,M_base0*M_02,2)
Camera3DDrawColor(scale,M_base0*M_03,3)
Camera3DDrawColor(scale,M_base0*M_04,4)
Camera3DDrawColor(scale,M_base0*M_05,5)
Camera3DDrawColor(scale,M_base0*M_06,6)
show_pixmap()

// le robot est un robot plan sur xy
sommets=[origin P_0 A_0 C_0];
y = sommets (1,:);
x = sommets (2,:);

//trac de la figure
xset("window",2);
xset("pixmap",1);
clear_pixmap()//et buffer
h1=scf(2);
h1.figure_name = "Sagital plane";
ha=h1.children;
ha.box="on"; 
ha.view="2d";
ha.thickness=1;
ha.foreground=0;
axe=gca(); // recupere un pointeur sur les axes
axe.data_bounds=[[-0.1 2]'; [-0.1 2]'];
axe.grid =[0.1 0.1 -0.1];
axe.auto_clear = "off" ;

plot(x,y);
show_pixmap()


// comme le robot sagital comprend 6 axes paralelles, on peut ecrire

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
              
Pdirect1_0 =  Mdirect_01 *origin;
Pdirect2_0 =  Mdirect_02 *origin;
Pdirect3_0 =  Mdirect_03 *origin;
Pdirect4_0 =  Mdirect_04 *origin;
Pdirect5_0 =  Mdirect_05 *origin;
Pdirect6_0 =  Mdirect_06 *origin;

// --- position desiree ----/
// ON VOUDRAIT QUE LA MAIN SOIT a hauteur X=dh et  une distance Y=0.3
// ON VOUDRAIT AUSSI une rotation de 90 degres suivant l'axe y du dernier repere
// d'ou la pose desiree
// ------------------------/

// pose desiree
poseDes_06 = [dh .3 0 0 %pi/2 0];


// matrice desiree
Mdes_06 = homogeneousMatrixFromPos;



