// 21 janvier 2013
// construction du modele du bonhomme
// modelisation 2 D 5 segments pour le bonhomme et 
// le deambulateur est un objet rigide a trois branches
// On ajoute un PI pour vrifier que la pose est telle que les pieds touchent le sol.











// read qualysis data
path = '/home/dune/Documents/data/AnalysisQualisys/MADN/';
pathres = 'results/essai1-';

pathMatFile = path+ 'KBM-essai1.mat';
loadmatfile(pathMatFile);

         [P1x,P1y,P1z,...
          P2x,P2y,P2z,...
          P3x,P3y,P3z,...
          P4x,P4y,P4z,...
          P5x,P5y,P5z,...
          P6x,P6y,P6z,...
          P7x,P7y,P7z,...
          P8x,P8y,P8z,...
          P9x,P9y,P9z,...
          P10x,P10y,P10z,...
          P11x,P11y,P11z,...
          P12x,P12y,P12z] = mocapData(essai1);
          


index=100;
//jeu de données de test
global Ptest ;
Ptest = [       P2x(index),P2y(index),P2z(index),1000;
                P3x(index),P3y(index),P3z(index),1000;
                P4x(index),P4y(index),P4z(index),1000;
                P5x(index),P5y(index),P5z(index),1000;
                P6x(index),P6y(index),P6z(index),1000;
                P8x(index),P8y(index),P8z(index),1000;
                P9x(index),P9y(index),P9z(index),1000;
                P10x(index),P10y(index),P10z(index),1000;
                P11x(index),P11y(index),P11z(index),1000;
                P12x(index),P12y(index),P12z(index),1000];
                
Ptest = Ptest'/1000; //tout mettre en metre
//Ptest = Ptest-[Ptest(1:3,6);0]*ones(1,12); // pour que le repère soit celui du premier pied

xset("window",10);
humanMocapPlot(Ptest);
show_pixmap();






// le robot est un robot plan sur xy
// simuInit 

//q1  = 0 ;           // angle plante du pied/sol G
q2  = 0.72546 ;  // cheville G
q3  = -0.7 ; // genou G
q4  = 0.4;   // hanche gauche 
q5  = %pi-.3;  // hanche droite
q6  = 0.2;   // genou D
q7  = 0.3926745 ;  // cheville D
//q8  = 0;            // pied D
q9  = 3.3 ; // epaule gauche
q10 = -0.8 ; // coude G
//q11 = -1.89 ;      // poignet G
q12 = 3;  // epaule droite
q13 = -0.4;  // coude droit
////q14 = 0.6577 ;      // poignet D


// nouveau modele
q0   = [q2,q3,q4,q5,q6, q7, q9, q10, q12, q13];
global d0;
d0   = H2LWSagSeg();
d0(1)=d0(1)+0.1;
d0(5)=d0(1);
d0(2)=d0(2)+0.1;
d0(4)=d0(2);
d0(3)=d0(3)+0.15;
tx0 = 0;//.0864232;//;Ptest(2,6); // position du pied dans le plan sagital 
tz0 = 0;//-0.0976375 ;//Ptest(3,6); // elevation du pied au sol 
x0  = [d0,q0,tx0,tz0]; 
P2  = computePest(x0);// le robot est un robot plan sur xy


// display
//plotHuman14dof(P2)
 plotHuman10dots(P2);
show_pixmap();




disp('Pause avant optimisation')
pause
//simplex nelder&mead
//x = fminsearch ( costfunction , x0 );

// moindre carre
[fopt,x]=leastsq(costfunction, x0);

P3 = computePest(x);
plotHuman10dots(P3);
