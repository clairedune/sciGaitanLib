//---------------------------//
// Predictive Visual Servoing
// With a free 3d camera
// December 11
// Dune 
//---------------------------//
function predictiveControl()

OPT_DISPLAY   = %T;
OPT_SAVE		  = %F;
OPT_CONTROL   = 'AV';
// OPT_CONTROL = 'PRED';
path_exp      = '/Users/dune/devel-src/scilab/AsserVisu/results/2011-12-1/';
name_exp      = 'exp1';
save_path     = path_exp+'/'+name_exp;

//--- Create the camera pose and the object pose  
posecDesMo_m  = [0 0 0.5 0 0 0 ];                  // pose target/object desired
posecMo_m     = [0.1 0.1 1 0 0 0 ];								 // pose target/object init  
posewMo_m     = [0 0 0 0 %pi 0 ];                  // pose of the target in the World Frame
cDesMo_m      = homogeneousMatrixFromPos(posecDesMo_m);
cMo_m         = homogeneousMatrixFromPos(posecMo_m);
wMo_m         = homogeneousMatrixFromPos(posewMo_m); 
a_m           = 0.20;                              // dimension of the target  
oP_m          = mire4points(a_m);                  // create the Npbts Points Target
Nbpts_m       = length(oP_m)/3 ;

//--- compute the init projection on the view
cP_m          = changeFrameMire(oP_m,cMo_m);
s_m           = projectMireDirect(cP_m);
Z_m           = cP_m(3:3:$)  ;
cDesP_m       = changeFrameMire(oP_m,cDesMo_m);
sDes_m        = projectMireDirect(cDesP_m);
ZDes_m        = cDesP_m(3:3:$);      

// ------ Sampling time
Te_m          = 1/25;                              // to be consistant with the image frame rate
Te_simu			  = 1/25;

//--- Predictive control parameters
Np_m          = 1;                                 // horizon lenght
Nc_m          = 1;                                 // command horizon length  
v_m           = [0; 0; 0; 0; 0; 0];                // init velocity

thres_m       = 1e-2;                              // error threshold  
lambda        = 1; 

// ------ Constraints definition
xu_m          = [  0.22 ;  0.22 ];                 // position max of the a 2D point in the image plane 
xl_m          = [ -0.22 ; -0.22 ];                 // position min of the a 2D point in the image plane 
bu_m          = 1e3*0.25*ones(6,1);  // command bounds
bl_m          = -bu_m;                             // command bounds on the horizon

// ------ LFunction and Q definition
Lfunction    = matIntMireC;                       // Lc(t) classical visual servo s(t) Z(t)
//Lfunction    = matIntMireP;                     // Lp(t) classical visual servo s(t) Z*
//Lfunction    = matIntMireM;                     // Lm(t) mixte (L*+Lc(t))
//Lfunction    = matIntMireD;                     // Ld    classical interaction matrix desired
Q_m          = matWeightIdentity(Np_m,Nbpts_m);
//Q_m          = matWeightIdentityZero(Np_m,Nbpts_m,1);
//Q_m          = matWeightTV(Np,Nbpts);

//funcost_m    = cld_costLocalMire;
funcost_m    = ga_costLocalMire;
funcst_m     = ga_constraintsLocalMire;
jaccost_m    = "grobfd";
jaccst_m     = "grcnfd";

// -------- Display
hf2d1_m      = createPlanImage(1,xl_m,xu_m,"Point 2D");
mire2DDraw(s_m,0.01,3);        
mire2DDraw(sDes_m,0.01,5);
show_pixmap();
disp("sm");
disp(s_m);
disp("sDesm");
disp(sDes_m);


disp('Fin de la definition des parametres de l expe');
pause;

// ----- Global param

e0_m		 		  = zeros(Nbpts_m*2,1);

global e0_global;				e0_global			    = e0_m;				// error between model and truth
global Nc_global ;			Nc_global			    = Nc_m;				// control horizon
global Np_global ;			Np_global			    = Np_m;				// prediction horizon
global Nbpts_global;		Nbpts_global	    = Nbpts_m;    // number of points of the target   
global Te_global ;			Te_global         = Te_m;       // the sampling time
global Zdes_global;			Zdes_global       = ZDes_m;     // the des depth
global sdes_global;			sdes_global       = sDes_m;     // the des features
global Z_global;				Z_global          = Z_m;        // the des depth
global s_global;				s_global          = s_m;        // the des features
global Q_global;				Q_global          = Q_m;        // weighted matrix
global computeL_global; computeL_global = Lfunction;		// name of the function to compute the interaction matrix 
global STORE;						STORE             = [];

global Sp_global;
global Up_global;
global Sv_global;
global Uv_global;  
global Sa_global; 
global Ua_global;
[Sp_global, Sv_global, Sa_global, Up_global, Uv_global, Ua_global] = buildC(Np_m,Te_m);

xmax    = xu_m(1);                    // position max of the a 2D point in the image plane x axis
xmin    = xl_m(1);                    // position min of the a 2D point in the image plane x axis
ymax    = xu_m(2);                    // position max of the a 2D point in the image plane y axis
ymin    = xl_m(2);                    // position min of the a 2D point in the image plane y axis 

global constraints_global;           // constraint limit vector needed in the contraint function eval
constraints_global = ga_constraintsLimits(Np_global,Nbpts_global,xmax,ymax,xmin,ymin);

// build the constraints.
global bl_global;
   bl_global	= [];
global bu_global;
   bu_global	= [];
for i=1:Nc_global
    bu_global = [bu_global;bu_m];                           // upper bounds
    bl_global = [bl_global;bl_m];                           // upper bounds
end

global ipar_global;
nf						= 1;							// nombre de fonction de cout
nineqn				= 0;         // nombre de contraintes d'inegalite nl
option_ineq		= 0;
tol_in				=	1e-5;

if(option_ineq)
   nineq  = Np_in*Nbpts_in*4; // nombre de contraintes d'inegalites l
else 
   nineq			= 0;
 end
 
neqn					= 0;         // nombre de contraintes d'egalite nl
neq						= 0;         // nombre de contraintes d'egalite l
modefsqp			= 100; 
miter					= 100;			// maximum number of iteration allowed by the user 
iprint				= 2;        // displayed parameter objective and constraint is displayed
ipar_global		=	[nf,nineqn,nineq,neqn,neq,modefsqp,miter,iprint];

global rpar_global;
bigbnd				= 1e4;       // infinity value
eps						= tol_in;      // final norm requirement for d0k
epsneq				= 0.e0;      // maximum violation of linear equalite contraints
udelta				= 0.e0;      // the perturbation sixze the user suggest to compute the gradients
               // 0 if the user has no idea of what to suggest  
rpar_global=[bigbnd,eps,epsneq,udelta];
 

disp('Fin de la definition des parametres globaux de l expe');
pause;

U0_m					= [];   // create the first control horizon
for i = 1:Nc_m
  U0_m				= [U0_m ; v_m];
end ;

 [U_m,smhor_m,Uhor_m] =predControl(U0_m,s_m,,funcost,funcstr,gradobj,gradcstr);

// -------- Display
  hf2d5_m    = createPlanImage(2,xl_m,xu_m,"Prediction Mire");
  smvisu_m   =[s_m;smhor_m];
  mireEvolutionDraw(Np_m+1,smvisu_m,1);
  show_pixmap()
 
endfunction
