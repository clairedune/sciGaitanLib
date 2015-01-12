function [P1x,P1y,P1z,P2x,P2y,P2z,P3x,P3y,P3z,P4x,P4y,P4z,P5x,P5y,P5z,P6x,P6y,P6z,P7x,P7y,P7z,P8x,P8y,P8z,P9x,P9y,P9z,P10x,P10y,P10z,P11x,P11y,P11z,P12x,P12y,P12z] = mocapData(essai)
    
Mqualysis = essai.Trajectories.Labeled.Data ;
startFrame = essai.StartFrame;
frequency = essai.FrameRate;

// Index des points dans les resultats du qualysis
TG      =   12 ; //talon gauche
TD      =   6  ; //talon droit
MIG     =   9  ; //centre du pied
MMG     =   10 ; //maleole externe gauche
MLG     =   11 ; //maleole interne gauche
GTG     =   25 ; //grand trocard gauche
GTD     =   26 ; //grand troc
MIDPSIS =   31 ; //milieu du dos
ACD     =   36 ; //acro Droit
ACG     =   37 ; //acro Gauche 
LCG     =   19 ; //genou gauche externe
MCG     =   20 ; //genou gauche interne
LCD     =   15 ; //genou droit externe
MCD     =   16 ; //genou droit interne
MMD     =    5 ; //maleole externe droite
MLD     =    6 ; //maleole interne droite
ELD     =   38 ; //coude lateral droit
EMD     =   39 ; //coude medial droit
RSD     =   42 ; // poignet droit
USD     =   43 ; //poignet droit
ELG     =   40 ; //coude gauche
EMG     =   41 ; //coude droit
RSG     =   44 ; //poignet gauche
USG     =   45 ; //poignet droit  



// PIED GAUCHE
P1x = Mqualysis(TG,1,:);
P1x = P1x(1,:)
P1y = Mqualysis(TG,2,:);
P1y = P1y(1,:)
P1z = Mqualysis(TG,3,:);
P1z = P1z(1,:);


//xset("window",3);
//plot(P1ay,P1az) ;

// CHEVILLE GAUCHE ( MM + ML )/2
PMMx = Mqualysis(MMG,1,:);
PMMx = PMMx(1,:);
PMMy = Mqualysis(MMG,2,:);
PMMy = PMMy(1,:);
PMMz = Mqualysis(MMG,3,:);
PMMz = PMMz(1,:);

PMLx = Mqualysis(MLG,1,:);
PMLx = PMLx(1,:);
PMLy = Mqualysis(MLG,2,:);
PMLy = PMLy(1,:);
PMLz = Mqualysis(MLG,3,:);
PMLz = PMLz(1,:);

P2x  = (PMMx+PMLx)/2;
P2y  = (PMMy+PMLy)/2;
P2z  = (PMMz+PMLz)/2;

//xset("window",3);
//plot(P2y,P2z) ;

// GENOU GAUCHE LCG
P3x = Mqualysis(LCG,1,:);
P3x = P3x(1,:);
P3y = Mqualysis(LCG,2,:);
P3y = P3y(1,:);
P3z = Mqualysis(LCG,3,:);
P3z = P3z(1,:);

// Hanche
PGTGx = Mqualysis(GTG,1,:);
PGTGx = PGTGx(1,:);
PGTGy = Mqualysis(GTG,2,:);
PGTGy = PGTGy(1,:);
PGTGz = Mqualysis(GTG,3,:);
PGTGz = PGTGz(1,:);

PGTDx = Mqualysis(GTD,1,:);
PGTDx = PGTDx(1,:);
PGTDy = Mqualysis(GTD,2,:);
PGTDy = PGTDy(1,:);
PGTDz = Mqualysis(GTD,3,:);
PGTDz = PGTDz(1,:);

P4x  = (PGTGx+PGTDx)/2;
P4y  = (PGTGy+PGTDy)/2;
P4z  = (PGTGz+PGTDz)/2;


// genou droit
P5x = Mqualysis(LCD,1,:);
P5x = P5x(1,:);
P5y = Mqualysis(LCD,2,:);
P5y = P5y(1,:);
P5z = Mqualysis(LCD,3,:);
P5z = P5z(1,:);


// CHEVILLE DROITE ( MM + ML )/2
PMMx = Mqualysis(MMD,1,:);
PMMx = PMMx(1,:);
PMMy = Mqualysis(MMD,2,:);
PMMy = PMMy(1,:);
PMMz = Mqualysis(MMD,3,:);
PMMz = PMMz(1,:);

PMLx = Mqualysis(MLD,1,:);
PMLx = PMLx(1,:);
PMLy = Mqualysis(MLD,2,:);
PMLy = PMLy(1,:);
PMLz = Mqualysis(MLD,3,:);
PMLz = PMLz(1,:);

P6x  = (PMMx+PMLx)/2;
P6y  = (PMMy+PMLy)/2;
P6z  = (PMMz+PMLz)/2;

// PIED DROIT
P7x = Mqualysis(TD,1,:);
P7x = P7x(1,:)
P7y = Mqualysis(TD,2,:);
P7y = P7y(1,:)
P7z = Mqualysis(TD,3,:);
P7z = P7z(1,:);

// EPAULES

PACDx = Mqualysis(ACD,1,:);
PACDx = PACDx(1,:);
PACDy = Mqualysis(ACD,2,:);
PACDy = PACDy(1,:);
PACDz = Mqualysis(ACD,3,:);
PACDz = PACDz(1,:);

PACGx = Mqualysis(ACG,1,:);
PACGx = PACGx(1,:);
PACGy = Mqualysis(ACG,2,:);
PACGy = PACGy(1,:);
PACGz = Mqualysis(ACG,3,:);
PACGz = PACGz(1,:);

P8x  = (PACDx+PACGx)/2;
P8y  = (PACDy+PACGy)/2;
P8z  = (PACDz+PACGz)/2;

// COUDE GAUCHE ELG
P9x = Mqualysis(ELG,1,:);
P9x = P9x(1,:)
P9y = Mqualysis(ELG,2,:);
P9y = P9y(1,:)
P9z = Mqualysis(ELG,3,:);
P9z = P9z(1,:);

// POIGNET DROIT RSD
P10x = Mqualysis(RSG,1,:);
P10x = P10x(1,:)
P10y = Mqualysis(RSG,2,:);
P10y = P10y(1,:)
P10z = Mqualysis(RSG,3,:);
P10z = P10z(1,:);


// COUDE DROIT
P11x = Mqualysis(ELD,1,:);
P11x = P11x(1,:)
P11y = Mqualysis(ELD,2,:);
P11y = P11y(1,:)
P11z = Mqualysis(ELD,3,:);
P11z = P11z(1,:);

// POIGNET DROIT RSD
P12x = Mqualysis(RSD,1,:);
P12x = P12x(1,:)
P12y = Mqualysis(RSD,2,:);
P12y = P12y(1,:)
P12z = Mqualysis(RSD,3,:);
P12z = P12z(1,:);
endfunction

//function Ptest = testData(P)
//    Ptest= P;
//split data for test and estimation
//split = samples/2; 
//
//P1ax = P1x(1,1:split); //set a for estimation 
//P1bx = P1x(1,split+1:$);  // set b for test
//P1ay = P1y(1,1:split); //set a for estimation 
//P1by = P1y(1,split+1:$);  // set b for test
//P1az = P1z(1,1:split); //set a for estimation 
//P1bz = P1z(1,split+1:$);  // set b for test
//
//P2ax = P2x(1,1:split); //set a for estimation 
//P2bx = P2x(1,split+1:$);  // set b for test
//P2ay = P2y(1,1:split); //set a for estimation 
//P2by = P2y(1,split+1:$);  // set b for test
//P2az = P2z(1,1:split); //set a for estimation 
//P2bz = P2z(1,split+1:$);  // set b for test
//
//P3ax = P3x(1,1:split); //set a for estimation 
//P3bx = P3x(1,split+1:$);  // set b for test
//P3ay = P3y(1,1:split); //set a for estimation 
//P3by = P3y(1,split+1:$);  // set b for test
//P3az = P3z(1,1:split); //set a for estimation 
//P3bz = P3z(1,split+1:$);  // set b for test
//
//P4ax = P4x(1,1:split); //set a for estimation 
//P4bx = P4x(1,split+1:$);  // set b for test
//P4ay = P4y(1,1:split); //set a for estimation 
//P4by = P4y(1,split+1:$);  // set b for test
//P4az = P4z(1,1:split); //set a for estimation 
//P4bz = P4z(1,split+1:$);  // set b for test
//
//P5ax = P5x(1,1:split); //set a for estimation 
//P5bx = P5x(1,split+1:$);  // set b for test
//P5ay = P5y(1,1:split); //set a for estimation 
//P5by = P5y(1,split+1:$);  // set b for test
//P5az = P5z(1,1:split); //set a for estimation 
//P5bz = P5z(1,split+1:$);  // set b for test
//
//P6ax = P6x(1,1:split); //set a for estimation 
//P6bx = P6x(1,split+1:$);  // set b for test
//P6ay = P6y(1,1:split); //set a for estimation 
//P6by = P6y(1,split+1:$);  // set b for test
//P6az = P6z(1,1:split); //set a for estimation 
//P6bz = P6z(1,split+1:$);  // set b for test
//
//
//P7ax = P7x(1,1:split); //set a for estimation 
//P7bx = P7x(1,split+1:$);  // set b for test
//P7ay = P7y(1,1:split); //set a for estimation 
//P7by = P7y(1,split+1:$);  // set b for test
//P7az = P7z(1,1:split); //set a for estimation 
//P7bz = P7z(1,split+1:$);  // set b for test
//
//
//P8ax = P8x(1,1:split); //set a for estimation 
//P8bx = P8x(1,split+1:$);  // set b for test
//P8ay = P8y(1,1:split); //set a for estimation 
//P8by = P8y(1,split+1:$);  // set b for test
//P8az = P8z(1,1:split); //set a for estimation 
//P8bz = P8z(1,split+1:$);  // set b for test
//
//P9ax = P9x(1,1:split); //set a for estimation 
//P9bx = P9x(1,split+1:$);  // set b for test
//P9ay = P9y(1,1:split); //set a for estimation 
//P9by = P9y(1,split+1:$);  // set b for test
//P9az = P9z(1,1:split); //set a for estimation 
//P9bz = P9z(1,split+1:$);  // set b for test
//
//P10ax = P10x(1,1:split); //set a for estimation 
//P10bx = P10x(1,split+1:$);  // set b for test
//P10ay = P10y(1,1:split); //set a for estimation 
//P10by = P10y(1,split+1:$);  // set b for test
//P10az = P10z(1,1:split); //set a for estimation 
//P10bz = P10z(1,split+1:$);  // set b for test
//
//P11ax = P11x(1,1:split); //set a for estimation 
//P11bx = P11x(1,split+1:$);  // set b for test
//P11ay = P11y(1,1:split); //set a for estimation 
//P11by = P11y(1,split+1:$);  // set b for test
//P11az = P11z(1,1:split); //set a for estimation 
//P11bz = P11z(1,split+1:$);  // set b for test
//
//P12ax = P12x(1,1:split); //set a for estimation 
//P12bx = P12x(1,split+1:$);  // set b for test
//P12ay = P12y(1,1:split); //set a for estimation 
//P12by = P12y(1,split+1:$);  // set b for test
//P12az = P12z(1,1:split); //set a for estimation 
//P12bz = P12z(1,split+1:$);  // set b for test

//endfunction
    
    

