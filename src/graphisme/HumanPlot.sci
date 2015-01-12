// ce fichier contient toutes les fonction relative
// a l'affichage graphique d'un humanoide

// on reorganise les points pour les afficher avec un plot
function plotHuman14dof(P)

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

piedG   = [Pp3_0,Pp1_0,Pp2_0, Pp3_0]; //pied gauche
jambeG  = [P2_0, P3_0, P4_0];         //jambe gauche
jambeD  = [P6_0, P5_0, P4_0];         // jambe droite
piedD   = [Pp1_8,Pp3_8,Pp2_8,Pp3_8];  //pied droit
tronc   = [P4_0, P8_0];               //tronc
brasD   = [P8_0, P9_0, P10_0];        //bras gauche
brasG   = [P8_0, P11_0,P12_0];        //bras droit
   
plot(piedG(2,:),piedG(1,:),'r*--');
plot(jambeG(2,:),jambeG(1,:),'r*--');

plot(piedD(2,:),piedD(1,:),'g*--');
plot(jambeD(2,:),jambeD(1,:),'g*--');

plot(tronc(2,:),tronc(1,:),'k*--');

plot(brasG(2,:),brasG(1,:),'r*--');
plot(brasD(2,:),brasD(1,:),'g*--');
  
endfunction

function plotHuman10dots(P)

P2_0 = P(:,1);
P3_0 = P(:,2);
P4_0 = P(:,3);
P5_0 = P(:,4);
P6_0 = P(:,5);
P8_0 = P(:,6);
P9_0 = P(:,7);
P10_0 = P(:,8);
P11_0 = P(:,9);
P12_0 = P(:,10);

jambeG  = [P2_0, P3_0, P4_0];         //jambe gauche
jambeD  = [P6_0, P5_0, P4_0];         // jambe droite
tronc   = [P4_0, P8_0];               //tronc
brasD   = [P8_0, P9_0, P10_0];        //bras gauche
brasG   = [P8_0, P11_0,P12_0];        //bras droit
   
plot(jambeG(2,:),jambeG(1,:),'r*--');
plot(jambeD(2,:),jambeD(1,:),'g*--');

plot(tronc(2,:),tronc(1,:),'k*--');

plot(brasG(2,:),brasG(1,:),'r*--');
plot(brasD(2,:),brasD(1,:),'g*--');
  
endfunction



function humanMocapPlot(P)

// P1 pied, P2 cheville, P3 genou, 
// P4 hanches
// P5 genou, P6 cheville, P7 pied
// P8 epaules
// P9 coude, P10 poignet
// P11 coude, P12 poignet

//Pdisp = [P(1,:);P(2,:);P(3,:);P(4,:);         //jambe 1
//         P(5,:);P(6,:);P(7,:);P(6,:);P(5,:);P(4,:);         //jambe 2
//         P(8,:);P(9,:);P(10,:);P(9,:);P(8,:); //bras droit
//         P(11,:);P(12,:)];                    //bras gauche

//Pdisp = [P(1,:);P(2,:);P(3,:);         //jambe 1
//         P(3,:);P(4,:);P(5,:);         //jambe 2
//         P(6,:);P(7,:);P(8,:);P(7,:); //bras droit
//         P(6,:);P(9,:);P(10,:)];                    //bras gauche

P=P';
jambeG  = [P(1,:);P(2,:);P(3,:)];         //jambe gauche
jambeD  = [P(3,:);P(4,:);P(5,:)];         // jambe droite
tronc   = [P(3,:);P(6,:)];               //tronc
brasD   = [P(6,:);P(7,:);P(8,:)];        //bras gauche
brasG   = [P(6,:);P(9,:);P(10,:)];        //bras droit
   
//disp(jambeG)   
   
plot(jambeG(:,2),jambeG(:,3),'ro-');
plot(jambeD(:,2),jambeD(:,3),'go-');

plot(tronc(:,2),tronc(:,3),'ko-');

plot(brasG(:,2),brasG(:,3),'ro-');
plot(brasD(:,2),brasD(:,3),'go-');

endfunction
  

