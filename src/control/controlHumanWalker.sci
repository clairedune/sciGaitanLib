//----------------------------------------------//
//  ApplyConstraintWalker
//----------------------------------------------//
  function [qnew] = applyConstraintsWalker(q,d,dt,tol, yDes_06)
  // starting from a set of articular position q
  // find a new position qnew that garanty that the walker touch 
  // the ground.

  d1=d(1); d2=d(2); d3=d(3); d4=d(4); d5=d(5); d6=d(6); dh=d(7);dw=d(8);
  q1=q(1); q2=q(2); q3=q(3); q4=q(4); q5=q(5); q6=q(6);

  
  
  // position courante de l'effecteur
  [M_01, M_12, M_23, M_34, M_45, M_56, A_0, B_0, C_0, P_0] = computeMGDsagitalMan(d,q);
  M_06=M_01*M_12*M_23*M_34*M_45*M_56;
  posEffecteur = pFromHomogeneousMatrix(M_06);
    pos_06 = [posEffecteur(1);posEffecteur(6)];
  
//  posDes_06 = [dh; yDes_06; %pi/2];
  posDes_06 = [dh;  %pi/2];
  
  
  // descente du gradient pour trouver q tel que P=Pdes
  errPos_06 = pos_06-posDes_06;
  qprec = q';
  lambda = 0.6;
  errVect = [errPos_06];

  i=0;

  while (norm(errPos_06)>tol)
    //disp(i)
    //  disp(errPos_06)
    //disp(norm(errPos_06));
     
    
    J1 = computeJ06SagMan(d,qprec);
    J1=[J1(1,:);J1(3,:)];
    dotq = -lambda*pinv(J1)*errPos_06;
    qnew= qprec+dt*dotq;
    
    [M_01, M_12, M_23, M_34, M_45, M_56, A_0, B_0, C_0, P_0] = computeMGDsagitalMan(d,qnew);
    M_06=M_01*M_12*M_23*M_34*M_45*M_56;
    posEffecteur = pFromHomogeneousMatrix(M_06);
    pos_06 = [posEffecteur(1);posEffecteur(6)];
    //update
    qprec=qnew;
    errPos_06 = pos_06-posDes_06;
    errVect= [errVect errPos_06];
    i=i+1;
 end
  
endfunction
//----------------------------------------------//
//  ApplyConstraintWalker
//----------------------------------------------//
  function [qnew] = applyConsAndLimits(q,d,qlimit,dt,tol)
  //// starting from a set of articular position q
//  // find a new position qnew that garanty that the walker touch 
//  // the ground.

  d1=d(1); d2=d(2); d3=d(3); d4=d(4); d5=d(5); d6=d(6); dh=d(7);dw=d(8);
  q1=q(1); q2=q(2); q3=q(3); q4=q(4); q5=q(5); q6=q(6);
  
  // position courante de l'effecteur
  [M_01, M_12, M_23, M_34, M_45, M_56, A_0, B_0, C_0, P_0] = computeMGDsagitalMan(d,q);
  M_06=M_01*M_12*M_23*M_34*M_45*M_56;
  posEffecteur = pFromHomogeneousMatrix(M_06);
  //pos_06 = [posEffecteur(1:2);posEffecteur(6)];
//  posDes_06 = pos_06;
  pos_06 = [posEffecteur(1);posEffecteur(6)];
  
//  posDes_06 = [dh; yDes_06; %pi/2];
  posDes_06 = [dh;  %pi/2];
  
  // descente du gradient pour trouver q tel que P=Pdes
  er1 = pos_06-posDes_06;
  qprec = q';
  lambda =1;
  lambda2 = 1;

  errVect = [er1];
  i=0;

  marge = 10*%pi/180;

  while (norm(er1)>tol)  
    
    
    // ------ Tache 1 hauteur de la main ------ //
    J1 = computeJ06SagMan(d,qprec);
    J1=[J1(1,:);J1(3,:)];
    dotq1 = -lambda*pinv(J1)*er1;
    //disp(dotq1)     
   
    //projecteur
    Proj1 = (eye(6,6)-pinv(J1)*J1);
    
    // ----- Tache 1 eloignement des butees ----//    
    // matrice d'activation de la tache primaire
    A=zeros(6,6);
    B=zeros(6,6);
    for k=1:6
      if (qprec(k)<(qlimit(1,k)+marge) )// FIXME Attention angle a remettre entre -pi et pi
          A(k,k)=1;
      elseif (qprec(k)>(qlimit(2,k)-marge))
          B(k,k)=1;
      end
    end
    // Jacobien
    J2 = A+B;
    // Erreur
    er2 = A*((qprec-qlimit(1,:)')/marge)+B*((qprec-qlimit(2,:)')/marge);
    // Loi de commande robuste
    //dotq2 = -lambda2*Proj1*pinv(J2)*er2
    //loi de commande exacte
     dotq2 = -lambda2*pinv(J2*Proj1)*(er2-J2*pinv(J1)*er1);
     
     //disp(dotq2)
     
    // -------- Synthese tache 1 et 2 ----------//
    qnew= qprec+dt*(dotq2+dotq1);
    [M_01, M_12, M_23, M_34, M_45, M_56, A_0, B_0, C_0, P_0] = computeMGDsagitalMan(d,qnew);
    M_06=M_01*M_12*M_23*M_34*M_45*M_56;
    posEffecteur = pFromHomogeneousMatrix(M_06);
//    pos_06 = [posEffecteur(1:2);posEffecteur(6)];
    pos_06 = [posEffecteur(1);posEffecteur(6)];
    //-----------update-----------//
    qprec=qnew;
    er1 = pos_06-posDes_06;
    errVect= [errVect er1];
    i=i+1;

   //// le robot est un robot plan sur xy
//sommets=[zeros(4,1) P_0 A_0 C_0 A_0 P_0(:,$) B_0];
//     y = sommets (1,:);
//     x = sommets (2,:);
//     plot(x,y,'y-.');
//    show_pixmap()
   // pause
end

  
endfunction


//----------------------------------------------//
//  ApplyConstraintWalker
//----------------------------------------------//

 
 
 
 function [qnew] = applyLimitsandConst(q,d,qlimit,dt,tol)
  //// starting from a set of articular position q
//  // find a new position qnew that garanty that the walker touch 
//  // the ground.

  qnew=q;

  d1=d(1); d2=d(2); d3=d(3); d4=d(4); d5=d(5); d6=d(6); dh=d(7);dw=d(8);
  q1=q(1); q2=q(2); q3=q(3); q4=q(4); q5=q(5); q6=q(6);

  
  
  // position courante de l'effecteur
  [M_01, M_12, M_23, M_34, M_45, M_56, A_0, B_0, C_0, P_0] = computeMGDsagitalMan(d,q);
  M_06=M_01*M_12*M_23*M_34*M_45*M_56;
  posEffecteur = pFromHomogeneousMatrix(M_06);
  //pos_06 = [posEffecteur(1:2);posEffecteur(6)];
//  posDes_06 = pos_06;
  pos_06 = [posEffecteur(1);posEffecteur(6)];
  
//  posDes_06 = [dh; yDes_06; %pi/2];
  posDes_06 = [dh;  %pi/2];
  
  // descente du gradient pour trouver q tel que P=Pdes
  er2 = pos_06-posDes_06;
  qprec = q';
  lambda =0.5;
  lambda2 = 0.6;
  er1=er2;
  errVect = [er2];
  i=0;
   qdot1Vect=[];
   qdot1et2Vect=[];
  marge = 10*%pi/180;

  while (norm(er2)>tol )  
    
    //disp('-----')
    //disp(i)
    
    
    // ----- Tache 1 eloignement des butees ----//    
    // matrice d'activation de la tache primaire
    A=zeros(6,6);
    B=zeros(6,6);
    for k=1:6
      if (qprec(k)<(qlimit(1,k)+marge) )// FIXME Attention angle a remettre entre -pi et pi
          A(k,k)=1;
          
      elseif (qprec(k)>(qlimit(2,k)-marge))
          B(k,k)=1;
      end
    end
    
    // Jacobien
    J1 = A+B;
    // Erreur
    er1 =  A*((qprec-qlimit(1,:)')/marge)+B*((qprec-qlimit(2,:)')/marge);
    dotq1 = -lambda*pinv(J1)*er1;
     
    //projecteur
    Proj1 = (eye(6,6)-pinv(J1)*J1);
    // ------ Tache 2 hauteur de la main ------ //
    J2 = computeJ06SagMan(d,qprec);
    J2=[J2(1,:);J2(3,:)];
    
    //dotq2 = -lambda2*Proj1*pinv(J2)* er2;
    edotref1=-lambda*er1;
    edotref2=-lambda2*er2;

    dotq2 = pinv(J2*Proj1)*(edotref2-J2*pinv(J1)*edotref1);
    //disp(dotq2)  
    
    
    // -------- Synthese tache 1 et 2 ----------//
    qnew= qprec+dt*(dotq2+dotq1);
  
    
    [M_01, M_12, M_23, M_34, M_45, M_56, A_0, B_0, C_0, P_0] = computeMGDsagitalMan(d,qnew);
    M_06=M_01*M_12*M_23*M_34*M_45*M_56;
    posEffecteur = pFromHomogeneousMatrix(M_06);
//    pos_06 = [posEffecteur(1:2);posEffecteur(6)];
    pos_06 = [posEffecteur(1);posEffecteur(6)];
    //-----------update-----------//
    qprec=qnew;
    er2 = pos_06-posDes_06;
    errVect= [errVect er2];
    i=i+1;
    qdot1Vect=[qdot1Vect dotq1];
    qdot1et2Vect=[qdot1et2Vect (dotq1+dotq2)];
    

   // le robot est un robot plan sur xy
    //sommets=[zeros(4,1) P_0 A_0 C_0 A_0 P_0(:,$) B_0];
//     y = sommets (1,:);
//     x = sommets (2,:);
//     plot(x,y,'y-.');
//    show_pixmap()
end

////trac de la figure
//xset("window",4);
//xset("pixmap",1);
//clear_pixmap()//et buffer
//h1=scf(4);
//h1.figure_name = "Error q1";
//plot(qdot1Vect')
//    show_pixmap()
//
////trac de la figure
//xset("window",5);
//xset("pixmap",1);
//clear_pixmap()//et buffer
//h1=scf(5);
//h1.figure_name = "Error q1";
//plot(qdot1et2Vect')
//      show_pixmap()
//pause
endfunction


//----------------------------------------------//
//  ApplyActiveSet
//----------------------------------------------//
 
 function [qnew] = applyActivSet(q,d,qlimit,dt,tol)
  //// starting from a set of articular position q
//  // find a new position qnew that garanty that the walker touch 
//  // the ground.

  qnew=q;

  d1=d(1); d2=d(2); d3=d(3); d4=d(4); d5=d(5); d6=d(6); dh=d(7);dw=d(8);
  q1=q(1); q2=q(2); q3=q(3); q4=q(4); q5=q(5); q6=q(6);

  
  
  // position courante de l'effecteur
  [M_01, M_12, M_23, M_34, M_45, M_56, A_0, B_0, C_0, P_0] = computeMGDsagitalMan(d,q);
  M_06=M_01*M_12*M_23*M_34*M_45*M_56;
  posEffecteur = pFromHomogeneousMatrix(M_06);
  //pos_06 = [posEffecteur(1:2);posEffecteur(6)];
//  posDes_06 = pos_06;
  pos_06 = [posEffecteur(1);posEffecteur(6)];
  
//  posDes_06 = [dh; yDes_06; %pi/2];
  posDes_06 = [dh;  %pi/2];
  
  // descente du gradient pour trouver q tel que P=Pdes
  er2 = pos_06-posDes_06;
  qprec = q';
  lambda =0.5;
  lambda2 = 0.6;
  er1=er2;
  errVect = [er2];
  i=0;
   qdot1Vect=[];
   qdot1et2Vect=[];
  marge = 10*%pi/180;
  DT = 10*dt;
  while (norm(er2)>tol )  
    
    
     edotref1=-lambda*er1;
     edotref2=-lambda2*er2;
     
     J2 = computeJ06SagMan(d,qprec);
     J2=[J2(1,:);J2(3,:)];
     dotq = pinv(J2)* edotref2;
     
    dotqmin = (qlimit(1,:)'-qprec)/DT;
    dotqmax = (qlimit(2,:)'-qprec)/DT;
    
    [value,index]=min([(dotq-dotqmin) ;(dotqmax-dotq)])
    
    A=zeros(6,6);
    B=zeros(6,6);
    
    //tant que la contrainte la plus violee est sous la marge
    nbdof = 0;
    while (value <0 & nbdof<=length(q)) 
      // remplir la matrice d'activation
      if(index<=6) 
        A(index,index)=1;
      else 
        B(index-6,index-6)=1;
      end    
      // Jacobien
      J1       = A+B;
      // Erreur
      dotq1      =  A*(dotqmin)+B*(dotqmax); 
      //projecteur
      Proj1    = (eye(6,6)-pinv(J1)*J1);
      // ------ Tache 2 hauteur de la main ------ //
      dotq2    = pinv(J2*Proj1)*(edotref2-J2*dotq1);
      // -------- Synthese tache 1 et 2 ----------//
      [value,index]=min([(dotq1+dotq2-dotqmin) ;(dotqmax-dotq2-dotq1)]);
      nbdof=nbdof+1;
     // pause
    end
    qnew       = qprec+dt*(dotq2+dotq1);
    [M_01, M_12, M_23, M_34, M_45, M_56, A_0, B_0, C_0, P_0] = computeMGDsagitalMan(d,qnew);
     M_06      = M_01*M_12*M_23*M_34*M_45*M_56;
    posEffecteur = pFromHomogeneousMatrix(M_06);
    pos_06 = [posEffecteur(1);posEffecteur(6)];
    //-----------update-----------//
    qprec=qnew;
    er2 = pos_06-posDes_06;
    errVect= [errVect er2];
    i=i+1;
    qdot1Vect=[qdot1Vect dotq1];
    qdot1et2Vect=[qdot1et2Vect (dotq1+dotq2)];
    

   // le robot est un robot plan sur xy
    //sommets=[zeros(4,1) P_0 A_0 C_0 A_0 P_0(:,$) B_0];
//     y = sommets (1,:);
//     x = sommets (2,:);
//     plot(x,y,'y-.');
//    show_pixmap()
end

////trac de la figure
//xset("window",4);
//xset("pixmap",1);
//clear_pixmap()//et buffer
//h1=scf(4);
//h1.figure_name = "Error q1";
//plot(qdot1Vect')
//    show_pixmap()
//
////trac de la figure
//xset("window",5);
//xset("pixmap",1);
//clear_pixmap()//et buffer
//h1=scf(5);
//h1.figure_name = "Error q1";
//plot(qdot1et2Vect')
//      show_pixmap()
//pause
endfunction
