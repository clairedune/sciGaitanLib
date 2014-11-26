
  vitesse = [1,2,3];
  vmax = [0.5, 0.5, 0.1];
  dv = 0.05*vmax;
  vinf = 0.95*vmax;
  vsup = 1.05*vmax;
  
   fac  = 1;
  
  for i=1:3  
  
   absVel =abs(vitesse(i));
   fac = min(abs(fac),vmax(i)/(absVel+%eps));
   disp(fac);
  end 

//  for i=1:3  
    vitesse=fac*vitesse;
 // end 

  disp(vitesse);
  
