function P = computePest(x)
    //x=(d,q)    
    d = x(1:15);
    d(1) = 0.2; // longueur du pied gauche
    d(9) = 0.2; // longueur du pied droit
    d(14)= 0.5; // largeur deamb
    d(15)= 1;//hauteur deamb
    q    = x(16:$-2);
    //q(1) = 0 ;           // angle plante du pied/sol G
    //aq(8) = 0;            // pied D
    P   = H2LWScomputeMGD(d,q); 
   
    // on translate le point d'origine
    tx   = x($-1);
    tz   = x($);
    P    = P + [tx*ones(1,size(P,2));
            tz*ones(1, size(P,2));
            zeros(1,size(P,2));
            zeros(1,size(P,2))];  
endfunction

function e=computeError(P,Ptest)
    e=0;    
    e=e+(Ptest(3,2:12)-P(1,2:12))*(Ptest(3,2:12)-P(1,2:12))';
    e=e+(Ptest(2,2:12)-P(2,2:12))*(Ptest(2,2:12)-P(2,2:12))';
endfunction


function e = costfunction (x)
    Pest  = computePest(x);
    e     = computeError(Pest,Ptest);
endfunction