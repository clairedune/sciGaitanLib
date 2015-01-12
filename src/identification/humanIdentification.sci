function P = computePest(x)
    //x=(d,q) 
    //global d0;
       
    d = x(1:9);
    q = x(10:$-2);

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
    e=e+(Ptest(3,:)-P(1,:))*(Ptest(3,:)-P(1,:))';
    e=e+(Ptest(2,:)-P(2,:))*(Ptest(2,:)-P(2,:))';
endfunction


function e = costfunction (x)
    Pest  = computePest(x);
    e     = computeError(Pest,Ptest);
    disp("erreur");
    disp(e);
endfunction