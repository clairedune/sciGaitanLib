
// -----------------------------------------//
// build a matrix with A has diagonal
//------------------------------------------//
function bigA = bigDiag(A,n)

 Zer = zeros(size(A,1),size(A,2));
 bigA=[];
 for i = 1:n;
   L_ligne = [];
      for j=1:n
        if i==j
             L_ligne = [L_ligne A];
        else
             L_ligne = [L_ligne Zer];
       end
     end 
    bigA = [bigA;L_ligne];
  end

endfunction





//--------//
//
//--------//
function stateC = convertState(stateA)

stateC = [1 0 0 0 0 0 0 0 0 
               0 0 0 1 0 0 0 0 0
               0 0 0 0 0 0 1 0 0
               0 1 0 0 0 0 0 0 0 
               0 0 0 0 1 0 0 0 0
               0 0 0 0 0 0 0 1 0
               0 0 1 0 0 0 0 0 0
               0 0 0 0 0 1 0 0 0 
               0 0 0 0 0 0 0 0 1
                ] * stateA;

endfunction


function stateC = convertState6(stateA)

stateC = [1 0 0 0 0 0 0 0 0 
          0 0 0 1 0 0 0 0 0
               0 0 0 0 0 0 1 0 0
               0 1 0 0 0 0 0 0 0 
               0 0 0 0 1 0 0 0 0
               0 0 0 0 0 0 0 1 0
               0 0 1 0 0 0 0 0 0
               0 0 0 0 0 1 0 0 0 
               0 0 0 0 0 0 0 0 1
                ] * stateA;

endfunction


function p= crossProd(u,v)
  [nu,mu]=size(u);
  [nv,mv]=size(v);
  
  if nu*mu<>3 |nv*mv<>3 then
    error('Vectors must be 3D only');
    abort;
  end

  A1 = [u(2), u(3);v(2), v(3)];
  A2 = [u(3), u(1);v(3), v(1)];
  A3 = [u(1), u(2);v(1), v(2)];
  
  px = det(A1);
  py = det(A2);
  pz = det(A3);
  
  p = [px py pz]';
  
  
endfunction   
