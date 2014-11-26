clear
;exec('Load.sce');
getd('main/predControl');

//;exec('main/predControl/testPredictionICRA2013.sce');
//;exec('main/predControl/testAsservisueICRA2013.sce');


//OPT_RECOMPUTE_MATRICES = %F;
//;exec('LoadMarche.sce');
//compPred();
//testEulerAngleRates();
//testOneLoop()

//function start(OPT)
// if(OPT=="PRED")
//    ;exec('main/vs/testPrediction.sce');
//  elseif(OPT=="RECTIF")
//    ;exec('main/vs/testRectification.sce');
//  end
//endfunction

//OPT_RECOMPUTE_MATRICES = %T;
//;exec('LoadMarche.sce');


//function start(OPT,OPT_CORR,OPT_CENTER,OPT_DISTURB,pathExp)
 
//  function start()
 //   getd('main/vsWalk');
 //  disp('You enter the ExpMatrix');
 //    OPT_RECOMPUTE_MATRICES = %F;
 //   ;exec('LoadMarche.sce');
 //   testSwayCorrection3DExpM();
   // testSwayCorrection3D(%T,%F,'.');
    
 //   endfunction
    
    
    
  //
//  isTestSwayLoaded()
//  if(OPT=="PG")
//    disp('You enter the PG test');
//    OPT_RECOMPUTE_MATRICES = %F;
//    ;exec('LoadMarche.sce');
//    testPG(pathExp);
//  elseif(OPT=="ExpMatrix")
//    disp('You enter the ExpMatrix');
//     OPT_RECOMPUTE_MATRICES = %F;
//    ;exec('LoadMarche.sce');
//    testSwayCorrection3DExpM();
//  elseif(OPT=="2TASKS")
//    disp("You are testing the two tasks algorithm")
//    OPT_RECOMPUTE_MATRICES = %F;
//    ;exec('LoadMarche.sce');
//    testSwayCorr3D2Tasks(OPT_CORR,OPT_CENTER,OPT_DISTURB,pathExp);
//  elseif (OPT=="3D")
//    disp('You enterthe 3D test')
//     OPT_RECOMPUTE_MATRICES = %F;
//    ;exec('LoadMarche.sce');
//    testSwayCorrection3D(OPT_CORR,OPT_DISTURB,pathExp)
//  elseif (OPT=="PGCAM")
//    disp('You enter the PG CAMtest')
//    OPT_RECOMPUTE_MATRICES = %F;
//    ;exec('LoadMarche.sce');
//    testPGCam(pathExp);    
//  elseif (OPT=="s")
//   disp('You enter the HRP2test')
//    OPT_RECOMPUTE_MATRICES = %F;
//    ;exec('LoadMarche.sce');
//    testSwayCorrection(OPT_CORR,OPT_DISTURB,pathExp);
//  elseif( OPT =="3ddl")
//   disp('You enter the 3ddl')
//    testSway3ddl(OPT_CORR,%T);
//  elseif( OPT =="6ddl")
//   disp('You enter the 6ddl')
//    testSway(OPT_CORR,%T);
//  elseif( OPT =="Robot")
//   disp('You enter the 6ddl test')
//    testSwayRobot(OPT_CORR,%T,pathExp);   
//  end
//endfunction
