  
    
    // Drivative for l_c1 
    for ( int i = 0  ; i<(J_waist_l_c1_spa_0.cols()) ; i++ )  // i<(J_waist_l_c1_spa_0.cols()-1)
     {
       Q_l_c1_col_i =  (D_Jacob_spa_i(J_waist_l_c1_spa_0, (i+1) )).transposed()*    //(i+1)
              ( Adjoint( iHomogeneous(T_waist_l_c1_0))).transposed()*
                                        B *fc_l_c1  +
                                      J_waist_l_c1_spa_0.transposed()* 
              ( Adjoint( iHomogeneous(T_waist_l_c1_0))*
                         ad_lie( -1.0*J_waist_l_c1_spa_0.getCol(i))).transposed()*
                                        B *fc_l_c1;
       Q_l_c1.setCol(i, Q_l_c1_col_i ) ;       
     } ;   
   
    yarp::sig::Matrix Q_l_c1_1 =  Q_ci(J_waist_l_c1_spa_0, T_waist_l_c1_0, fc_l_c1 ) ;
 //   yarp::sig::Matrix Temp = Q_l_c1_1 - Q_l_c1;
 //   std::cout << " Temp = " <<  std::endl << Temp.toString() << std::endl;

    
    
 /* std::fstream fs;
    fs.open ("fc_l_c_1.m", std::fstream::out);
    fs<<"fc_l_c1"<<"=[";
      for (int j=0;j<fc_l_c1.length();j++)
      {
    fs<< fc_l_c1[j]<<" ";
      }
    fs<<"]';"<<std::endl;
    fs.close();
    fs.open ("T_waist_l_c1__0.m", std::fstream::out);
    fs<<"T_waist_l_c1_0"<<"=[";
    for (int i=0;i<T_waist_l_c1_0.rows();i++)  
    {
      for (int j=0;j<T_waist_l_c1_0.cols();j++)
      {
	fs<<T_waist_l_c1_0.getRow(i)[j]<<" ";
      }
      fs<<"; ";
    }
    fs<<"];"<<std::endl;
    fs.close();
    fs.open ("Q_l_c_1.m", std::fstream::out);
    fs<<"Q_l_c1"<<"=[";
    for (int i=0;i<Q_l_c1.rows();i++)  
    {
      for (int j=0;j<Q_l_c1.cols();j++)
      {
	fs<<Q_l_c1.getRow(i)[j]<<" ";
      }
      fs<<"; ";
    }
    fs<<"];"<<std::endl;
    fs.close();
    fs.open ("J_waist_l_c1_spa__0.m", std::fstream::out);
    fs<<"J_waist_l_c1_spa_0"<<"=[";
    for (int i=0;i<J_waist_l_c1_spa_0.rows();i++)  
    {
      for (int j=0;j<J_waist_l_c1_spa_0.cols();j++)
      {
	fs<<J_waist_l_c1_spa_0.getRow(i)[j]<<" ";
      }
      fs<<"; ";
    }
    fs<<"];"<<std::endl;
    fs.close();     */
     
 /*    std::cout << " Q_l_c1 = " <<  std::endl << Q_l_c1.toString() << std::endl; 
     std::cout << " fc_l_c1 = " <<  std::endl << fc_l_c1.toString() << std::endl;  */    

//-----------------------------------------------------------------------------------------------------     
    // Drivative for l_c2 
 /*    for ( int i = 0  ; i<(J_waist_l_c2_spa_0.cols()) ; i++ )  // i<(J_waist_l_c1_spa_0.cols()-1)
     {
       Q_l_c2_col_i =  (D_Jacob_spa_i(J_waist_l_c2_spa_0, (i+1))).transposed()*
              ( Adjoint( iHomogeneous(T_waist_l_c2_0))).transposed()*
                                        B *fc_l_c2  +
                                      J_waist_l_c2_spa_0.transposed()* 
              ( Adjoint( iHomogeneous(T_waist_l_c2_0))*
                         ad_lie( -1.0*J_waist_l_c2_spa_0.getCol(i))).transposed()*
                                        B *fc_l_c2;
       Q_l_c2.setCol(i, Q_l_c2_col_i ) ;       
     } ;   
    std::cout << " Q_l_c2 = " <<  std::endl << Q_l_c2.toString() << std::endl; 
     std::cout << " fc_l_c2 = " <<  std::endl << fc_l_c2.toString() << std::endl;    */  

    yarp::sig::Matrix Q_l_c2_1 =  Q_ci(J_waist_l_c2_spa_0, T_waist_l_c2_0, fc_l_c2 ) ;
    yarp::sig::Matrix Temp_2 = Q_l_c2_1 - Q_l_c2;
    std::cout << " Temp_2 = " <<  std::endl << Temp_2.toString() << std::endl;
 

//-----------------------------------------------------------------------------------------------------     
    // Drivative for l_c3 
    for ( int i = 0  ; i<(J_waist_l_c3_spa_0.cols()) ; i++ )  // i<(J_waist_l_c1_spa_0.cols()-1)
     {
       Q_l_c3_col_i =  (D_Jacob_spa_i(J_waist_l_c3_spa_0, (i+1))).transposed()*
              ( Adjoint( iHomogeneous(T_waist_l_c3_0))).transposed()*
                                        B *fc_l_c3  +
                                      J_waist_l_c3_spa_0.transposed()* 
              ( Adjoint( iHomogeneous(T_waist_l_c3_0))*
                         ad_lie( -1.0*J_waist_l_c3_spa_0.getCol(i))).transposed()*
                                        B *fc_l_c3;
       Q_l_c3.setCol(i, Q_l_c3_col_i ) ;       
     } ;   
  /*   std::cout << " Q_l_c3 = " <<  std::endl << Q_l_c3.toString() << std::endl; 
     std::cout << " fc_l_c3 = " <<  std::endl << fc_l_c3.toString() << std::endl;      */
     yarp::sig::Matrix Q_l_c3_1 =  Q_ci(J_waist_l_c3_spa_0, T_waist_l_c3_0, fc_l_c3 ) ;
    yarp::sig::Matrix Temp_3 = Q_l_c3_1 - Q_l_c3;
    std::cout << " Temp_3 = " <<  std::endl << Temp_3.toString() << std::endl;
 
  
  
  
  
//-----------------------------------------------------------------------------------------------------     
    // Drivative for l_c4 
    for ( int i = 0  ; i<(J_waist_l_c4_spa_0.cols()) ; i++ )  // i<(J_waist_l_c1_spa_0.cols()-1)
     {
       Q_l_c4_col_i =  (D_Jacob_spa_i(J_waist_l_c4_spa_0, (i+1))).transposed()*
              ( Adjoint( iHomogeneous(T_waist_l_c4_0))).transposed()*
                                        B *fc_l_c4  +
                                      J_waist_l_c4_spa_0.transposed()* 
              ( Adjoint( iHomogeneous(T_waist_l_c4_0))*
                         ad_lie( -1.0*J_waist_l_c4_spa_0.getCol(i))).transposed()*
                                        B *fc_l_c4;
       Q_l_c4.setCol(i, Q_l_c4_col_i ) ;       
     } ;   
    yarp::sig::Matrix Q_l_c = Q_l_c1 + Q_l_c2 + Q_l_c3 + Q_l_c4 ;


         yarp::sig::Matrix Q_l_c4_1 =  Q_ci(J_waist_l_c4_spa_0, T_waist_l_c4_0, fc_l_c4 ) ;
    yarp::sig::Matrix Temp_4 = Q_l_c4_1 - Q_l_c4;
    std::cout << " Temp_4 = " <<  std::endl << Temp_4.toString() << std::endl;
 
    
    
    //-------------------------------------------- 
        // Drivative for r_c1 
    for ( int i = 0  ; i<(J_waist_r_c1_spa_0.cols()) ; i++ )  // i<(J_waist_l_c1_spa_0.cols()-1)
     {
       Q_r_c1_col_i =  (D_Jacob_spa_i(J_waist_r_c1_spa_0, (i+1))).transposed()*
              ( Adjoint( iHomogeneous(T_waist_r_c1_0))).transposed()*
                                        B *fc_r_c1  +
                                      J_waist_r_c1_spa_0.transposed()* 
              ( Adjoint( iHomogeneous(T_waist_r_c1_0))*
                         ad_lie( -1.0*J_waist_r_c1_spa_0.getCol(i))).transposed()*
                                        B *fc_r_c1;
       Q_r_c1.setCol(i, Q_r_c1_col_i ) ;       
     } ;   

    yarp::sig::Matrix Q_r_c1_1 =  Q_ci(J_waist_r_c1_spa_0, T_waist_r_c1_0, fc_r_c1 ) ;
    yarp::sig::Matrix Temp_5 = Q_r_c1_1 - Q_r_c1;
    std::cout << " Temp_5 = " <<  std::endl << Temp_5.toString() << std::endl;
 
    
    //-------------------------------------------- 
   // Drivative for r_c2 
    for ( int i = 0  ; i<(J_waist_r_c2_spa_0.cols()) ; i++ )  // i<(J_waist_l_c1_spa_0.cols()-1)
     {
       Q_r_c2_col_i =  (D_Jacob_spa_i(J_waist_r_c2_spa_0, (i+1))).transposed()*
              ( Adjoint( iHomogeneous(T_waist_r_c2_0))).transposed()*
                                        B *fc_r_c2  +
                                      J_waist_r_c2_spa_0.transposed()* 
              ( Adjoint( iHomogeneous(T_waist_r_c2_0))*
                         ad_lie( -1.0*J_waist_r_c2_spa_0.getCol(i))).transposed()*
                                        B *fc_r_c2;
       Q_r_c2.setCol(i, Q_r_c2_col_i ) ;       
     } ;   
  /*   std::cout << " Q_r_c2 = " <<  std::endl << Q_r_c2.toString() << std::endl; 
     std::cout << " fc_r_c2 = " <<  std::endl << fc_r_c2.toString() << std::endl; */
      yarp::sig::Matrix Q_r_c2_1 =  Q_ci(J_waist_r_c2_spa_0, T_waist_r_c2_0, fc_r_c2 ) ;
    yarp::sig::Matrix Temp_6 = Q_r_c2_1 - Q_r_c2;
    std::cout << " Temp_6 = " <<  std::endl << Temp_6.toString() << std::endl;
 

  
  
 //-------------------------------------------- 
 // Drivative for r_c3 
    for ( int i = 0  ; i<(J_waist_r_c3_spa_0.cols()) ; i++ )  // i<(J_waist_l_c1_spa_0.cols()-1)
     {
       Q_r_c3_col_i =  (D_Jacob_spa_i(J_waist_r_c3_spa_0, (i+1))).transposed()*
              ( Adjoint( iHomogeneous(T_waist_r_c3_0))).transposed()*
                                        B *fc_r_c3  +
                                      J_waist_r_c3_spa_0.transposed()* 
              ( Adjoint( iHomogeneous(T_waist_r_c3_0))*
                         ad_lie( -1.0*J_waist_r_c3_spa_0.getCol(i))).transposed()*
                                        B *fc_r_c3;
       Q_r_c3.setCol(i, Q_r_c3_col_i ) ;       
     } ;   
  /*   std::cout << " Q_r_c3 = " <<  std::endl << Q_r_c3.toString() << std::endl; 
     std::cout << " fc_r_c3 = " <<  std::endl << fc_r_c3.toString() << std::endl;      */
         yarp::sig::Matrix Q_r_c3_1 =  Q_ci(J_waist_r_c3_spa_0, T_waist_r_c3_0, fc_r_c3 ) ;
    yarp::sig::Matrix Temp_7 = Q_r_c3_1 - Q_r_c3;
    std::cout << " Temp_7 = " <<  std::endl << Temp_7.toString() << std::endl;
 
 
  
  
  
  
 //-------------------------------------------- 
 // Drivative for r_c4
    for ( int i = 0  ; i<(J_waist_r_c4_spa_0.cols()) ; i++ )  // i<(J_waist_l_c1_spa_0.cols()-1)
     {
       Q_r_c4_col_i =  (D_Jacob_spa_i(J_waist_r_c4_spa_0, (i+1))).transposed()*
              ( Adjoint( iHomogeneous(T_waist_r_c4_0))).transposed()*
                                        B *fc_r_c4  +
                                      J_waist_r_c4_spa_0.transposed()* 
              ( Adjoint( iHomogeneous(T_waist_r_c4_0))*
                         ad_lie( -1.0*J_waist_r_c4_spa_0.getCol(i))).transposed()*
                                        B *fc_r_c4;
       Q_r_c4.setCol(i, Q_r_c4_col_i ) ;       
     } ;   
   yarp::sig::Matrix Q_r_c =  Q_r_c1 +Q_r_c2 + Q_r_c3 + Q_r_c4;
   yarp::sig::Matrix Q_c =  Q_l_c + Q_r_c ;
   
    yarp::sig::Matrix Q_r_c4_1 =  Q_ci(J_waist_r_c4_spa_0, T_waist_r_c4_0, fc_r_c4 ) ;
    yarp::sig::Matrix Temp_8 = Q_r_c4_1 - Q_r_c4;
    std::cout << " Temp_8 = " <<  std::endl << Temp_8.toString() << std::endl;
 
   
   
    
/*   std::cout << " Q_r_c4 = " <<  std::endl << Q_r_c4.toString() << std::endl; 
   std::cout << " fc_r_c4 = " <<  std::endl << fc_r_c4.toString() << std::endl;      
   std::cout << " Q_r_c = " <<  std::endl << Q_r_c.toString() << std::endl;
   std::cout << " Q_c = " <<  std::endl << Q_c.toString() << std::endl;     */
 
 
