rix T_l_c2_w_0 = iHomogeneous( T_w_l_c2_0 )  ;
     yarp::sig::Matrix T_l_c3_w_0 = iHomogeneous( T_w_l_c3_0 )  ;
     yarp::sig::Matrix T_l_c4_w_0 = iHomogeneous( T_w_l_c4_0 )  ;

     yarp::sig::Matrix T_r_c1_w_0 = iHomogeneous( T_w_r_c1_0 )  ;
     yarp::sig::Matrix T_r_c2_w_0 = iHomogeneous( T_w_r_c2_0 )  ;
     yarp::sig::Matrix T_r_c3_w_0 = iHomogeneous( T_w_r_c3_0 )  ;
     yarp::sig::Matrix T_r_c4_w_0 = iHomogeneous( T_w_r_c4_0 )  ;       
     //---------------------------------------------------------------------
     yarp::sig::Vector fc_l_c1(3) ;  //
     yarp::sig::Vector fc_l_c2(3) ;  // = fc_l_foot_upper_right_link ;
     yarp::sig::Vector fc_l_c3(3) ;  // = fc_l_foot_lower_left_link  ;
     yarp::sig::Vector fc_l_c4(3) ;  // = fc_l_foot_lower_right_link ;

     yarp::sig::Vector fc_r_c1(3) ;  // = fc_r_foot_upper_left_link  ;
     yarp::sig::Vector fc_r_c2(3) ;  // = fc_r_foot_upper_right_link ;
     yarp::sig::Vector fc_r_c3(3) ;  // = fc_r_foot_lower_left_link  ;
     yarp::sig::Vector fc_r_c4(3) ;  // = fc_r_foot_lower_right_link ;  
     
     fc_l_c1.setSubvector( 0, fc_l_contacts_to_world.subVector(0,2) )   ;
     fc_l_c2.setSubvector( 0, fc_l_contacts_to_world.subVector(3,5) )   ;
     fc_l_c3.setSubvector( 0, fc_l_contacts_to_world.subVector(6,8) )   ;
     fc_l_c4.setSubvector( 0, fc_l_contacts_to_world.subVector(9,11) )  ;

     fc_r_c1.setSubvector( 0, fc_r_contacts_to_world.subVector(0,2) )   ;
     fc_r_c2.setSubvector( 0, fc_r_contacts_to_world.subVector(3,5) )   ;
     fc_r_c3.setSubvector( 0, fc_r_contacts_to_world.subVector(6,8) )   ;
     fc_r_c4.setSubvector( 0, fc_r_contacts_to_world.subVector(9,11) )  ;

     //-------------------------------------------------------------------------------------------------------------------
     // REMIND NOTATION
     //  Jac_aw_l_foot_upper_left_link       -> spatial long (+6 joints) in the initial configuration (not incemented for derivatives)
     //  Jac_aw_l_foot_upper_left_link_body  -> body long (+6 joints) in the initial configuration (not incemented for derivatives)
     //  Jac_l_c1_spa_0                      -> spatial short in the initial configuration (not incemented for derivatives)
     //  Jac_l_c1_body_0                     -> spatial short in the initial configuration (not incemented for derivatives)
  
 //    std::cout << " qui 2" <<  std::endl   ; 	

     
     // Spatial Jacobian Short   
     yarp::sig::Matrix Jac_l_c1_spa_0 = Jac_aw_l_foot_upper_left_link.submatrix( 0,5 , 6, Jac_aw_l_foot_upper_left_link.cols() -1  )  ; // Spacial Jacobians Reduced
     yarp::sig::Matrix Jac_l_c2_spa_0 = Jac_aw_l_foot_upper_right_link.submatrix(0,5 , 6, Jac_aw_l_foot_upper_right_link.cols() -1  )  ;
     yarp::sig::Matrix Jac_l_c3_spa_0 = Jac_aw_l_foot_lower_left_link.submatrix( 0,5 , 6, Jac_aw_l_foot_lower_left_link.cols() -1  )  ;
     yarp::sig::Matrix Jac_l_c4_spa_0 = Jac_aw_l_foot_lower_right_link.submatrix(0,5 , 6, Jac_aw_l_foot_lower_right_link.cols() -1  )  ;
    
     yarp::sig::Matrix Jac_r_c1_spa_0 = Jac_aw_r_foot_upper_left_link.submatrix( 0,5 , 6, Jac_aw_r_foot_upper_left_link.cols() -1  )  ; // Spacial Jacobians Reduced
     yarp::sig::Matrix Jac_r_c2_spa_0 = Jac_aw_r_foot_upper_right_link.submatrix(0,5 , 6, Jac_aw_r_foot_upper_right_link.cols() -1  )  ; // Spacial Jacobians Reduced
     yarp::sig::Matrix Jac_r_c3_spa_0 = Jac_aw_r_foot_lower_left_link.submatrix( 0,5 , 6, Jac_aw_r_foot_lower_left_link.cols() -1  )  ; // Spacial Jacobians Reduced
     yarp::sig::Matrix Jac_r_c4_spa_0 = Jac_aw_r_foot_lower_right_link.submatrix(0,5 , 6, Jac_aw_r_foot_lower_right_link.cols() -1  )  ; // Spacial Jacobians Reduced
     
     // Body Jacobian Short   
     yarp::sig::Matrix Jac_l_c1_body_0 = Jac_aw_l_foot_upper_left_link_body.submatrix( 0,5 , 6, Jac_aw_l_foot_upper_left_link_body.cols() -1  )  ; // Spacial Jacobians Reduced
     yarp::sig::Matrix Jac_l_c2_body_0 = Jac_aw_l_foot_upper_right_link_body.submatrix(0,5 , 6, Jac_aw_l_foot_upper_right_link_body.cols() -1  )  ;
     yarp::sig::Matrix Jac_l_c3_body_0 = Jac_aw_l_foot_lower_left_link_body.submatrix( 0,5 , 6, Jac_aw_l_foot_lower_left_link_body.cols() -1  )  ;
     yarp::sig::Matrix Jac_l_c4_body_0 = Jac_aw_l_foot_lower_right_link_body.submatrix(0,5 , 6, Jac_aw_l_foot_lower_right_link_body.cols() -1  )  ;
    
     yarp::sig::Matrix Jac_r_c1_body_0 = Jac_aw_r_foot_upper_left_link_body.submatrix( 0,5 , 6, Jac_aw_r_foot_upper_left_link_body.cols() -1  )  ; // Spacial Jacobians Reduced
     yarp::sig::Matrix Jac_r_c2_body_0 = Jac_aw_r_foot_upper_right_link_body.submatrix(0,5 , 6, Jac_aw_r_foot_upper_right_link_body.cols() -1  )  ; // Spacial Jacobians Reduced
     yarp::sig::Matrix Jac_r_c3_body_0 = Jac_aw_r_foot_lower_left_link_body.submatrix( 0,5 , 6, Jac_aw_r_foot_lower_left_link_body.cols() -1  )  ; // Spacial Jacobians Reduced
     yarp::sig::Matrix Jac_r_c4_body_0 = Jac_aw_r_foot_lower_right_link_body.submatrix(0,5 , 6, Jac_aw_r_foot_lower_right_link_body.cols() -1  )  ; // Spacial Jacobians Reduced         
 
 
 //    std::cout << " qui 2.1" <<  std::endl   ; 	

     //------------------------------------------------------------------------------------------
     // Computing U_s
     yarp::sig::Vector xi_1(6, 0.0)  ;
     yarp::sig::Vector xi_2(6, 0.0)  ;
     yarp::sig::Vector xi_3(6, 0.0)  ;
     yarp::sig::Vector xi_4(6, 0.0)  ;
     yarp::sig::Vector xi_5(6, 0.0)  ;
     yarp::sig::Vector xi_6(6, 0.0)  ;
     
     xi_1[0] = 1 ;
     xi_2[1] = 1 ;
     xi_3[2] = 1 ;
     xi_4[3] = 1 ;
     xi_5[4] = 1 ;
     xi_6[5] = 1 ;
   
     yarp::sig::Vector u_curr( 6, 0.0 )  ;
     yarp::sig::Vector u_incr( 6, 0.0 )  ;

 //    std::cout << " qui 2.2" <<  std::endl   ; 	


     yarp::sig::Vector xi_2_ap = Adjoint( twistexp(xi_1, u_curr(0)) )  *xi_2 ;
     yarp::sig::Vector xi_3_ap = Adjoint( twistexp(xi_1, u_curr(0))
                                         *twistexp(xi_2, u_curr(1))  ) *xi_3 ;
     yarp::sig::Vector xi_4_ap = Adjoint( twistexp(xi_1, u_curr(0))*
                                          twistexp(xi_2, u_curr(1))*
                                          twistexp(xi_3, u_curr(2))  ) *xi_4 ;
     yarp::sig::Vector xi_5_ap = Adjoint( twistexp(xi_1, u_curr(0))*
                                          twistexp(xi_2, u_curr(1))*
                                          twistexp(xi_3, u_curr(2))*
                                          twistexp(xi_4, u_curr(3))  ) *xi_5 ;
     yarp::sig::Vector xi_6_ap = Adjoint( twistexp(xi_1, u_curr(0))
                                         *twistexp(xi_2, u_curr(1))*
                                          twistexp(xi_3, u_curr(2))*
                                          twistexp(xi_4, u_curr(3))*
                                          twistexp(xi_5, u_curr(4))  ) *xi_6 ;
 //    std::cout << " qui 3" <<  std::endl   ; 	

       // Initial Complete Jacobian
     // First block, about VKC
     yarp::sig::Matrix Jac_u_l_c1_0 = Adjoint( T_l_c1_aw_0)* Eye_6 ;
     yarp::sig::Matrix Jac_u_l_c2_0 = Adjoint( T_l_c2_aw_0)* Eye_6 ;
     yarp::sig::Matrix Jac_u_l_c3_0 = Adjoint( T_l_c3_aw_0)* Eye_6 ;
     yarp::sig::Matrix Jac_u_l_c4_0 = Adjoint( T_l_c4_aw_0)* Eye_6 ;

     yarp::sig::Matrix Jac_u_r_c1_0 = Adjoint( T_r_c1_aw_0)* Eye_6 ;
     yarp::sig::Matrix Jac_u_r_c2_0 = Adjoint( T_r_c2_aw_0)* Eye_6 ;
     yarp::sig::Matrix Jac_u_r_c3_0 = Adjoint( T_r_c3_aw_0)* Eye_6 ;
     yarp::sig::Matrix Jac_u_r_c4_0 = Adjoint( T_r_c4_aw_0)* Eye_6 ;
    
  /*   yarp::sig::Matrix Stance_transp_temp( 3*8,6); 
     Stance_transp_temp.setSubmatrix( B_select*Jac_u_l_c1_0 , 0, 0) ;
     Stance_transp_temp.setSubmatrix( B_select*Jac_u_l_c2_0 , 3, 0) ;
     Stance_transp_temp.setSubmatrix( B_select*Jac_u_l_c3_0 , 2*3 , 0) ;
     Stance_transp_temp.setSubmatrix( B_select*Jac_u_l_c4_0 , 3*3, 0) ;

     Stance_transp_temp.setSubmatrix( B_select*Jac_u_r_c1_0 , 4*3, 0) ;
     Stance_transp_temp.setSubmatrix( B_select*Jac_u_r_c2_0 , 5*3, 0) ;
     Stance_transp_temp.setSubmatrix( B_select*Jac_u_r_c3_0 , 6*3, 0) ;
     Stance_transp_temp.setSubmatrix( B_select*Jac_u_r_c4_0 , 7*3, 0) ;
     
     std::cout << " d_Stance_trv = " <<  std::endl  << (Stance_c_tranposed-Stance_transp_temp).toString() << std::endl   ; 	
      */
     
     // Second Block, about the real robot
     yarp::sig::Matrix Jac_q_l_c1_0 = Adjoint( T_l_c1_aw_0) * Jac_l_c1_spa_0 ;   //These are body Jacobians short				   
     yarp::sig::Matrix Jac_q_l_c2_0 = Adjoint( T_l_c2_aw_0) * Jac_l_c2_spa_0 ; 
     yarp::sig::Matrix Jac_q_l_c3_0 = Adjoint( T_l_c3_aw_0) * Jac_l_c3_spa_0 ; 
     yarp::sig::Matrix Jac_q_l_c4_0 = Adjoint( T_l_c4_aw_0) * Jac_l_c4_spa_0 ; 	  

     yarp::sig::Matrix Jac_q_r_c1_0 = Adjoint( T_r_c1_aw_0) * Jac_r_c1_spa_0 ;   				   
     yarp::sig::Matrix Jac_q_r_c2_0 = Adjoint( T_r_c2_aw_0) * Jac_r_c2_spa_0 ; 
     yarp::sig::Matrix Jac_q_r_c3_0 = Adjoint( T_r_c3_aw_0) * Jac_r_c3_spa_0 ; 
     yarp::sig::Matrix Jac_q_r_c4_0 = Adjoint( T_r_c4_aw_0) * Jac_r_c4_spa_0 ; 
     
     // Complete Jacobian
     yarp::sig::Matrix Jac_uq_l_c1_0(6, robot.getNumberOfJoints() + 6 )  ;
     yarp::sig::Matrix Jac_uq_l_c2_0(6, robot.getNumberOfJoints() + 6 )  ;
     yarp::sig::Matrix Jac_uq_l_c3_0(6, robot.getNumberOfJoints() + 6 )  ;
     yarp::sig::Matrix Jac_uq_l_c4_0(6, robot.getNumberOfJoints() + 6 )  ;

     yarp::sig::Matrix Jac_uq_r_c1_0(6, robot.getNumberOfJoints() + 6 )  ;
     yarp::sig::Matrix Jac_uq_r_c2_0(6, robot.getNumberOfJoints() + 6 )  ;
     yarp::sig::Matrix Jac_uq_r_c3_0(6, robot.getNumberOfJoints() + 6 )  ;
     yarp::sig::Matrix Jac_uq_r_c4_0(6, robot.getNumberOfJoints() + 6 )  ;
 //    std::cout << " qui 4" <<  std::endl   ; 	

     Jac_uq_l_c1_0.setSubmatrix( Jac_u_l_c1_0 , 0 ,  0 )  ;
     Jac_uq_l_c2_0.setSubmatrix( Jac_u_l_c2_0 , 0 ,  0 )  ;
     Jac_uq_l_c3_0.setSubmatrix( Jac_u_l_c3_0 , 0 ,  0 )  ;
     Jac_uq_l_c4_0.setSubmatrix( Jac_u_l_c4_0 , 0 ,  0 )  ;

     Jac_uq_r_c1_0.setSubmatrix( Jac_u_r_c1_0 , 0 ,  0 )  ;
     Jac_uq_r_c2_0.setSubmatrix( Jac_u_r_c2_0 , 0 ,  0 )  ;
     Jac_uq_r_c3_0.setSubmatrix( Jac_u_r_c3_0 , 0 ,  0 )  ;
     Jac_uq_r_c4_0.setSubmatrix( Jac_u_r_c4_0 , 0 ,  0 )  ;

     Jac_uq_l_c1_0.setSubmatrix( Jac_q_l_c1_0 , 0 ,  6 )  ;
     Jac_uq_l_c2_0.setSubmatrix( Jac_q_l_c2_0 , 0 ,  6 )  ;
     Jac_uq_l_c3_0.setSubmatrix( Jac_q_l_c3_0 , 0 ,  6 )  ;
     Jac_uq_l_c4_0.setSubmatrix( Jac_q_l_c4_0 , 0 ,  6 )  ;

     Jac_uq_r_c1_0.setSubmatrix( Jac_q_r_c1_0 , 0 ,  6 )  ;
     Jac_uq_r_c2_0.setSubmatrix( Jac_q_r_c2_0 , 0 ,  6 )  ;
     Jac_uq_r_c3_0.setSubmatrix( Jac_q_r_c3_0 , 0 ,  6 )  ;
     Jac_uq_r_c4_0.setSubmatrix( Jac_q_r_c4_0 , 0 ,  6 )  ;

     yarp::sig::Matrix Jac_aw_b_u1(6,6) ;
     yarp::sig::Matrix Jac_b_aw_b_u1( 6, 6 ) ;
     yarp::sig::Matrix Jac_u_l_c1_u1(6,6) ;
     yarp::sig::Matrix Jac_u_l_c2_u1(6,6) ;
     yarp::sig::Matrix Jac_u_l_c3_u1(6,6) ;
     yarp::sig::Matrix Jac_u_l_c4_u1(6,6) ;

     yarp::sig::Matrix Jac_u_r_c1_u1(6,6) ;
     yarp::sig::Matrix Jac_u_r_c2_u1(6,6) ;
     yarp::sig::Matrix Jac_u_r_c3_u1(6,6) ;
     yarp::sig::Matrix Jac_u_r_c4_u1(6,6) ;

     yarp::sig::Matrix Jac_q_l_c1_u1(6,6) ;
     yarp::sig::Matrix Jac_q_l_c2_u1(6,6) ;
     yarp::sig::Matrix Jac_q_l_c3_u1(6,6) ;
     yarp::sig::Matrix Jac_q_l_c4_u1(6,6) ;

     yarp::sig::Matrix Jac_q_r_c1_u1(6,6) ;
     yarp::sig::Matrix Jac_q_r_c2_u1(6,6) ;
     yarp::sig::Matrix Jac_q_r_c3_u1(6,6) ;
     yarp::sig::Matrix Jac_q_r_c4_u1(6,6) ;
     
     yarp::sig::Matrix Jac_uq_l_c1_u1(6, robot.getNumberOfJoints() + 6 ) ;
     yarp::sig::Matrix Jac_uq_l_c2_u1(6, robot.getNumberOfJoints() + 6 ) ;
     yarp::sig::Matrix Jac_uq_l_c3_u1(6, robot.getNumberOfJoints() + 6 )  ;
     yarp::sig::Matrix Jac_uq_l_c4_u1(6, robot.getNumberOfJoints() + 6 ) ;

     yarp::sig::Matrix Jac_uq_r_c1_u1(6, robot.getNumberOfJoints() + 6 ) ;
     yarp::sig::Matrix Jac_uq_r_c2_u1(6, robot.getNumberOfJoints() + 6 ) ;
     yarp::sig::Matrix Jac_uq_r_c3_u1(6, robot.getNumberOfJoints() + 6 ) ;
     yarp::sig::Matrix Jac_uq_r_c4_u1(6, robot.getNumberOfJoints() + 6 ) ;
     
     yarp::sig::Matrix T_aw_b_u1(6,6) ;
     yarp::sig::Matrix T_b_aw_u1(6,6) ;
     
     yarp::sig::Matrix d_Jac_uq_l_c1_u1( 6, robot.getNumberOfJoints() + 6  )  ;
     yarp::sig::Matrix d_Jac_uq_l_c2_u1( 6, robot.getNumberOfJoints() + 6  )  ;    
     yarp::sig::Matrix d_Jac_uq_l_c3_u1( 6, robot.getNumberOfJoints() + 6  )  ; 
     yarp::sig::Matrix d_Jac_uq_l_c4_u1( 6, robot.getNumberOfJoints() + 6  )  ;

     yarp::sig::Matrix d_Jac_uq_r_c1_u1( 6, robot.getNumberOfJoints() + 6  )  ;
     yarp::sig::Matrix d_Jac_uq_r_c2_u1( 6, robot.getNumberOfJoints() + 6  )  ;    
     yarp::sig::Matrix d_Jac_uq_r_c3_u1( 6, robot.getNumberOfJoints() + 6  )  ; 
     yarp::sig::Matrix d_Jac_uq_r_c4_u1( 6, robot.getNumberOfJoints() + 6  )  ;
     
     yarp::sig::Vector U_l_c1_col_i( robot.getNumberOfJoints() + 6   ) ;   //  = d_Jac_uq_l_c1.transposed()*B_select.transposed()*fc_l_c1  ;
     yarp::sig::Vector U_l_c2_col_i( robot.getNumberOfJoints() + 6   ) ;   //  = d_Jac_uq_l_c1.transposed()*B_select.transposed()*fc_l_c1  ;
     yarp::sig::Vector U_l_c3_col_i( robot.getNumberOfJoints() + 6   ) ;   //  = d_Jac_uq_l_c1.transposed()*B_select.transposed()*fc_l_c1  ;
     yarp::sig::Vector U_l_c4_col_i( robot.getNumberOfJoints() + 6   ) ;   //  = d_Jac_uq_l_c1.transposed()*B_select.transposed()*fc_l_c1  ;

     yarp::sig::Vector U_r_c1_col_i( robot.getNumberOfJoints() + 6   ) ;   //  = d_Jac_uq_l_c1.transposed()*B_select.transposed()*fc_l_c1  ;
     yarp::sig::Vector U_r_c2_col_i( robot.getNumberOfJoints() + 6   ) ;   //  = d_Jac_uq_l_c1.transposed()*B_select.transposed()*fc_l_c1  ;
     yarp::sig::Vector U_r_c3_col_i( robot.getNumberOfJoints() + 6   ) ;   //  = d_Jac_uq_l_c1.transposed()*B_select.transposed()*fc_l_c1  ;
     yarp::sig::Vector U_r_c4_col_i( robot.getNumberOfJoints() + 6   ) ;   //  = d_Jac_uq_l_c1.transposed()*B_select.transposed()*fc_l_c1  ;
 
     yarp::sig::Matrix U_l_c1( robot.getNumberOfJoints() + 6 , 6 ) ; 
     yarp::sig::Matrix U_l_c2( robot.getNumberOfJoints() + 6 , 6 ) ; 
     yarp::sig::Matrix U_l_c3( robot.getNumberOfJoints() + 6 , 6 ) ; 
     yarp::sig::Matrix U_l_c4( robot.getNumberOfJoints() + 6 , 6 ) ; 

     yarp::sig::Matrix U_r_c1( robot.getNumberOfJoints() + 6 , 6 ) ; 
     yarp::sig::Matrix U_r_c2( robot.getNumberOfJoints() + 6 , 6 ) ; 
     yarp::sig::Matrix U_r_c3( robot.getNumberOfJoints() + 6 , 6 ) ; 
     yarp::sig::Matrix U_r_c4( robot.getNumberOfJoints() + 6 , 6 ) ; 
 
 //    std::cout << " ----------------------------------------------------------------------" <<  std::endl   ; 	
 //    std::cout << " Primo ciclo" <<  std::endl   ; 	

     yarp::sig::Matrix xi_4_temp(4,4) ;
      yarp::sig::Vector J_u_temp ;
     
    for ( int i = 0  ; i<6 ; i++ )  // i<6
     {
   //         std::cout << " i = " << i << std::endl   ; 	

      u_incr = u_curr ;
      u_incr[i] += h ; 
      
      T_aw_b_u1 = twistexp(xi_1, u_incr(0))*
                  twistexp(xi_2, u_incr(1))*
                  twistexp(xi_3, u_incr(2))*
                  twistexp(xi_4, u_incr(3))*
                  twistexp(xi_5, u_incr(4))*
                  twistexp(xi_6, u_incr(5));
      T_b_aw_u1 = iHomogeneous( T_aw_b_u1 )   ;
      
 //     std::cout << " T_aw_b_u1 = " <<  std::endl  <<  T_aw_b_u1.toString() << std::endl   ; 	
 //     std::cout << " T_b_aw_u1 = " <<  std::endl <<  T_b_aw_u1.toString() << std::endl   ; 	

      
      xi_2_ap = Adjoint( twistexp(xi_1, u_incr(0)) ) *xi_2 ;
      xi_3_ap = Adjoint( twistexp(xi_1, u_incr(0)) 
                        *twistexp(xi_2, u_incr(1)) ) *xi_3 ;
      xi_4_ap = Adjoint( twistexp(xi_1, u_incr(0))*
                         twistexp(xi_2, u_incr(1))*
                         twistexp(xi_3, u_incr(2)) ) *xi_4 ;
      xi_5_ap = Adjoint( twistexp(xi_1, u_incr(0))*
                         twistexp(xi_2, u_incr(1))*
                         twistexp(xi_3, u_incr(2))
			*twistexp(xi_4, u_incr(3)) ) *xi_5 ;
      xi_6_ap = Adjoint( twistexp(xi_1, u_incr(0))
                        *twistexp(xi_2, u_incr(1))*
                         twistexp(xi_3, u_incr(2))*
                         twistexp(xi_4, u_incr(3))
		        *twistexp(xi_5, u_incr(4)) ) *xi_6 ;
 //     std::cout << " u_incr = " <<  std::endl << (u_incr/h).toString() <<  std::endl   ; 	
     
     // xi_4_temp =  (twistexp(xi_4, u_incr(3)))/h ;
 //     std::cout << " xi_4_temp = " <<  std::endl << xi_4_temp.toString() <<  std::endl   ; 	
      
      
      Jac_aw_b_u1.setCol(0, xi_1) ; //Spatial version, in {AW}				   
      Jac_aw_b_u1.setCol(1, xi_2_ap) ;
      Jac_aw_b_u1.setCol(2, xi_3_ap) ;
      Jac_aw_b_u1.setCol(3, xi_4_ap) ;
      Jac_aw_b_u1.setCol(4, xi_5_ap) ;
      Jac_aw_b_u1.setCol(5, xi_6_ap) ; 
      
  //     std::cout << " Jac_aw_b_u1 = "<<  std::endl  <<  Jac_aw_b_u1.toString() << std::endl   ; 	

  //    std::cout << " qui 6" <<  std::endl   ; 	

      Jac_b_aw_b_u1 = Adjoint( T_b_aw_u1 ) * Jac_aw_b_u1 ; // Body version, in {B}

      //       std::cout << " Jac_b_aw_b_u1 = " <<  std::endl <<  Jac_b_aw_b_u1.toString() << std::endl   ; 	
      // Ok fin qui

      
      Jac_u_l_c1_u1 =  Adjoint( T_l_c1_b_0 ) * Jac_b_aw_b_u1 ; // body in {C_i}
    //         std::cout << " T_l_c1_b_0 = "<<  std::endl  <<  T_l_c1_b_0.toString() << std::endl   ; 	
    //  std::cout << " Jac_u_l_c1_u1 = "<<  std::endl  <<  Jac_u_l_c1_u1.toString() << std::endl   ; 	
      
      Jac_u_l_c2_u1 =  Adjoint( T_l_c2_b_0 )* Jac_b_aw_b_u1 ; // body in {C_i}
      Jac_u_l_c3_u1 =  Adjoint( T_l_c3_b_0 )* Jac_b_aw_b_u1 ; // body in {C_i}
      Jac_u_l_c4_u1 =  Adjoint( T_l_c4_b_0 )* Jac_b_aw_b_u1 ; // body in {C_i}

      Jac_u_r_c1_u1 =  Adjoint( T_r_c1_b_0 )* Jac_b_aw_b_u1 ; // body in {C_i}
      Jac_u_r_c2_u1 =  Adjoint( T_r_c2_b_0 )* Jac_b_aw_b_u1 ; // body in {C_i}
      Jac_u_r_c3_u1 =  Adjoint( T_r_c3_b_0 )* Jac_b_aw_b_u1 ; // body in {C_i}
      Jac_u_r_c4_u1 =  Adjoint( T_r_c4_b_0 )* Jac_b_aw_b_u1 ; // body in {C_i}
  //    std::cout << " qui 7" <<  std::endl   ; 	

      Jac_q_l_c1_u1 = Jac_q_l_c1_0  ;
      Jac_q_l_c2_u1 = Jac_q_l_c2_0  ;
      Jac_q_l_c3_u1 = Jac_q_l_c3_0  ;
      Jac_q_l_c4_u1 = Jac_q_l_c4_0  ;

      Jac_q_r_c1_u1 = Jac_q_r_c1_0  ;
      Jac_q_r_c2_u1 = Jac_q_r_c2_0  ;
      Jac_q_r_c3_u1 = Jac_q_r_c3_0  ;
      Jac_q_r_c4_u1 = Jac_q_r_c4_0  ;
  //     std::cout << " qui 8" <<  std::endl   ; 	
    
      J_u_temp = Jac_u_l_c1_u1.transposed()*B_select.transposed()*fc_l_c1 -
                 Jac_u_l_c1_0.transposed()*B_select.transposed()*fc_l_c1 ;

      Jac_uq_l_c1_u1.setSubmatrix( Jac_u_l_c1_u1 , 0 ,  0 )  ;
      Jac_uq_l_c2_u1.setSubmatrix( Jac_u_l_c2_u1 , 0 ,  0 )  ;
      Jac_uq_l_c3_u1.setSubmatrix( Jac_u_l_c3_u1 , 0 ,  0 )  ;
      Jac_uq_l_c4_u1.setSubmatrix( Jac_u_l_c4_u1 , 0 ,  0 )  ;

      Jac_uq_r_c1_u1.setSubmatrix( Jac_u_r_c1_u1 , 0 ,  0 )  ; 
      Jac_uq_r_c2_u1.setSubmatrix( Jac_u_r_c2_u1 , 0 ,  0 )  ;
      Jac_uq_r_c3_u1.setSubmatrix( Jac_u_r_c3_u1 , 0 ,  0 )  ;
      Jac_uq_r_c4_u1.setSubmatrix( Jac_u_r_c4_u1 , 0 ,  0 )  ;

      Jac_uq_l_c1_u1.setSubmatrix( Jac_q_l_c1_u1 , 0 ,  6 )  ;
      Jac_uq_l_c2_u1.setSubmatrix( Jac_q_l_c2_u1 , 0 ,  6 )  ;
      Jac_uq_l_c3_u1.setSubmatrix( Jac_q_l_c3_u1 , 0 ,  6 )  ;
      Jac_uq_l_c4_u1.setSubmatrix( Jac_q_l_c4_u1 , 0 ,  6 )  ;

      Jac_uq_r_c1_u1.setSubmatrix( Jac_q_r_c1_u1 , 0 ,  6 )  ;
      Jac_uq_r_c2_u1.setSubmatrix( Jac_q_r_c2_u1 , 0 ,  6 )  ;
      Jac_uq_r_c3_u1.setSubmatrix( Jac_q_r_c3_u1 , 0 ,  6 )  ;
      Jac_uq_r_c4_u1.setSubmatrix( Jac_q_r_c4_u1 , 0 ,  6 )  ;
    
//      std::cout << "Jac_uq_l_c1_u1.cols() " <<  std::endl << Jac_uq_l_c1_u1.cols() <<  std::endl  ; 	
//     std::cout << "Jac_uq_l_c1_0.cols() " <<  std::endl << Jac_uq_l_c1_0.cols() <<  std::endl  ; 	
      
      d_Jac_uq_l_c1_u1 = ( Jac_uq_l_c1_u1 - Jac_uq_l_c1_0 )/h ;
      d_Jac_uq_l_c2_u1 = ( Jac_uq_l_c2_u1 - Jac_uq_l_c2_0 )/h ;
      d_Jac_uq_l_c3_u1 = ( Jac_uq_l_c3_u1 - Jac_uq_l_c3_0 )/h ;
      d_Jac_uq_l_c4_u1 = ( Jac_uq_l_c4_u1 - Jac_uq_l_c4_0 )/h ;

      d_Jac_uq_r_c1_u1 = ( Jac_uq_r_c1_u1 - Jac_uq_r_c1_0 )/h ;
      d_Jac_uq_r_c2_u1 = ( Jac_uq_r_c2_u1 - Jac_uq_r_c2_0 )/h ;
      d_Jac_uq_r_c3_u1 = ( Jac_uq_r_c3_u1 - Jac_uq_r_c3_0 )/h ;
      d_Jac_uq_r_c4_u1 = ( Jac_uq_r_c4_u1 - Jac_uq_r_c4_0 )/h ;
      //      std::cout << "Jac_uq_l_c1_u1 =  "<<  std::endl  << std::endl << Jac_uq_l_c1_u1.toString() << std::endl;  
      //      std::cout << "Jac_uq_l_c1_0 =  "<<  std::endl  << std::endl << Jac_uq_l_c1_0.toString() << std::endl;  

	    
      U_l_c1_col_i= d_Jac_uq_l_c1_u1.transposed()*B_select.transposed()*fc_l_c1  ;
      U_l_c2_col_i= d_Jac_uq_l_c2_u1.transposed()*B_select.transposed()*fc_l_c2  ;
      U_l_c3_col_i= d_Jac_uq_l_c3_u1.transposed()*B_select.transposed()*fc_l_c3  ;
      U_l_c4_col_i= d_Jac_uq_l_c4_u1.transposed()*B_select.transposed()*fc_l_c4  ;

      U_r_c1_col_i= d_Jac_uq_r_c1_u1.transposed()*B_select.transposed()*fc_r_c1  ;
      U_r_c2_col_i= d_Jac_uq_r_c2_u1.transposed()*B_select.transposed()*fc_r_c2  ;
      U_r_c3_col_i= d_Jac_uq_r_c3_u1.transposed()*B_select.transposed()*fc_r_c3  ;
      U_r_c4_col_i= d_Jac_uq_r_c4_u1.transposed()*B_select.transposed()*fc_r_c4  ;
       //     std::cout << "Jac_uq_l_c1_u1 =  " << std::endl << Jac_uq_l_c1_u1.toString() << std::endl;  
    
      U_l_c1.setCol(i, U_l_c1_col_i) ; 
      U_l_c2.setCol(i, U_l_c2_col_i) ; 
      U_l_c3.setCol(i, U_l_c3_col_i) ; 
      U_l_c4.setCol(i, U_l_c4_col_i) ; 

      U_r_c1.setCol(i, U_r_c1_col_i) ; 
      U_r_c2.setCol(i, U_r_c2_col_i) ; 
      U_r_c3.setCol(i, U_r_c3_col_i) ; 
      U_r_c4.setCol(i, U_r_c4_col_i) ;

    } ;  
    
 //   std::cout << "U_l_c1 =  " << std::endl << U_l_c1.toString() << std::endl;  

  /*    std::cout << "U_l_c1 =  " << std::endl << U_l_c1.toString() << std::endl;  
      std::cout << "U_l_c2 =  " << std::endl << U_l_c2.toString() << std::endl;  
      std::cout << "U_l_c3 =  " << std::endl << U_l_c3.toString() << std::endl;  
      std::cout << "U_l_c4 =  " << std::endl << U_l_c4.toString() << std::endl;  

      std::cout << "U_r_c1 =  " << std::endl << U_r_c1.toString() << std::endl;  
      std::cout << "U_r_c2 =  " << std::endl << U_r_c2.toString() << std::endl;  
      std::cout << "U_r_c3 =  " << std::endl << U_r_c3.toString() << std::endl;  
      std::cout << "U_r_c4 =  " << std::endl << U_r_c4.toString() << std::endl;     */

      yarp::sig::Matrix U_l_c_tot = U_l_c1 + U_l_c2 + U_l_c3  + U_l_c4 ;
      yarp::sig::Matrix U_r_c_tot = U_r_c1 + U_r_c2 + U_r_c3  + U_r_c4 ;

  //    std::cout << "U_l_c_tot =  " << std::endl << U_l_c_tot.toString() << std::endl;   
  //    std::cout << "U_r_c_tot =  " << std::endl << U_r_c_tot.toString() << std::endl;   

      yarp::sig::Matrix U_c_tot = U_l_c_tot + U_r_c_tot   ;
  /*   std::cout << "U_l_c1 =  " << std::endl << U_l_c1.toString() << std::endl;   
     std::cout << "U_l_c2 =  " << std::endl << U_l_c2.toString() << std::endl;   
     std::cout << "U_l_c3 =  " << std::endl << U_l_c3.toString() << std::endl;   
     std::cout << "U_l_c4 =  " << std::endl << U_l_c4.toString() << std::endl;   */

            
   // A second Version for computing U_....
      
      //int l_c1_index = l_foot_upper_left_link_index  ;

     yarp::sig::Matrix T_w_l_c1_init = model.iDyn3_model.getPosition( l_c1_index  )  ;
     yarp::sig::Matrix T_w_l_c2_init = model.iDyn3_model.getPosition( l_c2_index  )  ;
     yarp::sig::Matrix T_w_l_c3_init = model.iDyn3_model.getPosition( l_c3_index  )  ;
     yarp::sig::Matrix T_w_l_c4_init = model.iDyn3_model.getPosition( l_c4_index  )  ;

     yarp::sig::Matrix T_w_r_c1_init = model.iDyn3_model.getPosition( r_c1_index  )  ;
     yarp::sig::Matrix T_w_r_c2_init = model.iDyn3_model.getPosition( r_c2_index  )  ;
     yarp::sig::Matrix T_w_r_c3_init = model.iDyn3_model.getPosition( r_c3_index  )  ;
     yarp::sig::Matrix T_w_r_c4_init = model.iDyn3_model.getPosition( r_c4_index  )  ;

     yarp::sig::Matrix T_l_c1_w_init = iHomogeneous(T_w_l_c1_init ) ;
     yarp::sig::Matrix T_l_c2_w_init = iHomogeneous(T_w_l_c2_init ) ;
     yarp::sig::Matrix T_l_c3_w_init = iHomogeneous(T_w_l_c3_init ) ;
     yarp::sig::Matrix T_l_c4_w_init = iHomogeneous(T_w_l_c4_init ) ;

     yarp::sig::Matrix T_r_c1_w_init = iHomogeneous(T_w_r_c1_init ) ;
     yarp::sig::Matrix T_r_c2_w_init = iHomogeneous(T_w_r_c2_init ) ;
     yarp::sig::Matrix T_r_c3_w_init = iHomogeneous(T_w_r_c3_init ) ;
     yarp::sig::Matrix T_r_c4_w_init = iHomogeneous(T_w_r_c4_init ) ;
     
     yarp::sig::Matrix T_w_aw_init = T_w_aw ;
     yarp::sig::Matrix T_aw_w_init = iHomogeneous(T_w_aw_init) ;
   
     yarp::sig::Matrix T_l_c1_aw_init = T_l_c1_w_init * T_w_aw_init ;
     yarp::sig::Matrix T_l_c2_aw_init = T_l_c2_w_init * T_w_aw_init ;
     yarp::sig::Matrix T_l_c3_aw_init = T_l_c3_w_init * T_w_aw_init ;
     yarp::sig::Matrix T_l_c4_aw_init = T_l_c4_w_init * T_w_aw_init ;

     yarp::sig::Matrix T_r_c1_aw_init = T_r_c1_w_init * T_w_aw_init ;
     yarp::sig::Matrix T_r_c2_aw_init = T_r_c2_w_init * T_w_aw_init ;
     yarp::sig::Matrix T_r_c3_aw_init = T_r_c3_w_init * T_w_aw_init ;
     yarp::sig::Matrix T_r_c4_aw_init = T_r_c4_w_init * T_w_aw_init ;

     yarp::sig::Matrix J_u_l_c1_init = Adjoint( T_l_c1_aw_init) ;
     
  //   yarp::sig::Matrix jac_0_temp = (Jac_u_l_c1_0 - J_u_l_c1_init)/h ;
  //  std::cout << " jac_0_temp = " <<  std::endl << jac_0_temp.toString() <<  std::endl   ; 	

     yarp::sig::Matrix J_u_l_c2_init = Adjoint( T_l_c2_aw_init) ;
     yarp::sig::Matrix J_u_l_c3_init = Adjoint( T_l_c3_aw_init) ;
     yarp::sig::Matrix J_u_l_c4_init = Adjoint( T_l_c4_aw_init) ;

     yarp::sig::Matrix J_u_r_c1_init = Adjoint( T_r_c1_aw_init) ;
     yarp::sig::Matrix J_u_r_c2_init = Adjoint( T_r_c2_aw_init) ;
     yarp::sig::Matrix J_u_r_c3_init = Adjoint( T_r_c3_aw_init) ;
     yarp::sig::Matrix J_u_r_c4_init = Adjoint( T_r_c4_aw_init) ;

     robot.idynutils.updateiDyn3Model( q_current, true );   //update model first  

     yarp::sig::Matrix J_q_l_c1_mix_long_init(6, size_q + 6 ) ;
     yarp::sig::Matrix J_q_l_c2_mix_long_init(6, size_q + 6) ;
     yarp::sig::Matrix J_q_l_c3_mix_long_init(6, size_q + 6) ;
     yarp::sig::Matrix J_q_l_c4_mix_long_init(6, size_q + 6) ;

     yarp::sig::Matrix J_q_r_c1_mix_long_init(6, size_q + 6) ;
     yarp::sig::Matrix J_q_r_c2_mix_long_init(6, size_q + 6) ;
     yarp::sig::Matrix J_q_r_c3_mix_long_init(6, size_q + 6) ;
     yarp::sig::Matrix J_q_r_c4_mix_long_init(6, size_q + 6) ;

     yarp::sig::Matrix J_q_l_c1_mix_init(6, size_q ) ;
     yarp::sig::Matrix J_q_l_c2_mix_init(6, size_q ) ;
     yarp::sig::Matrix J_q_l_c3_mix_init(6, size_q ) ;
     yarp::sig::Matrix J_q_l_c4_mix_init(6, size_q ) ;

     yarp::sig::Matrix J_q_r_c1_mix_init(6, size_q ) ;
     yarp::sig::Matrix J_q_r_c2_mix_init(6, size_q ) ;
     yarp::sig::Matrix J_q_r_c3_mix_init(6, size_q ) ;
     yarp::sig::Matrix J_q_r_c4_mix_init(6, size_q ) ;

     model.iDyn3_model.getJacobian( l_c1_index  , J_q_l_c1_mix_long_init  , false  ) ; //false= mixed version jacobian //true= body jacobian
     model.iDyn3_model.getJacobian( l_c2_index  , J_q_l_c2_mix_long_init  , false  ) ; //false= mixed version jacobian //true= body jacobian
     model.iDyn3_model.getJacobian( l_c3_index  , J_q_l_c3_mix_long_init  , false  ) ; //false= mixed version jacobian //true= body jacobian
     model.iDyn3_model.getJacobian( l_c4_index  , J_q_l_c4_mix_long_init  , false  ) ; //false= mixed version jacobian //true= body jacobian

     model.iDyn3_model.getJacobian( r_c1_index  , J_q_r_c1_mix_long_init  , false  ) ; //false= mixed version jacobian //true= body jacobian
     model.iDyn3_model.getJacobian( r_c2_index  , J_q_r_c2_mix_long_init  , false  ) ; //false= mixed version jacobian //true= body jacobian
     model.iDyn3_model.getJacobian( r_c3_index  , J_q_r_c3_mix_long_init  , false  ) ; //false= mixed version jacobian //true= body jacobian
     model.iDyn3_model.getJacobian( r_c4_index  , J_q_r_c4_mix_long_init  , false  ) ; //false= mixed version jacobian //true= body jacobian

     J_q_l_c1_mix_init = J_q_l_c1_mix_long_init.submatrix(0,5, 6, J_q_l_c1_mix_long_init.cols()-1 ) ;
     J_q_l_c2_mix_init = J_q_l_c2_mix_long_init.submatrix(0,5, 6, J_q_l_c2_mix_long_init.cols()-1 ) ;
     J_q_l_c3_mix_init = J_q_l_c3_mix_long_init.submatrix(0,5, 6, J_q_l_c3_mix_long_init.cols()-1 ) ;
     J_q_l_c4_mix_init = J_q_l_c4_mix_long_init.submatrix(0,5, 6, J_q_l_c4_mix_long_init.cols()-1 ) ;

     J_q_r_c1_mix_init = J_q_r_c1_mix_long_init.submatrix(0,5, 6, J_q_r_c1_mix_long_init.cols()-1 ) ;
     J_q_r_c2_mix_init = J_q_r_c2_mix_long_init.submatrix(0,5, 6, J_q_r_c2_mix_long_init.cols()-1 ) ;
     J_q_r_c3_mix_init = J_q_r_c3_mix_long_init.submatrix(0,5, 6, J_q_r_c3_mix_long_init.cols()-1 ) ;
     J_q_r_c4_mix_init = J_q_r_c4_mix_long_init.submatrix(0,5, 6, J_q_r_c4_mix_long_init.cols()-1 ) ;

     yarp::sig::Matrix J_q_l_c1_init(6, size_q)  ;
     yarp::sig::Matrix J_q_l_c2_init(6, size_q)  ;
     yarp::sig::Matrix J_q_l_c3_init(6, size_q)  ;
     yarp::sig::Matrix J_q_l_c4_init(6, size_q)  ;

     yarp::sig::Matrix J_q_r_c1_init(6, size_q)  ;
     yarp::sig::Matrix J_q_r_c2_init(6, size_q)  ;
     yarp::sig::Matrix J_q_r_c3_init(6, size_q)  ;
     yarp::sig::Matrix J_q_r_c4_init(6, size_q)  ;

     J_q_l_c1_init = Adjoint( Homogeneous(  getRot(T_l_c1_w_init) , zero_3 ) ) * J_q_l_c1_mix_init ;
     J_q_l_c2_init = Adjoint( Homogeneous(  getRot(T_l_c2_w_init) , zero_3 ) ) * J_q_l_c2_mix_init ;
     J_q_l_c3_init = Adjoint( Homogeneous(  getRot(T_l_c3_w_init) , zero_3 ) ) * J_q_l_c3_mix_init ;
     J_q_l_c4_init = Adjoint( Homogeneous(  getRot(T_l_c4_w_init) , zero_3 ) ) * J_q_l_c4_mix_init ;

     J_q_r_c1_init = Adjoint( Homogeneous(  getRot(T_r_c1_w_init) , zero_3 ) ) * J_q_r_c1_mix_init ;
     J_q_r_c2_init = Adjoint( Homogeneous(  getRot(T_r_c2_w_init) , zero_3 ) ) * J_q_r_c2_mix_init ;
     J_q_r_c3_init = Adjoint( Homogeneous(  getRot(T_r_c3_w_init) , zero_3 ) ) * J_q_r_c3_mix_init ;
     J_q_r_c4_init = Adjoint( Homogeneous(  getRot(T_r_c4_w_init) , zero_3 ) ) * J_q_r_c4_mix_init ;

     yarp::sig::Matrix J_uq_l_c1_init(6 , size_q + 6) ;
     yarp::sig::Matrix J_uq_l_c2_init(6 , size_q + 6) ;
     yarp::sig::Matrix J_uq_l_c3_init(6 , size_q + 6) ;
     yarp::sig::Matrix J_uq_l_c4_init(6 , size_q + 6) ;

     yarp::sig::Matrix J_uq_r_c1_init(6 , size_q + 6) ;
     yarp::sig::Matrix J_uq_r_c2_init(6 , size_q + 6) ;
     yarp::sig::Matrix J_uq_r_c3_init(6 , size_q + 6) ;
     yarp::sig::Matrix J_uq_r_c4_init(6 , size_q + 6) ;

     J_uq_l_c1_init.setSubmatrix(J_u_l_c1_init , 0, 0) ;
     J_uq_l_c2_init.setSubmatrix(J_u_l_c2_init , 0, 0) ;
     J_uq_l_c3_init.setSubmatrix(J_u_l_c3_init , 0, 0) ;
     J_uq_l_c4_init.setSubmatrix(J_u_l_c4_init , 0, 0) ;

     J_uq_r_c1_init.setSubmatrix(J_u_r_c1_init , 0, 0) ;
     J_uq_r_c2_init.setSubmatrix(J_u_r_c2_init , 0, 0) ;
     J_uq_r_c3_init.setSubmatrix(J_u_r_c3_init , 0, 0) ;
     J_uq_r_c4_init.setSubmatrix(J_u_r_c4_init , 0, 0) ;
     
     J_uq_l_c1_init.setSubmatrix(J_q_l_c1_init , 0, 6) ;
     J_uq_l_c2_init.setSubmatrix(J_q_l_c2_init , 0, 6) ;
     J_uq_l_c3_init.setSubmatrix(J_q_l_c3_init , 0, 6) ;
     J_uq_l_c4_init.setSubmatrix(J_q_l_c4_init , 0, 6) ;

     J_uq_r_c1_init.setSubmatrix(J_q_r_c1_init , 0, 6) ;
     J_uq_r_c2_init.setSubmatrix(J_q_r_c2_init , 0, 6) ;
     J_uq_r_c3_init.setSubmatrix(J_q_r_c3_init , 0, 6) ;
     J_uq_r_c4_init.setSubmatrix(J_q_r_c4_init , 0, 6) ;   

     yarp::sig::Vector U_l_c1_init_col_i  =  J_uq_l_c1_init.transposed()*B_select.transposed()*fc_l_c1 ;
     yarp::sig::Vector U_l_c2_init_col_i  =  J_uq_l_c2_init.transposed()*B_select.transposed()*fc_l_c2 ;
     yarp::sig::Vector U_l_c3_init_col_i  =  J_uq_l_c3_init.transposed()*B_select.transposed()*fc_l_c3 ;
     yarp::sig::Vector U_l_c4_init_col_i  =  J_uq_l_c4_init.transposed()*B_select.transposed()*fc_l_c4 ;

     yarp::sig::Vector U_r_c1_init_col_i  =  J_uq_r_c1_init.transposed()*B_select.transposed()*fc_r_c1 ;
     yarp::sig::Vector U_r_c2_init_col_i  =  J_uq_r_c2_init.transposed()*B_select.transposed()*fc_r_c1 ;
     yarp::sig::Vector U_r_c3_init_col_i  =  J_uq_r_c3_init.transposed()*B_select.transposed()*fc_r_c1 ;
     yarp::sig::Vector U_r_c4_init_col_i  =  J_uq_r_c4_init.transposed()*B_select.transposed()*fc_r_c1 ;
     
     yarp::sig::Matrix J_u_l_c1_incr(6,6) ; // = Adjoint( T_l_c1_aw_init) ;
     yarp::sig::Matrix J_u_l_c2_incr(6,6) ; // = Adjoint( T_l_c2_aw_init) ;
     yarp::sig::Matrix J_u_l_c3_incr(6,6) ; // = Adjoint( T_l_c3_aw_init) ;
     yarp::sig::Matrix J_u_l_c4_incr(6,6) ; // = Adjoint( T_l_c4_aw_init) ;

     yarp::sig::Matrix J_u_r_c1_incr(6,6) ; // = Adjoint( T_r_c1_aw_init) ;
     yarp::sig::Matrix J_u_r_c2_incr(6,6) ; // = Adjoint( T_r_c2_aw_init) ;
     yarp::sig::Matrix J_u_r_c3_incr(6,6) ; // = Adjoint( T_r_c3_aw_init) ;
     yarp::sig::Matrix J_u_r_c4_incr(6,6) ; // = Adjoint( T_r_c4_aw_init) ;
     
     yarp::sig::Matrix J_uq_l_c1_incr(6, size_q + 6)  ;//.setSubmatrix(J_u_l_c1_incr , 0, 0) ;
     yarp::sig::Matrix J_uq_l_c2_incr(6, size_q + 6)  ;//.setSubmatrix(J_u_l_c2_incr , 0, 0) ;
     yarp::sig::Matrix J_uq_l_c3_incr(6, size_q + 6)  ;//.setSubmatrix(J_u_l_c3_incr , 0, 0) ;
     yarp::sig::Matrix J_uq_l_c4_incr(6, size_q + 6)  ;//.setSubmatrix(J_u_l_c4_incr , 0, 0) ;

     yarp::sig::Matrix J_uq_r_c1_incr(6, size_q + 6)  ;//.setSubmatrix(J_u_r_c1_incr , 0, 0) ;
     yarp::sig::Matrix J_uq_r_c2_incr(6, size_q + 6)  ;//.setSubmatrix(J_u_r_c2_incr , 0, 0) ;
     yarp::sig::Matrix J_uq_r_c3_incr(6, size_q + 6)  ;//.setSubmatrix(J_u_r_c3_incr , 0, 0) ;
     yarp::sig::Matrix J_uq_r_c4_incr(6, size_q + 6)  ;//.setSubmatrix(J_u_r_c4_incr , 0, 0) ;
     
  //    std::cout << " qui 0" <<  std::endl   ; 	

     yarp::sig::Matrix T_aw_b_incr(4,4) ;
     yarp::sig::Matrix T_b_aw_incr(4,4) ;
     yarp::sig::Matrix J_spa_VKC_incr(6,6) ;
     
     yarp::sig::Vector J_spa_VKC_incr_col_1(6) ;
     yarp::sig::Vector J_spa_VKC_incr_col_2(6) ;
     yarp::sig::Vector J_spa_VKC_incr_col_3(6) ;
     yarp::sig::Vector J_spa_VKC_incr_col_4(6) ;
     yarp::sig::Vector J_spa_VKC_incr_col_5(6) ;
     yarp::sig::Vector J_spa_VKC_incr_col_6(6) ;
     
     yarp::sig::Vector U_l_c1_incr_col_i(size_q + 6) ;
     yarp::sig::Vector U_l_c2_incr_col_i(size_q + 6) ;
     yarp::sig::Vector U_l_c3_incr_col_i(size_q + 6) ;
     yarp::sig::Vector U_l_c4_incr_col_i(size_q + 6) ;

     yarp::sig::Vector U_r_c1_incr_col_i(size_q + 6) ;
     yarp::sig::Vector U_r_c2_incr_col_i(size_q + 6) ;
     yarp::sig::Vector U_r_c3_incr_col_i(size_q + 6) ;
     yarp::sig::Vector U_r_c4_incr_col_i(size_q + 6) ;
     
     yarp::sig::Matrix U_l_c1_incr_init(size_q + 6, 6) ;     
     yarp::sig::Matrix U_l_c2_incr_init(size_q + 6, 6) ;     
     yarp::sig::Matrix U_l_c3_incr_init(size_q + 6, 6) ;     
     yarp::sig::Matrix U_l_c4_incr_init(size_q + 6, 6) ;     

     yarp::sig::Matrix U_r_c1_incr_init(size_q + 6, 6) ;     
     yarp::sig::Matrix U_r_c2_incr_init(size_q + 6, 6) ;     
     yarp::sig::Matrix U_r_c3_incr_init(size_q + 6, 6) ;     
     yarp::sig::Matrix U_r_c4_incr_init(size_q + 6, 6) ;     
     
     yarp::sig::Vector U_l_c1_incr_init_col_i(size_q + 6) ;
     yarp::sig::Vector U_l_c2_incr_init_col_i(size_q + 6) ;
     yarp::sig::Vector U_l_c3_incr_init_col_i(size_q + 6) ;
     yarp::sig::Vector U_l_c4_incr_init_col_i(size_q + 6) ;

     yarp::sig::Vector U_r_c1_incr_init_col_i(size_q + 6) ;
     yarp::sig::Vector U_r_c2_incr_init_col_i(size_q + 6) ;
     yarp::sig::Vector U_r_c3_incr_init_col_i(size_q + 6) ;
     yarp::sig::Vector U_r_c4_incr_init_col_i(size_q + 6) ;
     
     yarp::sig::Vector U_l_c1_col_i_temp ;
      yarp::sig::Matrix    U_l_c1_temp( size_q + 6,6) ;
       
//     std::cout << " ----------------------------------------------------------------------" <<  std::endl   ; 	
//     std::cout << " Secondo ciclo" <<  std::endl   ; 	
     for ( int i = 0  ; i<6 ; i++ )     //i<6 
     {
       
 //           std::cout << " i = " << i <<  std::endl   ; 	

      u_incr = u_curr ;
      u_incr[i] += h ;
      
      T_aw_b_incr = twistexp( xi_1 , u_incr[0] ) *
                    twistexp( xi_2 , u_incr[1] ) *
                    twistexp( xi_3 , u_incr[2]) *
                    twistexp( xi_4 , u_incr[3]) *
                    twistexp( xi_5 , u_incr[4]) *
                    twistexp( xi_6 , u_incr[5]) ;
      
		    
      T_b_aw_incr = iHomogeneous(T_aw_b_incr ) ;
  //       std::cout << " T_aw_b_incr = " <<  std::endl << T_aw_b_incr.toString() <<  std::endl   ; 	
  //       std::cout << " T_b_aw_incr = " <<  std::endl << T_b_aw_incr.toString() <<  std::endl   ; 	

      
      J_spa_VKC_incr_col_1 =   xi_1 ;
      J_spa_VKC_incr_col_2 =  Adjoint( twistexp( xi_1 , u_incr[0] ) ) *xi_2  ;
      J_spa_VKC_incr_col_3 =  Adjoint(  twistexp( xi_1 , u_incr[0] ) * twistexp( xi_2 , u_incr[1] ) ) * xi_3  ;
      J_spa_VKC_incr_col_4 =  Adjoint(  twistexp( xi_1 , u_incr[0] ) * twistexp( xi_2 , u_incr[1] ) *
                               twistexp( xi_3, u_incr[2] ) )* xi_4  ;
      J_spa_VKC_incr_col_5 = Adjoint(   twistexp( xi_1 , u_incr[0] ) * twistexp( xi_2 , u_incr[1] ) *
                               twistexp( xi_3, u_incr[2] )* twistexp( xi_4, u_incr[3] ) ) *xi_5  ;
      J_spa_VKC_incr_col_6 = Adjoint(   twistexp( xi_1 , u_incr[0] ) * 
                                        twistexp( xi_2 , u_incr[1] ) *
                                        twistexp( xi_3,  u_incr[2] )*
                                        twistexp( xi_4,  u_incr[3] )*
                                        twistexp( xi_5,  u_incr[4] ) ) *xi_6  ;

 //    std::cout << " u_incr = " <<  std::endl << (u_incr/h).toString() <<  std::endl   ; 	
   //   yarp::sig::Matrix xi_4_temp_2 =  (twistexp(xi_4, u_incr(3)))/h ;
   //   std::cout << " xi_4_temp_2 = " <<  std::endl << xi_4_temp_2.toString() <<  std::endl   ; 						
	
 //    yarp::sig::Matrix xi_4_temp_3 = xi_4_temp - xi_4_temp_2 ;
 //     std::cout << " xi_4_temp_3 = " <<  std::endl << xi_4_temp_3.toString() <<  std::endl   ; 						
      
      
//      yarp::sig::Vector xi_temp = (xi_6_ap -J_spa_VKC_incr_col_6)/h; 				
 //    std::cout << " xi_temp = " <<  std::endl << xi_temp.toString() <<  std::endl   ; 	
					
  //   std::cout << " T_b_aw_incr = " <<  std::endl << T_b_aw_incr.toString() <<  std::endl   ; 	

			       
      J_spa_VKC_incr.setCol(0, J_spa_VKC_incr_col_1 ) ;		       
      J_spa_VKC_incr.setCol(1, J_spa_VKC_incr_col_2 ) ;		       
      J_spa_VKC_incr.setCol(2, J_spa_VKC_incr_col_3 ) ;		       
      J_spa_VKC_incr.setCol(3, J_spa_VKC_incr_col_4 ) ;		       
      J_spa_VKC_incr.setCol(4, J_spa_VKC_incr_col_5 ) ;		       
      J_spa_VKC_incr.setCol(5, J_spa_VKC_incr_col_6 ) ;		       
 //    std::cout << " J_spa_VKC_incr = " <<  std::endl << J_spa_VKC_incr.toString() <<  std::endl   ; 	
	       
      J_u_l_c1_incr = Adjoint( T_l_c1_aw_0) * Adjoint( T_b_aw_incr) * J_spa_VKC_incr ;
      J_u_l_c2_incr = Adjoint( T_l_c2_aw_0) * Adjoint( T_b_aw_incr) * J_spa_VKC_incr ;
      J_u_l_c3_incr = Adjoint( T_l_c3_aw_0) * Adjoint( T_b_aw_incr) * J_spa_VKC_incr ;
      J_u_l_c4_incr = Adjoint( T_l_c4_aw_0) * Adjoint( T_b_aw_incr) * J_spa_VKC_incr ;

      J_u_r_c1_incr = Adjoint( T_r_c1_aw_0) * Adjoint( T_b_aw_incr) * J_spa_VKC_incr ;
      J_u_r_c2_incr = Adjoint( T_r_c2_aw_0) * Adjoint( T_b_aw_incr) * J_spa_VKC_incr ;
      J_u_r_c3_incr = Adjoint( T_r_c3_aw_0) * Adjoint( T_b_aw_incr) * J_spa_VKC_incr ;
      J_u_r_c4_incr = Adjoint( T_r_c4_aw_0) * Adjoint( T_b_aw_incr) * J_spa_VKC_incr ;

     J_uq_l_c1_incr.setSubmatrix(J_u_l_c1_incr , 0, 0) ; 
     J_uq_l_c2_incr.setSubmatrix(J_u_l_c2_incr , 0, 0) ;
     J_uq_l_c3_incr.setSubmatrix(J_u_l_c3_incr , 0, 0) ;
     J_uq_l_c4_incr.setSubmatrix(J_u_l_c4_incr , 0, 0) ;

 /*    std::cout << " J_u_l_c1_incr = " <<  std::endl << J_u_l_c1_incr.toString() <<  std::endl   ; 	

               yarp::sig::Matrix Jac_temp_incr =    (J_u_l_c1_incr - Jac_u_l_c1_u1)/h ;
     std::cout << " Jac_temp_incr = " <<  std::endl << Jac_temp_incr.toString() <<std::endl   ; 	

         yarp::sig::Matrix Adj_temp =   (Adjoint( T_l_c1_aw_0)-Adjoint( T_l_c1_b_0 ) )/h ;
     std::cout << " Adj_temp = " <<  std::endl << Adj_temp.toString() <<std::endl   ; 	
             yarp::sig::Matrix Adj_temp_2 =  Adjoint( T_b_aw_u1 ) - Adjoint( T_b_aw_incr) ;
     std::cout << " Adj_temp_2 = " <<  std::endl << Adj_temp_2.toString() <<std::endl   ; 	

     yarp::sig::Matrix Jac_temp_1 =    (Jac_b_aw_b_u1 -  Adjoint( T_b_aw_incr) * J_spa_VKC_incr)/h;
     std::cout << " Jac_temp_1 = " <<  std::endl << Jac_temp_1.toString() <<std::endl   ;    

        yarp::sig::Matrix Jac_temp_2 =    (J_spa_VKC_incr -  Jac_aw_b_u1  )/h;
     std::cout << " Jac_temp_2 = " <<  std::endl << Jac_temp_2.toString() <<std::endl   ; 	

     yarp::sig::Matrix Jac_temp =    (Jac_u_l_c1_u1 - J_u_l_c1_incr)/h ;
     std::cout << " Jac_temp = " <<  std::endl << Jac_temp.toString() <<std::endl   ; 	
     
     yarp::sig::Vector J_u_temp_1 =   J_u_l_c1_incr.transposed()*B_select.transposed()*fc_l_c1 -
     J_u_l_c1_init.transposed()*B_select.transposed()*fc_l_c1 ;

     yarp::sig::Vector J_u_temp_2 = (J_u_temp - J_u_temp_1)/h ;
          std::cout << " J_u_temp_2 = " <<  std::endl << J_u_temp_2.toString() <<std::endl   ; 	  */

     J_uq_r_c1_incr.setSubmatrix(J_u_r_c1_incr , 0, 0) ;
     J_uq_r_c2_incr.setSubmatrix(J_u_r_c2_incr , 0, 0) ;
     J_uq_r_c3_incr.setSubmatrix(J_u_r_c3_incr , 0, 0) ;
     J_uq_r_c4_incr.setSubmatrix(J_u_r_c4_incr , 0, 0) ;
     
     J_uq_l_c1_incr.setSubmatrix(J_q_l_c1_init , 0, 6) ;
     J_uq_l_c2_incr.setSubmatrix(J_q_l_c2_init , 0, 6) ;
     J_uq_l_c3_incr.setSubmatrix(J_q_l_c3_init , 0, 6) ;
     J_uq_l_c4_incr.setSubmatrix(J_q_l_c4_init , 0, 6) ;

     J_uq_r_c1_incr.setSubmatrix(J_q_r_c1_init , 0, 6) ;
     J_uq_r_c2_incr.setSubmatrix(J_q_r_c2_init , 0, 6) ;
     J_uq_r_c3_incr.setSubmatrix(J_q_r_c3_init , 0, 6) ;
     J_uq_r_c4_incr.setSubmatrix(J_q_r_c4_init , 0, 6) ;  

     U_l_c1_incr_col_i  =  J_uq_l_c1_incr.transposed()*B_select.transposed()*fc_l_c1 ;
     
     U_l_c2_incr_col_i  =  J_uq_l_c2_incr.transposed()*B_select.transposed()*fc_l_c2 ;
     U_l_c3_incr_col_i  =  J_uq_l_c3_incr.transposed()*B_select.transposed()*fc_l_c3 ;
     U_l_c4_incr_col_i  =  J_uq_l_c4_incr.transposed()*B_select.transposed()*fc_l_c4 ;

     U_r_c1_incr_col_i  =  J_uq_r_c1_incr.transposed()*B_select.transposed()*fc_r_c1 ;
     U_r_c2_incr_col_i  =  J_uq_r_c2_incr.transposed()*B_select.transposed()*fc_r_c1 ;
     U_r_c3_incr_col_i  =  J_uq_r_c3_incr.transposed()*B_select.transposed()*fc_r_c1 ;
     U_r_c4_incr_col_i  =  J_uq_r_c4_incr.transposed()*B_select.transposed()*fc_r_c1 ;
     
     U_l_c1_incr_init_col_i = (U_l_c1_incr_col_i - U_l_c1_init_col_i)/h ;
 //               std::cout << "U_l_c1_init_col_i =  " << std::endl << U_l_c1_init_col_i.toString() << std::endl;  
//                std::cout << "U_l_c1_incr_col_i =  " << std::endl << U_l_c1_incr_col_i.toString() << std::endl;  
   
     U_l_c2_incr_init_col_i = (U_l_c2_incr_col_i - U_l_c2_init_col_i)/h ;
     U_l_c3_incr_init_col_i = (U_l_c3_incr_col_i - U_l_c3_init_col_i)/h ;
     U_l_c4_incr_init_col_i = (U_l_c4_incr_col_i - U_l_c4_init_col_i)/h ;

     U_r_c1_incr_init_col_i = (U_r_c1_incr_col_i - U_r_c1_init_col_i)/h ;
     U_r_c2_incr_init_col_i = (U_r_c2_incr_col_i - U_r_c2_init_col_i)/h ;
     U_r_c3_incr_init_col_i = (U_r_c3_incr_col_i - U_r_c3_init_col_i)/h ;
     U_r_c4_incr_init_col_i = (U_r_c4_incr_col_i - U_r_c4_init_col_i)/h ;
  //        std::cout << " qui 3" <<  std::endl   ; 	
 
     U_l_c1_incr_init.setCol(i, U_l_c1_incr_init_col_i) ;
 //          std::cout << "U_l_c1_incr_init =  " << std::endl << U_l_c1_incr_init.toString() << std::endl;  
     U_l_c2_incr_init.setCol(i, U_l_c2_incr_init_col_i) ;
     U_l_c3_incr_init.setCol(i, U_l_c3_incr_init_col_i) ;
     U_l_c4_incr_init.setCol(i, U_l_c4_incr_init_col_i) ;

     U_r_c1_incr_init.setCol(i, U_r_c1_incr_init_col_i) ;
     U_r_c2_incr_init.setCol(i, U_r_c2_incr_init_col_i) ;
     U_r_c3_incr_init.setCol(i, U_r_c3_incr_init_col_i) ;
     U_r_c4_incr_init.setCol(i, U_r_c4_incr_init_col_i) ;
     
     //------------------------------------------------------------------------
    // J_u_l_c1_incr
 //     Jac_uq_l_c1_u1.setSubmatrix( J_u_l_c1_incr  , 0 ,  0 )  ;
   /*   Jac_uq_l_c2_u1.setSubmatrix( Jac_u_l_c2_u1 , 0 ,  0 )  ;
      Jac_uq_l_c3_u1.setSubmatrix( Jac_u_l_c3_u1 , 0 ,  0 )  ;
      Jac_uq_l_c4_u1.setSubmatrix( Jac_u_l_c4_u1 , 0 ,  0 )  ;

      Jac_uq_r_c1_u1.setSubmatrix( Jac_u_r_c1_u1 , 0 ,  0 )  ; 
      Jac_uq_r_c2_u1.setSubmatrix( Jac_u_r_c2_u1 , 0 ,  0 )  ;
      Jac_uq_r_c3_u1.setSubmatrix( Jac_u_r_c3_u1 , 0 ,  0 )  ;
      Jac_uq_r_c4_u1.setSubmatrix( Jac_u_r_c4_u1 , 0 ,  0 )  ;  */

  //    Jac_uq_l_c1_u1.setSubmatrix( J_q_l_c1_init , 0 ,  6 )  ;
  /*    Jac_uq_l_c2_u1.setSubmatrix( Jac_q_l_c2_u1 , 0 ,  6 )  ;
      Jac_uq_l_c3_u1.setSubmatrix( Jac_q_l_c3_u1 , 0 ,  6 )  ;
      Jac_uq_l_c4_u1.setSubmatrix( Jac_q_l_c4_u1 , 0 ,  6 )  ;

      Jac_uq_r_c1_u1.setSubmatrix( Jac_q_r_c1_u1 , 0 ,  6 )  ;
      Jac_uq_r_c2_u1.setSubmatrix( Jac_q_r_c2_u1 , 0 ,  6 )  ;
      Jac_uq_r_c3_u1.setSubmatrix( Jac_q_r_c3_u1 , 0 ,  6 )  ;
      Jac_uq_r_c4_u1.setSubmatrix( Jac_q_r_c4_u1 , 0 ,  6 )  ;  */
  
 /*    Jac_uq_l_c1_0.setSubmatrix( J_u_l_c1_init  , 0 ,  0 )  ;
     Jac_uq_l_c1_0.setSubmatrix( J_q_l_c1_init , 0 ,  6 )  ;  */
 
//      std::cout << "Jac_uq_l_c1_u1.cols() " <<  std::endl << Jac_uq_l_c1_u1.cols() <<  std::endl  ; 	
//     std::cout << "Jac_uq_l_c1_0.cols() " <<  std::endl << Jac_uq_l_c1_0.cols() <<  std::endl  ; 	
      
   /*    d_Jac_uq_l_c1_u1 = ( Jac_uq_l_c1_u1 - Jac_uq_l_c1_0 )/h ;
     d_Jac_uq_l_c2_u1 = ( Jac_uq_l_c2_u1 - Jac_uq_l_c2_0 )/h ;
      d_Jac_uq_l_c3_u1 = ( Jac_uq_l_c3_u1 - Jac_uq_l_c3_0 )/h ;
      d_Jac_uq_l_c4_u1 = ( Jac_uq_l_c4_u1 - Jac_uq_l_c4_0 )/h ;

      d_Jac_uq_r_c1_u1 = ( Jac_uq_r_c1_u1 - Jac_uq_r_c1_0 )/h ;
      d_Jac_uq_r_c2_u1 = ( Jac_uq_r_c2_u1 - Jac_uq_r_c2_0 )/h ;
      d_Jac_uq_r_c3_u1 = ( Jac_uq_r_c3_u1 - Jac_uq_r_c3_0 )/h ;
      d_Jac_uq_r_c4_u1 = ( Jac_uq_r_c4_u1 - Jac_uq_r_c4_0 )/h ;  */
   /*         std::cout << "Jac_uq_l_c1_u1 =  "<<  std::endl  << std::endl << Jac_uq_l_c1_u1.toString() << std::endl;  
            std::cout << "Jac_uq_l_c1_0 =  "<<  std::endl  << std::endl << Jac_uq_l_c1_0.toString() << std::endl;  

      std::cout << "d_Jac_uq_l_c1_u1.rows =  "<<  std::endl  << std::endl << d_Jac_uq_l_c1_u1.rows() << std::endl;  
      std::cout << "d_Jac_uq_l_c1_u1.cols =  "<<  std::endl  << std::endl << d_Jac_uq_l_c1_u1.cols() << std::endl;  */

	    
	    
 /*     U_l_c1_col_i_temp= d_Jac_uq_l_c1_u1.transposed()*B_select.transposed()*fc_l_c1  ;

yarp::sig::Vector    Col_i_temp= d_Jac_uq_l_c1_u1.transposed()*B_select.transposed()*fc_l_c1  ;

  std::cout << " Col_i_temp =  "<<  std::endl  << std::endl << Col_i_temp.toString() << std::endl;  */

      /*   U_l_c2_col_i= d_Jac_uq_l_c2_u1.transposed()*B_select.transposed()*fc_l_c2  ;
      U_l_c3_col_i= d_Jac_uq_l_c3_u1.transposed()*B_select.transposed()*fc_l_c3  ;
      U_l_c4_col_i= d_Jac_uq_l_c4_u1.transposed()*B_select.transposed()*fc_l_c4  ;

      U_r_c1_col_i= d_Jac_uq_r_c1_u1.transposed()*B_select.transposed()*fc_r_c1  ;
      U_r_c2_col_i= d_Jac_uq_r_c2_u1.transposed()*B_select.transposed()*fc_r_c2  ;
      U_r_c3_col_i= d_Jac_uq_r_c3_u1.transposed()*B_select.transposed()*fc_r_c3  ;
      U_r_c4_col_i= d_Jac_uq_r_c4_u1.transposed()*B_select.transposed()*fc_r_c4  ;  */
       //     std::cout << "Jac_uq_l_c1_u1 =  " << std::endl << Jac_uq_l_c1_u1.toString() << std::endl;  
    
    
  /*        std::cout << "U_l_c1_temp.rows =  "<<  std::endl  << std::endl << U_l_c1_temp.rows() << std::endl;  
      std::cout << "U_l_c1_temp.cols =  "<<  std::endl  << std::endl << U_l_c1_temp.cols() << std::endl;  

      std::cout << "U_l_c1_col_i_temp.length() =  "<<  std::endl  << std::endl << U_l_c1_col_i_temp.length() << std::endl;  
      U_l_c1_temp.setCol(i, U_l_c1_col_i_temp) ; 
            std::cout << "U_l_c1_temp =  " << std::endl << U_l_c1_temp.toString() << std::endl;  */

      
   /*   U_l_c2.setCol(i, U_l_c2_col_i) ; 
      U_l_c3.setCol(i, U_l_c3_col_i) ; 
      U_l_c4.setCol(i, U_l_c4_col_i) ; 

      U_r_c1.setCol(i, U_r_c1_col_i) ; 
      U_r_c2.setCol(i, U_r_c2_col_i) ; 
      U_r_c3.setCol(i, U_r_c3_col_i) ; 
      U_r_c4.setCol(i, U_r_c4_col_i) ; */

     } ;
     
 //          std::cout << " U_l_c1  = " <<  std::endl <<  U_l_c1.toString() << std::endl   ; 	
//     std::cout << " U_l_c1_incr_init = " <<  std::endl <<  U_l_c1_incr_init.toString() << std::endl   ; 	


  /*  yarp::sig::Matrix Ul_temp = (U_l_c1_temp - U_l_c1_incr_init ) ;
           std::cout << " Ul_temp = " <<  std::endl <<  Ul_temp.toString() << std::endl   ; 	
    yarp::sig::Matrix Ul_temp_1 = (U_l_c1_temp - U_l_c1_incr_init )/h ;
           std::cout << " Ul_temp_1 = " <<  std::endl <<  Ul_temp_1.toString() << std::endl   ; 	*/
	   
    //  yarp::sig::Matrix Ul_temp_2 = (U_l_c1 - U_l_c1_incr_init ) ;
   //        std::cout << " Ul_temp_2 = " <<  std::endl <<  Ul_temp_2.toString() << std::endl   ; 	
 //   yarp::sig::Matrix Ul_temp_3 = (U_l_c1 - U_l_c1_incr_init )/h ;
 //          std::cout << " Ul_temp_3 = " <<  std::endl <<  Ul_temp_3.toString() << std::endl   ; 	
	   
  //   std::cout << " qui 4" <<  std::endl   ; 	

     yarp::sig::Matrix U_l_tot_2 =  U_l_c1_incr_init + U_l_c2_incr_init + U_l_c3_incr_init + U_l_c4_incr_init;
     yarp::sig::Matrix U_r_tot_2 =  U_r_c1_incr_init + U_r_c2_incr_init + U_r_c3_incr_init + U_r_c4_incr_init;
     
     yarp::sig::Matrix U_tot_2 =  U_l_tot_2 + U_r_tot_2 ;
     //      std::cout << "U_tot_2 =  " << std::endl << U_tot_2.toString() << std::endl;    

     
     
    /*      std::cout << "U_l_c1_incr_init =  " << std::endl << U_l_c1_incr_init.toString() << std::endl;    
    std::cout << "U_l_c2_incr_init =  " << std::endl << U_l_c2_incr_init.toString() << std::endl;    
    std::cout << "U_l_c3_incr_init =  " << std::endl << U_l_c3_incr_init.toString() << std::endl;    
    std::cout << "U_l_c4_incr_init =  " << std::endl << U_l_c4_incr_init.toString() << std::endl;   

    std::cout << "U_r_c1_incr_init =  " << std::endl << U_r_c1_incr_init.toString() << std::endl;    
    std::cout << "U_r_c2_incr_init =  " << std::endl << U_r_c2_incr_init.toString() << std::endl;    
    std::cout << "U_r_c3_incr_init =  " << std::endl << U_r_c3_incr_init.toString() << std::endl;    
    std::cout << "U_r_c4_incr_init =  " << std::endl << U_r_c4_incr_init.toString() << std::endl;  
    
    
   //   std::cout << "U_r_tot_2 =  " << std::endl << U_r_tot_2.toString() << std::endl;     */

      
//---------------------------------------------------------------------------------------------------      

 
      yarp::sig::Vector q_incr( robot.getNumberOfJoints(), 0.0 );


      yarp::sig::Matrix T_imu_w_q1(4,4) ;   
      yarp::sig::Matrix T_w_imu_q1(4,4) ;
      
      yarp::sig::Matrix T_w_l_c1_q1(4, 4 ) ;
      yarp::sig::Matrix T_w_l_c2_q1(4, 4 ) ;
      yarp::sig::Matrix T_w_l_c3_q1(4, 4 ) ;
      yarp::sig::Matrix T_w_l_c4_q1(4, 4 ) ;

      yarp::sig::Matrix T_w_r_c1_q1(4, 4 ) ;
      yarp::sig::Matrix T_w_r_c2_q1(4, 4 ) ;
      yarp::sig::Matrix T_w_r_c3_q1(4, 4 ) ;
      yarp::sig::Matrix T_w_r_c4_q1(4, 4 ) ;  
      
      yarp::sig::Matrix T_l_c1_w_q1(4,4) ;
      yarp::sig::Matrix T_l_c2_w_q1(4,4) ;
      yarp::sig::Matrix T_l_c3_w_q1(4,4) ;
      yarp::sig::Matrix T_l_c4_w_q1(4,4) ;

      yarp::sig::Matrix T_r_c1_w_q1(4,4) ;
      yarp::sig::Matrix T_r_c2_w_q1(4,4) ;
      yarp::sig::Matrix T_r_c3_w_q1(4,4) ;
      yarp::sig::Matrix T_r_c4_w_q1(4,4) ;

      yarp::sig::Matrix T_b_l_c1_q1(4,4) ;
      yarp::sig::Matrix T_b_l_c2_q1(4,4) ;
      yarp::sig::Matrix T_b_l_c3_q1(4,4) ;
      yarp::sig::Matrix T_b_l_c4_q1(4,4) ;

      yarp::sig::Matrix T_b_r_c1_q1(4,4) ;
      yarp::sig::Matrix T_b_r_c2_q1(4,4) ;
      yarp::sig::Matrix T_b_r_c3_q1(4,4) ;
      yarp::sig::Matrix T_b_r_c4_q1(4,4) ;

      yarp::sig::Matrix T_l_c1_b_q1(4,4 ) ;
      yarp::sig::Matrix T_l_c2_b_q1(4,4 ) ;
      yarp::sig::Matrix T_l_c3_b_q1(4,4 ) ;
      yarp::sig::Matrix T_l_c4_b_q1(4,4 ) ;

      yarp::sig::Matrix T_r_c1_b_q1(4,4 ) ;
      yarp::sig::Matrix T_r_c2_b_q1(4,4 ) ;
      yarp::sig::Matrix T_r_c3_b_q1(4,4 ) ;
      yarp::sig::Matrix T_r_c4_b_q1(4,4 ) ;

      yarp::sig::Matrix Jac_u_l_c1_q1(6,6) ;
      yarp::sig::Matrix Jac_u_l_c2_q1(6,6) ;
      yarp::sig::Matrix Jac_u_l_c3_q1(6,6) ;
      yarp::sig::Matrix Jac_u_l_c4_q1(6,6) ;

      yarp::sig::Matrix Jac_u_r_c1_q1(6,6) ;
      yarp::sig::Matrix Jac_u_r_c2_q1(6,6) ;
      yarp::sig::Matrix Jac_u_r_c3_q1(6,6) ;
      yarp::sig::Matrix Jac_u_r_c4_q1(6,6) ;

      yarp::sig::Matrix Jac_q_pl_c1_q1_long(6 , robot.getNumberOfJoints()+6 )  ; // pole in l_c1, long version
      yarp::sig::Matrix Jac_q_pl_c2_q1_long(6 , robot.getNumberOfJoints()+6 )  ;
      yarp::sig::Matrix Jac_q_pl_c3_q1_long(6 , robot.getNumberOfJoints()+6 )  ;
      yarp::sig::Matrix Jac_q_pl_c4_q1_long(6 , robot.getNumberOfJoints()+6 )  ;

      yarp::sig::Matrix Jac_q_pr_c1_q1_long(6 , robot.getNumberOfJoints()+6 )  ;
      yarp::sig::Matrix Jac_q_pr_c2_q1_long(6 , robot.getNumberOfJoints()+6 )  ;
      yarp::sig::Matrix Jac_q_pr_c3_q1_long(6 , robot.getNumberOfJoints()+6 )  ;
      yarp::sig::Matrix Jac_q_pr_c4_q1_long(6 , robot.getNumberOfJoints()+6 )  ;

      yarp::sig::Matrix Jac_q_pl_c1_q1( 6, robot.getNumberOfJoints() )  ;   // pole in l_c1, short version
      yarp::sig::Matrix Jac_q_pl_c2_q1( 6, robot.getNumberOfJoints() )  ;
      yarp::sig::Matrix Jac_q_pl_c3_q1( 6, robot.getNumberOfJoints() )  ;
      yarp::sig::Matrix Jac_q_pl_c4_q1( 6, robot.getNumberOfJoints() )  ;

      yarp::sig::Matrix Jac_q_pr_c1_q1( 6, robot.getNumberOfJoints() )  ;
      yarp::sig::Matrix Jac_q_pr_c2_q1( 6, robot.getNumberOfJoints() )  ;
      yarp::sig::Matrix Jac_q_pr_c3_q1( 6, robot.getNumberOfJoints() )  ;
      yarp::sig::Matrix Jac_q_pr_c4_q1( 6, robot.getNumberOfJoints() )  ;

      yarp::sig::Matrix Jac_q_l_c1_q1( 6, robot.getNumberOfJoints() )   ;  // body, in l_c1, short version
      yarp::sig::Matrix Jac_q_l_c2_q1( 6, robot.getNumberOfJoints() )   ;
      yarp::sig::Matrix Jac_q_l_c3_q1( 6, robot.getNumberOfJoints() )   ;
      yarp::sig::Matrix Jac_q_l_c4_q1( 6, robot.getNumberOfJoints() )   ;

      yarp::sig::Matrix Jac_q_r_c1_q1( 6, robot.getNumberOfJoints() )   ;
      yarp::sig::Matrix Jac_q_r_c2_q1( 6, robot.getNumberOfJoints() )   ;
      yarp::sig::Matrix Jac_q_r_c3_q1( 6, robot.getNumberOfJoints() )   ;
      yarp::sig::Matrix Jac_q_r_c4_q1( 6, robot.getNumberOfJoints() )   ;

      yarp::sig::Matrix Jac_uq_l_c1_q1( 6, robot.getNumberOfJoints() +6 ) ; 	    
      yarp::sig::Matrix Jac_uq_l_c2_q1( 6, robot.getNumberOfJoints() +6 ) ; 	    
      yarp::sig::Matrix Jac_uq_l_c3_q1( 6, robot.getNumberOfJoints() +6 ) ; 	    
      yarp::sig::Matrix Jac_uq_l_c4_q1( 6, robot.getNumberOfJoints() +6 ) ; 	    

      yarp::sig::Matrix Jac_uq_r_c1_q1( 6, robot.getNumberOfJoints() +6 ) ; 	    
      yarp::sig::Matrix Jac_uq_r_c2_q1( 6, robot.getNumberOfJoints() +6 ) ; 	    
      yarp::sig::Matrix Jac_uq_r_c3_q1( 6, robot.getNumberOfJoints() +6 ) ; 	    
      yarp::sig::Matrix Jac_uq_r_c4_q1( 6, robot.getNumberOfJoints() +6 ) ; 	    

      yarp::sig::Matrix d_Jac_uq_l_c1_q1( 6, robot.getNumberOfJoints() +6 )  ;
      yarp::sig::Matrix d_Jac_uq_l_c2_q1( 6, robot.getNumberOfJoints() +6 )  ;
      yarp::sig::Matrix d_Jac_uq_l_c3_q1( 6, robot.getNumberOfJoints() +6 )  ;
      yarp::sig::Matrix d_Jac_uq_l_c4_q1( 6, robot.getNumberOfJoints() +6 )  ;

      yarp::sig::Matrix d_Jac_uq_r_c1_q1( 6, robot.getNumberOfJoints() +6 )  ;
      yarp::sig::Matrix d_Jac_uq_r_c2_q1( 6, robot.getNumberOfJoints() +6 )  ;
      yarp::sig::Matrix d_Jac_uq_r_c3_q1( 6, robot.getNumberOfJoints() +6 )  ;
      yarp::sig::Matrix d_Jac_uq_r_c4_q1( 6, robot.getNumberOfJoints() +6 )  ;

      yarp::sig::Vector Q_l_c1_col_i( robot.getNumberOfJoints() +6 )   ;
      yarp::sig::Vector Q_l_c2_col_i( robot.getNumberOfJoints() +6 )   ;
      yarp::sig::Vector Q_l_c3_col_i( robot.getNumberOfJoints() +6 )   ;
      yarp::sig::Vector Q_l_c4_col_i( robot.getNumberOfJoints() +6 )   ;

      yarp::sig::Vector Q_r_c1_col_i( robot.getNumberOfJoints() +6 )   ;
      yarp::sig::Vector Q_r_c2_col_i( robot.getNumberOfJoints() +6 )   ;
      yarp::sig::Vector Q_r_c3_col_i( robot.getNumberOfJoints() +6 )   ;
      yarp::sig::Vector Q_r_c4_col_i( robot.getNumberOfJoints() +6 )   ;

      yarp::sig::Matrix Q_l_c1( robot.getNumberOfJoints() +6 , robot.getNumberOfJoints()  )  ;
      yarp::sig::Matrix Q_l_c2( robot.getNumberOfJoints() +6 , robot.getNumberOfJoints()  )  ;
      yarp::sig::Matrix Q_l_c3( robot.getNumberOfJoints() +6 , robot.getNumberOfJoints()  )  ;
      yarp::sig::Matrix Q_l_c4( robot.getNumberOfJoints() +6 , robot.getNumberOfJoints()  )  ;

      yarp::sig::Matrix Q_r_c1( robot.getNumberOfJoints() +6 , robot.getNumberOfJoints()  )  ;
      yarp::sig::Matrix Q_r_c2( robot.getNumberOfJoints() +6 , robot.getNumberOfJoints()  )  ;
      yarp::sig::Matrix Q_r_c3( robot.getNumberOfJoints() +6 , robot.getNumberOfJoints()  )  ;
      yarp::sig::Matrix Q_r_c4( robot.getNumberOfJoints() +6 , robot.getNumberOfJoints()  )  ;

     for ( int i = 0  ; i<robot.getNumberOfJoints() ; i++ ) 
     {
      q_incr = q_current ;
      q_incr[i] += h ;
      robot.idynutils.updateiDyn3Model( q_incr, true );   //update model first  

      T_w_imu_q1  = model.iDyn3_model.getPosition( imu_link_index  ) ;    
      T_imu_w_q1 = iHomogeneous(T_w_imu_q1) ;
      
      T_w_l_c1_q1 = model.iDyn3_model.getPosition( l_c1_index  )  ;
      T_w_l_c2_q1 = model.iDyn3_model.getPosition( l_c2_index  )  ;
      T_w_l_c3_q1 = model.iDyn3_model.getPosition( l_c3_index  )  ;
      T_w_l_c4_q1 = model.iDyn3_model.getPosition( l_c4_index  )  ;

      T_w_r_c1_q1 = model.iDyn3_model.getPosition( r_c1_index  )  ;
      T_w_r_c2_q1 = model.iDyn3_model.getPosition( r_c2_index  )  ;
      T_w_r_c3_q1 = model.iDyn3_model.getPosition( r_c3_index  )  ;
      T_w_r_c4_q1 = model.iDyn3_model.getPosition( r_c4_index  )  ;
      
      T_l_c1_w_q1 = iHomogeneous(T_w_l_c1_q1 );
      T_l_c2_w_q1 = iHomogeneous(T_w_l_c2_q1 );
      T_l_c3_w_q1 = iHomogeneous(T_w_l_c3_q1 );
      T_l_c4_w_q1 = iHomogeneous(T_w_l_c4_q1 );

      T_r_c1_w_q1 = iHomogeneous(T_w_r_c1_q1 );
      T_r_c2_w_q1 = iHomogeneous(T_w_r_c2_q1 );
      T_r_c3_w_q1 = iHomogeneous(T_w_r_c3_q1 );
      T_r_c4_w_q1 = iHomogeneous(T_w_r_c4_q1 );
      
      T_b_l_c1_q1 = T_b_imu_0*T_imu_w_q1* T_w_l_c1_q1  ;
      T_b_l_c2_q1 = T_b_imu_0*T_imu_w_q1* T_w_l_c2_q1  ;
      T_b_l_c3_q1 = T_b_imu_0*T_imu_w_q1* T_w_l_c3_q1  ;
      T_b_l_c4_q1 = T_b_imu_0*T_imu_w_q1* T_w_l_c4_q1  ;

      T_b_r_c1_q1 = T_b_imu_0*T_imu_w_q1* T_w_r_c1_q1  ;
      T_b_r_c2_q1 = T_b_imu_0*T_imu_w_q1* T_w_r_c2_q1  ;
      T_b_r_c3_q1 = T_b_imu_0*T_imu_w_q1* T_w_r_c3_q1  ;
      T_b_r_c4_q1 = T_b_imu_0*T_imu_w_q1* T_w_r_c4_q1  ;

      T_l_c1_b_q1 = iHomogeneous(T_b_l_c1_q1) ;
      T_l_c2_b_q1 = iHomogeneous(T_b_l_c2_q1) ;
      T_l_c3_b_q1 = iHomogeneous(T_b_l_c3_q1) ;
      T_l_c4_b_q1 = iHomogeneous(T_b_l_c4_q1) ;

      T_r_c1_b_q1 = iHomogeneous(T_b_r_c1_q1) ;
      T_r_c2_b_q1 = iHomogeneous(T_b_r_c2_q1) ;
      T_r_c3_b_q1 = iHomogeneous(T_b_r_c3_q1) ;
      T_r_c4_b_q1 = iHomogeneous(T_b_r_c4_q1) ;
      //----------------------------
      Jac_u_l_c1_q1 = Adjoint( T_l_c1_b_q1) *Eye_6  ;
      Jac_u_l_c2_q1 = Adjoint( T_l_c2_b_q1) *Eye_6  ;
      Jac_u_l_c3_q1 = Adjoint( T_l_c3_b_q1) *Eye_6  ;
      Jac_u_l_c4_q1 = Adjoint( T_l_c4_b_q1) *Eye_6  ;

      Jac_u_r_c1_q1 = Adjoint( T_r_c1_b_q1) *Eye_6  ;
      Jac_u_r_c2_q1 = Adjoint( T_r_c2_b_q1) *Eye_6  ;
      Jac_u_r_c3_q1 = Adjoint( T_r_c3_b_q1) *Eye_6  ;
      Jac_u_r_c4_q1 = Adjoint( T_r_c4_b_q1) *Eye_6  ;

      model.iDyn3_model.getJacobian( l_c1_index  , Jac_q_pl_c1_q1_long  , false  ) ; //false= mixed version jacobian //true= body jacobian
      model.iDyn3_model.getJacobian( l_c2_index  , Jac_q_pl_c2_q1_long  , false  ) ; //false= mixed version jacobian //true= body jacobian
      model.iDyn3_model.getJacobian( l_c3_index  , Jac_q_pl_c3_q1_long  , false  ) ; //false= mixed version jacobian //true= body jacobian
      model.iDyn3_model.getJacobian( l_c4_index  , Jac_q_pl_c4_q1_long  , false  ) ; //false= mixed version jacobian //true= body jacobian

      model.iDyn3_model.getJacobian( r_c1_index  , Jac_q_pr_c1_q1_long  , false  ) ; //false= mixed version jacobian //true= body jacobian
      model.iDyn3_model.getJacobian( r_c2_index  , Jac_q_pr_c2_q1_long  , false  ) ; //false= mixed version jacobian //true= body jacobian
      model.iDyn3_model.getJacobian( r_c3_index  , Jac_q_pr_c3_q1_long  , false  ) ; //false= mixed version jacobian //true= body jacobian
      model.iDyn3_model.getJacobian( r_c4_index  , Jac_q_pr_c4_q1_long  , false  ) ; //false= mixed version jacobian //true= body jacobian

      Jac_q_pl_c1_q1 =  Jac_q_pl_c1_q1_long.submatrix(0,5, 6, Jac_q_pl_c1_q1_long.cols()-1 ) ;
      Jac_q_pl_c2_q1 =  Jac_q_pl_c2_q1_long.submatrix(0,5, 6, Jac_q_pl_c2_q1_long.cols()-1 ) ;
      Jac_q_pl_c3_q1 =  Jac_q_pl_c3_q1_long.submatrix(0,5, 6, Jac_q_pl_c3_q1_long.cols()-1 ) ;
      Jac_q_pl_c4_q1 =  Jac_q_pl_c4_q1_long.submatrix(0,5, 6, Jac_q_pl_c4_q1_long.cols()-1 ) ;

      Jac_q_pr_c1_q1 =  Jac_q_pr_c1_q1_long.submatrix(0,5, 6, Jac_q_pr_c1_q1_long.cols()-1 ) ;
      Jac_q_pr_c2_q1 =  Jac_q_pr_c2_q1_long.submatrix(0,5, 6, Jac_q_pr_c2_q1_long.cols()-1 ) ;
      Jac_q_pr_c3_q1 =  Jac_q_pr_c3_q1_long.submatrix(0,5, 6, Jac_q_pr_c3_q1_long.cols()-1 ) ;
      Jac_q_pr_c4_q1 =  Jac_q_pr_c4_q1_long.submatrix(0,5, 6, Jac_q_pr_c4_q1_long.cols()-1 ) ;

      Jac_q_l_c1_q1 = Adjoint( Homogeneous( getRot(T_l_c1_w_q1  ), zero_3 ) )  *  Jac_q_pl_c1_q1 ;
      Jac_q_l_c2_q1 = Adjoint( Homogeneous( getRot(T_l_c2_w_q1  ), zero_3 ) )  *  Jac_q_pl_c2_q1 ;
      Jac_q_l_c3_q1 = Adjoint( Homogeneous( getRot(T_l_c3_w_q1  ), zero_3 ) )  *  Jac_q_pl_c3_q1 ;
      Jac_q_l_c4_q1 = Adjoint( Homogeneous( getRot(T_l_c4_w_q1  ), zero_3 ) )  *  Jac_q_pl_c4_q1 ;

      Jac_q_r_c1_q1 = Adjoint( Homogeneous( getRot(T_r_c1_w_q1  ), zero_3 ) )  *  Jac_q_pr_c1_q1 ;
      Jac_q_r_c2_q1 = Adjoint( Homogeneous( getRot(T_r_c2_w_q1  ), zero_3 ) )  *  Jac_q_pr_c2_q1 ;
      Jac_q_r_c3_q1 = Adjoint( Homogeneous( getRot(T_r_c3_w_q1  ), zero_3 ) )  *  Jac_q_pr_c3_q1 ;
      Jac_q_r_c4_q1 = Adjoint( Homogeneous( getRot(T_r_c4_w_q1  ), zero_3 ) )  *  Jac_q_pr_c4_q1 ;

      Jac_uq_l_c1_q1.setSubmatrix( Jac_u_l_c1_q1 , 0 ,  0 )  ;
      Jac_uq_l_c2_q1.setSubmatrix( Jac_u_l_c2_q1 , 0 ,  0 )  ;
      Jac_uq_l_c3_q1.setSubmatrix( Jac_u_l_c3_q1 , 0 ,  0 )  ;
      Jac_uq_l_c4_q1.setSubmatrix( Jac_u_l_c4_q1 , 0 ,  0 )  ;

      Jac_uq_r_c1_q1.setSubmatrix( Jac_u_r_c1_q1 , 0 ,  0 )  ;
      Jac_uq_r_c2_q1.setSubmatrix( Jac_u_r_c2_q1 , 0 ,  0 )  ;
      Jac_uq_r_c3_q1.setSubmatrix( Jac_u_r_c3_q1 , 0 ,  0 )  ;
      Jac_uq_r_c4_q1.setSubmatrix( Jac_u_r_c4_q1 , 0 ,  0 )  ;

      Jac_uq_l_c1_q1.setSubmatrix( Jac_q_l_c1_q1 , 0 ,  6 )  ;
      Jac_uq_l_c2_q1.setSubmatrix( Jac_q_l_c2_q1 , 0 ,  6 )  ;
      Jac_uq_l_c3_q1.setSubmatrix( Jac_q_l_c3_q1 , 0 ,  6 )  ;
      Jac_uq_l_c4_q1.setSubmatrix( Jac_q_l_c4_q1 , 0 ,  6 )  ;

      Jac_uq_r_c1_q1.setSubmatrix( Jac_q_r_c1_q1 , 0 ,  6 )  ;
      Jac_uq_r_c2_q1.setSubmatrix( Jac_q_r_c2_q1 , 0 ,  6 )  ;
      Jac_uq_r_c3_q1.setSubmatrix( Jac_q_r_c3_q1 , 0 ,  6 )  ;
      Jac_uq_r_c4_q1.setSubmatrix( Jac_q_r_c4_q1 , 0 ,  6 )  ;

      d_Jac_uq_l_c1_q1 = ( Jac_uq_l_c1_q1 - Jac_uq_l_c1_0 )/h ;
      d_Jac_uq_l_c2_q1 = ( Jac_uq_l_c2_q1 - Jac_uq_l_c2_0 )/h ;
      d_Jac_uq_l_c3_q1 = ( Jac_uq_l_c3_q1 - Jac_uq_l_c3_0 )/h ;
      d_Jac_uq_l_c4_q1 = ( Jac_uq_l_c4_q1 - Jac_uq_l_c4_0 )/h ;

      d_Jac_uq_r_c1_q1 = ( Jac_uq_r_c1_q1 - Jac_uq_r_c1_0 )/h ;
      d_Jac_uq_r_c2_q1 = ( Jac_uq_r_c2_q1 - Jac_uq_r_c2_0 )/h ;
      d_Jac_uq_r_c3_q1 = ( Jac_uq_r_c3_q1 - Jac_uq_r_c3_0 )/h ;
      d_Jac_uq_r_c4_q1 = ( Jac_uq_r_c4_q1 - Jac_uq_r_c4_0 )/h ;

      Q_l_c1_col_i= d_Jac_uq_l_c1_q1.transposed()*B_select.transposed()*fc_l_c1  ;
      Q_l_c2_col_i= d_Jac_uq_l_c2_q1.transposed()*B_select.transposed()*fc_l_c2  ;
      Q_l_c3_col_i= d_Jac_uq_l_c3_q1.transposed()*B_select.transposed()*fc_l_c3  ;
      Q_l_c4_col_i= d_Jac_uq_l_c4_q1.transposed()*B_select.transposed()*fc_l_c4  ;

      Q_r_c1_col_i= d_Jac_uq_r_c1_q1.transposed()*B_select.transposed()*fc_r_c1  ;
      Q_r_c2_col_i= d_Jac_uq_r_c2_q1.transposed()*B_select.transposed()*fc_r_c2  ;
      Q_r_c3_col_i= d_Jac_uq_r_c3_q1.transposed()*B_select.transposed()*fc_r_c3  ;
      Q_r_c4_col_i= d_Jac_uq_r_c4_q1.transposed()*B_select.transposed()*fc_r_c4  ;

      Q_l_c1.setCol(i, Q_l_c1_col_i) ; 
      Q_l_c2.setCol(i, Q_l_c2_col_i) ; 
      Q_l_c3.setCol(i, Q_l_c3_col_i) ; 
      Q_l_c4.setCol(i, Q_l_c4_col_i) ; 

      Q_r_c1.setCol(i, Q_r_c1_col_i) ; 
      Q_r_c2.setCol(i, Q_r_c2_col_i) ; 
      Q_r_c3.setCol(i, Q_r_c3_col_i) ; 
      Q_r_c4.setCol(i, Q_r_c4_col_i) ;      
     } ; 
      
 /*    yarp::sig::Matrix Q_l_c1_6 = Q_l_c1.submatrix(0,11 , 0,11 ) ;
     yarp::sig::Matrix Q_l_c2_6 = Q_l_c2.submatrix(0,11 , 0,11 ) ;
     yarp::sig::Matrix Q_l_c3_6 = Q_l_c3.submatrix(0,11 , 0,11 ) ;
     yarp::sig::Matrix Q_l_c4_6 = Q_l_c4.submatrix(0,11 , 0,11 ) ;
     
      std::cout << "Q_l_c1_6 =  " << std::endl << Q_l_c1_6.toString() << std::endl;  
      std::cout << "Q_l_c2_6 =  " << std::endl << Q_l_c2_6.toString() << std::endl;  
      std::cout << "Q_l_c3_6 =  " << std::endl << Q_l_c3_6.toString() << std::endl;  
      std::cout << "Q_l_c4_6 =  " << std::endl << Q_l_c4_6.toString() << std::endl;  

      std::cout << "Q_r_c1 =  " << std::endl << Q_r_c1.toString() << std::endl;  
      std::cout << "Q_r_c2 =  " << std::endl << Q_r_c2.toString() << std::endl;  
      std::cout << "Q_r_c3 =  " << std::endl << Q_r_c3.toString() << std::endl;  
      std::cout << "Q_r_c4 =  " << std::endl << Q_r_c4.toString() << std::endl;      */

      yarp::sig::Matrix Q_l_c_tot = Q_l_c1 + Q_l_c2 + Q_l_c3  + Q_l_c4 ;
   
      yarp::sig::Matrix Q_r_c_tot = Q_r_c1 + Q_r_c2 + Q_r_c3  + Q_r_c4 ;

   //   std::cout << "Q_l_c_tot =  " << std::endl << Q_l_c_tot.toString() << std::endl;   
   //   std::cout << "Q_r_c_tot =  " << std::endl << Q_r_c_tot.toString() << std::endl;   

      yarp::sig::Matrix Q_c_tot = Q_l_c_tot + Q_r_c_tot   ;
   //   std::cout << "Q_c_tot =  " << std::endl << Q_c_tot.toString() << std::endl;    
      
  //    yarp::sig::Matrix Q_c_tot_part = Q_c_tot.submatrix(0, robot.getNumberOfJoints()+6  , 0,11)  ;      
  //    std::cout << "Q_c_tot_part =  " << std::endl << Q_c_tot_part.toString() << std::endl;    

      
      //---------------------------------------------------------------------------------------
      // Computing the Derivative using the Lie adjoint matrix
      
      // data 

      robot.idynutils.updateiDyn3Model( q_current, true ); //update model first

      yarp::sig::Matrix J_l_c1_mix_0(6, size_q + 6) ;
      yarp::sig::Matrix J_l_c2_mix_0(6, size_q + 6) ;
      yarp::sig::Matrix J_l_c3_mix_0(6, size_q + 6) ;
      yarp::sig::Matrix J_l_c4_mix_0(6, size_q + 6) ;
      
      yarp::sig::Matrix J_r_c1_mix_0(6, size_q + 6) ;
      yarp::sig::Matrix J_r_c2_mix_0(6, size_q + 6) ;
      yarp::sig::Matrix J_r_c3_mix_0(6, size_q + 6) ;
      yarp::sig::Matrix J_r_c4_mix_0(6, size_q + 6) ;      

      model.iDyn3_model.getJacobian( l_c1_index  , J_l_c1_mix_0  , false  ) ; //false= mixed version jacobian //true= body jacobian
      model.iDyn3_model.getJacobian( l_c2_index  , J_l_c2_mix_0  , false  ) ; //false= mixed version jacobian //true= body jacobian
      model.iDyn3_model.getJacobian( l_c3_index  , J_l_c3_mix_0  , false  ) ; //false= mixed version jacobian //true= body jacobian
      model.iDyn3_model.getJacobian( l_c4_index  , J_l_c4_mix_0  , false  ) ; //false= mixed version jacobian //true= body jacobian

      model.iDyn3_model.getJacobian( r_c1_index  , J_r_c1_mix_0  , false  ) ; //false= mixed version jacobian //true= body jacobian
      model.iDyn3_model.getJacobian( r_c2_index  , J_r_c2_mix_0  , false  ) ; //false= mixed version jacobian //true= body jacobian
      model.iDyn3_model.getJacobian( r_c3_index  , J_r_c3_mix_0  , false  ) ; //false= mixed version jacobian //true= body jacobian
      model.iDyn3_model.getJacobian( r_c4_index  , J_r_c4_mix_0  , false  ) ; //false= mixed version jacobian //true= body jacobian
       
      yarp::sig::Matrix R_l_c1_w_0= getRot( T_l_c1_w_0 ) ;
      yarp::sig::Matrix R_l_c2_w_0= getRot( T_l_c2_w_0 ) ;
      yarp::sig::Matrix R_l_c3_w_0= getRot( T_l_c3_w_0 ) ;
      yarp::sig::Matrix R_l_c4_w_0= getRot( T_l_c4_w_0 ) ;

      yarp::sig::Matrix R_r_c1_w_0= getRot( T_r_c1_w_0 ) ;
      yarp::sig::Matrix R_r_c2_w_0= getRot( T_r_c2_w_0 ) ;
      yarp::sig::Matrix R_r_c3_w_0= getRot( T_r_c3_w_0 ) ;
      yarp::sig::Matrix R_r_c4_w_0= getRot( T_r_c4_w_0 ) ;      
      
      yarp::sig::Matrix J_l_c1_body_0 =  Adjoint(Homogeneous(R_l_c1_w_0, zero_3))*  J_l_c1_mix_0 ;
      yarp::sig::Matrix J_l_c2_body_0 =  Adjoint(Homogeneous(R_l_c2_w_0, zero_3))*  J_l_c2_mix_0 ;
      yarp::sig::Matrix J_l_c3_body_0 =  Adjoint(Homogeneous(R_l_c3_w_0, zero_3))*  J_l_c3_mix_0 ;
      yarp::sig::Matrix J_l_c4_body_0 =  Adjoint(Homogeneous(R_l_c4_w_0, zero_3))*  J_l_c4_mix_0 ;

      yarp::sig::Matrix J_r_c1_body_0 =  Adjoint(Homogeneous(R_r_c1_w_0, zero_3))*  J_r_c1_mix_0 ;
      yarp::sig::Matrix J_r_c2_body_0 =  Adjoint(Homogeneous(R_r_c2_w_0, zero_3))*  J_r_c2_mix_0 ;
      yarp::sig::Matrix J_r_c3_body_0 =  Adjoint(Homogeneous(R_r_c3_w_0, zero_3))*  J_r_c3_mix_0 ;
      yarp::sig::Matrix J_r_c4_body_0 =  Adjoint(Homogeneous(R_r_c4_w_0, zero_3))*  J_r_c4_mix_0 ;
      
      yarp::sig::Matrix J_l_c1_spa_0 =  Adjoint( T_aw_l_c1_0)*  J_l_c1_body_0 ;
      yarp::sig::Matrix J_l_c2_spa_0 =  Adjoint( T_aw_l_c2_0)*  J_l_c2_body_0 ;
      yarp::sig::Matrix J_l_c3_spa_0 =  Adjoint( T_aw_l_c3_0)*  J_l_c3_body_0 ;
      yarp::sig::Matrix J_l_c4_spa_0 =  Adjoint( T_aw_l_c4_0)*  J_l_c4_body_0 ;
      
      /*std::cout << "(J_l_c1_spa_0-J_l_c2_spa_0).toString() =  " << std::endl << (J_l_c1_spa_0-J_l_c2_spa_0).toString() << std::endl;     
      std::cout << "(J_l_c1_spa_0-J_l_c3_spa_0).toString() =  " << std::endl << (J_l_c1_spa_0-J_l_c3_spa_0).toString() << std::endl;     
      std::cout << "(J_l_c1_spa_0-J_l_c4_spa_0).toString() =  " << std::endl << (J_l_c1_spa_0-J_l_c4_spa_0).toString() << std::endl;   */  

      yarp::sig::Matrix J_r_c1_spa_0 =  Adjoint( T_aw_r_c1_0)*  J_r_c1_body_0 ;
      yarp::sig::Matrix J_r_c2_spa_0 =  Adjoint( T_aw_r_c2_0)*  J_r_c2_body_0 ;
      yarp::sig::Matrix J_r_c3_spa_0 =  Adjoint( T_aw_r_c3_0)*  J_r_c3_body_0 ;
      yarp::sig::Matrix J_r_c4_spa_0 =  Adjoint( T_aw_r_c4_0)*  J_r_c4_body_0 ;      
      
  /*     std::cout << "(J_r_c1_spa_0-J_r_c2_spa_0).toString() =  " << std::endl << (J_r_c1_spa_0-J_r_c2_spa_0).toString() << std::endl;      
      std::cout << "(J_r_c1_spa_0-J_r_c3_spa_0).toString() =  " << std::endl << (J_r_c1_spa_0-J_r_c3_spa_0).toString() << std::endl;     
      std::cout << "(J_r_c1_spa_0-J_r_c4_spa_0).toString() =  " << std::endl << (J_r_c1_spa_0-J_r_c4_spa_0).toString() << std::endl;   */

       /*   std::cout << "J_l_c1_spa_0.rows =  " << std::endl << J_l_c1_spa_0.rows() << std::endl;    
          std::cout << "J_l_c1_spa_0.cols =  " << std::endl << J_l_c1_spa_0.cols() << std::endl;    
          std::cout << "J_l_c1_spa_0 =  " << std::endl << J_l_c1_spa_0.toString() << std::endl;      */

     J_l_c1_spa_0.setSubmatrix( Eye_6 , 0,0) ;  // spatial jacobina with spatila floating base
     J_l_c2_spa_0.setSubmatrix( Eye_6 , 0,0) ;     //    std::cout << "J_l_c1_spa_0_eye =  " << std::endl << J_l_c1_spa_0.toString() << std::endl;    
     J_l_c3_spa_0.setSubmatrix( Eye_6 , 0,0) ;
     J_l_c4_spa_0.setSubmatrix( Eye_6 , 0,0) ;

     J_r_c1_spa_0.setSubmatrix( Eye_6 , 0,0) ;  // spatial jacobina with spatila floating base
     J_r_c2_spa_0.setSubmatrix( Eye_6 , 0,0) ;     //    std::cout << "J_l_c1_spa_0_eye =  " << std::endl << J_l_c1_spa_0.toString() << std::endl;    
     J_r_c3_spa_0.setSubmatrix( Eye_6 , 0,0) ;
     J_r_c4_spa_0.setSubmatrix( Eye_6 , 0,0) ;
     
     yarp::sig::Matrix Q_l_c1_lie(size_q +6, size_q +6 ) ;
     yarp::sig::Matrix Q_l_c2_lie(size_q +6, size_q +6 ) ;
     yarp::sig::Matrix Q_l_c3_lie(size_q +6, size_q +6 ) ;
     yarp::sig::Matrix Q_l_c4_lie(size_q +6, size_q +6 ) ;

     yarp::sig::Matrix Q_r_c1_lie(size_q +6, size_q +6 ) ;
     yarp::sig::Matrix Q_r_c2_lie(size_q +6, size_q +6 ) ;
     yarp::sig::Matrix Q_r_c3_lie(size_q +6, size_q +6 ) ;
     yarp::sig::Matrix Q_r_c4_lie(size_q +6, size_q +6 ) ;
     
     yarp::sig::Vector D_J_complete_l_c1_i(size_q +6 )  ;
     yarp::sig::Vector D_J_complete_l_c2_i(size_q +6 )  ;
     yarp::sig::Vector D_J_complete_l_c3_i(size_q +6 )  ;
     yarp::sig::Vector D_J_complete_l_c4_i(size_q +6 )  ;

     yarp::sig::Vector D_J_complete_r_c1_i(size_q +6 )  ;
     yarp::sig::Vector D_J_complete_r_c2_i(size_q +6 )  ;
     yarp::sig::Vector D_J_complete_r_c3_i(size_q +6 )  ;
     yarp::sig::Vector D_J_complete_r_c4_i(size_q +6 )  ;      
      
     for ( int i = 0  ; i<(size_q + 6) ; i++ ) 
     {     
       D_J_complete_l_c1_i = D_Jacob_spa_i(J_l_c1_spa_0,i+1).transposed() *
                                   Adjoint(T_l_c1_aw_0).transposed()*
                    B_select.transposed()*fc_l_c1 +
                                           J_l_c1_spa_0.transposed()*
                                 ( Adjoint(T_l_c1_aw_0)*
                              ad_lie(- 1.0*J_l_c1_spa_0.getCol(i)) ).transposed()*
                    B_select.transposed()*fc_l_c1 ;     
       Q_l_c1_lie.setCol(i,     D_J_complete_l_c1_i ) ;
       //----------------------------------------------------------------
       D_J_complete_l_c2_i = D_Jacob_spa_i(J_l_c2_spa_0,i+1).transposed() *
                                   Adjoint(T_l_c2_aw_0).transposed()*
                    B_select.transposed()*fc_l_c2 +
                                           J_l_c2_spa_0.transposed()*
                                 ( Adjoint(T_l_c2_aw_0)*
                              ad_lie(- 1.0*J_l_c2_spa_0.getCol(i)) ).transposed()*
                    B_select.transposed()*fc_l_c2 ;     
       Q_l_c2_lie.setCol(i,     D_J_complete_l_c2_i ) ;
       //----------------------------------------------------------------
       D_J_complete_l_c3_i = D_Jacob_spa_i(J_l_c3_spa_0,i+1).transposed() *
                                   Adjoint(T_l_c3_aw_0).transposed()*
                    B_select.transposed()*fc_l_c3 +
                                           J_l_c3_spa_0.transposed()*
                                 ( Adjoint(T_l_c3_aw_0)*
                              ad_lie(- 1.0*J_l_c3_spa_0.getCol(i)) ).transposed()*
                    B_select.transposed()*fc_l_c3 ;     
       Q_l_c3_lie.setCol(i,     D_J_complete_l_c3_i ) ;
       //----------------------------------------------------------------
       D_J_complete_l_c4_i = D_Jacob_spa_i(J_l_c4_spa_0,i+1).transposed() *
                                   Adjoint(T_l_c4_aw_0).transposed()*
                    B_select.transposed()*fc_l_c4 +
                                           J_l_c4_spa_0.transposed()*
                                 ( Adjoint(T_l_c4_aw_0)*
                              ad_lie(- 1.0*J_l_c4_spa_0.getCol(i)) ).transposed()*
                    B_select.transposed()*fc_l_c4 ;     
       Q_l_c4_lie.setCol(i,     D_J_complete_l_c4_i ) ;       
       //----------------------------------------------------------------
       D_J_complete_r_c1_i = D_Jacob_spa_i(J_r_c1_spa_0,i+1).transposed() *
                                   Adjoint(T_r_c1_aw_0).transposed()*
                    B_select.transposed()*fc_r_c1 +
                                           J_r_c1_spa_0.transposed()*
                                 ( Adjoint(T_r_c1_aw_0)*
                              ad_lie(- 1.0*J_r_c1_spa_0.getCol(i)) ).transposed()*
                    B_select.transposed()*fc_r_c1 ;     
       Q_r_c1_lie.setCol(i,     D_J_complete_r_c1_i ) ;    
       //----------------------------------------------------------------
       D_J_complete_r_c2_i = D_Jacob_spa_i(J_r_c2_spa_0,i+1).transposed() *
                                   Adjoint(T_r_c2_aw_0).transposed()*
                    B_select.transposed()*fc_r_c2 +
                                           J_r_c2_spa_0.transposed()*
                                 ( Adjoint(T_r_c2_aw_0)*
                              ad_lie(- 1.0*J_r_c2_spa_0.getCol(i)) ).transposed()*
                    B_select.transposed()*fc_r_c2 ;     
       Q_r_c2_lie.setCol(i,     D_J_complete_r_c2_i ) ;          
       //----------------------------------------------------------------
       D_J_complete_r_c3_i = D_Jacob_spa_i(J_r_c3_spa_0,i+1).transposed() *
                                   Adjoint(T_r_c3_aw_0).transposed()*
                    B_select.transposed()*fc_r_c3 +
                                           J_r_c3_spa_0.transposed()*
                                 ( Adjoint(T_r_c3_aw_0)*
                              ad_lie(- 1.0*J_r_c3_spa_0.getCol(i)) ).transposed()*
                    B_select.transposed()*fc_r_c3 ;     
       Q_r_c3_lie.setCol(i,     D_J_complete_r_c3_i ) ;     
       //----------------------------------------------------------------
       D_J_complete_r_c4_i = D_Jacob_spa_i(J_r_c4_spa_0,i+1).transposed() *
                                   Adjoint(T_r_c4_aw_0).transposed()*
                    B_select.transposed()*fc_r_c4 +
                                           J_r_c4_spa_0.transposed()*
                                 ( Adjoint(T_r_c4_aw_0)*
                              ad_lie(- 1.0*J_r_c4_spa_0.getCol(i)) ).transposed()*
                    B_select.transposed()*fc_r_c4 ;     
       Q_r_c4_lie.setCol(i,     D_J_complete_r_c4_i ) ;          
    } ;
      
     
 /*     yarp::sig::Matrix Q_l_c1_lie_6 = Q_l_c1_lie.submatrix(0,11 , 0,11 ) ;
      yarp::sig::Matrix Q_l_c2_lie_6 = Q_l_c2_lie.submatrix(0,11 , 0,11 ) ;
      yarp::sig::Matrix Q_l_c3_lie_6 = Q_l_c3_lie.submatrix(0,11 , 0,11 ) ;
      yarp::sig::Matrix Q_l_c4_lie_6 = Q_l_c4_lie.submatrix(0,11 , 0,11 ) ;

     
      yarp::sig::Matrix Q_r_c1_lie_6 = Q_r_c1_lie.submatrix(0,size_q+6  , 0,size_q+6 ) ;
      yarp::sig::Matrix Q_r_c2_lie_6 = Q_r_c2_lie.submatrix(0,size_q+6  , 0,size_q+6 ) ;
      yarp::sig::Matrix Q_r_c3_lie_6 = Q_r_c3_lie.submatrix(0,size_q+6  , 0,size_q+6 ) ;
      yarp::sig::Matrix Q_r_c4_lie_6 = Q_r_c4_lie.submatrix(0,size_q+6  , 0,size_q+6 ) ;      
      
      std::cout << "Q_l_c1_lie_6 =  " << std::endl << Q_l_c1_lie_6.toString() << std::endl;      
      std::cout << "Q_l_c2_lie_6 =  " << std::endl << Q_l_c2_lie_6.toString() << std::endl;  
      std::cout << "Q_l_c3_lie_6 =  " << std::endl << Q_l_c3_lie_6.toString() << std::endl;        
      std::cout << "Q_l_c4_lie_6 =  " << std::endl << Q_l_c4_lie_6.toString() << std::endl;        
      
      std::cout << "Q_r_c1_lie_6 =  " << std::endl << Q_r_c1_lie_6.toString() << std::endl;      
      std::cout << "Q_r_c2_lie_6 =  " << std::endl << Q_r_c2_lie_6.toString() << std::endl;  
      std::cout << "Q_r_c3_lie_6 =  " << std::endl << Q_r_c3_lie_6.toString() << std::endl;        
      std::cout << "Q_r_c4_lie_6 =  " << std::endl << Q_r_c4_lie_6.toString() << std::endl;       */ 
   //  std::cout << "Q_l_c1_lie =  " << std::endl << Q_l_c1_lie.toString() << std::endl;    

     
     
       yarp::sig::Matrix Q_l_c_lie = Q_l_c1_lie + Q_l_c2_lie + Q_l_c3_lie + Q_l_c4_lie ;
       yarp::sig::Matrix Q_r_c_lie = Q_r_c1_lie + Q_r_c2_lie + Q_r_c3_lie + Q_r_c4_lie ;
      
      
     yarp::sig::Matrix Q_l_c_lie_6 = Q_l_c_lie.submatrix(0, Q_r_c_lie.rows() , 0,11 ) ;
     yarp::sig::Matrix Q_r_c_lie_6 = Q_r_c_lie.submatrix(0,Q_r_c_lie.rows() , 12,18 ) ;

      
   /*   std::cout << "Q_l_c_lie_6 =  " << std::endl << Q_l_c_lie_6.toString() << std::endl;      
      std::cout << "Q_r_c_lie_6 =  " << std::endl << Q_r_c_lie_6.toString() << std::endl;     */ 
      
      
      
      yarp::sig::Matrix Q_tot_lie = Q_l_c_lie + Q_r_c_lie ;

      
				   
   /*  yarp::sig::Vector u_incr(6) ;
     u_incr = u_curr ;
     u_incr[0] += h ; */     

 //-----------------------------------------------------------------------------------------------------------------------
 /*    yarp::sig::Vector u_curr( 6, 0.0 )  ;
     yarp::sig::Vector u_incr(6) ;
     u_incr = u_curr ;
     u_incr[0] += h ;
 
 //------------------------------------------------------------------------------------------------------    
     // 'For' loop for computing  U_s 
     
     yarp::sig::Matrix U_s_l_c1(6 , 6) ;
     yarp::sig::Matrix U_s_l_c2(6 , 6) ;
     yarp::sig::Matrix U_s_l_c3(6 , 6) ;
     yarp::sig::Matrix U_s_l_c4(6 , 6) ;

     yarp::sig::Matrix U_s_r_c1(6 , 6) ;
     yarp::sig::Matrix U_s_r_c2(6 , 6) ;
     yarp::sig::Matrix U_s_r_c3(6 , 6) ;
     yarp::sig::Matrix U_s_r_c4(6 , 6) ;

     U_s_l_c1.zero();
     U_s_l_c2.zero();
     U_s_l_c3.zero();
     U_s_l_c4.zero();

     U_s_r_c1.zero();
     U_s_r_c2.zero();
     U_s_r_c3.zero();
     U_s_r_c4.zero();
     
     yarp::sig::Vector U_s_l_c1_i(6, 0.0) ;
     yarp::sig::Vector U_s_l_c2_i(6, 0.0) ;
     yarp::sig::Vector U_s_l_c3_i(6, 0.0) ;
     yarp::sig::Vector U_s_l_c4_i(6, 0.0) ;

     yarp::sig::Vector U_s_r_c1_i(6, 0.0) ;
     yarp::sig::Vector U_s_r_c2_i(6, 0.0) ;
     yarp::sig::Vector U_s_r_c3_i(6, 0.0) ;
     yarp::sig::Vector U_s_r_c4_i(6, 0.0) ;
     
     yarp::sig::Matrix d_adj_fix_aw_u_i(6,6) ;
     
     
     //--------------------------------------
     yarp::sig::Matrix Jac_COM_w_0( 6 , robot.getNumberOfJoints()  + 6 ) ; 
     model.iDyn3_model.getCOMJacobian( Jac_COM_w_0 ) ;  
     yarp::sig::Vector d_w_COM_0(3) ;
     yarp::sig::Vector zero_3(3, 0.0) ;

     robot.idynutils.updateiDyn3Model( q_current, true ); //update model first  
     
     d_w_COM_0 = model.iDyn3_model.getCOM()  ;               
     yarp::sig::Matrix T_w_COM_0 = Homogeneous(Eye_3, d_w_COM_0 ) ;
     yarp::sig::Matrix T_aw_COM_0 = T_aw_w *  T_w_COM_0 ;
     yarp::sig::Vector d_aw_COM_0 = getTrasl(T_aw_COM_0) ;
     yarp::sig::Vector g_dir_aw(3, 0.0) ; //direction of the gravity vector expressed in {AW}
     g_dir_aw(2) = -1 ;
     
     yarp::sig::Matrix  T_aw_w_0 = T_aw_w ;
           // robot.idynutils.updateiDyn3Model( q_current, true ); 
     yarp::sig::Matrix Jac_COM_aw_temp = Adjoint( Homogeneous( getRot( T_aw_w_0 ) , zero_3) )*Jac_COM_w_0 ;    
     yarp::sig::Matrix Jac_COM_aw_0 = Jac_COM_aw_temp.submatrix( 0, 2, 6, Jac_COM_aw_temp.cols()-1 ) ;      
     
     yarp::sig::Matrix T_fix_COM_incr(4,4) ; // = g_fix_aw_u_i *  T_aw_COM_0 ;
     yarp::sig::Vector d_fix_COM_incr(3)  ;  // = getTrasl(T_fix_COM_incr) ;
     yarp::sig::Vector d_com_i(3) ;
     
     yarp::sig::Matrix U_sg(6,6)  ;     
     yarp::sig::Matrix U_sg_1(3,6) ;
     U_sg_1.zero(); 
     yarp::sig::Matrix U_sg_2(3,6)  ;

     yarp::sig::Matrix T_fix_w_incr(4,4) ;
     yarp::sig::Matrix U_jg(robot.getNumberOfJoints() ,6)  ;     
  //   yarp::sig::Matrix U_jg_2(robot.getNumberOfJoints() ,6)  ;     
     yarp::sig::Vector temp_adj(robot.getNumberOfJoints() + 6) ;
     yarp::sig::Vector temp_adj_2(robot.getNumberOfJoints() ) ;

     yarp::sig::Vector U_jg_i(robot.getNumberOfJoints()) ;
     
     yarp::sig::Matrix Jac_COM_fix_temp( 6, Jac_COM_fix_temp.cols() ) ;
     yarp::sig::Matrix Jac_COM_fix_incr( 6, robot.getNumberOfJoints()) ;// = Jac_COM_aw_temp.submatrix( 0, 2, 6, Jac_COM_fix_temp.cols()-1 ) ;

     for ( int i = 0  ; i<6 ; i++ ) 
     {
      u_incr = u_curr ;
      u_incr[i] += h ;
      
      yarp::sig::Matrix g_fix_aw_u_i = twistexp(xi_1, u_incr[0]) * // this is the only point in which it is relevant the fact that
				       twistexp(xi_2, u_incr[1]) * // the frame {AW} is attached the world and can move a frame
				       twistexp(xi_3, u_incr[2]) * // {AW'} which, in the initial config., is coincident with {AW}
				       twistexp(xi_4, u_incr[3]) * // here {AW} is moving with respect to {fix}
				       twistexp(xi_5, u_incr[4]) *
				       twistexp(xi_6, u_incr[5]) ;
     d_adj_fix_aw_u_i = ( ( Adjoint_MT( g_fix_aw_u_i ) - Eye_6)/h ) ;
     U_s_l_c1_i = d_adj_fix_aw_u_i * Adjoint_MT( T_aw_l_c1_0 ) * B_select.transposed() * fc_l_c1 ;    
     U_s_l_c2_i = d_adj_fix_aw_u_i * Adjoint_MT( T_aw_l_c2_0 ) * B_select.transposed() * fc_l_c2 ;
     U_s_l_c3_i = d_adj_fix_aw_u_i * Adjoint_MT( T_aw_l_c3_0 ) * B_select.transposed() * fc_l_c3 ;
     U_s_l_c4_i = d_adj_fix_aw_u_i * Adjoint_MT( T_aw_l_c4_0 ) * B_select.transposed() * fc_l_c4 ;

     U_s_r_c1_i = d_adj_fix_aw_u_i * Adjoint_MT( T_aw_r_c1_0 ) * B_select.transposed() * fc_r_c1 ;
     U_s_r_c2_i = d_adj_fix_aw_u_i * Adjoint_MT( T_aw_r_c2_0 ) * B_select.transposed() * fc_r_c2 ;
     U_s_r_c3_i = d_adj_fix_aw_u_i * Adjoint_MT( T_aw_r_c3_0 ) * B_select.transposed() * fc_r_c3 ;
     U_s_r_c4_i = d_adj_fix_aw_u_i * Adjoint_MT( T_aw_r_c4_0 ) * B_select.transposed() * fc_r_c4 ;

     U_s_l_c1.setCol( i, U_s_l_c1_i )  ;      
     U_s_l_c2.setCol( i, U_s_l_c2_i )  ;
     U_s_l_c3.setCol( i, U_s_l_c3_i )  ;
     U_s_l_c4.setCol( i, U_s_l_c4_i )  ;

     U_s_r_c1.setCol( i, U_s_r_c1_i )  ;
     U_s_r_c2.setCol( i, U_s_r_c2_i )  ;
     U_s_r_c3.setCol( i, U_s_r_c3_i )  ;
     U_s_r_c4.setCol( i, U_s_r_c4_i )  ;
     
     //------------------------------------------------
     // COM Part
     T_fix_COM_incr = g_fix_aw_u_i *  T_aw_COM_0 ;
     d_fix_COM_incr = getTrasl(T_fix_COM_incr) ;
     d_com_i = (d_fix_COM_incr - d_aw_COM_0)/h ;
     U_sg_2.setCol( i, d_com_i )  ;
     //-----------------------------------------------
     T_fix_w_incr = g_fix_aw_u_i * T_aw_w_0 ;    

     temp_adj =  ( ( (Adjoint(Homogeneous( getRot( T_fix_w_incr), zero_3) ) - 
                                         Adjoint( Homogeneous( getRot(T_aw_w_0), zero_3) ))/h )  *Jac_COM_w_0).transposed() 
					 *B_select.transposed()  *mg*(-1.0*g_dir_aw)    ;
    
     temp_adj_2 = temp_adj.subVector( 6 ,  temp_adj.length()-1  ) ;
 //    std::cout << "temp_adj =  " << std::endl << temp_adj.length() << std::endl;  
 //    std::cout << "temp_adj_2 =  " << std::endl << temp_adj_2.toString() << std::endl;  
 //     std::cout << "temp_adj_2 =  " << std::endl << temp_adj_2.length() << std::endl;  

     U_jg.setCol(i, temp_adj_2) ;
   
     } ;
           //       std::cout << "U_jg =  " << std::endl << U_jg.toString() << std::endl;  
    // std::cout << "U_jg =  " << std::endl << U_jg.toString() << std::endl;  
     
     U_sg_2 = mg*crossProductMatrix(g_dir_aw)* U_sg_2 ;
     U_sg.setSubmatrix( U_sg_1, 0, 0) ;
     U_sg.setSubmatrix( U_sg_2, 3, 0) ;
     
     yarp::sig::Matrix U_s_l_tot(6 , 6) ;
     yarp::sig::Matrix U_s_r_tot(6 , 6) ;
     yarp::sig::Matrix U_s_lr(6 , 6) ;
     yarp::sig::Matrix U_s_tot(6 , 6) ;
     
     U_s_l_tot = U_s_l_c1 + U_s_l_c2 + U_s_l_c3 + U_s_l_c4 ;     
     U_s_r_tot = U_s_r_c1 + U_s_r_c2 + U_s_r_c3 + U_s_r_c4 ;
     U_s_lr = U_s_l_tot + U_s_r_tot  ; 
     U_s_tot = U_s_l_tot + U_s_r_tot + U_sg ;     

//     std::cout << "U_sg =  " << std::endl << U_sg.toString() << std::endl;       
//     std::cout << "U_s_lr =  " << std::endl << U_s_lr.toString() << std::endl;            
//     std::cout << "U_s_tot =  " << std::endl << U_s_tot.toString() << std::endl;       
     
//-------------------------------------------------------------------------------------------------------------

//     yarp::sig::Matrix U_s_r_tot(6 , 6) ;
//     U_s_r_tot = U_s_r_c1 + U_s_r_c2 + U_s_r_c3 + U_s_r_c4 ;
  //-------------------------------------------------------------------------------------------------------------------
     //  Computing Q_s

     //-------------------------------------------------------------------------------------------------------------------
     //  Computing Q_s_l     

     yarp::sig::Vector q_incr = q_current ; 
     
     yarp::sig::Matrix Q_s_l_c1(6,6 ) ;
     yarp::sig::Matrix Q_s_l_c2(6,6 ) ;
     yarp::sig::Matrix Q_s_l_c3(6,6 ) ;
     yarp::sig::Matrix Q_s_l_c4(6,6 ) ;
     Q_s_l_c1.zero();     
     Q_s_l_c2.zero();
     Q_s_l_c3.zero();
     Q_s_l_c4.zero();
     
     yarp::sig::Vector Q_s_l_c1_i(6, 0.0) ;
     yarp::sig::Vector Q_s_l_c2_i(6, 0.0) ;
     yarp::sig::Vector Q_s_l_c3_i(6, 0.0) ;
     yarp::sig::Vector Q_s_l_c4_i(6, 0.0) ;

     yarp::sig::Matrix T_imu_w_incr(4,4) ;     //  = iHomogeneous(T_w_imu_incr) ;
     yarp::sig::Matrix T_aw_w_incr(4,4) ;     //  = T_aw_imu * T_imu_incr_w ;

     T_imu_w_incr.zero();
     T_aw_w_incr.zero();

     yarp::sig::Matrix T_w_l_c1_incr(4,4) ;    
     yarp::sig::Matrix T_w_l_c2_incr(4,4) ;
     yarp::sig::Matrix T_w_l_c3_incr(4,4) ;
     yarp::sig::Matrix T_w_l_c4_incr(4,4) ;
     T_w_l_c1_incr.zero();
     T_w_l_c2_incr.zero();
     T_w_l_c3_incr.zero();
     T_w_l_c4_incr.zero();
  // 
     yarp::sig::Matrix T_aw_l_c1_incr(4,4) ;
     yarp::sig::Matrix T_aw_l_c2_incr(4,4) ;
     yarp::sig::Matrix T_aw_l_c3_incr(4,4) ;
     yarp::sig::Matrix T_aw_l_c4_incr(4,4) ;
     T_aw_l_c1_incr.zero();
     T_aw_l_c2_incr.zero();
     T_aw_l_c3_incr.zero();
     T_aw_l_c4_incr.zero();   
     
     for ( int i = 0  ; i<6 ; i++ ) 
     {
      q_incr = q_current ;
      q_incr[i] += h ;
      robot.idynutils.updateiDyn3Model( q_incr, true ); //update model first  

      T_imu_w_incr  = model.iDyn3_model.getPosition( imu_link_index ,true ) ;    
      T_aw_w_incr   = T_aw_imu * T_imu_w_incr ;
     
      T_w_l_c1_incr = model.iDyn3_model.getPosition( l_foot_upper_left_link_index  )  ;    // l_c1 = l_foot_upper_left_link_index
      T_w_l_c2_incr = model.iDyn3_model.getPosition( l_foot_upper_right_link_index )  ;    // l_c2 = l_foot_upper_right_link_index
      T_w_l_c3_incr = model.iDyn3_model.getPosition( l_foot_lower_left_link_index  )  ;    // l_c3 = l_foot_lower_left_link_index
      T_w_l_c4_incr = model.iDyn3_model.getPosition( l_foot_lower_right_link_index )  ;    // l_c4 = l_foot_lower_right_link_index
  
      T_aw_l_c1_incr  = T_aw_w_incr * T_w_l_c1_incr ;  
      T_aw_l_c2_incr  = T_aw_w_incr * T_w_l_c2_incr ;  
      T_aw_l_c3_incr  = T_aw_w_incr * T_w_l_c3_incr ;  
      T_aw_l_c4_incr  = T_aw_w_incr * T_w_l_c4_incr ;  

      Q_s_l_c1_i = ( (Adjoint_MT(T_aw_l_c1_incr) - Adjoint_MT(T_aw_l_c1_0 )) /h )*B_select.transposed()*fc_l_c1  ;
      Q_s_l_c2_i = ( (Adjoint_MT(T_aw_l_c2_incr) - Adjoint_MT(T_aw_l_c2_0 )) /h )*B_select.transposed()*fc_l_c2  ;
      Q_s_l_c3_i = ( (Adjoint_MT(T_aw_l_c3_incr) - Adjoint_MT(T_aw_l_c3_0 )) /h )*B_select.transposed()*fc_l_c3  ;
      Q_s_l_c4_i = ( (Adjoint_MT(T_aw_l_c4_incr) - Adjoint_MT(T_aw_l_c4_0 )) /h )*B_select.transposed()*fc_l_c4  ;
         
      Q_s_l_c1.setCol( i ,  Q_s_l_c1_i ) ;
      Q_s_l_c2.setCol( i ,  Q_s_l_c2_i ) ;
      Q_s_l_c3.setCol( i ,  Q_s_l_c3_i ) ;
      Q_s_l_c4.setCol( i ,  Q_s_l_c4_i ) ;   
     } ;
      
   //  std::cout << "Q_s_l_c1 =  " << std::endl << Q_s_l_c1.toString() << std::endl; 
   //   std::cout << "Q_s_l_c2 =  " << std::endl << Q_s_l_c2.toString() << std::endl; 
   //  std::cout << "Q_s_l_c3 =  " << std::endl << Q_s_l_c3.toString() << std::endl; 
   //  std::cout << "Q_s_l_c4 =  " << std::endl << Q_s_l_c4.toString() << std::endl;  

     yarp::sig::Matrix Q_s_l = Q_s_l_c1 + Q_s_l_c2 + Q_s_l_c3 + Q_s_l_c4 ;
   //  std::cout << "Q_s_l =  " << std::endl << Q_s_l.toString() << std::endl; 

     //-------------------------------------------------------------------------------------------------------------------
     //  Computing Q_s_r     
     //-------------------------------------------------------------------
    
     yarp::sig::Matrix Q_s_r_c1(6,6 ) ;
     yarp::sig::Matrix Q_s_r_c2(6,6 ) ;
     yarp::sig::Matrix Q_s_r_c3(6,6 ) ;
     yarp::sig::Matrix Q_s_r_c4(6,6 ) ;
     Q_s_r_c1.zero();  
     Q_s_r_c2.zero();
     Q_s_r_c3.zero();    
     Q_s_r_c4.zero();
     
     yarp::sig::Vector Q_s_r_c1_i(6, 0.0) ;
     yarp::sig::Vector Q_s_r_c2_i(6, 0.0) ;
     yarp::sig::Vector Q_s_r_c3_i(6, 0.0) ;
     yarp::sig::Vector Q_s_r_c4_i(6, 0.0) ; 
           
     yarp::sig::Matrix T_w_r_c1_incr(4,4) ;
     yarp::sig::Matrix T_w_r_c2_incr(4,4) ;
     yarp::sig::Matrix T_w_r_c3_incr(4,4) ;
     yarp::sig::Matrix T_w_r_c4_incr(4,4) ;
     T_w_r_c1_incr.zero();     
     T_w_r_c2_incr.zero();
     T_w_r_c3_incr.zero();
     T_w_r_c4_incr.zero();
  // 
     yarp::sig::Matrix T_aw_r_c1_incr(4,4) ;
     yarp::sig::Matrix T_aw_r_c2_incr(4,4) ;
     yarp::sig::Matrix T_aw_r_c3_incr(4,4) ;
     yarp::sig::Matrix T_aw_r_c4_incr(4,4) ;
     T_aw_r_c1_incr.zero();
     T_aw_r_c2_incr.zero();
     T_aw_r_c3_incr.zero();  
     T_aw_r_c4_incr.zero();
     
     for ( int i = 6  ; i<12 ; i++ ) 
     {
      q_incr = q_current ;
      q_incr[i] += h ;
      robot.idynutils.updateiDyn3Model( q_incr, true ); //update model first  

      T_imu_w_incr  = model.iDyn3_model.getPosition( imu_link_index ,true ) ;    
      T_aw_w_incr   = T_aw_imu * T_imu_w_incr ;
     
      T_w_r_c1_incr = model.iDyn3_model.getPosition( r_foot_upper_left_link_index  )  ;    // r_c1 = r_foot_upper_left_link_index
      T_w_r_c2_incr = model.iDyn3_model.getPosition( r_foot_upper_right_link_index )  ;    // r_c2 = r_foot_upper_right_link_index
      T_w_r_c3_incr = model.iDyn3_model.getPosition( r_foot_lower_left_link_index  )  ;    // r_c3 = r_foot_lower_left_link_index
      T_w_r_c4_incr = model.iDyn3_model.getPosition( r_foot_lower_right_link_index )  ;    // r_c4 = r_foot_lower_right_link_index
  
      T_aw_r_c1_incr  = T_aw_w_incr * T_w_r_c1_incr ;  
      T_aw_r_c2_incr  = T_aw_w_incr * T_w_r_c2_incr ;  
      T_aw_r_c3_incr  = T_aw_w_incr * T_w_r_c3_incr ;  
      T_aw_r_c4_incr  = T_aw_w_incr * T_w_r_c4_incr ;  

      Q_s_r_c1_i = ( (Adjoint_MT(T_aw_r_c1_incr) - Adjoint_MT( T_aw_r_c1_0 )) /h )*B_select.transposed()*fc_r_c1  ;
      Q_s_r_c2_i = ( (Adjoint_MT(T_aw_r_c2_incr) - Adjoint_MT( T_aw_r_c2_0 )) /h )*B_select.transposed()*fc_r_c2  ;
      Q_s_r_c3_i = ( (Adjoint_MT(T_aw_r_c3_incr) - Adjoint_MT( T_aw_r_c3_0 )) /h )*B_select.transposed()*fc_r_c3  ;
      Q_s_r_c4_i = ( (Adjoint_MT(T_aw_r_c4_incr) - Adjoint_MT( T_aw_r_c4_0 )) /h )*B_select.transposed()*fc_r_c4  ;
         
      Q_s_r_c1.setCol( i-6 ,  Q_s_r_c1_i ) ;
      Q_s_r_c2.setCol( i-6 ,  Q_s_r_c2_i ) ;
      Q_s_r_c3.setCol( i-6 ,  Q_s_r_c3_i ) ;
      Q_s_r_c4.setCol( i-6 ,  Q_s_r_c4_i ) ;   // QUI
     } ;
      
   //  std::cout << "Q_s_l_c1 =  " << std::endl << Q_s_l_c1.toString() << std::endl; 
   //  std::cout << "Q_s_l_c2 =  " << std::endl << Q_s_l_c2.toString() << std::endl; 
   //  std::cout << "Q_s_l_c3 =  " << std::endl << Q_s_l_c3.toString() << std::endl; 
   //  std::cout << "Q_s_l_c4 =  " << std::endl << Q_s_l_c4.toString() << std::endl;   

     yarp::sig::Matrix  Q_s_r = Q_s_r_c1 + Q_s_r_c2 + Q_s_r_c3 + Q_s_r_c4 ;
 //    std::cout << "Q_s_r =  " << std::endl << Q_s_r.toString() << std::endl; 
      
     yarp::sig::Matrix Q_s_lr( 6 , robot.getNumberOfJoints() ) ;
     
     Q_s_lr.setSubmatrix(Q_s_l , 0 , 0) ;
     Q_s_lr.setSubmatrix(Q_s_r , 0 , 6) ;

     yarp::sig::Matrix Q_sg(6,  robot.getNumberOfJoints() ) ;
     yarp::sig::Matrix Q_sg_1(3,  robot.getNumberOfJoints() )  ;
     yarp::sig::Matrix Q_sg_2 = mg*crossProductMatrix(g_dir_aw) * Jac_COM_aw_0    ;
     Q_sg.setSubmatrix(Q_sg_1, 0 , 0) ;
     Q_sg.setSubmatrix(Q_sg_2, 3 , 0) ; 

     yarp::sig::Matrix Q_s_tot = Q_s_lr + Q_sg   ;     

//     std::cout << "Q_s_lr =  " << std::endl << Q_s_lr.toString() << std::endl; 
//     std::cout << "Q_sg =  " << std::endl << Q_sg.toString() << std::endl; 
     
//     std::cout << "Q_s_tot =  " << std::endl << Q_s_tot.toString() << std::endl; 

  //   U_s_tot = U_s_l_tot + U_s_r_tot + U_mg;
  //   Q_s_tot = Q_s_lr + Q_mg;     
    
      
//---------------------------------------------------------------------------------------------------------------------
      // Starting Q_j Computation
  
     yarp::sig::Matrix Q_j_lr(robot.getNumberOfJoints(), robot.getNumberOfJoints())  ;
     Q_j_lr.zero() ; 
     yarp::sig::Matrix Q_j_l( robot.getNumberOfJoints() , 6) ;
     yarp::sig::Matrix Q_j_l_c1( robot.getNumberOfJoints() , 6) ;
     yarp::sig::Matrix Q_j_l_c2( robot.getNumberOfJoints() , 6) ;
     yarp::sig::Matrix Q_j_l_c3( robot.getNumberOfJoints() , 6) ;
     yarp::sig::Matrix Q_j_l_c4( robot.getNumberOfJoints() , 6) ;
     
     yarp::sig::Matrix Q_j_r( robot.getNumberOfJoints() , 6) ;
     yarp::sig::Matrix Q_j_r_c1( robot.getNumberOfJoints() , 6) ;
     yarp::sig::Matrix Q_j_r_c2( robot.getNumberOfJoints() , 6) ;
     yarp::sig::Matrix Q_j_r_c3( robot.getNumberOfJoints() , 6) ;
     yarp::sig::Matrix Q_j_r_c4( robot.getNumberOfJoints() , 6) ;

     yarp::sig::Matrix Jac_l_c1_incr_long( robot.getNumberOfJoints() + 6 , 6)  ; //mixed, long, incremented version
     yarp::sig::Matrix Jac_l_c2_incr_long( robot.getNumberOfJoints() + 6 , 6)  ; //mixed, long, incremented version
     yarp::sig::Matrix Jac_l_c3_incr_long( robot.getNumberOfJoints() + 6 , 6)  ; //mixed, long, incremented version
     yarp::sig::Matrix Jac_l_c4_incr_long( robot.getNumberOfJoints() + 6 , 6)  ; //mixed, long, incremented version

     yarp::sig::Matrix Jac_r_c1_incr_long( robot.getNumberOfJoints() + 6 , 6)  ; //mixed, long, incremented version
     yarp::sig::Matrix Jac_r_c2_incr_long( robot.getNumberOfJoints() + 6 , 6)  ; //mixed, long, incremented version
     yarp::sig::Matrix Jac_r_c3_incr_long( robot.getNumberOfJoints() + 6 , 6)  ; //mixed, long, incremented version
     yarp::sig::Matrix Jac_r_c4_incr_long( robot.getNumberOfJoints() + 6 , 6)  ; //mixed, long, incremented version     

     yarp::sig::Matrix Jac_l_c1_incr( robot.getNumberOfJoints(), 6) ; //spatial short, incremented version
     yarp::sig::Matrix Jac_l_c2_incr( robot.getNumberOfJoints(), 6) ; //spatial short, incremented version
     yarp::sig::Matrix Jac_l_c3_incr( robot.getNumberOfJoints(), 6) ; //spatial short, incremented version
     yarp::sig::Matrix Jac_l_c4_incr( robot.getNumberOfJoints(), 6) ; //spatial short, incremented version

     yarp::sig::Matrix Jac_r_c1_incr( robot.getNumberOfJoints(), 6) ; //spatial short, incremented version
     yarp::sig::Matrix Jac_r_c2_incr( robot.getNumberOfJoints(), 6) ; //spatial short, incremented version
     yarp::sig::Matrix Jac_r_c3_incr( robot.getNumberOfJoints(), 6) ; //spatial short, incremented version
     yarp::sig::Matrix Jac_r_c4_incr( robot.getNumberOfJoints(), 6) ; //spatial short, incremented version
     
     yarp::sig::Matrix T_l_c1_w_incr(4,4) ;
     yarp::sig::Matrix T_l_c2_w_incr(4,4) ;
     yarp::sig::Matrix T_l_c3_w_incr(4,4) ;
     yarp::sig::Matrix T_l_c4_w_incr(4,4) ;

     yarp::sig::Matrix T_r_c1_w_incr(4,4) ;
     yarp::sig::Matrix T_r_c2_w_incr(4,4) ;
     yarp::sig::Matrix T_r_c3_w_incr(4,4) ;
     yarp::sig::Matrix T_r_c4_w_incr(4,4) ;

     yarp::sig::Matrix R_T_l_c1_w_incr(4,4) ;
     yarp::sig::Matrix R_T_l_c2_w_incr(4,4) ;
     yarp::sig::Matrix R_T_l_c3_w_incr(4,4) ;
     yarp::sig::Matrix R_T_l_c4_w_incr(4,4) ;

     yarp::sig::Matrix R_T_r_c1_w_incr(4,4) ;
     yarp::sig::Matrix R_T_r_c2_w_incr(4,4) ;
     yarp::sig::Matrix R_T_r_c3_w_incr(4,4) ;
     yarp::sig::Matrix R_T_r_c4_w_incr(4,4) ;
     
     yarp::sig::Matrix Jac_l_c1_incr_body(6, robot.getNumberOfJoints() ) ;  //body short, incremented version
     yarp::sig::Matrix Jac_l_c2_incr_body(6, robot.getNumberOfJoints() ) ;  //body short, incremented version
     yarp::sig::Matrix Jac_l_c3_incr_body(6, robot.getNumberOfJoints() ) ;  //body short, incremented version
     yarp::sig::Matrix Jac_l_c4_incr_body(6, robot.getNumberOfJoints() ) ;  //body short, incremented version

     yarp::sig::Matrix Jac_r_c1_incr_body(6, robot.getNumberOfJoints() ) ;  //body short, incremented version
     yarp::sig::Matrix Jac_r_c2_incr_body(6, robot.getNumberOfJoints() ) ;  //body short, incremented version
     yarp::sig::Matrix Jac_r_c3_incr_body(6, robot.getNumberOfJoints() ) ;  //body short, incremented version
     yarp::sig::Matrix Jac_r_c4_incr_body(6, robot.getNumberOfJoints() ) ;  //body short, incremented version
     
     //-------------------------------------------------------------------------------------------------------------------
     // REMIND NOTATION
     //  Jac_aw_l_foot_upper_left_link       -> spatial long (+6 joints) in the initial configuration (not incemented for derivatives)
     //  Jac_aw_l_foot_upper_left_link_body  -> body long (+6 joints) in the initial configuration (not incemented for derivatives)
     //  Jac_l_c1_spa_0                      -> spatial short in the initial configuration (not incemented for derivatives)
     //  Jac_l_c1_body_0                     -> spatial short in the initial configuration (not incemented for derivatives)
        
     yarp::sig::Vector Q_j_l_c1_i( robot.getNumberOfJoints() ) ;
     yarp::sig::Vector Q_j_l_c2_i( robot.getNumberOfJoints() ) ;
     yarp::sig::Vector Q_j_l_c3_i( robot.getNumberOfJoints() ) ;
     yarp::sig::Vector Q_j_l_c4_i( robot.getNumberOfJoints() ) ;

     yarp::sig::Vector Q_j_r_c1_i( robot.getNumberOfJoints() ) ;
     yarp::sig::Vector Q_j_r_c2_i( robot.getNumberOfJoints() ) ;
     yarp::sig::Vector Q_j_r_c3_i( robot.getNumberOfJoints() ) ;
     yarp::sig::Vector Q_j_r_c4_i( robot.getNumberOfJoints() ) ;   
     
     //---------------------------------------------------------------------------
     // Q_j_l computation     
     for ( int i = 0  ; i<6 ; i++ ) 
     {
      q_incr = q_current ;
      q_incr[i] += h ;
      robot.idynutils.updateiDyn3Model( q_incr, true ); //update model first  
      //
      model.iDyn3_model.getJacobian( l_foot_upper_left_link_index  , Jac_l_c1_incr_long  , false  ) ; //false= mixed version jacobian //true= body jacobian
      model.iDyn3_model.getJacobian( l_foot_upper_right_link_index , Jac_l_c2_incr_long  , false  ) ; //false= mixed version jacobian //true= body jacobian
      model.iDyn3_model.getJacobian( l_foot_lower_left_link_index  , Jac_l_c3_incr_long  , false  ) ; //false= mixed version jacobian //true= body jacobian
      model.iDyn3_model.getJacobian( l_foot_lower_right_link_index , Jac_l_c4_incr_long  , false  ) ; //false= mixed version jacobian //true= body jacobian

      Jac_l_c1_incr = Jac_l_c1_incr_long.submatrix(0,5, 6, Jac_l_c1_incr_long.cols()-1 ) ; // mixed short
      Jac_l_c2_incr = Jac_l_c2_incr_long.submatrix(0,5, 6, Jac_l_c2_incr_long.cols()-1 ) ;
      Jac_l_c3_incr = Jac_l_c3_incr_long.submatrix(0,5, 6, Jac_l_c3_incr_long.cols()-1 ) ;
      Jac_l_c4_incr = Jac_l_c4_incr_long.submatrix(0,5, 6, Jac_l_c4_incr_long.cols()-1 ) ;
      
      T_l_c1_w_incr  = model.iDyn3_model.getPosition(  l_foot_upper_left_link_index  , true ) ;
      T_l_c2_w_incr  = model.iDyn3_model.getPosition(  l_foot_upper_right_link_index , true ) ;
      T_l_c3_w_incr  = model.iDyn3_model.getPosition(  l_foot_lower_left_link_index  , true ) ;
      T_l_c4_w_incr  = model.iDyn3_model.getPosition(  l_foot_lower_right_link_index , true) ;
      
      R_T_l_c1_w_incr =  getRot( T_l_c1_w_incr  );
      R_T_l_c2_w_incr =  getRot( T_l_c2_w_incr  );
      R_T_l_c3_w_incr =  getRot( T_l_c3_w_incr  );
      R_T_l_c4_w_incr =  getRot( T_l_c4_w_incr  );
      
      Jac_l_c1_incr_body = Adjoint(  Homogeneous(R_T_l_c1_w_incr, zero_3) ) *  Jac_l_c1_incr ;  
      Jac_l_c2_incr_body = Adjoint(  Homogeneous(R_T_l_c2_w_incr, zero_3) ) *  Jac_l_c2_incr ;  
      Jac_l_c3_incr_body = Adjoint(  Homogeneous(R_T_l_c3_w_incr, zero_3) ) *  Jac_l_c3_incr ;  
      Jac_l_c4_incr_body = Adjoint(  Homogeneous(R_T_l_c4_w_incr, zero_3) ) *  Jac_l_c4_incr ;  
     
      Q_j_l_c1_i = ( ( Jac_l_c1_incr_body - Jac_l_c1_body_0)/h  ).transposed()*B_select.transposed()*  fc_l_c1 ;
      Q_j_l_c2_i = ( ( Jac_l_c2_incr_body - Jac_l_c2_body_0)/h  ).transposed()*B_select.transposed()*  fc_l_c2 ;
      Q_j_l_c3_i = ( ( Jac_l_c3_incr_body - Jac_l_c3_body_0)/h  ).transposed()*B_select.transposed()*  fc_l_c3 ;
      Q_j_l_c4_i = ( ( Jac_l_c4_incr_body - Jac_l_c4_body_0)/h  ).transposed()*B_select.transposed()*  fc_l_c4 ;
      
      Q_j_l_c1.setCol( i ,  Q_j_l_c1_i ) ;
      Q_j_l_c2.setCol( i ,  Q_j_l_c2_i ) ;
      Q_j_l_c3.setCol( i ,  Q_j_l_c3_i ) ;
      Q_j_l_c4.setCol( i ,  Q_j_l_c4_i ) ;
     } ;   

     Q_j_l = Q_j_l_c1 + Q_j_l_c2 + Q_j_l_c3 + Q_j_l_c4 ;

     
     //---------------------------------------------------------------------------
     // Q_j_r computation
     for ( int i = 6  ; i<12 ; i++ ) {
      q_incr = q_current ;      
      q_incr[i] += h ;

      robot.idynutils.updateiDyn3Model( q_incr, true ); //update model first  
      
      model.iDyn3_model.getJacobian( r_foot_upper_left_link_index  , Jac_r_c1_incr_long  , false  ) ; //false= mixed version jacobian //true= body jacobian
      model.iDyn3_model.getJacobian( r_foot_upper_right_link_index , Jac_r_c2_incr_long  , false  ) ; //false= mixed version jacobian //true= body jacobian
      model.iDyn3_model.getJacobian( r_foot_lower_left_link_index  , Jac_r_c3_incr_long  , false  ) ; //false= mixed version jacobian //true= body jacobian
      model.iDyn3_model.getJacobian( r_foot_lower_right_link_index , Jac_r_c4_incr_long  , false  ) ;
       
      Jac_r_c1_incr = Jac_r_c1_incr_long.submatrix(0,5, 6, Jac_r_c1_incr_long.cols()-1 ) ;
      Jac_r_c2_incr = Jac_r_c2_incr_long.submatrix(0,5, 6, Jac_r_c2_incr_long.cols()-1 ) ;
      Jac_r_c3_incr = Jac_r_c3_incr_long.submatrix(0,5, 6, Jac_r_c3_incr_long.cols()-1 ) ;
      Jac_r_c4_incr = Jac_r_c4_incr_long.submatrix(0,5, 6, Jac_r_c4_incr_long.cols()-1 ) ;
       
      T_r_c1_w_incr  = model.iDyn3_model.getPosition(  r_foot_upper_left_link_index  , true ) ;
      T_r_c2_w_incr  = model.iDyn3_model.getPosition(  r_foot_upper_right_link_index , true ) ;
      T_r_c3_w_incr  = model.iDyn3_model.getPosition(  r_foot_lower_left_link_index  , true ) ;
      T_r_c4_w_incr  = model.iDyn3_model.getPosition(  r_foot_lower_right_link_index , true ) ;
       
      R_T_r_c1_w_incr =  getRot( T_r_c1_w_incr  );
      R_T_r_c2_w_incr =  getRot( T_r_c2_w_incr  );
      R_T_r_c3_w_incr =  getRot( T_r_c3_w_incr  );
      R_T_r_c4_w_incr =  getRot( T_r_c4_w_incr  );
       
      Jac_r_c1_incr_body = Adjoint(  Homogeneous(R_T_r_c1_w_incr, zero_3) ) *  Jac_r_c1_incr ;  
      Jac_r_c2_incr_body = Adjoint(  Homogeneous(R_T_r_c2_w_incr, zero_3) ) *  Jac_r_c2_incr ;  
      Jac_r_c3_incr_body = Adjoint(  Homogeneous(R_T_r_c3_w_incr, zero_3) ) *  Jac_r_c3_incr ;  
      Jac_r_c4_incr_body = Adjoint(  Homogeneous(R_T_r_c4_w_incr, zero_3) ) *  Jac_r_c4_incr ;  
       
      Q_j_r_c1_i = ( ( Jac_r_c1_incr_body - Jac_r_c1_body_0)/h  ).transposed()*B_select.transposed()*  fc_r_c1 ;
      Q_j_r_c2_i = ( ( Jac_r_c2_incr_body - Jac_r_c2_body_0)/h  ).transposed()*B_select.transposed()*  fc_r_c2 ;
      Q_j_r_c3_i = ( ( Jac_r_c3_incr_body - Jac_r_c3_body_0)/h  ).transposed()*B_select.transposed()*  fc_r_c3 ;
      Q_j_r_c4_i = ( ( Jac_r_c4_incr_body - Jac_r_c4_body_0)/h  ).transposed()*B_select.transposed()*  fc_r_c4 ;
       
      Q_j_r_c1.setCol( i-6 ,  Q_j_r_c1_i ) ;
      Q_j_r_c2.setCol( i-6 ,  Q_j_r_c2_i ) ;
      Q_j_r_c3.setCol( i-6 ,  Q_j_r_c3_i ) ;
      Q_j_r_c4.setCol( i-6 ,  Q_j_r_c4_i ) ; 
     }     
     
     Q_j_r = Q_j_r_c1 + Q_j_r_c2 + Q_j_r_c3 + Q_j_r_c4 ;
     Q_j_lr.setSubmatrix(Q_j_l, 0, 0 )  ;
     Q_j_lr.setSubmatrix(Q_j_r, 0, 6 )  ;
 
     //--------------------------------------------------------------------------------------------------
     // Computing Q_jg
     //--------------------------------------
 
     yarp::sig::Matrix Jac_COM_w_incr(6, robot.getNumberOfJoints()+6 ) ;
     yarp::sig::Matrix T_w_imu_incr(4,4)  ;     
     yarp::sig::Matrix Jac_COM_aw_long_incr(6, robot.getNumberOfJoints()+6 ) ;
     yarp::sig::Matrix Jac_COM_aw_incr( 3, robot.getNumberOfJoints() )     ;
     yarp::sig::Vector d_Jg_i(robot.getNumberOfJoints()) ;
     yarp::sig::Matrix Q_jg( robot.getNumberOfJoints() , robot.getNumberOfJoints()) ;
     
     for ( int i = 0  ; i<(robot.getNumberOfJoints()-1) ; i++ ) 
     {
      q_incr = q_current ;
      q_incr[i] += h ;
      robot.idynutils.updateiDyn3Model( q_incr, true ); //update model first  
      model.iDyn3_model.getCOMJacobian( Jac_COM_w_incr ) ;  
      T_w_imu_incr  = model.iDyn3_model.getPosition( imu_link_index  )   ;
      T_aw_w_incr = iHomogeneous( T_w_imu_incr * T_imu_aw  ) ;    
      Jac_COM_aw_long_incr = Adjoint( Homogeneous( getRot( T_aw_w_incr ) , zero_3) )*Jac_COM_w_incr ;    
      Jac_COM_aw_incr = Jac_COM_aw_long_incr.submatrix( 0, 2, 6, Jac_COM_aw_long_incr.cols()-1 ) ; 
      d_Jg_i = ((Jac_COM_aw_incr - Jac_COM_aw_0)/h).transposed()* mg*g_dir_aw;   
      Q_jg.setCol( i ,  d_Jg_i ) ;     
     }
     
    //    std::cout << "Q_jg : "  << std::endl << Q_jg.toString() << std::endl;
    //  std::cout << "Q_jg.rows : "  << std::endl << Q_jg.rows() << std::endl;
    //  std::cout << "Q_jg.cols : "  << std::endl << Q_jg.cols() << std::endl;
     
      yarp::sig::Matrix Q_j_tot = Q_j_lr + Q_jg ;   
  //    std::cout << "Q_j_tot : "  << std::endl << Q_j_tot.toString() << std::endl;

  //  U_j = U_jg ;  
    U_s = U_s_tot ;
    Q_j = Q_j_tot;
    Q_s = Q_s_tot ;       */
    
    //-----------------------------------------------------------------------------------------------------------------------

    U_j.zero();
    U_s.zero();
    Q_j.zero();
    Q_s.zero();  
    
  /*  U_s = U_c_tot.submatrix( 0 , 5, 0 , U_c_tot.cols()-1 ) ;
    Q_s = Q_c_tot.submatrix( 0 , 5, 0 , Q_c_tot.cols()-1 ) ;
    U_j = U_c_tot.submatrix( 6 , U_c_tot.rows()-1 , 0 , U_c_tot.cols()-1 ) ;
    Q_j = Q_c_tot.submatrix( 6 , Q_c_tot.rows()-1 , 0 , Q_c_tot.cols()-1 ) ;     */

    U_s = Q_tot_lie.submatrix( 0 , 5, 0 , 5 ) ;
    Q_s = Q_tot_lie.submatrix( 0 , 5, 6 , Q_tot_lie.cols()-1 ) ;
    U_j = Q_tot_lie.submatrix( 6 , U_c_tot.rows()-1 , 0 , 5) ;
    Q_j = Q_tot_lie.submatrix( 6 , Q_c_tot.rows()-1 , 6 , Q_tot_lie.cols()-1 ) ;    

  /*  std::cout << "U_s.rows =  " << std::endl << U_s.rows() << std::endl;  
    std::cout << "U_s.cols =  " << std::endl << U_s.cols()<< std::endl;  
    std::cout << "U_j.rows =  " << std::endl << U_j.rows() << std::endl;  
    std::cout << "U_j.cols =  " << std::endl << U_j.cols() << std::endl;    
    std::cout << "Q_s.rows =  " << std::endl << Q_s.rows() << std::endl;    
    std::cout << "Q_s.cols =  " << std::endl << Q_s.cols() << std::endl;    
    std::cout << "Q_j.rows =  " << std::endl << Q_j.rows() << std::endl;     
    std::cout << "Q_j.cols =  " << std::endl << Q_j.cols() << std::endl;  */
    
 /*   std::cout << "U_s =  " << std::endl << U_s.toString() << std::endl;    
    std::cout << "U_j =  " << std::endl << U_j.toString() << std::endl;    
    std::cout << "Q_s =  " << std::endl << Q_s.toString() << std::endl;    
    std::cout << "Q_j =  " << std::endl << Q_j.toString() << std::endl;     */

    //-------------------------------------------------------------------------------------//
    //
    int r_FLMM = size_fc + 6 + 2*size_q;
    int c_FLMM = size_fc + 6 + 3*size_q;
    yarp::sig::Matrix FLMM(r_FLMM, c_FLMM) ;
    
    // Setting the first block-row of the FLMM
    FLMM.setSubmatrix( Eye_fc                           , 0 , 0                   ) ;
    FLMM.setSubmatrix( Zeros_fc_q                       , 0 , size_fc             ) ;
    FLMM.setSubmatrix( -1.0 * Kc*Stance_c.transposed()  , 0 , size_fc+size_q      ) ;
    FLMM.setSubmatrix( -1.0 * Kc*Jacob_c                , 0 , size_fc+size_q+6    ) ;
    FLMM.setSubmatrix( Zeros_fc_q                       , 0 , size_fc+2*size_q+6  ) ;

    // Setting the second block-row of the FLMM
    FLMM.setSubmatrix( -1.0*Jacob_c.transposed()    , size_fc , 0                  ) ;
    FLMM.setSubmatrix( Eye_tau                      , size_fc , size_fc            ) ;
    FLMM.setSubmatrix( -1.0 * U_j                   , size_fc , size_fc+size_q     ) ;
    FLMM.setSubmatrix( -1.0 * Q_j                   , size_fc , size_fc+size_q+6   ) ;
    FLMM.setSubmatrix( Zeros_q_q                    , size_fc , size_fc+2*size_q+6 ) ;

    // Setting the third block-row of the FLMM
    FLMM.setSubmatrix( -1.0*Stance_c , size_fc +size_q , 0                  ) ;
    FLMM.setSubmatrix( Zeros_6_q     , size_fc +size_q , size_fc            ) ;
    FLMM.setSubmatrix( -1.0 * U_s    , size_fc +size_q , size_fc+size_q     ) ;
    FLMM.setSubmatrix( -1.0 * Q_s    , size_fc +size_q , size_fc+size_q+6   ) ;
    FLMM.setSubmatrix( Zeros_6_q     , size_fc +size_q , size_fc+2*size_q+6 ) ;

    // Setting the fourth block-row of the FLMM
    FLMM.setSubmatrix( Zeros_q_fc  , size_fc +size_q +6  , 0                  ) ;
    FLMM.setSubmatrix( Eye_tau     , size_fc +size_q +6  , size_fc            ) ;
    FLMM.setSubmatrix( Zeros_q_6   , size_fc +size_q +6  , size_fc+size_q     ) ;
    FLMM.setSubmatrix( Kq          , size_fc +size_q +6  , size_fc+size_q+6   ) ;
    FLMM.setSubmatrix( -1.0*Kq     , size_fc +size_q +6  , size_fc+2*size_q+6 ) ;
 
    yarp::sig::Matrix Phi_star_d = FLMM.submatrix(0, FLMM.rows()-1, 0,   FLMM.rows()-1    )  ;

    yarp::sig::Matrix cFLMM =  yarp::math::luinv(Phi_star_d)*FLMM  ;
       
    yarp::sig::Matrix Phi_d = cFLMM.submatrix(0, cFLMM.rows()-1, 0 ,  size_fc + 2*size_q + 6-1     )  ;    
    yarp::sig::Matrix Phi_i = cFLMM.submatrix(0, cFLMM.rows()-1, size_fc + 2*size_q + 6 ,   cFLMM.cols()-1     )  ;

    yarp::sig::Matrix R_f = Phi_i.submatrix(0, size_fc-1 , 0 , Phi_i.cols()-1 ) ;    

    //-------------------------------------------------------------------
   // Alternative formulation for R_f
 /*   yarp::sig::Matrix Q_j_1 = -1.0*Q_j - 1.0* Jacob_c.transposed()*Kc*Jacob_c  ;
    yarp::sig::Matrix U_j_1 = -1.0*U_j -1.0*Jacob_c.transposed() *Kc*Stance_c.transposed() ;    
    yarp::sig::Matrix Q_s_1 =  -1.0*Q_s-1.0*Stance_c*Kc*Jacob_c  ;
    yarp::sig::Matrix U_s_1 = -1.0*U_s-1.0*Stance_c*Kc*Stance_c.transposed() ;    
    yarp::sig::Matrix L = yarp::math::luinv(U_s_1)* Q_s_1 ;
    yarp::sig::Matrix M = Q_j_1-U_j_1*L ;    
    yarp::sig::Matrix H = Kq-M ;
    yarp::sig::Matrix F = -1.0*yarp::math::luinv(H)*Kq ;    
    yarp::sig::Matrix E = -1.0*Kc* Stance_c.transposed()* L *F ;
    yarp::sig::Matrix R_f_1 = E+Kc*Jacob_c*F  ;
    yarp::sig::Matrix d_R_f = R_f-R_f_1  ;
    std::cout << " R_f = " <<  std::endl << R_f.toString() << std::endl; 
    std::cout << " R_f_1 = " <<  std::endl << R_f_1.toString() << std::endl; 
    std::cout << " d_R_f = " <<  std::endl << d_R_f.toString() << std::endl;   */

    //------------------------------------------------------------------------------//
    yarp::sig::Vector fc_actual_to_world( fc_l_contacts_to_world.length() + fc_r_contacts_to_world.length())  ;      
    
    fc_actual_to_world.setSubvector( 0 , fc_l_contacts_to_world ) ;
    fc_actual_to_world.setSubvector( fc_l_contacts_to_world.length() , fc_r_contacts_to_world ) ;
    
    // desired contact force definition
    yarp::sig::Vector fc_desired_to_world( fc_l_contacts_to_world.length() + fc_l_contacts_to_world.length())  ;
    
    fc_desired_to_world[0] = 0  ;
    fc_desired_to_world[1] = 0  ;
    fc_desired_to_world[2] = - mg/8 ;
    fc_desired_to_world[3] = 0  ;
    fc_desired_to_world[4] = 0  ;
    fc_desired_to_world[5] = - mg/8  ;
    fc_desired_to_world[6] = 0 ;
    fc_desired_to_world[7] = 0 ;
    fc_desired_to_world[8] = - mg/8  ;
    fc_desired_to_world[9] =  0 ;
    fc_desired_to_world[10] = 0 ;
    fc_desired_to_world[11] = - mg/8  ;

    fc_desired_to_world[12] = 0  ;
    fc_desired_to_world[13] = 0  ;
    fc_desired_to_world[14] = - mg/8 ;
    fc_desired_to_world[15] = 0  ;
    fc_desired_to_world[16] = 0  ;
    fc_desired_to_world[17] = - mg/8  ;
    fc_desired_to_world[18] = 0 ;
    fc_desired_to_world[19] = 0 ;
    fc_desired_to_world[20] = - mg/8  ;
    fc_desired_to_world[21] = 0 ;
    fc_desired_to_world[22] = 0 ;
    fc_desired_to_world[23] = - mg/8  ;    
  
    yarp::sig::Vector d_fc_desired_to_world = fc_desired_to_world - fc_actual_to_world ;
    
    yarp::sig::Vector d_q_motor_desired(robot.getNumberOfJoints() ) ;
 
 //  d_q_motor_desired = -1.0* pinv( R_f , 1E-7 ) * d_fc_desired_to_world ; 
  //      std::cout << " d_q_motor_desired_7 = " <<  std::endl << d_q_motor_desired.toString() << std::endl; 

  d_q_motor_desired = -1.0* pinv( R_f , 1E-6 ) * d_fc_desired_to_world ; 
 //       std::cout << " d_q_motor_desired_6 = " <<  std::endl << d_q_motor_desired.toString() << std::endl;     
    
 //   d_q_motor_desired = -1.0* pinv( R_f , 1E-5 ) * d_fc_desired_to_world ; 
   //     std::cout << " d_q_motor_desired_5 = " <<  std::endl << d_q_motor_desired.toString() << std::endl; 	
    
//    d_q_motor_desired = -1.0* pinv( R_f , 1E-4 ) * d_fc_desired_to_world ; 
     //   std::cout << " d_q_motor_desired_4 = " <<  std::endl << d_q_motor_desired.toString() << std::endl; 		
    
    
    yarp::sig::Vector fc_teor   = d_fc_desired_to_world - 1.0*R_f *d_q_motor_desired ;
    yarp::sig::Vector d_fc_teor = d_fc_desired_to_world - fc_teor ;
    
    std::cout << " fc_actual_to_world  = " <<  std::endl << fc_actual_to_world.toString() << std::endl; 
    std::cout << " fc_desired_to_world  = " <<  std::endl << fc_desired_to_world.toString() << std::endl;    
    std::cout << " norm( d_fc_desired_to_world ) = " <<  std::endl << norm( d_fc_desired_to_world ) << std::endl; 
    
    
    