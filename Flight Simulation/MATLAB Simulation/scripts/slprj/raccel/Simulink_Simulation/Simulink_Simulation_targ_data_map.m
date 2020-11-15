  function targMap = targDataMap(),

  ;%***********************
  ;% Create Parameter Map *
  ;%***********************
      
    nTotData      = 0; %add to this count as we go
    nTotSects     = 9;
    sectIdxOffset = 0;
    
    ;%
    ;% Define dummy sections & preallocate arrays
    ;%
    dumSection.nData = -1;  
    dumSection.data  = [];
    
    dumData.logicalSrcIdx = -1;
    dumData.dtTransOffset = -1;
    
    ;%
    ;% Init/prealloc paramMap
    ;%
    paramMap.nSections           = nTotSects;
    paramMap.sectIdxOffset       = sectIdxOffset;
      paramMap.sections(nTotSects) = dumSection; %prealloc
    paramMap.nTotData            = -1;
    
    ;%
    ;% Auto data (rtP)
    ;%
      section.nData     = 90;
      section.data(90)  = dumData; %prealloc
      
	  ;% rtP.LatLong0
	  section.data(1).logicalSrcIdx = 0;
	  section.data(1).dtTransOffset = 0;
	
	  ;% rtP.Sref
	  section.data(2).logicalSrcIdx = 1;
	  section.data(2).dtTransOffset = 2;
	
	  ;% rtP.V0
	  section.data(3).logicalSrcIdx = 2;
	  section.data(3).dtTransOffset = 3;
	
	  ;% rtP.alpha0
	  section.data(4).logicalSrcIdx = 3;
	  section.data(4).dtTransOffset = 4;
	
	  ;% rtP.alt0
	  section.data(5).logicalSrcIdx = 4;
	  section.data(5).dtTransOffset = 5;
	
	  ;% rtP.bref
	  section.data(6).logicalSrcIdx = 5;
	  section.data(6).dtTransOffset = 6;
	
	  ;% rtP.cbar
	  section.data(7).logicalSrcIdx = 6;
	  section.data(7).dtTransOffset = 7;
	
	  ;% rtP.heading0
	  section.data(8).logicalSrcIdx = 7;
	  section.data(8).dtTransOffset = 8;
	
	  ;% rtP.mass
	  section.data(9).logicalSrcIdx = 8;
	  section.data(9).dtTransOffset = 9;
	
	  ;% rtP.maxdef_aileron
	  section.data(10).logicalSrcIdx = 9;
	  section.data(10).dtTransOffset = 10;
	
	  ;% rtP.maxdef_elevator
	  section.data(11).logicalSrcIdx = 10;
	  section.data(11).dtTransOffset = 11;
	
	  ;% rtP.maxdef_rudder
	  section.data(12).logicalSrcIdx = 11;
	  section.data(12).dtTransOffset = 12;
	
	  ;% rtP.mindef_aileron
	  section.data(13).logicalSrcIdx = 12;
	  section.data(13).dtTransOffset = 13;
	
	  ;% rtP.mindef_elevator
	  section.data(14).logicalSrcIdx = 13;
	  section.data(14).dtTransOffset = 14;
	
	  ;% rtP.mindef_rudder
	  section.data(15).logicalSrcIdx = 14;
	  section.data(15).dtTransOffset = 15;
	
	  ;% rtP.theta0
	  section.data(16).logicalSrcIdx = 15;
	  section.data(16).dtTransOffset = 16;
	
	  ;% rtP.wn_act
	  section.data(17).logicalSrcIdx = 16;
	  section.data(17).dtTransOffset = 17;
	
	  ;% rtP.wy0
	  section.data(18).logicalSrcIdx = 17;
	  section.data(18).dtTransOffset = 18;
	
	  ;% rtP.z_act
	  section.data(19).logicalSrcIdx = 18;
	  section.data(19).dtTransOffset = 19;
	
	  ;% rtP.DiscreteWindGustModel_Gx
	  section.data(20).logicalSrcIdx = 19;
	  section.data(20).dtTransOffset = 20;
	
	  ;% rtP.DiscreteWindGustModel_Gy
	  section.data(21).logicalSrcIdx = 20;
	  section.data(21).dtTransOffset = 21;
	
	  ;% rtP.DiscreteWindGustModel_Gz
	  section.data(22).logicalSrcIdx = 21;
	  section.data(22).dtTransOffset = 22;
	
	  ;% rtP.uDOFBodyAxes_Iyy
	  section.data(23).logicalSrcIdx = 22;
	  section.data(23).dtTransOffset = 23;
	
	  ;% rtP.TAS2CAS_LUTM0
	  section.data(24).logicalSrcIdx = 23;
	  section.data(24).dtTransOffset = 24;
	
	  ;% rtP.CAS2TAS_LUTM0
	  section.data(25).logicalSrcIdx = 24;
	  section.data(25).dtTransOffset = 322104;
	
	  ;% rtP.TAS2CAS_LUTM1
	  section.data(26).logicalSrcIdx = 25;
	  section.data(26).dtTransOffset = 644184;
	
	  ;% rtP.CAS2TAS_LUTM1
	  section.data(27).logicalSrcIdx = 26;
	  section.data(27).dtTransOffset = 966264;
	
	  ;% rtP.TAS2CAS_LUTM2
	  section.data(28).logicalSrcIdx = 27;
	  section.data(28).dtTransOffset = 1288344;
	
	  ;% rtP.CAS2TAS_LUTM2
	  section.data(29).logicalSrcIdx = 28;
	  section.data(29).dtTransOffset = 1610424;
	
	  ;% rtP.TAS2CAS_LUTM3
	  section.data(30).logicalSrcIdx = 29;
	  section.data(30).dtTransOffset = 1932504;
	
	  ;% rtP.CAS2TAS_LUTM3
	  section.data(31).logicalSrcIdx = 30;
	  section.data(31).dtTransOffset = 2254584;
	
	  ;% rtP.TAS2CAS_LUTM4
	  section.data(32).logicalSrcIdx = 31;
	  section.data(32).dtTransOffset = 2576664;
	
	  ;% rtP.CAS2TAS_LUTM4
	  section.data(33).logicalSrcIdx = 32;
	  section.data(33).dtTransOffset = 2898744;
	
	  ;% rtP.DrydenWindTurbulenceModelContinuousqr_L_high
	  section.data(34).logicalSrcIdx = 33;
	  section.data(34).dtTransOffset = 3220824;
	
	  ;% rtP.TAS2CAS_P_bp
	  section.data(35).logicalSrcIdx = 34;
	  section.data(35).dtTransOffset = 3220825;
	
	  ;% rtP.CAS2TAS_P_bp
	  section.data(36).logicalSrcIdx = 35;
	  section.data(36).dtTransOffset = 3220913;
	
	  ;% rtP.AerodynamicForcesandMoments_S
	  section.data(37).logicalSrcIdx = 36;
	  section.data(37).dtTransOffset = 3221001;
	
	  ;% rtP.AerodynamicForcesandMoments_S_miuqcjxiwz
	  section.data(38).logicalSrcIdx = 37;
	  section.data(38).dtTransOffset = 3221002;
	
	  ;% rtP.TAS2CAS_SOS_bp
	  section.data(39).logicalSrcIdx = 38;
	  section.data(39).dtTransOffset = 3221003;
	
	  ;% rtP.CAS2TAS_SOS_bp
	  section.data(40).logicalSrcIdx = 39;
	  section.data(40).dtTransOffset = 3221064;
	
	  ;% rtP.DrydenWindTurbulenceModelContinuousqr_Seed
	  section.data(41).logicalSrcIdx = 40;
	  section.data(41).dtTransOffset = 3221125;
	
	  ;% rtP.DrydenWindTurbulenceModelContinuousqr_T_on
	  section.data(42).logicalSrcIdx = 41;
	  section.data(42).dtTransOffset = 3221129;
	
	  ;% rtP.WhiteNoise_Ts
	  section.data(43).logicalSrcIdx = 42;
	  section.data(43).dtTransOffset = 3221130;
	
	  ;% rtP.DrydenWindTurbulenceModelContinuousqr_TurbProb
	  section.data(44).logicalSrcIdx = 43;
	  section.data(44).dtTransOffset = 3221131;
	
	  ;% rtP.TAS2CAS_V_bp_M0
	  section.data(45).logicalSrcIdx = 44;
	  section.data(45).dtTransOffset = 3221132;
	
	  ;% rtP.CAS2TAS_V_bp_M0
	  section.data(46).logicalSrcIdx = 45;
	  section.data(46).dtTransOffset = 3221192;
	
	  ;% rtP.TAS2CAS_V_bp_M1
	  section.data(47).logicalSrcIdx = 46;
	  section.data(47).dtTransOffset = 3221252;
	
	  ;% rtP.CAS2TAS_V_bp_M1
	  section.data(48).logicalSrcIdx = 47;
	  section.data(48).dtTransOffset = 3221312;
	
	  ;% rtP.TAS2CAS_V_bp_M2
	  section.data(49).logicalSrcIdx = 48;
	  section.data(49).dtTransOffset = 3221372;
	
	  ;% rtP.CAS2TAS_V_bp_M2
	  section.data(50).logicalSrcIdx = 49;
	  section.data(50).dtTransOffset = 3221432;
	
	  ;% rtP.TAS2CAS_V_bp_M3
	  section.data(51).logicalSrcIdx = 50;
	  section.data(51).dtTransOffset = 3221492;
	
	  ;% rtP.CAS2TAS_V_bp_M3
	  section.data(52).logicalSrcIdx = 51;
	  section.data(52).dtTransOffset = 3221552;
	
	  ;% rtP.TAS2CAS_V_bp_M4
	  section.data(53).logicalSrcIdx = 52;
	  section.data(53).dtTransOffset = 3221612;
	
	  ;% rtP.CAS2TAS_V_bp_M4
	  section.data(54).logicalSrcIdx = 53;
	  section.data(54).dtTransOffset = 3221672;
	
	  ;% rtP.DrydenWindTurbulenceModelContinuousqr_W20
	  section.data(55).logicalSrcIdx = 54;
	  section.data(55).dtTransOffset = 3221732;
	
	  ;% rtP.WindShearModel_W_20
	  section.data(56).logicalSrcIdx = 55;
	  section.data(56).dtTransOffset = 3221733;
	
	  ;% rtP.DrydenWindTurbulenceModelContinuousqr_Wdeg
	  section.data(57).logicalSrcIdx = 56;
	  section.data(57).dtTransOffset = 3221734;
	
	  ;% rtP.WindShearModel_Wdeg
	  section.data(58).logicalSrcIdx = 57;
	  section.data(58).dtTransOffset = 3221735;
	
	  ;% rtP.DrydenWindTurbulenceModelContinuousqr_Wingspan
	  section.data(59).logicalSrcIdx = 58;
	  section.data(59).dtTransOffset = 3221736;
	
	  ;% rtP.COESAAtmosphereModel_action
	  section.data(60).logicalSrcIdx = 59;
	  section.data(60).dtTransOffset = 3221737;
	
	  ;% rtP.AerodynamicForcesandMoments_b
	  section.data(61).logicalSrcIdx = 60;
	  section.data(61).dtTransOffset = 3221738;
	
	  ;% rtP.AerodynamicForcesandMoments_b_blpao4glfe
	  section.data(62).logicalSrcIdx = 61;
	  section.data(62).dtTransOffset = 3221739;
	
	  ;% rtP.AerodynamicForcesandMoments_cbar
	  section.data(63).logicalSrcIdx = 62;
	  section.data(63).dtTransOffset = 3221740;
	
	  ;% rtP.AerodynamicForcesandMoments_cbar_ect3p4btxl
	  section.data(64).logicalSrcIdx = 63;
	  section.data(64).dtTransOffset = 3221741;
	
	  ;% rtP.CompareToConstant_const
	  section.data(65).logicalSrcIdx = 64;
	  section.data(65).dtTransOffset = 3221742;
	
	  ;% rtP.CompareToConstant_const_j4cxad0q0g
	  section.data(66).logicalSrcIdx = 65;
	  section.data(66).dtTransOffset = 3221743;
	
	  ;% rtP.CompareToConstant_const_omj1fbfglm
	  section.data(67).logicalSrcIdx = 66;
	  section.data(67).dtTransOffset = 3221744;
	
	  ;% rtP.CompareToConstant_const_n1mct5qu1a
	  section.data(68).logicalSrcIdx = 67;
	  section.data(68).dtTransOffset = 3221745;
	
	  ;% rtP.CompareToConstant_const_d4qgtkvaqb
	  section.data(69).logicalSrcIdx = 68;
	  section.data(69).dtTransOffset = 3221746;
	
	  ;% rtP.CompareToConstant_const_fkrbb1l4we
	  section.data(70).logicalSrcIdx = 69;
	  section.data(70).dtTransOffset = 3221747;
	
	  ;% rtP.CompareToConstant_const_o4jsn2vuck
	  section.data(71).logicalSrcIdx = 70;
	  section.data(71).dtTransOffset = 3221748;
	
	  ;% rtP.CompareToConstant_const_n1tmrxoekt
	  section.data(72).logicalSrcIdx = 71;
	  section.data(72).dtTransOffset = 3221749;
	
	  ;% rtP.Distanceintogustx_d_m
	  section.data(73).logicalSrcIdx = 72;
	  section.data(73).dtTransOffset = 3221750;
	
	  ;% rtP.Distanceintogusty_d_m
	  section.data(74).logicalSrcIdx = 73;
	  section.data(74).dtTransOffset = 3221751;
	
	  ;% rtP.Distanceintogustz_d_m
	  section.data(75).logicalSrcIdx = 74;
	  section.data(75).dtTransOffset = 3221752;
	
	  ;% rtP.DiscreteWindGustModel_d_m
	  section.data(76).logicalSrcIdx = 75;
	  section.data(76).dtTransOffset = 3221753;
	
	  ;% rtP.NonlinearSecondOrderActuator1_fin_act_0
	  section.data(77).logicalSrcIdx = 76;
	  section.data(77).dtTransOffset = 3221756;
	
	  ;% rtP.NonlinearSecondOrderActuator_fin_act_0
	  section.data(78).logicalSrcIdx = 77;
	  section.data(78).dtTransOffset = 3221757;
	
	  ;% rtP.NonlinearSecondOrderActuator2_fin_act_0
	  section.data(79).logicalSrcIdx = 78;
	  section.data(79).dtTransOffset = 3221758;
	
	  ;% rtP.NonlinearSecondOrderActuator1_fin_act_vel
	  section.data(80).logicalSrcIdx = 79;
	  section.data(80).dtTransOffset = 3221759;
	
	  ;% rtP.NonlinearSecondOrderActuator_fin_act_vel
	  section.data(81).logicalSrcIdx = 80;
	  section.data(81).dtTransOffset = 3221760;
	
	  ;% rtP.NonlinearSecondOrderActuator2_fin_act_vel
	  section.data(82).logicalSrcIdx = 81;
	  section.data(82).dtTransOffset = 3221761;
	
	  ;% rtP.NonlinearSecondOrderActuator1_fin_maxrate
	  section.data(83).logicalSrcIdx = 82;
	  section.data(83).dtTransOffset = 3221762;
	
	  ;% rtP.NonlinearSecondOrderActuator_fin_maxrate
	  section.data(84).logicalSrcIdx = 83;
	  section.data(84).dtTransOffset = 3221763;
	
	  ;% rtP.NonlinearSecondOrderActuator2_fin_maxrate
	  section.data(85).logicalSrcIdx = 84;
	  section.data(85).dtTransOffset = 3221764;
	
	  ;% rtP.uDOFBodyAxes_g
	  section.data(86).logicalSrcIdx = 85;
	  section.data(86).dtTransOffset = 3221765;
	
	  ;% rtP.uDOFBodyAxes_pos_ini
	  section.data(87).logicalSrcIdx = 86;
	  section.data(87).dtTransOffset = 3221766;
	
	  ;% rtP.WhiteNoise_pwr
	  section.data(88).logicalSrcIdx = 87;
	  section.data(88).dtTransOffset = 3221768;
	
	  ;% rtP.DiscreteWindGustModel_t_0
	  section.data(89).logicalSrcIdx = 88;
	  section.data(89).dtTransOffset = 3221772;
	
	  ;% rtP.DiscreteWindGustModel_v_m
	  section.data(90).logicalSrcIdx = 89;
	  section.data(90).dtTransOffset = 3221773;
	
      nTotData = nTotData + section.nData;
      paramMap.sections(1) = section;
      clear section
      
      section.nData     = 3;
      section.data(3)  = dumData; %prealloc
      
	  ;% rtP.FlightGearPreconfigured6DoFAnimation_DestinationPort
	  section.data(1).logicalSrcIdx = 90;
	  section.data(1).dtTransOffset = 0;
	
	  ;% rtP.CompareToConstant1_const
	  section.data(2).logicalSrcIdx = 91;
	  section.data(2).dtTransOffset = 1;
	
	  ;% rtP.CompareToConstant1_const_gvieufb20s
	  section.data(3).logicalSrcIdx = 92;
	  section.data(3).dtTransOffset = 2;
	
      nTotData = nTotData + section.nData;
      paramMap.sections(2) = section;
      clear section
      
      section.nData     = 260;
      section.data(260)  = dumData; %prealloc
      
	  ;% rtP.x_Y0
	  section.data(1).logicalSrcIdx = 93;
	  section.data(1).dtTransOffset = 0;
	
	  ;% rtP.DistanceintoGustxLimitedtogustlengthd_IC
	  section.data(2).logicalSrcIdx = 94;
	  section.data(2).dtTransOffset = 1;
	
	  ;% rtP.DistanceintoGustxLimitedtogustlengthd_LowerSat
	  section.data(3).logicalSrcIdx = 95;
	  section.data(3).dtTransOffset = 2;
	
	  ;% rtP.pgw_Y0
	  section.data(4).logicalSrcIdx = 96;
	  section.data(4).dtTransOffset = 3;
	
	  ;% rtP.pgw_p_IC
	  section.data(5).logicalSrcIdx = 97;
	  section.data(5).dtTransOffset = 4;
	
	  ;% rtP.Constant1_Value
	  section.data(6).logicalSrcIdx = 98;
	  section.data(6).dtTransOffset = 5;
	
	  ;% rtP.Constant2_Value
	  section.data(7).logicalSrcIdx = 99;
	  section.data(7).dtTransOffset = 6;
	
	  ;% rtP.Constant3_Value
	  section.data(8).logicalSrcIdx = 100;
	  section.data(8).dtTransOffset = 7;
	
	  ;% rtP.qgw_Y0
	  section.data(9).logicalSrcIdx = 101;
	  section.data(9).dtTransOffset = 8;
	
	  ;% rtP.qgw_p_IC
	  section.data(10).logicalSrcIdx = 102;
	  section.data(10).dtTransOffset = 9;
	
	  ;% rtP.pi4_Gain
	  section.data(11).logicalSrcIdx = 103;
	  section.data(11).dtTransOffset = 10;
	
	  ;% rtP.rgw_Y0
	  section.data(12).logicalSrcIdx = 104;
	  section.data(12).dtTransOffset = 11;
	
	  ;% rtP.rgw_p_IC
	  section.data(13).logicalSrcIdx = 105;
	  section.data(13).dtTransOffset = 12;
	
	  ;% rtP.pi3_Gain
	  section.data(14).logicalSrcIdx = 106;
	  section.data(14).dtTransOffset = 13;
	
	  ;% rtP.ugw_Y0
	  section.data(15).logicalSrcIdx = 107;
	  section.data(15).dtTransOffset = 14;
	
	  ;% rtP.upi_Gain
	  section.data(16).logicalSrcIdx = 108;
	  section.data(16).dtTransOffset = 15;
	
	  ;% rtP.ug_p_IC
	  section.data(17).logicalSrcIdx = 109;
	  section.data(17).dtTransOffset = 16;
	
	  ;% rtP.vgw_Y0
	  section.data(18).logicalSrcIdx = 110;
	  section.data(18).dtTransOffset = 17;
	
	  ;% rtP.upi_Gain_dmjomzxo4q
	  section.data(19).logicalSrcIdx = 111;
	  section.data(19).dtTransOffset = 18;
	
	  ;% rtP.vg_p1_IC
	  section.data(20).logicalSrcIdx = 112;
	  section.data(20).dtTransOffset = 19;
	
	  ;% rtP.vgw_p2_IC
	  section.data(21).logicalSrcIdx = 113;
	  section.data(21).dtTransOffset = 20;
	
	  ;% rtP.sqrt3_Gain
	  section.data(22).logicalSrcIdx = 114;
	  section.data(22).dtTransOffset = 21;
	
	  ;% rtP.wgw_Y0
	  section.data(23).logicalSrcIdx = 115;
	  section.data(23).dtTransOffset = 22;
	
	  ;% rtP.upi_Gain_ii5rsz2pui
	  section.data(24).logicalSrcIdx = 116;
	  section.data(24).dtTransOffset = 23;
	
	  ;% rtP.wg_p1_IC
	  section.data(25).logicalSrcIdx = 117;
	  section.data(25).dtTransOffset = 24;
	
	  ;% rtP.wg_p2_IC
	  section.data(26).logicalSrcIdx = 118;
	  section.data(26).dtTransOffset = 25;
	
	  ;% rtP.Constant_Value
	  section.data(27).logicalSrcIdx = 119;
	  section.data(27).dtTransOffset = 26;
	
	  ;% rtP.Gain_Gain
	  section.data(28).logicalSrcIdx = 120;
	  section.data(28).dtTransOffset = 27;
	
	  ;% rtP.max_height_low_Value
	  section.data(29).logicalSrcIdx = 121;
	  section.data(29).dtTransOffset = 28;
	
	  ;% rtP.min_height_high_Value
	  section.data(30).logicalSrcIdx = 122;
	  section.data(30).dtTransOffset = 29;
	
	  ;% rtP.Gain_Gain_bzpcr2c2vd
	  section.data(31).logicalSrcIdx = 123;
	  section.data(31).dtTransOffset = 30;
	
	  ;% rtP.max_height_low_Value_bkbuta4yqd
	  section.data(32).logicalSrcIdx = 124;
	  section.data(32).dtTransOffset = 31;
	
	  ;% rtP.min_height_high_Value_okjndayw2f
	  section.data(33).logicalSrcIdx = 125;
	  section.data(33).dtTransOffset = 32;
	
	  ;% rtP.GravityinEarthAxes_Gain
	  section.data(34).logicalSrcIdx = 126;
	  section.data(34).dtTransOffset = 33;
	
	  ;% rtP.LimitFunction10ftto1000ft_UpperSat
	  section.data(35).logicalSrcIdx = 127;
	  section.data(35).dtTransOffset = 36;
	
	  ;% rtP.LimitFunction10ftto1000ft_LowerSat
	  section.data(36).logicalSrcIdx = 128;
	  section.data(36).dtTransOffset = 37;
	
	  ;% rtP.Lw_Gain
	  section.data(37).logicalSrcIdx = 129;
	  section.data(37).dtTransOffset = 38;
	
	  ;% rtP.PreLookUpIndexSearchaltitude_BreakpointsData
	  section.data(38).logicalSrcIdx = 130;
	  section.data(38).dtTransOffset = 39;
	
	  ;% rtP.MediumHighAltitudeIntensity_Table
	  section.data(39).logicalSrcIdx = 131;
	  section.data(39).dtTransOffset = 51;
	
	  ;% rtP.WhiteNoise_Mean
	  section.data(40).logicalSrcIdx = 132;
	  section.data(40).dtTransOffset = 135;
	
	  ;% rtP.WhiteNoise_StdDev
	  section.data(41).logicalSrcIdx = 133;
	  section.data(41).dtTransOffset = 136;
	
	  ;% rtP.LimitHeighth1000ft_UpperSat
	  section.data(42).logicalSrcIdx = 134;
	  section.data(42).dtTransOffset = 137;
	
	  ;% rtP.LimitHeighth1000ft_LowerSat
	  section.data(43).logicalSrcIdx = 135;
	  section.data(43).dtTransOffset = 138;
	
	  ;% rtP.Lv_Gain
	  section.data(44).logicalSrcIdx = 136;
	  section.data(44).dtTransOffset = 139;
	
	  ;% rtP.uftinf_UpperSat
	  section.data(45).logicalSrcIdx = 137;
	  section.data(45).dtTransOffset = 140;
	
	  ;% rtP.uftinf_LowerSat
	  section.data(46).logicalSrcIdx = 138;
	  section.data(46).dtTransOffset = 141;
	
	  ;% rtP.hz0_Gain
	  section.data(47).logicalSrcIdx = 139;
	  section.data(47).dtTransOffset = 142;
	
	  ;% rtP.Longitude_Value
	  section.data(48).logicalSrcIdx = 140;
	  section.data(48).dtTransOffset = 143;
	
	  ;% rtP.u_Value
	  section.data(49).logicalSrcIdx = 141;
	  section.data(49).dtTransOffset = 144;
	
	  ;% rtP.sigma_wg_Gain
	  section.data(50).logicalSrcIdx = 142;
	  section.data(50).dtTransOffset = 145;
	
	  ;% rtP.PreLookUpIndexSearchprobofexceed_BreakpointsData
	  section.data(51).logicalSrcIdx = 143;
	  section.data(51).dtTransOffset = 146;
	
	  ;% rtP.Wdeg1_Value
	  section.data(52).logicalSrcIdx = 144;
	  section.data(52).dtTransOffset = 153;
	
	  ;% rtP.ref_heightz0_Value
	  section.data(53).logicalSrcIdx = 145;
	  section.data(53).dtTransOffset = 154;
	
	  ;% rtP.Step_Time
	  section.data(54).logicalSrcIdx = 146;
	  section.data(54).dtTransOffset = 155;
	
	  ;% rtP.Step_Y0
	  section.data(55).logicalSrcIdx = 147;
	  section.data(55).dtTransOffset = 156;
	
	  ;% rtP.Step_YFinal
	  section.data(56).logicalSrcIdx = 148;
	  section.data(56).dtTransOffset = 157;
	
	  ;% rtP.StateSpace_A
	  section.data(57).logicalSrcIdx = 149;
	  section.data(57).dtTransOffset = 158;
	
	  ;% rtP.StateSpace_B
	  section.data(58).logicalSrcIdx = 150;
	  section.data(58).dtTransOffset = 159;
	
	  ;% rtP.StateSpace_C
	  section.data(59).logicalSrcIdx = 151;
	  section.data(59).dtTransOffset = 160;
	
	  ;% rtP.StateSpace_InitialCondition
	  section.data(60).logicalSrcIdx = 152;
	  section.data(60).dtTransOffset = 161;
	
	  ;% rtP.RateLimiter_RisingLim
	  section.data(61).logicalSrcIdx = 153;
	  section.data(61).dtTransOffset = 162;
	
	  ;% rtP.RateLimiter_FallingLim
	  section.data(62).logicalSrcIdx = 154;
	  section.data(62).dtTransOffset = 163;
	
	  ;% rtP.throttle_Value
	  section.data(63).logicalSrcIdx = 155;
	  section.data(63).dtTransOffset = 164;
	
	  ;% rtP.asOut_Y0
	  section.data(64).logicalSrcIdx = 156;
	  section.data(64).dtTransOffset = 165;
	
	  ;% rtP.Memory_InitialCondition
	  section.data(65).logicalSrcIdx = 157;
	  section.data(65).dtTransOffset = 166;
	
	  ;% rtP.Bias6_Bias
	  section.data(66).logicalSrcIdx = 158;
	  section.data(66).dtTransOffset = 167;
	
	  ;% rtP.Bias5_Bias
	  section.data(67).logicalSrcIdx = 159;
	  section.data(67).dtTransOffset = 168;
	
	  ;% rtP.Bias4_Bias
	  section.data(68).logicalSrcIdx = 160;
	  section.data(68).dtTransOffset = 169;
	
	  ;% rtP.Constant1_Value_iv5ptljryc
	  section.data(69).logicalSrcIdx = 161;
	  section.data(69).dtTransOffset = 170;
	
	  ;% rtP.Constant3_Value_h0acrtutky
	  section.data(70).logicalSrcIdx = 162;
	  section.data(70).dtTransOffset = 171;
	
	  ;% rtP.Constant4_Value
	  section.data(71).logicalSrcIdx = 163;
	  section.data(71).dtTransOffset = 172;
	
	  ;% rtP.Bias1_Bias
	  section.data(72).logicalSrcIdx = 164;
	  section.data(72).dtTransOffset = 173;
	
	  ;% rtP.Bias3_Bias
	  section.data(73).logicalSrcIdx = 165;
	  section.data(73).dtTransOffset = 174;
	
	  ;% rtP.Bias1_Bias_cjyekcjkqd
	  section.data(74).logicalSrcIdx = 166;
	  section.data(74).dtTransOffset = 175;
	
	  ;% rtP.Bias_Bias
	  section.data(75).logicalSrcIdx = 167;
	  section.data(75).dtTransOffset = 176;
	
	  ;% rtP.Bias2_Bias
	  section.data(76).logicalSrcIdx = 168;
	  section.data(76).dtTransOffset = 177;
	
	  ;% rtP.Bias5_Bias_czj0v4rf55
	  section.data(77).logicalSrcIdx = 169;
	  section.data(77).dtTransOffset = 178;
	
	  ;% rtP.Bias4_Bias_cy2cy0rkru
	  section.data(78).logicalSrcIdx = 170;
	  section.data(78).dtTransOffset = 179;
	
	  ;% rtP.Bias6_Bias_cgt3io1in0
	  section.data(79).logicalSrcIdx = 171;
	  section.data(79).dtTransOffset = 180;
	
	  ;% rtP.Constant1_Value_jb5j0tvdpj
	  section.data(80).logicalSrcIdx = 172;
	  section.data(80).dtTransOffset = 181;
	
	  ;% rtP.Constant_Value_nv1uo0d3mg
	  section.data(81).logicalSrcIdx = 173;
	  section.data(81).dtTransOffset = 182;
	
	  ;% rtP.Constant2_Value_pdjiwz3ftx
	  section.data(82).logicalSrcIdx = 174;
	  section.data(82).dtTransOffset = 183;
	
	  ;% rtP.Constant_Value_bfoige4wh4
	  section.data(83).logicalSrcIdx = 175;
	  section.data(83).dtTransOffset = 184;
	
	  ;% rtP.asOut_Y0_fruk2gwjsm
	  section.data(84).logicalSrcIdx = 176;
	  section.data(84).dtTransOffset = 185;
	
	  ;% rtP.Memory_InitialCondition_lgdo1d41mj
	  section.data(85).logicalSrcIdx = 177;
	  section.data(85).dtTransOffset = 186;
	
	  ;% rtP.Bias6_Bias_hm1r0zlf2u
	  section.data(86).logicalSrcIdx = 178;
	  section.data(86).dtTransOffset = 187;
	
	  ;% rtP.Bias5_Bias_mcem25dje2
	  section.data(87).logicalSrcIdx = 179;
	  section.data(87).dtTransOffset = 188;
	
	  ;% rtP.Bias4_Bias_pl5eddn2p1
	  section.data(88).logicalSrcIdx = 180;
	  section.data(88).dtTransOffset = 189;
	
	  ;% rtP.Constant1_Value_pb5yn2hc0w
	  section.data(89).logicalSrcIdx = 181;
	  section.data(89).dtTransOffset = 190;
	
	  ;% rtP.Constant3_Value_kvwuhyt22j
	  section.data(90).logicalSrcIdx = 182;
	  section.data(90).dtTransOffset = 191;
	
	  ;% rtP.Constant4_Value_kox0vku1gy
	  section.data(91).logicalSrcIdx = 183;
	  section.data(91).dtTransOffset = 192;
	
	  ;% rtP.Bias1_Bias_hfkjqppc2q
	  section.data(92).logicalSrcIdx = 184;
	  section.data(92).dtTransOffset = 193;
	
	  ;% rtP.Bias3_Bias_jd2w4riqmn
	  section.data(93).logicalSrcIdx = 185;
	  section.data(93).dtTransOffset = 194;
	
	  ;% rtP.Bias1_Bias_ncd3jtrabd
	  section.data(94).logicalSrcIdx = 186;
	  section.data(94).dtTransOffset = 195;
	
	  ;% rtP.Bias_Bias_kruphzigt4
	  section.data(95).logicalSrcIdx = 187;
	  section.data(95).dtTransOffset = 196;
	
	  ;% rtP.Bias2_Bias_lotf5js1s3
	  section.data(96).logicalSrcIdx = 188;
	  section.data(96).dtTransOffset = 197;
	
	  ;% rtP.Bias5_Bias_meya4vfioi
	  section.data(97).logicalSrcIdx = 189;
	  section.data(97).dtTransOffset = 198;
	
	  ;% rtP.Bias4_Bias_odezqxp5ct
	  section.data(98).logicalSrcIdx = 190;
	  section.data(98).dtTransOffset = 199;
	
	  ;% rtP.Bias6_Bias_jskr1b1mi4
	  section.data(99).logicalSrcIdx = 191;
	  section.data(99).dtTransOffset = 200;
	
	  ;% rtP.Constant1_Value_ej01j13uct
	  section.data(100).logicalSrcIdx = 192;
	  section.data(100).dtTransOffset = 201;
	
	  ;% rtP.Constant_Value_lbccuktfh2
	  section.data(101).logicalSrcIdx = 193;
	  section.data(101).dtTransOffset = 202;
	
	  ;% rtP.Constant2_Value_bopb4ki4xd
	  section.data(102).logicalSrcIdx = 194;
	  section.data(102).dtTransOffset = 203;
	
	  ;% rtP.Constant_Value_kueaisvxhv
	  section.data(103).logicalSrcIdx = 195;
	  section.data(103).dtTransOffset = 204;
	
	  ;% rtP.Merge1_1_InitialOutput
	  section.data(104).logicalSrcIdx = 196;
	  section.data(104).dtTransOffset = 205;
	
	  ;% rtP.Merge1_2_InitialOutput
	  section.data(105).logicalSrcIdx = 197;
	  section.data(105).dtTransOffset = 206;
	
	  ;% rtP.Merge1_3_InitialOutput
	  section.data(106).logicalSrcIdx = 198;
	  section.data(106).dtTransOffset = 207;
	
	  ;% rtP.Merge1_4_InitialOutput
	  section.data(107).logicalSrcIdx = 199;
	  section.data(107).dtTransOffset = 208;
	
	  ;% rtP.Merge1_5_InitialOutput
	  section.data(108).logicalSrcIdx = 200;
	  section.data(108).dtTransOffset = 209;
	
	  ;% rtP.Merge1_6_InitialOutput
	  section.data(109).logicalSrcIdx = 201;
	  section.data(109).dtTransOffset = 210;
	
	  ;% rtP.Merge1_7_InitialOutput
	  section.data(110).logicalSrcIdx = 202;
	  section.data(110).dtTransOffset = 211;
	
	  ;% rtP.Merge1_8_InitialOutput
	  section.data(111).logicalSrcIdx = 203;
	  section.data(111).dtTransOffset = 212;
	
	  ;% rtP.Merge1_9_InitialOutput
	  section.data(112).logicalSrcIdx = 204;
	  section.data(112).dtTransOffset = 213;
	
	  ;% rtP.Constant_Value_pmyhtxrqrr
	  section.data(113).logicalSrcIdx = 205;
	  section.data(113).dtTransOffset = 214;
	
	  ;% rtP.Constant_Value_gun4ws4ai2
	  section.data(114).logicalSrcIdx = 206;
	  section.data(114).dtTransOffset = 215;
	
	  ;% rtP.Constant_Value_mq1pkcil10
	  section.data(115).logicalSrcIdx = 207;
	  section.data(115).dtTransOffset = 216;
	
	  ;% rtP.Bias_Bias_k5qjjnbf1r
	  section.data(116).logicalSrcIdx = 208;
	  section.data(116).dtTransOffset = 217;
	
	  ;% rtP.Gain_Gain_opd2wvnqlj
	  section.data(117).logicalSrcIdx = 209;
	  section.data(117).dtTransOffset = 218;
	
	  ;% rtP.Bias1_Bias_oluwwwumlb
	  section.data(118).logicalSrcIdx = 210;
	  section.data(118).dtTransOffset = 219;
	
	  ;% rtP.Bias_Bias_mahsvgbvzq
	  section.data(119).logicalSrcIdx = 211;
	  section.data(119).dtTransOffset = 220;
	
	  ;% rtP.Bias1_Bias_fiyyfd4u5t
	  section.data(120).logicalSrcIdx = 212;
	  section.data(120).dtTransOffset = 221;
	
	  ;% rtP.Bias_Bias_du0tiht0fj
	  section.data(121).logicalSrcIdx = 213;
	  section.data(121).dtTransOffset = 222;
	
	  ;% rtP.Bias1_Bias_nbxpttwzhh
	  section.data(122).logicalSrcIdx = 214;
	  section.data(122).dtTransOffset = 223;
	
	  ;% rtP.Packnet_fdmPacketforFlightGear_P8
	  section.data(123).logicalSrcIdx = 215;
	  section.data(123).dtTransOffset = 224;
	
	  ;% rtP.SimulationPace_P1
	  section.data(124).logicalSrcIdx = 216;
	  section.data(124).dtTransOffset = 225;
	
	  ;% rtP.SimulationPace_P2
	  section.data(125).logicalSrcIdx = 217;
	  section.data(125).dtTransOffset = 226;
	
	  ;% rtP.SimulationPace_P3
	  section.data(126).logicalSrcIdx = 218;
	  section.data(126).dtTransOffset = 227;
	
	  ;% rtP.SimulationPace_P4
	  section.data(127).logicalSrcIdx = 219;
	  section.data(127).dtTransOffset = 228;
	
	  ;% rtP.PacketSize_Value
	  section.data(128).logicalSrcIdx = 220;
	  section.data(128).dtTransOffset = 229;
	
	  ;% rtP.Theta_WrappedStateUpperValue
	  section.data(129).logicalSrcIdx = 221;
	  section.data(129).dtTransOffset = 230;
	
	  ;% rtP.Theta_WrappedStateLowerValue
	  section.data(130).logicalSrcIdx = 222;
	  section.data(130).dtTransOffset = 231;
	
	  ;% rtP.AltController_A
	  section.data(131).logicalSrcIdx = 223;
	  section.data(131).dtTransOffset = 232;
	
	  ;% rtP.AltController_B
	  section.data(132).logicalSrcIdx = 224;
	  section.data(132).dtTransOffset = 235;
	
	  ;% rtP.AltController_C
	  section.data(133).logicalSrcIdx = 225;
	  section.data(133).dtTransOffset = 236;
	
	  ;% rtP.AltController_D
	  section.data(134).logicalSrcIdx = 226;
	  section.data(134).dtTransOffset = 238;
	
	  ;% rtP.ThetaController_A
	  section.data(135).logicalSrcIdx = 228;
	  section.data(135).dtTransOffset = 239;
	
	  ;% rtP.ThetaController_C
	  section.data(136).logicalSrcIdx = 229;
	  section.data(136).dtTransOffset = 240;
	
	  ;% rtP.ThetaController_D
	  section.data(137).logicalSrcIdx = 230;
	  section.data(137).dtTransOffset = 241;
	
	  ;% rtP.Merge_InitialOutput
	  section.data(138).logicalSrcIdx = 232;
	  section.data(138).dtTransOffset = 242;
	
	  ;% rtP.u2rhoV2_Gain
	  section.data(139).logicalSrcIdx = 233;
	  section.data(139).dtTransOffset = 243;
	
	  ;% rtP.alpha_BreakpointsData
	  section.data(140).logicalSrcIdx = 234;
	  section.data(140).dtTransOffset = 244;
	
	  ;% rtP.Mach_BreakpointsData
	  section.data(141).logicalSrcIdx = 235;
	  section.data(141).dtTransOffset = 254;
	
	  ;% rtP.altitude_BreakpointsData
	  section.data(142).logicalSrcIdx = 236;
	  section.data(142).dtTransOffset = 258;
	
	  ;% rtP.CD_Table
	  section.data(143).logicalSrcIdx = 237;
	  section.data(143).dtTransOffset = 266;
	
	  ;% rtP.CYb_Table
	  section.data(144).logicalSrcIdx = 238;
	  section.data(144).dtTransOffset = 586;
	
	  ;% rtP.CL_Table
	  section.data(145).logicalSrcIdx = 239;
	  section.data(145).dtTransOffset = 906;
	
	  ;% rtP.coefAdjust_Gain
	  section.data(146).logicalSrcIdx = 240;
	  section.data(146).dtTransOffset = 1226;
	
	  ;% rtP.Clb_Table
	  section.data(147).logicalSrcIdx = 241;
	  section.data(147).dtTransOffset = 1229;
	
	  ;% rtP.Cm_Table
	  section.data(148).logicalSrcIdx = 242;
	  section.data(148).dtTransOffset = 1549;
	
	  ;% rtP.Cnb_Table
	  section.data(149).logicalSrcIdx = 243;
	  section.data(149).dtTransOffset = 1869;
	
	  ;% rtP.CYp_Table
	  section.data(150).logicalSrcIdx = 244;
	  section.data(150).dtTransOffset = 2189;
	
	  ;% rtP.CLad_Table
	  section.data(151).logicalSrcIdx = 245;
	  section.data(151).dtTransOffset = 2509;
	
	  ;% rtP.CLq_Table
	  section.data(152).logicalSrcIdx = 246;
	  section.data(152).dtTransOffset = 2829;
	
	  ;% rtP.Clp_Table
	  section.data(153).logicalSrcIdx = 247;
	  section.data(153).dtTransOffset = 3149;
	
	  ;% rtP.Clr_Table
	  section.data(154).logicalSrcIdx = 248;
	  section.data(154).dtTransOffset = 3469;
	
	  ;% rtP.Cmq_Table
	  section.data(155).logicalSrcIdx = 249;
	  section.data(155).dtTransOffset = 3789;
	
	  ;% rtP.Cmad_Table
	  section.data(156).logicalSrcIdx = 250;
	  section.data(156).dtTransOffset = 4109;
	
	  ;% rtP.Cnp_Table
	  section.data(157).logicalSrcIdx = 251;
	  section.data(157).dtTransOffset = 4429;
	
	  ;% rtP.Cnr_Table
	  section.data(158).logicalSrcIdx = 252;
	  section.data(158).dtTransOffset = 4749;
	
	  ;% rtP.Gain1_Gain
	  section.data(159).logicalSrcIdx = 253;
	  section.data(159).dtTransOffset = 5069;
	
	  ;% rtP.coefAdjust_Gain_m5bwkewsju
	  section.data(160).logicalSrcIdx = 254;
	  section.data(160).dtTransOffset = 5075;
	
	  ;% rtP.alpha_BreakpointsData_gcueg2tgd0
	  section.data(161).logicalSrcIdx = 255;
	  section.data(161).dtTransOffset = 5078;
	
	  ;% rtP.Mach_BreakpointsData_jg33uk3jci
	  section.data(162).logicalSrcIdx = 256;
	  section.data(162).dtTransOffset = 5088;
	
	  ;% rtP.altitude_BreakpointsData_p0oyeupgfw
	  section.data(163).logicalSrcIdx = 257;
	  section.data(163).dtTransOffset = 5092;
	
	  ;% rtP.delta_BreakpointsData
	  section.data(164).logicalSrcIdx = 258;
	  section.data(164).dtTransOffset = 5100;
	
	  ;% rtP.DCDI_Table
	  section.data(165).logicalSrcIdx = 259;
	  section.data(165).dtTransOffset = 5105;
	
	  ;% rtP.DCL_Table
	  section.data(166).logicalSrcIdx = 260;
	  section.data(166).dtTransOffset = 6705;
	
	  ;% rtP.DCm_Table
	  section.data(167).logicalSrcIdx = 261;
	  section.data(167).dtTransOffset = 6865;
	
	  ;% rtP.coefAdjust_Gain_oacgeai0xk
	  section.data(168).logicalSrcIdx = 262;
	  section.data(168).dtTransOffset = 7025;
	
	  ;% rtP.Xcp_Table
	  section.data(169).logicalSrcIdx = 263;
	  section.data(169).dtTransOffset = 7028;
	
	  ;% rtP.Thrust_LMN_Value
	  section.data(170).logicalSrcIdx = 264;
	  section.data(170).dtTransOffset = 7348;
	
	  ;% rtP.MatrixGain_Gain
	  section.data(171).logicalSrcIdx = 265;
	  section.data(171).dtTransOffset = 7351;
	
	  ;% rtP.ThrustX_tableData
	  section.data(172).logicalSrcIdx = 266;
	  section.data(172).dtTransOffset = 7355;
	
	  ;% rtP.ThrustX_bp01Data
	  section.data(173).logicalSrcIdx = 267;
	  section.data(173).dtTransOffset = 7362;
	
	  ;% rtP.Thrust_YZ_Value
	  section.data(174).logicalSrcIdx = 268;
	  section.data(174).dtTransOffset = 7369;
	
	  ;% rtP.Constant6_Value
	  section.data(175).logicalSrcIdx = 269;
	  section.data(175).dtTransOffset = 7371;
	
	  ;% rtP.gamma_Value
	  section.data(176).logicalSrcIdx = 270;
	  section.data(176).dtTransOffset = 7372;
	
	  ;% rtP.one1_Value
	  section.data(177).logicalSrcIdx = 271;
	  section.data(177).dtTransOffset = 7373;
	
	  ;% rtP.const_Value
	  section.data(178).logicalSrcIdx = 272;
	  section.data(178).dtTransOffset = 7374;
	
	  ;% rtP.two_Value
	  section.data(179).logicalSrcIdx = 273;
	  section.data(179).dtTransOffset = 7375;
	
	  ;% rtP.one_Value
	  section.data(180).logicalSrcIdx = 274;
	  section.data(180).dtTransOffset = 7376;
	
	  ;% rtP.Po_Value
	  section.data(181).logicalSrcIdx = 275;
	  section.data(181).dtTransOffset = 7377;
	
	  ;% rtP.Constant_Value_iw00e14laz
	  section.data(182).logicalSrcIdx = 276;
	  section.data(182).dtTransOffset = 7378;
	
	  ;% rtP.Constant2_Value_nwrewbsrgx
	  section.data(183).logicalSrcIdx = 277;
	  section.data(183).dtTransOffset = 7379;
	
	  ;% rtP.seaLevelPstatic_Value
	  section.data(184).logicalSrcIdx = 278;
	  section.data(184).dtTransOffset = 7380;
	
	  ;% rtP.seaLevelSOS_Value
	  section.data(185).logicalSrcIdx = 279;
	  section.data(185).dtTransOffset = 7381;
	
	  ;% rtP.Constant2_Value_gr5f00mfnm
	  section.data(186).logicalSrcIdx = 280;
	  section.data(186).dtTransOffset = 7382;
	
	  ;% rtP.Constant_Value_n35gokrdfy
	  section.data(187).logicalSrcIdx = 281;
	  section.data(187).dtTransOffset = 7383;
	
	  ;% rtP.Constant1_Value_aypmrruauh
	  section.data(188).logicalSrcIdx = 282;
	  section.data(188).dtTransOffset = 7384;
	
	  ;% rtP.Constant12_Value
	  section.data(189).logicalSrcIdx = 283;
	  section.data(189).dtTransOffset = 7385;
	
	  ;% rtP.Constant2_Value_dp3hb0zdw4
	  section.data(190).logicalSrcIdx = 284;
	  section.data(190).dtTransOffset = 7386;
	
	  ;% rtP.Constant3_Value_dde35bhip2
	  section.data(191).logicalSrcIdx = 285;
	  section.data(191).dtTransOffset = 7387;
	
	  ;% rtP.Constant_Value_jdmivpanfl
	  section.data(192).logicalSrcIdx = 286;
	  section.data(192).dtTransOffset = 7388;
	
	  ;% rtP.Constant_Value_fwsa41mimo
	  section.data(193).logicalSrcIdx = 287;
	  section.data(193).dtTransOffset = 7389;
	
	  ;% rtP.Constant10_Value
	  section.data(194).logicalSrcIdx = 288;
	  section.data(194).dtTransOffset = 7390;
	
	  ;% rtP.Constant1_Value_n5qw4ehq0c
	  section.data(195).logicalSrcIdx = 289;
	  section.data(195).dtTransOffset = 7391;
	
	  ;% rtP.Constant1_Value_budxhsb1cr
	  section.data(196).logicalSrcIdx = 290;
	  section.data(196).dtTransOffset = 7392;
	
	  ;% rtP.Constant1_Value_nyr0ia51bl
	  section.data(197).logicalSrcIdx = 291;
	  section.data(197).dtTransOffset = 7393;
	
	  ;% rtP.zero1_Value
	  section.data(198).logicalSrcIdx = 292;
	  section.data(198).dtTransOffset = 7394;
	
	  ;% rtP.Constant_Value_olio5liftr
	  section.data(199).logicalSrcIdx = 293;
	  section.data(199).dtTransOffset = 7395;
	
	  ;% rtP.u_Value_pyce5cxhso
	  section.data(200).logicalSrcIdx = 294;
	  section.data(200).dtTransOffset = 7404;
	
	  ;% rtP.u_Value_nkmodb5shf
	  section.data(201).logicalSrcIdx = 295;
	  section.data(201).dtTransOffset = 7405;
	
	  ;% rtP.u_Value_a3c51jcqno
	  section.data(202).logicalSrcIdx = 296;
	  section.data(202).dtTransOffset = 7406;
	
	  ;% rtP.Constant_Value_lr2cknagpa
	  section.data(203).logicalSrcIdx = 297;
	  section.data(203).dtTransOffset = 7407;
	
	  ;% rtP.Constant1_Value_idys3ziitq
	  section.data(204).logicalSrcIdx = 298;
	  section.data(204).dtTransOffset = 7408;
	
	  ;% rtP.Constant2_Value_axatlcv2rs
	  section.data(205).logicalSrcIdx = 299;
	  section.data(205).dtTransOffset = 7409;
	
	  ;% rtP.Constant3_Value_ockepn4d1g
	  section.data(206).logicalSrcIdx = 300;
	  section.data(206).dtTransOffset = 7410;
	
	  ;% rtP.Constant4_Value_aprubsstdi
	  section.data(207).logicalSrcIdx = 301;
	  section.data(207).dtTransOffset = 7411;
	
	  ;% rtP.Constant_Value_mbry10wmmx
	  section.data(208).logicalSrcIdx = 302;
	  section.data(208).dtTransOffset = 7412;
	
	  ;% rtP.Constant1_Value_ishhczx4h3
	  section.data(209).logicalSrcIdx = 303;
	  section.data(209).dtTransOffset = 7413;
	
	  ;% rtP.Constant2_Value_bed4d3ljnh
	  section.data(210).logicalSrcIdx = 304;
	  section.data(210).dtTransOffset = 7414;
	
	  ;% rtP.Constant3_Value_l1ogwvmgck
	  section.data(211).logicalSrcIdx = 305;
	  section.data(211).dtTransOffset = 7415;
	
	  ;% rtP.Constant4_Value_env11uw4lz
	  section.data(212).logicalSrcIdx = 306;
	  section.data(212).dtTransOffset = 7416;
	
	  ;% rtP.Constant_Value_jkivyeh0ov
	  section.data(213).logicalSrcIdx = 307;
	  section.data(213).dtTransOffset = 7417;
	
	  ;% rtP.Constant1_Value_dttko5tnn2
	  section.data(214).logicalSrcIdx = 308;
	  section.data(214).dtTransOffset = 7418;
	
	  ;% rtP.Constant2_Value_d2teymf5hj
	  section.data(215).logicalSrcIdx = 309;
	  section.data(215).dtTransOffset = 7419;
	
	  ;% rtP.Constant3_Value_goletwl4mi
	  section.data(216).logicalSrcIdx = 310;
	  section.data(216).dtTransOffset = 7420;
	
	  ;% rtP.Constant4_Value_iqinklay2r
	  section.data(217).logicalSrcIdx = 311;
	  section.data(217).dtTransOffset = 7421;
	
	  ;% rtP.zero1_Value_jquxqq4qvz
	  section.data(218).logicalSrcIdx = 312;
	  section.data(218).dtTransOffset = 7422;
	
	  ;% rtP.zero3_Value
	  section.data(219).logicalSrcIdx = 313;
	  section.data(219).dtTransOffset = 7425;
	
	  ;% rtP.Constant_Value_nm4nmql4xz
	  section.data(220).logicalSrcIdx = 314;
	  section.data(220).dtTransOffset = 7428;
	
	  ;% rtP.zero_Value
	  section.data(221).logicalSrcIdx = 315;
	  section.data(221).dtTransOffset = 7429;
	
	  ;% rtP.Constant_Value_dk11ift0i1
	  section.data(222).logicalSrcIdx = 316;
	  section.data(222).dtTransOffset = 7430;
	
	  ;% rtP.Constant1_Value_e0wp0jqtlj
	  section.data(223).logicalSrcIdx = 317;
	  section.data(223).dtTransOffset = 7431;
	
	  ;% rtP.Constant2_Value_b5ilyky2la
	  section.data(224).logicalSrcIdx = 318;
	  section.data(224).dtTransOffset = 7432;
	
	  ;% rtP.Constant3_Value_d4mh0cif3h
	  section.data(225).logicalSrcIdx = 319;
	  section.data(225).dtTransOffset = 7433;
	
	  ;% rtP.Constant4_Value_bqgwz3qflq
	  section.data(226).logicalSrcIdx = 320;
	  section.data(226).dtTransOffset = 7434;
	
	  ;% rtP.Constant_Value_pzjjbfwyj5
	  section.data(227).logicalSrcIdx = 321;
	  section.data(227).dtTransOffset = 7435;
	
	  ;% rtP.Constant1_Value_fvdtk4fzi5
	  section.data(228).logicalSrcIdx = 322;
	  section.data(228).dtTransOffset = 7436;
	
	  ;% rtP.Constant2_Value_ms43idsvzx
	  section.data(229).logicalSrcIdx = 323;
	  section.data(229).dtTransOffset = 7437;
	
	  ;% rtP.Constant3_Value_mgyvggblh0
	  section.data(230).logicalSrcIdx = 324;
	  section.data(230).dtTransOffset = 7438;
	
	  ;% rtP.Constant4_Value_dv4oyrtdrl
	  section.data(231).logicalSrcIdx = 325;
	  section.data(231).dtTransOffset = 7439;
	
	  ;% rtP.Constant_Value_aagndz5zvc
	  section.data(232).logicalSrcIdx = 326;
	  section.data(232).dtTransOffset = 7440;
	
	  ;% rtP.Constant1_Value_cviw4ezr5s
	  section.data(233).logicalSrcIdx = 327;
	  section.data(233).dtTransOffset = 7441;
	
	  ;% rtP.Constant2_Value_faxwqrunex
	  section.data(234).logicalSrcIdx = 328;
	  section.data(234).dtTransOffset = 7442;
	
	  ;% rtP.Constant3_Value_hkckgaoniw
	  section.data(235).logicalSrcIdx = 329;
	  section.data(235).dtTransOffset = 7443;
	
	  ;% rtP.Constant4_Value_ctnx5cdtqt
	  section.data(236).logicalSrcIdx = 330;
	  section.data(236).dtTransOffset = 7444;
	
	  ;% rtP.Constant_Value_klve0yes4x
	  section.data(237).logicalSrcIdx = 331;
	  section.data(237).dtTransOffset = 7445;
	
	  ;% rtP.Constant_Value_gddg5y0bon
	  section.data(238).logicalSrcIdx = 332;
	  section.data(238).dtTransOffset = 7446;
	
	  ;% rtP.Constant1_Value_fdgj4x4yvk
	  section.data(239).logicalSrcIdx = 333;
	  section.data(239).dtTransOffset = 7447;
	
	  ;% rtP.Constant2_Value_g4cw5n3yku
	  section.data(240).logicalSrcIdx = 334;
	  section.data(240).dtTransOffset = 7448;
	
	  ;% rtP.Constant2_Value_f3imtyqlu2
	  section.data(241).logicalSrcIdx = 335;
	  section.data(241).dtTransOffset = 7449;
	
	  ;% rtP.Constant_Value_iatzfmuax5
	  section.data(242).logicalSrcIdx = 336;
	  section.data(242).dtTransOffset = 7450;
	
	  ;% rtP.Constant1_Value_jbkvuyvxyw
	  section.data(243).logicalSrcIdx = 337;
	  section.data(243).dtTransOffset = 7451;
	
	  ;% rtP.Bias_Bias_epi214vgpp
	  section.data(244).logicalSrcIdx = 338;
	  section.data(244).dtTransOffset = 7452;
	
	  ;% rtP.Constant2_Value_oe5zwfuhbf
	  section.data(245).logicalSrcIdx = 339;
	  section.data(245).dtTransOffset = 7453;
	
	  ;% rtP.Bias1_Bias_itbskgc1ah
	  section.data(246).logicalSrcIdx = 340;
	  section.data(246).dtTransOffset = 7454;
	
	  ;% rtP.Bias_Bias_aujzrf41ca
	  section.data(247).logicalSrcIdx = 341;
	  section.data(247).dtTransOffset = 7455;
	
	  ;% rtP.Gain_Gain_fmc2fxjyzt
	  section.data(248).logicalSrcIdx = 342;
	  section.data(248).dtTransOffset = 7456;
	
	  ;% rtP.Bias1_Bias_jirwvifxga
	  section.data(249).logicalSrcIdx = 343;
	  section.data(249).dtTransOffset = 7457;
	
	  ;% rtP.Bias_Bias_gall5umyqi
	  section.data(250).logicalSrcIdx = 344;
	  section.data(250).dtTransOffset = 7458;
	
	  ;% rtP.Constant2_Value_pkv2f4fmve
	  section.data(251).logicalSrcIdx = 345;
	  section.data(251).dtTransOffset = 7459;
	
	  ;% rtP.Bias1_Bias_g0mi2jjfta
	  section.data(252).logicalSrcIdx = 346;
	  section.data(252).dtTransOffset = 7460;
	
	  ;% rtP.Constant_Value_bf4fbrpvxe
	  section.data(253).logicalSrcIdx = 347;
	  section.data(253).dtTransOffset = 7461;
	
	  ;% rtP.Constant1_Value_hjtl2lfjek
	  section.data(254).logicalSrcIdx = 348;
	  section.data(254).dtTransOffset = 7462;
	
	  ;% rtP.Constant2_Value_dnwbrdkxjw
	  section.data(255).logicalSrcIdx = 349;
	  section.data(255).dtTransOffset = 7463;
	
	  ;% rtP.Constant3_Value_hlef1jp2hn
	  section.data(256).logicalSrcIdx = 350;
	  section.data(256).dtTransOffset = 7464;
	
	  ;% rtP.Constant_Value_ltgnw1q5be
	  section.data(257).logicalSrcIdx = 351;
	  section.data(257).dtTransOffset = 7465;
	
	  ;% rtP.Constant_Value_hdq4ys1wku
	  section.data(258).logicalSrcIdx = 352;
	  section.data(258).dtTransOffset = 7466;
	
	  ;% rtP.Constant_Value_ddl3nhb2x0
	  section.data(259).logicalSrcIdx = 353;
	  section.data(259).dtTransOffset = 7467;
	
	  ;% rtP.f_Value
	  section.data(260).logicalSrcIdx = 354;
	  section.data(260).dtTransOffset = 7468;
	
      nTotData = nTotData + section.nData;
      paramMap.sections(3) = section;
      clear section
      
      section.nData     = 20;
      section.data(20)  = dumData; %prealloc
      
	  ;% rtP.MediumHighAltitudeIntensity_maxIndex
	  section.data(1).logicalSrcIdx = 355;
	  section.data(1).dtTransOffset = 0;
	
	  ;% rtP.CD_dimSize
	  section.data(2).logicalSrcIdx = 356;
	  section.data(2).dtTransOffset = 2;
	
	  ;% rtP.CYb_dimSize
	  section.data(3).logicalSrcIdx = 357;
	  section.data(3).dtTransOffset = 5;
	
	  ;% rtP.CL_dimSize
	  section.data(4).logicalSrcIdx = 358;
	  section.data(4).dtTransOffset = 8;
	
	  ;% rtP.Clb_dimSize
	  section.data(5).logicalSrcIdx = 359;
	  section.data(5).dtTransOffset = 11;
	
	  ;% rtP.Cm_dimSize
	  section.data(6).logicalSrcIdx = 360;
	  section.data(6).dtTransOffset = 14;
	
	  ;% rtP.Cnb_dimSize
	  section.data(7).logicalSrcIdx = 361;
	  section.data(7).dtTransOffset = 17;
	
	  ;% rtP.CYp_dimSize
	  section.data(8).logicalSrcIdx = 362;
	  section.data(8).dtTransOffset = 20;
	
	  ;% rtP.CLad_dimSize
	  section.data(9).logicalSrcIdx = 363;
	  section.data(9).dtTransOffset = 23;
	
	  ;% rtP.CLq_dimSize
	  section.data(10).logicalSrcIdx = 364;
	  section.data(10).dtTransOffset = 26;
	
	  ;% rtP.Clp_dimSize
	  section.data(11).logicalSrcIdx = 365;
	  section.data(11).dtTransOffset = 29;
	
	  ;% rtP.Clr_dimSize
	  section.data(12).logicalSrcIdx = 366;
	  section.data(12).dtTransOffset = 32;
	
	  ;% rtP.Cmq_dimSize
	  section.data(13).logicalSrcIdx = 367;
	  section.data(13).dtTransOffset = 35;
	
	  ;% rtP.Cmad_dimSize
	  section.data(14).logicalSrcIdx = 368;
	  section.data(14).dtTransOffset = 38;
	
	  ;% rtP.Cnp_dimSize
	  section.data(15).logicalSrcIdx = 369;
	  section.data(15).dtTransOffset = 41;
	
	  ;% rtP.Cnr_dimSize
	  section.data(16).logicalSrcIdx = 370;
	  section.data(16).dtTransOffset = 44;
	
	  ;% rtP.DCDI_dimSize
	  section.data(17).logicalSrcIdx = 371;
	  section.data(17).dtTransOffset = 47;
	
	  ;% rtP.DCL_dimSize
	  section.data(18).logicalSrcIdx = 372;
	  section.data(18).dtTransOffset = 51;
	
	  ;% rtP.DCm_dimSize
	  section.data(19).logicalSrcIdx = 373;
	  section.data(19).dtTransOffset = 54;
	
	  ;% rtP.Xcp_dimSize
	  section.data(20).logicalSrcIdx = 374;
	  section.data(20).dtTransOffset = 57;
	
      nTotData = nTotData + section.nData;
      paramMap.sections(4) = section;
      clear section
      
      section.nData     = 7;
      section.data(7)  = dumData; %prealloc
      
	  ;% rtP.Packnet_fdmPacketforFlightGear_P1
	  section.data(1).logicalSrcIdx = 375;
	  section.data(1).dtTransOffset = 0;
	
	  ;% rtP.Packnet_fdmPacketforFlightGear_P2
	  section.data(2).logicalSrcIdx = 376;
	  section.data(2).dtTransOffset = 2;
	
	  ;% rtP.Packnet_fdmPacketforFlightGear_P3
	  section.data(3).logicalSrcIdx = 377;
	  section.data(3).dtTransOffset = 4;
	
	  ;% rtP.Packnet_fdmPacketforFlightGear_P4
	  section.data(4).logicalSrcIdx = 378;
	  section.data(4).dtTransOffset = 7;
	
	  ;% rtP.Packnet_fdmPacketforFlightGear_P5
	  section.data(5).logicalSrcIdx = 379;
	  section.data(5).dtTransOffset = 10;
	
	  ;% rtP.Packnet_fdmPacketforFlightGear_P6
	  section.data(6).logicalSrcIdx = 380;
	  section.data(6).dtTransOffset = 13;
	
	  ;% rtP.Packnet_fdmPacketforFlightGear_P7
	  section.data(7).logicalSrcIdx = 381;
	  section.data(7).dtTransOffset = 16;
	
      nTotData = nTotData + section.nData;
      paramMap.sections(5) = section;
      clear section
      
      section.nData     = 15;
      section.data(15)  = dumData; %prealloc
      
	  ;% rtP.aqq0sq0gty.M0_maxIndex
	  section.data(1).logicalSrcIdx = 382;
	  section.data(1).dtTransOffset = 0;
	
	  ;% rtP.aqq0sq0gty.M0_dimSizes
	  section.data(2).logicalSrcIdx = 383;
	  section.data(2).dtTransOffset = 3;
	
	  ;% rtP.aqq0sq0gty.M0_numYWorkElts
	  section.data(3).logicalSrcIdx = 384;
	  section.data(3).dtTransOffset = 6;
	
	  ;% rtP.aqq0sq0gty.M1_maxIndex
	  section.data(4).logicalSrcIdx = 385;
	  section.data(4).dtTransOffset = 10;
	
	  ;% rtP.aqq0sq0gty.M1_dimSizes
	  section.data(5).logicalSrcIdx = 386;
	  section.data(5).dtTransOffset = 13;
	
	  ;% rtP.aqq0sq0gty.M1_numYWorkElts
	  section.data(6).logicalSrcIdx = 387;
	  section.data(6).dtTransOffset = 16;
	
	  ;% rtP.aqq0sq0gty.M2_maxIndex
	  section.data(7).logicalSrcIdx = 388;
	  section.data(7).dtTransOffset = 20;
	
	  ;% rtP.aqq0sq0gty.M2_dimSizes
	  section.data(8).logicalSrcIdx = 389;
	  section.data(8).dtTransOffset = 23;
	
	  ;% rtP.aqq0sq0gty.M2_numYWorkElts
	  section.data(9).logicalSrcIdx = 390;
	  section.data(9).dtTransOffset = 26;
	
	  ;% rtP.aqq0sq0gty.M3_maxIndex
	  section.data(10).logicalSrcIdx = 391;
	  section.data(10).dtTransOffset = 30;
	
	  ;% rtP.aqq0sq0gty.M3_dimSizes
	  section.data(11).logicalSrcIdx = 392;
	  section.data(11).dtTransOffset = 33;
	
	  ;% rtP.aqq0sq0gty.M3_numYWorkElts
	  section.data(12).logicalSrcIdx = 393;
	  section.data(12).dtTransOffset = 36;
	
	  ;% rtP.aqq0sq0gty.M4_maxIndex
	  section.data(13).logicalSrcIdx = 394;
	  section.data(13).dtTransOffset = 40;
	
	  ;% rtP.aqq0sq0gty.M4_dimSizes
	  section.data(14).logicalSrcIdx = 395;
	  section.data(14).dtTransOffset = 43;
	
	  ;% rtP.aqq0sq0gty.M4_numYWorkElts
	  section.data(15).logicalSrcIdx = 396;
	  section.data(15).dtTransOffset = 46;
	
      nTotData = nTotData + section.nData;
      paramMap.sections(6) = section;
      clear section
      
      section.nData     = 15;
      section.data(15)  = dumData; %prealloc
      
	  ;% rtP.cjigmwyptgo.M0_maxIndex
	  section.data(1).logicalSrcIdx = 397;
	  section.data(1).dtTransOffset = 0;
	
	  ;% rtP.cjigmwyptgo.M0_dimSizes
	  section.data(2).logicalSrcIdx = 398;
	  section.data(2).dtTransOffset = 3;
	
	  ;% rtP.cjigmwyptgo.M0_numYWorkElts
	  section.data(3).logicalSrcIdx = 399;
	  section.data(3).dtTransOffset = 6;
	
	  ;% rtP.cjigmwyptgo.M1_maxIndex
	  section.data(4).logicalSrcIdx = 400;
	  section.data(4).dtTransOffset = 10;
	
	  ;% rtP.cjigmwyptgo.M1_dimSizes
	  section.data(5).logicalSrcIdx = 401;
	  section.data(5).dtTransOffset = 13;
	
	  ;% rtP.cjigmwyptgo.M1_numYWorkElts
	  section.data(6).logicalSrcIdx = 402;
	  section.data(6).dtTransOffset = 16;
	
	  ;% rtP.cjigmwyptgo.M2_maxIndex
	  section.data(7).logicalSrcIdx = 403;
	  section.data(7).dtTransOffset = 20;
	
	  ;% rtP.cjigmwyptgo.M2_dimSizes
	  section.data(8).logicalSrcIdx = 404;
	  section.data(8).dtTransOffset = 23;
	
	  ;% rtP.cjigmwyptgo.M2_numYWorkElts
	  section.data(9).logicalSrcIdx = 405;
	  section.data(9).dtTransOffset = 26;
	
	  ;% rtP.cjigmwyptgo.M3_maxIndex
	  section.data(10).logicalSrcIdx = 406;
	  section.data(10).dtTransOffset = 30;
	
	  ;% rtP.cjigmwyptgo.M3_dimSizes
	  section.data(11).logicalSrcIdx = 407;
	  section.data(11).dtTransOffset = 33;
	
	  ;% rtP.cjigmwyptgo.M3_numYWorkElts
	  section.data(12).logicalSrcIdx = 408;
	  section.data(12).dtTransOffset = 36;
	
	  ;% rtP.cjigmwyptgo.M4_maxIndex
	  section.data(13).logicalSrcIdx = 409;
	  section.data(13).dtTransOffset = 40;
	
	  ;% rtP.cjigmwyptgo.M4_dimSizes
	  section.data(14).logicalSrcIdx = 410;
	  section.data(14).dtTransOffset = 43;
	
	  ;% rtP.cjigmwyptgo.M4_numYWorkElts
	  section.data(15).logicalSrcIdx = 411;
	  section.data(15).dtTransOffset = 46;
	
      nTotData = nTotData + section.nData;
      paramMap.sections(7) = section;
      clear section
      
      section.nData     = 3;
      section.data(3)  = dumData; %prealloc
      
	  ;% rtP.i5wp2ffbry.x_Y0
	  section.data(1).logicalSrcIdx = 412;
	  section.data(1).dtTransOffset = 0;
	
	  ;% rtP.i5wp2ffbry.DistanceintoGustxLimitedtogustlengthd_IC
	  section.data(2).logicalSrcIdx = 413;
	  section.data(2).dtTransOffset = 1;
	
	  ;% rtP.i5wp2ffbry.DistanceintoGustxLimitedtogustlengthd_LowerSat
	  section.data(3).logicalSrcIdx = 414;
	  section.data(3).dtTransOffset = 2;
	
      nTotData = nTotData + section.nData;
      paramMap.sections(8) = section;
      clear section
      
      section.nData     = 3;
      section.data(3)  = dumData; %prealloc
      
	  ;% rtP.mirhnoymfvo.x_Y0
	  section.data(1).logicalSrcIdx = 415;
	  section.data(1).dtTransOffset = 0;
	
	  ;% rtP.mirhnoymfvo.DistanceintoGustxLimitedtogustlengthd_IC
	  section.data(2).logicalSrcIdx = 416;
	  section.data(2).dtTransOffset = 1;
	
	  ;% rtP.mirhnoymfvo.DistanceintoGustxLimitedtogustlengthd_LowerSat
	  section.data(3).logicalSrcIdx = 417;
	  section.data(3).dtTransOffset = 2;
	
      nTotData = nTotData + section.nData;
      paramMap.sections(9) = section;
      clear section
      
    
      ;%
      ;% Non-auto Data (parameter)
      ;%
    

    ;%
    ;% Add final counts to struct.
    ;%
    paramMap.nTotData = nTotData;
    


  ;%**************************
  ;% Create Block Output Map *
  ;%**************************
      
    nTotData      = 0; %add to this count as we go
    nTotSects     = 7;
    sectIdxOffset = 0;
    
    ;%
    ;% Define dummy sections & preallocate arrays
    ;%
    dumSection.nData = -1;  
    dumSection.data  = [];
    
    dumData.logicalSrcIdx = -1;
    dumData.dtTransOffset = -1;
    
    ;%
    ;% Init/prealloc sigMap
    ;%
    sigMap.nSections           = nTotSects;
    sigMap.sectIdxOffset       = sectIdxOffset;
      sigMap.sections(nTotSects) = dumSection; %prealloc
    sigMap.nTotData            = -1;
    
    ;%
    ;% Auto data (rtB)
    ;%
      section.nData     = 105;
      section.data(105)  = dumData; %prealloc
      
	  ;% rtB.pkgvmqtajm
	  section.data(1).logicalSrcIdx = 0;
	  section.data(1).dtTransOffset = 0;
	
	  ;% rtB.cbt1v4nio1
	  section.data(2).logicalSrcIdx = 1;
	  section.data(2).dtTransOffset = 2;
	
	  ;% rtB.chuqvl4sya
	  section.data(3).logicalSrcIdx = 2;
	  section.data(3).dtTransOffset = 4;
	
	  ;% rtB.hpaztoaur1
	  section.data(4).logicalSrcIdx = 3;
	  section.data(4).dtTransOffset = 5;
	
	  ;% rtB.pwdhurk11o
	  section.data(5).logicalSrcIdx = 4;
	  section.data(5).dtTransOffset = 6;
	
	  ;% rtB.akzllltikr
	  section.data(6).logicalSrcIdx = 5;
	  section.data(6).dtTransOffset = 7;
	
	  ;% rtB.pstvhythqv
	  section.data(7).logicalSrcIdx = 6;
	  section.data(7).dtTransOffset = 8;
	
	  ;% rtB.hjs5mprgdw
	  section.data(8).logicalSrcIdx = 7;
	  section.data(8).dtTransOffset = 9;
	
	  ;% rtB.ckkqbbvet5
	  section.data(9).logicalSrcIdx = 8;
	  section.data(9).dtTransOffset = 10;
	
	  ;% rtB.lauuzxpsuv
	  section.data(10).logicalSrcIdx = 9;
	  section.data(10).dtTransOffset = 11;
	
	  ;% rtB.kpfxhsrdzc
	  section.data(11).logicalSrcIdx = 10;
	  section.data(11).dtTransOffset = 12;
	
	  ;% rtB.d2gfe2p42l
	  section.data(12).logicalSrcIdx = 11;
	  section.data(12).dtTransOffset = 13;
	
	  ;% rtB.fdu3hdd1wq
	  section.data(13).logicalSrcIdx = 12;
	  section.data(13).dtTransOffset = 14;
	
	  ;% rtB.gr2e1rit1v
	  section.data(14).logicalSrcIdx = 13;
	  section.data(14).dtTransOffset = 15;
	
	  ;% rtB.jeyqu0v1xd
	  section.data(15).logicalSrcIdx = 14;
	  section.data(15).dtTransOffset = 17;
	
	  ;% rtB.erkqk25euy
	  section.data(16).logicalSrcIdx = 15;
	  section.data(16).dtTransOffset = 18;
	
	  ;% rtB.chm2baiq0o
	  section.data(17).logicalSrcIdx = 16;
	  section.data(17).dtTransOffset = 19;
	
	  ;% rtB.hevlxlfvxw
	  section.data(18).logicalSrcIdx = 17;
	  section.data(18).dtTransOffset = 20;
	
	  ;% rtB.j1pkkc3ac5
	  section.data(19).logicalSrcIdx = 18;
	  section.data(19).dtTransOffset = 21;
	
	  ;% rtB.iiwmf2tmrv
	  section.data(20).logicalSrcIdx = 19;
	  section.data(20).dtTransOffset = 22;
	
	  ;% rtB.onh5wf0mbq
	  section.data(21).logicalSrcIdx = 21;
	  section.data(21).dtTransOffset = 23;
	
	  ;% rtB.a4jldatvmm
	  section.data(22).logicalSrcIdx = 22;
	  section.data(22).dtTransOffset = 24;
	
	  ;% rtB.gectl3vjgg
	  section.data(23).logicalSrcIdx = 23;
	  section.data(23).dtTransOffset = 25;
	
	  ;% rtB.nrzu1kxyjs
	  section.data(24).logicalSrcIdx = 24;
	  section.data(24).dtTransOffset = 26;
	
	  ;% rtB.bf5sj4v0nw
	  section.data(25).logicalSrcIdx = 25;
	  section.data(25).dtTransOffset = 29;
	
	  ;% rtB.nw4xrcz5qc
	  section.data(26).logicalSrcIdx = 26;
	  section.data(26).dtTransOffset = 30;
	
	  ;% rtB.gtmxqvxrzk
	  section.data(27).logicalSrcIdx = 27;
	  section.data(27).dtTransOffset = 32;
	
	  ;% rtB.nazotke0rm
	  section.data(28).logicalSrcIdx = 28;
	  section.data(28).dtTransOffset = 34;
	
	  ;% rtB.pxggbjiorr
	  section.data(29).logicalSrcIdx = 29;
	  section.data(29).dtTransOffset = 36;
	
	  ;% rtB.kjlhrll14x
	  section.data(30).logicalSrcIdx = 30;
	  section.data(30).dtTransOffset = 37;
	
	  ;% rtB.jfhgj1iwfd
	  section.data(31).logicalSrcIdx = 31;
	  section.data(31).dtTransOffset = 38;
	
	  ;% rtB.c12vlekqly
	  section.data(32).logicalSrcIdx = 32;
	  section.data(32).dtTransOffset = 39;
	
	  ;% rtB.nf1iua4byr
	  section.data(33).logicalSrcIdx = 33;
	  section.data(33).dtTransOffset = 40;
	
	  ;% rtB.n0vzjwkja5
	  section.data(34).logicalSrcIdx = 34;
	  section.data(34).dtTransOffset = 41;
	
	  ;% rtB.cchzsu55nk
	  section.data(35).logicalSrcIdx = 35;
	  section.data(35).dtTransOffset = 42;
	
	  ;% rtB.chmbb41gxy
	  section.data(36).logicalSrcIdx = 36;
	  section.data(36).dtTransOffset = 43;
	
	  ;% rtB.dlbqs3jbq5
	  section.data(37).logicalSrcIdx = 37;
	  section.data(37).dtTransOffset = 44;
	
	  ;% rtB.a2xdtn2hjy
	  section.data(38).logicalSrcIdx = 38;
	  section.data(38).dtTransOffset = 45;
	
	  ;% rtB.kbkk0nabqt
	  section.data(39).logicalSrcIdx = 39;
	  section.data(39).dtTransOffset = 46;
	
	  ;% rtB.ohifllm5xd
	  section.data(40).logicalSrcIdx = 40;
	  section.data(40).dtTransOffset = 48;
	
	  ;% rtB.ot3vk2b455
	  section.data(41).logicalSrcIdx = 41;
	  section.data(41).dtTransOffset = 49;
	
	  ;% rtB.ddawuzzdzz
	  section.data(42).logicalSrcIdx = 42;
	  section.data(42).dtTransOffset = 52;
	
	  ;% rtB.mzb2t2xbjn
	  section.data(43).logicalSrcIdx = 43;
	  section.data(43).dtTransOffset = 61;
	
	  ;% rtB.exldqi4iq0
	  section.data(44).logicalSrcIdx = 44;
	  section.data(44).dtTransOffset = 64;
	
	  ;% rtB.eyzfoa2rgd
	  section.data(45).logicalSrcIdx = 45;
	  section.data(45).dtTransOffset = 65;
	
	  ;% rtB.knraoiqthj
	  section.data(46).logicalSrcIdx = 46;
	  section.data(46).dtTransOffset = 66;
	
	  ;% rtB.oxryesp4l3
	  section.data(47).logicalSrcIdx = 47;
	  section.data(47).dtTransOffset = 67;
	
	  ;% rtB.ktgttcsket
	  section.data(48).logicalSrcIdx = 48;
	  section.data(48).dtTransOffset = 68;
	
	  ;% rtB.dtrbet0r1l
	  section.data(49).logicalSrcIdx = 49;
	  section.data(49).dtTransOffset = 69;
	
	  ;% rtB.dojqnhucc1
	  section.data(50).logicalSrcIdx = 50;
	  section.data(50).dtTransOffset = 70;
	
	  ;% rtB.kkwizmpnnu
	  section.data(51).logicalSrcIdx = 51;
	  section.data(51).dtTransOffset = 71;
	
	  ;% rtB.p1ud1zl0l5
	  section.data(52).logicalSrcIdx = 52;
	  section.data(52).dtTransOffset = 72;
	
	  ;% rtB.h1os02q0lx
	  section.data(53).logicalSrcIdx = 53;
	  section.data(53).dtTransOffset = 73;
	
	  ;% rtB.di0bxuhii0
	  section.data(54).logicalSrcIdx = 54;
	  section.data(54).dtTransOffset = 74;
	
	  ;% rtB.fh1jzoqwae
	  section.data(55).logicalSrcIdx = 55;
	  section.data(55).dtTransOffset = 80;
	
	  ;% rtB.netnloma2z
	  section.data(56).logicalSrcIdx = 56;
	  section.data(56).dtTransOffset = 81;
	
	  ;% rtB.bz2agq32f5
	  section.data(57).logicalSrcIdx = 57;
	  section.data(57).dtTransOffset = 82;
	
	  ;% rtB.gakkrw1mku
	  section.data(58).logicalSrcIdx = 58;
	  section.data(58).dtTransOffset = 83;
	
	  ;% rtB.lldfszhlwc
	  section.data(59).logicalSrcIdx = 60;
	  section.data(59).dtTransOffset = 84;
	
	  ;% rtB.iqcro5zqc5
	  section.data(60).logicalSrcIdx = 61;
	  section.data(60).dtTransOffset = 85;
	
	  ;% rtB.khr4lzxr01
	  section.data(61).logicalSrcIdx = 62;
	  section.data(61).dtTransOffset = 86;
	
	  ;% rtB.otbeb0smep
	  section.data(62).logicalSrcIdx = 63;
	  section.data(62).dtTransOffset = 87;
	
	  ;% rtB.mbiqcrghxm
	  section.data(63).logicalSrcIdx = 64;
	  section.data(63).dtTransOffset = 88;
	
	  ;% rtB.gxcomph2oi
	  section.data(64).logicalSrcIdx = 65;
	  section.data(64).dtTransOffset = 89;
	
	  ;% rtB.pcxo4cuead
	  section.data(65).logicalSrcIdx = 67;
	  section.data(65).dtTransOffset = 90;
	
	  ;% rtB.euq0m2wkrk
	  section.data(66).logicalSrcIdx = 68;
	  section.data(66).dtTransOffset = 91;
	
	  ;% rtB.lzfl1px3we
	  section.data(67).logicalSrcIdx = 69;
	  section.data(67).dtTransOffset = 92;
	
	  ;% rtB.c2uwt5jnat
	  section.data(68).logicalSrcIdx = 70;
	  section.data(68).dtTransOffset = 93;
	
	  ;% rtB.e4lsqrzqjh
	  section.data(69).logicalSrcIdx = 71;
	  section.data(69).dtTransOffset = 94;
	
	  ;% rtB.j2oyzznxjz
	  section.data(70).logicalSrcIdx = 73;
	  section.data(70).dtTransOffset = 95;
	
	  ;% rtB.hkzk0mcpdz
	  section.data(71).logicalSrcIdx = 74;
	  section.data(71).dtTransOffset = 96;
	
	  ;% rtB.cjcp2dooxg
	  section.data(72).logicalSrcIdx = 75;
	  section.data(72).dtTransOffset = 97;
	
	  ;% rtB.ivvuf1kwbw
	  section.data(73).logicalSrcIdx = 76;
	  section.data(73).dtTransOffset = 98;
	
	  ;% rtB.dv0mxcbo4y
	  section.data(74).logicalSrcIdx = 77;
	  section.data(74).dtTransOffset = 99;
	
	  ;% rtB.nts3iqoscx
	  section.data(75).logicalSrcIdx = 78;
	  section.data(75).dtTransOffset = 100;
	
	  ;% rtB.lvp0utv5zh
	  section.data(76).logicalSrcIdx = 79;
	  section.data(76).dtTransOffset = 101;
	
	  ;% rtB.bs4j1k4omq
	  section.data(77).logicalSrcIdx = 80;
	  section.data(77).dtTransOffset = 102;
	
	  ;% rtB.nsbfkssthw
	  section.data(78).logicalSrcIdx = 81;
	  section.data(78).dtTransOffset = 103;
	
	  ;% rtB.gsfqejjcfo
	  section.data(79).logicalSrcIdx = 82;
	  section.data(79).dtTransOffset = 107;
	
	  ;% rtB.ewd50jnfc5
	  section.data(80).logicalSrcIdx = 83;
	  section.data(80).dtTransOffset = 108;
	
	  ;% rtB.pjsvxmsmf0
	  section.data(81).logicalSrcIdx = 84;
	  section.data(81).dtTransOffset = 109;
	
	  ;% rtB.pk4ybsmrx1
	  section.data(82).logicalSrcIdx = 85;
	  section.data(82).dtTransOffset = 110;
	
	  ;% rtB.bav3d1hobe
	  section.data(83).logicalSrcIdx = 86;
	  section.data(83).dtTransOffset = 111;
	
	  ;% rtB.o1nidekcl5
	  section.data(84).logicalSrcIdx = 87;
	  section.data(84).dtTransOffset = 112;
	
	  ;% rtB.pt0yyggmye
	  section.data(85).logicalSrcIdx = 88;
	  section.data(85).dtTransOffset = 113;
	
	  ;% rtB.dp0qd0ktnr
	  section.data(86).logicalSrcIdx = 89;
	  section.data(86).dtTransOffset = 114;
	
	  ;% rtB.fzxirwzcjx
	  section.data(87).logicalSrcIdx = 90;
	  section.data(87).dtTransOffset = 118;
	
	  ;% rtB.or3uazt4fw
	  section.data(88).logicalSrcIdx = 91;
	  section.data(88).dtTransOffset = 121;
	
	  ;% rtB.ntt1gar3xf
	  section.data(89).logicalSrcIdx = 92;
	  section.data(89).dtTransOffset = 122;
	
	  ;% rtB.mb51iyxxjy
	  section.data(90).logicalSrcIdx = 93;
	  section.data(90).dtTransOffset = 123;
	
	  ;% rtB.lxqebjchye
	  section.data(91).logicalSrcIdx = 94;
	  section.data(91).dtTransOffset = 124;
	
	  ;% rtB.ircgwytey3
	  section.data(92).logicalSrcIdx = 95;
	  section.data(92).dtTransOffset = 126;
	
	  ;% rtB.c5gx5xznq1
	  section.data(93).logicalSrcIdx = 96;
	  section.data(93).dtTransOffset = 128;
	
	  ;% rtB.kxuhls1vu4
	  section.data(94).logicalSrcIdx = 97;
	  section.data(94).dtTransOffset = 130;
	
	  ;% rtB.ckuncjpeqo
	  section.data(95).logicalSrcIdx = 98;
	  section.data(95).dtTransOffset = 131;
	
	  ;% rtB.gehuvldt5n
	  section.data(96).logicalSrcIdx = 99;
	  section.data(96).dtTransOffset = 133;
	
	  ;% rtB.k3touvbkw2
	  section.data(97).logicalSrcIdx = 100;
	  section.data(97).dtTransOffset = 135;
	
	  ;% rtB.fosv245qat
	  section.data(98).logicalSrcIdx = 101;
	  section.data(98).dtTransOffset = 137;
	
	  ;% rtB.leujf5c51t
	  section.data(99).logicalSrcIdx = 102;
	  section.data(99).dtTransOffset = 139;
	
	  ;% rtB.mv5ztdhgch
	  section.data(100).logicalSrcIdx = 103;
	  section.data(100).dtTransOffset = 141;
	
	  ;% rtB.agqbvy2aau
	  section.data(101).logicalSrcIdx = 104;
	  section.data(101).dtTransOffset = 143;
	
	  ;% rtB.f3yq1g0dcx
	  section.data(102).logicalSrcIdx = 106;
	  section.data(102).dtTransOffset = 145;
	
	  ;% rtB.plmewtggpi
	  section.data(103).logicalSrcIdx = 107;
	  section.data(103).dtTransOffset = 147;
	
	  ;% rtB.fffhvqsyn4
	  section.data(104).logicalSrcIdx = 108;
	  section.data(104).dtTransOffset = 148;
	
	  ;% rtB.ohglmemv1w
	  section.data(105).logicalSrcIdx = 109;
	  section.data(105).dtTransOffset = 149;
	
      nTotData = nTotData + section.nData;
      sigMap.sections(1) = section;
      clear section
      
      section.nData     = 1;
      section.data(1)  = dumData; %prealloc
      
	  ;% rtB.prdmace1ry
	  section.data(1).logicalSrcIdx = 110;
	  section.data(1).dtTransOffset = 0;
	
      nTotData = nTotData + section.nData;
      sigMap.sections(2) = section;
      clear section
      
      section.nData     = 3;
      section.data(3)  = dumData; %prealloc
      
	  ;% rtB.h1fm141rey
	  section.data(1).logicalSrcIdx = 111;
	  section.data(1).dtTransOffset = 0;
	
	  ;% rtB.hllvbobsss
	  section.data(2).logicalSrcIdx = 112;
	  section.data(2).dtTransOffset = 1;
	
	  ;% rtB.augghaqbk3
	  section.data(3).logicalSrcIdx = 113;
	  section.data(3).dtTransOffset = 2;
	
      nTotData = nTotData + section.nData;
      sigMap.sections(3) = section;
      clear section
      
      section.nData     = 1;
      section.data(1)  = dumData; %prealloc
      
	  ;% rtB.fcdyl4g5vg
	  section.data(1).logicalSrcIdx = 114;
	  section.data(1).dtTransOffset = 0;
	
      nTotData = nTotData + section.nData;
      sigMap.sections(4) = section;
      clear section
      
      section.nData     = 6;
      section.data(6)  = dumData; %prealloc
      
	  ;% rtB.oov0isvnf1
	  section.data(1).logicalSrcIdx = 115;
	  section.data(1).dtTransOffset = 0;
	
	  ;% rtB.beyrovclvb
	  section.data(2).logicalSrcIdx = 116;
	  section.data(2).dtTransOffset = 1;
	
	  ;% rtB.b5r5v125p5
	  section.data(3).logicalSrcIdx = 117;
	  section.data(3).dtTransOffset = 2;
	
	  ;% rtB.kulaji43pi
	  section.data(4).logicalSrcIdx = 120;
	  section.data(4).dtTransOffset = 3;
	
	  ;% rtB.drwselis2e
	  section.data(5).logicalSrcIdx = 121;
	  section.data(5).dtTransOffset = 4;
	
	  ;% rtB.pxhra0n40a
	  section.data(6).logicalSrcIdx = 122;
	  section.data(6).dtTransOffset = 5;
	
      nTotData = nTotData + section.nData;
      sigMap.sections(5) = section;
      clear section
      
      section.nData     = 1;
      section.data(1)  = dumData; %prealloc
      
	  ;% rtB.i5wp2ffbry.cwtrsfu0ve
	  section.data(1).logicalSrcIdx = 123;
	  section.data(1).dtTransOffset = 0;
	
      nTotData = nTotData + section.nData;
      sigMap.sections(6) = section;
      clear section
      
      section.nData     = 1;
      section.data(1)  = dumData; %prealloc
      
	  ;% rtB.mirhnoymfvo.cwtrsfu0ve
	  section.data(1).logicalSrcIdx = 124;
	  section.data(1).dtTransOffset = 0;
	
      nTotData = nTotData + section.nData;
      sigMap.sections(7) = section;
      clear section
      
    
      ;%
      ;% Non-auto Data (signal)
      ;%
    

    ;%
    ;% Add final counts to struct.
    ;%
    sigMap.nTotData = nTotData;
    


  ;%*******************
  ;% Create DWork Map *
  ;%*******************
      
    nTotData      = 0; %add to this count as we go
    nTotSects     = 24;
    sectIdxOffset = 7;
    
    ;%
    ;% Define dummy sections & preallocate arrays
    ;%
    dumSection.nData = -1;  
    dumSection.data  = [];
    
    dumData.logicalSrcIdx = -1;
    dumData.dtTransOffset = -1;
    
    ;%
    ;% Init/prealloc dworkMap
    ;%
    dworkMap.nSections           = nTotSects;
    dworkMap.sectIdxOffset       = sectIdxOffset;
      dworkMap.sections(nTotSects) = dumSection; %prealloc
    dworkMap.nTotData            = -1;
    
    ;%
    ;% Auto data (rtDW)
    ;%
      section.nData     = 12;
      section.data(12)  = dumData; %prealloc
      
	  ;% rtDW.dh0tof3kep
	  section.data(1).logicalSrcIdx = 0;
	  section.data(1).dtTransOffset = 0;
	
	  ;% rtDW.glj000y2ul
	  section.data(2).logicalSrcIdx = 1;
	  section.data(2).dtTransOffset = 2;
	
	  ;% rtDW.gyluhpuobq
	  section.data(3).logicalSrcIdx = 2;
	  section.data(3).dtTransOffset = 3;
	
	  ;% rtDW.hfu1jte3yo
	  section.data(4).logicalSrcIdx = 3;
	  section.data(4).dtTransOffset = 11;
	
	  ;% rtDW.hgabqtd4hg
	  section.data(5).logicalSrcIdx = 4;
	  section.data(5).dtTransOffset = 19;
	
	  ;% rtDW.lwgbxvspmn
	  section.data(6).logicalSrcIdx = 5;
	  section.data(6).dtTransOffset = 20;
	
	  ;% rtDW.btjxdj1nnm
	  section.data(7).logicalSrcIdx = 6;
	  section.data(7).dtTransOffset = 21;
	
	  ;% rtDW.foeszin0cg
	  section.data(8).logicalSrcIdx = 7;
	  section.data(8).dtTransOffset = 22;
	
	  ;% rtDW.e0khmaiz1x
	  section.data(9).logicalSrcIdx = 8;
	  section.data(9).dtTransOffset = 23;
	
	  ;% rtDW.csnixtyelx
	  section.data(10).logicalSrcIdx = 9;
	  section.data(10).dtTransOffset = 24;
	
	  ;% rtDW.aqckczktpc
	  section.data(11).logicalSrcIdx = 10;
	  section.data(11).dtTransOffset = 32;
	
	  ;% rtDW.joib1qjnje
	  section.data(12).logicalSrcIdx = 11;
	  section.data(12).dtTransOffset = 40;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(1) = section;
      clear section
      
      section.nData     = 6;
      section.data(6)  = dumData; %prealloc
      
	  ;% rtDW.jqnbqtb4sh.LoggedData
	  section.data(1).logicalSrcIdx = 12;
	  section.data(1).dtTransOffset = 0;
	
	  ;% rtDW.cygydsjrfb
	  section.data(2).logicalSrcIdx = 13;
	  section.data(2).dtTransOffset = 1;
	
	  ;% rtDW.gu0lptjayg
	  section.data(3).logicalSrcIdx = 14;
	  section.data(3).dtTransOffset = 2;
	
	  ;% rtDW.h4fprkvunp
	  section.data(4).logicalSrcIdx = 15;
	  section.data(4).dtTransOffset = 3;
	
	  ;% rtDW.ib0jqzpceq
	  section.data(5).logicalSrcIdx = 16;
	  section.data(5).dtTransOffset = 4;
	
	  ;% rtDW.fodolp1u3u
	  section.data(6).logicalSrcIdx = 17;
	  section.data(6).dtTransOffset = 5;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(2) = section;
      clear section
      
      section.nData     = 7;
      section.data(7)  = dumData; %prealloc
      
	  ;% rtDW.bu3swunyz2
	  section.data(1).logicalSrcIdx = 18;
	  section.data(1).dtTransOffset = 0;
	
	  ;% rtDW.eyrf2vyldh
	  section.data(2).logicalSrcIdx = 19;
	  section.data(2).dtTransOffset = 1;
	
	  ;% rtDW.nzxqnu5jmv
	  section.data(3).logicalSrcIdx = 20;
	  section.data(3).dtTransOffset = 2;
	
	  ;% rtDW.osz5yvmvo1
	  section.data(4).logicalSrcIdx = 21;
	  section.data(4).dtTransOffset = 3;
	
	  ;% rtDW.otuccisgv4
	  section.data(5).logicalSrcIdx = 22;
	  section.data(5).dtTransOffset = 4;
	
	  ;% rtDW.kz3kayz3v2
	  section.data(6).logicalSrcIdx = 23;
	  section.data(6).dtTransOffset = 5;
	
	  ;% rtDW.kjeb1ibj4p
	  section.data(7).logicalSrcIdx = 24;
	  section.data(7).dtTransOffset = 6;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(3) = section;
      clear section
      
      section.nData     = 4;
      section.data(4)  = dumData; %prealloc
      
	  ;% rtDW.mjfthrgmwy
	  section.data(1).logicalSrcIdx = 25;
	  section.data(1).dtTransOffset = 0;
	
	  ;% rtDW.ffudfy2unb
	  section.data(2).logicalSrcIdx = 26;
	  section.data(2).dtTransOffset = 1;
	
	  ;% rtDW.khqvqeq4d4
	  section.data(3).logicalSrcIdx = 27;
	  section.data(3).dtTransOffset = 2;
	
	  ;% rtDW.bxnoyp5430
	  section.data(4).logicalSrcIdx = 28;
	  section.data(4).dtTransOffset = 6;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(4) = section;
      clear section
      
      section.nData     = 17;
      section.data(17)  = dumData; %prealloc
      
	  ;% rtDW.afe03te4gm
	  section.data(1).logicalSrcIdx = 29;
	  section.data(1).dtTransOffset = 0;
	
	  ;% rtDW.bp1ctbtl0z
	  section.data(2).logicalSrcIdx = 30;
	  section.data(2).dtTransOffset = 1;
	
	  ;% rtDW.p0uqfawr0j
	  section.data(3).logicalSrcIdx = 31;
	  section.data(3).dtTransOffset = 2;
	
	  ;% rtDW.mff001ptpl
	  section.data(4).logicalSrcIdx = 32;
	  section.data(4).dtTransOffset = 3;
	
	  ;% rtDW.m5ksfibjla
	  section.data(5).logicalSrcIdx = 33;
	  section.data(5).dtTransOffset = 4;
	
	  ;% rtDW.li4wvvnvo0
	  section.data(6).logicalSrcIdx = 34;
	  section.data(6).dtTransOffset = 5;
	
	  ;% rtDW.bkbk0mdq0s
	  section.data(7).logicalSrcIdx = 35;
	  section.data(7).dtTransOffset = 6;
	
	  ;% rtDW.ogz3an5p1g
	  section.data(8).logicalSrcIdx = 36;
	  section.data(8).dtTransOffset = 7;
	
	  ;% rtDW.afsnxqrf1q
	  section.data(9).logicalSrcIdx = 37;
	  section.data(9).dtTransOffset = 8;
	
	  ;% rtDW.b02jv5hvxo
	  section.data(10).logicalSrcIdx = 38;
	  section.data(10).dtTransOffset = 9;
	
	  ;% rtDW.le0andlo2k
	  section.data(11).logicalSrcIdx = 39;
	  section.data(11).dtTransOffset = 10;
	
	  ;% rtDW.gofb51vunu
	  section.data(12).logicalSrcIdx = 40;
	  section.data(12).dtTransOffset = 11;
	
	  ;% rtDW.bzco1krw01
	  section.data(13).logicalSrcIdx = 41;
	  section.data(13).dtTransOffset = 12;
	
	  ;% rtDW.d0gx1hbrnk
	  section.data(14).logicalSrcIdx = 42;
	  section.data(14).dtTransOffset = 13;
	
	  ;% rtDW.m2dc0faool
	  section.data(15).logicalSrcIdx = 43;
	  section.data(15).dtTransOffset = 14;
	
	  ;% rtDW.n4rcpokqpc
	  section.data(16).logicalSrcIdx = 44;
	  section.data(16).dtTransOffset = 15;
	
	  ;% rtDW.hueaazkach
	  section.data(17).logicalSrcIdx = 45;
	  section.data(17).dtTransOffset = 16;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(5) = section;
      clear section
      
      section.nData     = 63;
      section.data(63)  = dumData; %prealloc
      
	  ;% rtDW.bh43qqaomf
	  section.data(1).logicalSrcIdx = 46;
	  section.data(1).dtTransOffset = 0;
	
	  ;% rtDW.cal4znigzs
	  section.data(2).logicalSrcIdx = 47;
	  section.data(2).dtTransOffset = 1;
	
	  ;% rtDW.o3p0d0pvrk
	  section.data(3).logicalSrcIdx = 48;
	  section.data(3).dtTransOffset = 2;
	
	  ;% rtDW.mxjlbo5rjs
	  section.data(4).logicalSrcIdx = 49;
	  section.data(4).dtTransOffset = 3;
	
	  ;% rtDW.hmyyl45jap
	  section.data(5).logicalSrcIdx = 50;
	  section.data(5).dtTransOffset = 4;
	
	  ;% rtDW.mdzj2hr4jc
	  section.data(6).logicalSrcIdx = 51;
	  section.data(6).dtTransOffset = 5;
	
	  ;% rtDW.fk0au12fsx
	  section.data(7).logicalSrcIdx = 52;
	  section.data(7).dtTransOffset = 6;
	
	  ;% rtDW.pa1f5cjxua
	  section.data(8).logicalSrcIdx = 53;
	  section.data(8).dtTransOffset = 7;
	
	  ;% rtDW.mewl43zgov
	  section.data(9).logicalSrcIdx = 54;
	  section.data(9).dtTransOffset = 8;
	
	  ;% rtDW.jutuodwsau
	  section.data(10).logicalSrcIdx = 55;
	  section.data(10).dtTransOffset = 9;
	
	  ;% rtDW.n12ashwpk5
	  section.data(11).logicalSrcIdx = 56;
	  section.data(11).dtTransOffset = 10;
	
	  ;% rtDW.poolnl3e1h
	  section.data(12).logicalSrcIdx = 57;
	  section.data(12).dtTransOffset = 11;
	
	  ;% rtDW.p4vcgbddhj
	  section.data(13).logicalSrcIdx = 58;
	  section.data(13).dtTransOffset = 12;
	
	  ;% rtDW.l3k54bxdj1
	  section.data(14).logicalSrcIdx = 59;
	  section.data(14).dtTransOffset = 13;
	
	  ;% rtDW.dd1fatoipk
	  section.data(15).logicalSrcIdx = 60;
	  section.data(15).dtTransOffset = 14;
	
	  ;% rtDW.dvxffgw44x
	  section.data(16).logicalSrcIdx = 61;
	  section.data(16).dtTransOffset = 15;
	
	  ;% rtDW.lfjjbagrjo
	  section.data(17).logicalSrcIdx = 62;
	  section.data(17).dtTransOffset = 16;
	
	  ;% rtDW.bvueotiv4b
	  section.data(18).logicalSrcIdx = 63;
	  section.data(18).dtTransOffset = 17;
	
	  ;% rtDW.hug50n4wps
	  section.data(19).logicalSrcIdx = 64;
	  section.data(19).dtTransOffset = 18;
	
	  ;% rtDW.i1eyrq4uwi
	  section.data(20).logicalSrcIdx = 65;
	  section.data(20).dtTransOffset = 19;
	
	  ;% rtDW.bfxedqpzaa
	  section.data(21).logicalSrcIdx = 66;
	  section.data(21).dtTransOffset = 20;
	
	  ;% rtDW.fc1bretpi5
	  section.data(22).logicalSrcIdx = 67;
	  section.data(22).dtTransOffset = 21;
	
	  ;% rtDW.fg4worawde
	  section.data(23).logicalSrcIdx = 68;
	  section.data(23).dtTransOffset = 22;
	
	  ;% rtDW.mxamlbyck0
	  section.data(24).logicalSrcIdx = 69;
	  section.data(24).dtTransOffset = 23;
	
	  ;% rtDW.n4ytmke31n
	  section.data(25).logicalSrcIdx = 70;
	  section.data(25).dtTransOffset = 24;
	
	  ;% rtDW.hudho2xp0v
	  section.data(26).logicalSrcIdx = 71;
	  section.data(26).dtTransOffset = 25;
	
	  ;% rtDW.glvphpzltx
	  section.data(27).logicalSrcIdx = 72;
	  section.data(27).dtTransOffset = 26;
	
	  ;% rtDW.giro4eplfi
	  section.data(28).logicalSrcIdx = 73;
	  section.data(28).dtTransOffset = 27;
	
	  ;% rtDW.fk3ir50wko
	  section.data(29).logicalSrcIdx = 74;
	  section.data(29).dtTransOffset = 28;
	
	  ;% rtDW.ojqxnfreck
	  section.data(30).logicalSrcIdx = 75;
	  section.data(30).dtTransOffset = 29;
	
	  ;% rtDW.g4vr3lujuz
	  section.data(31).logicalSrcIdx = 76;
	  section.data(31).dtTransOffset = 30;
	
	  ;% rtDW.drdj3xeohi
	  section.data(32).logicalSrcIdx = 77;
	  section.data(32).dtTransOffset = 31;
	
	  ;% rtDW.ngahvntjaq
	  section.data(33).logicalSrcIdx = 78;
	  section.data(33).dtTransOffset = 32;
	
	  ;% rtDW.axi4q1he0b
	  section.data(34).logicalSrcIdx = 79;
	  section.data(34).dtTransOffset = 33;
	
	  ;% rtDW.f1edxk4pdy
	  section.data(35).logicalSrcIdx = 80;
	  section.data(35).dtTransOffset = 34;
	
	  ;% rtDW.b3phozbfji
	  section.data(36).logicalSrcIdx = 81;
	  section.data(36).dtTransOffset = 35;
	
	  ;% rtDW.faa1lc2tvb
	  section.data(37).logicalSrcIdx = 82;
	  section.data(37).dtTransOffset = 36;
	
	  ;% rtDW.pdsijyzqws
	  section.data(38).logicalSrcIdx = 83;
	  section.data(38).dtTransOffset = 37;
	
	  ;% rtDW.dnxut4pgtq
	  section.data(39).logicalSrcIdx = 84;
	  section.data(39).dtTransOffset = 38;
	
	  ;% rtDW.oo4gabx5lo
	  section.data(40).logicalSrcIdx = 85;
	  section.data(40).dtTransOffset = 39;
	
	  ;% rtDW.jbrbgwy42h
	  section.data(41).logicalSrcIdx = 86;
	  section.data(41).dtTransOffset = 40;
	
	  ;% rtDW.nb15cebgib
	  section.data(42).logicalSrcIdx = 87;
	  section.data(42).dtTransOffset = 41;
	
	  ;% rtDW.fnjwuvlmmj
	  section.data(43).logicalSrcIdx = 88;
	  section.data(43).dtTransOffset = 42;
	
	  ;% rtDW.ijpkeyczg1
	  section.data(44).logicalSrcIdx = 89;
	  section.data(44).dtTransOffset = 43;
	
	  ;% rtDW.e25tjl33um
	  section.data(45).logicalSrcIdx = 90;
	  section.data(45).dtTransOffset = 44;
	
	  ;% rtDW.auligz0bp0
	  section.data(46).logicalSrcIdx = 91;
	  section.data(46).dtTransOffset = 45;
	
	  ;% rtDW.cvvf5wjoub
	  section.data(47).logicalSrcIdx = 92;
	  section.data(47).dtTransOffset = 46;
	
	  ;% rtDW.nv0y4bxbiz
	  section.data(48).logicalSrcIdx = 93;
	  section.data(48).dtTransOffset = 47;
	
	  ;% rtDW.jwt5flmp0h
	  section.data(49).logicalSrcIdx = 94;
	  section.data(49).dtTransOffset = 48;
	
	  ;% rtDW.pkklmgxn3w
	  section.data(50).logicalSrcIdx = 95;
	  section.data(50).dtTransOffset = 49;
	
	  ;% rtDW.ejm2q33rwd
	  section.data(51).logicalSrcIdx = 96;
	  section.data(51).dtTransOffset = 50;
	
	  ;% rtDW.fvgndvljcd
	  section.data(52).logicalSrcIdx = 97;
	  section.data(52).dtTransOffset = 51;
	
	  ;% rtDW.n5gzkaa0nd
	  section.data(53).logicalSrcIdx = 98;
	  section.data(53).dtTransOffset = 52;
	
	  ;% rtDW.gylng33jck
	  section.data(54).logicalSrcIdx = 99;
	  section.data(54).dtTransOffset = 53;
	
	  ;% rtDW.olplly2yli
	  section.data(55).logicalSrcIdx = 100;
	  section.data(55).dtTransOffset = 54;
	
	  ;% rtDW.gkuxqeqcdi
	  section.data(56).logicalSrcIdx = 101;
	  section.data(56).dtTransOffset = 55;
	
	  ;% rtDW.o2i2c0ygga
	  section.data(57).logicalSrcIdx = 102;
	  section.data(57).dtTransOffset = 56;
	
	  ;% rtDW.krzqaoxmqz
	  section.data(58).logicalSrcIdx = 103;
	  section.data(58).dtTransOffset = 57;
	
	  ;% rtDW.jddl4uj35t
	  section.data(59).logicalSrcIdx = 104;
	  section.data(59).dtTransOffset = 58;
	
	  ;% rtDW.adhcumf2n5
	  section.data(60).logicalSrcIdx = 105;
	  section.data(60).dtTransOffset = 59;
	
	  ;% rtDW.npachunt01
	  section.data(61).logicalSrcIdx = 106;
	  section.data(61).dtTransOffset = 60;
	
	  ;% rtDW.byzsjh0px3
	  section.data(62).logicalSrcIdx = 107;
	  section.data(62).dtTransOffset = 61;
	
	  ;% rtDW.au2zmey5pc
	  section.data(63).logicalSrcIdx = 108;
	  section.data(63).dtTransOffset = 62;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(6) = section;
      clear section
      
      section.nData     = 1;
      section.data(1)  = dumData; %prealloc
      
	  ;% rtDW.b0fymed3im
	  section.data(1).logicalSrcIdx = 109;
	  section.data(1).dtTransOffset = 0;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(7) = section;
      clear section
      
      section.nData     = 20;
      section.data(20)  = dumData; %prealloc
      
	  ;% rtDW.es24y1hork
	  section.data(1).logicalSrcIdx = 110;
	  section.data(1).dtTransOffset = 0;
	
	  ;% rtDW.iuswyxudvk
	  section.data(2).logicalSrcIdx = 111;
	  section.data(2).dtTransOffset = 1;
	
	  ;% rtDW.hk5aeqxoie
	  section.data(3).logicalSrcIdx = 112;
	  section.data(3).dtTransOffset = 2;
	
	  ;% rtDW.ex1irzrias
	  section.data(4).logicalSrcIdx = 113;
	  section.data(4).dtTransOffset = 3;
	
	  ;% rtDW.pldxv1lqbx
	  section.data(5).logicalSrcIdx = 114;
	  section.data(5).dtTransOffset = 4;
	
	  ;% rtDW.guox3atybt
	  section.data(6).logicalSrcIdx = 115;
	  section.data(6).dtTransOffset = 5;
	
	  ;% rtDW.jvi410hnvz
	  section.data(7).logicalSrcIdx = 116;
	  section.data(7).dtTransOffset = 6;
	
	  ;% rtDW.en1aw5kkit
	  section.data(8).logicalSrcIdx = 117;
	  section.data(8).dtTransOffset = 7;
	
	  ;% rtDW.kvd0keqbti
	  section.data(9).logicalSrcIdx = 118;
	  section.data(9).dtTransOffset = 8;
	
	  ;% rtDW.oq5swglxht
	  section.data(10).logicalSrcIdx = 119;
	  section.data(10).dtTransOffset = 9;
	
	  ;% rtDW.dvdytaltsj
	  section.data(11).logicalSrcIdx = 120;
	  section.data(11).dtTransOffset = 10;
	
	  ;% rtDW.jteqln0zcu
	  section.data(12).logicalSrcIdx = 121;
	  section.data(12).dtTransOffset = 11;
	
	  ;% rtDW.ouupu23dos
	  section.data(13).logicalSrcIdx = 122;
	  section.data(13).dtTransOffset = 12;
	
	  ;% rtDW.nztae4vpib
	  section.data(14).logicalSrcIdx = 123;
	  section.data(14).dtTransOffset = 13;
	
	  ;% rtDW.ahb01lfm54
	  section.data(15).logicalSrcIdx = 124;
	  section.data(15).dtTransOffset = 14;
	
	  ;% rtDW.aovs2hjxyi
	  section.data(16).logicalSrcIdx = 125;
	  section.data(16).dtTransOffset = 15;
	
	  ;% rtDW.bdjnrykkzo
	  section.data(17).logicalSrcIdx = 126;
	  section.data(17).dtTransOffset = 16;
	
	  ;% rtDW.auringonfr
	  section.data(18).logicalSrcIdx = 127;
	  section.data(18).dtTransOffset = 17;
	
	  ;% rtDW.ikp5xo5ucv
	  section.data(19).logicalSrcIdx = 128;
	  section.data(19).dtTransOffset = 18;
	
	  ;% rtDW.kk3mavtp0m
	  section.data(20).logicalSrcIdx = 129;
	  section.data(20).dtTransOffset = 19;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(8) = section;
      clear section
      
      section.nData     = 30;
      section.data(30)  = dumData; %prealloc
      
	  ;% rtDW.aqq0sq0gty.czibmh2e1r
	  section.data(1).logicalSrcIdx = 130;
	  section.data(1).dtTransOffset = 0;
	
	  ;% rtDW.aqq0sq0gty.bsnqpkum4j
	  section.data(2).logicalSrcIdx = 131;
	  section.data(2).dtTransOffset = 3;
	
	  ;% rtDW.aqq0sq0gty.ng2ho4wdw5
	  section.data(3).logicalSrcIdx = 132;
	  section.data(3).dtTransOffset = 3663;
	
	  ;% rtDW.aqq0sq0gty.emyp25k2ra
	  section.data(4).logicalSrcIdx = 133;
	  section.data(4).dtTransOffset = 7323;
	
	  ;% rtDW.aqq0sq0gty.ke3jfhfxg1
	  section.data(5).logicalSrcIdx = 134;
	  section.data(5).dtTransOffset = 10983;
	
	  ;% rtDW.aqq0sq0gty.c1uyo1uy5e
	  section.data(6).logicalSrcIdx = 135;
	  section.data(6).dtTransOffset = 11071;
	
	  ;% rtDW.aqq0sq0gty.l0xygmktrt
	  section.data(7).logicalSrcIdx = 136;
	  section.data(7).dtTransOffset = 333151;
	
	  ;% rtDW.aqq0sq0gty.ec4ih5lgh0
	  section.data(8).logicalSrcIdx = 137;
	  section.data(8).dtTransOffset = 333154;
	
	  ;% rtDW.aqq0sq0gty.hf0jovm0ds
	  section.data(9).logicalSrcIdx = 138;
	  section.data(9).dtTransOffset = 336814;
	
	  ;% rtDW.aqq0sq0gty.d2jzmsgxqy
	  section.data(10).logicalSrcIdx = 139;
	  section.data(10).dtTransOffset = 340474;
	
	  ;% rtDW.aqq0sq0gty.iel1urtae2
	  section.data(11).logicalSrcIdx = 140;
	  section.data(11).dtTransOffset = 344134;
	
	  ;% rtDW.aqq0sq0gty.mv4eqkyxzg
	  section.data(12).logicalSrcIdx = 141;
	  section.data(12).dtTransOffset = 344222;
	
	  ;% rtDW.aqq0sq0gty.o3sl5fyijx
	  section.data(13).logicalSrcIdx = 142;
	  section.data(13).dtTransOffset = 666302;
	
	  ;% rtDW.aqq0sq0gty.iownbpqjgp
	  section.data(14).logicalSrcIdx = 143;
	  section.data(14).dtTransOffset = 666305;
	
	  ;% rtDW.aqq0sq0gty.htq3wzf05k
	  section.data(15).logicalSrcIdx = 144;
	  section.data(15).dtTransOffset = 669965;
	
	  ;% rtDW.aqq0sq0gty.mhvuogwz5g
	  section.data(16).logicalSrcIdx = 145;
	  section.data(16).dtTransOffset = 673625;
	
	  ;% rtDW.aqq0sq0gty.e4boznuo11
	  section.data(17).logicalSrcIdx = 146;
	  section.data(17).dtTransOffset = 677285;
	
	  ;% rtDW.aqq0sq0gty.oltjiqpr2r
	  section.data(18).logicalSrcIdx = 147;
	  section.data(18).dtTransOffset = 677373;
	
	  ;% rtDW.aqq0sq0gty.kbkjlm3ow2
	  section.data(19).logicalSrcIdx = 148;
	  section.data(19).dtTransOffset = 999453;
	
	  ;% rtDW.aqq0sq0gty.gpo4oh4yoc
	  section.data(20).logicalSrcIdx = 149;
	  section.data(20).dtTransOffset = 999456;
	
	  ;% rtDW.aqq0sq0gty.lbrztkveec
	  section.data(21).logicalSrcIdx = 150;
	  section.data(21).dtTransOffset = 1003116;
	
	  ;% rtDW.aqq0sq0gty.hbisxcpsrg
	  section.data(22).logicalSrcIdx = 151;
	  section.data(22).dtTransOffset = 1006776;
	
	  ;% rtDW.aqq0sq0gty.maz32p5ch2
	  section.data(23).logicalSrcIdx = 152;
	  section.data(23).dtTransOffset = 1010436;
	
	  ;% rtDW.aqq0sq0gty.otcphh42tj
	  section.data(24).logicalSrcIdx = 153;
	  section.data(24).dtTransOffset = 1010524;
	
	  ;% rtDW.aqq0sq0gty.mucstbyucg
	  section.data(25).logicalSrcIdx = 154;
	  section.data(25).dtTransOffset = 1332604;
	
	  ;% rtDW.aqq0sq0gty.lvndzs1soq
	  section.data(26).logicalSrcIdx = 155;
	  section.data(26).dtTransOffset = 1332607;
	
	  ;% rtDW.aqq0sq0gty.pcfohnqayd
	  section.data(27).logicalSrcIdx = 156;
	  section.data(27).dtTransOffset = 1336267;
	
	  ;% rtDW.aqq0sq0gty.gvwl2w40zt
	  section.data(28).logicalSrcIdx = 157;
	  section.data(28).dtTransOffset = 1339927;
	
	  ;% rtDW.aqq0sq0gty.llvsxfyht1
	  section.data(29).logicalSrcIdx = 158;
	  section.data(29).dtTransOffset = 1343587;
	
	  ;% rtDW.aqq0sq0gty.c1emqgsxgf
	  section.data(30).logicalSrcIdx = 159;
	  section.data(30).dtTransOffset = 1343675;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(9) = section;
      clear section
      
      section.nData     = 15;
      section.data(15)  = dumData; %prealloc
      
	  ;% rtDW.aqq0sq0gty.js2t0wh1ap
	  section.data(1).logicalSrcIdx = 160;
	  section.data(1).dtTransOffset = 0;
	
	  ;% rtDW.aqq0sq0gty.dldelxht02
	  section.data(2).logicalSrcIdx = 161;
	  section.data(2).dtTransOffset = 3;
	
	  ;% rtDW.aqq0sq0gty.jvfpfqeupc
	  section.data(3).logicalSrcIdx = 162;
	  section.data(3).dtTransOffset = 9;
	
	  ;% rtDW.aqq0sq0gty.mxyuzcexca
	  section.data(4).logicalSrcIdx = 163;
	  section.data(4).dtTransOffset = 18;
	
	  ;% rtDW.aqq0sq0gty.eioxfitten
	  section.data(5).logicalSrcIdx = 164;
	  section.data(5).dtTransOffset = 21;
	
	  ;% rtDW.aqq0sq0gty.npajdwg2lh
	  section.data(6).logicalSrcIdx = 165;
	  section.data(6).dtTransOffset = 27;
	
	  ;% rtDW.aqq0sq0gty.czje14zbes
	  section.data(7).logicalSrcIdx = 166;
	  section.data(7).dtTransOffset = 36;
	
	  ;% rtDW.aqq0sq0gty.ddpi0aehhj
	  section.data(8).logicalSrcIdx = 167;
	  section.data(8).dtTransOffset = 39;
	
	  ;% rtDW.aqq0sq0gty.bjlmawntgi
	  section.data(9).logicalSrcIdx = 168;
	  section.data(9).dtTransOffset = 45;
	
	  ;% rtDW.aqq0sq0gty.czreq4mspp
	  section.data(10).logicalSrcIdx = 169;
	  section.data(10).dtTransOffset = 54;
	
	  ;% rtDW.aqq0sq0gty.eogjamfv2u
	  section.data(11).logicalSrcIdx = 170;
	  section.data(11).dtTransOffset = 57;
	
	  ;% rtDW.aqq0sq0gty.cughybvon0
	  section.data(12).logicalSrcIdx = 171;
	  section.data(12).dtTransOffset = 63;
	
	  ;% rtDW.aqq0sq0gty.hmckm540f5
	  section.data(13).logicalSrcIdx = 172;
	  section.data(13).dtTransOffset = 72;
	
	  ;% rtDW.aqq0sq0gty.k5z3ljbn5f
	  section.data(14).logicalSrcIdx = 173;
	  section.data(14).dtTransOffset = 75;
	
	  ;% rtDW.aqq0sq0gty.nzik33yqcn
	  section.data(15).logicalSrcIdx = 174;
	  section.data(15).dtTransOffset = 81;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(10) = section;
      clear section
      
      section.nData     = 5;
      section.data(5)  = dumData; %prealloc
      
	  ;% rtDW.aqq0sq0gty.pkty1tadnq
	  section.data(1).logicalSrcIdx = 175;
	  section.data(1).dtTransOffset = 0;
	
	  ;% rtDW.aqq0sq0gty.pr4m2g3lsg
	  section.data(2).logicalSrcIdx = 176;
	  section.data(2).dtTransOffset = 3;
	
	  ;% rtDW.aqq0sq0gty.fr1myblrol
	  section.data(3).logicalSrcIdx = 177;
	  section.data(3).dtTransOffset = 6;
	
	  ;% rtDW.aqq0sq0gty.li4rk12cor
	  section.data(4).logicalSrcIdx = 178;
	  section.data(4).dtTransOffset = 9;
	
	  ;% rtDW.aqq0sq0gty.bbfw2gdxcn
	  section.data(5).logicalSrcIdx = 179;
	  section.data(5).dtTransOffset = 12;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(11) = section;
      clear section
      
      section.nData     = 7;
      section.data(7)  = dumData; %prealloc
      
	  ;% rtDW.aqq0sq0gty.jijajg5nkb
	  section.data(1).logicalSrcIdx = 180;
	  section.data(1).dtTransOffset = 0;
	
	  ;% rtDW.aqq0sq0gty.gcr4x1ri1s
	  section.data(2).logicalSrcIdx = 181;
	  section.data(2).dtTransOffset = 1;
	
	  ;% rtDW.aqq0sq0gty.biatop040u
	  section.data(3).logicalSrcIdx = 182;
	  section.data(3).dtTransOffset = 2;
	
	  ;% rtDW.aqq0sq0gty.meuzevkn2y
	  section.data(4).logicalSrcIdx = 183;
	  section.data(4).dtTransOffset = 3;
	
	  ;% rtDW.aqq0sq0gty.ckxwrcsgi3
	  section.data(5).logicalSrcIdx = 184;
	  section.data(5).dtTransOffset = 4;
	
	  ;% rtDW.aqq0sq0gty.kglzzcbeka
	  section.data(6).logicalSrcIdx = 185;
	  section.data(6).dtTransOffset = 5;
	
	  ;% rtDW.aqq0sq0gty.johe0vk44j
	  section.data(7).logicalSrcIdx = 186;
	  section.data(7).dtTransOffset = 6;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(12) = section;
      clear section
      
      section.nData     = 5;
      section.data(5)  = dumData; %prealloc
      
	  ;% rtDW.aqq0sq0gty.dgmohvowou
	  section.data(1).logicalSrcIdx = 187;
	  section.data(1).dtTransOffset = 0;
	
	  ;% rtDW.aqq0sq0gty.nogzbokry4
	  section.data(2).logicalSrcIdx = 188;
	  section.data(2).dtTransOffset = 1;
	
	  ;% rtDW.aqq0sq0gty.dhoi4kxfar
	  section.data(3).logicalSrcIdx = 189;
	  section.data(3).dtTransOffset = 2;
	
	  ;% rtDW.aqq0sq0gty.czjauhqmhr
	  section.data(4).logicalSrcIdx = 190;
	  section.data(4).dtTransOffset = 3;
	
	  ;% rtDW.aqq0sq0gty.fg35qxl3vi
	  section.data(5).logicalSrcIdx = 191;
	  section.data(5).dtTransOffset = 4;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(13) = section;
      clear section
      
      section.nData     = 30;
      section.data(30)  = dumData; %prealloc
      
	  ;% rtDW.cjigmwyptgo.czibmh2e1r
	  section.data(1).logicalSrcIdx = 192;
	  section.data(1).dtTransOffset = 0;
	
	  ;% rtDW.cjigmwyptgo.bsnqpkum4j
	  section.data(2).logicalSrcIdx = 193;
	  section.data(2).dtTransOffset = 3;
	
	  ;% rtDW.cjigmwyptgo.ng2ho4wdw5
	  section.data(3).logicalSrcIdx = 194;
	  section.data(3).dtTransOffset = 3663;
	
	  ;% rtDW.cjigmwyptgo.emyp25k2ra
	  section.data(4).logicalSrcIdx = 195;
	  section.data(4).dtTransOffset = 7323;
	
	  ;% rtDW.cjigmwyptgo.ke3jfhfxg1
	  section.data(5).logicalSrcIdx = 196;
	  section.data(5).dtTransOffset = 10983;
	
	  ;% rtDW.cjigmwyptgo.c1uyo1uy5e
	  section.data(6).logicalSrcIdx = 197;
	  section.data(6).dtTransOffset = 11071;
	
	  ;% rtDW.cjigmwyptgo.l0xygmktrt
	  section.data(7).logicalSrcIdx = 198;
	  section.data(7).dtTransOffset = 333151;
	
	  ;% rtDW.cjigmwyptgo.ec4ih5lgh0
	  section.data(8).logicalSrcIdx = 199;
	  section.data(8).dtTransOffset = 333154;
	
	  ;% rtDW.cjigmwyptgo.hf0jovm0ds
	  section.data(9).logicalSrcIdx = 200;
	  section.data(9).dtTransOffset = 336814;
	
	  ;% rtDW.cjigmwyptgo.d2jzmsgxqy
	  section.data(10).logicalSrcIdx = 201;
	  section.data(10).dtTransOffset = 340474;
	
	  ;% rtDW.cjigmwyptgo.iel1urtae2
	  section.data(11).logicalSrcIdx = 202;
	  section.data(11).dtTransOffset = 344134;
	
	  ;% rtDW.cjigmwyptgo.mv4eqkyxzg
	  section.data(12).logicalSrcIdx = 203;
	  section.data(12).dtTransOffset = 344222;
	
	  ;% rtDW.cjigmwyptgo.o3sl5fyijx
	  section.data(13).logicalSrcIdx = 204;
	  section.data(13).dtTransOffset = 666302;
	
	  ;% rtDW.cjigmwyptgo.iownbpqjgp
	  section.data(14).logicalSrcIdx = 205;
	  section.data(14).dtTransOffset = 666305;
	
	  ;% rtDW.cjigmwyptgo.htq3wzf05k
	  section.data(15).logicalSrcIdx = 206;
	  section.data(15).dtTransOffset = 669965;
	
	  ;% rtDW.cjigmwyptgo.mhvuogwz5g
	  section.data(16).logicalSrcIdx = 207;
	  section.data(16).dtTransOffset = 673625;
	
	  ;% rtDW.cjigmwyptgo.e4boznuo11
	  section.data(17).logicalSrcIdx = 208;
	  section.data(17).dtTransOffset = 677285;
	
	  ;% rtDW.cjigmwyptgo.oltjiqpr2r
	  section.data(18).logicalSrcIdx = 209;
	  section.data(18).dtTransOffset = 677373;
	
	  ;% rtDW.cjigmwyptgo.kbkjlm3ow2
	  section.data(19).logicalSrcIdx = 210;
	  section.data(19).dtTransOffset = 999453;
	
	  ;% rtDW.cjigmwyptgo.gpo4oh4yoc
	  section.data(20).logicalSrcIdx = 211;
	  section.data(20).dtTransOffset = 999456;
	
	  ;% rtDW.cjigmwyptgo.lbrztkveec
	  section.data(21).logicalSrcIdx = 212;
	  section.data(21).dtTransOffset = 1003116;
	
	  ;% rtDW.cjigmwyptgo.hbisxcpsrg
	  section.data(22).logicalSrcIdx = 213;
	  section.data(22).dtTransOffset = 1006776;
	
	  ;% rtDW.cjigmwyptgo.maz32p5ch2
	  section.data(23).logicalSrcIdx = 214;
	  section.data(23).dtTransOffset = 1010436;
	
	  ;% rtDW.cjigmwyptgo.otcphh42tj
	  section.data(24).logicalSrcIdx = 215;
	  section.data(24).dtTransOffset = 1010524;
	
	  ;% rtDW.cjigmwyptgo.mucstbyucg
	  section.data(25).logicalSrcIdx = 216;
	  section.data(25).dtTransOffset = 1332604;
	
	  ;% rtDW.cjigmwyptgo.lvndzs1soq
	  section.data(26).logicalSrcIdx = 217;
	  section.data(26).dtTransOffset = 1332607;
	
	  ;% rtDW.cjigmwyptgo.pcfohnqayd
	  section.data(27).logicalSrcIdx = 218;
	  section.data(27).dtTransOffset = 1336267;
	
	  ;% rtDW.cjigmwyptgo.gvwl2w40zt
	  section.data(28).logicalSrcIdx = 219;
	  section.data(28).dtTransOffset = 1339927;
	
	  ;% rtDW.cjigmwyptgo.llvsxfyht1
	  section.data(29).logicalSrcIdx = 220;
	  section.data(29).dtTransOffset = 1343587;
	
	  ;% rtDW.cjigmwyptgo.c1emqgsxgf
	  section.data(30).logicalSrcIdx = 221;
	  section.data(30).dtTransOffset = 1343675;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(14) = section;
      clear section
      
      section.nData     = 15;
      section.data(15)  = dumData; %prealloc
      
	  ;% rtDW.cjigmwyptgo.js2t0wh1ap
	  section.data(1).logicalSrcIdx = 222;
	  section.data(1).dtTransOffset = 0;
	
	  ;% rtDW.cjigmwyptgo.dldelxht02
	  section.data(2).logicalSrcIdx = 223;
	  section.data(2).dtTransOffset = 3;
	
	  ;% rtDW.cjigmwyptgo.jvfpfqeupc
	  section.data(3).logicalSrcIdx = 224;
	  section.data(3).dtTransOffset = 9;
	
	  ;% rtDW.cjigmwyptgo.mxyuzcexca
	  section.data(4).logicalSrcIdx = 225;
	  section.data(4).dtTransOffset = 18;
	
	  ;% rtDW.cjigmwyptgo.eioxfitten
	  section.data(5).logicalSrcIdx = 226;
	  section.data(5).dtTransOffset = 21;
	
	  ;% rtDW.cjigmwyptgo.npajdwg2lh
	  section.data(6).logicalSrcIdx = 227;
	  section.data(6).dtTransOffset = 27;
	
	  ;% rtDW.cjigmwyptgo.czje14zbes
	  section.data(7).logicalSrcIdx = 228;
	  section.data(7).dtTransOffset = 36;
	
	  ;% rtDW.cjigmwyptgo.ddpi0aehhj
	  section.data(8).logicalSrcIdx = 229;
	  section.data(8).dtTransOffset = 39;
	
	  ;% rtDW.cjigmwyptgo.bjlmawntgi
	  section.data(9).logicalSrcIdx = 230;
	  section.data(9).dtTransOffset = 45;
	
	  ;% rtDW.cjigmwyptgo.czreq4mspp
	  section.data(10).logicalSrcIdx = 231;
	  section.data(10).dtTransOffset = 54;
	
	  ;% rtDW.cjigmwyptgo.eogjamfv2u
	  section.data(11).logicalSrcIdx = 232;
	  section.data(11).dtTransOffset = 57;
	
	  ;% rtDW.cjigmwyptgo.cughybvon0
	  section.data(12).logicalSrcIdx = 233;
	  section.data(12).dtTransOffset = 63;
	
	  ;% rtDW.cjigmwyptgo.hmckm540f5
	  section.data(13).logicalSrcIdx = 234;
	  section.data(13).dtTransOffset = 72;
	
	  ;% rtDW.cjigmwyptgo.k5z3ljbn5f
	  section.data(14).logicalSrcIdx = 235;
	  section.data(14).dtTransOffset = 75;
	
	  ;% rtDW.cjigmwyptgo.nzik33yqcn
	  section.data(15).logicalSrcIdx = 236;
	  section.data(15).dtTransOffset = 81;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(15) = section;
      clear section
      
      section.nData     = 5;
      section.data(5)  = dumData; %prealloc
      
	  ;% rtDW.cjigmwyptgo.pkty1tadnq
	  section.data(1).logicalSrcIdx = 237;
	  section.data(1).dtTransOffset = 0;
	
	  ;% rtDW.cjigmwyptgo.pr4m2g3lsg
	  section.data(2).logicalSrcIdx = 238;
	  section.data(2).dtTransOffset = 3;
	
	  ;% rtDW.cjigmwyptgo.fr1myblrol
	  section.data(3).logicalSrcIdx = 239;
	  section.data(3).dtTransOffset = 6;
	
	  ;% rtDW.cjigmwyptgo.li4rk12cor
	  section.data(4).logicalSrcIdx = 240;
	  section.data(4).dtTransOffset = 9;
	
	  ;% rtDW.cjigmwyptgo.bbfw2gdxcn
	  section.data(5).logicalSrcIdx = 241;
	  section.data(5).dtTransOffset = 12;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(16) = section;
      clear section
      
      section.nData     = 7;
      section.data(7)  = dumData; %prealloc
      
	  ;% rtDW.cjigmwyptgo.jijajg5nkb
	  section.data(1).logicalSrcIdx = 242;
	  section.data(1).dtTransOffset = 0;
	
	  ;% rtDW.cjigmwyptgo.gcr4x1ri1s
	  section.data(2).logicalSrcIdx = 243;
	  section.data(2).dtTransOffset = 1;
	
	  ;% rtDW.cjigmwyptgo.biatop040u
	  section.data(3).logicalSrcIdx = 244;
	  section.data(3).dtTransOffset = 2;
	
	  ;% rtDW.cjigmwyptgo.meuzevkn2y
	  section.data(4).logicalSrcIdx = 245;
	  section.data(4).dtTransOffset = 3;
	
	  ;% rtDW.cjigmwyptgo.ckxwrcsgi3
	  section.data(5).logicalSrcIdx = 246;
	  section.data(5).dtTransOffset = 4;
	
	  ;% rtDW.cjigmwyptgo.kglzzcbeka
	  section.data(6).logicalSrcIdx = 247;
	  section.data(6).dtTransOffset = 5;
	
	  ;% rtDW.cjigmwyptgo.johe0vk44j
	  section.data(7).logicalSrcIdx = 248;
	  section.data(7).dtTransOffset = 6;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(17) = section;
      clear section
      
      section.nData     = 5;
      section.data(5)  = dumData; %prealloc
      
	  ;% rtDW.cjigmwyptgo.dgmohvowou
	  section.data(1).logicalSrcIdx = 249;
	  section.data(1).dtTransOffset = 0;
	
	  ;% rtDW.cjigmwyptgo.nogzbokry4
	  section.data(2).logicalSrcIdx = 250;
	  section.data(2).dtTransOffset = 1;
	
	  ;% rtDW.cjigmwyptgo.dhoi4kxfar
	  section.data(3).logicalSrcIdx = 251;
	  section.data(3).dtTransOffset = 2;
	
	  ;% rtDW.cjigmwyptgo.czjauhqmhr
	  section.data(4).logicalSrcIdx = 252;
	  section.data(4).dtTransOffset = 3;
	
	  ;% rtDW.cjigmwyptgo.fg35qxl3vi
	  section.data(5).logicalSrcIdx = 253;
	  section.data(5).dtTransOffset = 4;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(18) = section;
      clear section
      
      section.nData     = 1;
      section.data(1)  = dumData; %prealloc
      
	  ;% rtDW.i5wp2ffbry.kaoqin3vcx
	  section.data(1).logicalSrcIdx = 254;
	  section.data(1).dtTransOffset = 0;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(19) = section;
      clear section
      
      section.nData     = 1;
      section.data(1)  = dumData; %prealloc
      
	  ;% rtDW.i5wp2ffbry.jyd1ppenco
	  section.data(1).logicalSrcIdx = 255;
	  section.data(1).dtTransOffset = 0;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(20) = section;
      clear section
      
      section.nData     = 1;
      section.data(1)  = dumData; %prealloc
      
	  ;% rtDW.i5wp2ffbry.ltfrc3b41y
	  section.data(1).logicalSrcIdx = 256;
	  section.data(1).dtTransOffset = 0;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(21) = section;
      clear section
      
      section.nData     = 1;
      section.data(1)  = dumData; %prealloc
      
	  ;% rtDW.mirhnoymfvo.kaoqin3vcx
	  section.data(1).logicalSrcIdx = 257;
	  section.data(1).dtTransOffset = 0;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(22) = section;
      clear section
      
      section.nData     = 1;
      section.data(1)  = dumData; %prealloc
      
	  ;% rtDW.mirhnoymfvo.jyd1ppenco
	  section.data(1).logicalSrcIdx = 258;
	  section.data(1).dtTransOffset = 0;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(23) = section;
      clear section
      
      section.nData     = 1;
      section.data(1)  = dumData; %prealloc
      
	  ;% rtDW.mirhnoymfvo.ltfrc3b41y
	  section.data(1).logicalSrcIdx = 259;
	  section.data(1).dtTransOffset = 0;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(24) = section;
      clear section
      
    
      ;%
      ;% Non-auto Data (dwork)
      ;%
    

    ;%
    ;% Add final counts to struct.
    ;%
    dworkMap.nTotData = nTotData;
    


  ;%
  ;% Add individual maps to base struct.
  ;%

  targMap.paramMap  = paramMap;    
  targMap.signalMap = sigMap;
  targMap.dworkMap  = dworkMap;
  
  ;%
  ;% Add checksums to base struct.
  ;%


  targMap.checksum0 = 3787883694;
  targMap.checksum1 = 3449449585;
  targMap.checksum2 = 185154508;
  targMap.checksum3 = 4030028274;

