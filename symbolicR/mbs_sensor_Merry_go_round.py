#
#	MBsysTran - Release 8.1
#
#	Copyright 
#	Universite catholique de Louvain (UCLouvain) 
#	Mechatronic, Electrical Energy, and Dynamic systems (MEED Division) 
#	2, Place du Levant
#	1348 Louvain-la-Neuve 
#	Belgium 
#
#	http://www.robotran.be 
#
#	==> Generation Date: Tue Apr  7 15:28:19 2026
#	==> using automatic loading with extension .mbs 
#
#	==> Project name: Merry_go_round
#
#	==> Number of joints: 19
#
#	==> Function: F6 - Sensors Kinematics
#
#	==> Git hash: 0e4e6a608eeee06956095d2f2ad315abdb092777
#
##

from math import sin, cos, sqrt

def sensor(sens, s, isens):
  q = s.q
  qd = s.qd
  qdd = s.qdd

  dpt = s.dpt
 
# Trigonometric functions

  S1 = sin(q[1])
  C1 = cos(q[1])
  S2 = sin(q[2])
  C2 = cos(q[2])
  S3 = sin(q[3])
  C3 = cos(q[3])
  S4 = sin(q[4])
  C4 = cos(q[4])
  S5 = sin(q[5])
  C5 = cos(q[5])
  S6 = sin(q[6])
  C6 = cos(q[6])
  S10 = sin(q[10])
  C10 = cos(q[10])
  S11 = sin(q[11])
  C11 = cos(q[11])
  S12 = sin(q[12])
  C12 = cos(q[12])
  S13 = sin(q[13])
  C13 = cos(q[13])
  S14 = sin(q[14])
  C14 = cos(q[14])
  S15 = sin(q[15])
  C15 = cos(q[15])
  S16 = sin(q[16])
  C16 = cos(q[16])
  S17 = sin(q[17])
  C17 = cos(q[17])
  S18 = sin(q[18])
  C18 = cos(q[18])
  S19 = sin(q[19])
  C19 = cos(q[19])
 
# Augmented Joint Position Vectors

 
# Sensor Kinematics


  if (isens == 1): 

    ROcp1_42 = -S1*C2
    ROcp1_52 = C1*C2
    ROcp1_72 = S1*S2
    ROcp1_82 = -C1*S2
    ROcp1_13 = -ROcp1_72*S3+C1*C3
    ROcp1_23 = -ROcp1_82*S3+S1*C3
    ROcp1_33 = -C2*S3
    ROcp1_73 = ROcp1_72*C3+C1*S3
    ROcp1_83 = ROcp1_82*C3+S1*S3
    ROcp1_93 = C2*C3
    ROcp1_14 = ROcp1_13*C4-ROcp1_73*S4
    ROcp1_24 = ROcp1_23*C4-ROcp1_83*S4
    ROcp1_34 = ROcp1_33*C4-ROcp1_93*S4
    ROcp1_74 = ROcp1_13*S4+ROcp1_73*C4
    ROcp1_84 = ROcp1_23*S4+ROcp1_83*C4
    ROcp1_94 = ROcp1_33*S4+ROcp1_93*C4
    ROcp1_15 = ROcp1_14*C5-ROcp1_74*S5
    ROcp1_25 = ROcp1_24*C5-ROcp1_84*S5
    ROcp1_35 = ROcp1_34*C5-ROcp1_94*S5
    ROcp1_75 = ROcp1_14*S5+ROcp1_74*C5
    ROcp1_85 = ROcp1_24*S5+ROcp1_84*C5
    ROcp1_95 = ROcp1_34*S5+ROcp1_94*C5
    ROcp1_46 = ROcp1_42*C6+ROcp1_75*S6
    ROcp1_56 = ROcp1_52*C6+ROcp1_85*S6
    ROcp1_66 = ROcp1_95*S6+S2*C6
    ROcp1_76 = -ROcp1_42*S6+ROcp1_75*C6
    ROcp1_86 = -ROcp1_52*S6+ROcp1_85*C6
    ROcp1_96 = ROcp1_95*C6-S2*S6
    OMcp1_12 = qd[2]*C1
    OMcp1_22 = qd[2]*S1
    OPcp1_12 = qdd[2]*C1-qd[1]*qd[2]*S1
    OPcp1_22 = qdd[2]*S1+qd[1]*qd[2]*C1
    OMcp1_13 = OMcp1_12+ROcp1_42*qd[3]
    OMcp1_23 = OMcp1_22+ROcp1_52*qd[3]
    OMcp1_33 = qd[1]+qd[3]*S2
    OPcp1_13 = OPcp1_12+ROcp1_42*qdd[3]+qd[3]*(OMcp1_22*S2-ROcp1_52*qd[1])
    OPcp1_23 = OPcp1_22+ROcp1_52*qdd[3]+qd[3]*(-OMcp1_12*S2+ROcp1_42*qd[1])
    OPcp1_33 = qdd[1]+qdd[3]*S2+qd[3]*(OMcp1_12*ROcp1_52-OMcp1_22*ROcp1_42)
    RLcp1_14 = ROcp1_13*s.dpt[1,1]+ROcp1_73*s.dpt[3,1]
    RLcp1_24 = ROcp1_23*s.dpt[1,1]+ROcp1_83*s.dpt[3,1]
    RLcp1_34 = ROcp1_33*s.dpt[1,1]+ROcp1_93*s.dpt[3,1]
    OMcp1_14 = OMcp1_13+ROcp1_42*qd[4]
    OMcp1_24 = OMcp1_23+ROcp1_52*qd[4]
    OMcp1_34 = OMcp1_33+qd[4]*S2
    ORcp1_14 = OMcp1_23*RLcp1_34-OMcp1_33*RLcp1_24
    ORcp1_24 = -OMcp1_13*RLcp1_34+OMcp1_33*RLcp1_14
    ORcp1_34 = OMcp1_13*RLcp1_24-OMcp1_23*RLcp1_14
    OPcp1_14 = OPcp1_13+ROcp1_42*qdd[4]+qd[4]*(OMcp1_23*S2-OMcp1_33*ROcp1_52)
    OPcp1_24 = OPcp1_23+ROcp1_52*qdd[4]+qd[4]*(-OMcp1_13*S2+OMcp1_33*ROcp1_42)
    OPcp1_34 = OPcp1_33+qdd[4]*S2+qd[4]*(OMcp1_13*ROcp1_52-OMcp1_23*ROcp1_42)
    ACcp1_14 = OMcp1_23*ORcp1_34-OMcp1_33*ORcp1_24+OPcp1_23*RLcp1_34-OPcp1_33*RLcp1_24
    ACcp1_24 = -OMcp1_13*ORcp1_34+OMcp1_33*ORcp1_14-OPcp1_13*RLcp1_34+OPcp1_33*RLcp1_14
    ACcp1_34 = OMcp1_13*ORcp1_24-OMcp1_23*ORcp1_14+OPcp1_13*RLcp1_24-OPcp1_23*RLcp1_14
    RLcp1_15 = ROcp1_74*s.dpt[3,5]
    RLcp1_25 = ROcp1_84*s.dpt[3,5]
    RLcp1_35 = ROcp1_94*s.dpt[3,5]
    POcp1_15 = RLcp1_14+RLcp1_15
    POcp1_25 = RLcp1_24+RLcp1_25
    POcp1_35 = RLcp1_34+RLcp1_35
    OMcp1_15 = OMcp1_14+ROcp1_42*qd[5]
    OMcp1_25 = OMcp1_24+ROcp1_52*qd[5]
    OMcp1_35 = OMcp1_34+qd[5]*S2
    ORcp1_15 = OMcp1_24*RLcp1_35-OMcp1_34*RLcp1_25
    ORcp1_25 = -OMcp1_14*RLcp1_35+OMcp1_34*RLcp1_15
    ORcp1_35 = OMcp1_14*RLcp1_25-OMcp1_24*RLcp1_15
    VIcp1_15 = ORcp1_14+ORcp1_15
    VIcp1_25 = ORcp1_24+ORcp1_25
    VIcp1_35 = ORcp1_34+ORcp1_35
    OPcp1_15 = OPcp1_14+ROcp1_42*qdd[5]+qd[5]*(OMcp1_24*S2-OMcp1_34*ROcp1_52)
    OPcp1_25 = OPcp1_24+ROcp1_52*qdd[5]+qd[5]*(-OMcp1_14*S2+OMcp1_34*ROcp1_42)
    OPcp1_35 = OPcp1_34+qdd[5]*S2+qd[5]*(OMcp1_14*ROcp1_52-OMcp1_24*ROcp1_42)
    ACcp1_15 = ACcp1_14+OMcp1_24*ORcp1_35-OMcp1_34*ORcp1_25+OPcp1_24*RLcp1_35-OPcp1_34*RLcp1_25
    ACcp1_25 = ACcp1_24-OMcp1_14*ORcp1_35+OMcp1_34*ORcp1_15-OPcp1_14*RLcp1_35+OPcp1_34*RLcp1_15
    ACcp1_35 = ACcp1_34+OMcp1_14*ORcp1_25-OMcp1_24*ORcp1_15+OPcp1_14*RLcp1_25-OPcp1_24*RLcp1_15
    OMcp1_16 = OMcp1_15+ROcp1_15*qd[6]
    OMcp1_26 = OMcp1_25+ROcp1_25*qd[6]
    OMcp1_36 = OMcp1_35+ROcp1_35*qd[6]
    OPcp1_16 = OPcp1_15+ROcp1_15*qdd[6]+qd[6]*(OMcp1_25*ROcp1_35-OMcp1_35*ROcp1_25)
    OPcp1_26 = OPcp1_25+ROcp1_25*qdd[6]+qd[6]*(-OMcp1_15*ROcp1_35+OMcp1_35*ROcp1_15)
    OPcp1_36 = OPcp1_35+ROcp1_35*qdd[6]+qd[6]*(OMcp1_15*ROcp1_25-OMcp1_25*ROcp1_15)
    RLcp1_17 = ROcp1_76*s.dpt[3,6]
    RLcp1_27 = ROcp1_86*s.dpt[3,6]
    RLcp1_37 = ROcp1_96*s.dpt[3,6]
    POcp1_17 = POcp1_15+RLcp1_17
    POcp1_27 = POcp1_25+RLcp1_27
    POcp1_37 = POcp1_35+RLcp1_37
    ORcp1_17 = OMcp1_26*RLcp1_37-OMcp1_36*RLcp1_27
    ORcp1_27 = -OMcp1_16*RLcp1_37+OMcp1_36*RLcp1_17
    ORcp1_37 = OMcp1_16*RLcp1_27-OMcp1_26*RLcp1_17
    VIcp1_17 = ORcp1_17+VIcp1_15
    VIcp1_27 = ORcp1_27+VIcp1_25
    VIcp1_37 = ORcp1_37+VIcp1_35
    ACcp1_17 = ACcp1_15+OMcp1_26*ORcp1_37-OMcp1_36*ORcp1_27+OPcp1_26*RLcp1_37-OPcp1_36*RLcp1_27
    ACcp1_27 = ACcp1_25-OMcp1_16*ORcp1_37+OMcp1_36*ORcp1_17-OPcp1_16*RLcp1_37+OPcp1_36*RLcp1_17
    ACcp1_37 = ACcp1_35+OMcp1_16*ORcp1_27-OMcp1_26*ORcp1_17+OPcp1_16*RLcp1_27-OPcp1_26*RLcp1_17
    sens.P[1] = POcp1_17
    sens.P[2] = POcp1_27
    sens.P[3] = POcp1_37
    sens.R[1,1] = ROcp1_15
    sens.R[1,2] = ROcp1_25
    sens.R[1,3] = ROcp1_35
    sens.R[2,1] = ROcp1_46
    sens.R[2,2] = ROcp1_56
    sens.R[2,3] = ROcp1_66
    sens.R[3,1] = ROcp1_76
    sens.R[3,2] = ROcp1_86
    sens.R[3,3] = ROcp1_96
    sens.V[1] = VIcp1_17
    sens.V[2] = VIcp1_27
    sens.V[3] = VIcp1_37
    sens.OM[1] = OMcp1_16
    sens.OM[2] = OMcp1_26
    sens.OM[3] = OMcp1_36
    sens.A[1] = ACcp1_17
    sens.A[2] = ACcp1_27
    sens.A[3] = ACcp1_37
    sens.OMP[1] = OPcp1_16
    sens.OMP[2] = OPcp1_26
    sens.OMP[3] = OPcp1_36

  if (isens == 2): 

    ROcp2_42 = -S1*C2
    ROcp2_52 = C1*C2
    ROcp2_72 = S1*S2
    ROcp2_82 = -C1*S2
    ROcp2_13 = -ROcp2_72*S3+C1*C3
    ROcp2_23 = -ROcp2_82*S3+S1*C3
    ROcp2_33 = -C2*S3
    ROcp2_73 = ROcp2_72*C3+C1*S3
    ROcp2_83 = ROcp2_82*C3+S1*S3
    ROcp2_93 = C2*C3
    ROcp2_110 = ROcp2_13*C10-ROcp2_73*S10
    ROcp2_210 = ROcp2_23*C10-ROcp2_83*S10
    ROcp2_310 = ROcp2_33*C10-ROcp2_93*S10
    ROcp2_710 = ROcp2_13*S10+ROcp2_73*C10
    ROcp2_810 = ROcp2_23*S10+ROcp2_83*C10
    ROcp2_910 = ROcp2_33*S10+ROcp2_93*C10
    ROcp2_411 = ROcp2_42*C11+ROcp2_710*S11
    ROcp2_511 = ROcp2_52*C11+ROcp2_810*S11
    ROcp2_611 = ROcp2_910*S11+C11*S2
    ROcp2_711 = -ROcp2_42*S11+ROcp2_710*C11
    ROcp2_811 = -ROcp2_52*S11+ROcp2_810*C11
    ROcp2_911 = ROcp2_910*C11-S11*S2
    ROcp2_412 = ROcp2_411*C12+ROcp2_711*S12
    ROcp2_512 = ROcp2_511*C12+ROcp2_811*S12
    ROcp2_612 = ROcp2_611*C12+ROcp2_911*S12
    ROcp2_712 = -ROcp2_411*S12+ROcp2_711*C12
    ROcp2_812 = -ROcp2_511*S12+ROcp2_811*C12
    ROcp2_912 = -ROcp2_611*S12+ROcp2_911*C12
    ROcp2_113 = ROcp2_110*C13-ROcp2_712*S13
    ROcp2_213 = ROcp2_210*C13-ROcp2_812*S13
    ROcp2_313 = ROcp2_310*C13-ROcp2_912*S13
    ROcp2_713 = ROcp2_110*S13+ROcp2_712*C13
    ROcp2_813 = ROcp2_210*S13+ROcp2_812*C13
    ROcp2_913 = ROcp2_310*S13+ROcp2_912*C13
    OMcp2_12 = qd[2]*C1
    OMcp2_22 = qd[2]*S1
    OPcp2_12 = qdd[2]*C1-qd[1]*qd[2]*S1
    OPcp2_22 = qdd[2]*S1+qd[1]*qd[2]*C1
    OMcp2_13 = OMcp2_12+ROcp2_42*qd[3]
    OMcp2_23 = OMcp2_22+ROcp2_52*qd[3]
    OMcp2_33 = qd[1]+qd[3]*S2
    OPcp2_13 = OPcp2_12+ROcp2_42*qdd[3]+qd[3]*(OMcp2_22*S2-ROcp2_52*qd[1])
    OPcp2_23 = OPcp2_22+ROcp2_52*qdd[3]+qd[3]*(-OMcp2_12*S2+ROcp2_42*qd[1])
    OPcp2_33 = qdd[1]+qdd[3]*S2+qd[3]*(OMcp2_12*ROcp2_52-OMcp2_22*ROcp2_42)
    RLcp2_14 = ROcp2_13*q[7]+ROcp2_73*s.dpt[3,2]
    RLcp2_24 = ROcp2_23*q[7]+ROcp2_83*s.dpt[3,2]
    RLcp2_34 = ROcp2_33*q[7]+ROcp2_93*s.dpt[3,2]
    ORcp2_14 = OMcp2_23*RLcp2_34-OMcp2_33*RLcp2_24
    ORcp2_24 = -OMcp2_13*RLcp2_34+OMcp2_33*RLcp2_14
    ORcp2_34 = OMcp2_13*RLcp2_24-OMcp2_23*RLcp2_14
    VIcp2_14 = ORcp2_14+ROcp2_13*qd[7]
    VIcp2_24 = ORcp2_24+ROcp2_23*qd[7]
    VIcp2_34 = ORcp2_34+ROcp2_33*qd[7]
    ACcp2_14 = OMcp2_23*ORcp2_34-OMcp2_33*ORcp2_24+OPcp2_23*RLcp2_34-OPcp2_33*RLcp2_24+ROcp2_13*qdd[7]+(2.0)*qd[7]*( \
 	  OMcp2_23*ROcp2_33-OMcp2_33*ROcp2_23)
    ACcp2_24 = -OMcp2_13*ORcp2_34+OMcp2_33*ORcp2_14-OPcp2_13*RLcp2_34+OPcp2_33*RLcp2_14+ROcp2_23*qdd[7]+(2.0)*qd[7]*(- \
 	  OMcp2_13*ROcp2_33+OMcp2_33*ROcp2_13)
    ACcp2_34 = OMcp2_13*ORcp2_24-OMcp2_23*ORcp2_14+OPcp2_13*RLcp2_24-OPcp2_23*RLcp2_14+ROcp2_33*qdd[7]+(2.0)*qd[7]*( \
 	  OMcp2_13*ROcp2_23-OMcp2_23*ROcp2_13)
    RLcp2_15 = ROcp2_13*q[8]+ROcp2_42*s.dpt[2,7]
    RLcp2_25 = ROcp2_23*q[8]+ROcp2_52*s.dpt[2,7]
    RLcp2_35 = ROcp2_33*q[8]+s.dpt[2,7]*S2
    POcp2_15 = RLcp2_14+RLcp2_15
    POcp2_25 = RLcp2_24+RLcp2_25
    POcp2_35 = RLcp2_34+RLcp2_35
    ORcp2_15 = OMcp2_23*RLcp2_35-OMcp2_33*RLcp2_25
    ORcp2_25 = -OMcp2_13*RLcp2_35+OMcp2_33*RLcp2_15
    ORcp2_35 = OMcp2_13*RLcp2_25-OMcp2_23*RLcp2_15
    VIcp2_15 = ORcp2_15+VIcp2_14+ROcp2_13*qd[8]
    VIcp2_25 = ORcp2_25+VIcp2_24+ROcp2_23*qd[8]
    VIcp2_35 = ORcp2_35+VIcp2_34+ROcp2_33*qd[8]
    ACcp2_15 = ACcp2_14+OMcp2_23*ORcp2_35-OMcp2_33*ORcp2_25+OPcp2_23*RLcp2_35-OPcp2_33*RLcp2_25+ROcp2_13*qdd[8]+(2.0)* \
 	  qd[8]*(OMcp2_23*ROcp2_33-OMcp2_33*ROcp2_23)
    ACcp2_25 = ACcp2_24-OMcp2_13*ORcp2_35+OMcp2_33*ORcp2_15-OPcp2_13*RLcp2_35+OPcp2_33*RLcp2_15+ROcp2_23*qdd[8]+(2.0)* \
 	  qd[8]*(-OMcp2_13*ROcp2_33+OMcp2_33*ROcp2_13)
    ACcp2_35 = ACcp2_34+OMcp2_13*ORcp2_25-OMcp2_23*ORcp2_15+OPcp2_13*RLcp2_25-OPcp2_23*RLcp2_15+ROcp2_33*qdd[8]+(2.0)* \
 	  qd[8]*(OMcp2_13*ROcp2_23-OMcp2_23*ROcp2_13)
    RLcp2_16 = ROcp2_73*q[9]
    RLcp2_26 = ROcp2_83*q[9]
    RLcp2_36 = ROcp2_93*q[9]
    POcp2_16 = POcp2_15+RLcp2_16
    POcp2_26 = POcp2_25+RLcp2_26
    POcp2_36 = POcp2_35+RLcp2_36
    ORcp2_16 = OMcp2_23*RLcp2_36-OMcp2_33*RLcp2_26
    ORcp2_26 = -OMcp2_13*RLcp2_36+OMcp2_33*RLcp2_16
    ORcp2_36 = OMcp2_13*RLcp2_26-OMcp2_23*RLcp2_16
    VIcp2_16 = ORcp2_16+VIcp2_15+ROcp2_73*qd[9]
    VIcp2_26 = ORcp2_26+VIcp2_25+ROcp2_83*qd[9]
    VIcp2_36 = ORcp2_36+VIcp2_35+ROcp2_93*qd[9]
    ACcp2_16 = ACcp2_15+OMcp2_23*ORcp2_36-OMcp2_33*ORcp2_26+OPcp2_23*RLcp2_36-OPcp2_33*RLcp2_26+ROcp2_73*qdd[9]+(2.0)* \
 	  qd[9]*(OMcp2_23*ROcp2_93-OMcp2_33*ROcp2_83)
    ACcp2_26 = ACcp2_25-OMcp2_13*ORcp2_36+OMcp2_33*ORcp2_16-OPcp2_13*RLcp2_36+OPcp2_33*RLcp2_16+ROcp2_83*qdd[9]+(2.0)* \
 	  qd[9]*(-OMcp2_13*ROcp2_93+OMcp2_33*ROcp2_73)
    ACcp2_36 = ACcp2_35+OMcp2_13*ORcp2_26-OMcp2_23*ORcp2_16+OPcp2_13*RLcp2_26-OPcp2_23*RLcp2_16+ROcp2_93*qdd[9]+(2.0)* \
 	  qd[9]*(OMcp2_13*ROcp2_83-OMcp2_23*ROcp2_73)
    OMcp2_17 = OMcp2_13+ROcp2_42*qd[10]
    OMcp2_27 = OMcp2_23+ROcp2_52*qd[10]
    OMcp2_37 = OMcp2_33+qd[10]*S2
    OPcp2_17 = OPcp2_13+ROcp2_42*qdd[10]+qd[10]*(OMcp2_23*S2-OMcp2_33*ROcp2_52)
    OPcp2_27 = OPcp2_23+ROcp2_52*qdd[10]+qd[10]*(-OMcp2_13*S2+OMcp2_33*ROcp2_42)
    OPcp2_37 = OPcp2_33+qdd[10]*S2+qd[10]*(OMcp2_13*ROcp2_52-OMcp2_23*ROcp2_42)
    OMcp2_18 = OMcp2_17+ROcp2_110*qd[11]
    OMcp2_28 = OMcp2_27+ROcp2_210*qd[11]
    OMcp2_38 = OMcp2_37+ROcp2_310*qd[11]
    OPcp2_18 = OPcp2_17+ROcp2_110*qdd[11]+qd[11]*(OMcp2_27*ROcp2_310-OMcp2_37*ROcp2_210)
    OPcp2_28 = OPcp2_27+ROcp2_210*qdd[11]+qd[11]*(-OMcp2_17*ROcp2_310+OMcp2_37*ROcp2_110)
    OPcp2_38 = OPcp2_37+ROcp2_310*qdd[11]+qd[11]*(OMcp2_17*ROcp2_210-OMcp2_27*ROcp2_110)
    RLcp2_19 = ROcp2_711*s.dpt[3,9]
    RLcp2_29 = ROcp2_811*s.dpt[3,9]
    RLcp2_39 = ROcp2_911*s.dpt[3,9]
    POcp2_19 = POcp2_16+RLcp2_19
    POcp2_29 = POcp2_26+RLcp2_29
    POcp2_39 = POcp2_36+RLcp2_39
    OMcp2_19 = OMcp2_18+ROcp2_110*qd[12]
    OMcp2_29 = OMcp2_28+ROcp2_210*qd[12]
    OMcp2_39 = OMcp2_38+ROcp2_310*qd[12]
    ORcp2_19 = OMcp2_28*RLcp2_39-OMcp2_38*RLcp2_29
    ORcp2_29 = -OMcp2_18*RLcp2_39+OMcp2_38*RLcp2_19
    ORcp2_39 = OMcp2_18*RLcp2_29-OMcp2_28*RLcp2_19
    VIcp2_19 = ORcp2_19+VIcp2_16
    VIcp2_29 = ORcp2_29+VIcp2_26
    VIcp2_39 = ORcp2_39+VIcp2_36
    OPcp2_19 = OPcp2_18+ROcp2_110*qdd[12]+qd[12]*(OMcp2_28*ROcp2_310-OMcp2_38*ROcp2_210)
    OPcp2_29 = OPcp2_28+ROcp2_210*qdd[12]+qd[12]*(-OMcp2_18*ROcp2_310+OMcp2_38*ROcp2_110)
    OPcp2_39 = OPcp2_38+ROcp2_310*qdd[12]+qd[12]*(OMcp2_18*ROcp2_210-OMcp2_28*ROcp2_110)
    ACcp2_19 = ACcp2_16+OMcp2_28*ORcp2_39-OMcp2_38*ORcp2_29+OPcp2_28*RLcp2_39-OPcp2_38*RLcp2_29
    ACcp2_29 = ACcp2_26-OMcp2_18*ORcp2_39+OMcp2_38*ORcp2_19-OPcp2_18*RLcp2_39+OPcp2_38*RLcp2_19
    ACcp2_39 = ACcp2_36+OMcp2_18*ORcp2_29-OMcp2_28*ORcp2_19+OPcp2_18*RLcp2_29-OPcp2_28*RLcp2_19
    OMcp2_110 = OMcp2_19+ROcp2_412*qd[13]
    OMcp2_210 = OMcp2_29+ROcp2_512*qd[13]
    OMcp2_310 = OMcp2_39+ROcp2_612*qd[13]
    OPcp2_110 = OPcp2_19+ROcp2_412*qdd[13]+qd[13]*(OMcp2_29*ROcp2_612-OMcp2_39*ROcp2_512)
    OPcp2_210 = OPcp2_29+ROcp2_512*qdd[13]+qd[13]*(-OMcp2_19*ROcp2_612+OMcp2_39*ROcp2_412)
    OPcp2_310 = OPcp2_39+ROcp2_612*qdd[13]+qd[13]*(OMcp2_19*ROcp2_512-OMcp2_29*ROcp2_412)
    RLcp2_111 = ROcp2_713*s.dpt[3,10]
    RLcp2_211 = ROcp2_813*s.dpt[3,10]
    RLcp2_311 = ROcp2_913*s.dpt[3,10]
    POcp2_111 = POcp2_19+RLcp2_111
    POcp2_211 = POcp2_29+RLcp2_211
    POcp2_311 = POcp2_39+RLcp2_311
    ORcp2_111 = OMcp2_210*RLcp2_311-OMcp2_310*RLcp2_211
    ORcp2_211 = -OMcp2_110*RLcp2_311+OMcp2_310*RLcp2_111
    ORcp2_311 = OMcp2_110*RLcp2_211-OMcp2_210*RLcp2_111
    VIcp2_111 = ORcp2_111+VIcp2_19
    VIcp2_211 = ORcp2_211+VIcp2_29
    VIcp2_311 = ORcp2_311+VIcp2_39
    ACcp2_111 = ACcp2_19+OMcp2_210*ORcp2_311-OMcp2_310*ORcp2_211+OPcp2_210*RLcp2_311-OPcp2_310*RLcp2_211
    ACcp2_211 = ACcp2_29-OMcp2_110*ORcp2_311+OMcp2_310*ORcp2_111-OPcp2_110*RLcp2_311+OPcp2_310*RLcp2_111
    ACcp2_311 = ACcp2_39+OMcp2_110*ORcp2_211-OMcp2_210*ORcp2_111+OPcp2_110*RLcp2_211-OPcp2_210*RLcp2_111
    sens.P[1] = POcp2_111
    sens.P[2] = POcp2_211
    sens.P[3] = POcp2_311
    sens.R[1,1] = ROcp2_113
    sens.R[1,2] = ROcp2_213
    sens.R[1,3] = ROcp2_313
    sens.R[2,1] = ROcp2_412
    sens.R[2,2] = ROcp2_512
    sens.R[2,3] = ROcp2_612
    sens.R[3,1] = ROcp2_713
    sens.R[3,2] = ROcp2_813
    sens.R[3,3] = ROcp2_913
    sens.V[1] = VIcp2_111
    sens.V[2] = VIcp2_211
    sens.V[3] = VIcp2_311
    sens.OM[1] = OMcp2_110
    sens.OM[2] = OMcp2_210
    sens.OM[3] = OMcp2_310
    sens.A[1] = ACcp2_111
    sens.A[2] = ACcp2_211
    sens.A[3] = ACcp2_311
    sens.OMP[1] = OPcp2_110
    sens.OMP[2] = OPcp2_210
    sens.OMP[3] = OPcp2_310

  if (isens == 3): 

    ROcp3_42 = -S1*C2
    ROcp3_52 = C1*C2
    ROcp3_72 = S1*S2
    ROcp3_82 = -C1*S2
    ROcp3_13 = -ROcp3_72*S3+C1*C3
    ROcp3_23 = -ROcp3_82*S3+S1*C3
    ROcp3_33 = -C2*S3
    ROcp3_73 = ROcp3_72*C3+C1*S3
    ROcp3_83 = ROcp3_82*C3+S1*S3
    ROcp3_93 = C2*C3
    ROcp3_114 = ROcp3_13*C14-ROcp3_73*S14
    ROcp3_214 = ROcp3_23*C14-ROcp3_83*S14
    ROcp3_314 = ROcp3_33*C14-ROcp3_93*S14
    ROcp3_714 = ROcp3_13*S14+ROcp3_73*C14
    ROcp3_814 = ROcp3_23*S14+ROcp3_83*C14
    ROcp3_914 = ROcp3_33*S14+ROcp3_93*C14
    ROcp3_115 = ROcp3_114*C15-ROcp3_714*S15
    ROcp3_215 = ROcp3_214*C15-ROcp3_814*S15
    ROcp3_315 = ROcp3_314*C15-ROcp3_914*S15
    ROcp3_715 = ROcp3_114*S15+ROcp3_714*C15
    ROcp3_815 = ROcp3_214*S15+ROcp3_814*C15
    ROcp3_915 = ROcp3_314*S15+ROcp3_914*C15
    ROcp3_416 = ROcp3_42*C16+ROcp3_715*S16
    ROcp3_516 = ROcp3_52*C16+ROcp3_815*S16
    ROcp3_616 = ROcp3_915*S16+C16*S2
    ROcp3_716 = -ROcp3_42*S16+ROcp3_715*C16
    ROcp3_816 = -ROcp3_52*S16+ROcp3_815*C16
    ROcp3_916 = ROcp3_915*C16-S16*S2
    OMcp3_12 = qd[2]*C1
    OMcp3_22 = qd[2]*S1
    OPcp3_12 = qdd[2]*C1-qd[1]*qd[2]*S1
    OPcp3_22 = qdd[2]*S1+qd[1]*qd[2]*C1
    OMcp3_13 = OMcp3_12+ROcp3_42*qd[3]
    OMcp3_23 = OMcp3_22+ROcp3_52*qd[3]
    OMcp3_33 = qd[1]+qd[3]*S2
    OPcp3_13 = OPcp3_12+ROcp3_42*qdd[3]+qd[3]*(OMcp3_22*S2-ROcp3_52*qd[1])
    OPcp3_23 = OPcp3_22+ROcp3_52*qdd[3]+qd[3]*(-OMcp3_12*S2+ROcp3_42*qd[1])
    OPcp3_33 = qdd[1]+qdd[3]*S2+qd[3]*(OMcp3_12*ROcp3_52-OMcp3_22*ROcp3_42)
    RLcp3_14 = ROcp3_13*s.dpt[1,3]+ROcp3_73*s.dpt[3,3]
    RLcp3_24 = ROcp3_23*s.dpt[1,3]+ROcp3_83*s.dpt[3,3]
    RLcp3_34 = ROcp3_33*s.dpt[1,3]+ROcp3_93*s.dpt[3,3]
    OMcp3_14 = OMcp3_13+ROcp3_42*qd[14]
    OMcp3_24 = OMcp3_23+ROcp3_52*qd[14]
    OMcp3_34 = OMcp3_33+qd[14]*S2
    ORcp3_14 = OMcp3_23*RLcp3_34-OMcp3_33*RLcp3_24
    ORcp3_24 = -OMcp3_13*RLcp3_34+OMcp3_33*RLcp3_14
    ORcp3_34 = OMcp3_13*RLcp3_24-OMcp3_23*RLcp3_14
    OPcp3_14 = OPcp3_13+ROcp3_42*qdd[14]+qd[14]*(OMcp3_23*S2-OMcp3_33*ROcp3_52)
    OPcp3_24 = OPcp3_23+ROcp3_52*qdd[14]+qd[14]*(-OMcp3_13*S2+OMcp3_33*ROcp3_42)
    OPcp3_34 = OPcp3_33+qdd[14]*S2+qd[14]*(OMcp3_13*ROcp3_52-OMcp3_23*ROcp3_42)
    ACcp3_14 = OMcp3_23*ORcp3_34-OMcp3_33*ORcp3_24+OPcp3_23*RLcp3_34-OPcp3_33*RLcp3_24
    ACcp3_24 = -OMcp3_13*ORcp3_34+OMcp3_33*ORcp3_14-OPcp3_13*RLcp3_34+OPcp3_33*RLcp3_14
    ACcp3_34 = OMcp3_13*ORcp3_24-OMcp3_23*ORcp3_14+OPcp3_13*RLcp3_24-OPcp3_23*RLcp3_14
    RLcp3_15 = ROcp3_714*s.dpt[3,11]
    RLcp3_25 = ROcp3_814*s.dpt[3,11]
    RLcp3_35 = ROcp3_914*s.dpt[3,11]
    POcp3_15 = RLcp3_14+RLcp3_15
    POcp3_25 = RLcp3_24+RLcp3_25
    POcp3_35 = RLcp3_34+RLcp3_35
    OMcp3_15 = OMcp3_14+ROcp3_42*qd[15]
    OMcp3_25 = OMcp3_24+ROcp3_52*qd[15]
    OMcp3_35 = OMcp3_34+qd[15]*S2
    ORcp3_15 = OMcp3_24*RLcp3_35-OMcp3_34*RLcp3_25
    ORcp3_25 = -OMcp3_14*RLcp3_35+OMcp3_34*RLcp3_15
    ORcp3_35 = OMcp3_14*RLcp3_25-OMcp3_24*RLcp3_15
    VIcp3_15 = ORcp3_14+ORcp3_15
    VIcp3_25 = ORcp3_24+ORcp3_25
    VIcp3_35 = ORcp3_34+ORcp3_35
    OPcp3_15 = OPcp3_14+ROcp3_42*qdd[15]+qd[15]*(OMcp3_24*S2-OMcp3_34*ROcp3_52)
    OPcp3_25 = OPcp3_24+ROcp3_52*qdd[15]+qd[15]*(-OMcp3_14*S2+OMcp3_34*ROcp3_42)
    OPcp3_35 = OPcp3_34+qdd[15]*S2+qd[15]*(OMcp3_14*ROcp3_52-OMcp3_24*ROcp3_42)
    ACcp3_15 = ACcp3_14+OMcp3_24*ORcp3_35-OMcp3_34*ORcp3_25+OPcp3_24*RLcp3_35-OPcp3_34*RLcp3_25
    ACcp3_25 = ACcp3_24-OMcp3_14*ORcp3_35+OMcp3_34*ORcp3_15-OPcp3_14*RLcp3_35+OPcp3_34*RLcp3_15
    ACcp3_35 = ACcp3_34+OMcp3_14*ORcp3_25-OMcp3_24*ORcp3_15+OPcp3_14*RLcp3_25-OPcp3_24*RLcp3_15
    OMcp3_16 = OMcp3_15+ROcp3_115*qd[16]
    OMcp3_26 = OMcp3_25+ROcp3_215*qd[16]
    OMcp3_36 = OMcp3_35+ROcp3_315*qd[16]
    OPcp3_16 = OPcp3_15+ROcp3_115*qdd[16]+qd[16]*(OMcp3_25*ROcp3_315-OMcp3_35*ROcp3_215)
    OPcp3_26 = OPcp3_25+ROcp3_215*qdd[16]+qd[16]*(-OMcp3_15*ROcp3_315+OMcp3_35*ROcp3_115)
    OPcp3_36 = OPcp3_35+ROcp3_315*qdd[16]+qd[16]*(OMcp3_15*ROcp3_215-OMcp3_25*ROcp3_115)
    RLcp3_17 = ROcp3_716*s.dpt[3,12]
    RLcp3_27 = ROcp3_816*s.dpt[3,12]
    RLcp3_37 = ROcp3_916*s.dpt[3,12]
    POcp3_17 = POcp3_15+RLcp3_17
    POcp3_27 = POcp3_25+RLcp3_27
    POcp3_37 = POcp3_35+RLcp3_37
    ORcp3_17 = OMcp3_26*RLcp3_37-OMcp3_36*RLcp3_27
    ORcp3_27 = -OMcp3_16*RLcp3_37+OMcp3_36*RLcp3_17
    ORcp3_37 = OMcp3_16*RLcp3_27-OMcp3_26*RLcp3_17
    VIcp3_17 = ORcp3_17+VIcp3_15
    VIcp3_27 = ORcp3_27+VIcp3_25
    VIcp3_37 = ORcp3_37+VIcp3_35
    ACcp3_17 = ACcp3_15+OMcp3_26*ORcp3_37-OMcp3_36*ORcp3_27+OPcp3_26*RLcp3_37-OPcp3_36*RLcp3_27
    ACcp3_27 = ACcp3_25-OMcp3_16*ORcp3_37+OMcp3_36*ORcp3_17-OPcp3_16*RLcp3_37+OPcp3_36*RLcp3_17
    ACcp3_37 = ACcp3_35+OMcp3_16*ORcp3_27-OMcp3_26*ORcp3_17+OPcp3_16*RLcp3_27-OPcp3_26*RLcp3_17
    sens.P[1] = POcp3_17
    sens.P[2] = POcp3_27
    sens.P[3] = POcp3_37
    sens.R[1,1] = ROcp3_115
    sens.R[1,2] = ROcp3_215
    sens.R[1,3] = ROcp3_315
    sens.R[2,1] = ROcp3_416
    sens.R[2,2] = ROcp3_516
    sens.R[2,3] = ROcp3_616
    sens.R[3,1] = ROcp3_716
    sens.R[3,2] = ROcp3_816
    sens.R[3,3] = ROcp3_916
    sens.V[1] = VIcp3_17
    sens.V[2] = VIcp3_27
    sens.V[3] = VIcp3_37
    sens.OM[1] = OMcp3_16
    sens.OM[2] = OMcp3_26
    sens.OM[3] = OMcp3_36
    sens.A[1] = ACcp3_17
    sens.A[2] = ACcp3_27
    sens.A[3] = ACcp3_37
    sens.OMP[1] = OPcp3_16
    sens.OMP[2] = OPcp3_26
    sens.OMP[3] = OPcp3_36

  if (isens == 4): 

    ROcp4_42 = -S1*C2
    ROcp4_52 = C1*C2
    ROcp4_72 = S1*S2
    ROcp4_82 = -C1*S2
    ROcp4_13 = -ROcp4_72*S3+C1*C3
    ROcp4_23 = -ROcp4_82*S3+S1*C3
    ROcp4_33 = -C2*S3
    ROcp4_73 = ROcp4_72*C3+C1*S3
    ROcp4_83 = ROcp4_82*C3+S1*S3
    ROcp4_93 = C2*C3
    ROcp4_417 = ROcp4_42*C17+ROcp4_73*S17
    ROcp4_517 = ROcp4_52*C17+ROcp4_83*S17
    ROcp4_617 = ROcp4_93*S17+C17*S2
    ROcp4_717 = -ROcp4_42*S17+ROcp4_73*C17
    ROcp4_817 = -ROcp4_52*S17+ROcp4_83*C17
    ROcp4_917 = ROcp4_93*C17-S17*S2
    ROcp4_418 = ROcp4_417*C18+ROcp4_717*S18
    ROcp4_518 = ROcp4_517*C18+ROcp4_817*S18
    ROcp4_618 = ROcp4_617*C18+ROcp4_917*S18
    ROcp4_718 = -ROcp4_417*S18+ROcp4_717*C18
    ROcp4_818 = -ROcp4_517*S18+ROcp4_817*C18
    ROcp4_918 = -ROcp4_617*S18+ROcp4_917*C18
    ROcp4_119 = ROcp4_13*C19-ROcp4_718*S19
    ROcp4_219 = ROcp4_23*C19-ROcp4_818*S19
    ROcp4_319 = ROcp4_33*C19-ROcp4_918*S19
    ROcp4_719 = ROcp4_13*S19+ROcp4_718*C19
    ROcp4_819 = ROcp4_23*S19+ROcp4_818*C19
    ROcp4_919 = ROcp4_33*S19+ROcp4_918*C19
    OMcp4_12 = qd[2]*C1
    OMcp4_22 = qd[2]*S1
    OPcp4_12 = qdd[2]*C1-qd[1]*qd[2]*S1
    OPcp4_22 = qdd[2]*S1+qd[1]*qd[2]*C1
    OMcp4_13 = OMcp4_12+ROcp4_42*qd[3]
    OMcp4_23 = OMcp4_22+ROcp4_52*qd[3]
    OMcp4_33 = qd[1]+qd[3]*S2
    OPcp4_13 = OPcp4_12+ROcp4_42*qdd[3]+qd[3]*(OMcp4_22*S2-ROcp4_52*qd[1])
    OPcp4_23 = OPcp4_22+ROcp4_52*qdd[3]+qd[3]*(-OMcp4_12*S2+ROcp4_42*qd[1])
    OPcp4_33 = qdd[1]+qdd[3]*S2+qd[3]*(OMcp4_12*ROcp4_52-OMcp4_22*ROcp4_42)
    RLcp4_14 = ROcp4_42*s.dpt[2,4]+ROcp4_73*s.dpt[3,4]
    RLcp4_24 = ROcp4_52*s.dpt[2,4]+ROcp4_83*s.dpt[3,4]
    RLcp4_34 = ROcp4_93*s.dpt[3,4]+s.dpt[2,4]*S2
    OMcp4_14 = OMcp4_13+ROcp4_13*qd[17]
    OMcp4_24 = OMcp4_23+ROcp4_23*qd[17]
    OMcp4_34 = OMcp4_33+ROcp4_33*qd[17]
    ORcp4_14 = OMcp4_23*RLcp4_34-OMcp4_33*RLcp4_24
    ORcp4_24 = -OMcp4_13*RLcp4_34+OMcp4_33*RLcp4_14
    ORcp4_34 = OMcp4_13*RLcp4_24-OMcp4_23*RLcp4_14
    OPcp4_14 = OPcp4_13+ROcp4_13*qdd[17]+qd[17]*(OMcp4_23*ROcp4_33-OMcp4_33*ROcp4_23)
    OPcp4_24 = OPcp4_23+ROcp4_23*qdd[17]+qd[17]*(-OMcp4_13*ROcp4_33+OMcp4_33*ROcp4_13)
    OPcp4_34 = OPcp4_33+ROcp4_33*qdd[17]+qd[17]*(OMcp4_13*ROcp4_23-OMcp4_23*ROcp4_13)
    ACcp4_14 = OMcp4_23*ORcp4_34-OMcp4_33*ORcp4_24+OPcp4_23*RLcp4_34-OPcp4_33*RLcp4_24
    ACcp4_24 = -OMcp4_13*ORcp4_34+OMcp4_33*ORcp4_14-OPcp4_13*RLcp4_34+OPcp4_33*RLcp4_14
    ACcp4_34 = OMcp4_13*ORcp4_24-OMcp4_23*ORcp4_14+OPcp4_13*RLcp4_24-OPcp4_23*RLcp4_14
    RLcp4_15 = ROcp4_717*s.dpt[3,13]
    RLcp4_25 = ROcp4_817*s.dpt[3,13]
    RLcp4_35 = ROcp4_917*s.dpt[3,13]
    POcp4_15 = RLcp4_14+RLcp4_15
    POcp4_25 = RLcp4_24+RLcp4_25
    POcp4_35 = RLcp4_34+RLcp4_35
    OMcp4_15 = OMcp4_14+ROcp4_13*qd[18]
    OMcp4_25 = OMcp4_24+ROcp4_23*qd[18]
    OMcp4_35 = OMcp4_34+ROcp4_33*qd[18]
    ORcp4_15 = OMcp4_24*RLcp4_35-OMcp4_34*RLcp4_25
    ORcp4_25 = -OMcp4_14*RLcp4_35+OMcp4_34*RLcp4_15
    ORcp4_35 = OMcp4_14*RLcp4_25-OMcp4_24*RLcp4_15
    VIcp4_15 = ORcp4_14+ORcp4_15
    VIcp4_25 = ORcp4_24+ORcp4_25
    VIcp4_35 = ORcp4_34+ORcp4_35
    OPcp4_15 = OPcp4_14+ROcp4_13*qdd[18]+qd[18]*(OMcp4_24*ROcp4_33-OMcp4_34*ROcp4_23)
    OPcp4_25 = OPcp4_24+ROcp4_23*qdd[18]+qd[18]*(-OMcp4_14*ROcp4_33+OMcp4_34*ROcp4_13)
    OPcp4_35 = OPcp4_34+ROcp4_33*qdd[18]+qd[18]*(OMcp4_14*ROcp4_23-OMcp4_24*ROcp4_13)
    ACcp4_15 = ACcp4_14+OMcp4_24*ORcp4_35-OMcp4_34*ORcp4_25+OPcp4_24*RLcp4_35-OPcp4_34*RLcp4_25
    ACcp4_25 = ACcp4_24-OMcp4_14*ORcp4_35+OMcp4_34*ORcp4_15-OPcp4_14*RLcp4_35+OPcp4_34*RLcp4_15
    ACcp4_35 = ACcp4_34+OMcp4_14*ORcp4_25-OMcp4_24*ORcp4_15+OPcp4_14*RLcp4_25-OPcp4_24*RLcp4_15
    OMcp4_16 = OMcp4_15+ROcp4_418*qd[19]
    OMcp4_26 = OMcp4_25+ROcp4_518*qd[19]
    OMcp4_36 = OMcp4_35+ROcp4_618*qd[19]
    OPcp4_16 = OPcp4_15+ROcp4_418*qdd[19]+qd[19]*(OMcp4_25*ROcp4_618-OMcp4_35*ROcp4_518)
    OPcp4_26 = OPcp4_25+ROcp4_518*qdd[19]+qd[19]*(-OMcp4_15*ROcp4_618+OMcp4_35*ROcp4_418)
    OPcp4_36 = OPcp4_35+ROcp4_618*qdd[19]+qd[19]*(OMcp4_15*ROcp4_518-OMcp4_25*ROcp4_418)
    RLcp4_17 = ROcp4_719*s.dpt[3,14]
    RLcp4_27 = ROcp4_819*s.dpt[3,14]
    RLcp4_37 = ROcp4_919*s.dpt[3,14]
    POcp4_17 = POcp4_15+RLcp4_17
    POcp4_27 = POcp4_25+RLcp4_27
    POcp4_37 = POcp4_35+RLcp4_37
    ORcp4_17 = OMcp4_26*RLcp4_37-OMcp4_36*RLcp4_27
    ORcp4_27 = -OMcp4_16*RLcp4_37+OMcp4_36*RLcp4_17
    ORcp4_37 = OMcp4_16*RLcp4_27-OMcp4_26*RLcp4_17
    VIcp4_17 = ORcp4_17+VIcp4_15
    VIcp4_27 = ORcp4_27+VIcp4_25
    VIcp4_37 = ORcp4_37+VIcp4_35
    ACcp4_17 = ACcp4_15+OMcp4_26*ORcp4_37-OMcp4_36*ORcp4_27+OPcp4_26*RLcp4_37-OPcp4_36*RLcp4_27
    ACcp4_27 = ACcp4_25-OMcp4_16*ORcp4_37+OMcp4_36*ORcp4_17-OPcp4_16*RLcp4_37+OPcp4_36*RLcp4_17
    ACcp4_37 = ACcp4_35+OMcp4_16*ORcp4_27-OMcp4_26*ORcp4_17+OPcp4_16*RLcp4_27-OPcp4_26*RLcp4_17
    sens.P[1] = POcp4_17
    sens.P[2] = POcp4_27
    sens.P[3] = POcp4_37
    sens.R[1,1] = ROcp4_119
    sens.R[1,2] = ROcp4_219
    sens.R[1,3] = ROcp4_319
    sens.R[2,1] = ROcp4_418
    sens.R[2,2] = ROcp4_518
    sens.R[2,3] = ROcp4_618
    sens.R[3,1] = ROcp4_719
    sens.R[3,2] = ROcp4_819
    sens.R[3,3] = ROcp4_919
    sens.V[1] = VIcp4_17
    sens.V[2] = VIcp4_27
    sens.V[3] = VIcp4_37
    sens.OM[1] = OMcp4_16
    sens.OM[2] = OMcp4_26
    sens.OM[3] = OMcp4_36
    sens.A[1] = ACcp4_17
    sens.A[2] = ACcp4_27
    sens.A[3] = ACcp4_37
    sens.OMP[1] = OPcp4_16
    sens.OMP[2] = OPcp4_26
    sens.OMP[3] = OPcp4_36

 


# Number of continuation lines = 1


