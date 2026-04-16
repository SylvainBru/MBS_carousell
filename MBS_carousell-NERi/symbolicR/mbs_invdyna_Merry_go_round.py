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
#	==> Generation Date: Tue Apr 14 11:47:41 2026
#	==> using automatic loading with extension .mbs 
#
#	==> Project name: Merry_go_round
#
#	==> Number of joints: 19
#
#	==> Function: F2 - Recursive Inverse Dynamics of tree-like MBS
#
#	==> Git hash: 0e4e6a608eeee06956095d2f2ad315abdb092777
#
##

from math import sin, cos

def invdyna(phi,s,tsim):
    Qq = phi  # compatibility with symbolic generation
    q = s.q
    qd = s.qd
    qdd = s.qdd
 
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

 
# Forward Kinematics

    OM22 = qd[1]*S2
    OM32 = qd[1]*C2
    OMp22 = qd[1]*qd[2]*C2+qdd[1]*S2
    OMp32 = -qd[1]*qd[2]*S2+qdd[1]*C2
    ALPHA22 = -s.g[3]*S2
    ALPHA32 = -s.g[3]*C2
    OM13 = qd[2]*C3-OM32*S3
    OM23 = qd[3]+OM22
    OM33 = qd[2]*S3+OM32*C3
    OMp13 = C3*(qdd[2]-qd[3]*OM32)-S3*(OMp32+qd[2]*qd[3])
    OMp23 = qdd[3]+OMp22
    OMp33 = C3*(OMp32+qd[2]*qd[3])+S3*(qdd[2]-qd[3]*OM32)
    BS13 = -OM23*OM23-OM33*OM33
    BS23 = OM13*OM23
    BS33 = OM13*OM33
    BS53 = -OM13*OM13-OM33*OM33
    BS63 = OM23*OM33
    BS93 = -OM13*OM13-OM23*OM23
    BETA23 = BS23-OMp33
    BETA33 = BS33+OMp23
    BETA43 = BS23+OMp33
    BETA63 = BS63-OMp13
    BETA73 = BS33-OMp23
    BETA83 = BS63+OMp13
    ALPHA13 = -ALPHA32*S3
    ALPHA33 = ALPHA32*C3
    OM14 = OM13*C4-OM33*S4
    OM24 = qd[4]+OM23
    OM34 = OM13*S4+OM33*C4
    OMp14 = C4*(OMp13-qd[4]*OM33)-S4*(OMp33+qd[4]*OM13)
    OMp24 = qdd[4]+OMp23
    OMp34 = C4*(OMp33+qd[4]*OM13)+S4*(OMp13-qd[4]*OM33)
    BS34 = OM14*OM34
    BS64 = OM24*OM34
    BS94 = -OM14*OM14-OM24*OM24
    BETA34 = BS34+OMp24
    BETA64 = BS64-OMp14
    ALPHA14 = C4*(ALPHA13+BETA33*s.dpt[3,1]+BS13*s.dpt[1,1])-S4*(ALPHA33+BETA73*s.dpt[1,1]+BS93*s.dpt[3,1])
    ALPHA24 = ALPHA22+BETA43*s.dpt[1,1]+BETA63*s.dpt[3,1]
    ALPHA34 = C4*(ALPHA33+BETA73*s.dpt[1,1]+BS93*s.dpt[3,1])+S4*(ALPHA13+BETA33*s.dpt[3,1]+BS13*s.dpt[1,1])
    OM15 = OM14*C5-OM34*S5
    OM25 = qd[5]+OM24
    OM35 = OM14*S5+OM34*C5
    OMp15 = C5*(OMp14-qd[5]*OM34)-S5*(OMp34+qd[5]*OM14)
    OMp25 = qdd[5]+OMp24
    OMp35 = C5*(OMp34+qd[5]*OM14)+S5*(OMp14-qd[5]*OM34)
    ALPHA15 = C5*(ALPHA14+BETA34*s.dpt[3,5])-S5*(ALPHA34+BS94*s.dpt[3,5])
    ALPHA25 = ALPHA24+BETA64*s.dpt[3,5]
    ALPHA35 = C5*(ALPHA34+BS94*s.dpt[3,5])+S5*(ALPHA14+BETA34*s.dpt[3,5])
    OM16 = qd[6]+OM15
    OM26 = OM25*C6+OM35*S6
    OM36 = -OM25*S6+OM35*C6
    OMp16 = qdd[6]+OMp15
    OMp26 = C6*(OMp25+qd[6]*OM35)+S6*(OMp35-qd[6]*OM25)
    OMp36 = C6*(OMp35-qd[6]*OM25)-S6*(OMp25+qd[6]*OM35)
    BS36 = OM16*OM36
    BS66 = OM26*OM36
    BS96 = -OM16*OM16-OM26*OM26
    BETA36 = BS36+OMp26
    BETA66 = BS66-OMp16
    ALPHA26 = ALPHA25*C6+ALPHA35*S6
    ALPHA36 = -ALPHA25*S6+ALPHA35*C6
    BS17 = -OM23*OM23-OM33*OM33
    BS27 = OM13*OM23
    BS37 = OM13*OM33
    BS57 = -OM13*OM13-OM33*OM33
    BS67 = OM23*OM33
    BETA27 = BS27-OMp33
    BETA47 = BS27+OMp33
    BETA77 = BS37-OMp23
    BETA87 = BS67+OMp13
    ALPHA17 = qdd[7]+ALPHA13+q[7]*BS13+BETA33*s.dpt[3,2]
    ALPHA27 = ALPHA22+q[7]*BETA43+(2.0)*qd[7]*OM33+BETA63*s.dpt[3,2]
    ALPHA37 = ALPHA33+q[7]*BETA73-(2.0)*qd[7]*OM23+BS93*s.dpt[3,2]
    BS38 = OM13*OM33
    BS68 = OM23*OM33
    BS98 = -OM13*OM13-OM23*OM23
    BETA38 = BS38+OMp23
    BETA68 = BS68-OMp13
    ALPHA18 = qdd[8]+ALPHA17+q[8]*BS17+BETA27*s.dpt[2,7]
    ALPHA28 = ALPHA27+q[8]*BETA47+(2.0)*qd[8]*OM33+BS57*s.dpt[2,7]
    ALPHA38 = ALPHA37+q[8]*BETA77-(2.0)*qd[8]*OM23+BETA87*s.dpt[2,7]
    ALPHA19 = ALPHA18+q[9]*BETA38+(2.0)*qd[9]*OM23
    ALPHA29 = ALPHA28+q[9]*BETA68-(2.0)*qd[9]*OM13
    ALPHA39 = qdd[9]+ALPHA38+q[9]*BS98
    OM110 = OM13*C10-OM33*S10
    OM210 = qd[10]+OM23
    OM310 = OM13*S10+OM33*C10
    OMp110 = C10*(OMp13-qd[10]*OM33)-S10*(OMp33+qd[10]*OM13)
    OMp210 = qdd[10]+OMp23
    OMp310 = C10*(OMp33+qd[10]*OM13)+S10*(OMp13-qd[10]*OM33)
    ALPHA110 = ALPHA19*C10-ALPHA39*S10
    ALPHA310 = ALPHA19*S10+ALPHA39*C10
    OM111 = qd[11]+OM110
    OM211 = OM210*C11+OM310*S11
    OM311 = -OM210*S11+OM310*C11
    OMp111 = qdd[11]+OMp110
    OMp211 = C11*(OMp210+qd[11]*OM310)+S11*(OMp310-qd[11]*OM210)
    OMp311 = C11*(OMp310-qd[11]*OM210)-S11*(OMp210+qd[11]*OM310)
    BS311 = OM111*OM311
    BS611 = OM211*OM311
    BS911 = -OM111*OM111-OM211*OM211
    BETA311 = BS311+OMp211
    BETA611 = BS611-OMp111
    ALPHA211 = ALPHA29*C11+ALPHA310*S11
    ALPHA311 = -ALPHA29*S11+ALPHA310*C11
    OM112 = qd[12]+OM111
    OM212 = OM211*C12+OM311*S12
    OM312 = -OM211*S12+OM311*C12
    OMp112 = qdd[12]+OMp111
    OMp212 = C12*(OMp211+qd[12]*OM311)+S12*(OMp311-qd[12]*OM211)
    OMp312 = C12*(OMp311-qd[12]*OM211)-S12*(OMp211+qd[12]*OM311)
    ALPHA112 = ALPHA110+BETA311*s.dpt[3,9]
    ALPHA212 = C12*(ALPHA211+BETA611*s.dpt[3,9])+S12*(ALPHA311+BS911*s.dpt[3,9])
    ALPHA312 = C12*(ALPHA311+BS911*s.dpt[3,9])-S12*(ALPHA211+BETA611*s.dpt[3,9])
    OM113 = OM112*C13-OM312*S13
    OM213 = qd[13]+OM212
    OM313 = OM112*S13+OM312*C13
    OMp113 = C13*(OMp112-qd[13]*OM312)-S13*(OMp312+qd[13]*OM112)
    OMp213 = qdd[13]+OMp212
    OMp313 = C13*(OMp312+qd[13]*OM112)+S13*(OMp112-qd[13]*OM312)
    BS313 = OM113*OM313
    BS613 = OM213*OM313
    BS913 = -OM113*OM113-OM213*OM213
    BETA313 = BS313+OMp213
    BETA613 = BS613-OMp113
    ALPHA113 = ALPHA112*C13-ALPHA312*S13
    ALPHA313 = ALPHA112*S13+ALPHA312*C13
    OM114 = OM13*C14-OM33*S14
    OM214 = qd[14]+OM23
    OM314 = OM13*S14+OM33*C14
    OMp114 = C14*(OMp13-qd[14]*OM33)-S14*(OMp33+qd[14]*OM13)
    OMp214 = qdd[14]+OMp23
    OMp314 = C14*(OMp33+qd[14]*OM13)+S14*(OMp13-qd[14]*OM33)
    BS314 = OM114*OM314
    BS614 = OM214*OM314
    BS914 = -OM114*OM114-OM214*OM214
    BETA314 = BS314+OMp214
    BETA614 = BS614-OMp114
    ALPHA114 = C14*(ALPHA13+BETA33*s.dpt[3,3]+BS13*s.dpt[1,3])-S14*(ALPHA33+BETA73*s.dpt[1,3]+BS93*s.dpt[3,3])
    ALPHA214 = ALPHA22+BETA43*s.dpt[1,3]+BETA63*s.dpt[3,3]
    ALPHA314 = C14*(ALPHA33+BETA73*s.dpt[1,3]+BS93*s.dpt[3,3])+S14*(ALPHA13+BETA33*s.dpt[3,3]+BS13*s.dpt[1,3])
    OM115 = OM114*C15-OM314*S15
    OM215 = qd[15]+OM214
    OM315 = OM114*S15+OM314*C15
    OMp115 = C15*(OMp114-qd[15]*OM314)-S15*(OMp314+qd[15]*OM114)
    OMp215 = qdd[15]+OMp214
    OMp315 = C15*(OMp314+qd[15]*OM114)+S15*(OMp114-qd[15]*OM314)
    ALPHA115 = C15*(ALPHA114+BETA314*s.dpt[3,11])-S15*(ALPHA314+BS914*s.dpt[3,11])
    ALPHA215 = ALPHA214+BETA614*s.dpt[3,11]
    ALPHA315 = C15*(ALPHA314+BS914*s.dpt[3,11])+S15*(ALPHA114+BETA314*s.dpt[3,11])
    OM116 = qd[16]+OM115
    OM216 = OM215*C16+OM315*S16
    OM316 = -OM215*S16+OM315*C16
    OMp116 = qdd[16]+OMp115
    OMp216 = C16*(OMp215+qd[16]*OM315)+S16*(OMp315-qd[16]*OM215)
    OMp316 = C16*(OMp315-qd[16]*OM215)-S16*(OMp215+qd[16]*OM315)
    BS316 = OM116*OM316
    BS616 = OM216*OM316
    BS916 = -OM116*OM116-OM216*OM216
    BETA316 = BS316+OMp216
    BETA616 = BS616-OMp116
    ALPHA216 = ALPHA215*C16+ALPHA315*S16
    ALPHA316 = -ALPHA215*S16+ALPHA315*C16
    OM117 = qd[17]+OM13
    OM217 = OM23*C17+OM33*S17
    OM317 = -OM23*S17+OM33*C17
    OMp117 = qdd[17]+OMp13
    OMp217 = C17*(OMp23+qd[17]*OM33)+S17*(OMp33-qd[17]*OM23)
    OMp317 = C17*(OMp33-qd[17]*OM23)-S17*(OMp23+qd[17]*OM33)
    BS317 = OM117*OM317
    BS617 = OM217*OM317
    BS917 = -OM117*OM117-OM217*OM217
    BETA317 = BS317+OMp217
    BETA617 = BS617-OMp117
    ALPHA117 = ALPHA13+BETA23*s.dpt[2,4]+BETA33*s.dpt[3,4]
    ALPHA217 = C17*(ALPHA22+BETA63*s.dpt[3,4]+BS53*s.dpt[2,4])+S17*(ALPHA33+BETA83*s.dpt[2,4]+BS93*s.dpt[3,4])
    ALPHA317 = C17*(ALPHA33+BETA83*s.dpt[2,4]+BS93*s.dpt[3,4])-S17*(ALPHA22+BETA63*s.dpt[3,4]+BS53*s.dpt[2,4])
    OM118 = qd[18]+OM117
    OM218 = OM217*C18+OM317*S18
    OM318 = -OM217*S18+OM317*C18
    OMp118 = qdd[18]+OMp117
    OMp218 = C18*(OMp217+qd[18]*OM317)+S18*(OMp317-qd[18]*OM217)
    OMp318 = C18*(OMp317-qd[18]*OM217)-S18*(OMp217+qd[18]*OM317)
    ALPHA118 = ALPHA117+BETA317*s.dpt[3,13]
    ALPHA218 = C18*(ALPHA217+BETA617*s.dpt[3,13])+S18*(ALPHA317+BS917*s.dpt[3,13])
    ALPHA318 = C18*(ALPHA317+BS917*s.dpt[3,13])-S18*(ALPHA217+BETA617*s.dpt[3,13])
    OM119 = OM118*C19-OM318*S19
    OM219 = qd[19]+OM218
    OM319 = OM118*S19+OM318*C19
    OMp119 = C19*(OMp118-qd[19]*OM318)-S19*(OMp318+qd[19]*OM118)
    OMp219 = qdd[19]+OMp218
    OMp319 = C19*(OMp318+qd[19]*OM118)+S19*(OMp118-qd[19]*OM318)
    BS319 = OM119*OM319
    BS619 = OM219*OM319
    BS919 = -OM119*OM119-OM219*OM219
    BETA319 = BS319+OMp219
    BETA619 = BS619-OMp119
    ALPHA119 = ALPHA118*C19-ALPHA318*S19
    ALPHA319 = ALPHA118*S19+ALPHA318*C19
 
# Backward Dynamics

    Fs119 = -s.frc[1,19]+s.m[19]*(ALPHA119+BETA319*s.l[3,19])
    Fs219 = -s.frc[2,19]+s.m[19]*(ALPHA218+BETA619*s.l[3,19])
    Fs319 = -s.frc[3,19]+s.m[19]*(ALPHA319+BS919*s.l[3,19])
    Cq119 = -s.trq[1,19]+s.In[1,19]*OMp119-s.In[5,19]*OM219*OM319+s.In[9,19]*OM219*OM319-Fs219*s.l[3,19]
    Cq219 = -s.trq[2,19]+s.In[1,19]*OM119*OM319+s.In[5,19]*OMp219-s.In[9,19]*OM119*OM319+Fs119*s.l[3,19]
    Cq319 = -s.trq[3,19]-s.In[1,19]*OM119*OM219+s.In[5,19]*OM119*OM219+s.In[9,19]*OMp319
    Fq118 = Fs119*C19+Fs319*S19
    Fq318 = -Fs119*S19+Fs319*C19
    Cq118 = Cq119*C19+Cq319*S19
    Cq318 = -Cq119*S19+Cq319*C19
    Fs117 = -s.frc[1,17]+s.m[17]*(ALPHA117+BETA317*s.l[3,17])
    Fs217 = -s.frc[2,17]+s.m[17]*(ALPHA217+BETA617*s.l[3,17])
    Fs317 = -s.frc[3,17]+s.m[17]*(ALPHA317+BS917*s.l[3,17])
    Fq117 = Fq118+Fs117
    Fq217 = Fs217-Fq318*S18+Fs219*C18
    Fq317 = Fs317+Fq318*C18+Fs219*S18
    Cq117 = -s.trq[1,17]+Cq118+s.In[1,17]*OMp117-s.In[5,17]*OM217*OM317+s.In[9,17]*OM217*OM317-Fs217*s.l[3,17]- \
 	  s.dpt[3,13]*(-Fq318*S18+Fs219*C18)
    Cq217 = -s.trq[2,17]+s.In[1,17]*OM117*OM317+s.In[5,17]*OMp217-s.In[9,17]*OM117*OM317+Cq219*C18-Cq318*S18+Fq118* \
 	  s.dpt[3,13]+Fs117*s.l[3,17]
    Cq317 = -s.trq[3,17]-s.In[1,17]*OM117*OM217+s.In[5,17]*OM117*OM217+s.In[9,17]*OMp317+Cq219*S18+Cq318*C18
    Fs116 = -s.frc[1,16]+s.m[16]*(ALPHA115+BETA316*s.l[3,16])
    Fs216 = -s.frc[2,16]+s.m[16]*(ALPHA216+BETA616*s.l[3,16])
    Fs316 = -s.frc[3,16]+s.m[16]*(ALPHA316+BS916*s.l[3,16])
    Cq116 = -s.trq[1,16]+s.In[1,16]*OMp116-s.In[5,16]*OM216*OM316+s.In[9,16]*OM216*OM316-Fs216*s.l[3,16]
    Cq216 = -s.trq[2,16]+s.In[1,16]*OM116*OM316+s.In[5,16]*OMp216-s.In[9,16]*OM116*OM316+Fs116*s.l[3,16]
    Cq316 = -s.trq[3,16]-s.In[1,16]*OM116*OM216+s.In[5,16]*OM116*OM216+s.In[9,16]*OMp316
    Fq215 = Fs216*C16-Fs316*S16
    Fq315 = Fs216*S16+Fs316*C16
    Cq215 = Cq216*C16-Cq316*S16
    Cq315 = Cq216*S16+Cq316*C16
    Fs114 = -s.frc[1,14]+s.m[14]*(ALPHA114+BETA314*s.l[3,14])
    Fs214 = -s.frc[2,14]+s.m[14]*(ALPHA214+BETA614*s.l[3,14])
    Fs314 = -s.frc[3,14]+s.m[14]*(ALPHA314+BS914*s.l[3,14])
    Fq114 = Fs114+Fq315*S15+Fs116*C15
    Fq214 = Fq215+Fs214
    Fq314 = Fs314+Fq315*C15-Fs116*S15
    Cq114 = -s.trq[1,14]+s.In[1,14]*OMp114-s.In[5,14]*OM214*OM314+s.In[9,14]*OM214*OM314+Cq116*C15+Cq315*S15-Fq215* \
 	  s.dpt[3,11]-Fs214*s.l[3,14]
    Cq214 = -s.trq[2,14]+Cq215+s.In[1,14]*OM114*OM314+s.In[5,14]*OMp214-s.In[9,14]*OM114*OM314+Fs114*s.l[3,14]+ \
 	  s.dpt[3,11]*(Fq315*S15+Fs116*C15)
    Cq314 = -s.trq[3,14]-s.In[1,14]*OM114*OM214+s.In[5,14]*OM114*OM214+s.In[9,14]*OMp314-Cq116*S15+Cq315*C15
    Fs113 = -s.frc[1,13]+s.m[13]*(ALPHA113+BETA313*s.l[3,13])
    Fs213 = -s.frc[2,13]+s.m[13]*(ALPHA212+BETA613*s.l[3,13])
    Fs313 = -s.frc[3,13]+s.m[13]*(ALPHA313+BS913*s.l[3,13])
    Cq113 = -s.trq[1,13]+s.In[1,13]*OMp113-s.In[5,13]*OM213*OM313+s.In[9,13]*OM213*OM313-Fs213*s.l[3,13]
    Cq213 = -s.trq[2,13]+s.In[1,13]*OM113*OM313+s.In[5,13]*OMp213-s.In[9,13]*OM113*OM313+Fs113*s.l[3,13]
    Cq313 = -s.trq[3,13]-s.In[1,13]*OM113*OM213+s.In[5,13]*OM113*OM213+s.In[9,13]*OMp313
    Fq112 = Fs113*C13+Fs313*S13
    Fq312 = -Fs113*S13+Fs313*C13
    Cq112 = Cq113*C13+Cq313*S13
    Cq312 = -Cq113*S13+Cq313*C13
    Fs111 = -s.frc[1,11]+s.m[11]*(ALPHA110+BETA311*s.l[3,11])
    Fs211 = -s.frc[2,11]+s.m[11]*(ALPHA211+BETA611*s.l[3,11])
    Fs311 = -s.frc[3,11]+s.m[11]*(ALPHA311+BS911*s.l[3,11])
    Fq111 = Fq112+Fs111
    Fq211 = Fs211-Fq312*S12+Fs213*C12
    Fq311 = Fs311+Fq312*C12+Fs213*S12
    Cq111 = -s.trq[1,11]+Cq112+s.In[1,11]*OMp111-s.In[5,11]*OM211*OM311+s.In[9,11]*OM211*OM311-Fs211*s.l[3,11]- \
 	  s.dpt[3,9]*(-Fq312*S12+Fs213*C12)
    Cq211 = -s.trq[2,11]+s.In[1,11]*OM111*OM311+s.In[5,11]*OMp211-s.In[9,11]*OM111*OM311+Cq213*C12-Cq312*S12+Fq112* \
 	  s.dpt[3,9]+Fs111*s.l[3,11]
    Cq311 = -s.trq[3,11]-s.In[1,11]*OM111*OM211+s.In[5,11]*OM111*OM211+s.In[9,11]*OMp311+Cq213*S12+Cq312*C12
    Fs110 = -s.frc[1,10]+s.m[10]*ALPHA110
    Fs210 = -s.frc[2,10]+s.m[10]*ALPHA29
    Fs310 = -s.frc[3,10]+s.m[10]*ALPHA310
    Fq110 = Fq111+Fs110
    Fq210 = Fs210+Fq211*C11-Fq311*S11
    Fq310 = Fs310+Fq211*S11+Fq311*C11
    Cq110 = -s.trq[1,10]+Cq111
    Cq210 = -s.trq[2,10]+Cq211*C11-Cq311*S11
    Cq310 = -s.trq[3,10]+Cq211*S11+Cq311*C11
    Fq19 = Fq110*C10+Fq310*S10
    Fq39 = -Fq110*S10+Fq310*C10
    Cq19 = Cq110*C10+Cq310*S10
    Cq39 = -Cq110*S10+Cq310*C10
    Cq18 = Cq19-q[9]*Fq210
    Cq28 = Cq210+q[9]*Fq19
    Fs17 = -s.frc[1,7]+s.m[7]*(ALPHA17+BS17*s.l[1,7])
    Fs27 = -s.frc[2,7]+s.m[7]*(ALPHA27+BETA47*s.l[1,7])
    Fs37 = -s.frc[3,7]+s.m[7]*(ALPHA37+BETA77*s.l[1,7])
    Fq17 = Fq19+Fs17
    Fq27 = Fq210+Fs27
    Fq37 = Fq39+Fs37
    Cq17 = -s.trq[1,7]+Cq18+s.In[1,7]*OMp13-s.In[5,7]*OM23*OM33+s.In[9,7]*OM23*OM33+Fq39*s.dpt[2,7]
    Cq27 = -s.trq[2,7]+Cq28-q[8]*Fq39+s.In[1,7]*OM13*OM33+s.In[5,7]*OMp23-s.In[9,7]*OM13*OM33-Fs37*s.l[1,7]
    Cq37 = -s.trq[3,7]+Cq39+q[8]*Fq210-s.In[1,7]*OM13*OM23+s.In[5,7]*OM13*OM23+s.In[9,7]*OMp33-Fq19*s.dpt[2,7]+Fs27* \
 	  s.l[1,7]
    Fs16 = -s.frc[1,6]+s.m[6]*(ALPHA15+BETA36*s.l[3,6])
    Fs26 = -s.frc[2,6]+s.m[6]*(ALPHA26+BETA66*s.l[3,6])
    Fs36 = -s.frc[3,6]+s.m[6]*(ALPHA36+BS96*s.l[3,6])
    Cq16 = -s.trq[1,6]+s.In[1,6]*OMp16-s.In[5,6]*OM26*OM36+s.In[9,6]*OM26*OM36-Fs26*s.l[3,6]
    Cq26 = -s.trq[2,6]+s.In[1,6]*OM16*OM36+s.In[5,6]*OMp26-s.In[9,6]*OM16*OM36+Fs16*s.l[3,6]
    Cq36 = -s.trq[3,6]-s.In[1,6]*OM16*OM26+s.In[5,6]*OM16*OM26+s.In[9,6]*OMp36
    Fq25 = Fs26*C6-Fs36*S6
    Fq35 = Fs26*S6+Fs36*C6
    Cq25 = Cq26*C6-Cq36*S6
    Cq35 = Cq26*S6+Cq36*C6
    Fs14 = -s.frc[1,4]+s.m[4]*(ALPHA14+BETA34*s.l[3,4])
    Fs24 = -s.frc[2,4]+s.m[4]*(ALPHA24+BETA64*s.l[3,4])
    Fs34 = -s.frc[3,4]+s.m[4]*(ALPHA34+BS94*s.l[3,4])
    Fq14 = Fs14+Fq35*S5+Fs16*C5
    Fq24 = Fq25+Fs24
    Fq34 = Fs34+Fq35*C5-Fs16*S5
    Cq14 = -s.trq[1,4]+s.In[1,4]*OMp14-s.In[5,4]*OM24*OM34+s.In[9,4]*OM24*OM34+Cq16*C5+Cq35*S5-Fq25*s.dpt[3,5]-Fs24* \
 	  s.l[3,4]
    Cq24 = -s.trq[2,4]+Cq25+s.In[1,4]*OM14*OM34+s.In[5,4]*OMp24-s.In[9,4]*OM14*OM34+Fs14*s.l[3,4]+s.dpt[3,5]*(Fq35*S5 \
 	  +Fs16*C5)
    Cq34 = -s.trq[3,4]-s.In[1,4]*OM14*OM24+s.In[5,4]*OM14*OM24+s.In[9,4]*OMp34-Cq16*S5+Cq35*C5
    Fs13 = -s.frc[1,3]+s.m[3]*(ALPHA13+BETA33*s.l[3,3])
    Fs23 = -s.frc[2,3]+s.m[3]*(ALPHA22+BETA63*s.l[3,3])
    Cq13 = -s.trq[1,3]+Cq117+Cq17+s.In[1,3]*OMp13-s.In[5,3]*OM23*OM33+s.In[9,3]*OM23*OM33+Cq114*C14+Cq14*C4+Cq314*S14 \
 	  +Cq34*S4-Fq214*s.dpt[3,3]-Fq24*s.dpt[3,1]-Fq27*s.dpt[3,2]-Fs23*s.l[3,3]+s.dpt[2,4]*(Fq217*S17+Fq317*C17)-s.dpt[3,4]*( \
 	  Fq217*C17-Fq317*S17)
    Cq23 = -s.trq[2,3]+Cq214+Cq24+Cq27-q[7]*Fq37+s.In[1,3]*OM13*OM33+s.In[5,3]*OMp23-s.In[9,3]*OM13*OM33+Cq217*C17- \
 	  Cq317*S17+Fq117*s.dpt[3,4]+Fq17*s.dpt[3,2]+Fs13*s.l[3,3]-s.dpt[1,1]*(-Fq14*S4+Fq34*C4)-s.dpt[1,3]*(-Fq114*S14+Fq314* \
 	  C14)+s.dpt[3,1]*(Fq14*C4+Fq34*S4)+s.dpt[3,3]*(Fq114*C14+Fq314*S14)
    Cq33 = -s.trq[3,3]+Cq37+q[7]*Fq27-s.In[1,3]*OM13*OM23+s.In[5,3]*OM13*OM23+s.In[9,3]*OMp33-Cq114*S14-Cq14*S4+Cq217 \
 	  *S17+Cq314*C14+Cq317*C17+Cq34*C4-Fq117*s.dpt[2,4]+Fq214*s.dpt[1,3]+Fq24*s.dpt[1,1]
    Cq12 = Cq13*C3+Cq33*S3
    Cq32 = -Cq13*S3+Cq33*C3
    Cq31 = Cq23*S2+Cq32*C2
 
# Symbolic model output

    Qq[1] = Cq31
    Qq[2] = Cq12
    Qq[3] = Cq23
    Qq[4] = Cq24
    Qq[5] = Cq25
    Qq[6] = Cq16
    Qq[7] = Fq17
    Qq[8] = Fq19
    Qq[9] = Fq39
    Qq[10] = Cq210
    Qq[11] = Cq111
    Qq[12] = Cq112
    Qq[13] = Cq213
    Qq[14] = Cq214
    Qq[15] = Cq215
    Qq[16] = Cq116
    Qq[17] = Cq117
    Qq[18] = Cq118
    Qq[19] = Cq219

# Number of continuation lines = 2


