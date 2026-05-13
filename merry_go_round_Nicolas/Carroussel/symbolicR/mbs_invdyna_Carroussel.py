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
#	==> Generation Date: Wed Apr  8 11:28:43 2026
#	==> using automatic loading with extension .mbs 
#
#	==> Project name: Carroussel
#
#	==> Number of joints: 14
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
    S3 = sin(q[3])
    C3 = cos(q[3])
    S4 = sin(q[4])
    C4 = cos(q[4])
    S5 = sin(q[5])
    C5 = cos(q[5])
    S6 = sin(q[6])
    C6 = cos(q[6])
    S7 = sin(q[7])
    C7 = cos(q[7])
    S8 = sin(q[8])
    C8 = cos(q[8])
    S9 = sin(q[9])
    C9 = cos(q[9])
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
 
# Augmented Joint Position Vectors

    Dz23 = q[2]+s.dpt[3,1]
 
# Augmented Joint Position Vectors

 
# Forward Kinematics

    BS12 = -qd[1]*qd[1]
    BS52 = -qd[1]*qd[1]
    ALPHA32 = qdd[2]-s.g[3]
    OM23 = qd[1]*S3
    OM33 = qd[1]*C3
    OMp23 = qd[1]*qd[3]*C3+qdd[1]*S3
    OMp33 = -qd[1]*qd[3]*S3+qdd[1]*C3
    BS23 = qd[3]*OM23
    BS53 = -qd[3]*qd[3]-OM33*OM33
    BS63 = OM23*OM33
    BETA23 = BS23-OMp33
    BETA83 = qdd[3]+BS63
    ALPHA13 = -qdd[1]*s.dpt[2,2]
    ALPHA23 = ALPHA32*S3+BS52*s.dpt[2,2]*C3
    ALPHA33 = ALPHA32*C3-BS52*s.dpt[2,2]*S3
    OM14 = qd[3]+qd[4]
    OM24 = OM23*C4+OM33*S4
    OM34 = -OM23*S4+OM33*C4
    OMp14 = qdd[3]+qdd[4]
    OMp24 = C4*(OMp23+qd[4]*OM33)+S4*(OMp33-qd[4]*OM23)
    OMp34 = C4*(OMp33-qd[4]*OM23)-S4*(OMp23+qd[4]*OM33)
    BS34 = OM14*OM34
    BS64 = OM24*OM34
    BS94 = -OM14*OM14-OM24*OM24
    BETA34 = BS34+OMp24
    BETA64 = BS64-OMp14
    ALPHA14 = ALPHA13+BETA23*s.dpt[2,6]
    ALPHA24 = C4*(ALPHA23+BS53*s.dpt[2,6])+S4*(ALPHA33+BETA83*s.dpt[2,6])
    ALPHA34 = C4*(ALPHA33+BETA83*s.dpt[2,6])-S4*(ALPHA23+BS53*s.dpt[2,6])
    OM15 = qd[5]+OM14
    OM25 = OM24*C5+OM34*S5
    OM35 = -OM24*S5+OM34*C5
    OMp15 = qdd[5]+OMp14
    OMp25 = C5*(OMp24+qd[5]*OM34)+S5*(OMp34-qd[5]*OM24)
    OMp35 = C5*(OMp34-qd[5]*OM24)-S5*(OMp24+qd[5]*OM34)
    ALPHA15 = ALPHA14+BETA34*s.dpt[3,7]
    ALPHA25 = C5*(ALPHA24+BETA64*s.dpt[3,7])+S5*(ALPHA34+BS94*s.dpt[3,7])
    ALPHA35 = C5*(ALPHA34+BS94*s.dpt[3,7])-S5*(ALPHA24+BETA64*s.dpt[3,7])
    OM26 = qd[1]*S6
    OM36 = qd[1]*C6
    OMp26 = qd[1]*qd[6]*C6+qdd[1]*S6
    OMp36 = -qd[1]*qd[6]*S6+qdd[1]*C6
    BS26 = qd[6]*OM26
    BS56 = -qd[6]*qd[6]-OM36*OM36
    BS66 = OM26*OM36
    BETA26 = BS26-OMp36
    BETA86 = qdd[6]+BS66
    ALPHA16 = -qdd[1]*s.dpt[2,3]
    ALPHA26 = ALPHA32*S6+BS52*s.dpt[2,3]*C6
    ALPHA36 = ALPHA32*C6-BS52*s.dpt[2,3]*S6
    OM17 = qd[6]+qd[7]
    OM27 = OM26*C7+OM36*S7
    OM37 = -OM26*S7+OM36*C7
    OMp17 = qdd[6]+qdd[7]
    OMp27 = C7*(OMp26+qd[7]*OM36)+S7*(OMp36-qd[7]*OM26)
    OMp37 = C7*(OMp36-qd[7]*OM26)-S7*(OMp26+qd[7]*OM36)
    BS37 = OM17*OM37
    BS67 = OM27*OM37
    BS97 = -OM17*OM17-OM27*OM27
    BETA37 = BS37+OMp27
    BETA67 = BS67-OMp17
    ALPHA17 = ALPHA16+BETA26*s.dpt[2,8]
    ALPHA27 = C7*(ALPHA26+BS56*s.dpt[2,8])+S7*(ALPHA36+BETA86*s.dpt[2,8])
    ALPHA37 = C7*(ALPHA36+BETA86*s.dpt[2,8])-S7*(ALPHA26+BS56*s.dpt[2,8])
    OM18 = qd[8]+OM17
    OM28 = OM27*C8+OM37*S8
    OM38 = -OM27*S8+OM37*C8
    OMp18 = qdd[8]+OMp17
    OMp28 = C8*(OMp27+qd[8]*OM37)+S8*(OMp37-qd[8]*OM27)
    OMp38 = C8*(OMp37-qd[8]*OM27)-S8*(OMp27+qd[8]*OM37)
    ALPHA18 = ALPHA17+BETA37*s.dpt[3,9]
    ALPHA28 = C8*(ALPHA27+BETA67*s.dpt[3,9])+S8*(ALPHA37+BS97*s.dpt[3,9])
    ALPHA38 = C8*(ALPHA37+BS97*s.dpt[3,9])-S8*(ALPHA27+BETA67*s.dpt[3,9])
    OM19 = -qd[1]*S9
    OM39 = qd[1]*C9
    OMp19 = -qd[1]*qd[9]*C9-qdd[1]*S9
    OMp39 = -qd[1]*qd[9]*S9+qdd[1]*C9
    BS19 = -qd[9]*qd[9]-OM39*OM39
    BS29 = qd[9]*OM19
    BS39 = OM19*OM39
    BETA49 = BS29+OMp39
    BETA79 = -qdd[9]+BS39
    ALPHA19 = -ALPHA32*S9+BS12*s.dpt[1,4]*C9
    ALPHA29 = qdd[1]*s.dpt[1,4]
    ALPHA39 = ALPHA32*C9+BS12*s.dpt[1,4]*S9
    OM110 = OM19*C10-OM39*S10
    OM210 = qd[10]+qd[9]
    OM310 = OM19*S10+OM39*C10
    OMp110 = C10*(OMp19-qd[10]*OM39)-S10*(OMp39+qd[10]*OM19)
    OMp210 = qdd[10]+qdd[9]
    OMp310 = C10*(OMp39+qd[10]*OM19)+S10*(OMp19-qd[10]*OM39)
    BS310 = OM110*OM310
    BS610 = OM210*OM310
    BS910 = -OM110*OM110-OM210*OM210
    BETA310 = BS310+OMp210
    BETA610 = BS610-OMp110
    ALPHA110 = C10*(ALPHA19+BS19*s.dpt[1,10])-S10*(ALPHA39+BETA79*s.dpt[1,10])
    ALPHA210 = ALPHA29+BETA49*s.dpt[1,10]
    ALPHA310 = C10*(ALPHA39+BETA79*s.dpt[1,10])+S10*(ALPHA19+BS19*s.dpt[1,10])
    OM111 = OM110*C11-OM310*S11
    OM211 = qd[11]+OM210
    OM311 = OM110*S11+OM310*C11
    OMp111 = C11*(OMp110-qd[11]*OM310)-S11*(OMp310+qd[11]*OM110)
    OMp211 = qdd[11]+OMp210
    OMp311 = C11*(OMp310+qd[11]*OM110)+S11*(OMp110-qd[11]*OM310)
    ALPHA111 = C11*(ALPHA110+BETA310*s.dpt[3,11])-S11*(ALPHA310+BS910*s.dpt[3,11])
    ALPHA211 = ALPHA210+BETA610*s.dpt[3,11]
    ALPHA311 = C11*(ALPHA310+BS910*s.dpt[3,11])+S11*(ALPHA110+BETA310*s.dpt[3,11])
    OM112 = -qd[1]*S12
    OM312 = qd[1]*C12
    OMp112 = -qd[12]*qd[1]*C12-qdd[1]*S12
    OMp312 = -qd[12]*qd[1]*S12+qdd[1]*C12
    BS112 = -qd[12]*qd[12]-OM312*OM312
    BS212 = qd[12]*OM112
    BS312 = OM112*OM312
    BETA412 = BS212+OMp312
    BETA712 = -qdd[12]+BS312
    ALPHA112 = -ALPHA32*S12+BS12*s.dpt[1,5]*C12
    ALPHA212 = qdd[1]*s.dpt[1,5]
    ALPHA312 = ALPHA32*C12+BS12*s.dpt[1,5]*S12
    OM113 = OM112*C13-OM312*S13
    OM213 = qd[12]+qd[13]
    OM313 = OM112*S13+OM312*C13
    OMp113 = C13*(OMp112-qd[13]*OM312)-S13*(OMp312+qd[13]*OM112)
    OMp213 = qdd[12]+qdd[13]
    OMp313 = C13*(OMp312+qd[13]*OM112)+S13*(OMp112-qd[13]*OM312)
    BS313 = OM113*OM313
    BS613 = OM213*OM313
    BS913 = -OM113*OM113-OM213*OM213
    BETA313 = BS313+OMp213
    BETA613 = BS613-OMp113
    ALPHA113 = C13*(ALPHA112+BS112*s.dpt[1,12])-S13*(ALPHA312+BETA712*s.dpt[1,12])
    ALPHA213 = ALPHA212+BETA412*s.dpt[1,12]
    ALPHA313 = C13*(ALPHA312+BETA712*s.dpt[1,12])+S13*(ALPHA112+BS112*s.dpt[1,12])
    OM114 = OM113*C14-OM313*S14
    OM214 = qd[14]+OM213
    OM314 = OM113*S14+OM313*C14
    OMp114 = C14*(OMp113-qd[14]*OM313)-S14*(OMp313+qd[14]*OM113)
    OMp214 = qdd[14]+OMp213
    OMp314 = C14*(OMp313+qd[14]*OM113)+S14*(OMp113-qd[14]*OM313)
    ALPHA114 = C14*(ALPHA113+BETA313*s.dpt[3,13])-S14*(ALPHA313+BS913*s.dpt[3,13])
    ALPHA214 = ALPHA213+BETA613*s.dpt[3,13]
    ALPHA314 = C14*(ALPHA313+BS913*s.dpt[3,13])+S14*(ALPHA113+BETA313*s.dpt[3,13])
 
# Backward Dynamics

    Fs114 = -s.frc[1,14]+s.m[14]*ALPHA114
    Fs214 = -s.frc[2,14]+s.m[14]*ALPHA214
    Fs314 = -s.frc[3,14]+s.m[14]*ALPHA314
    Cq114 = -s.trq[1,14]+s.In[1,14]*OMp114-s.In[5,14]*OM214*OM314+s.In[9,14]*OM214*OM314
    Cq214 = -s.trq[2,14]+s.In[1,14]*OM114*OM314+s.In[5,14]*OMp214-s.In[9,14]*OM114*OM314
    Cq314 = -s.trq[3,14]-s.In[1,14]*OM114*OM214+s.In[5,14]*OM114*OM214+s.In[9,14]*OMp314
    Fs113 = -s.frc[1,13]+s.m[13]*(ALPHA113+BETA313*s.l[3,13])
    Fs213 = -s.frc[2,13]+s.m[13]*(ALPHA213+BETA613*s.l[3,13])
    Fs313 = -s.frc[3,13]+s.m[13]*(ALPHA313+BS913*s.l[3,13])
    Fq113 = Fs113+Fs114*C14+Fs314*S14
    Fq213 = Fs213+Fs214
    Fq313 = Fs313-Fs114*S14+Fs314*C14
    Cq113 = -s.trq[1,13]+s.In[1,13]*OMp113-s.In[5,13]*OM213*OM313+s.In[9,13]*OM213*OM313+Cq114*C14+Cq314*S14-Fs213* \
 	  s.l[3,13]-Fs214*s.dpt[3,13]
    Cq213 = -s.trq[2,13]+Cq214+s.In[1,13]*OM113*OM313+s.In[5,13]*OMp213-s.In[9,13]*OM113*OM313+Fs113*s.l[3,13]+ \
 	  s.dpt[3,13]*(Fs114*C14+Fs314*S14)
    Cq313 = -s.trq[3,13]-s.In[1,13]*OM113*OM213+s.In[5,13]*OM113*OM213+s.In[9,13]*OMp313-Cq114*S14+Cq314*C14
    Fs112 = -s.frc[1,12]+s.m[12]*(ALPHA112+BS112*s.l[1,12])
    Fs212 = -s.frc[2,12]+s.m[12]*(ALPHA212+BETA412*s.l[1,12])
    Fs312 = -s.frc[3,12]+s.m[12]*(ALPHA312+BETA712*s.l[1,12])
    Fq112 = Fs112+Fq113*C13+Fq313*S13
    Fq212 = Fq213+Fs212
    Fq312 = Fs312-Fq113*S13+Fq313*C13
    Cq112 = -s.trq[1,12]-qd[12]*s.In[5,12]*OM312+qd[12]*s.In[9,12]*OM312+s.In[1,12]*OMp112+Cq113*C13+Cq313*S13
    Cq212 = -s.trq[2,12]+Cq213+qdd[12]*s.In[5,12]+s.In[1,12]*OM112*OM312-s.In[9,12]*OM112*OM312-Fs312*s.l[1,12]- \
 	  s.dpt[1,12]*(-Fq113*S13+Fq313*C13)
    Cq312 = -s.trq[3,12]-qd[12]*s.In[1,12]*OM112+qd[12]*s.In[5,12]*OM112+s.In[9,12]*OMp312-Cq113*S13+Cq313*C13+Fq213* \
 	  s.dpt[1,12]+Fs212*s.l[1,12]
    Fs111 = -s.frc[1,11]+s.m[11]*ALPHA111
    Fs211 = -s.frc[2,11]+s.m[11]*ALPHA211
    Fs311 = -s.frc[3,11]+s.m[11]*ALPHA311
    Cq111 = -s.trq[1,11]+s.In[1,11]*OMp111-s.In[5,11]*OM211*OM311+s.In[9,11]*OM211*OM311
    Cq211 = -s.trq[2,11]+s.In[1,11]*OM111*OM311+s.In[5,11]*OMp211-s.In[9,11]*OM111*OM311
    Cq311 = -s.trq[3,11]-s.In[1,11]*OM111*OM211+s.In[5,11]*OM111*OM211+s.In[9,11]*OMp311
    Fs110 = -s.frc[1,10]+s.m[10]*(ALPHA110+BETA310*s.l[3,10])
    Fs210 = -s.frc[2,10]+s.m[10]*(ALPHA210+BETA610*s.l[3,10])
    Fs310 = -s.frc[3,10]+s.m[10]*(ALPHA310+BS910*s.l[3,10])
    Fq110 = Fs110+Fs111*C11+Fs311*S11
    Fq210 = Fs210+Fs211
    Fq310 = Fs310-Fs111*S11+Fs311*C11
    Cq110 = -s.trq[1,10]+s.In[1,10]*OMp110-s.In[5,10]*OM210*OM310+s.In[9,10]*OM210*OM310+Cq111*C11+Cq311*S11-Fs210* \
 	  s.l[3,10]-Fs211*s.dpt[3,11]
    Cq210 = -s.trq[2,10]+Cq211+s.In[1,10]*OM110*OM310+s.In[5,10]*OMp210-s.In[9,10]*OM110*OM310+Fs110*s.l[3,10]+ \
 	  s.dpt[3,11]*(Fs111*C11+Fs311*S11)
    Cq310 = -s.trq[3,10]-s.In[1,10]*OM110*OM210+s.In[5,10]*OM110*OM210+s.In[9,10]*OMp310-Cq111*S11+Cq311*C11
    Fs19 = -s.frc[1,9]+s.m[9]*(ALPHA19+BS19*s.l[1,9])
    Fs29 = -s.frc[2,9]+s.m[9]*(ALPHA29+BETA49*s.l[1,9])
    Fs39 = -s.frc[3,9]+s.m[9]*(ALPHA39+BETA79*s.l[1,9])
    Fq19 = Fs19+Fq110*C10+Fq310*S10
    Fq29 = Fq210+Fs29
    Fq39 = Fs39-Fq110*S10+Fq310*C10
    Cq19 = -s.trq[1,9]-qd[9]*s.In[5,9]*OM39+qd[9]*s.In[9,9]*OM39+s.In[1,9]*OMp19+Cq110*C10+Cq310*S10
    Cq29 = -s.trq[2,9]+Cq210+qdd[9]*s.In[5,9]+s.In[1,9]*OM19*OM39-s.In[9,9]*OM19*OM39-Fs39*s.l[1,9]-s.dpt[1,10]*(- \
 	  Fq110*S10+Fq310*C10)
    Cq39 = -s.trq[3,9]-qd[9]*s.In[1,9]*OM19+qd[9]*s.In[5,9]*OM19+s.In[9,9]*OMp39-Cq110*S10+Cq310*C10+Fq210* \
 	  s.dpt[1,10]+Fs29*s.l[1,9]
    Fs18 = -s.frc[1,8]+s.m[8]*ALPHA18
    Fs28 = -s.frc[2,8]+s.m[8]*ALPHA28
    Fs38 = -s.frc[3,8]+s.m[8]*ALPHA38
    Cq18 = -s.trq[1,8]+s.In[1,8]*OMp18-s.In[5,8]*OM28*OM38+s.In[9,8]*OM28*OM38
    Cq28 = -s.trq[2,8]+s.In[1,8]*OM18*OM38+s.In[5,8]*OMp28-s.In[9,8]*OM18*OM38
    Cq38 = -s.trq[3,8]-s.In[1,8]*OM18*OM28+s.In[5,8]*OM18*OM28+s.In[9,8]*OMp38
    Fs17 = -s.frc[1,7]+s.m[7]*(ALPHA17+BETA37*s.l[3,7])
    Fs27 = -s.frc[2,7]+s.m[7]*(ALPHA27+BETA67*s.l[3,7])
    Fs37 = -s.frc[3,7]+s.m[7]*(ALPHA37+BS97*s.l[3,7])
    Fq17 = Fs17+Fs18
    Fq27 = Fs27+Fs28*C8-Fs38*S8
    Fq37 = Fs37+Fs28*S8+Fs38*C8
    Cq17 = -s.trq[1,7]+Cq18+s.In[1,7]*OMp17-s.In[5,7]*OM27*OM37+s.In[9,7]*OM27*OM37-Fs27*s.l[3,7]-s.dpt[3,9]*(Fs28*C8 \
 	  -Fs38*S8)
    Cq27 = -s.trq[2,7]+s.In[1,7]*OM17*OM37+s.In[5,7]*OMp27-s.In[9,7]*OM17*OM37+Cq28*C8-Cq38*S8+Fs17*s.l[3,7]+Fs18* \
 	  s.dpt[3,9]
    Cq37 = -s.trq[3,7]-s.In[1,7]*OM17*OM27+s.In[5,7]*OM17*OM27+s.In[9,7]*OMp37+Cq28*S8+Cq38*C8
    Fs16 = -s.frc[1,6]+s.m[6]*(ALPHA16+BETA26*s.l[2,6])
    Fs26 = -s.frc[2,6]+s.m[6]*(ALPHA26+BS56*s.l[2,6])
    Fs36 = -s.frc[3,6]+s.m[6]*(ALPHA36+BETA86*s.l[2,6])
    Fq16 = Fq17+Fs16
    Fq26 = Fs26+Fq27*C7-Fq37*S7
    Fq36 = Fs36+Fq27*S7+Fq37*C7
    Cq16 = -s.trq[1,6]+Cq17+qdd[6]*s.In[1,6]-s.In[5,6]*OM26*OM36+s.In[9,6]*OM26*OM36+Fs36*s.l[2,6]+s.dpt[2,8]*(Fq27* \
 	  S7+Fq37*C7)
    Cq26 = -s.trq[2,6]+qd[6]*s.In[1,6]*OM36-qd[6]*s.In[9,6]*OM36+s.In[5,6]*OMp26+Cq27*C7-Cq37*S7
    Cq36 = -s.trq[3,6]-qd[6]*s.In[1,6]*OM26+qd[6]*s.In[5,6]*OM26+s.In[9,6]*OMp36+Cq27*S7+Cq37*C7-Fq17*s.dpt[2,8]-Fs16 \
 	  *s.l[2,6]
    Fs15 = -s.frc[1,5]+s.m[5]*ALPHA15
    Fs25 = -s.frc[2,5]+s.m[5]*ALPHA25
    Fs35 = -s.frc[3,5]+s.m[5]*ALPHA35
    Cq15 = -s.trq[1,5]+s.In[1,5]*OMp15-s.In[5,5]*OM25*OM35+s.In[9,5]*OM25*OM35
    Cq25 = -s.trq[2,5]+s.In[1,5]*OM15*OM35+s.In[5,5]*OMp25-s.In[9,5]*OM15*OM35
    Cq35 = -s.trq[3,5]-s.In[1,5]*OM15*OM25+s.In[5,5]*OM15*OM25+s.In[9,5]*OMp35
    Fs14 = -s.frc[1,4]+s.m[4]*(ALPHA14+BETA34*s.l[3,4])
    Fs24 = -s.frc[2,4]+s.m[4]*(ALPHA24+BETA64*s.l[3,4])
    Fs34 = -s.frc[3,4]+s.m[4]*(ALPHA34+BS94*s.l[3,4])
    Fq14 = Fs14+Fs15
    Fq24 = Fs24+Fs25*C5-Fs35*S5
    Fq34 = Fs34+Fs25*S5+Fs35*C5
    Cq14 = -s.trq[1,4]+Cq15+s.In[1,4]*OMp14-s.In[5,4]*OM24*OM34+s.In[9,4]*OM24*OM34-Fs24*s.l[3,4]-s.dpt[3,7]*(Fs25*C5 \
 	  -Fs35*S5)
    Cq24 = -s.trq[2,4]+s.In[1,4]*OM14*OM34+s.In[5,4]*OMp24-s.In[9,4]*OM14*OM34+Cq25*C5-Cq35*S5+Fs14*s.l[3,4]+Fs15* \
 	  s.dpt[3,7]
    Cq34 = -s.trq[3,4]-s.In[1,4]*OM14*OM24+s.In[5,4]*OM14*OM24+s.In[9,4]*OMp34+Cq25*S5+Cq35*C5
    Fs13 = -s.frc[1,3]+s.m[3]*(ALPHA13+BETA23*s.l[2,3])
    Fs23 = -s.frc[2,3]+s.m[3]*(ALPHA23+BS53*s.l[2,3])
    Fs33 = -s.frc[3,3]+s.m[3]*(ALPHA33+BETA83*s.l[2,3])
    Fq13 = Fq14+Fs13
    Fq23 = Fs23+Fq24*C4-Fq34*S4
    Fq33 = Fs33+Fq24*S4+Fq34*C4
    Cq13 = -s.trq[1,3]+Cq14+qdd[3]*s.In[1,3]-s.In[5,3]*OM23*OM33+s.In[9,3]*OM23*OM33+Fs33*s.l[2,3]+s.dpt[2,6]*(Fq24* \
 	  S4+Fq34*C4)
    Cq23 = -s.trq[2,3]+qd[3]*s.In[1,3]*OM33-qd[3]*s.In[9,3]*OM33+s.In[5,3]*OMp23+Cq24*C4-Cq34*S4
    Cq33 = -s.trq[3,3]-qd[3]*s.In[1,3]*OM23+qd[3]*s.In[5,3]*OM23+s.In[9,3]*OMp33+Cq24*S4+Cq34*C4-Fq14*s.dpt[2,6]-Fs13 \
 	  *s.l[2,3]
    Fs32 = -s.frc[3,2]+s.m[2]*ALPHA32
    Fq32 = Fs32-Fq112*S12-Fq19*S9+Fq23*S3+Fq26*S6+Fq312*C12+Fq33*C3+Fq36*C6+Fq39*C9
    Cq32 = -s.trq[3,2]+qdd[1]*s.In[9,2]-Cq112*S12-Cq19*S9+Cq23*S3+Cq26*S6+Cq312*C12+Cq33*C3+Cq36*C6+Cq39*C9-Fq13* \
 	  s.dpt[2,2]-Fq16*s.dpt[2,3]+Fq212*s.dpt[1,5]+Fq29*s.dpt[1,4]
    Cq31 = -s.trq[3,1]+Cq32+qdd[1]*s.In[9,1]
 
# Symbolic model output

    Qq[1] = Cq31
    Qq[2] = Fq32
    Qq[3] = Cq13
    Qq[4] = Cq14
    Qq[5] = Cq15
    Qq[6] = Cq16
    Qq[7] = Cq17
    Qq[8] = Cq18
    Qq[9] = Cq29
    Qq[10] = Cq210
    Qq[11] = Cq211
    Qq[12] = Cq212
    Qq[13] = Cq213
    Qq[14] = Cq214

# Number of continuation lines = 1


