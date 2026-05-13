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
#	==> Function: F1 - Recursive Direct Dynamics of tree-like MBS
#
#	==> Git hash: 0e4e6a608eeee06956095d2f2ad315abdb092777
#
##

from math import sin, cos

def dirdyna(M, c, s, tsim):
    q = s.q
    qd = s.qd
 
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
    S3p4 = C3*S4+S3*C4
    C3p4 = C3*C4-S3*S4
    S3p4p5 = C3p4*S5+S3p4*C5
    C3p4p5 = C3p4*C5-S3p4*S5
    S6p7 = C6*S7+S6*C7
    C6p7 = C6*C7-S6*S7
    S6p7p8 = C6p7*S8+S6p7*C8
    C6p7p8 = C6p7*C8-S6p7*S8
    S10p9 = C10*S9+S10*C9
    C10p9 = C10*C9-S10*S9
    S10p9p11 = C10p9*S11+S10p9*C11
    C10p9p11 = C10p9*C11-S10p9*S11
    S12p13 = C12*S13+S12*C13
    C12p13 = C12*C13-S12*S13
    S12p13p14 = C12p13*S14+S12p13*C14
    C12p13p14 = C12p13*C14-S12p13*S14
 
# Augmented Joint Position Vectors

    Dz23 = q[2]+s.dpt[3,1]
 
# Forward Kinematics

    BS12 = -qd[1]*qd[1]
    BS52 = -qd[1]*qd[1]
    OM23 = qd[1]*S3
    OM33 = qd[1]*C3
    OA23 = qd[1]*qd[3]*C3
    OA33 = -qd[1]*qd[3]*S3
    BS23 = qd[3]*OM23
    BS53 = -qd[3]*qd[3]-OM33*OM33
    BS63 = OM23*OM33
    BEF23 = BS23-OA33
    AF23 = BS52*s.dpt[2,2]*C3-s.g[3]*S3
    AF33 = -BS52*s.dpt[2,2]*S3-s.g[3]*C3
    OM14 = qd[3]+qd[4]
    OM24 = OM23*C4+OM33*S4
    OM34 = -OM23*S4+OM33*C4
    OA24 = C4*(OA23+qd[4]*OM33)+S4*(OA33-qd[4]*OM23)
    OA34 = C4*(OA33-qd[4]*OM23)-S4*(OA23+qd[4]*OM33)
    BS34 = OM14*OM34
    BS64 = OM24*OM34
    BS94 = -OM14*OM14-OM24*OM24
    BEF34 = BS34+OA24
    AF14 = BEF23*s.dpt[2,6]
    AF24 = C4*(AF23+BS53*s.dpt[2,6])+S4*(AF33+BS63*s.dpt[2,6])
    AF34 = C4*(AF33+BS63*s.dpt[2,6])-S4*(AF23+BS53*s.dpt[2,6])
    AM14_1 = -s.dpt[2,2]-s.dpt[2,6]*C3
    AM24_2 = C3*S4+S3*C4
    AM34_2 = C3*C4-S3*S4
    AM24_3 = s.dpt[2,6]*S4
    AM34_3 = s.dpt[2,6]*C4
    OM15 = qd[5]+OM14
    OM25 = OM24*C5+OM34*S5
    OM35 = -OM24*S5+OM34*C5
    OA25 = C5*(OA24+qd[5]*OM34)+S5*(OA34-qd[5]*OM24)
    OA35 = C5*(OA34-qd[5]*OM24)-S5*(OA24+qd[5]*OM34)
    AF15 = AF14+BEF34*s.dpt[3,7]
    AF25 = C5*(AF24+BS64*s.dpt[3,7])+S5*(AF34+BS94*s.dpt[3,7])
    AF35 = C5*(AF34+BS94*s.dpt[3,7])-S5*(AF24+BS64*s.dpt[3,7])
    AM15_1 = AM14_1+s.dpt[3,7]*S3p4
    AM25_2 = AM24_2*C5+AM34_2*S5
    AM35_2 = -AM24_2*S5+AM34_2*C5
    AM25_3 = AM34_3*S5+C5*(AM24_3-s.dpt[3,7])
    AM35_3 = AM34_3*C5-S5*(AM24_3-s.dpt[3,7])
    AM25_4 = -s.dpt[3,7]*C5
    AM35_4 = s.dpt[3,7]*S5
    OM26 = qd[1]*S6
    OM36 = qd[1]*C6
    OA26 = qd[1]*qd[6]*C6
    OA36 = -qd[1]*qd[6]*S6
    BS26 = qd[6]*OM26
    BS56 = -qd[6]*qd[6]-OM36*OM36
    BS66 = OM26*OM36
    BEF26 = BS26-OA36
    AF26 = BS52*s.dpt[2,3]*C6-s.g[3]*S6
    AF36 = -BS52*s.dpt[2,3]*S6-s.g[3]*C6
    OM17 = qd[6]+qd[7]
    OM27 = OM26*C7+OM36*S7
    OM37 = -OM26*S7+OM36*C7
    OA27 = C7*(OA26+qd[7]*OM36)+S7*(OA36-qd[7]*OM26)
    OA37 = C7*(OA36-qd[7]*OM26)-S7*(OA26+qd[7]*OM36)
    BS37 = OM17*OM37
    BS67 = OM27*OM37
    BS97 = -OM17*OM17-OM27*OM27
    BEF37 = BS37+OA27
    AF17 = BEF26*s.dpt[2,8]
    AF27 = C7*(AF26+BS56*s.dpt[2,8])+S7*(AF36+BS66*s.dpt[2,8])
    AF37 = C7*(AF36+BS66*s.dpt[2,8])-S7*(AF26+BS56*s.dpt[2,8])
    AM17_1 = -s.dpt[2,3]-s.dpt[2,8]*C6
    AM27_2 = C6*S7+S6*C7
    AM37_2 = C6*C7-S6*S7
    AM27_6 = s.dpt[2,8]*S7
    AM37_6 = s.dpt[2,8]*C7
    OM18 = qd[8]+OM17
    OM28 = OM27*C8+OM37*S8
    OM38 = -OM27*S8+OM37*C8
    OA28 = C8*(OA27+qd[8]*OM37)+S8*(OA37-qd[8]*OM27)
    OA38 = C8*(OA37-qd[8]*OM27)-S8*(OA27+qd[8]*OM37)
    AF18 = AF17+BEF37*s.dpt[3,9]
    AF28 = C8*(AF27+BS67*s.dpt[3,9])+S8*(AF37+BS97*s.dpt[3,9])
    AF38 = C8*(AF37+BS97*s.dpt[3,9])-S8*(AF27+BS67*s.dpt[3,9])
    AM18_1 = AM17_1+s.dpt[3,9]*S6p7
    AM28_2 = AM27_2*C8+AM37_2*S8
    AM38_2 = -AM27_2*S8+AM37_2*C8
    AM28_6 = AM37_6*S8+C8*(AM27_6-s.dpt[3,9])
    AM38_6 = AM37_6*C8-S8*(AM27_6-s.dpt[3,9])
    AM28_7 = -s.dpt[3,9]*C8
    AM38_7 = s.dpt[3,9]*S8
    OM19 = -qd[1]*S9
    OM39 = qd[1]*C9
    OA19 = -qd[1]*qd[9]*C9
    OA39 = -qd[1]*qd[9]*S9
    BS19 = -qd[9]*qd[9]-OM39*OM39
    BS29 = qd[9]*OM19
    BS39 = OM19*OM39
    BEF49 = BS29+OA39
    AF19 = BS12*s.dpt[1,4]*C9+s.g[3]*S9
    AF39 = BS12*s.dpt[1,4]*S9-s.g[3]*C9
    OM110 = OM19*C10-OM39*S10
    OM210 = qd[10]+qd[9]
    OM310 = OM19*S10+OM39*C10
    OA110 = C10*(OA19-qd[10]*OM39)-S10*(OA39+qd[10]*OM19)
    OA310 = C10*(OA39+qd[10]*OM19)+S10*(OA19-qd[10]*OM39)
    BS310 = OM110*OM310
    BS610 = OM210*OM310
    BS910 = -OM110*OM110-OM210*OM210
    BEF610 = BS610-OA110
    AF110 = C10*(AF19+BS19*s.dpt[1,10])-S10*(AF39+BS39*s.dpt[1,10])
    AF210 = BEF49*s.dpt[1,10]
    AF310 = C10*(AF39+BS39*s.dpt[1,10])+S10*(AF19+BS19*s.dpt[1,10])
    AM210_1 = s.dpt[1,4]+s.dpt[1,10]*C9
    AM110_2 = -C10*S9-S10*C9
    AM310_2 = C10*C9-S10*S9
    AM110_9 = s.dpt[1,10]*S10
    AM310_9 = -s.dpt[1,10]*C10
    OM111 = OM110*C11-OM310*S11
    OM211 = qd[11]+OM210
    OM311 = OM110*S11+OM310*C11
    OA111 = C11*(OA110-qd[11]*OM310)-S11*(OA310+qd[11]*OM110)
    OA311 = C11*(OA310+qd[11]*OM110)+S11*(OA110-qd[11]*OM310)
    AF111 = C11*(AF110+BS310*s.dpt[3,11])-S11*(AF310+BS910*s.dpt[3,11])
    AF211 = AF210+BEF610*s.dpt[3,11]
    AF311 = C11*(AF310+BS910*s.dpt[3,11])+S11*(AF110+BS310*s.dpt[3,11])
    AM211_1 = AM210_1+s.dpt[3,11]*S10p9
    AM111_2 = AM110_2*C11-AM310_2*S11
    AM311_2 = AM110_2*S11+AM310_2*C11
    AM111_9 = -AM310_9*S11+C11*(AM110_9+s.dpt[3,11])
    AM311_9 = AM310_9*C11+S11*(AM110_9+s.dpt[3,11])
    AM111_10 = s.dpt[3,11]*C11
    AM311_10 = s.dpt[3,11]*S11
    OM112 = -qd[1]*S12
    OM312 = qd[1]*C12
    OA112 = -qd[12]*qd[1]*C12
    OA312 = -qd[12]*qd[1]*S12
    BS112 = -qd[12]*qd[12]-OM312*OM312
    BS212 = qd[12]*OM112
    BS312 = OM112*OM312
    BEF412 = BS212+OA312
    AF112 = BS12*s.dpt[1,5]*C12+s.g[3]*S12
    AF312 = BS12*s.dpt[1,5]*S12-s.g[3]*C12
    OM113 = OM112*C13-OM312*S13
    OM213 = qd[12]+qd[13]
    OM313 = OM112*S13+OM312*C13
    OA113 = C13*(OA112-qd[13]*OM312)-S13*(OA312+qd[13]*OM112)
    OA313 = C13*(OA312+qd[13]*OM112)+S13*(OA112-qd[13]*OM312)
    BS313 = OM113*OM313
    BS613 = OM213*OM313
    BS913 = -OM113*OM113-OM213*OM213
    BEF613 = BS613-OA113
    AF113 = C13*(AF112+BS112*s.dpt[1,12])-S13*(AF312+BS312*s.dpt[1,12])
    AF213 = BEF412*s.dpt[1,12]
    AF313 = C13*(AF312+BS312*s.dpt[1,12])+S13*(AF112+BS112*s.dpt[1,12])
    AM213_1 = s.dpt[1,5]+s.dpt[1,12]*C12
    AM113_2 = -C12*S13-S12*C13
    AM313_2 = C12*C13-S12*S13
    AM113_12 = s.dpt[1,12]*S13
    AM313_12 = -s.dpt[1,12]*C13
    OM114 = OM113*C14-OM313*S14
    OM214 = qd[14]+OM213
    OM314 = OM113*S14+OM313*C14
    OA114 = C14*(OA113-qd[14]*OM313)-S14*(OA313+qd[14]*OM113)
    OA314 = C14*(OA313+qd[14]*OM113)+S14*(OA113-qd[14]*OM313)
    AF114 = C14*(AF113+BS313*s.dpt[3,13])-S14*(AF313+BS913*s.dpt[3,13])
    AF214 = AF213+BEF613*s.dpt[3,13]
    AF314 = C14*(AF313+BS913*s.dpt[3,13])+S14*(AF113+BS313*s.dpt[3,13])
    AM214_1 = AM213_1+s.dpt[3,13]*S12p13
    AM114_2 = AM113_2*C14-AM313_2*S14
    AM314_2 = AM113_2*S14+AM313_2*C14
    AM114_12 = -AM313_12*S14+C14*(AM113_12+s.dpt[3,13])
    AM314_12 = AM313_12*C14+S14*(AM113_12+s.dpt[3,13])
    AM114_13 = s.dpt[3,13]*C14
    AM314_13 = s.dpt[3,13]*S14
 
# Backward Dynamics

    FA114 = -s.frc[1,14]+s.m[14]*AF114
    FA214 = -s.frc[2,14]+s.m[14]*AF214
    FA314 = -s.frc[3,14]+s.m[14]*AF314
    CF114 = -s.trq[1,14]+s.In[1,14]*OA114-s.In[5,14]*OM214*OM314+s.In[9,14]*OM214*OM314
    CF214 = -s.trq[2,14]+s.In[1,14]*OM114*OM314-s.In[9,14]*OM114*OM314
    CF314 = -s.trq[3,14]-s.In[1,14]*OM114*OM214+s.In[5,14]*OM114*OM214+s.In[9,14]*OA314
    FB214_1 = s.m[14]*AM214_1
    CM114_1 = -s.In[1,14]*S12p13p14
    CM314_1 = s.In[9,14]*C12p13p14
    FB114_2 = s.m[14]*AM114_2
    FB314_2 = s.m[14]*AM314_2
    FB114_12 = s.m[14]*AM114_12
    FB314_12 = s.m[14]*AM314_12
    FB114_13 = s.m[14]*AM114_13
    FB314_13 = s.m[14]*AM314_13
    FA113 = -s.frc[1,13]+s.m[13]*(AF113+BS313*s.l[3,13])
    FA213 = -s.frc[2,13]+s.m[13]*(AF213+BEF613*s.l[3,13])
    FA313 = -s.frc[3,13]+s.m[13]*(AF313+BS913*s.l[3,13])
    FF113 = FA113+FA114*C14+FA314*S14
    FF213 = FA213+FA214
    FF313 = FA313-FA114*S14+FA314*C14
    CF113 = -s.trq[1,13]+s.In[1,13]*OA113-s.In[5,13]*OM213*OM313+s.In[9,13]*OM213*OM313+CF114*C14+CF314*S14-FA213* \
 	  s.l[3,13]-FA214*s.dpt[3,13]
    CF213 = -s.trq[2,13]+CF214+s.In[1,13]*OM113*OM313-s.In[9,13]*OM113*OM313+FA113*s.l[3,13]+s.dpt[3,13]*(FA114*C14+ \
 	  FA314*S14)
    CF313 = -s.trq[3,13]-s.In[1,13]*OM113*OM213+s.In[5,13]*OM113*OM213+s.In[9,13]*OA313-CF114*S14+CF314*C14
    FB213_1 = s.m[13]*(AM213_1+s.l[3,13]*S12p13)
    FM213_1 = FB213_1+FB214_1
    CM113_1 = -s.In[1,13]*S12p13+CM114_1*C14+CM314_1*S14-FB213_1*s.l[3,13]-FB214_1*s.dpt[3,13]
    CM313_1 = s.In[9,13]*C12p13-CM114_1*S14+CM314_1*C14
    FB113_2 = s.m[13]*AM113_2
    FB313_2 = s.m[13]*AM313_2
    FM113_2 = FB113_2+FB114_2*C14+FB314_2*S14
    FM313_2 = FB313_2-FB114_2*S14+FB314_2*C14
    CM213_2 = FB113_2*s.l[3,13]+s.dpt[3,13]*(FB114_2*C14+FB314_2*S14)
    FB113_12 = s.m[13]*(AM113_12+s.l[3,13])
    FB313_12 = s.m[13]*AM313_12
    FM113_12 = FB113_12+FB114_12*C14+FB314_12*S14
    FM313_12 = FB313_12-FB114_12*S14+FB314_12*C14
    CM213_12 = s.In[5,13]+s.In[5,14]+FB113_12*s.l[3,13]+s.dpt[3,13]*(FB114_12*C14+FB314_12*S14)
    FB113_13 = s.m[13]*s.l[3,13]
    CM213_13 = s.In[5,13]+s.In[5,14]+FB113_13*s.l[3,13]+s.dpt[3,13]*(FB114_13*C14+FB314_13*S14)
    FA112 = -s.frc[1,12]+s.m[12]*(AF112+BS112*s.l[1,12])
    FA212 = -s.frc[2,12]+s.m[12]*BEF412*s.l[1,12]
    FA312 = -s.frc[3,12]+s.m[12]*(AF312+BS312*s.l[1,12])
    FF112 = FA112+FF113*C13+FF313*S13
    FF212 = FA212+FF213
    FF312 = FA312-FF113*S13+FF313*C13
    CF112 = -s.trq[1,12]-qd[12]*s.In[5,12]*OM312+qd[12]*s.In[9,12]*OM312+s.In[1,12]*OA112+CF113*C13+CF313*S13
    CF212 = -s.trq[2,12]+CF213+s.In[1,12]*OM112*OM312-s.In[9,12]*OM112*OM312-FA312*s.l[1,12]-s.dpt[1,12]*(-FF113*S13+ \
 	  FF313*C13)
    CF312 = -s.trq[3,12]-qd[12]*s.In[1,12]*OM112+qd[12]*s.In[5,12]*OM112+s.In[9,12]*OA312-CF113*S13+CF313*C13+FA212* \
 	  s.l[1,12]+FF213*s.dpt[1,12]
    FB212_1 = s.m[12]*(s.dpt[1,5]+s.l[1,12]*C12)
    FM212_1 = FB212_1+FM213_1
    CM112_1 = -s.In[1,12]*S12+CM113_1*C13+CM313_1*S13
    CM312_1 = s.In[9,12]*C12-CM113_1*S13+CM313_1*C13+FB212_1*s.l[1,12]+FM213_1*s.dpt[1,12]
    FB112_2 = -s.m[12]*S12
    FB312_2 = s.m[12]*C12
    FM112_2 = FB112_2+FM113_2*C13+FM313_2*S13
    FM312_2 = FB312_2-FM113_2*S13+FM313_2*C13
    CM212_2 = CM213_2-FB312_2*s.l[1,12]-s.dpt[1,12]*(-FM113_2*S13+FM313_2*C13)
    FB312_12 = -s.m[12]*s.l[1,12]
    CM212_12 = s.In[5,12]+CM213_12-FB312_12*s.l[1,12]-s.dpt[1,12]*(-FM113_12*S13+FM313_12*C13)
    FA111 = -s.frc[1,11]+s.m[11]*AF111
    FA211 = -s.frc[2,11]+s.m[11]*AF211
    FA311 = -s.frc[3,11]+s.m[11]*AF311
    CF111 = -s.trq[1,11]+s.In[1,11]*OA111-s.In[5,11]*OM211*OM311+s.In[9,11]*OM211*OM311
    CF211 = -s.trq[2,11]+s.In[1,11]*OM111*OM311-s.In[9,11]*OM111*OM311
    CF311 = -s.trq[3,11]-s.In[1,11]*OM111*OM211+s.In[5,11]*OM111*OM211+s.In[9,11]*OA311
    FB211_1 = s.m[11]*AM211_1
    CM111_1 = -s.In[1,11]*S10p9p11
    CM311_1 = s.In[9,11]*C10p9p11
    FB111_2 = s.m[11]*AM111_2
    FB311_2 = s.m[11]*AM311_2
    FB111_9 = s.m[11]*AM111_9
    FB311_9 = s.m[11]*AM311_9
    FB111_10 = s.m[11]*AM111_10
    FB311_10 = s.m[11]*AM311_10
    FA110 = -s.frc[1,10]+s.m[10]*(AF110+BS310*s.l[3,10])
    FA210 = -s.frc[2,10]+s.m[10]*(AF210+BEF610*s.l[3,10])
    FA310 = -s.frc[3,10]+s.m[10]*(AF310+BS910*s.l[3,10])
    FF110 = FA110+FA111*C11+FA311*S11
    FF210 = FA210+FA211
    FF310 = FA310-FA111*S11+FA311*C11
    CF110 = -s.trq[1,10]+s.In[1,10]*OA110-s.In[5,10]*OM210*OM310+s.In[9,10]*OM210*OM310+CF111*C11+CF311*S11-FA210* \
 	  s.l[3,10]-FA211*s.dpt[3,11]
    CF210 = -s.trq[2,10]+CF211+s.In[1,10]*OM110*OM310-s.In[9,10]*OM110*OM310+FA110*s.l[3,10]+s.dpt[3,11]*(FA111*C11+ \
 	  FA311*S11)
    CF310 = -s.trq[3,10]-s.In[1,10]*OM110*OM210+s.In[5,10]*OM110*OM210+s.In[9,10]*OA310-CF111*S11+CF311*C11
    FB210_1 = s.m[10]*(AM210_1+s.l[3,10]*S10p9)
    FM210_1 = FB210_1+FB211_1
    CM110_1 = -s.In[1,10]*S10p9+CM111_1*C11+CM311_1*S11-FB210_1*s.l[3,10]-FB211_1*s.dpt[3,11]
    CM310_1 = s.In[9,10]*C10p9-CM111_1*S11+CM311_1*C11
    FB110_2 = s.m[10]*AM110_2
    FB310_2 = s.m[10]*AM310_2
    FM110_2 = FB110_2+FB111_2*C11+FB311_2*S11
    FM310_2 = FB310_2-FB111_2*S11+FB311_2*C11
    CM210_2 = FB110_2*s.l[3,10]+s.dpt[3,11]*(FB111_2*C11+FB311_2*S11)
    FB110_9 = s.m[10]*(AM110_9+s.l[3,10])
    FB310_9 = s.m[10]*AM310_9
    FM110_9 = FB110_9+FB111_9*C11+FB311_9*S11
    FM310_9 = FB310_9-FB111_9*S11+FB311_9*C11
    CM210_9 = s.In[5,10]+s.In[5,11]+FB110_9*s.l[3,10]+s.dpt[3,11]*(FB111_9*C11+FB311_9*S11)
    FB110_10 = s.m[10]*s.l[3,10]
    CM210_10 = s.In[5,10]+s.In[5,11]+FB110_10*s.l[3,10]+s.dpt[3,11]*(FB111_10*C11+FB311_10*S11)
    FA19 = -s.frc[1,9]+s.m[9]*(AF19+BS19*s.l[1,9])
    FA29 = -s.frc[2,9]+s.m[9]*BEF49*s.l[1,9]
    FA39 = -s.frc[3,9]+s.m[9]*(AF39+BS39*s.l[1,9])
    FF19 = FA19+FF110*C10+FF310*S10
    FF29 = FA29+FF210
    FF39 = FA39-FF110*S10+FF310*C10
    CF19 = -s.trq[1,9]-qd[9]*s.In[5,9]*OM39+qd[9]*s.In[9,9]*OM39+s.In[1,9]*OA19+CF110*C10+CF310*S10
    CF29 = -s.trq[2,9]+CF210+s.In[1,9]*OM19*OM39-s.In[9,9]*OM19*OM39-FA39*s.l[1,9]-s.dpt[1,10]*(-FF110*S10+FF310*C10)
    CF39 = -s.trq[3,9]-qd[9]*s.In[1,9]*OM19+qd[9]*s.In[5,9]*OM19+s.In[9,9]*OA39-CF110*S10+CF310*C10+FA29*s.l[1,9]+ \
 	  FF210*s.dpt[1,10]
    FB29_1 = s.m[9]*(s.dpt[1,4]+s.l[1,9]*C9)
    FM29_1 = FB29_1+FM210_1
    CM19_1 = -s.In[1,9]*S9+CM110_1*C10+CM310_1*S10
    CM39_1 = s.In[9,9]*C9-CM110_1*S10+CM310_1*C10+FB29_1*s.l[1,9]+FM210_1*s.dpt[1,10]
    FB19_2 = -s.m[9]*S9
    FB39_2 = s.m[9]*C9
    FM19_2 = FB19_2+FM110_2*C10+FM310_2*S10
    FM39_2 = FB39_2-FM110_2*S10+FM310_2*C10
    CM29_2 = CM210_2-FB39_2*s.l[1,9]-s.dpt[1,10]*(-FM110_2*S10+FM310_2*C10)
    FB39_9 = -s.m[9]*s.l[1,9]
    CM29_9 = s.In[5,9]+CM210_9-FB39_9*s.l[1,9]-s.dpt[1,10]*(-FM110_9*S10+FM310_9*C10)
    FA18 = -s.frc[1,8]+s.m[8]*AF18
    FA28 = -s.frc[2,8]+s.m[8]*AF28
    FA38 = -s.frc[3,8]+s.m[8]*AF38
    CF18 = -s.trq[1,8]-s.In[5,8]*OM28*OM38+s.In[9,8]*OM28*OM38
    CF28 = -s.trq[2,8]+s.In[1,8]*OM18*OM38+s.In[5,8]*OA28-s.In[9,8]*OM18*OM38
    CF38 = -s.trq[3,8]-s.In[1,8]*OM18*OM28+s.In[5,8]*OM18*OM28+s.In[9,8]*OA38
    FB18_1 = s.m[8]*AM18_1
    CM28_1 = s.In[5,8]*S6p7p8
    CM38_1 = s.In[9,8]*C6p7p8
    FB28_2 = s.m[8]*AM28_2
    FB38_2 = s.m[8]*AM38_2
    FB28_6 = s.m[8]*AM28_6
    FB38_6 = s.m[8]*AM38_6
    FB28_7 = s.m[8]*AM28_7
    FB38_7 = s.m[8]*AM38_7
    FA17 = -s.frc[1,7]+s.m[7]*(AF17+BEF37*s.l[3,7])
    FA27 = -s.frc[2,7]+s.m[7]*(AF27+BS67*s.l[3,7])
    FA37 = -s.frc[3,7]+s.m[7]*(AF37+BS97*s.l[3,7])
    FF17 = FA17+FA18
    FF27 = FA27+FA28*C8-FA38*S8
    FF37 = FA37+FA28*S8+FA38*C8
    CF17 = -s.trq[1,7]+CF18-s.In[5,7]*OM27*OM37+s.In[9,7]*OM27*OM37-FA27*s.l[3,7]-s.dpt[3,9]*(FA28*C8-FA38*S8)
    CF27 = -s.trq[2,7]+s.In[1,7]*OM17*OM37+s.In[5,7]*OA27-s.In[9,7]*OM17*OM37+CF28*C8-CF38*S8+FA17*s.l[3,7]+FA18* \
 	  s.dpt[3,9]
    CF37 = -s.trq[3,7]-s.In[1,7]*OM17*OM27+s.In[5,7]*OM17*OM27+s.In[9,7]*OA37+CF28*S8+CF38*C8
    FB17_1 = s.m[7]*(AM17_1+s.l[3,7]*S6p7)
    FM17_1 = FB17_1+FB18_1
    CM27_1 = s.In[5,7]*S6p7+CM28_1*C8-CM38_1*S8+FB17_1*s.l[3,7]+FB18_1*s.dpt[3,9]
    CM37_1 = s.In[9,7]*C6p7+CM28_1*S8+CM38_1*C8
    FB27_2 = s.m[7]*AM27_2
    FB37_2 = s.m[7]*AM37_2
    FM27_2 = FB27_2+FB28_2*C8-FB38_2*S8
    FM37_2 = FB37_2+FB28_2*S8+FB38_2*C8
    CM17_2 = -FB27_2*s.l[3,7]-s.dpt[3,9]*(FB28_2*C8-FB38_2*S8)
    FB27_6 = s.m[7]*(AM27_6-s.l[3,7])
    FB37_6 = s.m[7]*AM37_6
    FM27_6 = FB27_6+FB28_6*C8-FB38_6*S8
    FM37_6 = FB37_6+FB28_6*S8+FB38_6*C8
    CM17_6 = s.In[1,7]+s.In[1,8]-FB27_6*s.l[3,7]-s.dpt[3,9]*(FB28_6*C8-FB38_6*S8)
    FB27_7 = -s.m[7]*s.l[3,7]
    CM17_7 = s.In[1,7]+s.In[1,8]-FB27_7*s.l[3,7]-s.dpt[3,9]*(FB28_7*C8-FB38_7*S8)
    FA16 = -s.frc[1,6]+s.m[6]*BEF26*s.l[2,6]
    FA26 = -s.frc[2,6]+s.m[6]*(AF26+BS56*s.l[2,6])
    FA36 = -s.frc[3,6]+s.m[6]*(AF36+BS66*s.l[2,6])
    FF16 = FA16+FF17
    FF26 = FA26+FF27*C7-FF37*S7
    FF36 = FA36+FF27*S7+FF37*C7
    CF16 = -s.trq[1,6]+CF17-s.In[5,6]*OM26*OM36+s.In[9,6]*OM26*OM36+FA36*s.l[2,6]+s.dpt[2,8]*(FF27*S7+FF37*C7)
    CF26 = -s.trq[2,6]+qd[6]*s.In[1,6]*OM36-qd[6]*s.In[9,6]*OM36+s.In[5,6]*OA26+CF27*C7-CF37*S7
    CF36 = -s.trq[3,6]-qd[6]*s.In[1,6]*OM26+qd[6]*s.In[5,6]*OM26+s.In[9,6]*OA36+CF27*S7+CF37*C7-FA16*s.l[2,6]-FF17* \
 	  s.dpt[2,8]
    FB16_1 = s.m[6]*(-s.dpt[2,3]-s.l[2,6]*C6)
    FM16_1 = FB16_1+FM17_1
    CM26_1 = s.In[5,6]*S6+CM27_1*C7-CM37_1*S7
    CM36_1 = s.In[9,6]*C6+CM27_1*S7+CM37_1*C7-FB16_1*s.l[2,6]-FM17_1*s.dpt[2,8]
    FB26_2 = s.m[6]*S6
    FB36_2 = s.m[6]*C6
    FM26_2 = FB26_2+FM27_2*C7-FM37_2*S7
    FM36_2 = FB36_2+FM27_2*S7+FM37_2*C7
    CM16_2 = CM17_2+FB36_2*s.l[2,6]+s.dpt[2,8]*(FM27_2*S7+FM37_2*C7)
    FB36_6 = s.m[6]*s.l[2,6]
    CM16_6 = s.In[1,6]+CM17_6+FB36_6*s.l[2,6]+s.dpt[2,8]*(FM27_6*S7+FM37_6*C7)
    FA15 = -s.frc[1,5]+s.m[5]*AF15
    FA25 = -s.frc[2,5]+s.m[5]*AF25
    FA35 = -s.frc[3,5]+s.m[5]*AF35
    CF15 = -s.trq[1,5]-s.In[5,5]*OM25*OM35+s.In[9,5]*OM25*OM35
    CF25 = -s.trq[2,5]+s.In[1,5]*OM15*OM35+s.In[5,5]*OA25-s.In[9,5]*OM15*OM35
    CF35 = -s.trq[3,5]-s.In[1,5]*OM15*OM25+s.In[5,5]*OM15*OM25+s.In[9,5]*OA35
    FB15_1 = s.m[5]*AM15_1
    CM25_1 = s.In[5,5]*S3p4p5
    CM35_1 = s.In[9,5]*C3p4p5
    FB25_2 = s.m[5]*AM25_2
    FB35_2 = s.m[5]*AM35_2
    FB25_3 = s.m[5]*AM25_3
    FB35_3 = s.m[5]*AM35_3
    FB25_4 = s.m[5]*AM25_4
    FB35_4 = s.m[5]*AM35_4
    FA14 = -s.frc[1,4]+s.m[4]*(AF14+BEF34*s.l[3,4])
    FA24 = -s.frc[2,4]+s.m[4]*(AF24+BS64*s.l[3,4])
    FA34 = -s.frc[3,4]+s.m[4]*(AF34+BS94*s.l[3,4])
    FF14 = FA14+FA15
    FF24 = FA24+FA25*C5-FA35*S5
    FF34 = FA34+FA25*S5+FA35*C5
    CF14 = -s.trq[1,4]+CF15-s.In[5,4]*OM24*OM34+s.In[9,4]*OM24*OM34-FA24*s.l[3,4]-s.dpt[3,7]*(FA25*C5-FA35*S5)
    CF24 = -s.trq[2,4]+s.In[1,4]*OM14*OM34+s.In[5,4]*OA24-s.In[9,4]*OM14*OM34+CF25*C5-CF35*S5+FA14*s.l[3,4]+FA15* \
 	  s.dpt[3,7]
    CF34 = -s.trq[3,4]-s.In[1,4]*OM14*OM24+s.In[5,4]*OM14*OM24+s.In[9,4]*OA34+CF25*S5+CF35*C5
    FB14_1 = s.m[4]*(AM14_1+s.l[3,4]*S3p4)
    FM14_1 = FB14_1+FB15_1
    CM24_1 = s.In[5,4]*S3p4+CM25_1*C5-CM35_1*S5+FB14_1*s.l[3,4]+FB15_1*s.dpt[3,7]
    CM34_1 = s.In[9,4]*C3p4+CM25_1*S5+CM35_1*C5
    FB24_2 = s.m[4]*AM24_2
    FB34_2 = s.m[4]*AM34_2
    FM24_2 = FB24_2+FB25_2*C5-FB35_2*S5
    FM34_2 = FB34_2+FB25_2*S5+FB35_2*C5
    CM14_2 = -FB24_2*s.l[3,4]-s.dpt[3,7]*(FB25_2*C5-FB35_2*S5)
    FB24_3 = s.m[4]*(AM24_3-s.l[3,4])
    FB34_3 = s.m[4]*AM34_3
    FM24_3 = FB24_3+FB25_3*C5-FB35_3*S5
    FM34_3 = FB34_3+FB25_3*S5+FB35_3*C5
    CM14_3 = s.In[1,4]+s.In[1,5]-FB24_3*s.l[3,4]-s.dpt[3,7]*(FB25_3*C5-FB35_3*S5)
    FB24_4 = -s.m[4]*s.l[3,4]
    CM14_4 = s.In[1,4]+s.In[1,5]-FB24_4*s.l[3,4]-s.dpt[3,7]*(FB25_4*C5-FB35_4*S5)
    FA13 = -s.frc[1,3]+s.m[3]*BEF23*s.l[2,3]
    FA23 = -s.frc[2,3]+s.m[3]*(AF23+BS53*s.l[2,3])
    FA33 = -s.frc[3,3]+s.m[3]*(AF33+BS63*s.l[2,3])
    FF13 = FA13+FF14
    FF23 = FA23+FF24*C4-FF34*S4
    FF33 = FA33+FF24*S4+FF34*C4
    CF13 = -s.trq[1,3]+CF14-s.In[5,3]*OM23*OM33+s.In[9,3]*OM23*OM33+FA33*s.l[2,3]+s.dpt[2,6]*(FF24*S4+FF34*C4)
    CF23 = -s.trq[2,3]+qd[3]*s.In[1,3]*OM33-qd[3]*s.In[9,3]*OM33+s.In[5,3]*OA23+CF24*C4-CF34*S4
    CF33 = -s.trq[3,3]-qd[3]*s.In[1,3]*OM23+qd[3]*s.In[5,3]*OM23+s.In[9,3]*OA33+CF24*S4+CF34*C4-FA13*s.l[2,3]-FF14* \
 	  s.dpt[2,6]
    FB13_1 = s.m[3]*(-s.dpt[2,2]-s.l[2,3]*C3)
    FM13_1 = FB13_1+FM14_1
    CM23_1 = s.In[5,3]*S3+CM24_1*C4-CM34_1*S4
    CM33_1 = s.In[9,3]*C3+CM24_1*S4+CM34_1*C4-FB13_1*s.l[2,3]-FM14_1*s.dpt[2,6]
    FB23_2 = s.m[3]*S3
    FB33_2 = s.m[3]*C3
    FM23_2 = FB23_2+FM24_2*C4-FM34_2*S4
    FM33_2 = FB33_2+FM24_2*S4+FM34_2*C4
    CM13_2 = CM14_2+FB33_2*s.l[2,3]+s.dpt[2,6]*(FM24_2*S4+FM34_2*C4)
    FB33_3 = s.m[3]*s.l[2,3]
    CM13_3 = s.In[1,3]+CM14_3+FB33_3*s.l[2,3]+s.dpt[2,6]*(FM24_3*S4+FM34_3*C4)
    FA32 = -s.frc[3,2]-s.m[2]*s.g[3]
    FF32 = FA32-FF112*S12-FF19*S9+FF23*S3+FF26*S6+FF312*C12+FF33*C3+FF36*C6+FF39*C9
    CF32 = -s.trq[3,2]-CF112*S12-CF19*S9+CF23*S3+CF26*S6+CF312*C12+CF33*C3+CF36*C6+CF39*C9-FF13*s.dpt[2,2]-FF16* \
 	  s.dpt[2,3]+FF212*s.dpt[1,5]+FF29*s.dpt[1,4]
    CM32_1 = s.In[9,2]-CM112_1*S12-CM19_1*S9+CM23_1*S3+CM26_1*S6+CM312_1*C12+CM33_1*C3+CM36_1*C6+CM39_1*C9-FM13_1* \
 	  s.dpt[2,2]-FM16_1*s.dpt[2,3]+FM212_1*s.dpt[1,5]+FM29_1*s.dpt[1,4]
    FM32_2 = s.m[2]-FM112_2*S12-FM19_2*S9+FM23_2*S3+FM26_2*S6+FM312_2*C12+FM33_2*C3+FM36_2*C6+FM39_2*C9
    CF31 = -s.trq[3,1]+CF32
    CM31_1 = s.In[9,1]+CM32_1
 
# Symbolic model output

    c[1] = CF31
    c[2] = FF32
    c[3] = CF13
    c[4] = CF14
    c[5] = CF15
    c[6] = CF16
    c[7] = CF17
    c[8] = CF18
    c[9] = CF29
    c[10] = CF210
    c[11] = CF211
    c[12] = CF212
    c[13] = CF213
    c[14] = CF214
    M[1,1] = CM31_1
    M[2,2] = FM32_2
    M[2,3] = CM13_2
    M[2,4] = CM14_2
    M[2,6] = CM16_2
    M[2,7] = CM17_2
    M[2,9] = CM29_2
    M[2,10] = CM210_2
    M[2,12] = CM212_2
    M[2,13] = CM213_2
    M[3,2] = CM13_2
    M[3,3] = CM13_3
    M[3,4] = CM14_3
    M[3,5] = s.In[1,5]
    M[4,2] = CM14_2
    M[4,3] = CM14_3
    M[4,4] = CM14_4
    M[4,5] = s.In[1,5]
    M[5,3] = s.In[1,5]
    M[5,4] = s.In[1,5]
    M[5,5] = s.In[1,5]
    M[6,2] = CM16_2
    M[6,6] = CM16_6
    M[6,7] = CM17_6
    M[6,8] = s.In[1,8]
    M[7,2] = CM17_2
    M[7,6] = CM17_6
    M[7,7] = CM17_7
    M[7,8] = s.In[1,8]
    M[8,6] = s.In[1,8]
    M[8,7] = s.In[1,8]
    M[8,8] = s.In[1,8]
    M[9,2] = CM29_2
    M[9,9] = CM29_9
    M[9,10] = CM210_9
    M[9,11] = s.In[5,11]
    M[10,2] = CM210_2
    M[10,9] = CM210_9
    M[10,10] = CM210_10
    M[10,11] = s.In[5,11]
    M[11,9] = s.In[5,11]
    M[11,10] = s.In[5,11]
    M[11,11] = s.In[5,11]
    M[12,2] = CM212_2
    M[12,12] = CM212_12
    M[12,13] = CM213_12
    M[12,14] = s.In[5,14]
    M[13,2] = CM213_2
    M[13,12] = CM213_12
    M[13,13] = CM213_13
    M[13,14] = s.In[5,14]
    M[14,12] = s.In[5,14]
    M[14,13] = s.In[5,14]
    M[14,14] = s.In[5,14]

# Number of continuation lines = 1


