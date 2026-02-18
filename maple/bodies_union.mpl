# bodies_union (add body inertia and compute resulting c.g)
bodies_union := proc(body1,body2,T3) # # T3 is the frame you want for the bodies union (body3)
   uses LinearAlgebra, MBSymba_r6:
   local X12,Y12,Z12,I1_master,I2_master,I1_shift,I2_shift,I12_master,G12,G12_master,m12,body3,G1_master,G2_master;

# # Center of mass of body1 & body2 - master_frame frame
G1_master := project(origin(body1[frame]),master_frame):        
G2_master := project(origin(body2[frame]),master_frame):

# # Center of mass evalutation - master_frame frame
X12 := (comp('X',G1_master)*body1[mass]+comp('X',G2_master)*body2[mass])/(body1[mass]+body2[mass]):
Y12 := (comp('Y',G1_master)*body1[mass]+comp('Y',G2_master)*body2[mass])/(body1[mass]+body2[mass]):
Z12 := (comp('Z',G1_master)*body1[mass]+comp('Z',G2_master)*body2[mass])/(body1[mass]+body2[mass]):

# # I_master = T * I_frameT * Ttranspose
I1_master := SubMatrix(body1[frame],1..3,1..3) . body1[inertia] . Transpose(SubMatrix(body1[frame],1..3,1..3)):
I2_master := SubMatrix(body2[frame],1..3,1..3) . body2[inertia] . Transpose(SubMatrix(body2[frame],1..3,1..3)) :

# #Translational terms of the Stainer's formula
I1_shift := Matrix([	[ (comp('Y',G1_master)-Y12)^2+(comp('Z',G1_master)-Z12)^2, -(comp('X',G1_master)-X12)  *(comp('Y',G1_master)-Y12),   -(comp('X',G1_master)-X12)  *(comp('Z',G1_master)-Z12)], 
							[-(comp('X',G1_master)-X12)  *(comp('Y',G1_master)-Y12),    (comp('X',G1_master)-X12)^2+(comp('Z',G1_master)-Z12)^2, -(comp('Y',G1_master)-Y12)  *(comp('Z',G1_master)-Z12)], 
							[-(comp('X',G1_master)-X12)  *(comp('Z',G1_master)-Z12),   -(comp('Y',G1_master)-Y12)*  (comp('Z',G1_master)-Z12),    (comp('X',G1_master)-X12)^2+(comp('Y',G1_master)-Y12)^2]	]):
I2_shift := Matrix([	[ (comp('Y',G2_master)-Y12)^2+(comp('Z',G2_master)-Z12)^2, -(comp('X',G2_master)-X12)*  (comp('Y',G2_master)-Y12),   -(comp('X',G2_master)-X12)  *(comp('Z',G2_master)-Z12)], 
							[-(comp('X',G2_master)-X12)  *(comp('Y',G2_master)-Y12),    (comp('X',G2_master)-X12)^2+(comp('Z',G2_master)-Z12)^2, -(comp('Y',G2_master)-Y12)  *(comp('Z',G2_master)-Z12)], 
							[-(comp('X',G2_master)-X12)  *(comp('Z',G2_master)-Z12),   -(comp('Y',G2_master)-Y12)  *(comp('Z',G2_master)-Z12),    (comp('X',G2_master)-X12)^2+(comp('Y',G2_master)-Y12)^2]]	):

# # Construction of the new body3, whose center of mass and inertia are the resultant of body1 and body2 union
# # Resultant inertia - master_frame frame
I12_master := I1_master + body1[mass]*I1_shift + I2_master + body2[mass]*I2_shift:        
# # Resultant center of mass - master_frame frame
G12_master := make_POINT(master_frame,X12,Y12,Z12):                                           
G12 := project(G12_master,T3):                                                                                 
# # __________________ - frame T3
m12 := body1[mass]+body2[mass]:                                                                                
# # Resultant mass
body3 := make_BODY(G12,m12,ix,iy,iz):                                                                   
# # Body3 is defined in frame T3 with the origin in G12
body3[inertia] := Transpose(SubMatrix(T3,1..3,1..3)) . eval(I12_master) . SubMatrix(T3,1..3,1..3):
# # Procedure output
eval(body3):
                                                                                                                 
end proc: