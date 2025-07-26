
# Dynamical car model
restart:
# MBSdir := cat(currentdir(),"\\MBSymba");
# MBSdir := "C:\\MBSymba\\lib";
# libname := MBSdir, libname:
with(MBSymba_r6):
with(ArrayTools):
with(LinearAlgebra):
with(codegen,cost,optimize):
with(FileTools): 
read("CarModel.mla"):
interface(rtablesize=30):
# Same assumption of CarModel.mw
linear_modeling(  {delta(t),
   psi__fl, phi__fl, mu__fl,
   psi__fr, phi__fr, mu__fr,
   psi__rl, phi__rl, mu__rl,
   psi__rr, phi__rr, mu__rr
} );
# Create dynamical equations
xx := [
phi(t), mu(t), z(t), delta(t),
z__fl(t), z__fr(t), z__rl(t), z__rr(t),
V__P(t), lambda__P(t), V__z(t),
delta__dot(t),
z__fldot(t), z__frdot(t), z__rldot(t), z__rrdot(t),
Omega__z(t), Omega__x(t), Omega__y(t),
omega__fl(t), omega__fr(t), omega__rl(t), omega__rr(t)];  # states
vv := [
v__phi(t), v__mu(t), v__z(t), v__delta(t),
v__z__fl(t), v__z__fr(t), v__z__rl(t), v__z__rr(t),
v__V__P(t), v__lambda__P(t), v__V__z(t),
v__delta__dot(t),
v__Omega__z(t), v__Omega__x(t), v__Omega__y(t),
v__z__fldot(t), v__z__frdot(t), v__z__rldot(t), v__z__rrdot(t),
v__omega__fl(t), v__omega__fr(t), v__omega__rl(t), v__omega__rr(t)];  # state derivatives v=xdot
uu;   # control u
# Convert xdot to v
xdot2v := [seq(diff(xx[i], t) = vv[i], i = 1 .. nops(xx))];
velocity_eqns := [
   op(velocity_eqns),
   diff(rr__fl(t),t)=rr__fldot(t),diff(rr__fr(t),t)=rr__frdot(t),diff(rr__rl(t),t)=rr__rldot(t),diff(rr__rr(t),t)=rr__rrdot(t),   # radii

   diff(xi__fl(t),t)=xi__fldot(t),diff(xi__fr(t),t)=xi__frdot(t),diff(xi__rl(t),t)=xi__rldot(t),diff(xi__rr(t),t)=xi__rrdot(t),   # tyre deflections
   diff(omega__x(t),t)=omega__xdot(t),diff(omega__y(t),t)=omega__ydot(t),diff(omega__z(t),t)=omega__zdot(t),diff(v__w(t),t)=v__wdot(t)   # road quantities

]:
# Tyre deflection equations
xi_eqns := xifl_eqn union xifr_eqn union xirl_eqn union xirr_eqn;
xidot_eqns := subs(velocity_eqns, diff(xi_eqns,t));
# Solz of velocity VX,VY of G as function of VP,lambdaP
op(simplify(solve(subs(velocity_eqns, [comp_X(VP) = V__P(t)*cos(lambda__P(t)), comp_Y(VP) = V__P(t)*sin(lambda__P(t))]), [V__x(t), V__y(t)]))): # a
G_vel := % :<%>;
# Solz tyre radii
rr_solz := op(simplify(solve(subs(xi_eqns,xx_eqns[-4..-1]), [rr__fl(t),rr__fr(t),rr__rl(t),rr__rr(t)]))):<%>;
rrdot_solz:=simplify(subs(velocity_eqns, G_vel, diff(rr_solz,t))):<%>;
# Use VP,lambdaP instead of VX,VY
velocity_eqns := simplify(subs(G_vel, velocity_eqns)):
VSfl := simplify(subs(G_vel,VSfl)):
VNfl := simplify(subs(G_vel,VNfl)):
VRfl := simplify(subs(G_vel,VRfl)):
VSfr := simplify(subs(G_vel,VSfr)):
VNfr := simplify(subs(G_vel,VNfr)):
VRfr := simplify(subs(G_vel,VRfr)):
VSrl := simplify(subs(G_vel,VSrl)):
VNrl := simplify(subs(G_vel,VNrl)):
VRrl := simplify(subs(G_vel,VRrl)):
VSrr := simplify(subs(G_vel,VSrr)):
VNrr := simplify(subs(G_vel,VNrr)):
VRrr := simplify(subs(G_vel,VRrr)):
# Dynamical equations
dyna_eqns := subs(G_vel, xx_eqns[1..-5]):
dyna_eqns := subs(xdot2v, 
diff(v__w(t),t)=v__wdot(t),
diff(omega__x(t),t)=omega__xdot(t),
diff(omega__y(t),t)=omega__ydot(t),
dyna_eqns):# replace derivatives
has(dyna_eqns, diff(xx,t)); # checks OK
;
has(dyna_eqns, omega__x(t)), has(dyna_eqns, omega__y(t)), has(dyna_eqns, omega__z(t)), has(dyna_eqns, v__w(t)); # checks OK
has(dyna_eqns, diff(omega__x(t),t)), has(dyna_eqns, diff(omega__y(t),t)),
has(dyna_eqns, diff(omega__z(t),t)), has(dyna_eqns, diff(v__w(t),t)); # checks OK
has(dyna_eqns, omega__xdot(t)), has(dyna_eqns, omega__ydot(t)),
has(dyna_eqns, omega__zdot(t)), has(dyna_eqns, v__wdot(t)); # checks OK
;
# Processing
# Tyre kinematics
front_left_kin := simplify(subs(velocity_eqns,[
rr_solz[1], rrdot_solz[1],
xi_eqns[1], xidot_eqns[1],
frontTyreAngles[1],
VSfl0=VSfl,
VNfl0=VNfl,
VRfl0=VRfl])): <%>:
front_right_kin := simplify(subs(velocity_eqns,[
rr_solz[2], rrdot_solz[2],
xi_eqns[2], xidot_eqns[2],
frontTyreAngles[2],
VSfr0=VSfr,
VNfr0=VNfr,
VRfr0=VRfr])): <%>:
rear_left_kin := simplify(subs(velocity_eqns,[
rr_solz[3], rrdot_solz[3],
xi_eqns[3], xidot_eqns[3],
rearTyreAngles[1],
VSrl0=VSrl,
VNrl0=VNrl,
VRrl0=VRrl])): <%>:
rear_right_kin := simplify(subs(velocity_eqns,[
rr_solz[4], rrdot_solz[4],
xi_eqns[4], xidot_eqns[4],
rearTyreAngles[2],
VSrr0=VSrr,
VNrr0=VNrr,
VRrr0=VRrr])): <%>:
# Wheel points
CPfl_coords:=[comp_XYZ(CPfl, T__P)]:<%>:# 
CPfr_coords:=[comp_XYZ(CPfr, T__P)]:<%>:
CPrl_coords:=[comp_XYZ(CPrl, T__P)]:<%>:# 
CPrr_coords:=[comp_XYZ(CPrr, T__P)]:<%>:
Wfl_coords:=[comp_XYZ(W__fl, T__P)]:<%>:
Wfr_coords:=[comp_XYZ(W__fr, T__P)]:<%>:
Wrl_coords:=[comp_XYZ(W__rl, T__P)]:<%>:
Wrr_coords:=[comp_XYZ(W__rr, T__P)]:<%>:
# Rate of G vel
G_veldot :=simplify(subs(xdot2v,diff(V__x(t),t)=V__xdot(t), diff(V__y(t),t)=V__ydot(t),diff(omega__x(t),t)=omega__xdot(t), diff(omega__y(t),t)=omega__ydot(t), (diff(G_vel,t)) union [V__zdot(t)=diff(V__z(t),t)])):
<%>;
# ovarall cog
Gall_coords:=[comp_XYZ(G__all, T__P)]:<%>:
# ovarall inertia
Iall_comps:=simplify([I__all[1][1], I__all[2][2], I__all[3][3], I__all[3][1], I__all[3][2], I__all[1][2]]):<%>:
# aerodynamic center
CA_coords:=[comp_XYZ(CA, T__P)]:<%>:
# ref point kinematics
yawrate := subs(omega__z(t)=yaw__rate(t), velocity_eqns[3]);
VP_xycomps:=[ 'V__xP'=simplify(subs(G_vel,velocity_eqns, comp_X(VP, T__P))),
              'V__yP'=simplify(subs(G_vel,velocity_eqns, comp_Y(VP, T__P)))]:
simplify(expand(subs(velocity_eqns, G_vel,[a__xP = comp_X(AP, T__R), a__yP = comp_Y(AP, T__R)]))):
simplify(subs(velocity_eqns, %)): 
AP_xycomps := simplify(subs(Omega__z(t) = solve(yawrate, Omega__z(t)), xdot2v, %)):<%>;
T__R1 := T__R * rotate('Z',lambda__P(t)):
simplify(expand(subs(velocity_eqns, G_vel,[a__tP = comp_X(AP, T__R1), a__nP = comp_Y(AP, T__R1)]))): 
simplify(subs(velocity_eqns, %)): 
AP_tncomps := simplify(subs(Omega__z(t) = solve(yawrate, Omega__z(t)), xdot2v, %)):
ref_point_kin := [yawrate, op(VP_xycomps), op(AP_xycomps), op(AP_tncomps)]: <%>;
# get vehice rates: V__z,Omega__i,ft__dot,... as a function of v__z, v__phi, ...
velocity_eqns[3..6]: # velocity equations omega__z, phi dot, mu dot ...
subs(xdot2v, 
op(simplify(solve(%, 
[V__z(t),Omega__x(t),Omega__y(t),Omega__z(t)])))):
rates_eqns := %: <%>:
# Solve delta
rates_eqns := rates_eqns union subs(xdot2v, [rhs(velocity_eqns[7]) = lhs(velocity_eqns[7])]): <%>: 
# Solve z__ij
velocity_eqns[9..12]:  # velocity equarions z__ij
subs(xdot2v, 
op(simplify(solve(%, 
[z__fldot(t), z__frdot(t), z__rldot(t), z__rrdot(t)])))):
rates_eqns := rates_eqns union linearize(%, {phi(t),mu(t)}): <%>;
# Remove time dependincy (i.e. (t))
remove_t := proc(eqns) local all_functions, vars_with_t, subs_set, f; all_functions := indets(eqns, 'function'); vars_with_t := select(f -> not (op(0, f) = 'sin' or op(0, f) = 'cos' or op(0, f) = 'arctan' or op(0, f) = 'tan' or op(0, f) = 'sec') and nops(f) = 1, all_functions); print("Variables with (t):", vars_with_t); subs_set := {seq(f = op(0, f), f in vars_with_t)}; return subs(subs_set, eqns); end proc;
xx0 := remove_t(xx);
vv0 := remove_t(vv);
uu0 := remove_t(uu);
dyna_eqns0 := remove_t(dyna_eqns): 
has(dyna_eqns,(t)), has(dyna_eqns0,(t)); # checks OK
for k from 1 to nops(dyna_eqns0) do 
   dyna_eqns0[k] := dynamicRes[k] = dyna_eqns0[k]:
end:
front_left_kin0 := remove_t(front_left_kin):
front_right_kin0 := remove_t(front_right_kin):
rear_left_kin0 := remove_t(rear_left_kin):
rear_right_kin0 := remove_t(rear_right_kin):
CPfl_coords0:=remove_t(CPfl_coords):
CPfr_coords0:=remove_t(CPfr_coords):
CPrl_coords0:=remove_t(CPrl_coords):
CPrr_coords0:=remove_t(CPrr_coords):
for k from 1 to nops(CPfl_coords0) do 
   CPfl_coords0[k] := CPfl[k] = CPfl_coords0[k]:
   CPfr_coords0[k] := CPfr[k] = CPfr_coords0[k]:
   CPrl_coords0[k] := CPrl[k] = CPrl_coords0[k]:
   CPrr_coords0[k] := CPrr[k] = CPrr_coords0[k]:
end:
Wfl_coords0:=remove_t(Wfl_coords):
Wfr_coords0:=remove_t(Wfr_coords):
Wrl_coords0:=remove_t(Wrl_coords):
Wrr_coords0:=remove_t(Wrr_coords):
for k from 1 to nops(Wfl_coords0) do 
   Wfl_coords0[k] := Wfl[k] = Wfl_coords0[k]:
   Wfr_coords0[k] := Wfr[k] = Wfr_coords0[k]:
   Wrl_coords0[k] := Wrl[k] = Wrl_coords0[k]:
   Wrr_coords0[k] := Wrr[k] = Wrr_coords0[k]:
end:
Gall_coords0:=remove_t(Gall_coords):
for k from 1 to nops(Gall_coords0) do 
   Gall_coords0[k] := G[k] = Gall_coords0[k]:
end:
Iall_comps0:=remove_t(Iall_comps):
for k from 1 to nops(Iall_comps0) do 
   Iall_comps0[k] := II[k] = Iall_comps0[k]:
end:
G_vel0:=remove_t(G_vel):
G_veldot0:=remove_t(G_veldot):
CA_coords0:=remove_t(CA_coords):
for k from 1 to nops(CA_coords) do 
   CA_coords0[k] := CA[k] = CA_coords0[k]:
end:NULL;
rr_solz0:=remove_t(rr_solz):
xi_eqns0:=remove_t(xi_eqns):
xidot_eqns0:=remove_t(xidot_eqns):
ref_point_kin0:=remove_t(ref_point_kin):
rates_eqns0:=remove_t(rates_eqns):
# Optimize to reduce cpu cost
optimizeProc := proc(expr) local c1, c2, dc, expropt; c1 := cost(expr); print("Original cost", c1); expropt := simplify(expr, trig); expropt := optimize(expropt); c2 := cost(expropt); print("Optimized cost ", c2); dc := c1 - c2; print("Difference cost", dc); return expropt; end proc;
dyna_eqns0:=optimizeProc(dyna_eqns0):
ref_point_kin0:=optimizeProc(ref_point_kin0):
front_tyre_kinematics := optimizeProc(front_left_kin0 union front_right_kin0):
rear_tyre_kinematics := optimizeProc(rear_left_kin0 union rear_right_kin0):
postproc_eqns := optimizeProc([
op(CPfl_coords0), op(CPfr_coords0),op(CPrl_coords0), op(CPrr_coords0),
op(Wfl_coords0), op(Wfr_coords0),op(Wrl_coords0), op(Wrr_coords0),
op(CA_coords0), op(Gall_coords0), 
op(Iall_comps0), op(G_vel0), op(G_veldot0)]):
rates_eqns0 := optimizeProc(rates_eqns0):
# Matlab code
# Create +maple folder
dirname:="+maple/";
with(FileTools): 
if Exists(dirname) then
  RemoveDirectory(dirname, recurse=true, forceremove=true);
end if;
MakeDirectory(dirname);
print2screen := false:
# output management
ffopen  := fname -> if print2screen then 0; else fopen(fname, WRITE); end;
ffclose := fd    -> if not(print2screen) then fclose(fd); end;
# Dynamical Equations residuals
#print2screen := true:
fd := ffopen(cat(dirname,"dynamicResiduals.m"), WRITE):
fprintf(fd,"%% Dynamic Equations  *** DO NOT EDIT ***\n"):
fprintf(fd,"%s", CodeGeneration[Matlab]([dyna_eqns0], output=string)):
ffclose(fd);
#print2screen := true:
# Reference point kinematics
fd := ffopen(cat(dirname,"refPointKinematics.m"), WRITE):
fprintf(fd,"%% Reference Point Kinematics  *** DO NOT EDIT ***\n"):
fprintf(fd,"%s",CodeGeneration[Matlab]([ref_point_kin0], output=string)):
ffclose(fd);
# Front tyre kinematics
#print2screen := true:
fd := ffopen(cat(dirname,"frontTyreKinematics.m"), WRITE):
fprintf(fd,"%% Front Tyre Kinematics  *** DO NOT EDIT ***\n"):
fprintf(fd,"%s",CodeGeneration[Matlab]([front_tyre_kinematics], output=string)):
fprintf(fd,"phit__fl = 0; %% Turn slip TODO calc from MAPLE\n"):
fprintf(fd,"phit__fr = 0; %% Turn slip TODO calc from MAPLE\n"):#  
ffclose(fd);
# Rear tyre kinematics
#print2screen := true:
fd := ffopen(cat(dirname,"rearTyreKinematics.m"), WRITE):
fprintf(fd,"%% Rear Tyre Kinematics  *** DO NOT EDIT ***\n"):
fprintf(fd,"%s",CodeGeneration[Matlab]([rear_tyre_kinematics], output=string)):
fprintf(fd,"phit__rl = 0; %% Turn slip TODO calc from MAPLE\n"):
fprintf(fd,"phit__rr = 0; %% Turn slip TODO calc from MAPLE\n"):#  
ffclose(fd);
# Post-processing
#print2screen := true:
fd := ffopen(cat(dirname,"postProcessing.m"), WRITE):
fprintf(fd,"%% Post-processing  *** DO NOT EDIT ***\n"):
fprintf(fd,"%s",CodeGeneration[Matlab]([postproc_eqns], output=string)):
ffclose(fd);
# Get rates
#print2screen := true:
fd := ffopen(cat(dirname,"vehRates.m"), WRITE):
fprintf(fd,"%% Get rates  *** DO NOT EDIT ***\n"):
fprintf(fd,"%s",CodeGeneration[Matlab]([rates_eqns0], output=string)):
ffclose(fd);
# Save output equations
save(dyna_eqns0, ref_point_kin0, 
front_tyre_kinematics, rear_tyre_kinematics, postproc_eqns,
"CarDynamics.mla");
NULL;
