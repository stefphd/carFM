
# Steady state car model
restart:
# MBSdir := cat(currentdir(),"\\MBSymba");
# MBSdir := "C:\\MBSymba\\lib";
# libname := MBSdir, libname:
with(MBSymba_r6):
with(codegen,cost,optimize);
read( "CarModel.mla"):
interface(rtablesize=30)
;
# Convert dynamic equations into steady state
S0 := [   # main variables
phi(t)=phi0,mu(t)=mu0,z(t)=z0,
delta(t)=delta0,delta__dot(t)=0,delta__ddot(t)=0,
z__fl(t)=z__fl0,z__fr(t)=z__fr0,z__rl(t)=z__rl0,z__rr(t)=z__rr0,
# velocities including tangential acceleration, a__t0
V__P(t)=V__P0+a__t0*t, lambda__P(t)=lambda__P0,
z__fldot(t)=0,z__frdot(t)=0,z__rldot(t)=0,z__rrdot(t)=0,  # suspensions
omega__z(t)=yaw__rate0, omega__x(t)=0,omega__y(t)=0,v__w(t)=0,  # 3D effects are accounted for into g__z
omega__fl(t)=omega__fl0+omega__dotfl0*t,
omega__fr(t)=omega__fr0+omega__dotfr0*t,
omega__rl(t)=omega__rl0+omega__dotrl0*t,
omega__rr(t)=omega__rr0+omega__dotrr0*t,
Omega__x(t) = Omega__x0, Omega__y(t) = Omega__y0, Omega__z(t) = Omega__z0,
V__x(t) = V__x0, V__y(t) = V__y0, V__z(t) = V__z0,
# forces & torques
X__fl(t)=X__fl0,Y__fl(t)=Y__fl0,N__fl(t)=N__fl0,F__fl(t)=F__fl0,
X__fr(t)=X__fr0,Y__fr(t)=Y__fr0,N__fr(t)=N__fr0,F__fr(t)=F__fr0,
X__rl(t)=X__rl0,Y__rl(t)=Y__rl0,N__rl(t)=N__rl0,F__rl(t)=F__rl0,
X__rr(t)=X__rr0,Y__rr(t)=Y__rr0,N__rr(t)=N__rr0,F__rr(t)=F__rr0,
# tyre deflections
xi__fl(t)=xi__fl0,xi__fr(t)=xi__fr0,xi__rl(t)=xi__rl0,xi__rr(t)=xi__rr0,
# tyre radii
rr__fl(t)=r0__fl,rr__fr(t)=r0__fr,rr__rl(t)=r0__rl,rr__rr(t)=r0__rr,
# tyre slip
kappa__fl(t)=kappa__fl0,kappa__fr(t)=kappa__fr0,kappa__rl(t)=kappa__rl0,kappa__rr(t)=kappa__rr0,
# wheel torques
Tau__fl(t)=Tau__fl0,Tau__fr(t)=Tau__fr0,Tau__rl(t)=Tau__rl0,Tau__rr(t)=Tau__rr0
]:<%>;
# Kinematics
pos; vel; #<velocity_eqns>;

# angular velocities
ang_vel := op(simplify(solve(velocity_eqns[3..5], vel[5..7]))): #<ang_vel>; # full expressions
ang_vel0 := subs(diff(psi(t),t)=yaw__rate0, S0, ang_vel): <ang_vel0>;
# CoG velocity
show(VG); # full expression
# steady state
[comp_X(VP, T__P)=V__P(t)*cos(lambda__P(t)), comp_Y(VP, T__P)=V__P(t)*sin(lambda__P(t)), comp_Z(VP, T__P)=0]:
op(solve(%, [V__x(t),V__y(t),V__z(t)])):
G_vel := simplify(%): 
G_vel0 := linearize(simplify(expand(subs(S0,ang_vel0,%))), {mu0,phi0}): <%>;
G_veldot0 := linearize(simplify(expand(subs(V__x(t)=V__x0+V__dotx0*t, V__y(t)=V__y0+V__doty0*t, V__z(t)=V__z0+V__dotz0*t,S0,ang_vel0,diff(G_vel,t)))),
{mu0,phi0}):<%>;
# Check: G_veldot0 gives z__ddot = 0
velocity_eqns[6]; # z__ddot

simplify(expand(subs(diff(z(t),t,t)=z__ddot,V__x(t)=V__x0+V__dotx0*t, V__y(t)=V__y0+V__doty0*t, V__z(t)=V__z0+V__dotz0*t, G_veldot0, S0, diff(%,t)))); #  = 0 OK
;
# Check: G_veldot0 gives lambda__Pdot = 0
[comp_X(VP, T__P)=V__P(t)*cos(lambda__P(t)), comp_Y(VP, T__P)=V__P(t)*sin(lambda__P(t))]: # V__P,lambda__P
simplify(expand(op(solve(diff(%,t), [diff(V__P(t),t),diff(lambda__P(t),t)])))): # sV__Pdot,lambda__Pdot

simplify(expand(subs(V__x(t)=V__x0+V__dotx0*t, V__y(t)=V__y0+V__doty0*t, V__z(t)=V__z0+V__dotz0*t, G_veldot0, S0, %, lambda__Pdot=diff(lambda__P(t),t)))); #  = 0 OK
;
# Solve radii
xx_eqns[-4..-1]: <%>:
S0rr := subs(S0, op(solve(%, [rr__fl(t),rr__fr(t),rr__rl(t),rr__rr(t)]))):<%>;
# all togheter
S0plus := ang_vel0 union G_vel0: <%>;
# steady state equations
map(simplify,subs(S0,S0plus,S0rr,xx_eqns)):
xx_eqns0 := subs(t=0,linearize(%, {mu0,phi0})):
# check the presence of unwanted acceleration terms
if ( norm(<diff(xx_eqns0,t,t)>) <> 0 or norm(<subs(omega__dotfl0=0,omega__dotrl0=0,omega__dotrl0=0,omega__dotrr0=0,a__t0=0,diff(xx_eqns0,t))>) <>0 )
then error("Steady State Equations still depend on time!"); end;
# discard null equations
ss_eqns := []: printf("Null equations list: "):
for k from 1 to nops(xx_eqns0) do
   if xx_eqns0[k] = 0 
      then printf("%d ",k):
      else ss_eqns := [op(ss_eqns), xx_eqns0[k]]:
   end:
end:
printf("\nFound %d Steady State Equations\n", nops(ss_eqns)):
# Nominal conditions for easier inspection
ssnc := [
phi0=0,mu0=0,z0=0,delta0=0,
z__fl0=0,z__fr0=0,z__rl0=0,z__rr0=0];
for k from 1 to nops(ss_eqns) do k = collect(simplify(subs(ssnc,ss_eqns[k])),V0); end;
# Matlab code
# tyre kinematics
linearize(subs(t=0,simplify(eval(subs(S0,S0plus, [VSfl0 = VSfl, VNfl0 = VNfl, VRfl0 = VRfl] )))),{mu0, phi0}):
front_left_kin0 := %:
<front_left_kin0>;
linearize(subs(t=0,simplify(eval(subs(S0,S0plus, [VSfr0 = VSfr, VNfr0 = VNfr, VRfr0 = VRfr] )))),{mu0, phi0}):
front_right_kin0 := %:
<front_right_kin0>;
linearize(subs(t=0,simplify(eval(subs(S0,S0plus, [VSrl0 = VSrl, VNrl0 = VNrl, VRrl0 = VRrl] )))),{mu0, phi0}):
rear_left_kin0 := %:
<rear_left_kin0>;
linearize(subs(t=0,simplify(eval(subs(S0,S0plus, [VSrr0 = VSrr, VNrr0 = VNrr, VRrr0 = VRrr] )))),{mu0, phi0}):
rear_right_kin0 := %:
<rear_right_kin0>;
# wheel points
CPfl_coords:=linearize(subs(S0,[comp_XYZ(CPfl, T__P)]),{mu0,phi0}):Vector(%):# 
CPfr_coords:=linearize(subs(S0,[comp_XYZ(CPfr, T__P)]),{mu0,phi0}):Vector(%):
CPrl_coords:=linearize(subs(S0,[comp_XYZ(CPrl, T__P)]),{mu0,phi0}):Vector(%):# 
CPrr_coords:=linearize(subs(S0,[comp_XYZ(CPrr, T__P)]),{mu0,phi0}):Vector(%):
Wfl_coords:=linearize(subs(S0,[comp_XYZ(W__fl, T__P)]),{mu0,phi0}):Vector(%):
Wfr_coords:=linearize(subs(S0,[comp_XYZ(W__fr, T__P)]),{mu0,phi0}):Vector(%):
Wrl_coords:=linearize(subs(S0,[comp_XYZ(W__rl, T__P)]),{mu0,phi0}):Vector(%):
Wrr_coords:=linearize(subs(S0,[comp_XYZ(W__rr, T__P)]),{mu0,phi0}):Vector(%):
# ovarall cog
Gall_coords:=linearize(subs(S0,[comp_XYZ(G__all, T__P)]),{mu0,phi0}):Vector(%);
# ovarall inertia
Iall_comps:=simplify(linearize(subs(S0,[I__all[1][1], I__all[2][2], I__all[3][3], I__all[3][1], I__all[3][2], I__all[1][2]]),{mu0,phi0,delta0})):<%>;
# reference point
AP_xycomps:=linearize(subs(t=0, simplify(expand(subs(S0, S0plus, [a__xP=comp_X(AP, T__P), a__yP=comp_Y(AP, T__P)])))),{mu0,phi0}):<%>;
AP_tncomps:=linearize(subs(t=0, simplify(expand(subs(S0, S0plus, [a__tP=comp_X(AP, T__P*rotate('Z',lambda__P(t))), a__nP=comp_Y(AP, T__P*rotate('Z',lambda__P(t)))])))),{mu0,phi0}):<%>;
# Find omegaidot from kappaidot=0 and omega as a function of kappa
# Note that this is only an internal definition of kappa, which is not seen by the end user
eqkappa := simplify(subs(velocity_eqns, [
   kappa__fl(t)=-(1+VRfl/VSfl),
   kappa__fr(t)=-(1+VRfr/VSfr),
   kappa__rl(t)=-(1+VRrl/VSrl),
   kappa__rr(t)=-(1+VRrr/VSrr)
])):<%>:   # using Vr=-omega * R loaded to calculate kappa
# eqkappa := simplify(subs(velocity_eqns, [
#    kappa__fl(t)=omega__fl(t)*R__f/VSfl-1,
#    kappa__fr(t)=omega__fr(t)*R__f/VSfr-1,
#    kappa__rl(t)=omega__rl(t)*R__r/VSrl-1,
#    kappa__rr(t)=omega__rr(t)*R__r/VSrr-1
# ])):<%>:   # using omega * R unloaded to calculate kappa
eqomega := simplify(op(solve(eqkappa, [omega__fl(t),omega__fr(t),omega__rl(t),omega__rr(t)]))): <%>:
eqkappa0 := linearize(subs(t=0,simplify(expand(simplify(subs(S0,S0plus,eqkappa))))),{mu0,phi0,delta0}):
eqomegadot0 := linearize(simplify(expand(subs(S0,S0plus, diff(eqomega,t)))),{mu0,phi0,delta0}):<%>;
eqomega0 := linearize(subs(t=0,simplify(expand(simplify(subs(S0,S0plus,eqomega))))),{mu0,delta0,phi0}):<%>;
front_left_kin0 := subs(S0,
    [S0rr[1], op(xifl_eqn), frontTyreAngles[1]]) union
    [eqomega0[1],eqomegadot0[1]] union
    front_left_kin0 union [alpha__fl0=-arctan(VNfl0/VSfl0)]:<%>
;
front_right_kin0 := subs(S0,
    [S0rr[2],op(xifr_eqn), frontTyreAngles[2]]) union
    [eqomega0[2],eqomegadot0[2]] union
    front_right_kin0 union [alpha__fr0=-arctan(VNfr0/VSfr0)]:<%>;
rear_left_kin0 := subs(S0,
    [S0rr[3], op(xirl_eqn), rearTyreAngles[1]]) union
    [eqomega0[3],eqomegadot0[3]] union
    rear_left_kin0 union [alpha__rl0=-arctan(VNrl0/VSrl0)]:<%>;
rear_right_kin0 := subs(S0,
    [S0rr[4], op(xirr_eqn), rearTyreAngles[2]]) union
    [eqomega0[4],eqomegadot0[4]] union
    rear_right_kin0 union [alpha__rr0=-arctan(VNrr0/VSrr0)]:<%>;
# aerodynamic center
CA_coords:=linearize(subs(S0,[comp_XYZ(CA, T__P)]),{mu0,phi0}): Vector(%);
# Optimize to reduce cpu cost
optimizeProc := proc(expr) local c1, c2, dc, expropt; c1 := cost(expr); print("Original cost", c1); expropt := simplify(expr, trig); expropt := optimize(expropt); c2 := cost(expropt); print("Optimized cost ", c2); dc := c1 - c2; print("Difference cost", dc); return expropt; end proc;
for k from 1 to nops(ss_eqns) do 
   ss_eqns[k] := steadyStateRes[k] = ss_eqns[k]:
end:
ss_eqns:=optimizeProc(ss_eqns):
front_tyre_kinematics := optimizeProc(front_left_kin0 union front_right_kin0):
rear_tyre_kinematics := optimizeProc(rear_left_kin0 union rear_right_kin0):
for k from 1 to nops(CPfl_coords) do 
   CPfl_coords[k] := CPfl[k] = CPfl_coords[k]:
   CPfr_coords[k] := CPfr[k] = CPfr_coords[k]:
   CPrl_coords[k] := CPrl[k] = CPrl_coords[k]:
   CPrr_coords[k] := CPrr[k] = CPrr_coords[k]:
   Wfl_coords[k] := Wfl[k] = Wfl_coords[k]:
   Wfr_coords[k] := Wfr[k] = Wfr_coords[k]:
   Wrl_coords[k] := Wrl[k] = Wrl_coords[k]:
   Wrr_coords[k] := Wrr[k] = Wrr_coords[k]:
   Gall_coords[k] := G[k] = Gall_coords[k]:
   CA_coords[k] := CA[k] = CA_coords[k]
end:
for k from 1 to nops(Iall_comps) do 
    Iall_comps[k] := II[k] = Iall_comps[k]:
end:
postproc_eqns := optimizeProc(subs(t=0,[op(G_vel0), op(ang_vel0), op(G_veldot0), op(AP_xycomps), op(AP_tncomps), 
op(CPfl_coords), op(CPfr_coords),op(CPrl_coords), op(CPrr_coords),
op(Wfl_coords), op(Wfr_coords),op(Wrl_coords), op(Wrr_coords),
op(CA_coords), 
op(Gall_coords), op(Iall_comps)])):
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
# Steady Sate Equations residuals
#print2screen := true:
fd := ffopen(cat(dirname,"steadyStateResiduals.m"), WRITE):
fprintf(fd,"%% Steady State Equations  *** DO NOT EDIT ***\n"):# a
fprintf(fd,"%s", CodeGeneration[Matlab]([ss_eqns], optimize=false, output=string)):# a
ffclose(fd);
# Front tyre kinematics
#print2screen := true:
fd := ffopen(cat(dirname,"frontTyreKinematics.m"), WRITE):
fprintf(fd,"%% Front tyre kinematics  *** DO NOT EDIT ***\n"):
fprintf(fd,"%s",CodeGeneration[Matlab]([front_tyre_kinematics], output=string)):
fprintf(fd,"phit__fl = 0; %% Turn slip TODO calc from MAPLE\n"):
fprintf(fd,"phit__fr = 0; %% Turn slip TODO calc from MAPLE\n"):
fprintf(fd,"\n"):
ffclose(fd);
# Rear tyre kinematics
#print2screen := true:
fd := ffopen(cat(dirname,"rearTyreKinematics.m"), WRITE):
fprintf(fd,"%% Rear tyre kinematics  *** DO NOT EDIT ***\n"):
fprintf(fd,"%s",CodeGeneration[Matlab]([rear_tyre_kinematics], output=string)):
fprintf(fd,"phit__rl = 0; %% Turn slip TODO calc from MAPLE\n"):
fprintf(fd,"phit__rr = 0; %% Turn slip TODO calc from MAPLE\n"):
fprintf(fd,"\n"):
ffclose(fd);
# Post-processing
print2screen := false:
fd := ffopen(cat(dirname,"postProcessing.m"), WRITE):
fprintf(fd,"%% Post-processing  *** DO NOT EDIT ***\n"):
fprintf(fd,"%s",CodeGeneration[Matlab]([postproc_eqns], output=string)):
fprintf(fd,"\n"):
ffclose(fd);
# Save output equations
save(ss_eqns, front_tyre_kinematics, rear_tyre_kinematics, postproc_eqns,
"CarSteadyState.mla");
NULL;
