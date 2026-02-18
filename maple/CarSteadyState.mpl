
# Steady state car model
restart:
# MBSdir := cat(currentdir(),"\\MBSymba");
# MBSdir := "C:\\MBSymba\\lib";
# libname := MBSdir, libname:
with(MBSymba_r6):
with(codegen,cost,optimize):
read( "CarModel.mla"):
interface(rtablesize=30):
status := GetLicenseStatus():
printf("Status: %s\n", status[Message]),
if not status[StatusCode]=0 then
   error("Invalid license"):
end if:
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
Omega__x(t) = Omega__x0 + Omega__dotx0 * t, # includes time derivative

Omega__y(t) = Omega__y0 + Omega__doty0 * t, # includes time derivative

Omega__z(t) = Omega__z0 + Omega__dotz0 * t, # includes time derivative

V__x(t) = V__x0 + V__dotx0 * t, # includes time derivative

V__y(t) = V__y0 + V__doty0 * t, # includes time derivative

V__z(t) = V__z0 + V__dotz0 * t, # includes time derivative

# forces & torques
X__fl(t)=X__fl0,Y__fl(t)=Y__fl0,N__fl(t)=N__fl0,F__fl(t)=F__fl0,S__fl(t)=S__fl0,
X__fr(t)=X__fr0,Y__fr(t)=Y__fr0,N__fr(t)=N__fr0,F__fr(t)=F__fr0,S__fr(t)=S__fr0,
X__rl(t)=X__rl0,Y__rl(t)=Y__rl0,N__rl(t)=N__rl0,F__rl(t)=F__rl0,S__rl(t)=S__rl0,
X__rr(t)=X__rr0,Y__rr(t)=Y__rr0,N__rr(t)=N__rr0,F__rr(t)=F__rr0,S__rr(t)=S__rr0,
# tyre deflections
xi__fl(t)=xi__fl0,xi__fr(t)=xi__fr0,xi__rl(t)=xi__rl0,xi__rr(t)=xi__rr0,
# tyre radii
rr__fl(t)=r0__fl,rr__fr(t)=r0__fr,rr__rl(t)=r0__rl,rr__rr(t)=r0__rr,
# tyre slip
kappa__fl(t)=kappa__fl0,kappa__fr(t)=kappa__fr0,kappa__rl(t)=kappa__rl0,kappa__rr(t)=kappa__rr0,
# wheel torques
Tau__fl(t)=Tau__fl0,Tau__fr(t)=Tau__fr0,Tau__rl(t)=Tau__rl0,Tau__rr(t)=Tau__rr0,
# road roughness
z__rfl(t)=0,z__rfr(t)=0,z__rrl(t)=0,z__rrr(t)=0
]:<%>;
# Kinematics
pos; vel; #<velocity_eqns>;

# angular velocities
ang_vel := op(simplify(solve(velocity_eqns[3..5], vel[5..7]))): #<ang_vel>; # full expressions
# steady state
subs(S0, ang_vel):
ang_vel0 := subs(t=0,%): <%>;
# angular velocity derivatives
diff(ang_vel,t):    # full expressions

# steady state
ang_veldot0 := eval(subs(
  diff(Omega__x(t),t)=Omega__dotx0,
  diff(Omega__y(t),t)=Omega__doty0,
  diff(Omega__z(t),t)=Omega__dotz0, 
S0,%)): <%>;  # angular accelerations are 0 in SS!
# CoG velocity and derivative
show(VG); # full expression
[comp('X', VP, T__P)=V__P(t)*cos(lambda__P(t)), comp('Y', VP, T__P)=V__P(t)*sin(lambda__P(t)), comp('Z', VP, T__P)=0]:
op(solve(%, [V__x(t),V__y(t),V__z(t)])):
G_vel := simplify(%): <%>:
G_veldot := simplify(diff(G_vel,t)): <%>:
# steady state
G_vel0 := simplify(subs(t=0,simplify(expand(subs(S0,ang_vel0,G_vel))))): <%>;
G_veldot0 := simplify(expand(subs(S0,ang_vel0,ang_veldot0, G_veldot))): <%>;
# Check: G_veldot0 gives z__ddot = 0
velocity_eqns[6]; # z__ddot

simplify(expand(subs(diff(z(t),t,t)=z__ddot, S0, ang_veldot0, G_veldot0, S0, diff(%,t)))); #  = 0 OK
;
# Check: G_veldot0 gives lambda__Pdot = 0
[comp('X', VP, T__P)=V__P(t)*cos(lambda__P(t)), comp('Y', VP, T__P)=V__P(t)*sin(lambda__P(t))]: # V__P,lambda__P
simplify(expand(op(solve(diff(%,t), [diff(V__P(t),t),diff(lambda__P(t),t)])))): # sV__Pdot,lambda__Pdot

simplify(expand(subs(S0, ang_veldot0, G_veldot0, S0, %, lambda__Pdot=diff(lambda__P(t),t)))); #  = 0 OK
;
# Solve radii
xx_eqns[-4..-1]: <%>:
S0rr := simplify(subs(S0, op(solve(%, [rr__fl(t),rr__fr(t),rr__rl(t),rr__rr(t)])))):<%>;
# all togheter
S0plus := ang_vel0 union G_vel0 union ang_veldot0 union G_veldot0: <%>;
# steady state equations
simplify(subs(S0,S0plus,xx_eqns[1..-5])):   # exclude radii eqns

xx_eqns0 := subs(t=0,%):
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
subs(t=0,simplify(eval(subs(S0,S0plus, [VSfl0 = VSfl, VNfl0 = VNfl, VRfl0 = VRfl] )))):
front_left_kin0 := %:
<front_left_kin0>;
subs(t=0,simplify(eval(subs(S0,S0plus, [VSfr0 = VSfr, VNfr0 = VNfr, VRfr0 = VRfr] )))):
front_right_kin0 := %:
<front_right_kin0>;
subs(t=0,simplify(eval(subs(S0,S0plus, [VSrl0 = VSrl, VNrl0 = VNrl, VRrl0 = VRrl] )))):
rear_left_kin0 := %:
<rear_left_kin0>;
subs(t=0,simplify(eval(subs(S0,S0plus, [VSrr0 = VSrr, VNrr0 = VNrr, VRrr0 = VRrr] )))):
rear_right_kin0 := %:
<rear_right_kin0>;
# wheel points
CPfl_coords:=subs(S0,[comp('XYZ', CPfl, T__P)]): <%>:# 
CPfr_coords:=subs(S0,[comp('XYZ', CPfr, T__P)]): <%>:
CPrl_coords:=subs(S0,[comp('XYZ', CPrl, T__P)]): <%>:# 
CPrr_coords:=subs(S0,[comp('XYZ', CPrr, T__P)]): <%>:
Wfl_coords:=subs(S0,[comp('XYZ', W__fl, T__P)]): <%>:
Wfr_coords:=subs(S0,[comp('XYZ', W__fr, T__P)]): <%>:
Wrl_coords:=subs(S0,[comp('XYZ', W__rl, T__P)]): <%>:
Wrr_coords:=subs(S0,[comp('XYZ', W__rr, T__P)]): <%>:
# full-ext centre of mass
G_coords:=subs(S0,[comp('XYZ', G, T__P)]):<%>;NULL;
# ovarall cog
G1_coords:=subs(S0,[comp('XYZ', G1, T__P)]): <%>;
# ovarall inertia
I1_comps:=simplify(subs(S0,[I1[1][1], I1[2][2], I1[3][3], I1[3][1], I1[3][2], I1[1][2]])):
# reference point
AP_xycomps:=linearize(subs(t=0, simplify(expand(subs(S0, S0plus, [a__xP=comp('X', AP, T__P), a__yP=comp('Y', AP, T__P)])))),{mu0,phi0}):<%>;
AP_tncomps:=linearize(subs(t=0, simplify(expand(subs(S0, S0plus, [a__tP=comp('X', AP, T__P*Rotate('Z',lambda__P(t))), a__nP=comp('Y', AP, T__P*Rotate('Z',lambda__P(t)))])))),{mu0,phi0}):<%>;
# G acceleration projected in T__P (x,y components)
[comp('XYZ', AG,T__P)]:
[a__xG = %[1], a__yG = %[2], a__zG = %[3]]:
simplify(subs(S0, S0plus, %)): 
AG_comps := simplify(subs(t=0,%)): <%>;
# G total acceleration in body-fixed frame, including gravity
[comp('XYZ', AG_tot, T__V)];
AG_tot_comps := simplify(subs(t=0,eval(subs(S0, S0plus,G_veldot0, %)))): <%>:
simplify(subs(mu0=0,lambda__P0=0,z0=0,g__x=0,g__y=0,h=0,d=b,g__z=g,%)): <%>;  # check OK
# P total acceleration in road-fixed frame, including gravity
[comp('XYZ', AP_tot, T__P)]:
AP_tot_comps := simplify(subs(t=0,eval(subs(S0, S0plus,G_veldot0, %)))): <%>:
simplify(subs(mu0=0,lambda__P0=0,z0=0,g__x=0,g__y=0,h=0,d=b,g__z=g,%)): <%>;  # check OK
# Find omegaidot from kappaidot=0 and omega as a function of kappa
# Note that this is only an internal definition of kappa, which is not seen by the end user
eqkappa: <%>; # definition of kappa
subs([VSfl0 = VSfl, VRfl0 = VRfl],[VSfr0 = VSfr, VRfr0 = VRfr],[VSrl0 = VSrl, VRrl0 = VRrl],[VSrr0 = VSrr, VRrr0 = VRrr],velocity_eqns,eqkappa):
eqkappa := simplify(%): <%>;
eqomega := simplify(op(solve(eqkappa, [omega__fl(t),omega__fr(t),omega__rl(t),omega__rr(t)]))):
eqkappa0 := subs(t=0,simplify(expand(simplify(subs(S0,S0plus,eqkappa))))):
eqomegadot0 := simplify(expand(subs(S0,S0plus, diff(eqomega,t)))):
eqomega0 := subs(t=0,simplify(expand(simplify(subs(S0,S0plus,eqomega))))):
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
CA_coords:=linearize(subs(S0,[comp('XYZ', CA, T__P)]),{mu0,phi0}): Vector(%);
# Suspension anti force
anti_force0 := subs(S0, S0plus, anti_force): <%>;
# Optimize to reduce cpu cost
optimizeProc := proc(expr) local c1, c2, dc, expropt; c1 := cost(expr); print("Original cost", c1); expropt := simplify(expr, trig); expropt := optimize(expropt); c2 := cost(expropt); print("Optimized cost ", c2); dc := c1 - c2; print("Difference cost", dc); return expropt; end proc;
for k from 1 to nops(ss_eqns) do 
   ss_eqns[k] := steadyStateRes[k] = ss_eqns[k]:
end:
ss_eqns:=optimizeProc(ss_eqns):
front_tyre_kinematics := optimizeProc(front_left_kin0 union front_right_kin0):
rear_tyre_kinematics := optimizeProc(rear_left_kin0 union rear_right_kin0):
suspension_anti_force := optimizeProc(anti_force0):
for k from 1 to nops(CPfl_coords) do 
   CPfl_coords[k] := CPfl[k] = CPfl_coords[k]:
   CPfr_coords[k] := CPfr[k] = CPfr_coords[k]:
   CPrl_coords[k] := CPrl[k] = CPrl_coords[k]:
   CPrr_coords[k] := CPrr[k] = CPrr_coords[k]:
   Wfl_coords[k] := Wfl[k] = Wfl_coords[k]:
   Wfr_coords[k] := Wfr[k] = Wfr_coords[k]:
   Wrl_coords[k] := Wrl[k] = Wrl_coords[k]:
   Wrr_coords[k] := Wrr[k] = Wrr_coords[k]:
   G_coords[k] := G[k] = G_coords[k]:
   G1_coords[k] := G1[k] = G1_coords[k]:
   CA_coords[k] := CA[k] = CA_coords[k]:
   AG_tot_comps[k] := AG[k] = AG_tot_comps[k]:
   AP_tot_comps[k] := AP[k] = AP_tot_comps[k]:
end:
unassign('I1');
for k from 1 to nops(I1_comps) do 
    I1_comps[k] := I1[k] = I1_comps[k]:
end:
postproc_eqns := optimizeProc(subs(t=0,[
op(G_vel0), op(ang_vel0), op(G_veldot0), op(ang_veldot0),
op(AP_xycomps), op(AP_tncomps), op(AG_comps),
op(AG_tot_comps), op(AP_tot_comps),
op(CPfl_coords), op(CPfr_coords),op(CPrl_coords), op(CPrr_coords),
op(Wfl_coords), op(Wfr_coords),op(Wrl_coords), op(Wrr_coords),
op(CA_coords), 
op(G_coords), op(G1_coords), op(I1_comps)])):
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
fprintf(fd,"\n"):
ffclose(fd);
# Rear tyre kinematics
#print2screen := true:
fd := ffopen(cat(dirname,"rearTyreKinematics.m"), WRITE):
fprintf(fd,"%% Rear tyre kinematics  *** DO NOT EDIT ***\n"):
fprintf(fd,"%s",CodeGeneration[Matlab]([rear_tyre_kinematics], output=string)):
fprintf(fd,"\n"):
ffclose(fd);
# Suspension anti force
#print2screen := true:
fd := ffopen(cat(dirname,"suspAntiForce.m"), WRITE):
fprintf(fd,"%% Suspension anti forces  *** DO NOT EDIT ***\n"):
fprintf(fd,"%s",CodeGeneration[Matlab]([suspension_anti_force], output=string)):
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
