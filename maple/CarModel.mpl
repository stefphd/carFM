
# Car model
restart;
# MBSdir := cat(currentdir(),"\\MBSymba");
# MBSdir := "C:\\MBSymba\\lib";
# libname := MBSdir, libname:
with(MBSymba_r6):  
with(MBSymba_r6_tire2):  # This tyre version include spin deformation and user-defined road frame. However, does not work for complicated kinematics (because Maple newest version sucks)
with(LinearAlgebra):
interface(displayprecision=3, rtablesize=50):
# Definitions
PDEtools[declare](
   x(t),y(t),zz(t),psi(t),sigma(t),beta(t), v__w(t), # tracking coordinates and absolute yaw

   z(t), mu(t), phi(t),  # bounce, pitch, roll
 
   delta(t), delta__dot(t), delta__ddot(t), #  steer angle

   z__fl(t), z__fr(t), z__rl(t), z__rr(t),   # suspension travels at wheel center

   theta__fl(t), theta__fr(t), theta__rl(t), theta__rr(t),   # wheel spin angles
   omega__fl(t), omega__fr(t), omega__rl(t), omega__rr(t),  #   wheel spin velocities
   # 
   Tau__fl(t), Tau__fr(t), Tau__rl(t), Tau__rr(t),   # wheel torques

   F__fl(t), F__fr(t), F__rl(t), F__rr(t),    # suspension forces

   rr__fl(t), rr__fr(t), rr__rl(t), rr__rr(t),    # tyre loaded radius

prime=t,quiet);
# Small variables
linear_modeling(  {delta(t),
   psi__fl, phi__fl, mu__fl,
   psi__fr, phi__fr, mu__fr,
   psi__rl, phi__rl, mu__rl,
   psi__rr, phi__rr, mu__rr
} );
# Tracking the point P
# (distance 'd' from rear wheel at road level, in nominal condition)
TP0 := translate(x(t),y(t),zz(t)) * rotate('Z',psi(t)) * rotate('Y',sigma(t)) * rotate('X',beta(t)); # frame atached to P, expressed in ground
# express psi',sigma',beta' as a function of the angular velocities of TP0 (omegax,omegay,omegaz), and zz' as a function of x',y' and the vertical velocity v__w
project(angular_velocity(TP0), TP0):
angvel_road_eqns:=[comp_X(%)=omega__x(t),comp_Y(%)=omega__y(t),comp_Z(%)=omega__z(t)]:
project(velocity(origin(TP0)), TP0): 
linvel_road_eqns:=[comp_X(%)=v__x(t),comp_Y(%)=v__y(t),comp_Z(%)=v__w(t)]:
road_eqns:=simplify(op(solve([linvel_road_eqns[3]] union angvel_road_eqns, diff([zz(t),psi(t),sigma(t),beta(t)],t)))): <%>;
# position variables
PDEtools[declare](ft(t) , sa(t), prime=t,quiet);
pos := [x(t),y(t),psi(t)] union [phi(t),mu(t),z(t),delta(t), z__fl(t),z__fr(t),z__rl(t),z__rr(t)];
# Frames
# FIGURE OF CAR
# the T__V reference frame is attached to the chassis in the CoG, it is chosen as a moving master frame
T__V := master_frame;
# 

# Transformation T1 moves from the car CoG to the road
T1 := translate(-d,0,0) * translate(b,0,z(t)-h)* rotate('X',phi(t)) * rotate('Y',mu(t)):
# T__P reference frame is at road level, at the reference point P, SAE convention
T__P := inv_frame(T1);
T__R := T__P * translate(-d,0,0);  # same of T__P but at rear axle
PDEtools[declare]( V__x(t),V__y(t),V__z(t), Omega__x(t),Omega__y(t),Omega__z(t),omega__x(t),omega__y(t),omega__z(t), delta__dot(t), z__fldot(t),z__frdot(t),z__rldot(t),z__rrdot(t), prime=t,quiet):
# master moving frame 
use_moving_frame(V__x(t),V__y(t),V__z(t),Omega__x(t),Omega__y(t),Omega__z(t),TP0*T1):
track_eqns := simplify(subs(road_eqns, %)):
# velocity equations for first order reduction

vel := [V__x(t),V__z(t),V__y(t),delta__dot(t),Omega__z(t),Omega__x(t),Omega__y(t),z__fldot(t),z__frdot(t),z__rldot(t),z__rrdot(t)]; 
[diff(x(t),t), diff(y(t),t), omega__z(t), diff(phi(t),t), diff(mu(t),t) , diff(z(t),t)];
velocity_eqns:=simplify(simplify(linearize(op(solve(track_eqns, %)))),size)       # track equations

  union [ diff(delta(t),t)=delta__dot(t),diff(delta__dot(t),t)=delta__ddot(t), diff(z__fl(t),t)=z__fldot(t), diff(z__fr(t),t)=z__frdot(t), diff(z__rl(t),t)=z__rldot(t), diff(z__rr(t),t)=z__rrdot(t)]: # standard first order reduction

<%>;
# Reference frames summary
'T__R' = T__R; set_frame_name(T__R,'T__R'):            # attached to the road, origin at the nominal rear axle middle at road level
'T__V' = T__V; set_frame_name(T__V,'T__V'):            # attached to the  chassis, origin in G (rear+front=overall centre of mass)
T2 := T__V*translate(-b,0,h); set_frame_name(T2,'T2'): # attached to the chassis, origin at the nominal rear axle middle at road level
T4 := T__V*translate(+a,0,h); set_frame_name(T4,'T4'): # attached to the chassis, origin at the nominal front axle middle at road level
#velocity(origin(T__R)): show(%); 
#simplify(subs(velocity_eqns,%%),size): show(%);
P:=origin(T__P): show(%);
VP:=project(velocity(P),T__P):show(%);
AP:=project(velocity(simplify(subs(velocity_eqns, VP))),T__P): #show(%);
# Nominal conditions 
# (used for linearisation too)
nominal_conditions := [seq(pos[i]=0,i=1..nops(pos)), seq(vel[i]=0,i=2..nops(vel)), 
     vel[1] = U, 
     omega__fl(t) = U / R__f, omega__fr(t) = U / R__f,
     omega__rl(t) = U / R__r, omega__rr(t) = U / R__r,
     rr__fl(t) = R__f, rr__fr(t) = R__f,
     rr__rl(t) = R__r, rr__rr(t) = R__r,
     x__fl=0,x__fr=0,x__rl=0,x__rr=0,
     y__fl=0,y__fr=0,y__rl=0,y__rr=0,
     psi__fl=0,psi__fr=0,psi__rl=0,psi__rr=0,
     phi__fl=0,phi__fr=0,phi__rl=0,phi__rr=0,
     mu__fl=0,mu__fr=0,mu__rl=0,mu__rr=0
]: 
nominal_conditions;
t_forces := [ X__fl(t),Y__fl(t), N__fl(t), X__fr(t),Y__fr(t), N__fr(t), X__rl(t),Y__rl(t), N__rl(t), X__rr(t),Y__rr(t), N__rr(t) ]; 
nominal := x -> collect( simplify(eval(subs(nominal_conditions, eval(x)))) , t_forces):
### T2 = nominal(T__R); # also, T2=T__R in nominal conditions
;
my_linearize := x -> linearize(x, {
   mu(t), phi(t)
});
# Bodies
# utilities
anti_body := proc(BB::BODY)
  local i,anti_BB:
  anti_BB := copy(BB):
  anti_BB[frame]   := map(evalm,subs( seq(args[i],i=2..nargs), BB[frame])): 
  anti_BB[mass]    := -BB[mass]:
  anti_BB[inertia] := evalm(-BB[inertia]):
  copy(anti_BB):  
end:
show(T__R);
G := origin(T__V): 
'G' = show(project(G,T__R));
'G0' = show(nominal(project(G,T__R)));
a := w - b;   # use wheelbase w and rear CoG distance b as model parameters
I__yz:=0:  I__xy:=0:  #because of the simmetry with respect XY plane
car_body := make_BODY(T__V,m,I__xx,I__yy,I__zz,0,I__xz,0): show(car_body);
# Wheel non-spinning reference frames at wheel center, rotated by steer for the front, translated by x__i,y__i, and rotated by toe, camber 
# t__f, t__r: half track-widths
# x__ij,y__ij,psi__ij,phi__ij,mu__ij are relative to the chassis, so they can be easily found from a kinematic simulation of the suspension only (i.e. fixed-chassis)
T__flw := T4 * translate(0,-t__f,0) * translate(0,0,-R__f+z__fl(t)) * rotate('Z', delta(t))
           * translate(x__fl,y__fl,0) * rotate('Z', psi__fl) * rotate('X', phi__fl) * rotate('Y', mu__fl); # front left
T__frw := T4 * translate(0,+t__f,0) * translate(0,0,-R__f+z__fr(t)) * rotate('Z', delta(t))
           * translate(x__fr,y__fr,0) * rotate('Z', psi__fr) * rotate('X', phi__fr) * rotate('Y', mu__fr); # front right
T__rlw := T2 * translate(0,-t__r,0) * translate(0,0,-R__r+z__rl(t))
           * translate(x__rl,y__rl,0) * rotate('Z', psi__rl) * rotate('X', phi__rl) * rotate('Y', mu__rl); # rear left
T__rrw := T2 * translate(0,+t__r,0) * translate(0,0,-R__r+z__rr(t))
           * translate(x__rr,y__rr,0) * rotate('Z', psi__rr) * rotate('X', phi__rr) * rotate('Y', mu__rr); # rear right
# Wheel centers
W__fl := origin(T__flw): show(W__fl):
project(W__fl,T__R): W__fl = show(simplify(%)),
W__fl0 = show(nominal(%));
W__fr := origin(T__frw): show(W__fr):
project(W__fr,T__R): W__fr = show(simplify(%)),
W__fr0 = show(nominal(%));
W__rl := origin(T__rlw): show(W__rl):
project(W__rl,T__R): W__rl = show(simplify(%)),
W__rl0 = show(nominal(%));
W__rr := origin(T__rrw): show(W__rl):
project(W__rr,T__R): W__rr = show(simplify(%)),
W__rr0 = show(nominal(%));
# Wheel bodies
`diff/theta__fl` := proc(t) omega__fl(t) end proc: # define FL wheel spin velocity
`diff/theta__fr` := proc(t) omega__fr(t) end proc: # define FR wheel spin velocity
`diff/theta__rl` := proc(t) omega__rl(t) end proc: # define RL wheel spin velocity
`diff/theta__rr` := proc(t) omega__rr(t) end proc: # define RR wheel spin velocity
;
front_left_wheel := make_BODY( combine(T__flw * rotate('Y',-theta__fl(t)) , trig),0,0,i__fy,0):  show(front_left_wheel);
# Anti-body
anti_front_left_wheel := simplify(subs(z__fl(t)=0,theta__fl(t)=0,delta(t)=0,copy(front_left_wheel))):
anti_front_left_wheel[mass] := -front_left_wheel[mass]: anti_front_left_wheel[inertia] := -front_left_wheel[inertia]:
show(anti_front_left_wheel);
front_right_wheel := make_BODY( combine(T__frw * rotate('Y',-theta__fr(t)) , trig),0,0,i__fy,0):  show(front_right_wheel);
# Anti-body
anti_front_right_wheel := simplify(subs(z__fr(t)=0,theta__fr(t)=0,delta(t)=0,copy(front_right_wheel))):
anti_front_right_wheel[mass] := -front_left_wheel[mass]: anti_front_right_wheel[inertia] := -front_right_wheel[inertia]:
show(anti_front_right_wheel);
rear_left_wheel := make_BODY( combine(T__rlw * rotate('Y',-theta__rl(t)) , trig),0,0,i__ry,0):  show(rear_left_wheel);
# Anti-body
anti_rear_left_wheel := simplify(subs(z__rl(t)=0,theta__rl(t)=0,copy(rear_left_wheel))):
anti_rear_left_wheel[mass] := -rear_left_wheel[mass]: anti_rear_left_wheel[inertia] := -rear_left_wheel[inertia]:
show(anti_rear_left_wheel);
rear_right_wheel := make_BODY( combine(T__rrw * rotate('Y',-theta__rr(t)) , trig),0,0,i__ry,0):  show(rear_right_wheel);
# Anti-body
anti_rear_right_wheel := simplify(subs(z__rr(t)=0,theta__rr(t)=0,copy(rear_right_wheel))):
anti_rear_right_wheel[mass] := -rear_right_wheel[mass]: anti_rear_right_wheel[inertia] := -rear_right_wheel[inertia]:
show(anti_rear_right_wheel);
# Unsprung masses include wheel/tyre, suspension mass, wheel hub, brakes etc.
front_left_unsprung := make_BODY(T__flw, m__fu, 0,0,0): show(front_left_unsprung);
# Anti-body
anti_front_left_unsprung := simplify(subs(z__fl(t)=0,copy(front_left_unsprung))):
anti_front_left_unsprung[mass] := -front_left_unsprung[mass]:
anti_front_left_unsprung[inertia] := -front_left_unsprung[inertia]:
show(anti_front_left_unsprung);
front_right_unsprung := make_BODY(T__frw, m__fu, 0,0,0): show(front_right_unsprung);
# Anti-body
anti_front_right_unsprung := simplify(subs(z__fr(t)=0,copy(front_right_unsprung))):
anti_front_right_unsprung[mass] := -front_right_unsprung[mass]:
anti_front_right_unsprung[inertia] := -front_right_unsprung[inertia]:
show(anti_front_right_unsprung);
rear_left_unsprung := make_BODY(T__rlw, m__ru, 0,0,0): show(rear_left_unsprung);
# Anti-body
anti_rear_left_unsprung := simplify(subs(z__rl(t)=0,copy(rear_left_unsprung))):
anti_rear_left_unsprung[mass] := -rear_left_unsprung[mass]:
anti_rear_left_unsprung[inertia] := -rear_left_unsprung[inertia]:
show(anti_rear_left_unsprung);
rear_right_unsprung := make_BODY(T__rrw, m__ru, 0,0,0): show(rear_right_unsprung);
# Anti-body
anti_rear_right_unsprung := simplify(subs(z__rr(t)=0,copy(rear_right_unsprung))):
anti_rear_right_unsprung[mass] := -rear_right_unsprung[mass]:
anti_rear_right_unsprung[inertia] := -rear_right_unsprung[inertia]:
show(anti_rear_right_unsprung);
# Transmission inertia
transmission_gyro := make_BODY(T2*rotate('Y', -1/2 * (theta__rl(t)+theta__rr(t))), 0,0,i__tg,0): show(transmission_gyro); # rotating at the average angular velocity of rear wheels
kinetic_energy(transmission_gyro);
# Tyre forces
Describe(make_TIRE);
PDEtools[declare](op(t_forces), prime=t, qiet):
# Front left tyre
show(front_left_wheel);
front_left_tyre := make_TIRE(front_left_wheel, 0, 0, 0, 0, rr__fl(t),
X__fl(t),Y__fl(t),-N__fl(t),T__flx,T__fly,T__flz,T__R): show(%);
(TF_fl,TT_fl) := tire_forces(front_left_tyre):
show(TF_fl); show(TT_fl);
# Kinematics
(CPfl, VSfl, VNfl, VRfl) := tire_kinematics(front_left_tyre):
show(nominal(project(CPfl,T__R)));
<VSfl,VNfl,VRfl>;
# Tire deflection
xifl_eqn := [xi__fl(t) = R__f - rr__fl(t)];
# Front right tyre
show(front_right_wheel);
front_right_tyre := make_TIRE(front_right_wheel, 0, 0, 0, 0, rr__fr(t),
X__fr(t),Y__fr(t),-N__fr(t),T__frx,T__fry,T__frz,T__R): show(%);
(TF_fr,TT_fr) := tire_forces(front_right_tyre):
# show(project(TF_fr, T__R)); show(project(TF_fr, ground));
# Kinematics
(CPfr, VSfr, VNfr, VRfr) := tire_kinematics(front_right_tyre):
show(nominal(project(CPfr,T__R)));
<VSfr,VNfr,VRfr>;
# Tire deflection
xifr_eqn := [xi__fr(t) = R__f - rr__fr(t)];
# Rear left tyre
show(rear_left_wheel);
rear_left_tyre := make_TIRE(rear_left_wheel, 0, 0, 0, 0, rr__rl(t),
X__rl(t),Y__rl(t),-N__rl(t),T__rlx,T__rly,T__rlz,T__R): show(%);
(TF_rl,TT_rl) := tire_forces(rear_left_tyre):
# show(project(TF_rl, T__R)); show(project(TF_rl, ground));
# Kinematics
(CPrl, VSrl, VNrl, VRrl) := tire_kinematics(rear_left_tyre):
show(nominal(project(CPrl,T__R)));
<VSrl,VNrl,VRrl>;
# Tire deflection
xirl_eqn := [xi__rl(t) = R__r - rr__rl(t)];
# Rear right tyre
show(rear_right_wheel);
rear_right_tyre := make_TIRE(rear_right_wheel, 0, 0, 0, 0, rr__rr(t),
X__rr(t),Y__rr(t),-N__rr(t),T__rrx,T__rry,T__rrz,T__R): show(%);
(TF_rr,TT_rr) := tire_forces(rear_right_tyre):
# show(project(TF_rr, T__R)); show(project(TF_rr, ground));
# Kinematics
(CPrr, VSrr, VNrr, VRrr) := tire_kinematics(rear_right_tyre):
show(nominal(project(CPrr,T__R)));
<VSrr,VNrr,VRrr>;
# Tire deflection
xirr_eqn := [xi__rr(t) = R__r - rr__rr(t)];
# Tire camber angles
frontTyreAngles := [ca__fl = rhs(front_left_tyre['angles'][2]), ca__fr = rhs(front_right_tyre['angles'][2])]:<%>;  # front 
rearTyreAngles := [ca__rl = rhs(rear_left_tyre['angles'][2]), ca__rr = rhs(rear_right_tyre['angles'][2])]:<%>;  # rear
# Force
# Gravity
_gravity := make_VECTOR(T__R, g__x,g__y,g__z):
show(_gravity);
show(project(_gravity,master_frame));
# Aerodynamics
simplify(1/2 * ( comp_X(CPrl,T__R) + comp_X(CPrr,T__R) ) ),
simplify(1/2 * ( comp_Y(CPrl,T__R) + comp_Y(CPrr,T__R) ) ),
simplify(1/2 * ( comp_Z(CPrl,T__R) + comp_Z(CPrr,T__R) ) ) :  # coords of point at the middle of the rear axle, at road level
T__R * translate(%):  # aligned with T__R at rear axle middle, at road level
CA := make_POINT(%, b__A,0,-h__A):show(nominal(CA));  # point of application of aerodynamic forces
aero_force  := make_FORCE (T__R, F__Ax, F__Ay, F__Az, CA, car_body): show(%);  # aero forces in SAE, applied at CA
aero_torque := make_TORQUE(T__R, M__Ax, M__Ay, M__Az, car_body): show(%);# aero moments in SAE
;
NULL;
# External forces and moments applied to ref point P
ext_force  := make_FORCE (T__R, F__x, F__y, F__z, P, car_body): show(%);# external force
ext_torque := make_TORQUE(T__R, M__x, M__y, M__z, car_body): show(%); # external torque
# Suspension forces
front_left_force := make_FORCE(T__flw, 0,0,F__fl(t), W__fl, front_left_wheel,car_body): show(%);
front_right_force := make_FORCE(T__frw, 0,0,F__fr(t), W__fr, front_right_wheel,car_body): show(%);
rear_left_force := make_FORCE(T__rlw, 0,0,F__rl(t), W__rl, rear_left_wheel,car_body): show(%);
rear_right_force := make_FORCE(T__rrw, 0,0,F__rr(t), W__rr, rear_right_wheel,car_body): show(%);
# Anti-geometry
# Use lagrange approach to find the generalized force due to suspension motion
Describe(generalized_force);
T0 := ground * translate(X(q),Y(q),q) * rotate('Z',Psi(q)) * rotate('X', Phi(q)) * rotate('Y', Mu(q)):  # attached to the wheel center
C := make_POINT(T0,0,0,rr): # at the contact point
body := make_BODY(T0,0): # a fake body where forces are applied
F0 := make_FORCE(ground, F__x0,F__y0,0, C, body):
M0 := make_TORQUE(ground, M__x0, M__y0, M__z0, body):
generalized_force({F0,M0},q):
subs(diff(Psi(q),q)=Psi__q, diff(Phi(q),q)=Phi__q, diff(Mu(q),q)=Mu__q, diff(X(q),q)=X__q, diff(Y(q),q)=Y__q, %): # remove diff(...)
linearize(%, {Mu(q),Phi(q),Psi(q)}):   # linearize for small angles
subs(Psi(q)=Psi,Phi(q)=Phi,Mu(q)=Mu,X(q)=X,Y(q)=Y,%):   # remove time
collect(%,{F__x0,F__y0,M__x0,M__y0,M__z0}):  # collect
susp_anti_force := unapply(%, rr, F__x0, F__y0, M__x0, M__y0, M__z0, Psi, Phi, Mu, X__q, Y__q, Psi__q, Phi__q, Mu__q);
# Equations of motion
# Equations of motion
full_car := [car_body, 
front_left_wheel, front_right_wheel, rear_left_wheel, rear_right_wheel,
 front_left_unsprung, front_right_unsprung, rear_left_unsprung,rear_right_unsprung,
anti_front_left_wheel, anti_front_right_wheel, anti_rear_left_wheel, anti_rear_right_wheel,
anti_front_left_unsprung, anti_front_right_unsprung, anti_rear_left_unsprung,anti_rear_right_unsprung]
union
[ext_force, ext_torque, aero_force, aero_torque, 
front_left_force, front_right_force, rear_left_force, rear_right_force,
TF_fl, TF_fr, TF_rl, TF_rr,
TT_fl, TT_fr, TT_rl, TT_rr,
transmission_gyro];
show(T2); show(T__R);
eqns_N := project(newton_equations(full_car),T2): show(eqns_N):
nominal(comp_X(eqns_N)); nominal(comp_Y(eqns_N)); nominal(comp_Z(eqns_N));
#eqns_EA := euler_equations(full_car, origin(T__R)): codegen[cost](comp_XYZ(eqns_EA));
eqns_EG := euler_equations(full_car, G): codegen[cost](comp_XYZ(eqns_EG));
eqns_EG := simplify(eqns_EG): codegen[cost](comp_XYZ(eqns_EG));
eqns_E := eqns_EG: 
#comp_X(eqns_E); comp_Y(eqns_E); comp_Z(eqns_E);
nominal(comp_X(eqns_E)); nominal(comp_Y(eqns_E)); nominal(comp_Z(eqns_E));
# Unsprung masses
# rr, F__x0, F__y0, M__x0, M__y0, M__z0, Psi, Phi, Mu, X__q, Y__q, Psi__q, Phi__q, Mu__q
newton_equations( {front_left_unsprung, front_left_wheel, anti_front_left_wheel, front_left_force, TF_fl, TT_fl}):
subs(velocity_eqns,eval( subs(velocity_eqns, comp_Z(%)))):
eqns_us_fl := % - susp_anti_force(rr__fl(t),X__fl(t),Y__fl(t),T__flx,T__fly,T__flz,psi__fl,phi__fl,mu__fl,D__x__fl,D__y__fl,D__psi__fl,D__phi__fl,D__mu__fl);
nominal(%);

newton_equations( {front_right_unsprung, front_right_wheel, anti_front_left_wheel, front_right_force, TF_fr, TT_fr}):
subs(velocity_eqns,eval( subs(velocity_eqns, comp_Z(%)))):
eqns_us_fr := % - susp_anti_force(rr__fr(t),X__fr(t),Y__fr(t),T__frx,T__fry,T__frz,psi__fr,phi__fr,mu__fr,D__x__fr,D__y__fr,D__psi__fr,D__phi__fr,D__mu__fr);
nominal(%);
newton_equations( {rear_left_unsprung, rear_left_wheel, anti_rear_left_wheel, rear_left_force, TF_rl, TT_rl}):
subs(velocity_eqns,eval( subs(velocity_eqns, comp_Z(%)))):
eqns_us_rl := % - susp_anti_force(rr__rl(t),X__rl(t),Y__rl(t),T__rlx,T__rly,T__rlz,psi__rl,phi__rl,mu__rl,D__x__rl,D__y__rl,D__psi__rl,D__phi__rl,D__mu__rl);
nominal(%);
newton_equations( {rear_right_unsprung, rear_right_wheel, anti_rear_left_wheel, rear_right_force, TF_rr, TT_rr}):
subs(velocity_eqns,eval( subs(velocity_eqns, comp_Z(%)))):
eqns_us_rr := % - susp_anti_force(rr__rr(t),X__rr(t),Y__rr(t),T__rrx,T__rry,T__rrz,psi__rr,phi__rr,mu__rr,D__x__rr,D__y__rr,D__psi__rr,D__phi__rr,D__mu__rr);
nominal(%);
# Wheel spin equations
euler_equations( {front_left_wheel, TF_fl, TT_fl}, CoM(front_left_wheel) ):
Tau__fl(t) + comp_Y(%, T__flw):
eqns_spin_fl := subs( velocity_eqns, eval(subs(velocity_eqns,%)));
euler_equations( {front_right_wheel, TF_fr, TT_fr}, CoM(front_right_wheel) ):
Tau__fr(t) + comp_Y(%, T__frw):
eqns_spin_fr := subs( velocity_eqns, eval(subs(velocity_eqns,%)));
euler_equations( {rear_left_wheel, TF_rl, TT_rl}, CoM(rear_left_wheel) ):
Tau__rl(t) - i__ta*diff(omega__rl(t),t) + comp_Y(%, T__rlw):
eqns_spin_rl := subs( velocity_eqns, eval(subs(velocity_eqns,%)));
euler_equations( {rear_right_wheel, TF_rr, TT_rr}, CoM(rear_right_wheel) ):
Tau__rr(t) -  i__ta*diff(omega__rr(t),t) + comp_Y(%, T__rrw):
eqns_spin_rr := subs( velocity_eqns, eval(subs(velocity_eqns,%)));
# Tyre contact equations
equ_CflZ := comp_Z(CPfl,T__R); nominal(%);
equ_CfrZ := comp_Z(CPfr,T__R); nominal(%);
equ_CrlZ := comp_Z(CPrl,T__R); nominal(%);
equ_CrrZ := comp_Z(CPrr,T__R); nominal(%);
# DAE formulation EE*dx/dt = F(x,t,u)
pos; vel;
# Assemble equation and convert to first order
xx := pos[4..-1] union vel
      union [omega__fl(t),omega__fr(t),omega__rl(t),omega__rr(t)]
      union [rr__fl(t),rr__fr(t),rr__rl(t),rr__rr(t)];
xx_eqns := 
map(lhs,velocity_eqns[4..-1])-map(rhs,velocity_eqns[4..-1]) union
subs(velocity_eqns,
eval(subs(velocity_eqns, [
   comp_XYZ(eqns_N,T2), comp_XYZ(eqns_E,T2),
   eqns_us_fl, eqns_us_fr, eqns_us_rl, eqns_us_rr,
   eqns_spin_fl, eqns_spin_fr, eqns_spin_rl, eqns_spin_rr,
   equ_CflZ, equ_CfrZ, equ_CrlZ, equ_CrrZ]))):
nops(xx_eqns) = nops(xx); if not(%) then error("The number of equations does not match the number of variables"); end;
   
# Inputs
uu := [delta__ddot(t), Tau__fl(t), Tau__fr(t), Tau__rl(t), Tau__rr(t)]:
nu := nops(uu): nu=uu;
# DAE formulation
EE,FF := GenerateMatrix(xx_eqns, diff(xx,t)): 
for var in vel union pos do
   print("EE has",var,": ", has(EE,var));
end do;
# Bodies composition
#show(G);
VG := project(velocity(G),T__R): show(VG);
# Find overall CG and inertia: merge all bodies into a single one
bodies := [car_body, 
front_left_wheel, front_right_wheel, rear_left_wheel, rear_right_wheel,
 front_left_unsprung, front_right_unsprung, rear_left_unsprung,rear_right_unsprung,
anti_front_left_wheel, anti_front_right_wheel, anti_rear_left_wheel, anti_rear_right_wheel,
anti_front_left_unsprung, anti_front_right_unsprung, anti_rear_left_unsprung,anti_rear_right_unsprung]; 
allbody := make_BODY(T__V,0,0,0,0,0,0,0):
for body in bodies do
   allbody := bodies_union(allbody, body, T__V);
end do:
G__all:=CoM(allbody):
I__all:=subs(psi(t)=0,sigma(t)=0,beta(t)=0,x(t)=0,y(t)=0, allbody['inertia']):
# Simplify the expression
I__all := simplify(I__all):
# Check total mass
allbody['mass'];   #  = m OK
simplify(nominal(I__all));  # = [Ixx, 0, -Ixz; 0, Iyy, 0; -Ixz, 0, Izz] OK
# This is to remove nonlinear terms in delta (like delta^2)
has(I__all, delta(t)), 
has(I__all, delta(t)^2),
has(I__all, cos(delta(t))),
has(I__all, sin(delta(t))),
has(I__all, tan(delta(t))); 
I__all_linearised := subs(delta(t)^2=0, I__all): 
has(I__all_linearised, delta(t)), 
has(I__all_linearised, delta(t)^2);
simplify(coeff(I__all, delta(t)^2)*delta(t)^2 + I__all_linearised - I__all);  # Check = 0
I__all := I__all_linearised:
# Save the model
save(pos,vel,velocity_eqns,G,VG,P,VP,AP,
VSfl,VNfl,VRfl,VSfr,VNfr,VRfr,
VSrl,VNrl,VRrl,VSrr,VNrr,VRrr,
CPfl,CPfr,CPrl,CPrr,
frontTyreAngles, rearTyreAngles,
W__fl,W__fr,W__rl,W__rr,
T__R,T__P,T__V,
G__all,I__all,CA,
xifl_eqn,xifr_eqn,xirl_eqn,xirr_eqn,
xx,uu,xx_eqns,
"CarModel.mla");
NULL;
