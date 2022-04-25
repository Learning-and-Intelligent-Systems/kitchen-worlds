# II) Operators

The skills and corresponding constraints with the world are included in the domain via PDDL operators.

## General skills

### **Pick**

```
(:action pick
  :parameters (?a ?o ?p ?g ?q ?t ?w)
  :precondition (and (KinWConf ?a ?o ?p ?g ?q ?t ?w)
                     (AtPose ?o ?p) (HandEmpty ?a)
                     (AtBConf ?q) (InWConf ?w)
                     (not (UnsafeApproach ?o ?p ?g))
                     (not (UnsafeATraj ?t))
                )
  :effect (and (AtGrasp ?a ?o ?g) (CanMove)
               (not (AtPose ?o ?p)) (not (HandEmpty ?a))
               (increase (total-cost) (PickCost))
          )
)
```

Pick with a single gripper grasp requires

* the joint sampling of grasp `?g` and arm trajectory `?t` with `(KinWConf ?a ?o ?p ?g ?q ?t ?w)`, which is a constraint that says _"pick with arm `?a` the object `?o` from its original pose `?p` by going to base configuration `?q` with arm trajectory `?t`, in world configuration `?w`"_

```
(:stream sample-grasp
  :inputs (?o)
  :domain (Graspable ?o)
  :outputs (?g)
  :certified (Grasp ?o ?g)
)
(:stream inverse-kinematics-wconf
  :inputs (?a ?o ?p ?g ?w)
  :domain (and (Controllable ?a) (Pose ?o ?p) (Grasp ?o ?g) (WConf ?w))
  :outputs (?q ?t)
  :certified (and (BConf ?q) (ATraj ?t) (KinWConf ?a ?o ?p ?g ?q ?t ?w))
)
```

* checking that the grasp approach path (straight line) is free of collisions from movable objects with `(not (UnsafeApproach ?o ?p ?g))`
* checking that the arm path from default arm pose to grasp approach pose is free of collisions from movable objects with `(not (UnsafeATraj ?t))`


### **Place**

```
(:action place
  :parameters (?a ?o ?p ?g ?q ?t ?w)
  :precondition (and (KinWConf ?a ?o ?p ?g ?q ?t ?w)
                     (AtGrasp ?a ?o ?g) (AtBConf ?q) (InWConf ?w)
                     (not (UnsafePose ?o ?p))
                     (not (UnsafeApproach ?o ?p ?g))
                     (not (UnsafeATraj ?t))
                )
  :effect (and (AtPose ?o ?p) (HandEmpty ?a) (CanMove)
               (not (AtGrasp ?a ?o ?g))
               (increase (total-cost) (PlaceCost))
          )
)
```

Place, compared to pick, requires additionally

* sampling of placement pose `?p` on possible surfaces that supports the object

```
(:stream sample-pose
  :inputs (?o ?r)
  :domain (Stackable ?o ?r)
  :outputs (?p)
  :certified (and (Pose ?o ?p) (Supported ?o ?p ?r))
)
```

### **GraspHandle**

```
(:action grasp_handle
  :parameters (?a ?o ?p ?g ?q ?aq1 ?aq2 ?t)
  :precondition (and (Joint ?o)
                     (KinGraspHandle ?a ?o ?p ?g ?q ?aq2 ?t)
                     (AtPosition ?o ?p) (HandEmpty ?a)
                     (AtBConf ?q) (AtAConf ?a ?aq1)
                )
  :effect (and (AtHandleGrasp ?a ?o ?g) (not (HandEmpty ?a))
               (not (CanMove)) (CanPull) (not (CanUngrasp))
               (not (AtAConf ?a ?aq1)) (AtAConf ?a ?aq2)
               (increase (total-cost) (PickCost))
          )
)
```

Grasp the handle to a door, a drawer, a knob, or a movable, is similar to pick,

## High-level operators
