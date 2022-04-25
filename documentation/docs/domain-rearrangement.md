# The Rearrangement Domain

As an example of long-horizon TAMP problems in large, complex environment, we created the domain `rearrangement.pddl` that includes the operators and axioms for a variety of household rearrangement problems.

## General-Purpose Skills

<table class="multicol">

<tr>
<td width="34%">

<img src="../gifs/skill-pick-place.gif"></img>
```txt

Pick and place
```

</td>
<td width="36.25%">

<img src="../gifs/skill-pull-drawer.gif"></img>

```txt

Open a drawer
```

</td>
<td width="27.5%">

<img src="../gifs/skill-knob-door.gif"></img>
```txt

Open a door
```

</td>
</tr>
</table>

Actions on one object/link may cause effects on the state of other object/link.

<table class="multicol">
<tr>
<td width="50%">

<img src="../gifs/skill-knob-faucet.gif"></img>

```txt

Turn a knob from the top
```

</td>
<td width="50%">

<img src="../gifs/skill-knob-stove.gif"></img>

```txt

Turn a knob from any direction
```

</td>
</tr>
</table>

## Four Categories of Problems



## The Domain

The skills and corresponding constraints with the world are included in the domain via PDDL operators.

### Operators

#### **Pick**

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
(:stream inverse-kinematics-wconf
  :inputs (?a ?o ?p ?g ?w)
  :domain (and (Controllable ?a) (Pose ?o ?p) (Grasp ?o ?g) (WConf ?w))
  :outputs (?q ?t)
  :certified (and (BConf ?q) (ATraj ?t) (KinWConf ?a ?o ?p ?g ?q ?t ?w))
)
```

* checking that the grasp approach path (straight line) is free of collisions from movable objects with `(not (UnsafeApproach ?o ?p ?g))`
* checking that the arm path from default arm pose to grasp approach pose is free of collisions from movable objects with `(not (UnsafeATraj ?t))`


#### **Place**

Place, compared to pick, requires additionally

* sampling of placement pose `?p` with

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

---

## The Axioms
