# III) Axioms

Axioms are important for many reasons:

* they can simplify the effects of operators, e.g. arm trajectory is valid if it doesn't collide with all movable objects
```
(:derived (UnsafeBTraj ?t)
  (exists (?o2 ?p2) (and
    (BTraj ?t) (Pose ?o2 ?p2) (AtPose ?o2 ?p2) (not (CFreeBTrajPose ?t ?o2 ?p2))))
)
```

* they can abstract away continuous variables, e.g. objects is `on` a surface as long as it's supported by the surface
```
(:derived (On ?o ?r)
  (exists (?p) (and (Supported ?o ?p ?r) (AtPose ?o ?p)))
)
```

* they can introduce semantic equivalence, e.g. a heated edible object is considered cooked while a heated inflammable object is considered dangerous
```
(:derived (Cooked ?o)
  (and (Heated ?o) (Edible ?o))
)
```


* they can encode complicated symbolic goals, e.g. make fried rice in my kitchen means having rice served in a container, where the rice has been cooked, seasoned with sea salt, and covered by oven-roasted assorted veggies and two pieces of cooked lunch meat.

```
( :derived ( EnableFridRice ?rice1 ?veggie1 ?veggie2 ?meat1 ?meat2 ?container1 )
  ( and
      ( Cooked ?rice1 )
      ( Cleaned ?veggie1 )
      ( Cleaned ?veggie2 )
      ( Roasted ?veggie1 )
      ( Roasted ?veggie2 )
      ( Cooked ?veggie1 )
      ( Cooked ?veggie2 )
      ( Cooked ?meat1 )
      ( Cooked ?meat2 )

      ( Above ?veggie1 ?rice1 )
      ( Above ?veggie2 ?rice1 )
      ( Above ?meat1 ?rice1 )
      ( Above ?meat2 ?rice1 )
      ( In ?rice1 ?container1 )
  )
)

( :derived ( ExistOmelette ?table1 )
    ( exists ( ?rice1 ?veggie1 ?veggie2 ?meat1 ?meat2 ?container1 )
        ( and
            ( Rice ?rice1 ) ( Plate ?plate1 ) ( Table ?table1 ) ( On ?container1 ?table1 )
            ( Veggie ?veggie1 ) ( Veggie ?veggie2 ) ( not ( Equal ?veggie1 ?veggie2 ) )
            ( Meat ?meat1 ) ( Meat ?meat2 ) ( not ( Equal ?meat1 ?meat2 ) )
            ( EnableFridRice ?egg1 ?veggie1 ?plate1 )
            ( On ?container1 ?table1 )
        )
    )
)
```


Let's categorize all the axioms included in the rearrangement domain by their level of abstractions.

## 1. Low-level routines

## 2. Abstracting away

## 3. Semantic equivalence

## 4. Task-specific goals



## Limitations

* The effects derived from axioms disappears once the preconditions doesn't hold. For example, it's impossible to encode the common sense rule that objects become and stay cooked after they are placed on a heating surface (pot) that's activated (as the burner underneath it is turned on). Instead, an high-level operator `cook` needs to be introduced.
```
(:action cook
  :parameters (?o ?r)
  :precondition (and (Stackable ?o ?r) (Stove ?r) (On ?o ?r) )
  :effect (and (Heated ?o))
)
```

* They cause a single effect, while many transitions have multiple effects on the geometric and symbolic levels


instead of
