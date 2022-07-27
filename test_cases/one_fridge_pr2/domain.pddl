(define (domain pr2-tamp)
  (:requirements :strips :equality)

  (:constants
    @movable @bottle @edible
  )

  (:predicates
    (arm ?a)
    (sink ?o)
    (stove ?o)
    (drawer ?o) ;;
    (door ?o) ;;
    (knob ?o) ;;
    (joint ?o)

    (edible ?o)
    (cleaningsurface ?s)
    (heatingsurface ?s)
    (controlledby ?s ?n)

    (aconf ?a ?q)
    (defaultaconf ?a ?q)
    (bconf ?q)
    (ungraspbconf ?q)
    (pose ?o ?p)
    (magicpose ?o ?p)  ;; for teleport debugging
    (linkpose ?o ?p)
    (position ?o ?p)  ;; joint position of a body
    (isopenedposition ?o ?p)  ;;
    (isclosedposition ?o ?p)  ;; assume things start out closed
    (grasp ?o ?g)
    (handlegrasp ?o ?g)

    (wconf ?w)
    (inwconf ?w)
    (newwconfp ?w1 ?o ?p ?w2)
    (newwconfpst ?w1 ?o ?pst ?w2)

    (controllable ?o)
    (graspable ?o)
    (stackable ?o ?r)
    (containable ?o ?r)  ;;

    (kin ?a ?o ?p ?g ?q ?t)
    (kinwconf ?a ?o ?p ?g ?q ?t ?w)
    (kingrasphandle ?a ?o ?p ?g ?q ?aq ?t)  ;; grasp a handle
    (kingrasphandlewconf ?a ?o ?p ?g ?q ?aq ?t ?w)  ;; grasp a handle
    (kinungrasphandle ?a ?o ?p ?g ?q ?aq1 ?aq2 ?t)  ;; ungrasp a handle
    (kinungrasphandlewconf ?a ?o ?p ?g ?q ?aq1 ?aq2 ?t ?w)  ;; ungrasp a handle
    (kinpulldrawerhandle ?a ?o ?p1 ?p2 ?g ?q1 ?q2 ?t)  ;; pull the handle
    (kinpulldoorhandle ?a ?o ?p1 ?p2 ?g ?q1 ?q2 ?bt ?aq1 ?aq2 ?at)  ;; pull the handle
    (kinturnknob ?a ?o ?p1 ?p2 ?g ?q ?aq1 ?aq2 ?at)

    (basemotionwconf ?q1 ?t ?q2 ?w)
    (basemotion ?q1 ?t ?q2)
    (armmotion ?a ?q1 ?t ?q2)
    (supported ?o ?p ?r)
    (contained ?o ?p ?s) ;; aabb contains
    (btraj ?t)
    (atraj ?t)

    (trajposecollision ?t ?o ?p)
    (trajarmcollision ?t ?a ?q)
    (trajgraspcollision ?t ?a ?o ?g)
    (cfreeposepose ?o ?p ?o2 ?p2)
    (cfreeapproachpose ?o ?p ?g ?o2 ?p2)
    (cfreetrajpose ?t ?o2 ?p2)
    (isclosedposition ?o ?p)  ;;
    (isopenposition ?o ?p)  ;;

    (cfreebtrajpose ?t ?o2 ?p2)

    (atpose ?o ?p)
    (atlinkpose ?o ?p)
    (atposition ?o ?p)  ;; joint position of a body
    (openposition ?o ?p)  ;; joint position of a body
    (closedposition ?o ?p)  ;; joint position of a body
    (atgrasp ?a ?o ?g)
    (athandlegrasp ?a ?o ?g)  ;; holding the handle
    (handlegrasped ?a ?o)  ;; holding the handle
    (knobturned ?a ?o)  ;; holding the knob
    (handempty ?a)
    (atbconf ?q)
    (ataconf ?a ?q)

    (canmove)
    (canpull)
    (canungrasp)
    (cleaned ?o)
    (cooked ?o)
    (openedjoint ?o) ;;
    (closedjoint ?o) ;;
    (graspedhandle ?o) ;;

    (on ?o ?r)
    (in ?o ?r) ;;
    (holding ?a ?o)

    (unsafepose ?o ?p)
    (unsafeapproach ?o ?p ?g)
    (unsafeatraj ?t)
    (unsafebtraj ?t)
    (poseobstacle ?o ?p ?o2)
    (approachobstacle ?o ?p ?g ?o2)
    (atrajobstacle ?t ?o)

    (debug1)
    (debug2)
    (debug3)
  
    (veggiecabbage ?x)
    (defaultconf ?x ?y)
    (object ?x)
    (floor ?x)
    (space ?x)
    (isjointto ?x ?y)
    (supporter ?x)
  )
)