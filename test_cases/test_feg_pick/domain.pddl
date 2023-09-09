(define (domain fe-gripper-tamp)
  (:requirements :strips :equality)

  (:constants
    @movable @bottle @edible @medicine
  )

  (:predicates

    (drawer ?o)
    (door ?o)
    (knob ?o)
    (joint ?o)

    (edible ?o)
    (cleaningsurface ?s)
    (heatingsurface ?s)
    (controlledby ?s ?n)

    (arm ?a)
    (controllable ?a)
    (handempty ?a)
    (seconf ?q)
    (pose ?o ?p)
    (position ?o ?p)
    (isopenedposition ?o ?p)
    (isclosedposition ?o ?p)
    (grasp ?o ?g)
    (handlegrasp ?o ?g)

    (graspable ?o)
    (stackable ?o ?r)
    (containable ?o ?r)

    (originalseconf ?q)

    (kin ?a ?o ?p ?g ?q ?t)
    (kinhandle ?a ?o ?p ?g ?q1)
    (kingrasphandle ?a ?o ?p ?g ?q1 ?t)
    (kinpulldoorhandle ?a ?o ?p1 ?p2 ?g ?q1 ?q2 ?t)

    (freemotion ?p1 ?t ?p2)
    (supported ?o ?p ?r)
    (contained ?o ?p ?s)
    (traj ?t)

    (trajposecollision ?t ?o ?p)
    (cfreeposepose ?o ?p ?o2 ?p2)
    (cfreeapproachpose ?o ?p ?g ?o2 ?p2)
    (cfreetrajpose ?t ?o2 ?p2)

    (atseconf ?q)
    (atpose ?o ?p)
    (atposition ?o ?p)
    (openposition ?o ?p)
    (closedposition ?o ?p)

    (atgrasp ?a ?o ?g)
    (athandlegrasp ?a ?o ?g)
    (handlegrasped ?a ?o)

    (canmove)
    (canpull)
    (canungrasp)

    (openedjoint ?o)
    (closedjoint ?o)
    (graspedhandle ?o)

    (on ?o ?r)
    (in ?o ?r) ;;
    (holding ?a ?o)

    (unsafepose ?o ?p)
    (unsafeapproach ?o ?p ?g)
    (unsafetraj ?t)

    (poseobstacle ?o ?p ?o2)
    (approachobstacle ?o ?p ?g ?o2)

    (debug1)
    (debug2)
    (debug3)
    (debug4)

    (oftype ?o ?t)
    (storedinspace ?t ?r)
    (space ?r)
    (containobj ?o)
    (atattachment ?o ?j)
    (newposefromattachment ?o ?p)

    (cleaned ?o)
    (cooked ?o)
  )
)
