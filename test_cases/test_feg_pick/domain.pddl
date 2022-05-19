(define (domain fe-gripper-tamp)
  (:requirements :strips :equality)
  (:predicates

    (drawer ?o) ;;
    (door ?o) ;;
    (knob ?o) ;;
    (joint ?o)

    (edible ?o)
    (cleaningsurface ?s)
    (heatingsurface ?s)
    (controlledby ?s ?n)

    (handempty ?a)
    (seconf ?q)
    (pose ?o ?p)
    (position ?o ?p)  ;; joint position of a body
    (isopenedposition ?o ?p)  ;;
    (isclosedposition ?o ?p)  ;; assume things start out closed
    (grasp ?o ?g)
    (handlegrasp ?o ?g)

    (wconf ?w)
    (inwconf ?w)
    (newwconfp ?w1 ?o ?p ?w2)
    (newwconfpst ?w1 ?o ?pst ?w2)

    (graspable ?o)
    (stackable ?o ?r)
    (containable ?o ?r)  ;;

    (reachablemovable ?o ?p ?g ?q ?w)
    (reachablepose ?o ?p ?w)
    (atreachablepose ?o ?p)
    (toggled ?o)
    (originalseconf ?q)

    (kin ?a ?o ?p ?g ?q ?t)
    (kinwconf ?a ?o ?p ?g ?q1 ?q2 ?t ?w)
    (kingrasphandle ?a ?o ?p ?g ?q1 ?q2 ?t ?w)
    (kinpulldoorhandle ?a ?o ?p1 ?p2 ?g ?q1 ?q2 ?t ?w1)

    (freemotionwconf ?p1 ?t ?p2 ?w)
    (freemotion ?p1 ?t ?p2)
    (supported ?o ?p ?r)
    (contained ?o ?p ?s) ;; aabb contains
    (traj ?t)

    (trajposecollision ?t ?o ?p)
    (cfreeposepose ?o ?p ?o2 ?p2)
    (cfreeapproachpose ?o ?p ?g ?o2 ?p2)

    (atseconf ?q)
    (atpose ?o ?p)
    (atposition ?o ?p)  ;; joint position of a body
    (openposition ?o ?p)  ;; joint position of a body
    (closedposition ?o ?p)  ;; joint position of a body

    (atgrasp ?a ?o ?g)
    (athandlegrasp ?a ?o ?g)  ;; holding the handle
    (handlegrasped ?a ?o)  ;; holding the handle
    (knobturned ?o)  ;; holding the knob

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
    (unsafetraj ?t)

    (poseobstacle ?o ?p ?o2)
    (approachobstacle ?o ?p ?g ?o2)

    (debug1)
    (debug2)
    (debug3)

  
    (meatturkeyleg ?x)
    (isjointto ?x ?y)
    (braiserlid ?x)
    (oilbottle ?x)
    (space ?x)
    (surface ?x)
    (oven ?x)
    (counter ?x)
    (controllable ?x)
    (braiserbody ?x)
    (floor ?x)
    (arm ?x)
  )
)