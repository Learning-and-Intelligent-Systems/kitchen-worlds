(define (domain pr2-tamp)
  (:requirements :strips :equality)
  (:predicates
    (arm ?a)
    (sink ?o)
    (stove ?o)
    (drawer ?o) ;;
    (door ?o) ;;
    (joint ?o)

    (aconf ?a ?q)
    (bconf ?q)
    (pose ?o ?p)
    (linkpose ?o ?p)
    (position ?o ?p)  ;; joint position of a body
    (grasp ?o ?g)
    (handlegrasp ?o ?g)

    (controllable ?o)
    (graspable ?o)
    (stackable ?o ?r)
    (containable ?o ?r)  ;;

    (kin ?a ?o ?p ?g ?q ?t)
    (kingrasphandle ?a ?o ?p ?g ?q ?t)  ;; grasp a handle
    (kinpullhandle ?a ?o ?p1 ?p2 ?g ?q1 ?q2 ?t)  ;; pull the handle
    (kinpullhandleopen ?a ?o ?g ?q1 ?q2 ?t)  ;; pull the handle
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

    (atpose ?o ?p)
    (atlinkpose ?o ?p)
    (atposition ?o ?p)  ;; joint position of a body
    (openposition ?o ?p)  ;; joint position of a body
    (closedposition ?o ?p)  ;; joint position of a body
    (atgrasp ?a ?o ?g)
    (athandlegrasp ?a ?o ?g)  ;; holding the handle
    (handlegrasped ?a ?o)  ;; holding the handle
    (handempty ?a)
    (atbconf ?q)
    (ataconf ?a ?q)
    (canmove)
    (cleaned ?o)
    (cooked ?o)
    (openeddrawer ?o) ;;
    (openeddoor ?o) ;;
    (opened ?o) ;;
    (openedjoint ?o) ;;
    (closedjoint ?o) ;;

    (on ?o ?r)
    (in ?o ?r) ;;
    (reachable ?o ?r) ;;
    (holding ?a ?o)

    (unsafepose ?o ?p)
    (unsafeapproach ?o ?p ?g)
    (unsafeatraj ?t)
    (unsafebtraj ?t)
    (poseobstacle ?o ?p ?o2)
    (approachobstacle ?o ?p ?g ?o2)
    (atrajobstacle ?t ?o)
  
    (supporter ?x)
    (defaultconf ?x ?y)
    (canpull)
  )
)