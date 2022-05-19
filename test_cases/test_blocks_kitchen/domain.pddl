(define (domain pr2-tamp)
  (:requirements :strips :equality)
  (:predicates
    (arm ?a)
    (stackable ?o ?r)

    (sink ?r)
    (stove ?r)
    (counter ?r)
    (table ?r)
    (salter ?o)
    (egg ?o)
    (veggie ?o)
    (plate ?o)
    (supporter ?o)
    (environment ?o)
    (cart ?o)
    (marker ?o)
    (marked ?o ?o2)

    (aconf ?a ?q)
    (bconf ?q)
    (bconfinregion ?q ?r)
    (poseinregion ?o ?p ?r)
    (inroom ?o ?r)
    (robinroom ?r)

    (pose ?o ?p)
    (grasp ?o ?g)
    (markergrasp ?o ?g)
    (kin ?a ?o ?p ?g ?q ?t)
    (kingraspmarker ?a ?o ?p ?g ?q ?t) ;;
    (kinungraspmarker ?a ?o ?p ?g ?q ?t) ;;
    (kinpullmarkerrandom ?a ?o ?p1 ?p2 ?g ?q1 ?q2 ?o2 ?p3 ?p4 ?t) ;;
    (kinpullmarkertopose ?a ?o ?p1 ?p2 ?g ?q1 ?q2 ?o2 ?p3 ?p4 ?t) ;;
    (kinpullmarkertobconf ?a ?o ?p1 ?p2 ?g ?q1 ?q2 ?o2 ?p3 ?p4 ?t) ;;

    (basemotion ?q1 ?t ?q2)
    (armmotion ?a ?q1 ?t ?q2)
    (supported ?o ?p ?r)
    (btraj ?t)
    (atraj ?t)

    (controllable ?o)
    (graspable ?o)
    (stackable ?o ?r)

    (trajposecollision ?t ?o ?p)
    (trajarmcollision ?t ?a ?q)
    (trajgraspcollision ?t ?a ?o ?g)
    (cfreeposepose ?o ?p ?o2 ?p2)
    (cfreeapproachpose ?o ?p ?g ?o2 ?p2)
    (cfreetrajpose ?t ?o2 ?p2)
    (cfreebtrajpose ?t ?o2 ?p2)
    ;;(cfreebtrajwithattachmentpose ?t ?o ?o2 ?p2) ;;

    (atpose ?o ?p)
    (atgrasp ?a ?o ?g)
    (atmarkergrasp ?a ?o ?g) ;;
    (handempty ?a)
    (atbconf ?q)
    (ataconf ?a ?q)
    (canmove)
    (canpull) ;;
    (canungrasp) ;;

    (cleaned ?o)
    (cooked ?o)
    (seasoned ?o)
    (served ?o ?o2)
    (enableomelette ?egg1 ?veggie1 ?plate1)
    (existomelette ?env1)

    (on ?o ?r)
    (holding ?a ?o)
    (holdingmarker ?a ?o) ;;
    (pulledmarker ?o) ;;
    (graspedmarker ?o) ;;
    (savedmarker ?o) ;;

    (unsafepose ?o ?p)
    (unsafeapproach ?o ?p ?g)
    (unsafeatraj ?t)
    (unsafebtraj ?t)
    (poseobstacle ?o ?p ?o2)
    (approachobstacle ?o ?p ?g ?o2)
    (atrajobstacle ?t ?o)
    ;; (unsafebtrajwithattachment ?t ?o) ;;
  
    (defaultconf ?x ?y)
  )
)