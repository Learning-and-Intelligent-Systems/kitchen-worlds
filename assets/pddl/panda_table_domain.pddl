(define
  (domain panda_table_domain)

  (:predicates
    ;; discrete facts (e.g. types, affordances)
    (canmove)

    (arm ?x)
    (handempty ?x)
    (controllable ?x)

    (floor ?x)
    (wall ?x)

    (graspable ?x)
    (pushable ?x)
    (pushtool ?x ?y)

    (surface ?x)

    (pose ?x ?y)
    (atpose ?x ?y)

    (aconf ?x ?y)
    (ataconf ?x ?y)
    (defaultconf ?x ?y)

    (stackable ?x ?y)
    (supported ?x ?xc ?y)
    (on ?x ?y)

    (grasp ?obj ?hand ?g)

  )
)

