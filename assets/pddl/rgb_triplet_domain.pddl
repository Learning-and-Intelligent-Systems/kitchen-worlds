(define
  (domain rgb_triplet_domain)

  (:predicates
    ;; discrete facts (e.g. types, affordances)
    (canmove)

    (arm ?x)
    (handempty ?x)
    (controllable ?x)

    (floor ?x)
    (wall ?x)

    (graspable ?x)

    (bconf ?x)
    (atbconf ?x)

    (surface ?x)

    (pose ?x ?y)
    (atpose ?x ?y)

    (aconf ?x ?y)
    (ataconf ?x ?y)
    (defaultconf ?x ?y)

    (stackable ?x ?y)
    (supported ?x ?xc ?y)
    (on ?x ?y)

    (workspace ?lo ?hi)

  )
)

