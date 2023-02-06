(define
  (domain grape_big_domain)

  (:predicates
    ; Overrides of LISDF
    (atpose ?x ?y)
    (atrconf ?x ?y) ; robot conf, chain ?x has conf ?y
    (atbconf ?x ?y) ; body conf, e.g. door angle
    (grasp ?obj ?hand ?g) ; ?g is transform

    ;; discrete facts (e.g. types, affordances)

    ; Whether the base can move, False if absent
    (canmove)
    ; Whether to use an arm, False if absent
    (arm ?x)
    ; ?x is of class ?y, some special classes are floor and wall
    (class ?x ?y)
    ; properties and relations
    (graspable ?x)
    (pushable ?x)
    (pushtool ?x ?y)
    (surface ?x)
    (stackable ?x ?y)

  )
)

