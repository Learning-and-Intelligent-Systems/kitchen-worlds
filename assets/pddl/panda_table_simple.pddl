(define
  (problem panda_table)
  (:domain pand_table_simple_domain)

  (:objects
    right
  )

  (:init
    ;; discrete facts (e.g. types, affordances)
    (arm right)

    (controllable right)
    (handempty right)

    (floor floor)
    (surface table)

    (graspable grape)
    ; under square cup
    ; (atpose grape p0=(0.2, -0.3, 0.015, 0))
    ; under hollow cup
    (atpose grape p0=(0.42, 0.2, 0.015, 0))
 )
  (:goal (and (on grape table)))
)

