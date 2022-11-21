(define
  (problem panda_table)
  (:domain pand_table_domain)

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

    (pushable steak)
    (pushtool steak right)
    ;(graspable steak)
    (graspable cabbage)
    (graspable salt)
    (graspable pepper)
    (graspable grape)

    (atpose steak p0=(0.2, -0.3, 0.05, 0))
    (atpose pepper p0=(0.75, -0.3, 0.05, 0))
    (atpose cabbage p0=(0.3, 0.0, 0.05, 0))
    (atpose grape p0=(0.42, 0.2, 0.015, 0))

 )
  (:goal (and (on steak table) (atpose steak p0=(0.3, 0.1, 0.05, 0))))
  ; (:goal (and (atpose cabbage p0=(0.42, 0.2, 0.05, 0))))
)

