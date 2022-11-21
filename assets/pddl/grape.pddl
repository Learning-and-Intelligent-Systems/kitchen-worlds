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

    ;(floor floor)
    (surface table)

    (graspable large_cap)
    (graspable small_cap)
    (graspable grape)

    (atpose grape p0=(0.42, 0.2, 0.015, 0))
    (atpose large_cap p0=(0.42, -0.3, 0.03, 0, 3.14159, 0))

 )
  (:goal (and (on grape large_cap)))
)

