(define
  (problem panda_table)
  (:domain pand_table_domain)

  (:objects
    right
  )

  (:init
    ;; discrete facts (e.g. types, affordances)
    (arm right)

    (floor floor)
    (surface table)

    (graspable small_cap)
    (graspable grape)

    (atpose grape p0=(0.4, 0.05, 0.015, 0))
    (atpose large_cap p0=(0.3, 0.0, 0.05, 0, 3.14159, 0))
    (atpose small_cap p0=(0.3, 0.2, 0.03, 0, 3.14159, 0))    

 )
  (:goal (and (on grape large_cap)))
)
