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
   ; (handempty right)

    ; (floor floor)
    (surface table)

    (graspable small_cap)
    (graspable large_cap)    
    (graspable grape)

    (atpose grape p0=(0.3, 0.0, 0.01, 0))
    (atpose small_cap p0=(0.3, 0.0, 0.03, 0, 3.14159, 0))

    (grasp large_cap right 1)

 )
  (:goal (and (on grape small_cap)))
)

