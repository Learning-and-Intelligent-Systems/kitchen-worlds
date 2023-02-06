(define
  (problem grape_big_1)
  (:domain grape_big_domain)

  (:objects
    ; constants not in LISDF, use to select objects to pay attention to
    right
    left
    base
  )

  (:init
    ;; overrides
    (atpose grape_1 p0=(0.42, -0.2, 0.015, 0))
    (atpose grape_2 p0=(0.42, -0.1, 0.015, 0))    
    (atpose grape_3 p0=(0.42, 0.0, 0.015, 0))
    (atpose grape_4 p0=(0.42, 0.1, 0.015, 0))    
    
    ;; discrete facts (e.g. types, affordances)
    (arm right)

    (surface table)
    (graspable grape_1)
    (graspable grape_2)
    (graspable grape_3)
    (graspable grape_4)

 )
  (:goal (and (on grape_1 grape_2)))
)

