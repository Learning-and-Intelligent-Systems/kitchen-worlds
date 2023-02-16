(define
  (problem grape_big_1)
  (:domain grape_big_domain)

  (:objects
    ; constants not in LISDF, use to select objects to pay attention to
    right
    grape_class
    table_class
    small_cap_class
    large_cap_class
  )

  (:init
    ;; overrides
    (atpose grape_1 p0=(0.42, -0.2, 0.015, 0))
    (atpose grape_2 p0=(0.42, -0.1, 0.015, 0))    
    (atpose grape_3 p0=(0.42, 0.0, 0.015, 0))
    (atpose grape_4 p0=(0.42, 0.1, 0.015, 0))    
    
    ;; discrete facts (e.g. types, affordances)
    (arm right)
    (class table table_class)
    (class grape_1 grape_class)
    (class grape_2 grape_class)
    (class grape_3 grape_class)
    (class grape_4 grape_class)
    (class small_cap_1 small_cap_class)
    (class small_cap_2 small_cap_class)    
    (class large_cap_1 large_cap_class)
    (class large_cap_2 large_cap_class)
    
    (surface table)
    (graspable grape_1)
    (graspable grape_2)
    (graspable grape_3)
    (graspable grape_4)
    (graspable small_cap_1)
    (graspable small_cap_2)    
    (graspable large_cap_1)
    (graspable large_cap_2)    
 )
  (:goal (and (on grape_1 small_cap_1)))
)

