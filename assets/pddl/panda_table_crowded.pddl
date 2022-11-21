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

    (graspable steak)
    ;(graspable cabbage)
    ;(graspable salt)
    ;(graspable pepper)
    ;(graspable banana)
    (graspable sugar_box)    
    (graspable grape)

    ;(atpose steak p0=(0.2, -0.3, 0.11, 0))
    (atpose grape p0=(0.25, 0.35, 0.05, 0))

    (surface table)

 )
  (:goal (and (on steak pepper) (on salt steak) (on cabbage salt)))
)

