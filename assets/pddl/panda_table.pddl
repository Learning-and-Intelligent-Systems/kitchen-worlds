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
    (graspable cabbage)
    (graspable salt)
    (graspable pepper)

    (surface table)

 )
  (:goal (and (on steak pepper) (on salt steak) (on cabbage salt)))
)

