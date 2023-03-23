(define
  (problem fetch_test)
  (:domain fetch_test_domain)

  (:objects
    right
  )

  (:init
    ;; discrete facts (e.g. types, affordances)
    (canmove)

    (arm right)
    (controllable right)
    (handempty right)

    (floor floor)

    (graspable box)
    (graspable other_box)    

    (bconf q=(-1.0, 0.0, 0.0))
    (atbconf q=(-1.0, 0.0, 0.0))

    (surface floor)
    (workspace lo=(-2.0, -2., 0.0) hi=(2.0, 2., 2.0))

 )

  (:goal (and
    (on box other_box)
  ))
)

